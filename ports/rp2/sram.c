// Copyright 2023 (c) Michael Bell
// The BSD 3 clause license applies
#include "py/runtime.h"
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <pico/multicore.h>
#include "hardware/structs/bus_ctrl.h"
#include <string.h>
#include <stdio.h>
#include "py/mpconfig.h"
#include "py/misc.h"

#include "sram.pio.h"

#include "sram.h"

//#define WANT_16K

// We define the SMs and DMA channels to avoid memory accesses
// looking them up which saves precious cycles processing the SPI commands.
#define pio_write_offset 0  // This must be 0

// For 24 bit addresses, the commands are left shifted.
#define READ_CMD (0x03 << (SIM_SRAM_ADDR_BITS - 16))
#define FAST_READ_CMD (0x0B << (SIM_SRAM_ADDR_BITS - 16))
#define WRITE_CMD (0x02 << (SIM_SRAM_ADDR_BITS - 16))

static int pio_read_offset;

static bool sram_enabled = false;

uint32_t gCmdCount = 0;
uint8_t *emu_ram= NULL;

#ifndef WANT_16K
static uint8_t *allocs[3] = {NULL,};
#endif

static void setup_sram_pio(uint8_t *ptr)
{
    uint32_t base;
    uint32_t want_16k;

    // Load either the 16K or the 4K read program (they have different
    // bit shift logic to pull different number of bits from MOSI line).
#ifdef WANT_16K
    base = ((uint32_t) ptr) >> 14;
    pio_read_offset = pio_add_program(SIM_SRAM_pio_read, &sram_read_16k_program);
    want_16k = 1;
#else
    base = ((uint32_t) ptr) >> 12;
    pio_read_offset = pio_add_program(SIM_SRAM_pio_read, &sram_read_4k_program);
    want_16k = 0;
#endif
    pio_sm_claim(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
    pio_add_program_at_offset(SIM_SRAM_pio_write, &sram_write_program, pio_write_offset);
    pio_sm_claim(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm);

    sram_read_program_init(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, pio_read_offset, SIM_SRAM_SPI_CS, SIM_SRAM_SPI_MOSI, SIM_SRAM_SPI_SCK, base, want_16k);
    sram_write_program_init(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, pio_write_offset, SIM_SRAM_SPI_CS, SIM_SRAM_SPI_MOSI, SIM_SRAM_SPI_MISO, SIM_SRAM_SPI_SCK);

    /*
    sram_read_program_init(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, pio_read_offset, SIM_SRAM_SPI_MOSI);
    sram_write_program_init(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, pio_write_offset, SIM_SRAM_SPI_MOSI, SIM_SRAM_SPI_MISO);
    */

#if SIM_SRAM_ADDR_BITS != 16
    SIM_SRAM_pio_read->instr_mem[pio_read_offset + 1] = pio_encode_set(pio_x, SIM_SRAM_ADDR_BITS - 9);
    SIM_SRAM_pio_write->instr_mem[pio_write_offset + 1] = pio_encode_set(pio_x, SIM_SRAM_ADDR_BITS + 6);
#endif
}

static void disable_sram_pio()
{
    pio_sm_set_enabled(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, false);
    pio_sm_clear_fifos(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
    pio_sm_set_enabled(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, false);
    pio_sm_clear_fifos(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm);

#ifdef WANT_16K
    pio_remove_program(SIM_SRAM_pio_read, &sram_read_16k_program, pio_read_offset);
#else
    pio_remove_program(SIM_SRAM_pio_read, &sram_read_4k_program, pio_read_offset);
#endif
    pio_sm_unclaim(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
    pio_remove_program(SIM_SRAM_pio_write, &sram_write_program, pio_write_offset);
    pio_sm_unclaim(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm);

    // Make MISO safe
    gpio_set_dir(SIM_SRAM_SPI_MISO, GPIO_IN);
    gpio_init(SIM_SRAM_SPI_MISO);
}

static void setup_rx_channel()
{
    dma_channel_claim(SIM_SRAM_rx_channel);

    dma_channel_config c = dma_channel_get_default_config(SIM_SRAM_rx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, false));

    dma_channel_configure(
        SIM_SRAM_rx_channel,          // Channel to be configured
        &c,            // The configuration we just created
        NULL,           // The initial write address
        &SIM_SRAM_pio_read->rxf[SIM_SRAM_pio_read_sm],           // The initial read address
#ifdef WANT_16K                                                                 
        16384, // Number of transfers; in this case each is 1 byte.
#else
        4096, // Number of transfers; in this case each is 1 byte.
#endif
        false           // Start immediately.
    );
}

static void setup_tx_channel()
{
    dma_channel_claim(SIM_SRAM_tx_channel);
    dma_channel_claim(SIM_SRAM_tx_channel2);

    dma_channel_config c = dma_channel_get_default_config(SIM_SRAM_tx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_bswap(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, true));

    dma_channel_configure(
        SIM_SRAM_tx_channel,          // Channel to be configured
        &c,            // The configuration we just created
        &SIM_SRAM_pio_write->txf[SIM_SRAM_pio_write_sm],           // The initial write address
        NULL,           // The initial read address
#ifdef WANT_16K                                                                 
        16384, // Number of transfers; in this case each is 1 byte.
#else
        4096, // Number of transfers; in this case each is 1 byte.
#endif
        false           // Start immediately.
    );

    c = dma_channel_get_default_config(SIM_SRAM_tx_channel2);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, false));

    dma_channel_configure(
        SIM_SRAM_tx_channel2,          // Channel to be configured
        &c,            // The configuration we just created
        &dma_hw->ch[SIM_SRAM_tx_channel].al3_read_addr_trig,           // The initial write address
        &SIM_SRAM_pio_read->rxf[SIM_SRAM_pio_read_sm],           // The initial read address
        1, // Number of transfers; in this case each is 1 byte.
        false           // Start immediately.
    );
}

static __always_inline bool is_cs_high() {
    if (gpio_get(SIM_SRAM_SPI_CS)) {
        if (gpio_get(SIM_SRAM_SPI_CS)) {
            // Must be high for 2 cycles to count - avoids deselecting on a glitch.
            return true;
        }
    }
    return false;
}

static __always_inline void wait_for_cs_high() {
    while (!is_cs_high());
}

static void __scratch_x("core1_main") core1_main()
{
    while (true) {
        uint32_t cmd = pio_sm_get_blocking(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
        gCmdCount++;
        if (is_cs_high()) {
            // Abort before the address
        }
        else if (cmd == READ_CMD) {
            // Read - this works by transferring the address direct from the Read PIO SM
            // direct to the read address of the transmit DMA channel.
            dma_channel_start(SIM_SRAM_tx_channel2);

            wait_for_cs_high();
            dma_channel_abort(SIM_SRAM_tx_channel);
        }
        else if (cmd == FAST_READ_CMD) {
            // Fast read
            // Need to patch the write program to do extra delay cycles
            SIM_SRAM_pio_write->instr_mem[sram_write_offset_addr_loop_end] = pio_encode_jmp(sram_write_offset_fast_read);

            // And change the write size to 8
            hw_clear_bits(&dma_hw->ch[SIM_SRAM_tx_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS);
            hw_set_bits(&SIM_SRAM_pio_write->sm[SIM_SRAM_pio_write_sm].shiftctrl, 8 << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB);

            // Transfer the address manually
            uint32_t addr = pio_sm_get_blocking(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
            addr |= pio_sm_get_blocking(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
            dma_hw->ch[SIM_SRAM_tx_channel].al3_read_addr_trig = addr;

            wait_for_cs_high();
            dma_channel_abort(SIM_SRAM_tx_channel);

            // Unpatch the write program
            SIM_SRAM_pio_write->instr_mem[sram_write_offset_addr_loop_end] = pio_encode_jmp_pin(sram_write_offset_addr_two);

            // And change the write size back to 32
            hw_set_bits(&dma_hw->ch[SIM_SRAM_tx_channel].al1_ctrl, 2 << DMA_CH10_CTRL_TRIG_DATA_SIZE_LSB);
            hw_clear_bits(&SIM_SRAM_pio_write->sm[SIM_SRAM_pio_write_sm].shiftctrl, PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);
        }
        else if (cmd == WRITE_CMD) {
            // Write
            uint32_t addr = pio_sm_get_blocking(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
            addr |= pio_sm_get_blocking(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
            dma_hw->ch[SIM_SRAM_rx_channel].al2_write_addr_trig = addr;

            wait_for_cs_high();
            while (!pio_sm_is_rx_fifo_empty(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm));
            dma_channel_abort(SIM_SRAM_rx_channel);
        }
        else {
            // Ignore unknown command
            wait_for_cs_high();
        }
        pio_sm_set_enabled(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, false);
        pio_sm_clear_fifos(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm);
        pio_sm_restart(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm);
        pio_sm_exec(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, pio_encode_jmp(pio_write_offset));
        pio_sm_set_enabled(SIM_SRAM_pio_write, SIM_SRAM_pio_write_sm, true);
        pio_sm_set_enabled(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, false);
        pio_sm_clear_fifos(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
        pio_sm_restart(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm);
        pio_sm_exec(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, pio_encode_jmp(pio_read_offset));
        pio_sm_set_enabled(SIM_SRAM_pio_read, SIM_SRAM_pio_read_sm, true);            
    }
}

void setup_simulated_sram() {
    setup_rx_channel();
    setup_tx_channel();

    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS);

    sram_enabled = false;
}

int enable_simulated_sram() {
#ifdef WANT_16K
    uint32_t   size;
    uint8_t   *ptr;
    uint8_t   *orig;
    uint8_t   *ptr2 = NULL;
    uint8_t   *ptr3 = NULL;
#else
    uint8_t   *pAlign[10] = {NULL,};
    int        alignIdx = 0;
    int       x;
#endif
    uint32_t   block_size;

    if (!sram_enabled) {
#ifdef WANT_16K
        // Allocate a 32K chunk to find a region we can fit
        emu_ram = m_malloc_maybe(32768);
        if (emu_ram == NULL)
           return 0;

        // Free the region to collect blocks
        m_free(emu_ram);

        // Allocate the chunk again to get the real address
        emu_ram = m_malloc_maybe(32768);

        // Save the pointer and free the block
        ptr = emu_ram;
        m_free(emu_ram);

        // Calculate the number bytes below the 16K alignment point
        ptr = (uint8_t *) (((int)emu_ram + 16383) & ~0x3FFF);
        size = (int) (ptr - emu_ram);

        // Allocate that number of bytes to consume RAM up to the 16K
        // alignment point
        ptr = m_malloc_maybe(size);

        // Test if our allocation came from the region we thought
        if (ptr != emu_ram)
        {
           // No, there was a chunk our size elsewhere.  Allocate again
           ptr2 = m_malloc_maybe(size);

           // Test if this is the right place
           if (ptr2 != emu_ram)
           {
              // NO!  Okay, try a third allocation
              ptr3 = m_malloc_maybe(size);
           }
        }

        // Okay, now allocate our block.  With any luck it will be our 16K
        // aligned region we calculated
        orig = emu_ram;
        emu_ram = m_malloc_maybe(16384);
        block_size = 16384;

        // Free the temp blocks we consumed tryin to get the 16K alignemnt
        m_free(ptr);
        if (ptr2)
           m_free(ptr2);
        if (ptr3)
           m_free(ptr3);

        // Okay, test if our memory is aligned
        if ((emu_ram == NULL) || ((int) emu_ram & 0x3FFF) != 0)
        {
           // HMM, okay.  not aligned.  Try just allocating 16K + size
           if (emu_ram != NULL)
              m_free(emu_ram);
           emu_ram = m_malloc_maybe(16384 + size);
           block_size = 16384 + size;

           // See if we are at the same place
           if (emu_ram != orig)
           {
              // Okay, we are now in totally different place!  I guess
              // just go back to the 32K allocation (give up)
              m_free(emu_ram);
              emu_ram = m_malloc_maybe(32768);
              block_size = 32768;
           }
        }

        // Double check the final emu_ram
        if (emu_ram == NULL)
           return 0;

        int x;
        uint16_t *u16 =(uint16_t *) emu_ram;
        for (x = 0; x < 8192; x++)
              *u16++ = ((x/256+1) << 8) | (x & 0xFF);

#else
        // Working with 4K aligned chunk.  First allocate 6K
        pAlign[0] = m_malloc_maybe(2048);
        pAlign[1] = m_malloc_maybe(2048);
        pAlign[2] = m_malloc_maybe(2048);

        // Find 3 2K chunks that are contiguous
        while (alignIdx < 8)
        {
           uint8_t *pBase = (uint8_t *) (((uint32_t) pAlign[alignIdx] + 2047) & ~0xFFF);

           // Test for contiguous 4K region where the first block
           // contains the 4K aligned address
           if (pAlign[alignIdx] + 2048 == pAlign[alignIdx + 1] &&
               pAlign[alignIdx+1] + 2048 == pAlign[alignIdx + 2] &&
               pBase >= pAlign[alignIdx] && pBase < pAlign[alignIdx] + 2048)
           {
              // Aligned block found
              mp_printf(MP_PYTHON_PRINTER, "BLK1: 0x%08X - 0x%08X\n", pAlign[alignIdx], pAlign[alignIdx]+2048);
              mp_printf(MP_PYTHON_PRINTER, "BLK2: 0x%08X - 0x%08X\n", pAlign[alignIdx+1], pAlign[alignIdx+1]+2048);
              mp_printf(MP_PYTHON_PRINTER, "BLK3: 0x%08X - 0x%08X\n", pAlign[alignIdx+2], pAlign[alignIdx+2]+2048);
        
              emu_ram = (uint8_t *) (((uint32_t) pAlign[alignIdx] + 2047) & ~0xFFF);
        
              // Free buffers not part of the alignment
              for (x = 0; x < alignIdx; x++)
                 m_free(pAlign[x]);
        
              // Save the allocs for freeing later
              for (x = 0; x < 3; x++)
                 allocs[x] = pAlign[alignIdx + x];
              break;
           }
           else
           {
              // Allocate another 2K chunk and try again
              if (alignIdx < 7)
              {
                 pAlign[alignIdx + 3] = m_malloc_maybe(2048);
                 if (pAlign[alignIdx+3] == NULL)
                 {
                    alignIdx = 8;
                    break;
                 }
                 alignIdx++;
              }
           }
        }
        
        // If alignment not found, free all blocks and return
        if (alignIdx == 8)
        {
           for (alignIdx = 0; alignIdx < 10; alignIdx++)
           {
              if (pAlign[alignIdx])
                 m_free(pAlign[alignIdx]);
           }
           return 0;
        }

        block_size = 4096;
#endif

        // Initialize the PIO
        mp_printf(MP_PYTHON_PRINTER, "SRAM: 0x%08X (%d bytes)\n", emu_ram, block_size);
        setup_sram_pio(emu_ram);
        multicore_launch_core1(core1_main);
    }
    sram_enabled = true;
    return 1;
}

void disable_simulated_sram() {
    if (sram_enabled) {
        disable_sram_pio();
        multicore_reset_core1();

        // Free the memory
#ifdef WANT_16K
        m_free(emu_ram);
#else
        m_free(allocs[0]);
        m_free(allocs[1]);
        m_free(allocs[2]);
#endif
        emu_ram = NULL;
    }
    sram_enabled = false;
}
