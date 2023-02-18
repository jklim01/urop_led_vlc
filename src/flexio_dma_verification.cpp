/*
    This file is used to verify the proper configuration of the FlexIO module and correct setup of DMA to periodically
    refill the FlexIO buffer. If the values values in `buf1` and `buf2` are unchanged, under the default bit pattern, the
    pin corresponding to bit `i` (where bit 0 is the LSB and bit 9 is the MSB) will output a repeating pattern of a high
    pulse lasting for `2i` sample periods followed by a low pulse lasting for `20-2i` sample periods, each separated by a
    pattern that is low for 1 sample period, high for 2 sample periods and low for another sample period.
    Example:
     - LSB pin: _11_11___________________11_...
     - 3rd pin: _11_111111_______________11_...
     - MSB pin: _11_11111111111111111111_11_...

    A 10-bit DAC attached to the output to recieve the 10 bit binary codes can also be verified. In `void setup()`, uncomment any
    one section of code that modifies `buf1` and `buf2` and upload the code. The DAC output should now display the corresponding
    waveform when viewed with an 0scilloscope.

    Copyright (c) 2023 Lim Jie-Khang

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "Arduino.h"
#include "DMAChannel.h"
#include "utils.h"


// shifters 0 and 4 do 4-bit parallel shift and each have an "effective shift register width" of 96 bits
// thus, the shifters will need to be reloaded from the shift buffers after 24 shifts
constexpr auto NUM_FLEXIO_SHIFTBUF = 8;
constexpr auto SHIFTS_PER_REFILL = 24;
constexpr auto BITS_PER_SAMPLE = 10;

constexpr auto SAMPLES_PER_DATABUF = 96;                                                // set to (4 * # samples for default bit pattern)
constexpr auto REFILLS_PER_DATABUF = SAMPLES_PER_DATABUF / SHIFTS_PER_REFILL ;          // (# samples per databuf) / (# samples shifted out per refill)
static_assert(SAMPLES_PER_DATABUF % SHIFTS_PER_REFILL == 0, "SAMPLES_PER_DATABUF is not a multiple of SHIFTS_PER_REFILL!");


static uint32_t buf1[SAMPLES_PER_DATABUF / 2] = {
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000
};
static uint32_t buf2[SAMPLES_PER_DATABUF / 2] = {
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000
};

constexpr auto NUM_DMA_SETTINGS = 2;
constexpr auto DMA_BUF_LEN = NUM_FLEXIO_SHIFTBUF * REFILLS_PER_DATABUF;                       // fill NUM_FLEXIO_SHIFTBUF SHIFTBUFs per refill
constexpr auto DMA_BUF_SIZE = DMA_BUF_LEN * sizeof(uint32_t);
DMASetting settings[NUM_DMA_SETTINGS];
DMAChannel my_channel { true };
static uint32_t dma_buf_1[DMA_BUF_LEN] DMAMEM __attribute__ ((aligned(32)));
static uint32_t dma_buf_2[DMA_BUF_LEN] DMAMEM __attribute__ ((aligned(32)));

static void fill_dma_buf(uint32_t* dma_buf, uint16_t* src, int soff=sizeof(uint16_t)) {
    memset(dma_buf, 0, DMA_BUF_SIZE);
    uint8_t* src_ptr = (uint8_t*)src;

    for (int i = 0; i < REFILLS_PER_DATABUF; i++) {
        uint8_t* buf_ptr_0 = (uint8_t*)(dma_buf + NUM_FLEXIO_SHIFTBUF*i);
        uint8_t* buf_ptr_4 = (uint8_t*)(dma_buf + NUM_FLEXIO_SHIFTBUF*i + 4);
        uint32_t* buf_ptr_3 = dma_buf + NUM_FLEXIO_SHIFTBUF*i + 3;
        uint32_t* buf_ptr_7 = dma_buf + NUM_FLEXIO_SHIFTBUF*i + 7;

        for (int j = 0; j < (SHIFTS_PER_REFILL / 2); j++) {
            uint32_t current_1 = *(uint16_t*)src_ptr;
            src_ptr += soff;
            uint32_t current_2 = *(uint16_t*)src_ptr;
            src_ptr += soff;

            *buf_ptr_0 |= ((current_2 & 0x0F) << 4) | (current_1 & 0x0F);
            buf_ptr_0++;

            *buf_ptr_4 |= (current_2 & 0xF0) | ((current_1 & 0xF0) >> 4);
            buf_ptr_4++;

            int8_t lshift_count = 2*j - 7;
            *buf_ptr_3 |= (lshift_count > 0) ? ((current_2 & 0x100) << lshift_count) : ((current_2 & 0x100) >> (-1 * lshift_count));

            lshift_count--;
            if (lshift_count > 0) {
                *buf_ptr_3 |= (current_1 & 0x100) << lshift_count;
                *buf_ptr_7 |= (current_2 & 0x200) << lshift_count;
            }
            else {
                *buf_ptr_3 |= (current_1 & 0x100) >> (-1 * lshift_count);
                *buf_ptr_7 |= (current_2 & 0x200) >> (-1 * lshift_count);
            }

            lshift_count--;
            *buf_ptr_7 |= (lshift_count > 0) ? ((current_1 & 0x200) << lshift_count) : ((current_1 & 0x200) >> (-1 * lshift_count));
        }
    }

    arm_dcache_flush(dma_buf, DMA_BUF_SIZE);
}

static void print_flexiobuf() {
    Serial.printf("SHIFTBUF0 = 0x%08X\n", FLEXIO2_SHIFTBUF0);
    Serial.printf("SHIFTBUF1 = 0x%08X\n", FLEXIO2_SHIFTBUF1);
    Serial.printf("SHIFTBUF2 = 0x%08X\n", FLEXIO2_SHIFTBUF2);
    Serial.printf("SHIFTBUF3 = 0x%08X\n", FLEXIO2_SHIFTBUF3);
    Serial.printf("SHIFTBUF4 = 0x%08X\n", FLEXIO2_SHIFTBUF4);
    Serial.printf("SHIFTBUF5 = 0x%08X\n", FLEXIO2_SHIFTBUF5);
    Serial.printf("SHIFTBUF6 = 0x%08X\n", FLEXIO2_SHIFTBUF6);
    Serial.printf("SHIFTBUF7 = 0x%08X\n\n", FLEXIO2_SHIFTBUF7);
}


static void my_isr(void) {
    static int count = 0;
    if (count > 10)
        return;
    Serial.println("Printing from InterruptHalf:");
    utils::print_tcd(my_channel);
    print_flexiobuf();
    count++;
}


void setup() {
    // uncomment a section to get the corresponding waveform at the DAC output
    for (size_t i = 0; i < SAMPLES_PER_DATABUF; i++) {
        // ramp
        // ((uint16_t*)buf1)[i] = i << 2;
        // ((uint16_t*)buf2)[i] = (SAMPLES_PER_DATABUF + i) << 2;

        // // sine
        // ((uint16_t*)buf1)[i] = (uint16_t)(0xFFFF * ((sin(TWO_PI / (2*SAMPLES_PER_DATABUF) * i) + 1) / 2) + 0.5) >> 6;
        // ((uint16_t*)buf2)[i] = (uint16_t)(0xFFFF * ((sin(TWO_PI / (2*SAMPLES_PER_DATABUF) * (SAMPLES_PER_DATABUF + i)) + 1) / 2) + 0.5) >> 6;

        // // triangle wave
        ((uint16_t*)buf1)[i] = i << 3;
        ((uint16_t*)buf2)[i] = (SAMPLES_PER_DATABUF - i) << 3;

        // // parabola
        // ((uint16_t*)buf1)[i] = (uint16_t)(i*i / 9.1 + 0.5);
        // ((uint16_t*)buf2)[i] = (uint16_t)((SAMPLES_PER_DATABUF - i) * (SAMPLES_PER_DATABUF - i) / 9.1 + 0.5);

        // ((uint16_t*)buf1)[i] = 0x0000;
        // ((uint16_t*)buf2)[i] = 0xFFFF;
    }
    fill_dma_buf(dma_buf_1, (uint16_t*)buf1);
    fill_dma_buf(dma_buf_2, (uint16_t*)buf2);


    // setup DMA channel and settings
    // ! DMOD must be used since we need to but cannot readjust DADDR back to the first SHIFTBUF after each minor loop completion using any other way
    for (size_t i = 0; i < NUM_DMA_SETTINGS; i++) {
        settings[i].TCD->SOFF = 4;                                                          // all 32-bit values read in each basic transfer contiguous in memory
        settings[i].TCD->ATTR_SRC = 2;                                                      // read 32 bits at a time (no SADDR modulo, ie SMOD = 0)
        // settings[i].TCD->SOFF = 32;                                                         // all 32-byte values for each basic transfer contiguously placed
        // settings[i].TCD->ATTR_SRC = 5;                                                      // read 32 bytes at a time (no SADDR modulo, ie SMOD = 0)
        settings[i].TCD->SLAST = -DMA_BUF_SIZE;                                             // negative offset back to start of dma buffer at the end of major loop

        settings[i].TCD->DADDR = &FLEXIO2_SHIFTBUF0;
        // settings[i].TCD->DOFF = 4;                                                          // all 32-bit SHIFTBUFs written to in each basic transfer contiguous in memory
        // settings[i].TCD->ATTR_DST = (5 << 3) | 2;                                           // mod-32 DADDR to cycle through SHIFTBUFs  +  write 32 bits at a time
        // settings[i].TCD->DOFF = 32;                                                         // all SHIFTBUFs written to in each basic transfer contiguous in memory
        // settings[i].TCD->ATTR_DST = (5 << 3) | 5;                                           // mod-32 DADDR to cycle through SHIFTBUFs  +  write 32 bytes at a time
        settings[i].TCD->DOFF = 0;                                                          // DADDR can always stay at the start since only 1 basic transfer needed
        settings[i].TCD->ATTR_DST = 5;                                                      // write 32 bytes at a time (DMOD not required since DADDR never changed)

        settings[i].TCD->NBYTES_MLNO = NUM_FLEXIO_SHIFTBUF * sizeof(uint32_t);              // fill NUM_FLEXIO_SHIFTBUF 32-bit SHIFTBUFs per minor loop
        settings[i].TCD->BITER_ELINKNO = REFILLS_PER_DATABUF;                               // refill the SHIFTBUFs REFILLS_PER_DATABUFs times per major loop
        settings[i].TCD->CITER_ELINKNO = REFILLS_PER_DATABUF;                               // BITER and CITER must be set to the same value

        settings[i].disableOnCompletion();                                                  // prevent underrun of dma buffer
        // settings[i].interruptAtCompletion();
        // settings[i].interruptAtHalf();
        settings[i].replaceSettingsOnCompletion(settings[(i+1) % NUM_DMA_SETTINGS]);        // cycle between the different settings
    }
    settings[0].TCD->SADDR = dma_buf_1;
    settings[1].TCD->SADDR = dma_buf_2;

    my_channel.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);
    // my_channel.attachInterrupt(&my_isr);
    my_channel = settings[0];


    // configure muxes to set clock frequency supplied to FlexIO2 module to 120MHz
    // default frequency  =  PLL3 frequency / CCM_CS1CDR_FLEXIO2_CLK_PRED / CCM_CS1CDR_FLEXIO2_CLK_PODF  =  480MHz / 2 / 8  =  30MHz
    CCM_CS1CDR &= ~CCM_CS1CDR_FLEXIO2_CLK_PODF(7);  // clear default bits (111)
    CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1);   // set clock divider of CCM_CS1CDR_FLEXIO2_CLK_PODF to 2

    // enable FlexIO2 functional clock
    // - must be enabled before accessing any of its registers
    CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);


    // enable FlexIO2
    FLEXIO2_CTRL |= 1;

    // enable DMA request generation when FlexIO2 SSF flag is set
    FLEXIO2_SHIFTSDEN |= 1;


    // configure external pads to FlexIO mode
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 4;       // 10, FLEXIO2_00 (LSB)
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 4;       // 12, FLEXIO2_01
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 4;       // 11, FLEXIO2_02
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 4;       // 13, FLEXIO2_03

    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 4;       // 8, FLEXIO2_16
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 4;       // 7, FLEXIO2_17
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02 = 4;       // 36, FLEXIO2_18
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03 = 4;       // 37, FLEXIO2_19

    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 4;       // 6, FLEXIO2_10

    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 4;       // 9, FLEXIO2_11 (MSB)


    // setup shifters
    // - choose the same timer since all need to work simultaneously
    // - only shifters 0 and 4 can perform parallle shifting

    // 4-bit parallel shift out with shifter 0 to FlexIO2 pins 0~3, choosing shifters 1 and 2 as its buffer
    FLEXIO2_SHIFTCTL0 =
        FLEXIO_SHIFTCTL_TIMSEL(0)       |               // use Timer 0 to generate shift clock
        // FLEXIO_SHIFTCTL_TIMPOL       |               // shift on posedge
        FLEXIO_SHIFTCTL_PINCFG(3)       |               // enable pin output
        FLEXIO_SHIFTCTL_PINSEL(0)       |               // select FLEXIO_D0 pin
        // FLEXIO_SHIFTCTL_PINPOL       |               // active high
        FLEXIO_SHIFTCTL_SMOD(2);                        // transmit mode

    FLEXIO2_SHIFTCFG0 =
        FLEXIO_SHIFTCFG_PWIDTH(3)       |               // parallel shift 4 bits
        FLEXIO_SHIFTCFG_INSRC           |               // use next shifter as its input / buffer
        FLEXIO_SHIFTCFG_SSTOP(0)        |               // no stop bits
        FLEXIO_SHIFTCFG_SSTART(0);                      // no start bits

    FLEXIO2_SHIFTCTL1 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG1 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO2_SHIFTCTL2 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG2 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);


    // 4-bit parallel shift out with shifter 4 to FlexIO pins 16~19, choosing shifters 5 and 6 as its buffer
    FLEXIO2_SHIFTCTL4 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(16) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG4 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO2_SHIFTCTL5 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG5 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO2_SHIFTCTL6 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG6 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);


    // 1-bit shift out with shifter 3 to FlexIO2 pin 10
    FLEXIO2_SHIFTCTL3 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(10) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG3 = FLEXIO_SHIFTCFG_PWIDTH(0) | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);


    // 1-bit shift out with shifter 7 to FlexIO2 pin 11
    FLEXIO2_SHIFTCTL7 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3) | FLEXIO_SHIFTCTL_PINSEL(11) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO2_SHIFTCFG7 = FLEXIO_SHIFTCFG_PWIDTH(0) | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);


    my_channel.enable();            // only enable after all shifters have been configured to load from their buffers


    // setup FlexIO timer 0 (4MHz shift clock)
    // TIMCFG should be configured before setting TIMOD
    FLEXIO2_TIMCMP0 = (((2*SHIFTS_PER_REFILL - 1) & 0xFF) << 8) | (30/2 - 1);           // set shifter clock frequency to 1/30 of FlexIO clock's frequency

    FLEXIO2_TIMCFG0 =
        FLEXIO_TIMCFG_TIMOUT(10)            |           // timer output logic 1 upon enable and not affected by timer reset
        FLEXIO_TIMCFG_TIMDEC(00)            |           // decrement counter on FLEXIO clock, shift clock is timer output
        FLEXIO_TIMCFG_TIMRST(000)           |           // timer never resets
        FLEXIO_TIMCFG_TIMDIS(000)           |           // timer never disables
        FLEXIO_TIMCFG_TIMENA(000)           |           // timer always enabled
        FLEXIO_TIMCFG_TSTOP(00);                        // stop bit disabled
        // | FLEXIO_TIMCFG_TSTART                       // start bit disabled


    while(!Serial);
    Serial.println("I'm started!");
    for (size_t i = 0; i < sizeof(buf1) / sizeof(*buf1); i++)
        Serial.printf("buf1[%2d]  = 0x%08X\n", i, buf1[i]);
    Serial.printf("\n");

    for (size_t i = 0; i < sizeof(buf2) / sizeof(*buf2); i++)
        Serial.printf("buf2[%2d]  = 0x%08X\n", i, buf2[i]);
    Serial.printf("\n--------------------------------------------------------------------------------------------------------------\n");

    Serial.printf("SHIFTBUF0 @ %08p\n", &FLEXIO2_SHIFTBUF0);
    print_flexiobuf();

    Serial.printf("\ndma_buf_1 @ %08p ---------------\n", dma_buf_1);
    for (int i = 0; i < DMA_BUF_LEN; i++)
        Serial.printf("dma_buf_1[%2d]  = 0x%08X\n", i, dma_buf_1[i]);
    Serial.printf("\ndma_buf_2 @ %08p ---------------\n", dma_buf_2);
    for (int i = 0; i < DMA_BUF_LEN; i++)
        Serial.printf("dma_buf_2[%2d]  = 0x%08X\n", i, dma_buf_2[i]);
    Serial.printf("\n--------------------------------------------------------------------------------------------------------------\n");


    FLEXIO2_TIMCTL0 =
        FLEXIO_TIMCTL_PINCFG(00)            |           // timer pin output disabled
        FLEXIO_TIMCTL_TIMOD(01);                        // enable timer in dual 8-bit counters mode
}


void loop() {
    static int count = 0;
    static uint32_t start = micros();
    static uint32_t* waiting_buf = buf1;
    static uint32_t* next_buf = buf2;
    static uint32_t* inuse_dma_buf = dma_buf_1;
    static uint32_t* free_dma_buf = dma_buf_2;

    // check for SHIFTBUF underrun
    if (FLEXIO2_SHIFTERR) {
        uint32_t now = micros();
        Serial.printf("[%u] SHIFTERR = 0x%08X occured at count = %d (%u us elapsed)\n", now, FLEXIO2_SHIFTERR, count, now - start);
        FLEXIO2_SHIFTERR = 0xFF;
    }

    // check for DMA error
    if (my_channel.error()) {
        uint32_t now = micros();
        Serial.printf("[%u] ES = 0x%08X at count  = %d (%u us elapsed)\n", now, DMA_ES, count, now - start);
        utils::print_tcd(my_channel);
        utils::panic(3);
    }


    fill_dma_buf(free_dma_buf, (uint16_t*)next_buf);


    // wait until DMA major loop has completed before moving on to prevent potential overrun of DMA buffer (the channel disables upon completion)
    while (DMA_ERQ & (0x1 << my_channel.channel));
    if (count < 10)
        utils::print_tcd(my_channel);

    my_channel.enable();

    if (count < 10) {
        print_flexiobuf();
        // Serial.printf("free_dma_buf[0] = 0x%08X\n", free_dma_buf[0]);
        // Serial.printf("free_dma_buf[1] = 0x%08X\n", free_dma_buf[1]);
        // Serial.printf("free_dma_buf[2] = 0x%08X\n", free_dma_buf[2]);
        // Serial.printf("free_dma_buf[3] = 0x%08X\n", free_dma_buf[3]);
        // Serial.printf("free_dma_buf[4] = 0x%08X\n", free_dma_buf[4]);
        // Serial.printf("free_dma_buf[5] = 0x%08X\n", free_dma_buf[5]);
        // Serial.printf("free_dma_buf[6] = 0x%08X\n", free_dma_buf[6]);
        // Serial.printf("free_dma_buf[7] = 0x%08X\n\n", free_dma_buf[7]);
    }

    uint32_t* temp = next_buf;
    next_buf = waiting_buf;
    waiting_buf = temp;

    temp = free_dma_buf;
    free_dma_buf = inuse_dma_buf;
    inuse_dma_buf = temp;
    count++;
}
