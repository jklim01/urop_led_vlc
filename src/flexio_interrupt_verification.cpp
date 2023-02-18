/*
    This file is used to verify the proper configuration of the FlexIO module and correct setup of interrupts to periodically
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

#define _USE_MATH_DEFINES
#include <math.h>

// 16 16-bit parallel shifts can be performed on the 8 shifters with a combined "effective shift register width" of 256
constexpr auto NUM_FLEXIO_SHIFTBUF = 8;
constexpr auto SHIFTS_PER_REFILL = 16;
constexpr auto BITS_PER_SAMPLE = 10;
constexpr auto SHIFT_BUFFER_SIZE = NUM_FLEXIO_SHIFTBUF * sizeof(uint32_t);

constexpr uint8_t FLEXIO3_INTERRUPT_PRIORITY = 64;
constexpr auto SAMPLES_PER_DATABUF = 96;                                                // set to (4 * # samples for default bit pattern)
constexpr auto REFILLS_PER_DATABUF = SAMPLES_PER_DATABUF / SHIFTS_PER_REFILL;          // (# samples per databuf) / (# samples shifted out per refill)
static_assert(SAMPLES_PER_DATABUF % SHIFTS_PER_REFILL == 0, "SAMPLES_PER_DATABUF is not a multiple of SHIFTS_PER_REFILL!");


static uint32_t buf1[SAMPLES_PER_DATABUF / 2] = {
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
};
static uint32_t buf2[SAMPLES_PER_DATABUF / 2] = {
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
    0x000003FF, 0x03FF03FF, 0x03FE03FE, 0x03FC03FC, 0x03F803F8, 0x03F003F0, 0x03E003E0, 0x03C003C0, 0x03800380, 0x03000300, 0x02000200, 0x03FF0000,
};

static void my_isr() {
    static uint8_t refill_count = 0;
    static uint32_t* buf_ptr = buf1;

    // if something weird happens, try manually clearing the SSF flag
    // FLEXIO3_SHIFTSTAT = 1;


    for (int i = 0; i < SHIFT_BUFFER_SIZE / sizeof(FLEXIO3_SHIFTBUF0); i++) {
        (&FLEXIO3_SHIFTBUF0)[i] = *buf_ptr;
        buf_ptr++;
    }
    refill_count++;


    switch (refill_count) {
        case REFILLS_PER_DATABUF: {
            NVIC_DISABLE_IRQ(IRQ_FLEXIO3);          // prevent underrun of data buffers (try moving to the top if weird things happend)
            buf_ptr = (uint32_t*)buf2;
            break;
        }

        case 2*REFILLS_PER_DATABUF: {
            NVIC_DISABLE_IRQ(IRQ_FLEXIO3);          // prevent underrun of data buffers (try moving to the top if weird things happend)
            refill_count = 0;
            buf_ptr = (uint32_t*)buf1;
            break;
        }
    }

    asm volatile("dsb");
}


static void print_flexiobuf() {
    Serial.printf("SHIFTBUF0 = 0x%08X\n", FLEXIO3_SHIFTBUF0);
    Serial.printf("SHIFTBUF1 = 0x%08X\n", FLEXIO3_SHIFTBUF1);
    Serial.printf("SHIFTBUF2 = 0x%08X\n", FLEXIO3_SHIFTBUF2);
    Serial.printf("SHIFTBUF3 = 0x%08X\n", FLEXIO3_SHIFTBUF3);
    Serial.printf("SHIFTBUF4 = 0x%08X\n", FLEXIO3_SHIFTBUF4);
    Serial.printf("SHIFTBUF5 = 0x%08X\n", FLEXIO3_SHIFTBUF5);
    Serial.printf("SHIFTBUF6 = 0x%08X\n", FLEXIO3_SHIFTBUF6);
    Serial.printf("SHIFTBUF7 = 0x%08X\n\n", FLEXIO3_SHIFTBUF7);
}


void setup() {
    // uncomment a section to get the corresponding waveform at the DAC output
    for (size_t i = 0; i < sizeof(buf1) / sizeof(*buf1); i++) {
        // ramp
        // buf1[i] = (((2*i + 1) << 16) | 2*i) << 2;
        // buf2[i] = (((2*i + 97) << 16) | (2*i + 96)) << 2;

        // sine
        // buf1[i] = ((uint32_t)(0xFFFF * ((sin(TWO_PI / 192 * (2*i + 1)) + 1))/2 + 0.5) << 10) |
        //     ((uint32_t)(0xFFFF * ((sin(TWO_PI / 192 * (2*i)) + 1)/2) + 0.5) >> 6);
        // buf2[i] = ((uint32_t)(0xFFFF * ((sin(TWO_PI / 192 * (2*i + 97)) + 1))/2 + 0.5) << 10) |
        //     ((uint32_t)(0xFFFF * ((sin(TWO_PI / 192 * (2*i + 96)) + 1)/2) + 0.5) >> 6);

        // triangle wave
        // buf1[i] = (((2*i + 1) << 16) | 2*i) << 3;
        // buf2[i] = (((94 - 2*i) << 16) | (95 -2*i)) << 3;

        // parabola
        // buf1[i] = ((uint32_t)((2*i + 1) * (2*i + 1) / 9.1 + 0.5) << 16) | (uint32_t)((2*i) * (2*i) / 9.1 + 0.5);
        // buf2[i] = ((uint32_t)((95 -2*i) * (95 -2*i) / 9.1 + 0.5) << 16) | (uint32_t)((96 -2*i) * (96 -2*i) / 9.1 + 0.5);
    }

    // setup IRQ for transfer to SHIFTBUF
    attachInterruptVector(IRQ_FLEXIO3, &my_isr);
    NVIC_SET_PRIORITY(IRQ_FLEXIO3, FLEXIO3_INTERRUPT_PRIORITY);


    // configure muxes to set clock frequency supplied to FlexIO3 module to 120MHz
    // default frequency  =  PLL3 frequency / CCM_CS1CDR_FLEXIO2_CLK_PRED / CCM_CS1CDR_FLEXIO2_CLK_PODF  =  480MHz / 2 / 8  =  30MHz
    // ! FlexIO3 clock root is the same as FlexIO2
    CCM_CS1CDR &= ~CCM_CS1CDR_FLEXIO2_CLK_PODF(7);  // clear default bits (111)
    // CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1);   // set clock divider of CCM_CS1CDR_FLEXIO2_CLK_PODF to 2
    CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(7);   // set clock divider of CCM_CS1CDR_FLEXIO2_CLK_PODF to 8

    // enable FlexIO3 functional clock
    // - must be enabled before accessing any of its registers
    CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);


    // enable FlexIO3
    FLEXIO3_CTRL |= 1;

    // enable interrupt generation when FlexIO3 SSF flag is set
    FLEXIO3_SHIFTSIEN |= 1;


    // configure external pads to FlexIO mode
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 9;       // 19, FLEXIO3_00 (LSB)
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 9;       // 18, FLEXIO3_01
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 9;       // 14, FLEXIO3_02
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 9;       // 15, FLEXIO3_03
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 9;       // 40, FLEXIO3_04
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 9;       // 41, FLEXIO3_05
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 9;       // 17, FLEXIO3_06
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 9;       // 16, FLEXIO3_07
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 9;       // 22, FLEXIO3_08
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 9;       // 23, FLEXIO3_09 (MSB)


    // setup shifters
    // 1-bit shift out with shifter 0 to FlexIO3 pin 3, choosing shifters 1~7 as its buffer
    FLEXIO3_SHIFTCTL0 =
        FLEXIO_SHIFTCTL_TIMSEL(0)       |               // use Timer 0 to generate shift clock
        // FLEXIO_SHIFTCTL_TIMPOL       |               // shift on posedge
        FLEXIO_SHIFTCTL_PINCFG(3)       |               // enable pin output
        FLEXIO_SHIFTCTL_PINSEL(0)       |               // select FLEXIO_D0 pin
        // FLEXIO_SHIFTCTL_PINPOL       |               // active high
        FLEXIO_SHIFTCTL_SMOD(2);                        // transmit mode

    FLEXIO3_SHIFTCFG0 =
        FLEXIO_SHIFTCFG_PWIDTH(9)       |               // parallel shift 10 bits
        FLEXIO_SHIFTCFG_INSRC           |               // use next shifter as its input / buffer
        FLEXIO_SHIFTCFG_SSTOP(0)        |               // no stop bits
        FLEXIO_SHIFTCFG_SSTART(0);                      // no start bits

    FLEXIO3_SHIFTCTL1 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG1 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL2 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG2 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL3 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG3 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL4 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG4 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL5 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG5 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL6 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG6 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_INSRC | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    FLEXIO3_SHIFTCTL7 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) | FLEXIO_SHIFTCTL_SMOD(2);
    FLEXIO3_SHIFTCFG7 = FLEXIO_SHIFTCFG_PWIDTH(9) | FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

    NVIC_ENABLE_IRQ(IRQ_FLEXIO3);           // only enable after all shifters have been configured to load from their buffers


    // setup FlexIO timer 0 (4MHz shift clock)
    // TIMCFG should be configured before setting TIMOD
    FLEXIO3_TIMCMP0 = (((2*SHIFTS_PER_REFILL - 1) & 0xFF) << 8) | (30/2 - 1);           // set shifter clock frequency to 1/30 of FlexIO clock's frequency

    FLEXIO3_TIMCFG0 =
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

    FLEXIO3_TIMCTL0 =
        FLEXIO_TIMCTL_PINCFG(00)            |           // timer pin output disabled
        FLEXIO_TIMCTL_TIMOD(01);                        // enable timer in dual 8-bit counters mode
}


void loop() {
    static int count = 0;
    static uint32_t start = micros();

    // check for SHIFTBUF underrun
    if (FLEXIO3_SHIFTERR) {
        uint32_t elapsed_time = micros() - start;
        Serial.printf("[%u] SHIFTERR = 0x%08X occured (%u us elapsed)\n", micros(), FLEXIO3_SHIFTERR, elapsed_time);
        FLEXIO3_SHIFTERR = 0xFF;
    }


    // wait until everything has been transmitted to prevent potential overrun of data buffers (the ISR that performs the transmission disables upon completion)
    while (NVIC_IS_ENABLED(IRQ_FLEXIO3));
    NVIC_ENABLE_IRQ(IRQ_FLEXIO3);

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

    count++;
}
