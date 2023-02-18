/*
    This code OOK-modulates a stream of bits, RRC-filters the signal, maps each sample to a 10-bit binary code, and
    parallle outputs them through the GPIO pins using FlexIO. This particular implementation involves filling the
    FlexIO3 shifter buffers using interrupts.

    Note:
    - adjustable parameters
        - `ONLINE`  : receievs the bits-to-transmit from "arduino_modulation_sim.m" if defined, otherwise take from local array
        - `VERSION` : specify the filtering method
        - `X_SPAN`  : number of bits-to-transmit in each filtering batch
        - `SPAN`    : number of symbols spanned by filter
        - `SPS`     : samples per symbool
        - `FLEXIO2_CLK_PODF_DIVIDER`, `FLEXIO2_CLK_PRED_DIVIDER`    : specify clk dividers for clk supply to FlexIO module
        - `FLEXIO2_BAUD_RATE_DIVIDER`                               : specify ratio between FlexIO clk and sampling rate

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
#include "utils.h"

#define ARM_MATH_CM7
#include "arm_math.h"

// if ONLINE is defined, the byte sequence to transmit will be read through the USB Serial port every iteration (significant delay in the transmitter is intolerable)
// otherwise, the byte sequence will be stored locally
// #define ONLINE
#ifndef ONLINE
#include "vlc_byte_seq.h"
extern const uint8_t byte_seq[] PROGMEM;
#endif

#define FIR_FILTER 0
#define CONV_FILTER 1
#define FFT_FILTER 2
#define FFT_FILTER_MANUAL 3
#define FIR_FILTER_Q15 4
#define CONV_FILTER_Q15 5
#define VERSION FIR_FILTER_Q15

// #define SPS4
#ifdef SPS4
constexpr auto X_SPAN = 24;
#else
// #if VERSION == FFT_FILTER || VERSION == FFT_FILTER_MANUAL
constexpr auto X_SPAN = 96;                                         // 2 samples wasted, alt 1024(xspan=192 wastes 34) 2048(384,98)
// #else
// constexpr auto X_SPAN = 48;
// #endif
#endif


#define FLOAT 6
#define Q15 7
#if VERSION == FIR_FILTER || VERSION == CONV_FILTER || VERSION == FFT_FILTER || VERSION == FFT_FILTER_MANUAL
#define NUM_TYPE FLOAT
#define NUM_TYPE_NAME float
#include "float_filter.h"
using namespace FloatFilter;

#elif VERSION == FIR_FILTER_Q15 || VERSION == CONV_FILTER_Q15
#define NUM_TYPE Q15
#define NUM_TYPE_NAME q15_t
#include "q15_filter.h"
using namespace Q15Filter;

#endif


#if VERSION == FIR_FILTER || VERSION == CONV_FILTER
#if VERSION == FIR_FILTER
constexpr auto BLOCK_SIZE = X_SPAN;
#define VAR_DECL static FIRFilter<NUM_TAPS, X_SPAN*SPS, BLOCK_SIZE> rrc_filter
#elif VERSION == CONV_FILTER
#define VAR_DECL static PartialConvFilter<NUM_TAPS, X_SPAN*SPS> rrc_filter
#endif
#ifdef SPS4
#include "rrc_float_4sps.inc"
#else
#include "rrc_float_5sps.inc"
#endif

#elif VERSION == FFT_FILTER || VERSION == FFT_FILTER_MANUAL
#define VAR_DECL static FFTFilter<NUM_TAPS, X_SPAN*SPS> rrc_filter
#ifdef SPS4
#include "rrc_float_4sps.inc"
#else
#include "rrc_float_5sps.inc"
#endif

#if VERSION == FFT_FILTER_MANUAL
#include "populate_RRC_filter_half.inc"
#endif

#elif VERSION == FIR_FILTER_Q15 || VERSION == CONV_FILTER_Q15
#if VERSION == FIR_FILTER_Q15
// add 1 more tap to the filter since the FIR impl of Q15Filter requires `numTaps` to be even
// the docs say to add one more tap and set the last coefficient to 0
// (we interpret this to mean "last in time", otherwise we will add on an extra delay of 1 sample to the filter)
constexpr auto BLOCK_SIZE = X_SPAN;
#define PADDED
#define VAR_DECL static FIRFilterQ15<NUM_TAPS+1, X_SPAN*SPS, BLOCK_SIZE> rrc_filter

#elif VERSION == CONV_FILTER_Q15
#define VAR_DECL static PartialConvFilterQ15<NUM_TAPS, X_SPAN*SPS> rrc_filter

#endif

#ifdef SPS4
#include "rrc_q15_4sps.inc"
#else
#include "rrc_q15_5sps.inc"
#endif

#endif


constexpr auto GROUP_DELAY = (SPAN * SPS) / 2;
constexpr auto X_LEN = X_SPAN * SPS;
constexpr auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;
static_assert(X_SPAN % 8 == 0, "X_SPAN is not a multiple of 8 (8 bits per byte)");


// shifters 0 and 4 do 4-bit parallel shift and each have an "effective shift register width" of 96 bits
// thus, the shifters will need to be reloaded from the shift buffers after 24 shifts
constexpr auto NUM_FLEXIO_SHIFTBUF = 8;
constexpr auto SHIFTS_PER_REFILL = 24;
constexpr auto SHIFT_BUFFER_SIZE = 32;
static_assert((SPS * X_SPAN) % SHIFTS_PER_REFILL == 0, "SPS * X_SPAN is not a multiple of SHIFTS_PER_REFILL!");

#ifdef ONLINE
constexpr auto NUM_ITERS = 1'000'000;
#else
constexpr auto NUM_ITERS = sizeof(byte_seq) / (X_SPAN / 8);
#endif

constexpr uint8_t FLEXIO2_INTERRUPT_PRIORITY = 64;
constexpr auto REFILLS_PER_DATABUF = (X_LEN * 10) / (SHIFTS_PER_REFILL * 10) ;       // (# bits to transfer per databuf) / (# bits transferred per refill) with 10-bit output

constexpr uint8_t FLEXIO2_CLK_PODF_DIVIDER = 2;
constexpr uint8_t FLEXIO2_CLK_PRED_DIVIDER = 2;
#ifdef SPS4
constexpr uint8_t FLEXIO2_BAUD_RATE_DIVIDER = 30;
#else
constexpr uint8_t FLEXIO2_BAUD_RATE_DIVIDER = 24;
#endif
constexpr auto SAMPLING_FREQ_MHZ = (double)480.0 / FLEXIO2_CLK_PODF_DIVIDER / FLEXIO2_CLK_PRED_DIVIDER / FLEXIO2_BAUD_RATE_DIVIDER;     // assume default PLL3 frequency
static_assert(FLEXIO2_CLK_PODF_DIVIDER != 0 && FLEXIO2_CLK_PODF_DIVIDER <= 8, "FLEXIO2_CLK_PODF_DIVIDER must be in [1, 8]!");
static_assert(FLEXIO2_CLK_PRED_DIVIDER != 0 && FLEXIO2_CLK_PRED_DIVIDER <= 8, "FLEXIO2_CLK_PRED_DIVIDER must be in [1, 8]!");
static_assert(FLEXIO2_BAUD_RATE_DIVIDER % 2 == 0, "FLEXIO2_BAUD_RATE_DIVIDER should be even to prevent truncating division!");


#if NUM_TYPE == Q15
// The final desired normalization factor at the filter output is MAX_AMPLITUDE_OUT. Meanwhile, the normalization factor for the convolution result will be the product
// of the normalization factors for the input signal and filter. Since the input signal is just a pulse train where a sample can be 0 or 1, it probably won't require
// too much precision. Thus, we can increase our achieved precision by concentrating our precision on the filter coefficients, which can be done by decreasing its
// normalization factor. In this case, we choose the value such that the maximum positive value of q15_t (32767) corresponds to the filter's largest coefficient.
// ! Given the actual value for the maximum possible output amplitude X, the correct normalization factor at the output should actually be X / 32767 * 32768. This
//   is because the largest positive value that q15_t can take is 32767 = 2^15 - 1. However, since MAX_AMPLITUDE_OUT is an overapproximation, this should be safe.
constexpr auto Q15_NORMALIZATION_H = MAX_AMPLITUDE_FILTER / 32767 * 32768;
constexpr auto Q15_NORMALIZATION_IN = MAX_AMPLITUDE_OUT / Q15_NORMALIZATION_H;
constexpr auto ONE_Q15_IN = (q15_t)(1 / Q15_NORMALIZATION_IN * 32768 + 0.5);                // + 0.5 to perform rounding instead of truncation
constexpr auto MIN_AMPLITUDE_Q15_OUT = (q15_t)(MIN_AMPLITUDE_OUT / MAX_AMPLITUDE_OUT * 32768 + 0.5);
constexpr auto MAX_AMPLITUDE_Q15_OUT = 32767;

#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
__STATIC_FORCEINLINE uint32_t __UADD16(uint32_t a, uint32_t b) {
	uint32_t out;
	asm volatile("uadd16 %0, %1, %2" : "=r" (out) : "r" (a), "r" (b));
	return(out);
}
void convert_to_10bit_binary_code(q15_t* in, uint16_t* out, size_t len) {
    constexpr uint32_t OFFSET_PAIR = ((uint32_t)MIN_AMPLITUDE_Q15_OUT & 0x0000FFFF) | (((uint32_t)MIN_AMPLITUDE_Q15_OUT << 16) & 0xFFFF0000);
    constexpr uint16_t RANGE = (uint16_t)MIN_AMPLITUDE_Q15_OUT + (uint16_t)MAX_AMPLITUDE_Q15_OUT;

    uint32_t* in_ptr = (uint32_t*)in;
    uint32_t* out_ptr = (uint32_t*)out;
    for (size_t i = 0; i < len / 2; i++) {
        // !!! there might be bugs in the release build if the ret value of `__UADD16` is not stored in an intermediate variable first
        // (build "uadd16_test.cpp" in release mode with debug symbols and check the disassembly)
        uint32_t res = __UADD16(OFFSET_PAIR, in_ptr[i]);          // used to perform 2 uint16_t additions in 1 instruction

        uint32_t res1 = (((res & 0x0000FFFF) * UINT16_MAX + (RANGE/2)) / RANGE) >> 6;
        uint32_t res2 = ((((res & 0xFFFF0000) >> 16) * UINT16_MAX + (RANGE/2)) / RANGE) >> 6;
        out_ptr[i] = (res1 & 0x0000FFFF) | ((res2 << 16) & 0xFFFF0000);
    }

    if (len % 2 != 0)
        out[len - 1] = ((((uint32_t)(in[len - 1] + MIN_AMPLITUDE_Q15_OUT)) * UINT16_MAX + (RANGE/2)) / RANGE) >> 6;
}

#elif NUM_TYPE == FLOAT
void convert_to_10bit_binary_code(float* in, uint16_t* out, size_t len) {
    for (size_t i = 0; i < len; i++)
        out[i] = (uint16_t)((in[i] + MIN_AMPLITUDE_OUT) * ((float)UINT16_MAX / (MIN_AMPLITUDE_OUT + MAX_AMPLITUDE_OUT))) >> 6;
}

#endif


utils::TicToc benchmarker;

static uint32_t storage_buf_1[NUM_FLEXIO_SHIFTBUF * REFILLS_PER_DATABUF] = { 0 };
static uint32_t storage_buf_2[NUM_FLEXIO_SHIFTBUF * REFILLS_PER_DATABUF] = { 0 };
constexpr auto STORAGE_BUF_SIZE = sizeof(storage_buf_1);
static void fill_storage_buf(uint32_t* storage_buf_ptr, uint16_t* src, int soff=sizeof(uint16_t)) {
    memset(storage_buf_ptr, 0, STORAGE_BUF_SIZE);
    uint8_t* src_ptr = (uint8_t*)src;

    for (int i = 0; i < REFILLS_PER_DATABUF; i++) {
        uint8_t* buf_ptr_0 = (uint8_t*)(storage_buf_ptr + NUM_FLEXIO_SHIFTBUF*i);
        uint8_t* buf_ptr_4 = (uint8_t*)(storage_buf_ptr + NUM_FLEXIO_SHIFTBUF*i + 4);
        uint32_t* buf_ptr_3 = storage_buf_ptr + NUM_FLEXIO_SHIFTBUF*i + 3;
        uint32_t* buf_ptr_7 = storage_buf_ptr + NUM_FLEXIO_SHIFTBUF*i + 7;

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
}

static void my_isr() {
    static uint8_t refill_count = 0;
    static uint32_t* buf_ptr = storage_buf_1;

    // if something weird happens, try manually clearing the SSF flag
    // FLEXIO2_SHIFTSTAT = 1;

    for (size_t i = 0; i < SHIFT_BUFFER_SIZE / sizeof(FLEXIO2_SHIFTBUF0); i++) {
        (&FLEXIO2_SHIFTBUF0)[i] = *buf_ptr;
        buf_ptr++;
    }
    refill_count++;


    switch (refill_count) {
        case REFILLS_PER_DATABUF: {
            NVIC_DISABLE_IRQ(IRQ_FLEXIO2);          // prevent underrun of data buffers (try moving to the start if weird things happend)
            buf_ptr = (uint32_t*)storage_buf_2;
            break;
        }

        case 2*REFILLS_PER_DATABUF: {
            NVIC_DISABLE_IRQ(IRQ_FLEXIO2);          // prevent underrun of data buffers
            refill_count = 0;
            buf_ptr = (uint32_t*)storage_buf_1;
            break;
        }
    }

    asm volatile("dsb");
}


void setup() {

    // filtering setup
#if VERSION == FFT_FILTER_MANUAL
    rrc_filter.fill_H(populate_RRC_filter_half);
#endif


    // setup IRQ for transfer to SHIFTBUF
    attachInterruptVector(IRQ_FLEXIO2, &my_isr);
    NVIC_SET_PRIORITY(IRQ_FLEXIO2, FLEXIO2_INTERRUPT_PRIORITY);


    // configure muxes to set clock frequency supplied to FlexIO2 module to 120MHz
    // default frequency  =  PLL3 frequency / CCM_CS1CDR_FLEXIO2_CLK_PRED / CCM_CS1CDR_FLEXIO2_CLK_PODF  =  480MHz / 2 / 8  =  30MHz
    CCM_CS1CDR &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF(7) | CCM_CS1CDR_FLEXIO2_CLK_PRED(7));       // clear default bits (111 for both)
    CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(FLEXIO2_CLK_PODF_DIVIDER-1)                   // set clock dividers
        | CCM_CS1CDR_FLEXIO2_CLK_PRED(FLEXIO2_CLK_PRED_DIVIDER-1);

    // enable FlexIO2 functional clock
    // - must be enabled before accessing any of its registers
    CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);


    // enable FlexIO2
    FLEXIO2_CTRL |= 1;

    // enable interrupt generation when FlexIO2 SSF flag is set
    FLEXIO2_SHIFTSIEN |= 1;


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


    // 4-bit parallel shift out with shifter 4 to FlexIO2 pins 16~19, choosing shifters 5 and 6 as its buffer
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


    NVIC_ENABLE_IRQ(IRQ_FLEXIO2);           // only enable after all shifters have been configured to load from their buffers


    // setup FlexIO2 timer 0
    // TIMCFG should be configured before setting TIMOD
    FLEXIO2_TIMCMP0 = (((2*SHIFTS_PER_REFILL - 1) & 0xFF) << 8) | (FLEXIO2_BAUD_RATE_DIVIDER/2 - 1);    // 8-bit baud counter mode

    FLEXIO2_TIMCFG0 =
        FLEXIO_TIMCFG_TIMOUT(10)            |           // timer output logic 1 upon enable and not affected by timer reset
        FLEXIO_TIMCFG_TIMDEC(00)            |           // decrement counter on FLEXIO clock, shift clock is timer output
        FLEXIO_TIMCFG_TIMRST(000)           |           // timer never resets
        FLEXIO_TIMCFG_TIMDIS(000)           |           // timer never disables
        FLEXIO_TIMCFG_TIMENA(000)           |           // timer always enabled
        FLEXIO_TIMCFG_TSTOP(00);                        // stop bit disabled
        // | FLEXIO_TIMCFG_TSTART                       // start bit disabled


    while(!Serial);
    Serial.printf(
#if VERSION == FIR_FILTER
        "FIR Filtering (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#elif VERSION == CONV_FILTER
        "Partial Convolution Filtering (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#elif VERSION == FFT_FILTER
        "FFT Filtering (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#elif VERSION == FFT_FILTER_MANUAL
        "FFT Filtering manual (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#elif VERSION == FIR_FILTER_Q15
        "FIR Filtering q15 (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#elif VERSION == CONV_FILTER_Q15
        "Partial Convolution Filtering q15 (FlexIO2 interrupt, SPS=%d, fs=%.2fMHz):\n",
#endif
        SPS, SAMPLING_FREQ_MHZ);

#ifdef ONLINE
    Serial.write((byte*)&X_SPAN, sizeof(X_SPAN));
    Serial.write((byte*)&NUM_ITERS, sizeof(NUM_ITERS));
#endif


    // only enable FlexIO timer to start shifting after serial connection is established (prevents SHIFTERR in online case)
    FLEXIO2_TIMCTL0 =
        FLEXIO_TIMCTL_PINCFG(00)            |           // timer pin output disabled
        FLEXIO_TIMCTL_TIMOD(01);                        // enable timer in dual 8-bit counters mode
}


void loop() {
    static unsigned int count = 0;
    static uint32_t start = micros();
    static uint32_t benchmark_start = 0;
    static uint32_t benchmark_end = 0;
    static uint32_t idle_time = 0;
    static uint32_t idle_max = 0;
    static uint32_t idle_min = UINT32_MAX;
    static double idle_std = 0;
    static unsigned int max_count = 0;
    static unsigned int min_count = 0;


    static uint32_t* inuse_storage_buf = storage_buf_1;
    static uint32_t* free_storage_buf = storage_buf_2;


    // check for SHIFTBUF underrun
    if (FLEXIO2_SHIFTERR) {
        uint32_t elapsed_time = micros() - start;
        Serial.printf("[%u] SHIFTERR = 0x%08X occured at count = %u (%u us elapsed)\n", micros(), FLEXIO2_SHIFTERR, count, elapsed_time);
        FLEXIO2_SHIFTERR = 0xFF;
    }


    if (count == NUM_ITERS) {
        // account for delayed samples in last iteration (slightly modified version of usual loop body)
        rrc_filter.fill_buf([](auto begin, auto end) {
            for (auto it = begin; it != end; it++)
                *it = 0;
        });
        rrc_filter.filter();
        convert_to_10bit_binary_code(rrc_filter.out_buf(), (uint16_t*)rrc_filter.out_buf(), X_LEN);
        fill_storage_buf(free_storage_buf, (uint16_t*)rrc_filter.out_buf());

        while (NVIC_IS_ENABLED(IRQ_FLEXIO2));
        NVIC_ENABLE_IRQ(IRQ_FLEXIO2);
        count++;

        if (FLEXIO2_SHIFTERR) {
            uint32_t now = micros();
            Serial.printf("[%u] SHIFTERR = 0x%08X occured at count = %u (%u us elapsed)\n", now, FLEXIO2_SHIFTERR, count, now - start);
        }

        while (NVIC_IS_ENABLED(IRQ_FLEXIO2));
        FLEXIO2_TIMCTL0 = 0;

#ifdef ONLINE
        Serial.print("0\n");                                                            // signal the end
#endif                                                           // signal the end

        uint32_t total_time = benchmark_end - benchmark_start;
        double f_cpu_MHz = F_CPU_ACTUAL / 1'000'000;
        double idle_mean = (double)idle_time / (NUM_ITERS - 1);
        idle_std -= pow(idle_mean, 2) / (NUM_ITERS - 2) * (NUM_ITERS - 1);              // reduce DOF by 1
        idle_std = sqrt(idle_std);
        Serial.printf("I was idle for %.4f us / %u counter ticks (F_CPU_ACTUAL = %u Hz) over %d iterations lasting %u us!\n",
            (double)idle_time / f_cpu_MHz, idle_time, F_CPU_ACTUAL, NUM_ITERS - 1, total_time);
        Serial.printf("(Idle Time: %2.2f%%, Busy Time: %2.2f%%)\n",
            (double)idle_time / f_cpu_MHz / total_time * 100, (total_time - idle_time/f_cpu_MHz) / total_time * 100);
        Serial.printf("\nIdle time stats for each filtering iteration:\n");
        Serial.printf("MAX  -> %8.4f us (count = %u)\n", (double)idle_max / f_cpu_MHz, max_count);
        Serial.printf("MIN  -> %8.4f us (count = %u)\n", (double)idle_min / f_cpu_MHz, min_count);
        Serial.printf("MEAN -> %8.4f us\n", idle_mean / f_cpu_MHz);
        Serial.printf("STD  -> %8.4f us\n", idle_std / f_cpu_MHz);

#ifdef ONLINE
        Serial.print("0\n");
#endif
        utils::panic(0);
    }


    // read bytes into data buffer
#ifdef ONLINE
    rrc_filter.fill_buf([](auto it, auto end) {
        uint8_t buf;
        for (int byte_count = 0; byte_count < X_SPAN / 8; byte_count++) {
            while (!Serial.available());
            if ((buf = Serial.read()) == -1)
                continue;

            for (uint8_t i = 0; i < 8; i++) {
#if NUM_TYPE == FLOAT
                float val = (buf & 0x80) ? 1.0 : 0.0;                               // MSB first
#elif NUM_TYPE == Q15
                q15_t val = (buf & 0x80) ? ONE_Q15_IN : 0;                          // MSB first
#endif
                for (uint8_t j = 0; j < SPS; j++) {
                    *it = val;
                    x1_ptr++;
                }
                buf <<= 1;
            }
        }
    });
#else
    static const uint8_t* byte_seq_ptr = byte_seq;
    rrc_filter.fill_buf([](auto it, auto end) {
        uint8_t buf;

        for (int byte_count = 0; byte_count < X_SPAN / 8; byte_count++) {
            buf = *byte_seq_ptr;
            byte_seq_ptr++;

            for (uint8_t i = 0; i < 8; i++) {
#if NUM_TYPE == FLOAT
                float val = (buf & 0x80) ? 1.0 : 0.0;                               // MSB first
#elif NUM_TYPE == Q15
                q15_t val = (buf & 0x80) ? ONE_Q15_IN : 0;                          // MSB first
#endif
                for (uint8_t j = 0; j < SPS; j++) {
                    *it = val;
                    it++;
                }
                buf <<= 1;
            }
        }
    });
#endif


    // perform filtering
    rrc_filter.filter();


    // convert to 10-bit binary code
    convert_to_10bit_binary_code(rrc_filter.out_buf(), (uint16_t*)rrc_filter.out_buf(), X_LEN);
    fill_storage_buf(free_storage_buf, (uint16_t*)rrc_filter.out_buf());


    // wait until everything has been transmitted to prevent potential overrun of data buffers (the ISR that performs the transmission disables upon completion)
    uint32_t idle_cycles = benchmarker.timeit([]() {
        while (NVIC_IS_ENABLED(IRQ_FLEXIO2));
    });

    // exclude first iteration from measurements
    if (count == 0)
        benchmark_start = micros();
    else if (count == NUM_ITERS-1)
        benchmark_end = micros();

    NVIC_ENABLE_IRQ(IRQ_FLEXIO2);

    if (count != 0) {
        idle_time += idle_cycles;
        if (idle_cycles > idle_max) {
            idle_max = idle_cycles;
            max_count = count;
        }
        if (idle_cycles < idle_min) {
            idle_min = idle_cycles;
            min_count = count;
        }
        idle_std += (double)idle_cycles / (NUM_ITERS - 2) * idle_cycles;            // reduce DOF by 1
    }

    uint32_t* temp = free_storage_buf;
    free_storage_buf = inuse_storage_buf;
    inuse_storage_buf = temp;
    count++;
}
