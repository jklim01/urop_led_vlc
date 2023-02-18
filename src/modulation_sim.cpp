/*
    Run this code while the board is connected to a computer running `arduino_modulation_sim.m`. This code accepts the
    sequence of bits sent by the computer and generates the samples for the corresponding OOK-modulated and RRC-filtered
    signal as 10-bit binary codes. The processing time of each stage (read bits, modulation, filtering) are sent to the
    computer too.

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

#define FIR_FILTER 0
#define CONV_FILTER 1
#define FFT_FILTER 2
#define FFT_FILTER_MANUAL 3
#define FIR_FILTER_Q15 4
#define CONV_FILTER_Q15 5
#define VERSION FFT_FILTER_MANUAL

// #define SPS4
#ifdef SPS4
constexpr auto X_SPAN = 56;
#else
// #if VERSION == FFT_FILTER || VERSION == FFT_FILTER_MANUAL
constexpr auto X_SPAN = 96;
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
constexpr auto BLOCK_SIZE = 48;
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
constexpr auto BLOCK_SIZE = 40;
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


constexpr auto NUM_ITERS = 2;
constexpr auto GROUP_DELAY = (SPAN * SPS) / 2;
constexpr auto X_LEN = X_SPAN * SPS;
constexpr auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;
static_assert(X_SPAN % 8 == 0, "X_SPAN is not a multiple of 8 (8 bits per byte)");
static_assert(NUM_ITERS >= 2, "NUM_ITERS needs to be at least 2 (to reduce DOF for variance calculation)");


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
void write_array(void* buf_ptr, size_t bytes_per_elem, size_t num_elems, int elem_offset){
    byte* ptr = (byte*)buf_ptr;
    for (size_t i = 0; i < num_elems; i++) {
        Serial.write(ptr + i * elem_offset, bytes_per_elem);
    }
    Serial.send_now();
}


void setup() {

#if VERSION == FFT_FILTER_MANUAL
    rrc_filter.fill_H(populate_RRC_filter_half);
#endif

    while(!Serial);
    Serial.println(
#if VERSION == FIR_FILTER
        "FIR Filtering:"
#elif VERSION == CONV_FILTER
        "Partial Convolution Filtering:"
#elif VERSION == FFT_FILTER
        "FFT Filtering:"
#elif VERSION == FFT_FILTER_MANUAL
        "FFT Filtering (manual):"
#elif VERSION == FIR_FILTER_Q15
        "FIR Filtering (q15):"
#elif VERSION == CONV_FILTER_Q15
        "Partial Convolution Filtering (q15):"
#endif
    );

    // send parameters
    Serial.write((byte*)&NUM_ITERS, sizeof(NUM_ITERS));
    Serial.write((byte*)&X_SPAN, sizeof(X_SPAN));
    Serial.write((byte*)&SPAN, sizeof(SPAN));
    Serial.write((byte*)&SPS, sizeof(SPS));
    Serial.write((byte*)&BETA, sizeof(BETA));
    Serial.write((byte*)&MAX_AMPLITUDE_OUT, sizeof(MAX_AMPLITUDE_OUT));
    Serial.write((byte*)&MIN_AMPLITUDE_OUT, sizeof(MIN_AMPLITUDE_OUT));
}


void loop() {
    unsigned long cycles;
    double time_taken;
    double rx_time = 0;
    double rx_time_sqr = 0;
    double filtering_time = 0;
    double filtering_time_sqr = 0;
    double mapping_time = 0;
    double mapping_time_sqr = 0;


    for (int i = 0; i < NUM_ITERS; i++) {

        // read bits into buffer
        while (!Serial.available());
        cycles = benchmarker.timeit([]() {
            rrc_filter.fill_buf([](auto it, auto end) {
                for (int byte_count = 0; byte_count < X_SPAN / 8; byte_count++) {
                    int buf;

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
                            it++;
                        }
                        buf <<= 1;
                    }
                }
            });
        });

        time_taken = (double)cycles / (F_CPU_ACTUAL / 1'000'000);
        Serial.printf("I took %.2f us to receive the bits and fill the buffer!\n", time_taken);
        rx_time += time_taken / NUM_ITERS;
        rx_time_sqr += time_taken / (NUM_ITERS - 1) * time_taken;       // use (NUM_ITERS - 1) instead to reduce DOF in variance calculation by 1


        // filtering
        cycles = benchmarker.timeit([]() { rrc_filter.filter(); });

        time_taken = (double)cycles / (F_CPU_ACTUAL / 1'000'000);
        Serial.printf("I took %.2f us to perform the filtering!\n", time_taken);
        filtering_time += time_taken / NUM_ITERS;
        filtering_time_sqr += time_taken / (NUM_ITERS - 1) * time_taken;


        // map to 10-bit binary code
        cycles = benchmarker.timeit([]() {
            convert_to_10bit_binary_code(rrc_filter.out_buf(), (uint16_t*)rrc_filter.out_buf(), X_LEN);
        });

        time_taken = (double)cycles / (F_CPU_ACTUAL / 1'000'000);
        Serial.printf("I took %.2f us to map the values to binary codes!\n", time_taken);
        mapping_time += time_taken / NUM_ITERS;
        mapping_time_sqr += time_taken / (NUM_ITERS - 1) * time_taken;


        // transmit filtered signal
        write_array(rrc_filter.out_buf(), sizeof(uint16_t), X_LEN, sizeof(uint16_t));
    }


    // account for delayed samples in last iteration
    rrc_filter.fill_buf([](auto begin, auto end) {
        for (auto it = begin; it != end; it++)
            *it = 0;
    });
    rrc_filter.filter();
    convert_to_10bit_binary_code(rrc_filter.out_buf(), (uint16_t*)rrc_filter.out_buf(), X_LEN);

    write_array(rrc_filter.out_buf(), sizeof(uint16_t), 2 * GROUP_DELAY, sizeof(uint16_t));


    // send total processing times
    Serial.write((byte*)&rx_time, sizeof(rx_time));
    Serial.write((byte*)&filtering_time, sizeof(filtering_time));
    Serial.write((byte*)&mapping_time, sizeof(mapping_time));

    Serial.write((byte*)&rx_time_sqr, sizeof(rx_time_sqr));
    Serial.write((byte*)&filtering_time_sqr, sizeof(filtering_time_sqr));
    Serial.write((byte*)&mapping_time_sqr, sizeof(mapping_time_sqr));

    while(1);
}
