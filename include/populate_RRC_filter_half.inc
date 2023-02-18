/*
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

#pragma once
#ifndef POPULATE_RRC_FILTER_HALF_NICER_H
#define POPULATE_RRC_FILTER_HALF_NICER_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <Complex32.h>
using namespace Complex;

#include <float_filter.h>


// assumes SPS > 1 + BETA
static void populate_RRC_filter_half(FloatFilter::FFTFilter<NUM_TAPS, X_SPAN*SPS>::H_iter it, FloatFilter::FFTFilter<NUM_TAPS, X_SPAN*SPS>::H_iter end) {
    constexpr auto FFT_SIZE = FloatFilter::FFTFilter<NUM_TAPS, X_SPAN*SPS>::FFT_SIZE;
    static_assert(SPAN * SPS % 2 == 0, "SPAN * SPS must be even");

    constexpr uint16_t transition_band_left_edge = (uint16_t)((1.0 - BETA) / 2 / SPS * FFT_SIZE + 1);
    constexpr uint16_t out_of_band_left_edge = (uint16_t)((1.0 + BETA) / 2 / SPS * FFT_SIZE + 1);

    uint16_t idx = 0;
    constexpr float PHASE_SHIFT = -1.0 * M_PI / FFT_SIZE * SPS * SPAN;
    Complex32 phase_shift_inc = Complex32(cosf(PHASE_SHIFT), sinf(PHASE_SHIFT));
    Complex32 phase_shift_factor = Complex32(1.0, 0.0);

    // passband
    // ! components at DC and nyquist frequency automatically handled correctly in H[0]
    // (SPS > 1 + BETA guarantees nyquist frequency out-of-band => magnitude 0)
    for (; idx < transition_band_left_edge; idx++) {
        *it = phase_shift_factor.re;
        it++;
        *it = phase_shift_factor.im;
        it++;
        phase_shift_factor *= phase_shift_inc;
    }

    // transition band |Hk|
    // = sqrt({1 + cos[ (SPS*PI/BETA/FFT_SIZE)   * idx -  PI*(1-BETA)/2/BETA ]} / 2)
    // = sqrt({1 + cos[      `cos_arg_inc`       * idx - (PI/2)*(1-BETA)/BETA]} / 2)
    constexpr float cos_arg_inc = M_PI / FFT_SIZE / BETA * SPS;
    float cos_arg = cos_arg_inc * idx - M_PI_2 * (1.0 - BETA) / BETA;
    for (; idx < out_of_band_left_edge; idx++) {
        float mag = sqrtf(0.5 + cosf(cos_arg) / 2.0);
        *it = phase_shift_factor.re * mag;
        it++;
        *it = phase_shift_factor.im * mag;
        it++;
        cos_arg += cos_arg_inc;
        phase_shift_factor *= phase_shift_inc;
    }

    // out of band
    while (it != end) {
        *it = 0.0;
        it++;
        *it = 0.0;
        it++;
    }
}

#endif