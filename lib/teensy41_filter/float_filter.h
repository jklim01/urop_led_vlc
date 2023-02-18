/*
    This implementation is specific to the Teensy 4.1 microncontroller. The CMSIS DSP library is used internally to efficiently perform
    the filtering functionality. (repo: https://github.com/ARM-software/CMSIS-DSP)

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
#ifndef FLOAT_FILTER_H
#define FLOAT_FILTER_H

#define ARM_MATH_CM7
#include "arm_math.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "Complex32.h"

#include <type_traits>
#include <functional>
#include <array>

#include "filter_utils.h"
using namespace Teensy41FilterUtils;


namespace FloatFilter {

// utils
template <typename... Ts>
using are_floating_point = and_<std::is_floating_point<Ts>...>;

template <typename... Us>
using are_all_float = std::is_same<std::common_type_t<Us...>, float>;


// FIR Function Impl
template <unsigned int NUM_TAPS, unsigned int X_LEN, unsigned int BLOCK_SIZE>
class FIRFilter {
public:
    constexpr static auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;

private:
    // Note: The filter coefficients are stored in time reversed order.
    std::array<float, X_LEN> _in_buf = { 0.0 };
    const std::array<const float, NUM_TAPS> _h;
    std::array<float, X_LEN> _out_buf = { 0.0 };
    std::array<float, NUM_TAPS + BLOCK_SIZE - 1> _fir_state = { 0.0 };
    arm_fir_instance_f32 _fir_inst;

public:
    static_assert(X_LEN % BLOCK_SIZE == 0, "X_LEN is not an integer multiple of BLOCK_SIZE!");

    // list-initialization (example: float{0.0}) is used to prevent (or generate a warning for) narrowing conversions
    // template <typename... Ts, std::enable_if_t<are_all_float<Ts...>::value, bool> = true>
    template <typename... Ts, std::enable_if_t<are_floating_point<Ts...>::value, bool> = true>
    FIRFilter(Ts... ts) : _h{{float{ts}...}} {
        static_assert(sizeof...(Ts) <= NUM_TAPS, "The number of coefficients provided exceeds NUM_TAPS!");

        // in performing the cast, we assume that the function does not attempt to modify `_h` (which is labeled const)
        arm_fir_init_f32(&_fir_inst, NUM_TAPS, (float*)_h.data(), _fir_state.data(), BLOCK_SIZE);
    }

    float* in_buf() {
        return _in_buf.data();
    }

    float* out_buf() {
        return _out_buf.data();
    }

    using inbuf_iter = typename decltype(_in_buf)::iterator;
    void fill_buf(std::function<void(inbuf_iter, inbuf_iter)> f) {
        f(_in_buf.begin(), _in_buf.end());
    }

    void filter() {
        float* x = _in_buf.data();
        float* y = _out_buf.data();
        for (unsigned int i = 0; i < X_LEN / BLOCK_SIZE; i++)
            arm_fir_f32(&_fir_inst, x + i * BLOCK_SIZE, y + i * BLOCK_SIZE, BLOCK_SIZE);
    }
};


// Partial Convolution Impl
template <unsigned int NUM_TAPS, unsigned int X_LEN>
class PartialConvFilter {
public:
    constexpr static auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;

private:
    const std::array<const float, NUM_TAPS> _h;
    std::array<float, RESULT_SIZE> _in_buf = { 0.0 };
    std::array<float, X_LEN> _out_buf = { 0.0 };
    std::array<float, NUM_TAPS-1> _overlap_buf = { 0.0 };

public:
    // list-initialization (example: float{0.0}) is used to prevent (or generate a warning for) narrowing conversions
    // template <typename... Ts, std::enable_if_t<are_all_float<Ts...>::value, bool> = true>
    template <typename... Ts, std::enable_if_t<are_floating_point<Ts...>::value, bool> = true>
    PartialConvFilter(Ts... ts) : _h{{float{ts}...}} {
        static_assert(sizeof...(Ts) <= NUM_TAPS, "The number of coefficients provided exceeds NUM_TAPS!");
    }

    constexpr float* in_buf() {
        return _in_buf.data() + NUM_TAPS - 1;
    }

    constexpr float* out_buf() {
        return _out_buf.data();
    }

    using inbuf_iter = typename decltype(_in_buf)::iterator;
    void fill_buf(std::function<void(inbuf_iter, inbuf_iter)> f) {
        // include overlap from last iteration
        memcpy(_in_buf.data(), _overlap_buf.data(), _overlap_buf.size() * sizeof(float));

        f(_in_buf.begin() + NUM_TAPS - 1, _in_buf.end());

        // save overlap for next iteration
        memcpy(_overlap_buf.data(), _in_buf.data() + X_LEN, _overlap_buf.size() * sizeof(float));
    }

    void filter() {
        // FIXME: works for now but there could be issues in creating the out-of-bounds pointer (_out_buf.data() - (NUM_TAPS - 1)), like
        // - the construction of the out-of-bounds pointer is undefined behaviour
        // - the internals of the conv function could change to dereference the pointer before adding the offset, accessing out-of-bounds memory
        //   (implementation as of 3 Dec 2022: https://github.com/ARM-software/CMSIS-DSP/blob/main/Source/FilteringFunctions/arm_conv_partial_f32.c)
        // the only reason we resort to this method is to avoid making _out_buf larger than it needs to be
        float* x = _in_buf.data();
        float* y = _out_buf.data() - (NUM_TAPS - 1);

        // in performing the cast, we assume that the function does not attempt to modify `_h` (which is labeled const)
        arm_conv_partial_f32(x, RESULT_SIZE, (float*)_h.data(), NUM_TAPS, y, NUM_TAPS-1, X_LEN);
    }
};


// FFT Filtering Impl
template <unsigned int NUM_TAPS, unsigned int X_LEN>
class FFTFilter {
public:
    constexpr static auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;
    constexpr static auto FFT_SIZE = next_powof2<RESULT_SIZE>();

private:
    std::array<float, FFT_SIZE> _H = { 0.0 };
    std::array<float, FFT_SIZE> _in_buf = { 0.0 };
    std::array<float, FFT_SIZE> _out_buf = { 0.0 };
    std::array<float, NUM_TAPS-1> _overlap_buf = { 0.0 };
    arm_rfft_fast_instance_f32 _rfft_inst;

public:
    static_assert(FFT_SIZE >= 32 &&  FFT_SIZE <= 4096, "The required `FFT_SIZE` is not supported (32/64/128/256/512/1024/2048/4096)!");

    // default constructor (if the user wishes to specify the frequency response instead of the impulse response)
    FFTFilter() {
        // this function fails if the assertion above on `FFT_SIZE` is not satisfied, but constructors are not allowed
        // to return anything so the assertion above is required to ensure we don't construct an unusable object
        arm_rfft_fast_init_f32(&_rfft_inst, FFT_SIZE);
    }

    // list-initialization (example: float{0.0}) is used to prevent (or generate a warning for) narrowing conversions
    // template <typename... Ts, std::enable_if_t<are_all_float<Ts...>::value, bool> = true>
    template <typename... Ts, std::enable_if_t<are_floating_point<Ts...>::value, bool> = true>
    FFTFilter(Ts... ts) : _in_buf{{float{ts}...}} {
        static_assert(sizeof...(Ts) <= NUM_TAPS, "The number of provided coefficients provided exceeds `NUM_TAPS`!");
        arm_rfft_fast_init_f32(&_rfft_inst, FFT_SIZE);

        // get frequency response of the filter, whose impulse response has been temporarily stored in `in_buf`
        arm_rfft_fast_f32(&_rfft_inst, _in_buf.data(), _H.data(), 0);

        // clear _in_buf to all zeros
        for (unsigned int i = 0; i < _in_buf.size(); i++)
            _in_buf[i] = 0.0;
    }

    float* in_buf() {
        return _in_buf.data() + NUM_TAPS - 1;
    }

    float* out_buf() {
        return _out_buf.data() + NUM_TAPS - 1;
    }

    using H_iter = typename decltype(_H)::iterator;
    // if only linear convolution is intended, the user must ensure that ifft(_H) has finite duration of at most NUM_TAPS
    void fill_H(std::function<void(H_iter, H_iter)> f) {
        f(_H.begin(), _H.end());
    }

    using inbuf_iter = typename decltype(_in_buf)::iterator;
    void fill_buf(std::function<void(inbuf_iter, inbuf_iter)> f) {
        // include overlap from last iteration
        memcpy(_in_buf.data(), _overlap_buf.data(), _overlap_buf.size() * sizeof(float));

        f(_in_buf.begin() + (NUM_TAPS - 1), _in_buf.begin() + RESULT_SIZE);

        // save overlap for next iteration
        memcpy(_overlap_buf.data(), _in_buf.data() + X_LEN, _overlap_buf.size() * sizeof(float));
    }

    void filter() {
        float* x1 = _in_buf.data();
        float* x2 = _out_buf.data();

        arm_rfft_fast_f32(&_rfft_inst, x1, x2, 0);

        // in performing the cast, we assume that the function does not attempt to modify `_h` (which is labeled const)
        x1[0] = x2[0] * _H[0];                                                                      // (real) DC component
        x1[1] = x2[1] * _H[1];                                                                      // (real) componnet at Nyquist frequency
        arm_cmplx_mult_cmplx_f32(x2 + 2, (float*)_H.data() + 2, x1 + 2, FFT_SIZE/2 - 1);         // all other complex components

        arm_rfft_fast_f32(&_rfft_inst, x1, x2, 1);
    }
};

}

#endif
