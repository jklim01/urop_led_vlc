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
#ifndef Q15_FILTER_H
#define Q15_FILTER_H

#define ARM_MATH_CM7
#include "arm_math.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <type_traits>
#include <functional>
#include <array>

#include "filter_utils.h"
using namespace Teensy41FilterUtils;


namespace Q15Filter {

// utils
template <typename... Ts>
using are_integral = and_<std::is_integral<Ts>...>;

template <typename... Ts>
using are_all_q15 = std::is_same<std::common_type_t<Ts...>, q15_t>;


// FIR Function Impl
template <unsigned int NUM_TAPS, unsigned int X_LEN, unsigned int BLOCK_SIZE>
class FIRFilterQ15 {
private:
    // Note:
    // 1. The filter coefficients are stored in time reversed order. Furthremore, in cases where the actual number of taps does not satisfy the
    //    conditions in the static assert, pad the filter coefficients with zeros from behind (later in time, which means they appear earlier in
    //    this array) as required to make NUM_TAPS satisfy the condition.
    // 2. `fir_state` requires length of NUM_TAPS + BLOCK_SIZE required for M3 & M4 and less 1 for M0, but unspeficifed for M7 (our target arch),
    //    so use the larger of the two to be safe.
    const std::array<const q15_t, NUM_TAPS> _h = { 0 };
    std::array<q15_t, X_LEN> _in_buf = { 0 };
    std::array<q15_t, X_LEN> _out_buf = { 0 };
    std::array<q15_t, NUM_TAPS + BLOCK_SIZE> _fir_state = { 0 };
    arm_fir_instance_q15 _fir_inst;

public:
    // arm_fir_init_q15 requires that NUM_TAPS be at least 4 and even
    static_assert(NUM_TAPS >= 4, "NUM_TAPS is smaller than 4!");
    static_assert(NUM_TAPS % 2 == 0, "NUM_TAPS is not even!");
    static_assert(X_LEN % BLOCK_SIZE == 0, "X_LEN is not an integer multiple of BLOCK_SIZE!");

    // list-initialization (example: q15_t{0}) is used to prevent (or generate a warning for) narrowing conversions
    // template <typename... Ts, std::enable_if_t<are_all_q15<Ts...>::value, bool> = true>
    template <typename... Ts, std::enable_if_t<are_integral<Ts...>::value, bool> = true>
    FIRFilterQ15(Ts... ts) : _h{{q15_t{ts}...}}  {
        static_assert(sizeof...(Ts) <= NUM_TAPS, "The number of coefficients provided exceeds NUM_TAPS!");

        // in performing the cast, we assume that the function does not attempt to modify `_h` (which is labeled const)
        arm_fir_init_q15(&_fir_inst, NUM_TAPS, (q15_t*)_h.data(), _fir_state.data(), BLOCK_SIZE);
    }

    q15_t* in_buf() {
        return _in_buf.data();
    }

    q15_t* out_buf() {
        return _out_buf.data();
    }

    using inbuf_iter = typename decltype(_in_buf)::iterator;
    void fill_buf(std::function<void(inbuf_iter, inbuf_iter)> f) {
        f(_in_buf.begin(), _in_buf.end());
    }

    void filter() {
        q15_t* x = _in_buf.data();
        q15_t* y = _out_buf.data();
        for (unsigned int i = 0; i < X_LEN / BLOCK_SIZE; i++)
            arm_fir_fast_q15(&_fir_inst, x + i * BLOCK_SIZE, y + i * BLOCK_SIZE, BLOCK_SIZE);
    }
};


// Partial Convolution Impl
template <unsigned int NUM_TAPS, unsigned int X_LEN>
class PartialConvFilterQ15 {
private:
    const std::array<const q15_t, NUM_TAPS> _h = { 0 };
    std::array<q15_t, X_LEN+NUM_TAPS-1> _in_buf = { 0 };
    std::array<q15_t, X_LEN> _out_buf = { 0 };
    std::array<q15_t, NUM_TAPS-1> _overlap_buf = { 0 };

public:
    // list-initialization (example: q15_t{0}) is used to prevent (or generate a warning for) narrowing conversions
    // template <typename... Ts, std::enable_if_t<are_all_q15<Ts...>::value, bool> = true>
    template <typename... Ts, std::enable_if_t<are_integral<Ts...>::value, bool> = true>
    PartialConvFilterQ15(Ts... ts) : _h{{q15_t{ts}...}} {
        static_assert(sizeof...(Ts) <= NUM_TAPS, "The number of coefficients provided exceeds NUM_TAPS!");
    }

    q15_t* in_buf() {
        return _in_buf.data() + NUM_TAPS - 1;
    }

    q15_t* out_buf() {
        return _out_buf.data();
    }

    using inbuf_iter = typename decltype(_in_buf)::iterator;
    void fill_buf(std::function<void(inbuf_iter, inbuf_iter)> f) {
        // include overlap from last iteration
        memcpy(_in_buf.data(), _overlap_buf.data(), _overlap_buf.size() * sizeof(q15_t));

        f(_in_buf.begin() + NUM_TAPS - 1, _in_buf.end());

        // save overlap for next iteration
        memcpy(_overlap_buf.data(), _in_buf.data() + X_LEN, _overlap_buf.size() * sizeof(q15_t));
    }

    void filter() {
        // FIXME: works for now but there could be issues in creating the out-of-bounds pointer (_out_buf.data() - (NUM_TAPS - 1)), like
        // - the construction of the out-of-bounds pointer is undefined behaviour
        // - the internals of the conv function could change to dereference the pointer before adding the offset, accessing out-of-bounds memory
        //   (implementation as of 3 Dec 2022: https://github.com/ARM-software/CMSIS-DSP/blob/main/Source/FilteringFunctions/arm_conv_partial_fast_q15.c)
        // the only reason we resort to this method is to avoid making _out_buf larger than it needs to be
        constexpr auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;
        q15_t* x = _in_buf.data();
        q15_t* y = _out_buf.data() - (NUM_TAPS - 1);

        // in performing the cast, we assume that the function does not attempt to modify `_h` (which is labeled const)
        arm_conv_partial_fast_q15(x, RESULT_SIZE, (q15_t*)_h.data(), NUM_TAPS, y, NUM_TAPS-1, X_LEN);
    }
};

}

#endif
