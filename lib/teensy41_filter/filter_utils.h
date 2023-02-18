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
#ifndef FILTER_UTILS_H
#define FILTER_UTILS_H

namespace Teensy41FilterUtils {

template <typename... Conds>
struct and_ : std::true_type {};

template <typename Cond, typename... Conds>
struct and_<Cond, Conds...> : std::conditional_t<Cond::value, and_<Conds...>, std::false_type> {};

template <unsigned int IN>
constexpr unsigned int flog2_unchecked() {
    constexpr unsigned int RSHIFTED_N = IN >> 1;
    return (RSHIFTED_N == 0) ? 0 : (flog2_unchecked<RSHIFTED_N>() + 1);
}

template <unsigned int IN>
constexpr unsigned int flog2() {
    static_assert(IN != 0, "Input cannot be 0!");
    constexpr unsigned int RSHIFTED_N = IN >> 1;
    return (RSHIFTED_N == 0) ? 0 : (flog2_unchecked<RSHIFTED_N>() + 1);
}

template <unsigned int IN>
constexpr unsigned int clog2() {
    static_assert(IN != 0, "Input cannot be 0!");
    return (IN == 1) ? 0 : (flog2<IN-1>() + 1);
}

template <unsigned int IN>
constexpr unsigned int next_powof2() {
    return (IN == 0) ? 1 : (1 << clog2<IN>());
}

}

#endif