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
#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"
#include "functional"
#include "DMAChannel.h"

namespace utils {

void panic(uint8_t panic_code);

void print_tcd(DMABaseClass const& setting);

class TicToc {
public:
    TicToc();

    // This method assumes that the 32-bit ARM_DWT_CYCCNT counter only overflows once during the idle period.
    // Thus, `f` must take at most (UINT32_MAX / F_CPU_ACTUAL). At 600MHz (default), this is around 7.158s, which
    // is easily more than enough for the use in short precise measurements.
    uint32_t timeit(std::function<void(void)> f);
};


}


#endif