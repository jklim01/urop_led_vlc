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
#ifndef COMPLEX32_H
#define COMPLEX32_H

#include <stdint.h>
#include <cstddef>
#include <type_traits>

namespace Complex {

class Complex32 {
public:
    float re;
    float im;

    Complex32() : re { 0 }, im { 0 } {}
    Complex32(float re, float im) : re { re }, im { im } {}

    Complex32 operator+(const Complex32& other) const {
        return Complex32(this->re + other.re, this->im + other.im);
    }

    Complex32 operator-(const Complex32& other) const {
        return Complex32(this->re - other.re, this->im - other.im);
    }

    Complex32 operator*(const Complex32& other) const {
        return Complex32(this->re * other.re - this->im * other.im, this->re * other.im + this->im * other.re);
    }

    Complex32& operator*=(const Complex32& other) {
        float old_re = this->re;
        float old_im = this->im;

        this->re = old_re * other.re - old_im * other.im;
        this->im = old_re * other.im + old_im * other.re;
        return *this;
    }

    Complex32 operator*(const float& factor) const {
        return Complex32(this->re * factor, this->im * factor);
    }

    Complex32 operator/(const float& denominator) const {
        return Complex32(this->re / denominator, this->im / denominator);
    }
};
static_assert(
    std::is_standard_layout<Complex32>::value,
    "Complex32 isn't standard layout!"
);
static_assert(offsetof(Complex32, re) == 0, "Complex32::re is not at offset 0!");
static_assert(offsetof(Complex32, im) == 4, "Complex32::im is not at offset 4!");
static_assert(sizeof(Complex32) == 8, "Complex32 doesn't have a size of 8!");

}

#endif