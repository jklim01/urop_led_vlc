/*
    This code was used to investigate the problem of `uadd16` compiling to incorrect assembly when the output value
    is not stored in an intermediate variable. Build this code in release mode with debug symbols enabled to make
    the problem appear. The assembly can be viewed by running
        `objdump path/to/firmware.elf -D -S -g -l -M reg-names-std > path/to/output/file`.
    Run the code on the board to see which of the cases give the correct result.

    Note: the print statements can be removed to make the assembly easier to read


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
#include "arm_math.h"
#include <cstdlib>

#if defined (__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
static inline uint32_t __UADD16(uint32_t a, uint32_t b) __attribute__((always_inline));
static inline uint32_t __UADD16(uint32_t a, uint32_t b) {
	uint32_t out;
	asm volatile("uadd16 %0, %1, %2" : "=r" (out) : "r" (a), "r" (b));
	return(out);
}
#endif

void print_array(uint16_t* ptr, size_t len) {
    Serial.printf("{ ");
    for (size_t i = 0; i < len; i++)
        Serial.printf("0x%04X, ", ptr[i]);
    Serial.printf("\b\b }\n");
}

void setup() {
    // while (!Serial);

    // correct result for 16-bit additions
    uint16_t a[8] = { 0 };
    uint16_t b[8] = { 0 };
    uint16_t res_0[8] = { 0 };
    for (int i = 0; i < 8; i++) {
        a[i] = rand();
        b[i] = rand();
        res_0[i] = a[i] + b[i];
    }
    // Serial.printf("  a   = ");
    // print_array(a, sizeof(a) / sizeof(*a));
    // Serial.printf("  b   = ");
    // print_array(b, sizeof(b) / sizeof(*b));
    Serial.printf("res_0 = ");
    print_array(res_0, sizeof(res_0) / sizeof(*res_0));


    // use `uadd16` with the output stored in an intermediate variable `res`
    uint16_t res_1[8] = { 0 };
    uint32_t* a_ptr = (uint32_t*)a;
    uint32_t* b_ptr = (uint32_t*)b;
    uint32_t* res_ptr = (uint32_t*)res_1;
    for (int i = 0; i < 4; i++) {
        uint32_t res = __UADD16(*b_ptr, *a_ptr);
        uint32_t res1 = (uint32_t)*(uint16_t*)&res;
        uint32_t res2 = (uint32_t)*((uint16_t*)&res + 1);
        *res_ptr = (res1 & 0x0000FFFF) | ((res2 << 16) & 0xFFFF0000);

        a_ptr++;
        b_ptr++;
        res_ptr++;
    }
    // Serial.printf("res_1 = ");
    // print_array(res_1, sizeof(res_1) / sizeof(*res_1));


    // use `uadd16` but store result in the array `res_2`
    uint16_t res_2[8] = { 0 };
    a_ptr = (uint32_t*)a;
    b_ptr = (uint32_t*)b;
    res_ptr = (uint32_t*)res_2;
    for (int i = 0; i < 4; i++) {
        *res_ptr = __UADD16(*b_ptr, *a_ptr);
        uint32_t res1 = (uint32_t)*(uint16_t*)res_ptr;
        uint32_t res2 = (uint32_t)*((uint16_t*)res_ptr + 1);
        *res_ptr = (res1 & 0x0000FFFF) | ((res2 << 16) & 0xFFFF0000);

        a_ptr++;
        b_ptr++;
        res_ptr++;
    }
    // Serial.printf("res_2 = ");
    // print_array(res_2, sizeof(res_2) / sizeof(*res_2));


    // use `uadd16` but store the result back into one of the input variables
    a_ptr = (uint32_t*)a;
    b_ptr = (uint32_t*)b;
    for (int i = 0; i < 4; i++) {
        *a_ptr = __UADD16(*b_ptr, *a_ptr);
        uint32_t res1 = (uint32_t)*(uint16_t*)a_ptr;
        uint32_t res2 = (uint32_t)*((uint16_t*)a_ptr + 1);
        *a_ptr = (res1 & 0x0000FFFF) | ((res2 << 16) & 0xFFFF0000);

        a_ptr++;
        b_ptr++;
    }
    // Serial.printf("lolol = ");
    // print_array(a, sizeof(a) / sizeof(*a));
}

void loop() {

}