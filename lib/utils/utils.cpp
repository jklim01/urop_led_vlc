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

#include "utils.h"
#include "DMAChannel.h"

namespace utils {

void panic(uint8_t panic_code) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5;               // set corresponding pad to GPIO mode
    IOMUXC_GPR_GPR27 |= 1 << 3;                         // select GPIO7 instead of GPIO2 for the corresponding pad
    pinMode(LED_BUILTIN, OUTPUT);

    while(1) {
        // fast blink for 3 seconds
        for (int i = 0; i < 2 * (1000 / 200); i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }

        // display panic code
        delay(1000);
        for (int i = 0; i < panic_code; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
}

void print_tcd(DMABaseClass const& tcd) {
    Serial.printf("TCD->SADDR    = %08p\n", tcd.TCD->SADDR);
    Serial.printf("TCD->DADDR    = %08p\n", tcd.TCD->DADDR);
    Serial.printf("TCD->SOFF     = %d\n", tcd.TCD->SOFF);
    Serial.printf("TCD->DOFF     = %d\n", tcd.TCD->DOFF);
    Serial.printf("TCD->ATTR_SRC = %d\n", tcd.TCD->ATTR_SRC);
    Serial.printf("TCD->ATTR_DST = %d\n", tcd.TCD->ATTR_DST);
    Serial.printf("TCD->SLAST    = %d\n", tcd.TCD->SLAST);
    Serial.printf("TCD->DLASTSGA = %d = %08p\n", tcd.TCD->DLASTSGA, tcd.TCD->DLASTSGA);
    Serial.printf("TCD->NBYTES_MLNO   = %d\n", tcd.TCD->NBYTES_MLNO);
    Serial.printf("TCD->BITER_ELINKNO = %d\n", tcd.TCD->BITER_ELINKNO);
    Serial.printf("TCD->CITER_ELINKNO = %d\n\n", tcd.TCD->CITER_ELINKNO);
}

TicToc::TicToc() {
    // enable cycle count register
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

uint32_t TicToc::timeit(std::function<void(void)> f) {
    uint32_t start = ARM_DWT_CYCCNT;
    f();
    uint32_t end = ARM_DWT_CYCCNT;

    return end - start;   // still works if only 1 counter overflow occurs
}

}