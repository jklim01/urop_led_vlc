/*
    This code benchmarks and compares the different filtering implementations when run on the board. The filter
    coefficients and parameters related to it can be changed to try different combinations. `SPS` and `X_SPAN`
    can also be tweaked to adjust the size of each filtering batch.

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
#include <cstdlib>
#include <utility>

#define ARM_MATH_CM7
#include "arm_math.h"

#include "float_filter.h"
#include "q15_filter.h"
using namespace FloatFilter;
using namespace Q15Filter;

#define SPS 4


#if SPS == 4
constexpr auto SPAN = 8;
constexpr auto X_SPAN = 56;
constexpr auto MAX_AMPLITUDE_OUT = 1.1100;
constexpr auto MIN_AMPLITUDE_OUT = 0.1100;
constexpr auto MAX_AMPLITUDE_FILTER = 0.30408403;
#elif SPS == 5
constexpr auto SPAN = 6;
constexpr auto X_SPAN = 96;
constexpr auto MAX_AMPLITUDE_OUT = 1.0958;
constexpr auto MIN_AMPLITUDE_OUT = 0.0958;
constexpr auto MAX_AMPLITUDE_FILTER = 0.24259566;
#endif

constexpr auto Q15_NORMALIZATION_H = MAX_AMPLITUDE_FILTER / 32767 * 32768;
constexpr auto Q15_NORMALIZATION_IN = MAX_AMPLITUDE_OUT / Q15_NORMALIZATION_H;
constexpr auto ONE_Q15_IN = (q15_t)(1 / Q15_NORMALIZATION_IN * 32768 + 0.5);                // + 0.5 to perform rounding instead of truncation

constexpr auto WARMRUP_ITERS = 100;
constexpr auto NUM_ITERS = 6000;

constexpr auto BETA = 0.8;
constexpr auto GROUP_DELAY = (SPAN * SPS) / 2;
constexpr auto X_LEN = X_SPAN * SPS;
constexpr auto NUM_TAPS = SPAN * SPS + 1;
constexpr auto RESULT_SIZE = NUM_TAPS + X_LEN - 1;
static_assert(X_SPAN % 8 == 0, "X_SPAN is not a multiple of 8 (8 bits per byte)");
static_assert(NUM_ITERS >= 2, "NUM_ITERS needs to be at least 2 (to reduce DOF for variance calculation)");


#if SPS == 4
// FIR Filter
constexpr auto BLOCK_SIZE = X_SPAN;
static FIRFilter<NUM_TAPS, X_LEN, BLOCK_SIZE> rrc_filter_fir {
    0.0011911152f, 0.0011521153f, -0.0013481304f, -0.0023166222f, 0.0005853885f, 0.0029110302f, -0.0005043186f, -0.0056238181f, -0.0029108152f, 0.0061275159f,
    0.0048348911f, -0.0149774985f, -0.0273074739f, 0.0141413911f, 0.1234908583f, 0.2485123556f, 0.3040840316f, 0.2485123556f, 0.1234908583f, 0.0141413911f,
    -0.0273074739f, -0.0149774985f, 0.0048348911f, 0.0061275159f, -0.0029108152f, -0.0056238181f, -0.0005043186f, 0.0029110302f, 0.0005853885f, -0.0023166222f,
    -0.0013481304f, 0.0011521153f, 0.0011911152f
};

// Conv Filter
static PartialConvFilter<NUM_TAPS, X_LEN> rrc_filter_conv {
    0.0011911152f, 0.0011521153f, -0.0013481304f, -0.0023166222f, 0.0005853885f, 0.0029110302f, -0.0005043186f, -0.0056238181f, -0.0029108152f, 0.0061275159f,
    0.0048348911f, -0.0149774985f, -0.0273074739f, 0.0141413911f, 0.1234908583f, 0.2485123556f, 0.3040840316f, 0.2485123556f, 0.1234908583f, 0.0141413911f,
    -0.0273074739f, -0.0149774985f, 0.0048348911f, 0.0061275159f, -0.0029108152f, -0.0056238181f, -0.0005043186f, 0.0029110302f, 0.0005853885f, -0.0023166222f,
    -0.0013481304f, 0.0011521153f, 0.0011911152f
};

// FFT Filter
static FFTFilter<NUM_TAPS, X_LEN> rrc_filter_fft {
    0.0011911152f, 0.0011521153f, -0.0013481304f, -0.0023166222f, 0.0005853885f, 0.0029110302f, -0.0005043186f, -0.0056238181f, -0.0029108152f, 0.0061275159f,
    0.0048348911f, -0.0149774985f, -0.0273074739f, 0.0141413911f, 0.1234908583f, 0.2485123556f, 0.3040840316f, 0.2485123556f, 0.1234908583f, 0.0141413911f,
    -0.0273074739f, -0.0149774985f, 0.0048348911f, 0.0061275159f, -0.0029108152f, -0.0056238181f, -0.0005043186f, 0.0029110302f, 0.0005853885f, -0.0023166222f,
    -0.0013481304f, 0.0011521153f, 0.0011911152f
};

// FIR Filter Q15
constexpr auto BLOCK_SIZE_Q15 = X_SPAN;
static FIRFilterQ15<NUM_TAPS+1, X_LEN, BLOCK_SIZE_Q15> rrc_filter_fir_q15 {
    q15_t{0}, q15_t{128}, q15_t{124}, q15_t{-145}, q15_t{-250}, q15_t{63}, q15_t{314}, q15_t{-54}, q15_t{-606}, q15_t{-314}, q15_t{660}, q15_t{521},
    q15_t{-1614}, q15_t{-2943}, q15_t{1524}, q15_t{13307}, q15_t{26779}, q15_t{32767}, q15_t{26779}, q15_t{13307}, q15_t{1524}, q15_t{-2943}, q15_t{-1614}, q15_t{521},
    q15_t{660}, q15_t{-314}, q15_t{-606}, q15_t{-54}, q15_t{314}, q15_t{63}, q15_t{-250}, q15_t{-145}, q15_t{124}, q15_t{128}
};

// Conv Filter Q15
static PartialConvFilterQ15<NUM_TAPS, X_LEN> rrc_filter_conv_q15 {
    q15_t{128}, q15_t{124}, q15_t{-145}, q15_t{-250}, q15_t{63}, q15_t{314}, q15_t{-54}, q15_t{-606}, q15_t{-314}, q15_t{660}, q15_t{521}, q15_t{-1614},
    q15_t{-2943}, q15_t{1524}, q15_t{13307}, q15_t{26779}, q15_t{32767}, q15_t{26779}, q15_t{13307}, q15_t{1524}, q15_t{-2943}, q15_t{-1614}, q15_t{521}, q15_t{660},
    q15_t{-314}, q15_t{-606}, q15_t{-54}, q15_t{314}, q15_t{63}, q15_t{-250}, q15_t{-145}, q15_t{124}, q15_t{128}
};

#elif SPS == 5
// FIR Filter
constexpr auto BLOCK_SIZE = X_SPAN;
static FIRFilter<NUM_TAPS, X_LEN, BLOCK_SIZE> rrc_filter_fir {
	0.0004670180f, 0.0022571456f, 0.0012361104f, -0.0023284039f, -0.0047254354f, -0.0023222237f, 0.0036039325f, 0.0061512472f, -0.0011611384f, -0.0155574481f,
	-0.0217856709f, -0.0000293462f, 0.0586397806f, 0.1407563926f, 0.2135002100f, 0.2425956589f, 0.2135002100f, 0.1407563926f, 0.0586397806f, -0.0000293462f,
	-0.0217856709f, -0.0155574481f, -0.0011611384f, 0.0061512472f, 0.0036039325f, -0.0023222237f, -0.0047254354f, -0.0023284039f, 0.0012361104f, 0.0022571456f,
	0.0004670180f
};

// Conv Filter
static PartialConvFilter<NUM_TAPS, X_LEN> rrc_filter_conv {
	0.0004670180f, 0.0022571456f, 0.0012361104f, -0.0023284039f, -0.0047254354f, -0.0023222237f, 0.0036039325f, 0.0061512472f, -0.0011611384f, -0.0155574481f,
	-0.0217856709f, -0.0000293462f, 0.0586397806f, 0.1407563926f, 0.2135002100f, 0.2425956589f, 0.2135002100f, 0.1407563926f, 0.0586397806f, -0.0000293462f,
	-0.0217856709f, -0.0155574481f, -0.0011611384f, 0.0061512472f, 0.0036039325f, -0.0023222237f, -0.0047254354f, -0.0023284039f, 0.0012361104f, 0.0022571456f,
	0.0004670180f,
};

// FFT Filter
static FFTFilter<NUM_TAPS, X_LEN> rrc_filter_fft {
	0.0004670180f, 0.0022571456f, 0.0012361104f, -0.0023284039f, -0.0047254354f, -0.0023222237f, 0.0036039325f, 0.0061512472f, -0.0011611384f, -0.0155574481f,
	-0.0217856709f, -0.0000293462f, 0.0586397806f, 0.1407563926f, 0.2135002100f, 0.2425956589f, 0.2135002100f, 0.1407563926f, 0.0586397806f, -0.0000293462f,
	-0.0217856709f, -0.0155574481f, -0.0011611384f, 0.0061512472f, 0.0036039325f, -0.0023222237f, -0.0047254354f, -0.0023284039f, 0.0012361104f, 0.0022571456f,
	0.0004670180f,
};

// FIR Filter Q15
constexpr auto BLOCK_SIZE_Q15 = X_SPAN;
static FIRFilterQ15<NUM_TAPS+1, X_LEN, BLOCK_SIZE_Q15> rrc_filter_fir_q15 {
	q15_t{0}, q15_t{63}, q15_t{305}, q15_t{167}, q15_t{-314}, q15_t{-638}, q15_t{-314}, q15_t{487}, q15_t{831}, q15_t{-157}, q15_t{-2101}, q15_t{-2943},
	q15_t{-4}, q15_t{7920}, q15_t{19012}, q15_t{28837}, q15_t{32767}, q15_t{28837}, q15_t{19012}, q15_t{7920}, q15_t{-4}, q15_t{-2943}, q15_t{-2101}, q15_t{-157},
	q15_t{831}, q15_t{487}, q15_t{-314}, q15_t{-638}, q15_t{-314}, q15_t{167}, q15_t{305}, q15_t{63},
};

// Conv Filter Q15
static PartialConvFilterQ15<NUM_TAPS, X_LEN> rrc_filter_conv_q15 {
	q15_t{63}, q15_t{305}, q15_t{167}, q15_t{-314}, q15_t{-638}, q15_t{-314}, q15_t{487}, q15_t{831}, q15_t{-157}, q15_t{-2101}, q15_t{-2943}, q15_t{-4},
	q15_t{7920}, q15_t{19012}, q15_t{28837}, q15_t{32767}, q15_t{28837}, q15_t{19012}, q15_t{7920}, q15_t{-4}, q15_t{-2943}, q15_t{-2101}, q15_t{-157}, q15_t{831},
	q15_t{487}, q15_t{-314}, q15_t{-638}, q15_t{-314}, q15_t{167}, q15_t{305}, q15_t{63},
};

#endif

// FFT Filter (manual)
static FFTFilter<NUM_TAPS, X_LEN> rrc_filter_fft_manual;
#include "populate_RRC_filter_half.inc"


utils::TicToc benchmarker;

void write_array(void* buf_ptr, size_t bytes_per_elem, size_t num_elems, int elem_offset){
    byte* ptr = (byte*)buf_ptr;
    for (size_t i = 0; i < num_elems; i++) {
        Serial.write(ptr + i * elem_offset, bytes_per_elem);
    }
    Serial.send_now();
}


float x_float[X_SPAN] = { 0.0 };
q15_t x_q15[X_SPAN] = { 0 };
auto float_filler = [](auto it, auto end) {
    float* x_ptr = x_float;
    for (int i = 0; i < SPS; i++) {
        *it = *x_ptr;
        it++;
    }
    x_ptr++;
};
auto q15_filler = [](auto it, auto end) {
    q15_t* x_ptr = x_q15;
    for (int i = 0; i < SPS; i++) {
        *it = *x_ptr;
        it++;
    }
    x_ptr++;
};

void setup() {
    float* x_float_ptr = x_float;
    q15_t* x_q15_ptr = x_q15;
    for (int i = 0; i < X_SPAN / 8; i++) {
        uint8_t x_byte = rand();

        for (int j = 0; j < 8; j++) {
            *x_float_ptr = (x_byte & 0x80) ? 1.0 : 0.0;
            *x_q15_ptr = (x_byte & 0x80) ? ONE_Q15_IN : 0;

            x_float_ptr++;
            x_q15_ptr++;
            x_byte <<= 1;
        }
    }

    rrc_filter_fft_manual.fill_H(populate_RRC_filter_half);

    while(!Serial);
}

struct BenchmarkRes {
    double mean;
    double std;
    double min;
    double max;

    BenchmarkRes() : mean(0.0), std(0.0), min(0.0), max(0.0) {}
};

template <typename F, typename G>
BenchmarkRes benchmark_filter(F f, G g) {
    for (int i = 0; i < WARMRUP_ITERS; i++) {
        f.fill_buf(g);
        f.filter();
    }

    BenchmarkRes res;
    uint32_t max_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    for (int i = 0; i < NUM_ITERS; i++) {
        uint32_t cycles = benchmarker.timeit([&f, &g]() {
            f.fill_buf(g);
            f.filter();
        });

        double time_taken = (double)cycles / (F_CPU_ACTUAL / 1'000'000);
        res.mean += time_taken / NUM_ITERS;
        res.std += time_taken / (NUM_ITERS - 1) * time_taken;

        if (cycles > max_cycles)
            max_cycles = cycles;
        else if (cycles < min_cycles)
            min_cycles = cycles;
    }

    res.std -= pow(res.mean, 2) / (NUM_ITERS - 1) * NUM_ITERS;
    res.std = sqrt(res.std);
    res.min = (double)min_cycles / (F_CPU_ACTUAL / 1'000'000);
    res.max = (double)max_cycles / (F_CPU_ACTUAL / 1'000'000);
    return res;
}


void loop() {
    constexpr auto NUM_ITEMS = 6;
    constexpr auto MAX_LABEL_LEN = 40;
    char labels[NUM_ITEMS][MAX_LABEL_LEN] = {
        "FIR       (BlkSize=%d)",
        "Conv",
        "FFT       (FFT_SIZE=%d)",
        "FFT       (manual, FFT_SIZE=%d)",
        "FIR q15   (BlkSize=%d)",
        "Conv q15"
    };
    snprintf(labels[0], MAX_LABEL_LEN, "FIR       (BlkSize=%d)"         , BLOCK_SIZE);
    snprintf(labels[2], MAX_LABEL_LEN, "FFT       (FFT_SIZE=%d)"        , rrc_filter_fft.FFT_SIZE);
    snprintf(labels[3], MAX_LABEL_LEN, "FFT       (manual, FFT_SIZE=%d)", rrc_filter_fft_manual.FFT_SIZE);
    snprintf(labels[4], MAX_LABEL_LEN, "FIR q15   (BlkSize=%d)"         , BLOCK_SIZE_Q15);

    BenchmarkRes results[NUM_ITEMS];
    results[0] = benchmark_filter(rrc_filter_fir, float_filler);
    results[1] = benchmark_filter(rrc_filter_conv, float_filler);
    results[2] = benchmark_filter(rrc_filter_fft, float_filler);
    results[3] = benchmark_filter(rrc_filter_fft_manual, float_filler);
    results[4] = benchmark_filter(rrc_filter_fir_q15, q15_filler);
    results[5] = benchmark_filter(rrc_filter_conv_q15, q15_filler);


    Serial.printf("X_SPAN = %d, SPAN = %d, BETA = %.2f, SPS = %d\n", X_SPAN, SPAN, BETA, SPS);
    Serial.printf("RESULT_SIZE  =  %d  =  (X_LEN = %d)  +  (NUM_TAPS = %d)  -  1\n\n", RESULT_SIZE, X_LEN, NUM_TAPS);

    Serial.printf("At %d iterations,\n", NUM_ITERS);
    Serial.printf("%-*s  |  %-12s  |  %-12s  |  %-12s  |  %-12s  |  %-23s  |  %-23s\n",
        MAX_LABEL_LEN, "Item", "mean (us)", "std (us)", "min (us)", "max (us)", "mean per sample (ns)", "std per sample (ns)");

    constexpr auto BAR_LEN = MAX_LABEL_LEN + 4*12 + 2*23 + 6*5 + 2;         // ... + NUM_VERTICAL_BARS * VERTICAL_BAR_SPACE + LAST_2_SPACES
    char BAR[BAR_LEN + 1];
    memset(BAR, '-', BAR_LEN);
    BAR[BAR_LEN] = '\0';
    Serial.println(BAR);

    for (size_t i = 0; i < NUM_ITEMS; i++)
        Serial.printf("%-*s  |  %-12.4f  |  %-12.4f  |  %-12.4f  |  %-12.4f  |  %-23.2f  |  %-23.2f\n",
            MAX_LABEL_LEN,
            labels[i], results[i].mean, results[i].std, results[i].min, results[i].max,
            results[i].mean / X_LEN * 1000, results[i].std / X_LEN * 1000);


    utils::panic(0);
}
