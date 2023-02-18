/*
    This file attempts to find the parameters that will give the highest efficiency when filtering using FFT under the
    overlap-save scheme. This is done by computing the cost per feasible sample from the FFT operation, where an FFT
    operation of size `N` is estimated to have a cost of:
    `N * [log2(N) + 1]`

    Note:
    - set `xlen_must_be_multiple_of` to 1 if there are no restrictions on the input signal length per filter batch
    - the found parameters are not always the best in real-life

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

close all; clearvars; clc;

% parameters
span = 6;
sps = 5;
samples_per_batch = 16;
BITS_PER_BYTE = 8;
fft_sizes = [32, 64, 128, 256, 512, 1024, 2048, 4096];

num_taps = span * sps + 1;
xlen_must_be_multiple_of = lcm(lcm(samples_per_batch, sps), BITS_PER_BYTE);
% xlen_must_be_multiple_of = 1;

m = 1:1024;
n_opt = ones(1, length(m));
cost = zeros(1, length(m));


% calculate optimal values for FFT filtering with overlap-save
for i = 1:length(m)
    feasible_sizes = fft_sizes((fft_sizes - m(i) + 1) > xlen_must_be_multiple_of);
    costs = cost_per_sample(feasible_sizes, m(i), xlen_must_be_multiple_of);
    [min_cost, min_idx] = min(costs);

    n_opt(i) = feasible_sizes(min_idx);
    cost(i) = min_cost;
end


% plot trend
% figure;
% plot(m, n_opt);
% title("Optimal FFT Size over Num Taps");
%
% figure;
% plot(m, cost);
% title("Cost Per Sample over Num Taps");


% print result for desired NUM_TAPS
opt_xlen = @(M) floor((n_opt(M) - M + 1) / xlen_must_be_multiple_of) * xlen_must_be_multiple_of;
opt_xspan = @(M) opt_xlen(M) / sps;
opt_wasted_samples = @(M) n_opt(M) - (M + opt_xlen(M) - 1);

fprintf("At %d filter taps (xlen_must_be_multiple_of=%d, sps=%d),\n",...
    num_taps, xlen_must_be_multiple_of, sps);
fprintf("opt_xspan          = %d\n", opt_xspan(num_taps));
fprintf("opt_xlen           = %d\n", opt_xlen(num_taps));
fprintf("opt_wasted_samples = %d\n", opt_wasted_samples(num_taps));


%% functions
function x = cost_per_sample(n, M, xlen_must_be_multiple_of)
    num_feasible_samples = floor((n - M + 1) / xlen_must_be_multiple_of) * xlen_must_be_multiple_of;
    x = n .* (log2(n) + 1) ./ num_feasible_samples;
end
