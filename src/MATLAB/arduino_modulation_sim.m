/*
    Run this file while a board that has been uploaded with "modulation_sim.cpp" is connected to the computer. This
    code is responsible for sending a sequence of bits to the board for processing and accepting the samples for the
    final modulated and filtered signal, which is plotted out. The code also simulates a VLC system by adding AWGN
    noise and acting as the receiver that performs receive-filtering and therholding to recover the bits. The noisy
    and receive-filtered signals are plotted for viewing; while the BER, numerical error and processing time statistics
    are also displayed.

    Note:
    If the total number of samples involved are large, it is not recommended to run the last section, which involves
    plotting, as it takes a very long time.

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

% open port
port = "COM3";
s = serial(port, "InputBufferSize", 1024);
fopen(s);
fprintf('Opened Serial Port "%s"!\n', port);
str = fscanf(s);
fprintf("Teensy: %s", str);

is_fixed_point = false;
if contains(str, "q15")
    is_fixed_point = true;
end

is_fft = false;
if contains (str, "FFT")
    is_fft = true;
end


% read parameters
num_iters = fread(s, 1, "int32");
x_span = fread(s, 1, "int32");
span = fread(s, 1, "int32");
sps = fread(s, 1, "int32");
beta = fread(s, 1, "single");
max_amplitude = fread(s, 1, "single");
min_amplitude = fread(s, 1, "single");
fprintf("[%d iterations] BETA = %.2f, SPAN = %d, SPS = %d, X_SPAN = %d\n",...
    num_iters, beta, span, sps, x_span);

group_delay = span * sps / 2;
min_amplitude_q15 = int16(min_amplitude / max_amplitude * 2^15);

bytes_per_iter = x_span / 8;
samples_per_iter = x_span * sps;
byte_sequence = uint8(randi([0, 255], 1, num_iters * bytes_per_iter));


% send bytes and receive modulated & filtered signal
% % send bytes all in one go
% fwrite(s, byte_sequence, "uint8");
% flushoutput(s);
y = uint16(zeros(1, length(byte_sequence) * 8 * sps + 2 * group_delay));
for iter = 1:num_iters
    % send bytes
    fwrite(s, byte_sequence((1+(iter-1)*bytes_per_iter):(iter*bytes_per_iter)),...
        "uint8");

    fprintf("\nSuccessfully sent signal!\n");
    fprintf("Arduino: %s", fscanf(s));
    fprintf("Arduino: %s", fscanf(s));
    fprintf("Arduino: %s", fscanf(s));

    % read signal
    element_datatype = "uint16";
    samples_per_iter = x_span * sps;
    y(((iter-1)*samples_per_iter+1):(iter*samples_per_iter)) = fread(s, samples_per_iter, element_datatype);
end

% read delayed samples of final iteration
% ! last span*sps/2 entries of y are zero to allow for delay in rx filtering
y((end - 2*group_delay + 1):end) = fread(s, 2 * group_delay, "uint16");


% convert binary code representation to double
if is_fixed_point
    % convert to q15_t
    y = double(bitshift(y, 6)) / 65535 * (double(min_amplitude_q15) + 32767);
    y = y - double(min_amplitude_q15);

    % convert to corresponding double value
    y = y / 2^15 * max_amplitude;
else
    y = double(bitshift(y, 6)) / (2^16 - 1) * (min_amplitude + max_amplitude) - min_amplitude;
end


% display total processing times of each stage
times_mean = fread(s, 3, "double");
times_std = fread(s, 3, "double");
times_mean = times_mean.';
times_std = times_std.' - times_mean.^2 / (num_iters-1) * num_iters; % reduce DOF by 1
times_std = sqrt(times_std);

total_time_avg = sum(times_mean);
total_time_3std = total_time_avg + 3*sum(times_std);
stages = ["Rx & Fill Buffer", "Filtering", "Map to Binary Code"];
fprintf("\nAverage Processing Times:\n");
fprintf("%-20s -> %5.2f us (std = %5.4f us)\n", [stages; times_mean; times_std]);
fprintf("%-20s -> %.2f us\n", "1 Iteration avg", total_time_avg);
fprintf("%-20s -> %.2f us\n", "1 Iteration 3std", total_time_3std);

fprintf("\n! Theoretical maximum if every iteration is:\n")
fprintf("\t- exactly the same as the average case -> %.2f bps\n",...
    x_span * 1e6 / total_time_avg);
fprintf("\t- excatly the same as the 3std case    -> %.2f bps\n",...
    x_span * 1e6 / total_time_3std);


% close port
fclose(s);
delete(s);
clear s;
fprintf("\nClosed Serial Port %s!\n", port);


% calculate error compared with double calculation
bit_sequence = flip(de2bi(byte_sequence), 2).';         % flip to get MSB first
x = zeros(1, numel(bit_sequence) * sps);
for i = 1:numel(bit_sequence)
    x(((i-1)*sps+1):(i*sps)) = bit_sequence(i);
end

if is_fft
    fft_size = 2^(ceil(log2((x_span+span)*sps)));
    H = get_rrcfilter_freqresp(beta, span, sps, fft_size);
    h = ifft(ifftshift(H));
    y_tx_true = fftfilt(h, [x, zeros(1, 2*group_delay)]);
else
    h = rcosdesign(beta, span, sps, "sqrt");
    h = h / sum(h);
    y_tx_true = filter(h, 1, [x, zeros(1, 2*group_delay)]);
end
rmse = sqrt(mean((y - y_tx_true).^2));
fprintf("RMSE of calculation: %.4f\n", rmse);


%% Simulation of channel & receiver + BER calculation

% add noise and apply receiver filter
snr = 15;
y_noised = awgn(y, snr);
y_final = filter(h, 1, [y_noised, zeros(1, 2*group_delay)]);
y_noised = y_noised((1 + group_delay):end);
y_final = y_final((1 + 2*group_delay):(1 + 2*group_delay + length(x)));


% demodulate signal
decoded_bit_sequence = y_final(round(sps/2):sps:end);
decoded_bit_sequence(decoded_bit_sequence < 0.5) = 0;
decoded_bit_sequence(decoded_bit_sequence >= 0.5) = 1;
decoded_bit_sequence = reshape(decoded_bit_sequence, size(bit_sequence));
ber = sum(decoded_bit_sequence ~= bit_sequence, 'all') / numel(bit_sequence);
fprintf("At SNR = %.2f dB, the BER is %.4e!\n", snr, ber);


%% Plots

% plot of pulse train before and after tx filter
hold on;
plot(1:length(x), x);
plot(1:(length(y)-group_delay), y((1+group_delay):end), 'LineWidth', 0.8);
for i = 1:(x_span * num_iters - 1)
    xline(i*sps + 0.5, 'k--');
end
hold off;
ylim([-0.3, 1.5]);
title("Plot of Signals");
legend("Original Signal", "TX Filtered");


% plot of transmitted signal before and after passing through, and after rx filter
figure;
hold on;
plot(1:length(x), x);
plot(1:length(y_noised), y_noised, 'm');
plot(1:length(y_final), y_final, 'r', 'LineWidth', 0.8);
for i = 1:(x_span * num_iters - 1)
    xline(i*sps + 0.5, 'k--');
end
hold off;
ylim([-0.3, 1.5]);
title("Plot of Signals");
legend("Original Signal", "Noised", "RX Filtered");


%% function

function H = get_rrcfilter_freqresp(beta, span, sps, N)

if mod(span*sps, 2) ~= 0
    error("product of `span` and `sps` must be even!");
end
if span * sps > N
    error("`span` * `sps` cannot be smaller than `N`!");
end

H = zeros(1, N);
f = (1/N) * ((-floor(N/2)):(N - 1 - floor(N/2)));

% passband
pass_indices = abs(f) <= (1 - beta) / (2 * sps);
H(pass_indices) = 1;

% transition band
transition_indices = ~pass_indices & (abs(f) <= (1 + beta) / (2 * sps));
H(transition_indices) = sqrt((1 + cos(pi * sps * (abs(f(transition_indices)) - (1-beta)/(2*sps)) / beta)) / 2);

% out of band
out_indices = ~pass_indices & ~transition_indices;
H(out_indices) = 0;

group_delay = span * sps / 2;
H = H .* exp(-1j * (2*pi*f) * group_delay);

end

