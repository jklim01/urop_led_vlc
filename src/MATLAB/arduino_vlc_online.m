/*
    Run this file while a board that has been uploaded with the vlc code ("*_vlc.cpp") and with `ONLINE` defined is
    connected to the computer. This code is responsible for sending a stream of bits to the board to be processed and
    transmitted. Any errors during transmission and the board's idle time statistics, which are sent by the board, are
    shown in the MATLAB command window

    Note:
    This code is not very reliable since the computer can buffer its output ports, which will cause the board to receive
    the bits late and become unable to generate the processed samples on time.

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
bytes_per_transfer = 384;
% s = serial(port, "OutputBufferSize", bytes_per_transfer);
s = serial(port);
fopen(s);
fprintf('Opened Serial Port "%s"!\n\n', port);
fprintf("Teensy: %s\n", fscanf(s));


% locally calculated values
x_span = fread(s, 1, "int32");
num_iters = fread(s, 1, "int32");

bytes_per_iter = x_span / 8;
total_bytes = num_iters * bytes_per_iter;
num_transfers = total_bytes / bytes_per_transfer;
iters_per_transfer = num_iters / num_transfers;
byte_sequence = uint8(randi([0, 255], num_transfers, bytes_per_transfer));


for i = 1:num_transfers
    fwrite(s, byte_sequence(i, :), "uint8");
    flushoutput(s);
end
fprintf("Completed transfers!\n");
str = fscanf(s);
while str ~= sprintf("0\n")
    fprintf("Teensy: %s\n", str);
    str = fscanf(s);
end

% display idle time statistics
fprintf("Teensy: %s\n", fscanf(s));
str = fscanf(s);
while str ~= sprintf("0\n")
    fprintf("%s", str);
    str = fscanf(s);
end


% close port
fclose(s);
delete(s);
clear s;
fprintf("\nClosed Serial Port %s!\n", port);

