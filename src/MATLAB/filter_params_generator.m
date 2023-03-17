%{
    This file is responsible for computed the filter coefficients and some of its related constants, and generating
    a corresponding .inc file to be included in the vlc C++ code. The parent C++ file must define `VAR_DECL`, which
    is contains part of the declaration statement for a variable that accepts the filter coefficients as a braced-
    init-list. It can also optionally define `COEFF_NAMESPACE` to put everything in the specified namespace.

    Note: the user tweaks `beta`, `span` and `sps` as required


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
%}

close all; clearvars; clc;

beta = 0.8;
span = 8;
sps = 4;

h = rcosdesign(beta, span, sps);
h = h / sum(h);

q15_normalization_in = max(h) / 32767 * 32768;
h_q15 = round(h / q15_normalization_in * 32768);

max_amplitude_out = sum(h(h > 0));
min_amplitude_out = abs(sum(h(h < 0)));
max_amplitude_filter = max(h);

% print to stdout
fprintf("MAX_AMPLITUDE_OUT = %.4f\n", max_amplitude_out);
fprintf("MIN_AMPLITUDE_OUT = %.4f\n", min_amplitude_out);
fprintf("MAX_AMPLITUDE_FILTER = %.10f\n", max_amplitude_filter);

fprintf("\nfloat coefficients: { ");
fprintf("%.10ff, ", h);
fprintf("\b\b }\n");

fprintf("\nq15_t coefficients: { ");
% fprintf("%d, ", h_q15);
fprintf("q15_t{%d}, ", h_q15);
fprintf("\b\b }\n\n");


% user chooses whether to create corresponding header file
choices = ["y", "n"];
choices_str = strjoin(choices, "/");
write_file = input(sprintf("Write to header file? (%s) ", choices_str), "s");
while all(write_file ~= choices)
    write_file = input(sprintf("Invalid input! Write to header file? (%s) ", choices_str), "s");
end

if write_file == "y"
    choices = ["q15", "float"];
    choices_str = strjoin(choices, "/");
    mode = input(sprintf("What mode? (%s) ", choices_str), "s");
    while all(mode ~= choices)
        mode = input(sprintf("Invalid input! What mode? (%) ", choices_str), "s");
    end

    filename = "rrc_" + mode + ".inc";
    fid = fopen(filename, "w");
    if fid == 0
        error("Failed to open ""%s"" for writing!", filename);
    end
    fprintf(fid, "/*\n\tThis file was auto-generated by ""filter_params_generator_nicer.m"".\n");
    fprintf(fid, "\tIt contains the filter coefficients for an rrc filter (parameters listed below).\n*/\n\n");
    fprintf(fid, "#ifndef VAR_DECL\n");
    fprintf(fid, "#error ""The declaration of the variable (VAR_DECL) receiving the filter coefficients must be specified!""\n");
    fprintf(fid, "#endif\n\n");

    fprintf(fid, "#ifdef COEFF_NAMESPACE\n");
    fprintf(fid, "namespace COEFF_NAMESPACE {\n");
    fprintf(fid, "#endif\n\n");

    if mode == choices(1)
        fprintf(fid, "#define ARM_MATH_CM7\n");
        fprintf(fid, "#include ""arm_math.h""\n\n");
        fprintf(fid, "constexpr auto MAX_AMPLITUDE_FILTER = %.8f;\n", max_amplitude_filter);
    end
    fprintf(fid, "constexpr auto BETA = %.2f;\n", beta);
    fprintf(fid, "constexpr auto SPAN = %d;\n", span);
    fprintf(fid, "constexpr auto SPS = %d;\n", sps);
    fprintf(fid, "constexpr auto NUM_TAPS = SPAN * SPS + 1;\n\n");
    fprintf(fid, "// approximations found by `sum(h(h > 0))` and `abs(sum(h(h < 0)))`\n");
    fprintf(fid, "constexpr auto MAX_AMPLITUDE_OUT = %.4f;\n", max_amplitude_out);
    fprintf(fid, "constexpr auto MIN_AMPLITUDE_OUT = %.4f;\n\n", min_amplitude_out);


    switch mode
        case choices(1)
            fprintf(fid, "#ifndef PADDED\n");
            fprintf(fid, "VAR_DECL {");
            for i = 1:length(h_q15)
                if mod(i-1, 12) == 0
                    fprintf(fid, "\n\t");
                end

                % fprintf(fid, "%d, ", h_q15(i));
                fprintf(fid, "q15_t{%d}, ", h_q15(i));
            end
            fprintf(fid, "\n};\n");
            fprintf(fid, "#else\n");
            fprintf(fid, "#undef PADDED\n");

            h_q15 = [0, h_q15];
            fprintf(fid, "VAR_DECL {");
            for i = 1:length(h_q15)
                if mod(i-1, 12) == 0
                    fprintf(fid, "\n\t");
                end

                % fprintf(fid, "%d, ", h_q15(i));
                fprintf(fid, "q15_t{%d}, ", h_q15(i));
            end
            fprintf(fid, "\n};\n");
            fprintf(fid, "#endif\n\n");

        case choices(2)
            fprintf(fid, "VAR_DECL {");
            for i = 1:length(h)
                if mod(i-1, 10) == 0
                    fprintf(fid, "\n\t");
                end

                fprintf(fid, "%.10ff, ", h(i));
            end
            fprintf(fid, "\n};\n\n");
    end

    fprintf(fid, "#ifdef COEFF_NAMESPACE\n");
    fprintf(fid, "}\n");
    fprintf(fid, "#undef COEFF_NAMESPACE\n");
    fprintf(fid, "#endif\n");
    fclose(fid);
end

