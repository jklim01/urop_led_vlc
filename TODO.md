## TODO:
- check if there exists better ways of writing to SHIFTBUF using the provided registers
- think of some way to prevent taking a long time to refill the storage buf or prove that we don't need to fix it yet
- document all code (including improved comments)
- test code after removing static inline function wrapping inline assembly calling `uadd16`

- check (using oscilloscope) vlc code
- increase sampling frequency to see how interrupt and dma impls react
- benchmarking (filtering + vlc code) -> calculate theoretical highest bitrate (3 std case?) + record memory usage
- tidy up files and post onto repo



## Possible Changes / Improvements:
- use `std::span` (C++20)
    - return `std::span` instead of a pointer for `in_buf` and `out_buf` functions to enforce range
    - let the lambda in `fill_buf` take `std::span` instead of a pair of begin & end iterators
        - cleaner
        - the number of elements is directly encoded within the type, which makes the code more elegant as begin & end could involve many elements but the lambda code already knows how many elements it needs to initialize
