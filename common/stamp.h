#pragma once
// Times on a Pico's 32-bit microsecond clock (time_us_32()) and their differences.
// The clock wraps every 71.6 min; a difference stays right across the wrap as long
// as the two times are less than 35.8 min apart.
#include <stdint.h>

// later − earlier in µs (negative when "later" is earlier).
static inline int32_t stamp_us(uint32_t later_us, uint32_t earlier_us) { return (int32_t)(later_us - earlier_us); }

// later − earlier in seconds.
static inline float stamp_s(uint32_t later_us, uint32_t earlier_us) {
    return (float)stamp_us(later_us, earlier_us) * 1e-6f;
}
