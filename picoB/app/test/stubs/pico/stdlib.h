#pragma once
// Just enough of the Pico SDK for picoB/app host tests: a clock the test advances.
#include <stdbool.h>
#include <stdint.h>
static uint64_t fake_now_us;
static inline uint64_t time_us_64(void) { return fake_now_us; }
