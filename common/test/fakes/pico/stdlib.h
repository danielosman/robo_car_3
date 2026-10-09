#pragma once
// Just enough of the Pico SDK for the app host tests (picoA/app, picoB/app):
// a clock the test advances by setting fake_now_us. It starts 30 s before
// time_us_32() wraps, as the robot's clock does (common/clock_start.h), so the tests cross the wrap: set it
// as FAKE_CLOCK_START_US + some time, not to a time of its own (a test shorter
// than 30 s starts nearer FAKE_CLOCK_WRAP_US).
#include <stdbool.h>
#include <stdint.h>
#define FAKE_CLOCK_WRAP_US  (1ull << 32)                  // time_us_32() is 0 again
#define FAKE_CLOCK_START_US (FAKE_CLOCK_WRAP_US - 30000000u)
typedef uint64_t absolute_time_t;
static uint64_t fake_now_us = FAKE_CLOCK_START_US;
static const absolute_time_t nil_time = 0;
static inline uint64_t time_us_64(void) { return fake_now_us; }
static inline uint32_t time_us_32(void) { return (uint32_t)fake_now_us; }
static inline absolute_time_t get_absolute_time(void) { return fake_now_us; }
static inline bool is_nil_time(absolute_time_t t) { return t == 0; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return fake_now_us + us; }
static inline absolute_time_t delayed_by_us(absolute_time_t t, uint64_t us) { return t + us; }
static inline bool time_reached(absolute_time_t t) { return fake_now_us >= t; }
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) { return (int64_t)(to - from); }
