#pragma once
// Both Picos start their microsecond clock 30 s before time_us_32() wraps (every
// 71.6 min), so every run crosses the wrap half a minute after power-up instead of
// after 71.6 min: a wrap bug shows in any robot test. The host tests' fake clock
// starts at the same time. clock_start() goes first in main(), before anything
// reads the clock or sets an alarm.
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/structs/timer.h"

#define CLOCK_START_US ((1ull << 32) - 30000000u)

static inline void clock_start(void) {
    timer_hw->timelw = (uint32_t)CLOCK_START_US; // taken over when timehw is written
    timer_hw->timehw = (uint32_t)(CLOCK_START_US >> 32);
}

// Time since power-up, for printing.
static inline uint64_t clock_since_start_us(void) { return time_us_64() - CLOCK_START_US; }
