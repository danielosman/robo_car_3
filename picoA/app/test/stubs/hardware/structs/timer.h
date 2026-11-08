#pragma once
// Fake timer registers: clock_start() writes them, nothing reads them (the fake
// clock already starts where clock_start() puts the robot's).
#include <stdint.h>
typedef struct { volatile uint32_t timelw, timehw; } fake_timer_hw_t;
static fake_timer_hw_t fake_timer_hw;
#define timer_hw (&fake_timer_hw)
