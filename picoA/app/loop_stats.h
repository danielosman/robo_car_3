#pragma once
// How long the main loop takes (ARCHITECTURE_PLAN.md A1). main marks the start of
// every iteration and the end of every stage in it. Per 10 s window: the
// iterations, their mean and longest time, and the stage that took longest once
// (the one to look at when the longest iteration is too long).
#include <stdint.h>

#define LOOP_STATS_WINDOW_US 10000000u

typedef struct {
    uint32_t iterations;
    uint32_t span_us;    // how long the window ran
    uint64_t total_us;   // the iterations' times added up
    uint32_t max_us;     // the longest iteration
    const char *slowest; // the stage that took longest once; 0 before any stage
    uint32_t slowest_us;
} loop_window_t;

void loop_stats_begin(uint32_t now_us);                    // an iteration starts (and the one before ends)
void loop_stats_stage(const char *stage, uint32_t now_us); // a stage ended; it began where the one before ended
// The last full window; before the first is full, the one running.
const loop_window_t *loop_stats_window(void);
// "Loop: N per s, mean X us, max Y us (slowest: <stage> Z us)".
void loop_stats_print(void);
