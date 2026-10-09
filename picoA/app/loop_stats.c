#include <stdbool.h>
#include <stdio.h>
#include "loop_stats.h"

static loop_window_t current, last;
static bool started, have_last;
static uint32_t window_start_us, iteration_start_us, stage_start_us;

void loop_stats_begin(uint32_t now_us) {
    if (!started) {
        started = true;
        window_start_us = now_us;
    } else {
        uint32_t us = now_us - iteration_start_us;
        current.iterations++;
        current.total_us += us;
        if (us > current.max_us) current.max_us = us;
    }
    if (now_us - window_start_us >= LOOP_STATS_WINDOW_US) {
        current.span_us = now_us - window_start_us;
        last = current;
        have_last = true;
        current = (loop_window_t){0};
        window_start_us = now_us;
    }
    iteration_start_us = stage_start_us = now_us;
}

void loop_stats_stage(const char *stage, uint32_t now_us) {
    uint32_t us = now_us - stage_start_us;
    if (!current.slowest || us > current.slowest_us) {
        current.slowest = stage;
        current.slowest_us = us;
    }
    stage_start_us = now_us;
}

const loop_window_t *loop_stats_window(void) {
    if (have_last) return &last;
    current.span_us = iteration_start_us - window_start_us;
    return &current;
}

void loop_stats_print(void) {
    const loop_window_t *w = loop_stats_window();
    if (!w->iterations || !w->span_us) { printf("Loop: not measured yet\n"); return; }
    printf("Loop: %lu per s, mean %lu us, max %lu us (slowest: %s %lu us), over %.1f s\n",
           (unsigned long)((uint64_t)w->iterations * 1000000u / w->span_us),
           (unsigned long)(w->total_us / w->iterations), (unsigned long)w->max_us,
           w->slowest ? w->slowest : "-", (unsigned long)w->slowest_us, (double)((float)w->span_us * 1e-6f));
}
