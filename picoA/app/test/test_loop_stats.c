// Host test for picoA/app/loop_stats.c: the mean and the longest iteration over a
// window, the window rolling over, the slowest stage named, across the clock's wrap.
// Run from the repo root:
//   cc -std=c11 -Wall -Wextra -o build/test_loop_stats picoA/app/test/test_loop_stats.c && build/test_loop_stats
#include <assert.h>
#include <string.h>
#include "../loop_stats.c"

static uint32_t now_us = 0u - 5000000u; // 5 s before the clock wraps: the first window spans it

// One iteration: body takes body_us, map map_us.
static void iteration(uint32_t body_us, uint32_t map_us) {
    loop_stats_begin(now_us);
    now_us += body_us;
    loop_stats_stage("body", now_us);
    now_us += map_us;
    loop_stats_stage("map", now_us);
}

int main(void) {
    loop_stats_print(); // nothing yet
    assert(loop_stats_window()->iterations == 0);

    // The first window, still running: 1000 iterations of 1 ms, one of 300 ms in the map.
    for (int i = 0; i < 1000; i++) iteration(400, 600);
    iteration(400, 300000);
    for (int i = 0; i < 10; i++) iteration(400, 600);
    const loop_window_t *w = loop_stats_window();
    assert(w->iterations == 1010); // the last one ends with the next begin
    assert(w->max_us == 300400);
    assert(w->total_us == 1009 * 1000ull + 300400);
    assert(strcmp(w->slowest, "map") == 0 && w->slowest_us == 300000);
    loop_stats_print();

    // Past 10 s: the window rolls over, the 300 ms iteration stays in the last full one.
    while (now_us - (0u - 5000000u) < LOOP_STATS_WINDOW_US) iteration(400, 600);
    iteration(400, 600);
    w = loop_stats_window();
    assert(w->max_us == 300400 && strcmp(w->slowest, "map") == 0);
    assert(w->span_us >= LOOP_STATS_WINDOW_US && w->span_us < LOOP_STATS_WINDOW_US + 2000);
    uint32_t per_s = (uint32_t)((uint64_t)w->iterations * 1000000u / w->span_us);
    assert(per_s > 960 && per_s < 975); // 9.7 s of 1 ms iterations and one of 300 ms in 10 s

    // The next window (full after 23 s more): no long iteration, the body slowest.
    for (int i = 0; i < 9000; i++) iteration(2000, 600);
    w = loop_stats_window();
    assert(w->max_us == 2600 && strcmp(w->slowest, "body") == 0 && w->slowest_us == 2000);
    assert(w->total_us / w->iterations == 2600);
    loop_stats_print();
    printf("OK: loop stats: mean, max, slowest stage, the window rolling over, across the wrap\n");
    return 0;
}
