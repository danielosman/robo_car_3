// Host test for the shared helpers: stamp.h (time differences across the clock's
// wrap), geom.h (bearing, wrapping an angle) and stats.h (n-th value, median, sort).
// Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon -o build/test_helpers common/test/test_helpers.c -lm && build/test_helpers
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stamp.h"
#include "geom.h"
#include "stats.h"

static int compare(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

int main(void) {
    // stamp: across the wrap, both ways.
    assert(stamp_us(5, 0xFFFFFFFFu) == 6);
    assert(stamp_us(0xFFFFFFFFu, 5) == -6);
    assert(stamp_us(1000, 400) == 600);
    assert(fabsf(stamp_s(250000, 0u - 250000u) - 0.5f) < 1e-6f);

    // geom: the four axes; ±π and ±3π.
    assert(fabsf(bearing_rad((float[3]){1, 0, 0})) < 1e-6f);
    assert(fabsf(bearing_rad((float[3]){0, 1, 0}) - PI_F / 2) < 1e-6f);
    assert(fabsf(bearing_rad((float[3]){0, -1, 0}) + PI_F / 2) < 1e-6f);
    assert(fabsf(fabsf(bearing_rad((float[3]){-1, 0, 0})) - PI_F) < 1e-6f);
    assert(wrap_pi(PI_F) == PI_F && wrap_pi(-PI_F) == PI_F); // −π < a ≤ π
    // ±3π: the same angle as ±π; rounding may land on either side
    assert(fabsf(fabsf(wrap_pi(3 * PI_F)) - PI_F) < 1e-5f && fabsf(fabsf(wrap_pi(-3 * PI_F)) - PI_F) < 1e-5f);
    assert(fabsf(wrap_pi(0.5f + 4 * PI_F) - 0.5f) < 1e-5f && wrap_pi(-0.5f) == -0.5f);

    // stats: against a sorted copy, on random arrays of 1-300 values (with repeats).
    srand(1);
    for (int trial = 0; trial < 2000; trial++) {
        int n = 1 + rand() % 300;
        float v[300], sorted[300], work[300];
        for (int i = 0; i < n; i++) v[i] = (float)(rand() % (trial % 2 ? 20 : 100000));
        memcpy(sorted, v, sizeof v);
        qsort(sorted, (size_t)n, sizeof sorted[0], compare);
        int k = rand() % n;
        memcpy(work, v, sizeof v);
        assert(nth_value(work, n, k) == sorted[k]);
        for (int i = 0; i < n; i++) assert(i <= k ? work[i] <= sorted[k] : work[i] >= sorted[k]);
        memcpy(work, v, sizeof v);
        float median = n % 2 ? sorted[n / 2] : 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);
        assert(median_value(work, n) == median);
        memcpy(work, v, sizeof v);
        sort_values(work, n);
        assert(memcmp(work, sorted, (size_t)n * sizeof work[0]) == 0);
    }
    assert(median_value((float[1]){0}, 0) == 0.0f);
    printf("OK: helpers: stamp across the wrap, bearing, wrap_pi, nth_value, median, sort\n");
    return 0;
}
