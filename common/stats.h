#pragma once
// Order statistics of a handful of floats (tens to a few hundred): no allocation.
#include <stdlib.h>

static inline int stats_compare(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

// Sorts v[0..n) ascending.
static inline void sort_values(float *v, int n) { qsort(v, (size_t)n, sizeof v[0], stats_compare); }

// The k-th smallest of v[0..n) (k = 0: the smallest). Reorders v: afterwards v[k]
// is that value, nothing before it is bigger and nothing after it smaller.
static inline float nth_value(float *v, int n, int k) {
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        float pivot = v[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (v[i] < pivot) i++;
            while (v[j] > pivot) j--;
            if (i <= j) { float t = v[i]; v[i] = v[j]; v[j] = t; i++; j--; }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else break;
    }
    return v[k];
}

// The middle value (the mean of the two middle ones for an even n; 0 for none).
// Reorders v.
static inline float median_value(float *v, int n) {
    if (n < 1) return 0.0f;
    float upper = nth_value(v, n, n / 2);
    if (n % 2) return upper;
    float lower = v[0]; // the biggest of v[0..n/2), all ≤ upper after nth_value
    for (int i = 1; i < n / 2; i++) if (v[i] > lower) lower = v[i];
    return 0.5f * (lower + upper);
}
