#include <math.h>
#include "units.h"
#include "geometry.h"
#include "stamp.h"
#include "tracker.h"

#define REACH_RAD    (10 * RAD_PER_DEG) // a blob reaching this close to where the target is expected can be it...
#define RANGE_M      0.5f               // ...unless it is this much nearer or farther
#define LOST_US      500000             // no blob near it for this long: lost
#define EDGE_RAD     (16 * RAD_PER_DEG) // beyond this: the VL53's outer zone columns (its view is ±22.5°)
#define HISTORY_US   750000             // directions kept for the angular speed: the last 0.75 s...
#define HISTORY      16                 // ...at most this many (0.75 s at 15 Hz is 12)
#define RATE_MIN_US  150000             // the angular speed needs at least this much history
#define LEAVING_RADPS (10 * RAD_PER_DEG) // at the edge, moving outward at least this fast: leaving

static target_t target;
static bool active, lost, leaving_told;
static float history_rad[HISTORY];
static uint32_t history_us[HISTORY];
static int history_n, history_next;

void tracker_reset(void) { active = lost = false; }
const target_t *tracker_target(void) { return active || lost ? &target : 0; }

// The angular speed: the slope of a straight line through the directions of the
// last 0.75 s (least squares: the jitter of single frames averages out).
static float rate(void) {
    int oldest = (history_next - history_n + HISTORY) % HISTORY;
    int newest = (history_next - 1 + HISTORY) % HISTORY;
    if (stamp_us(history_us[newest], history_us[oldest]) < RATE_MIN_US) return 0.0f;
    float st = 0.0f, sb = 0.0f, stt = 0.0f, stb = 0.0f;
    for (int k = 0; k < history_n; k++) {
        int i = (oldest + k) % HISTORY;
        float t = stamp_s(history_us[i], history_us[oldest]), b = history_rad[i];
        st += t; sb += b; stt += t * t; stb += t * b;
    }
    float n = (float)history_n;
    return (n * stb - st * sb) / (n * stt - st * st);
}

static void remember(const watch_observation_t *o, uint32_t t_us) {
    target.bearing_rad = bearing_rad(o->where);
    target.up_rad = asinf(o->where[2]);
    if (o->range_m >= 0) target.range_m = o->range_m;
    target.last_seen_us = t_us;
    history_rad[history_next] = target.bearing_rad;
    history_us[history_next] = t_us;
    history_next = (history_next + 1) % HISTORY;
    if (history_n < HISTORY) history_n++;
    while (history_n > 1 && stamp_us(t_us, history_us[(history_next - history_n + HISTORY) % HISTORY]) > HISTORY_US)
        history_n--; // older than the window
    target.rate_radps = rate();
}

track_event_t tracker_add(const watch_observation_t *observations, int n, uint32_t t_us) {
    lost = false;
    if (!active) {
        if (n == 0) return TRACK_NOTHING;
        active = true;
        leaving_told = false;
        target.range_m = -1.0f;
        history_n = history_next = 0;
        remember(&observations[0], t_us); // the biggest
        return TRACK_NEW;
    }
    // The biggest blob that reaches where the target is expected (a person close by
    // is a big blob and pieces: its centre says little about where it reaches).
    float expected = target.bearing_rad + target.rate_radps * stamp_s(t_us, target.last_seen_us);
    const watch_observation_t *best = 0;
    for (int k = 0; k < n; k++) {
        const watch_observation_t *o = &observations[k];
        if (expected > o->left_rad + REACH_RAD || expected < o->right_rad - REACH_RAD) continue;
        if (o->range_m >= 0 && target.range_m >= 0 && fabsf(o->range_m - target.range_m) > RANGE_M) continue;
        if (!best || o->cells > best->cells) best = o;
    }
    if (best) {
        remember(best, t_us);
        // At the edge, moving outward (the angular speed is 0 until it is measured).
        float b = target.bearing_rad, r = target.rate_radps;
        if (leaving_told || fabsf(b) < EDGE_RAD || fabsf(r) < LEAVING_RADPS || (b > 0) != (r > 0)) return TRACK_NOTHING;
        leaving_told = true;
        return b > 0 ? TRACK_LEAVING_LEFT : TRACK_LEAVING_RIGHT;
    }
    if (stamp_us(t_us, target.last_seen_us) < LOST_US) return TRACK_NOTHING;
    active = false;
    lost = true;
    if (target.bearing_rad > EDGE_RAD) return TRACK_EXITED_LEFT;
    if (target.bearing_rad < -EDGE_RAD) return TRACK_EXITED_RIGHT;
    return TRACK_STOPPED;
}
