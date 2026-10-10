#include <math.h>
#include "pico/stdlib.h"
#include "stamp.h"
#include "body.h"
#include "pose.h"

#define HISTORY        64      // reports: 1.28 s at 50 Hz
#define MAX_AHEAD_US   50000   // how far past the latest report pose_at() extrapolates

typedef struct {
    uint32_t t_us;             // PicoA's clock
    pose_t pose;
    float v_mps, w_radps;
} sample_t;

static sample_t history[HISTORY];
static int count, newest;      // newest: index of the latest sample
static uint32_t taken;          // body's reports taken so far

// Every report body has received since the last call, even several in one loop
// iteration.
void pose_update(void) {
    uint32_t n = body_odom_count();
    if (n - taken > BODY_ODOM_KEPT) taken = n - BODY_ODOM_KEPT; // the older ones are gone
    while (taken != n) {
        odom_report_t o;
        uint32_t t_us;
        if (!body_odom_get(++taken, &o, &t_us)) continue; // no longer kept
        newest = (newest + 1) % HISTORY;
        if (count < HISTORY) count++;
        history[newest] = (sample_t){
            .t_us = t_us,
            .pose = {o.x_m, o.y_m, o.yaw_rad, o.pitch_rad},
            .v_mps = o.v_mps, .w_radps = o.w_radps,
        };
    }
}

bool pose_now(pose_t *pose) {
    if (!count) return false;
    *pose = history[newest].pose;
    return true;
}

bool pose_at(uint32_t t_us, pose_t *pose) {
    if (!count) return false;
    const sample_t *latest = &history[newest];
    int32_t ahead_us = stamp_us(t_us, latest->t_us);
    if (ahead_us >= 0) {
        if (ahead_us > MAX_AHEAD_US) return false;
        float dt = (float)ahead_us * 1e-6f;
        *pose = latest->pose;
        pose->yaw_rad += latest->w_radps * dt;
        pose->x_m += latest->v_mps * dt * cosf(pose->yaw_rad);
        pose->y_m += latest->v_mps * dt * sinf(pose->yaw_rad);
        return true;
    }
    // Walk back to the pair of reports around t.
    for (int k = 1; k < count; k++) {
        const sample_t *a = &history[(newest - k + HISTORY) % HISTORY];
        const sample_t *b = &history[(newest - k + 1 + HISTORY) % HISTORY];
        if (stamp_us(t_us, a->t_us) < 0) continue;
        float span = (float)stamp_us(b->t_us, a->t_us);
        float f = span > 0 ? (float)stamp_us(t_us, a->t_us) / span : 0;
        pose->x_m = a->pose.x_m + f * (b->pose.x_m - a->pose.x_m);
        pose->y_m = a->pose.y_m + f * (b->pose.y_m - a->pose.y_m);
        pose->yaw_rad = a->pose.yaw_rad + f * (b->pose.yaw_rad - a->pose.yaw_rad);
        pose->pitch_rad = a->pose.pitch_rad + f * (b->pose.pitch_rad - a->pose.pitch_rad);
        return true;
    }
    return false;
}
