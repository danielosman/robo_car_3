#include <math.h>
#include "pico/stdlib.h"
#include "stamp.h"
#include "geom.h"
#include "body.h"
#include "pose.h"

#define POSE_HISTORY        64      // reports: 1.28 s at 50 Hz
#define MAX_AHEAD_US   50000   // how far past the latest report pose_at() extrapolates

typedef struct {
    uint32_t t_us;             // PicoA's clock
    pose_t pose;
    float v_mps, w_radps;
    bool still;                // PicoB: wheels and gyro still
} sample_t;

static sample_t history[POSE_HISTORY];
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
        newest = (newest + 1) % POSE_HISTORY;
        if (count < POSE_HISTORY) count++;
        history[newest] = (sample_t){
            .t_us = t_us,
            .pose = {o.x_m, o.y_m, o.yaw_rad, o.pitch_rad},
            .v_mps = o.v_mps, .w_radps = o.w_radps, .still = o.stationary,
        };
    }
}

bool pose_now(pose_t *pose) {
    if (!count) return false;
    *pose = history[newest].pose;
    return true;
}

// pose_at(), and whether the robot stood still then (the reports around t both said so).
static bool at(uint32_t t_us, pose_t *pose, bool *still) {
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
        *still = latest->still;
        return true;
    }
    // Walk back to the pair of reports around t.
    for (int k = 1; k < count; k++) {
        const sample_t *a = &history[(newest - k + POSE_HISTORY) % POSE_HISTORY];
        const sample_t *b = &history[(newest - k + 1 + POSE_HISTORY) % POSE_HISTORY];
        if (stamp_us(t_us, a->t_us) < 0) continue;
        float span = (float)stamp_us(b->t_us, a->t_us);
        float f = span > 0 ? (float)stamp_us(t_us, a->t_us) / span : 0;
        pose->x_m = a->pose.x_m + f * (b->pose.x_m - a->pose.x_m);
        pose->y_m = a->pose.y_m + f * (b->pose.y_m - a->pose.y_m);
        pose->yaw_rad = a->pose.yaw_rad + f * (b->pose.yaw_rad - a->pose.yaw_rad);
        pose->pitch_rad = a->pose.pitch_rad + f * (b->pose.pitch_rad - a->pose.pitch_rad);
        *still = a->still && b->still;
        return true;
    }
    return false;
}

bool pose_at(uint32_t t_us, pose_t *pose) {
    bool still;
    return at(t_us, pose, &still);
}

void pose_stamp(const tof_frame_t *frame, posed_frame_t *out) {
    out->frame = *frame;
    pose_t mid, start, end;
    bool still;
    out->pose_known = at(frame->t_us, &mid, &still);
    if (!out->pose_known) {
        out->x_m = out->y_m = out->yaw_rad = out->pitch_rad = out->turn_rad = 0;
        out->still = false;
        return;
    }
    out->x_m = mid.x_m;
    out->y_m = mid.y_m;
    out->yaw_rad = wrap_pi(mid.yaw_rad);
    out->pitch_rad = mid.pitch_rad;
    out->still = still;
    // The turn over the whole measurement; none known (its end not reported yet): none.
    bool s;
    uint32_t half_us = frame->period_us / 2;
    out->turn_rad = at(frame->t_us - half_us, &start, &s) && at(frame->t_us + half_us, &end, &s)
                  ? end.yaw_rad - start.yaw_rad : 0;
}
