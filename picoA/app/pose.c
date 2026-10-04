#include <math.h>
#include "pico/stdlib.h"
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
static uint32_t last_report_t_us; // PicoB's t_us of the latest report taken
static pose_t origin;             // the map frame's origin, in odometry's frame

void pose_update(void) {
    if (!body_connected()) return;
    const odom_report_t *o = body_odom();
    if (count && o->t_us == last_report_t_us) return;
    last_report_t_us = o->t_us;
    newest = (newest + 1) % HISTORY;
    if (count < HISTORY) count++;
    history[newest] = (sample_t){
        .t_us = body_odom_time_us(),
        .pose = {o->x_m, o->y_m, o->yaw_rad, o->pitch_rad},
        .v_mps = o->v_mps, .w_radps = o->w_radps,
    };
}

static int32_t after(uint32_t t_us, uint32_t ref_us) { return (int32_t)(t_us - ref_us); }

void pose_set_origin(void) {
    if (count) origin = history[newest].pose;
}

// From odometry's frame to the map's.
static pose_t in_map(pose_t p) {
    float dx = p.x_m - origin.x_m, dy = p.y_m - origin.y_m;
    float c = cosf(origin.yaw_rad), s = sinf(origin.yaw_rad);
    p.x_m = c * dx + s * dy;
    p.y_m = -s * dx + c * dy;
    p.yaw_rad -= origin.yaw_rad;
    return p;
}

bool pose_now(pose_t *pose) {
    if (!count) return false;
    *pose = in_map(history[newest].pose);
    return true;
}

static bool odometry_at(uint32_t t_us, pose_t *pose);

bool pose_at(uint32_t t_us, pose_t *pose) {
    if (!odometry_at(t_us, pose)) return false;
    *pose = in_map(*pose);
    return true;
}

static bool odometry_at(uint32_t t_us, pose_t *pose) {
    if (!count) return false;
    const sample_t *latest = &history[newest];
    int32_t ahead_us = after(t_us, latest->t_us);
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
        if (after(t_us, a->t_us) < 0) continue;
        float span = (float)after(b->t_us, a->t_us);
        float f = span > 0 ? (float)after(t_us, a->t_us) / span : 0;
        pose->x_m = a->pose.x_m + f * (b->pose.x_m - a->pose.x_m);
        pose->y_m = a->pose.y_m + f * (b->pose.y_m - a->pose.y_m);
        pose->yaw_rad = a->pose.yaw_rad + f * (b->pose.yaw_rad - a->pose.yaw_rad);
        pose->pitch_rad = a->pose.pitch_rad + f * (b->pose.pitch_rad - a->pose.pitch_rad);
        return true;
    }
    return false;
}
