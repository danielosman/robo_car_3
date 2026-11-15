#pragma once
// The PC's correction of the robot's pose (doc/OUTLINE_MAP_PLAN.md, step F): PicoB's
// odometry turned by yaw about its origin, then shifted by (x, y). The PC's twin is
// corrected() in pc/robot/src/correction.ts: keep the two the same. Pure.
#include <math.h>

typedef struct { float x_m, y_m, yaw_rad; } planar_t; // a pose on the floor, or a correction

// p (PicoB's odometry) corrected by c.
static inline planar_t correction_apply(planar_t c, planar_t p) {
    float cs = cosf(c.yaw_rad), sn = sinf(c.yaw_rad);
    return (planar_t){cs * p.x_m - sn * p.y_m + c.x_m, sn * p.x_m + cs * p.y_m + c.y_m, p.yaw_rad + c.yaw_rad};
}

// p corrected by c, back to PicoB's odometry.
static inline planar_t correction_undo(planar_t c, planar_t p) {
    float cs = cosf(c.yaw_rad), sn = sinf(c.yaw_rad), dx = p.x_m - c.x_m, dy = p.y_m - c.y_m;
    return (planar_t){cs * dx + sn * dy, -sn * dx + cs * dy, p.yaw_rad - c.yaw_rad};
}

// p corrected by `from`, corrected by `to` instead.
static inline planar_t correction_move(planar_t from, planar_t to, planar_t p) {
    return correction_apply(to, correction_undo(from, p));
}
