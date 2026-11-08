// Host test for picoA/app/pose.c: interpolation between odometry reports, the
// history's limits, extrapolating a little past the latest report; stamping a ToF
// frame: the pose at its middle (yaw wrapped), its turn over the frame's own
// period, still only when PicoB said so on both sides, no pose before the history. The test
// provides body.h's functions as PicoB. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -o build/test_pose picoA/app/test/test_pose.c -lm && build/test_pose
#include <assert.h>
#include <stdio.h>
#include "../pose.c"

static odom_report_t report;
static uint32_t report_time_us, report_count;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &report; }
uint32_t body_odom_time_us(void) { return report_time_us; }
uint32_t body_odom_count(void) { return report_count; }
bool body_odom_get(uint32_t n, odom_report_t *r, uint32_t *t_us) {
    if (n != report_count) return false; // the fake keeps only the latest
    *r = report;
    *t_us = report_time_us;
    return true;
}

// PicoA's clock: 2 s before it wraps, so the history below spans the wrap.
#define T(us) (0u - 2000000u + (uint32_t)(us))

// PicoB turning left at 1 rad/s and driving 0.1 m/s along x (simplified), a report every 20 ms.
static void report_at(uint32_t t_us) {
    report.t_us = T(t_us) + 777;   // PicoB's own clock: pose only uses body's translation
    report_time_us = T(t_us);
    report.x_m = 0.1f * (float)t_us * 1e-6f;
    report.yaw_rad = (float)t_us * 1e-6f;
    report.pitch_rad = 0.01f;
    report.v_mps = 0.1f;
    report.w_radps = 1.0f;
    report_count++;
    pose_update();
}

int main(void) {
    pose_t p;
    assert(!pose_at(T(0), &p));
    for (uint32_t t = 1000000; t <= 3000000; t += 20000) report_at(t);
    pose_update();                   // the same report again isn't stored twice

    assert(pose_at(T(2990000), &p));    // halfway between two reports
    assert(fabsf(p.yaw_rad - 2.99f) < 1e-4f && fabsf(p.x_m - 0.299f) < 1e-5f && p.pitch_rad == 0.01f);
    assert(pose_at(T(3000000), &p) && fabsf(p.yaw_rad - 3.0f) < 1e-5f);
    assert(pose_at(T(3030000), &p) && fabsf(p.yaw_rad - 3.03f) < 1e-4f); // extrapolated 30 ms
    assert(!pose_at(T(3060000), &p));   // too far ahead
    assert(pose_at(T(3000000 - 1260000), &p)); // the oldest kept report: 63 x 20 ms back
    assert(!pose_at(T(3000000 - 1300000), &p)); // older than the history
    printf("OK: pose interpolates, extrapolates 50 ms, keeps 1.26 s\n");

    // Stamping: a frame measured from 2.90 to 3.00 s (its middle 2.95 s).
    tof_frame_t f = {.number = 9, .t_us = T(2950000), .period_us = 100000};
    f.zone[3].distance_mm = 1234;
    posed_frame_t pf;
    pose_stamp(&f, &pf);
    assert(pf.pose_known && pf.frame.number == 9 && pf.frame.zone[3].distance_mm == 1234);
    assert(fabsf(pf.yaw_rad - 2.95f) < 1e-4f && fabsf(pf.x_m - 0.295f) < 1e-5f && pf.pitch_rad == 0.01f);
    assert(fabsf(pf.turn_rad - 0.1f) < 1e-4f && !pf.still); // 1 rad/s for 0.1 s
    f.period_us = 66666;
    pose_stamp(&f, &pf);
    assert(fabsf(pf.turn_rad - 0.0667f) < 1e-4f); // its own period
    // Yaw wrapped: 3.0 rad + a turn past π reads −π…π, the turn unwrapped.
    for (uint32_t t = 3020000; t <= 3300000; t += 20000) report_at(t);
    f.t_us = T(3200000), f.period_us = 100000;
    pose_stamp(&f, &pf);
    assert(fabsf(pf.yaw_rad - (3.2f - 2 * PI_F)) < 1e-4f && fabsf(pf.turn_rad - 0.1f) < 1e-4f);
    // Still: only when the reports on both sides of the middle said so.
    report.stationary = true;
    report_at(3320000);
    report_at(3340000);
    f.t_us = T(3330000), f.period_us = 20000;
    pose_stamp(&f, &pf);
    assert(pf.still);
    f.t_us = T(3310000);
    pose_stamp(&f, &pf);
    assert(!pf.still);
    // Its middle past the latest report (extrapolated, still as that report says), its
    // end past the 50 ms limit: the turn unknown, 0.
    f.t_us = T(3360000), f.period_us = 100000;
    pose_stamp(&f, &pf);
    assert(pf.pose_known && pf.still && pf.turn_rad == 0 && fabsf(pf.yaw_rad - (3.36f - 2 * PI_F)) < 1e-3f);
    // Before the history: no pose, a zero one.
    f.t_us = T(1000000);
    pose_stamp(&f, &pf);
    assert(!pf.pose_known && pf.x_m == 0 && pf.yaw_rad == 0 && pf.turn_rad == 0 && pf.frame.number == 9);
    printf("OK: stamping a frame: its middle's pose, yaw wrapped, its own period's turn, still, no turn past\n"
           "    the extrapolation limit, none before the history\n");
    return 0;
}
