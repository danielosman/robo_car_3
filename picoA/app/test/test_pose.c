// Host test for picoA/app/pose.c: interpolation between odometry reports, the
// history's limits, extrapolating a little past the latest report. The test
// provides body.h's functions as PicoB. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -o build/test_pose picoA/app/test/test_pose.c -lm && build/test_pose
#include <assert.h>
#include <stdio.h>
#include "../pose.c"

static odom_report_t report;
static uint32_t report_time_us;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &report; }
uint32_t body_odom_time_us(void) { return report_time_us; }

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
    return 0;
}
