// Host test for picoA/app/watch.c, the ToF side, fed as on the robot: a frame
// stamped by pose.c (PicoB's reports of a robot turning at 30°/s, one every 20 ms),
// packed and unpacked for the watch use, then watch_add(). The robot during a frame
// (its middle yaw ± half its turn) and the turn rate the log shows: 30°/s at 15 and
// at 10 Hz, and with frames of both rates mixed: each frame's own period counts,
// never a global rate. Across yaw ±180° the turn stays 30°/s. A frame without a
// pose: where the last one was, not turning. The camera sends nothing.
// Run from the repo root (run_tests.sh does).
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../pose.c"
#include "../tof_rays.c"
#include "../tof_format.c"
#include "../change_grid.c"
#include "../watch_tof.c"
#include "../tracker.c"
#include "../watch.c"
#include "robot_mounting.h"

// --- fakes: PicoB reporting a robot turning at turn_radps from yaw0; no camera ---
static float turn_radps, yaw0_rad;
static uint32_t t0_us, reports;
static odom_report_t odom = {.stationary = false};
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &odom; }
uint32_t body_odom_count(void) { return reports; }
bool body_odom_get(uint32_t n, odom_report_t *r, uint32_t *t_us) {
    if (n != reports) return false; // the fake keeps only the latest
    *r = odom;
    *t_us = odom.t_us;
    return true;
}
bool camera_frame(camera_frame_t *frame) { (void)frame; return false; }
uint32_t camera_frames_captured(void) { return 0; }
void watch_camera_restart(void) {}
bool watch_camera_ready(void) { return false; }
bool watch_camera_init(const mounting_t *m) { (void)m; return true; }
int watch_camera_add(const camera_frame_t *frame, watch_obs_t *obs, int max) { (void)frame; (void)obs; (void)max; return -1; }
uint32_t watch_camera_exposure_adjustments(void) { return 0; }

// PicoB's reports up to now (every 20 ms).
static void reports_until(uint32_t until_us) {
    while (stamp_us(until_us, odom.t_us) >= 20000) {
        odom.t_us += 20000;
        odom.yaw_rad = yaw0_rad + turn_radps * stamp_s(odom.t_us, t0_us);
        odom.w_radps = turn_radps;
        reports++;
        pose_update();
    }
}

// One frame of `period_us` read now, through the pipeline as main.c runs it.
static void frame(uint32_t period_us, bool posed) {
    fake_now_us += period_us;
    reports_until(time_us_32());
    static uint32_t number;
    tof_frame_t f = {.number = ++number, .t_us = time_us_32() - period_us / 2 - 2500, .period_us = period_us,
                     .beyond_mm = TOF_EXACT};
    for (int i = 0; i < TOF_ZONES; i++) f.zone[i] = (tof_zone_t){.distance_mm = 2000, .kind = TOF_SURE, .targets = 1};
    posed_frame_t p, used;
    pose_stamp(&f, &p);
    if (!posed) p.pose_known = false;
    uint8_t bytes[TOF_FORMAT_MAX];
    uint32_t n_bytes = tof_pack(TOF_USE_WATCH, &p, bytes);
    assert(tof_unpack(tof_format_of(TOF_USE_WATCH), bytes, n_bytes, &used));
    watch_add(&used);
}

static void turning(float yaw0_deg, const uint32_t *periods, int n, const char *what) {
    yaw0_rad = yaw0_deg * RAD_PER_DEG;
    turn_radps = 30 * RAD_PER_DEG;
    t0_us = time_us_32();
    odom.t_us = t0_us - 1000000; // a second of reports before
    reports_until(t0_us);
    for (int k = 0; k < n; k++) {
        frame(periods[k], true);
        float frame_rad = turn_radps * (float)periods[k] * 1e-6f;
        tof_view_t v = last_view;
        assert(fabsf(v.yaw_end_rad - v.yaw_start_rad - frame_rad) < 0.02f * RAD_PER_DEG); // its whole period
        assert(fabsf(turn_dps - 30.0f) < 0.1f);
    }
    printf("ok: %s: a 30 deg/s turn reads %.1f deg/s\n", what, (double)turn_dps);
}

int main(void) {
    tof_rays_init(&ROBOT_MOUNTING);
    watch_init(&ROBOT_MOUNTING);
    const uint32_t at15[] = {66666, 66666, 66666, 66666, 66666}, at10[] = {100000, 100000, 100000, 100000, 100000};
    const uint32_t mixed[] = {66666, 100000, 66666, 100000, 200000};
    turning(0, at15, 5, "at 15 Hz, a frame spans 66.7 ms");
    turning(0, at10, 5, "at 10 Hz, a frame spans 100 ms");
    turning(0, mixed, 5, "frames at 15, 10 and 5 Hz mixed, each its own period");
    turning(179, at10, 5, "turning left across 180 deg");
    turning(-179, at10, 5, "turning left across -180 deg (from -179)");
    // A frame without a pose: where the last one was, not turning.
    tof_view_t last = last_view;
    frame(100000, false);
    assert(turn_dps == 0);
    assert(last_view.yaw_start_rad == last.yaw_end_rad && last_view.yaw_end_rad == last.yaw_end_rad && last_view.x_m == last.x_m);
    printf("ok: a frame without a pose: where the last one was, not turning\n");
    printf("watch: all tests passed\n");
    return 0;
}
