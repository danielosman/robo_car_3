#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "body.h"
#include "surroundings.h"
#include "camera.h"
#include "tof_motion.h"
#include "camera_motion.h"
#include "tracker.h"
#include "motion_sense.h"

#define LOG_PERIOD_US 500000 // while movement goes on
#define MAX_OBS 4

// One detector's movement as the log shows it.
typedef struct {
    const char *name, *cells; // "ToF", "zones"
    bool moving;
    uint32_t last_log_us;
} detector_log_t;

static detector_log_t tof_log = {.name = "ToF", .cells = "zones"}, camera_log = {.name = "camera", .cells = "blocks"};
static bool still, logging, have_frame;
static uint32_t last_frame_us, adjustments, last_target_log_us;
static leaving_t leaving;
static bool have_leaving;

// Robot time in seconds, in front of every line of the movement log.
void motion_sense_stamp(void) { printf("%8.1f ", (double)((float)(time_us_64() / 1000) / 1000.0f)); }

static bool robot_still(void) {
    return !body_connected() || body_odom()->stationary;
}

static void log_movement(detector_log_t *d, const motion_obs_t *obs, int n) {
    uint32_t now = time_us_32();
    if (n == 0) {
        if (d->moving && logging) { motion_sense_stamp(); printf("Movement (%s) ended\n", d->name); }
        d->moving = false;
        return;
    }
    if (d->moving && now - d->last_log_us < LOG_PERIOD_US) return;
    d->moving = true;
    d->last_log_us = now;
    if (!logging) return;
    motion_sense_stamp();
    printf("Movement (%s):", d->name);
    for (int k = 0; k < n; k++) {
        const motion_obs_t *o = &obs[k];
        float bearing_deg = atan2f(o->where[1], o->where[0]) * DEG_PER_RAD; // printing only
        float up_deg = asinf(o->where[2]) * DEG_PER_RAD;
        printf("%s %d %s, %+.0f deg (+ = left), %+.0f deg up", k ? ";" : "", o->cells, d->cells,
               (double)bearing_deg, (double)up_deg);
        if (o->range_m >= 0) printf(", %.2f m", (double)o->range_m);
        else if (d == &tof_log) printf(", range ?"); // no sure reading in this frame
    }
    printf("\n");
}

// The target the tracker follows: printed when found, twice a second while it is
// seen, and when it is lost (and how).
static void log_target(track_event_t e, uint32_t frame_us) {
    const target_t *t = tracker_target();
    uint32_t now = time_us_32();
    if (!logging || !t) return;
    if (e == TRACK_NOTHING && t->last_seen_us != frame_us) return; // not seen in this frame
    if (e == TRACK_NOTHING && now - last_target_log_us < LOG_PERIOD_US) return;
    last_target_log_us = now;
    motion_sense_stamp();
    float bearing = t->bearing_rad * DEG_PER_RAD, rate = t->rate_radps * DEG_PER_RAD;
    if (e == TRACK_LEAVING_LEFT || e == TRACK_LEAVING_RIGHT)
        printf("Target leaving the view on the %s at %+.0f deg, %.0f deg/s\n", e == TRACK_LEAVING_LEFT ? "left" : "right",
               (double)bearing, (double)fabsf(rate));
    else if (e == TRACK_EXITED_LEFT || e == TRACK_EXITED_RIGHT)
        printf("Target left the view on the %s at %+.0f deg\n", e == TRACK_EXITED_LEFT ? "left" : "right",
               (double)bearing);
    else if (e == TRACK_STOPPED) printf("Target stopped (or too small to see) at %+.0f deg\n", (double)bearing);
    else {
        printf("Target%s: %+.0f deg (+ = left), %+.0f deg up, ", e == TRACK_NEW ? " (new)" : "", (double)bearing,
               (double)(t->up_rad * DEG_PER_RAD));
        if (t->range_m >= 0) printf("%.2f m, ", (double)t->range_m);
        if (fabsf(rate) < 3.0f) printf("about still\n");
        else printf("going %s at %.0f deg/s\n", rate > 0 ? "left" : "right", (double)fabsf(rate));
    }
}

static void update_tof(void) {
    const range_frame_t *f = surroundings_last_frame();
    if (!f || (have_frame && f->t_us == last_frame_us)) return;
    have_frame = true;
    last_frame_us = f->t_us;
    if (!still) return;
    motion_obs_t obs[MAX_OBS];
    bool was_ready = tof_motion_ready();
    int n = tof_motion_add(f, obs, MAX_OBS);
    log_movement(&tof_log, obs, n);
    if (!was_ready) {
        // Just learned: something may have passed out of the view meanwhile.
        for (int left = 0; left < 2; left++)
            if (tof_motion_passed(left, &leaving.bearing_rad, &leaving.t_us)) {
                leaving.rate_radps = 0;
                have_leaving = true;
                if (logging) {
                    motion_sense_stamp();
                    printf("Movement (ToF) while learning the view: something left it on the %s\n", left ? "left" : "right");
                }
            }
        return;
    }
    track_event_t e = tracker_add(obs, n, f->t_us);
    log_target(e, f->t_us);
    if (e == TRACK_LEAVING_LEFT || e == TRACK_LEAVING_RIGHT) {
        const target_t *t = tracker_target();
        leaving = (leaving_t){.bearing_rad = t->bearing_rad, .rate_radps = t->rate_radps, .t_us = t->last_seen_us};
        have_leaving = true;
    }
}

static void update_camera(void) {
    camera_frame_t f;
    if (!camera_frame(&f) || !still) return;
    motion_obs_t obs[MAX_OBS];
    int n = camera_motion_add(&f, obs, MAX_OBS);
    if (n >= 0) log_movement(&camera_log, obs, n);
    if (camera_motion_exposure_adjustments() != adjustments) {
        adjustments = camera_motion_exposure_adjustments();
        if (camera_log.moving) log_movement(&camera_log, 0, 0);
        if (logging) { motion_sense_stamp(); printf("Camera: adjusting the exposure, then learning the view again\n"); }
    }
}

void motion_sense_update(void) {
    bool now_still = robot_still();
    if (now_still && !still) tof_motion_restart(); // a new view: learn it
    if (!now_still && still && tracker_target()) {
        tracker_reset();
        if (logging) { motion_sense_stamp(); printf("Target forgotten: the robot moves\n"); }
    }
    if (now_still != still) camera_motion_restart(); // moving: the exposure follows the view; still: learn it
    if (!now_still) {
        have_leaving = false; // seen from where the robot was
        if (tof_log.moving) log_movement(&tof_log, 0, 0);
        if (camera_log.moving) log_movement(&camera_log, 0, 0);
    }
    still = now_still;
    update_tof();
    update_camera();
}

bool motion_sense_watching(void) {
    bool camera_ready = camera_motion_ready() || camera_frames_captured() == 0; // no camera: the VL53 alone
    return still && tof_motion_ready() && camera_ready;
}
bool motion_sense_still(void) { return still; }
bool motion_sense_leaving(leaving_t *l) {
    if (!have_leaving) return false;
    *l = leaving;
    have_leaving = false;
    return true;
}
void motion_sense_log(bool on) { logging = on; }
bool motion_sense_logging(void) { return logging; }
