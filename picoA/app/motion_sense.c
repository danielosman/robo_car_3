#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_start.h"
#include "units.h"
#include "geom.h"
#include "stamp.h"
#include "body.h"
#include "pose.h"
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
static bool moving;          // the latest ToF frame had movement...
static float strongest_rad;  // ...the biggest blob's direction (+ = left)...
static uint32_t strongest_us; // ...measured then

// Robot time in seconds, in front of every line of the movement log.
void motion_sense_stamp(void) { printf("%8.1f ", (double)((float)(clock_since_start_us() / 1000) / 1000.0f)); }

static bool robot_still(void) {
    return !body_connected() || body_odom()->stationary;
}

// The robot while a ToF frame was measured (the 66.7 ms before its middle time + half
// a period); standing at the origin without odometry.
static tof_view_t tof_view(uint32_t t_us) {
    const uint32_t half_us = 1000000u / RANGEFINDER_HZ / 2;
    pose_t a, b;
    if (!pose_at(t_us - half_us, &a) || !pose_at(t_us + half_us, &b)) {
        if (!pose_now(&a)) return (tof_view_t){0};
        b = a;
    }
    return (tof_view_t){.yaw_start_rad = a.yaw_rad, .yaw_end_rad = b.yaw_rad, .x_m = b.x_m, .y_m = b.y_m};
}

static float turn_dps; // the last ToF frame's turn rate, for the log

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
        float bearing_deg = bearing_rad(o->where) * DEG_PER_RAD;
        float up_deg = asinf(o->where[2]) * DEG_PER_RAD;
        printf("%s %d %s, %+.0f deg (+ = left), %+.0f deg up", k ? ";" : "", o->cells, d->cells,
               (double)bearing_deg, (double)up_deg);
        if (o->range_m >= 0) printf(", %.2f m", (double)o->range_m);
        else if (d == &tof_log) printf(", range ?"); // no sure reading in this frame
    }
    if (d == &tof_log && fabsf(turn_dps) >= 5.0f) printf(" (turning at %+.0f deg/s)", (double)turn_dps);
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
    tof_view_t view = tof_view(f->t_us);
    turn_dps = (view.yaw_end_rad - view.yaw_start_rad) * RANGEFINDER_HZ * DEG_PER_RAD;
    motion_obs_t obs[MAX_OBS];
    int n = tof_motion_add(f, &view, obs, MAX_OBS);
    log_movement(&tof_log, obs, n);
    moving = n > 0;
    if (moving) { strongest_rad = bearing_rad(obs[0].where); strongest_us = f->t_us; }
    // Diagnosis (robot test 9 Oct: the robot twitched towards one-zone blobs at the
    // edges): the zone's reading and its background.
    if (logging && n > 0 && obs[0].cells == 1)
        for (int z = 0; z < RANGEFINDER_RAYS; z++)
            if (tof_motion_moved(z))
                printf("         zone row %d col %d: reads %u mm, background %u mm (0: nothing)\n", z / RANGEFINDER_COLS,
                       z % RANGEFINDER_COLS, (unsigned)f->range_mm[z], (unsigned)tof_motion_background_mm(z));
    // The tracker follows directions as the robot sees them: only while it stands still.
    if (!still || !tof_motion_ready()) return;
    track_event_t e = tracker_add(obs, n, f->t_us);
    log_target(e, f->t_us);
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
    if (!now_still && still && tracker_target()) {
        tracker_reset();
        if (logging) { motion_sense_stamp(); printf("Target forgotten: the robot moves\n"); }
    }
    if (now_still != still) camera_motion_restart(); // moving: the exposure follows the view; still: learn it
    if (!now_still) {
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
#define STRONGEST_MAX_AGE_US 300000 // older than a few frames: the ToF has stopped

bool motion_sense_strongest(float *bearing_rad, uint32_t *t_us) {
    if (!moving || stamp_us(time_us_32(), strongest_us) > STRONGEST_MAX_AGE_US) return false;
    *bearing_rad = strongest_rad;
    *t_us = strongest_us;
    return true;
}
void motion_sense_log(bool on) { logging = on; }
