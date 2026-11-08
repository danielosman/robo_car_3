#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_start.h"
#include "units.h"
#include "geom.h"
#include "stamp.h"
#include "body.h"
#include "camera.h"
#include "watch_tof.h"
#include "watch_camera.h"
#include "tracker.h"
#include "watch.h"

#define LOG_PERIOD_US 500000 // while movement goes on
#define MAX_OBS 4

// One detector's movement as the log shows it.
typedef struct {
    const char *name, *cells; // "ToF", "zones"
    bool moving;
    uint32_t last_log_us;
} detector_log_t;

static detector_log_t tof_log = {.name = "ToF", .cells = "zones"}, camera_log = {.name = "camera", .cells = "blocks"};
static bool still, logging;
static uint32_t adjustments, last_target_log_us;
static bool moving;          // the latest ToF frame had movement...
static float strongest_rad;  // ...the biggest blob's direction (+ = left)...
static uint32_t strongest_us; // ...measured then

// Robot time in seconds, in front of every line of the movement log.
void watch_stamp(void) { printf("%8.1f ", (double)((float)(clock_since_start_us() / 1000) / 1000.0f)); }

static bool robot_still(void) {
    return !body_connected() || body_odom()->stationary;
}

bool watch_init(const mounting_t *m) { return watch_camera_init(m); }

// The robot while a ToF frame was measured: its yaw at the start and the end of the
// measurement (the frame's middle ± half its turn). A frame without a pose: where
// the last one with a pose was, not turning (none yet: at the origin).
static tof_view_t last_view; // the latest frame's

static tof_view_t tof_view(const posed_frame_t *f) {
    if (!f->pose_known) return (tof_view_t){.yaw_start_rad = last_view.yaw_end_rad, .yaw_end_rad = last_view.yaw_end_rad,
                                            .x_m = last_view.x_m, .y_m = last_view.y_m};
    return (tof_view_t){.yaw_start_rad = f->yaw_rad - f->turn_rad / 2, .yaw_end_rad = f->yaw_rad + f->turn_rad / 2,
                        .x_m = f->x_m, .y_m = f->y_m};
}

static float turn_dps; // the last ToF frame's turn rate, for the log

static void log_movement(detector_log_t *d, const watch_obs_t *obs, int n) {
    uint32_t now = time_us_32();
    if (n == 0) {
        if (d->moving && logging) { watch_stamp(); printf("Movement (%s) ended\n", d->name); }
        d->moving = false;
        return;
    }
    if (d->moving && now - d->last_log_us < LOG_PERIOD_US) return;
    d->moving = true;
    d->last_log_us = now;
    if (!logging) return;
    watch_stamp();
    printf("Movement (%s):", d->name);
    for (int k = 0; k < n; k++) {
        const watch_obs_t *o = &obs[k];
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
    watch_stamp();
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

// Whether the robot stands still (PicoB says so; with no PicoB it can't move). A
// change: the tracker's target is forgotten when it starts moving, the camera learns
// the view again.
static void follow_still(void) {
    bool now_still = robot_still();
    if (!now_still && still && tracker_target()) {
        tracker_reset();
        if (logging) { watch_stamp(); printf("Target forgotten: the robot moves\n"); }
    }
    if (now_still != still) watch_camera_restart(); // moving: the exposure follows the view; still: learn it
    if (!now_still) {
        if (camera_log.moving) log_movement(&camera_log, 0, 0);
    }
    still = now_still;
}

void watch_add(const posed_frame_t *pf) {
    follow_still();
    const tof_frame_t *f = &pf->frame;
    tof_view_t view = last_view = tof_view(pf);
    turn_dps = (view.yaw_end_rad - view.yaw_start_rad) / ((float)f->period_us * 1e-6f) * DEG_PER_RAD;
    watch_obs_t obs[MAX_OBS];
    int n = watch_tof_add(f, &view, obs, MAX_OBS);
    log_movement(&tof_log, obs, n);
    moving = n > 0;
    if (moving) { strongest_rad = bearing_rad(obs[0].where); strongest_us = f->t_us; }
    // Diagnosis (robot test 9 Oct: the robot twitched towards one-zone blobs at the
    // edges): the zone's reading and its background.
    if (logging && n > 0 && obs[0].cells == 1)
        for (int z = 0; z < TOF_ZONES; z++)
            if (watch_tof_moved(z))
                printf("         zone row %d col %d: reads %u mm (%s), background %u mm (0: nothing)\n", z / TOF_COLS + 1,
                       z % TOF_COLS + 1, (unsigned)f->zone[z].distance_mm,
                       tof_valid(&f->zone[z]) ? "valid" : f->zone[z].kind == TOF_NOTHING ? "nothing" : "unsure",
                       (unsigned)watch_tof_background_mm(z)); // 1-8 (README)
    // The tracker follows directions as the robot sees them: only while it stands still.
    if (!still || !watch_tof_ready()) return;
    track_event_t e = tracker_add(obs, n, f->t_us);
    log_target(e, f->t_us);
}

static void update_camera(void) {
    camera_frame_t f;
    if (!camera_frame(&f) || !still) return;
    watch_obs_t obs[MAX_OBS];
    int n = watch_camera_add(&f, obs, MAX_OBS);
    if (n >= 0) log_movement(&camera_log, obs, n);
    if (watch_camera_exposure_adjustments() != adjustments) {
        adjustments = watch_camera_exposure_adjustments();
        if (camera_log.moving) log_movement(&camera_log, 0, 0);
        if (logging) { watch_stamp(); printf("Camera: adjusting the exposure, then learning the view again\n"); }
    }
}

void watch_update(void) {
    follow_still();
    update_camera();
}

bool watch_watching(void) {
    bool camera_ready = watch_camera_ready() || camera_frames_captured() == 0; // no camera: the VL53 alone
    return still && watch_tof_ready() && camera_ready;
}
bool watch_still(void) { return still; }
#define STRONGEST_MAX_AGE_US 300000 // older than a few frames: the ToF has stopped

bool watch_strongest(float *bearing_rad, uint32_t *t_us) {
    if (!moving || stamp_us(time_us_32(), strongest_us) > STRONGEST_MAX_AGE_US) return false;
    *bearing_rad = strongest_rad;
    *t_us = strongest_us;
    return true;
}
void watch_log(bool on) { logging = on; }
