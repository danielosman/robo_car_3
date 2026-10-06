#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "body.h"
#include "surroundings.h"
#include "tof_motion.h"
#include "motion_sense.h"

#define LOG_PERIOD_US 500000 // while movement goes on
#define MAX_OBS 4

static bool still, logging, moving, have_frame;
static uint32_t last_frame_us, last_log_us;

static bool robot_still(void) {
    return !body_connected() || body_odom()->stationary;
}

static void log_movement(const motion_obs_t *obs, int n) {
    uint32_t now = time_us_32();
    if (n == 0) {
        if (moving && logging) printf("Movement (ToF) ended\n");
        moving = false;
        return;
    }
    if (moving && now - last_log_us < LOG_PERIOD_US) return;
    moving = true;
    last_log_us = now;
    if (!logging) return;
    printf("Movement (ToF):");
    for (int k = 0; k < n; k++) {
        const motion_obs_t *o = &obs[k];
        float bearing_deg = atan2f(o->where[1], o->where[0]) * DEG_PER_RAD; // printing only
        float up_deg = asinf(o->where[2]) * DEG_PER_RAD;
        printf("%s %d zones, %+.0f deg (+ = left), %+.0f deg up", k ? ";" : "", o->cells, (double)bearing_deg,
               (double)up_deg);
        if (o->range_m >= 0) printf(", %.2f m", (double)o->range_m);
        else printf(", range ?"); // no sure reading in this frame
    }
    printf("\n");
}

void motion_sense_update(void) {
    bool now_still = robot_still();
    if (now_still && !still) tof_motion_restart(); // a new view: learn it
    if (!now_still && moving) log_movement(0, 0);
    still = now_still;
    const range_frame_t *f = surroundings_last_frame();
    if (!f || (have_frame && f->t_us == last_frame_us)) return;
    have_frame = true;
    last_frame_us = f->t_us;
    if (!still) return;
    motion_obs_t obs[MAX_OBS];
    log_movement(obs, tof_motion_add(f, obs, MAX_OBS));
}

bool motion_sense_watching(void) { return still && tof_motion_ready(); }
void motion_sense_log(bool on) { logging = on; }
bool motion_sense_logging(void) { return logging; }
