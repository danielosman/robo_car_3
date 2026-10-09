#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "geom.h"
#include "stamp.h"
#include "body.h"
#include "motion.h"
#include "surroundings.h"
#include "cell_map.h"
#include "pose.h"
#include "motion_sense.h"
#include "behaviour.h"

#define SCAN_RAD         (390 * RAD_PER_DEG) // a full turn plus 30° of overlap (§5.4)
#define START_TIMEOUT_US 3000000   // the motors switching on and the robot standing still
#define SETTLE_MIN_US    300000    // after a turn, before using what the robot sees
#define SETTLE_MAX_US    3000000
#define HEADINGS         36        // candidate headings, every 10°
#define SAME_FREE_M      0.05f     // headings this close to the best count as equally open
#define VIEW_HALF_RAD    (25 * RAD_PER_DEG) // the camera's half field of view: where it will watch
#define VIEW_STEP_RAD    (5 * RAD_PER_DEG)
#define FREE_FROM_M      0.15f     // the robot's own footprint isn't looked at
#define FREE_MAX_M       2.0f      // the map reaches 2 m around the robot
#define FOLLOW_START_RAD (8 * RAD_PER_DEG) // the movement this far off straight ahead: turn towards it...
#define FOLLOW_STOP_RAD  (3 * RAD_PER_DEG) // ...until it is this close, and
#define FOLLOW_STILL_RADPS (5 * RAD_PER_DEG) // no faster than this
#define FOLLOW_GAIN      2.0f      // turn rate per angle still to go (1/s), on top of the movement's own angular speed
#define FOLLOW_FIT_US    500000    // its angular speed: a line through its directions of the last 0.5 s
#define FOLLOW_HISTORY   10
#define FOLLOW_GAP_US    300000    // no sighting this long, or a jump this big: a new movement
#define FOLLOW_JUMP_RAD  (15 * RAD_PER_DEG)
// printf takes doubles; the (double) casts below are for printing only.

// After each turn the robot settles (stands still) before the next step. Then it
// watches (WATCHING), turning towards the biggest movement.
typedef enum {
    IDLE, STARTING, SCANNING, SETTLING_AFTER_SCAN, FACING, SETTLING_AFTER_FACING,
    WATCH_STARTING, WATCHING
} state_t;
static state_t state;
static float start_yaw_rad, target_yaw_rad;
static absolute_time_t deadline, settle_min_end;
static bool following;        // turning towards a movement
// The movement's recent directions in the world (the map's yaw, unwrapped), when seen.
static float seen_rad[FOLLOW_HISTORY];
static uint32_t seen_us[FOLLOW_HISTORY];
static int seen_n;
static uint32_t last_seen_us;

static const char *activity(void) { return state >= WATCH_STARTING ? "Watching" : "Start-up scan"; }

static void end(const char *why) {
    if (state == IDLE) return;
    body_drive(0, 0);
    if (why) printf("%s stopped: %s\n", activity(), why);
    state = IDLE;
}

// The direction to watch from: the most free space across the camera's view (§7).
// Where several headings are about as open (often: the map's 2 m reach), the middle
// of the widest such sector.
static float most_open_heading(const pose_t *p, float *free_m) {
    float score_m[HEADINGS], best_m = -1;
    for (int h = 0; h < HEADINGS; h++) {
        float heading = (float)h * 2 * PI_F / HEADINGS, sum_m = 0;
        int n = 0;
        for (float a = heading - VIEW_HALF_RAD; a <= heading + VIEW_HALF_RAD + 1e-3f; a += VIEW_STEP_RAD, n++) {
            float dx = cosf(a), dy = sinf(a);
            sum_m += FREE_FROM_M + cell_map_free_distance(p->x_m + FREE_FROM_M * dx, p->y_m + FREE_FROM_M * dy,
                                                          dx, dy, FREE_MAX_M - FREE_FROM_M);
        }
        score_m[h] = sum_m / (float)n;
        best_m = fmaxf(best_m, score_m[h]);
    }
    // The longest run (around the circle) of headings within SAME_FREE_M of the best,
    // counted from where each run begins.
    bool open[HEADINGS];
    int n_open = 0;
    for (int h = 0; h < HEADINGS; h++) n_open += open[h] = score_m[h] >= best_m - SAME_FREE_M;
    int best_start = 0, best_len = n_open; // all open: any heading will do
    if (n_open < HEADINGS) {
        best_len = 0;
        for (int start = 0; start < HEADINGS; start++) {
            if (!open[start] || open[(start + HEADINGS - 1) % HEADINGS]) continue;
            int len = 0;
            while (open[(start + len) % HEADINGS]) len++;
            if (len > best_len) { best_len = len; best_start = start; }
        }
    }
    *free_m = best_m;
    return (float)(best_start + (best_len - 1) / 2.0f) * 2 * PI_F / HEADINGS;
}

static void settle(state_t next_state) {
    body_drive(0, 0);
    state = next_state;
    settle_min_end = make_timeout_time_us(SETTLE_MIN_US);
    deadline = make_timeout_time_us(SETTLE_MAX_US);
}

static bool settled(const odom_report_t *o) {
    return (time_reached(settle_min_end) && o->stationary) || time_reached(deadline);
}

// After the scan: turn to the most open direction. Headings are the map's: 0 = where
// the robot faced when the scan started.
static void face_most_open(void) {
    pose_t p;
    pose_now(&p);
    float free_m, heading = most_open_heading(&p, &free_m);
    float turn_rad = wrap_pi(heading - p.yaw_rad);
    target_yaw_rad = body_odom()->yaw_rad + turn_rad; // turning is measured on odometry's yaw
    printf("Most open direction: heading %+.0f deg, %.1f m free on average across the view; turning %+.0f deg\n",
           (double)(wrap_pi(heading) * DEG_PER_RAD), (double)free_m, (double)(turn_rad * DEG_PER_RAD));
    state = FACING;
}

void behaviour_scan(void) {
    end("started again");
    printf("Start-up scan: turning %.0f deg in place (~%.0f s) to map the surroundings\n",
           (double)(SCAN_RAD * DEG_PER_RAD), (double)(SCAN_RAD / MOTION_TURN_RADPS));
    body_drive(0, 0);
    body_motors(true);
    state = STARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
}

void behaviour_watch(void) {
    end("started again");
    body_drive(0, 0);
    body_motors(true);
    state = WATCH_STARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
}

void behaviour_stop(void) { end("by command"); }

bool behaviour_busy(void) { return state != IDLE; }
bool behaviour_watching(void) { return state == WATCHING; }

static void start_watching(void) {
    printf("Watching for movement: the robot turns towards the biggest movement (s stops)\n");
    following = false;
    seen_n = 0;
    state = WATCHING;
}

// A new sighting of the biggest movement, at t_us, in the world's directions. A gap
// or a jump starts the history again (another movement).
static void add_sighting(uint32_t t_us, float world_rad) {
    if (seen_n && (stamp_us(t_us, seen_us[seen_n - 1]) > FOLLOW_GAP_US ||
                   fabsf(world_rad - seen_rad[seen_n - 1]) > FOLLOW_JUMP_RAD)) seen_n = 0;
    if (seen_n == FOLLOW_HISTORY) {
        for (int k = 1; k < seen_n; k++) { seen_rad[k - 1] = seen_rad[k]; seen_us[k - 1] = seen_us[k]; }
        seen_n--;
    }
    seen_rad[seen_n] = world_rad;
    seen_us[seen_n++] = t_us;
    int old = 0; // older than the fit's window
    while (old < seen_n - 1 && stamp_us(t_us, seen_us[old]) > FOLLOW_FIT_US) old++;
    for (int k = old; k < seen_n; k++) { seen_rad[k - old] = seen_rad[k]; seen_us[k - old] = seen_us[k]; }
    seen_n -= old;
}

// A straight line through the recent directions: where the movement was at the
// latest sighting, and its angular speed (0 until there are 3 sightings over 0.2 s).
static void fit_sightings(float *where_rad, float *rate_radps) {
    uint32_t t0 = seen_us[seen_n - 1];
    float st = 0, sb = 0, stt = 0, stb = 0, n = (float)seen_n;
    for (int k = 0; k < seen_n; k++) {
        float t = stamp_s(seen_us[k], t0), b = seen_rad[k];
        st += t; sb += b; stt += t * t; stb += t * b;
    }
    float d = n * stt - st * st;
    bool enough = seen_n >= 3 && stamp_us(t0, seen_us[0]) >= 200000 && d > 0;
    *rate_radps = enough ? (n * stb - st * sb) / d : 0;
    *where_rad = enough ? (sb - *rate_radps * st) / n : seen_rad[seen_n - 1]; // the line at t0 (t = 0)
}

// Watching: turn towards the biggest movement and follow it at its own angular speed
// plus a correction for the angle still to go (at most ~29°/s: what moves faster
// gets away). The angle to go is from where the robot faces now, so its own turn
// since the frame was measured counts. Once the robot has turned past where the
// movement was last seen, its speed is dropped: no overshoot. Starts above 8°, stops
// below 3° when the movement is about still, or when nothing moves.
static void watch(void) {
    float bearing_rad;
    uint32_t t_us;
    pose_t now, then;
    bool moving = motion_sense_strongest(&bearing_rad, &t_us) && pose_now(&now);
    if (!moving) seen_n = 0;
    else if (seen_n == 0 || t_us != last_seen_us) {
        if (!pose_at(t_us, &then)) then = now;
        add_sighting(t_us, then.yaw_rad + bearing_rad);
        last_seen_us = t_us;
    }
    float where_rad = 0, rate_radps = 0, to_go_rad = 0;
    if (moving) {
        fit_sightings(&where_rad, &rate_radps);
        to_go_rad = where_rad - now.yaw_rad;
    }
    bool turn = following ? moving && (fabsf(to_go_rad) > FOLLOW_STOP_RAD || fabsf(rate_radps) > FOLLOW_STILL_RADPS)
                          : moving && fabsf(to_go_rad) > FOLLOW_START_RAD;
    if (turn != following) {
        motion_sense_stamp();
        if (turn) printf("Watching: movement at %+.0f deg, turning towards it\n", (double)(to_go_rad * DEG_PER_RAD));
        else printf("Watching: %s\n", moving ? "facing the movement" : "nothing moves");
        following = turn;
    }
    float ahead_radps = to_go_rad * rate_radps < 0 ? 0 : rate_radps; // past it: no more of its speed
    float w = ahead_radps + FOLLOW_GAIN * to_go_rad;
    body_drive(0, turn ? fmaxf(-MOTION_TURN_RADPS, fminf(MOTION_TURN_RADPS, w)) : 0);
}

void behaviour_update(void) {
    if (state == IDLE) return;
    const odom_report_t *o = body_odom();
    if (!body_connected()) { end("PicoB not connected"); return; }
    if (state != STARTING && state != WATCH_STARTING && !o->motors_on) {
        printf("%s stopped: PicoB switched the motors off (%s)\n", activity(), stop_reason_text(o->stop_reason));
        state = IDLE;
        return;
    }
    switch (state) {
    case STARTING:
        if (o->motors_on && o->stationary) {
            surroundings_restart();
            start_yaw_rad = o->yaw_rad;
            state = SCANNING;
        } else if (time_reached(deadline)) {
            end(o->motors_on ? "the robot isn't standing still" : "the motors didn't switch on");
        }
        break;
    case SCANNING: {
        float remaining_rad = SCAN_RAD - (o->yaw_rad - start_yaw_rad);
        if (remaining_rad <= 0) settle(SETTLING_AFTER_SCAN);
        else body_drive(0, motion_turn_rate(remaining_rad));
        break;
    }
    case SETTLING_AFTER_SCAN:
        if (settled(o)) face_most_open();
        break;
    case SETTLING_AFTER_FACING:
        if (settled(o)) {
            pose_t p;
            pose_now(&p);
            printf("Facing heading %+.0f deg (0 = where the scan started, + = left)\n",
                   (double)(wrap_pi(p.yaw_rad) * DEG_PER_RAD));
            start_watching();
        }
        break;
    case FACING: {
        float remaining_rad = target_yaw_rad - o->yaw_rad;
        if (fabsf(remaining_rad) < 1 * RAD_PER_DEG) settle(SETTLING_AFTER_FACING);
        else body_drive(0, motion_turn_rate(remaining_rad));
        break;
    }
    case WATCH_STARTING:
        if (o->motors_on) start_watching();
        else if (time_reached(deadline)) end("the motors didn't switch on");
        break;
    case WATCHING:
        watch();
        break;
    case IDLE:
        break;
    }
}
