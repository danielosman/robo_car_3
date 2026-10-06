#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "body.h"
#include "motion.h"
#include "surroundings.h"
#include "world_map.h"
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
#define LOOK_TURN_RADPS  1.0f      // turning after a target
#define LOOK_RAMP_S      0.5f      // a turn takes its angle at LOOK_TURN_RADPS plus about this (speeding up, slowing down)
#define LOOK_MAX_RAD     (90 * RAD_PER_DEG)
#define LOOK_DONE_RAD    (1 * RAD_PER_DEG)
// printf takes doubles; the (double) casts below are for printing only.

// After each turn the robot settles (stands still) before the next step.
// Building and printing the map keeps the loop busy longer than PicoB waits for
// drive commands, so the motors are off meanwhile (MAPPING) and come back on before
// the robot turns again (RESTARTING). Then it watches (WATCHING), turning after a
// target leaving the view (LOOKING).
typedef enum {
    IDLE, STARTING, SCANNING, SETTLING_AFTER_SCAN, MAPPING, RESTARTING, FACING, SETTLING_AFTER_FACING,
    WATCH_STARTING, WATCHING, LOOKING
} state_t;
static state_t state;
static float start_yaw_rad, target_yaw_rad;
static absolute_time_t deadline, settle_min_end;
static float look_rate_radps; // the target's angular speed when the robot last turned after it
static bool just_turned;      // stopped after a turn, learning the view: a target leaving meanwhile is followed on

static float wrap_angle(float a) {
    while (a > PI_F) a -= 2 * PI_F;
    while (a <= -PI_F) a += 2 * PI_F;
    return a;
}

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
            sum_m += FREE_FROM_M + map_free_distance(p->x_m + FREE_FROM_M * dx, p->y_m + FREE_FROM_M * dy,
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

// Headings are the map's: 0 = where the robot faced when the scan started.
// Runs once PicoB reports the motors off: nothing else happens in the loop meanwhile.
static void build_map(void) {
    surroundings_learn_finish();
    pose_t p;
    pose_now(&p);
    map_print(&p);
    float free_m, heading = most_open_heading(&p, &free_m);
    float turn_rad = wrap_angle(heading - p.yaw_rad);
    target_yaw_rad = body_odom()->yaw_rad + turn_rad; // turning is measured on odometry's yaw
    printf("Most open direction: heading %+.0f deg, %.1f m free on average across the view; turning %+.0f deg\n",
           (double)(wrap_angle(heading) * DEG_PER_RAD), (double)free_m, (double)(turn_rad * DEG_PER_RAD));
    body_motors(true);
    state = RESTARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
}

void behaviour_scan(void) {
    end("started again");
    printf("Start-up scan: turning %.0f deg in place (~%.0f s) to learn the floor and map the surroundings\n",
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
bool behaviour_watching(void) { return state == WATCHING || state == LOOKING; }

static void start_watching(void) {
    printf("Watching for movement: the robot turns after a target leaving the view (s stops)\n");
    just_turned = false;
    state = WATCHING;
}

// Where the target will be when the turn there ends, if it keeps its angular speed:
// the turn from heading_rad, the target seen at seen_rad since_s ago (both pose
// yaws, unwrapped); at most 90°, in the direction it goes (not wrapped: a fast target
// predicted beyond 180° is still that way).
static float look_turn(float heading_rad, float seen_rad, float rate_radps, float since_s) {
    float turn_rad = 0;
    for (int k = 0; k < 20; k++) { // the turn's length and its duration depend on each other
        float ends_s = since_s + fabsf(turn_rad) / LOOK_TURN_RADPS + LOOK_RAMP_S;
        turn_rad = fmaxf(-LOOK_MAX_RAD, fminf(LOOK_MAX_RAD, seen_rad + rate_radps * ends_s - heading_rad));
    }
    return turn_rad;
}

// The target leaving the view: turn after it. Seen only while learning the view
// after a turn (no angular speed), it goes on the way it went, at the same speed;
// otherwise that tells nothing about where it goes.
static void follow(const leaving_t *l) {
    bool seen_while_learning = l->rate_radps == 0;
    float rate_radps = seen_while_learning ? look_rate_radps : l->rate_radps;
    if (seen_while_learning && (!just_turned || (l->bearing_rad > 0) != (rate_radps > 0))) return;
    pose_t now, then;
    if (!pose_now(&now)) return;
    if (!pose_at(l->t_us, &then)) then = now;
    float since_s = (float)(time_us_32() - l->t_us) * 1e-6f;
    float turn_rad = look_turn(now.yaw_rad, then.yaw_rad + l->bearing_rad, rate_radps, since_s);
    motion_sense_stamp();
    printf("Watching: %s on the %s at %+.0f deg, going %s at %.0f deg/s; turning %+.0f deg\n",
           seen_while_learning ? "it left the view again while learning it" : "target leaving the view",
           l->bearing_rad > 0 ? "left" : "right", (double)(l->bearing_rad * DEG_PER_RAD),
           rate_radps > 0 ? "left" : "right", (double)(fabsf(rate_radps) * DEG_PER_RAD), (double)(turn_rad * DEG_PER_RAD));
    look_rate_radps = rate_radps;
    target_yaw_rad = body_odom()->yaw_rad + turn_rad; // turning is measured on odometry's yaw
    state = LOOKING;
}

void behaviour_update(void) {
    leaving_t leaving;
    bool target_leaving = motion_sense_leaving(&leaving); // taken in every state: only fresh ones count
    if (state == IDLE) return;
    const odom_report_t *o = body_odom();
    if (!body_connected()) { end("PicoB not connected"); return; }
    if (state != STARTING && state != MAPPING && state != RESTARTING && state != WATCH_STARTING && !o->motors_on) {
        printf("%s stopped: PicoB switched the motors off (%s)\n", activity(), stop_reason_text(o->stop_reason));
        state = IDLE;
        return;
    }
    switch (state) {
    case STARTING:
        if (o->motors_on && o->stationary) {
            surroundings_learn_start();
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
        if (settled(o)) {
            body_motors(false);
            state = MAPPING;
            deadline = make_timeout_time_us(START_TIMEOUT_US);
        }
        break;
    case MAPPING:
        if (!o->motors_on) build_map();
        else if (time_reached(deadline)) end("the motors didn't switch off");
        break;
    case RESTARTING:
        if (o->motors_on) state = FACING;
        else if (time_reached(deadline)) end(o->stop_reason != STOP_NONE ? stop_reason_text(o->stop_reason) : "the motors didn't switch on");
        break;
    case SETTLING_AFTER_FACING:
        if (settled(o)) {
            pose_t p;
            pose_now(&p);
            printf("Facing heading %+.0f deg (0 = where the scan started, + = left)\n",
                   (double)(wrap_angle(p.yaw_rad) * DEG_PER_RAD));
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
        if (target_leaving) follow(&leaving);
        else if (just_turned && motion_sense_watching()) just_turned = false; // learned, nothing left meanwhile
        break;
    case LOOKING: {
        float remaining_rad = target_yaw_rad - o->yaw_rad;
        if (fabsf(remaining_rad) < LOOK_DONE_RAD) {
            body_drive(0, 0);
            just_turned = true;
            state = WATCHING;
        } else {
            body_drive(0, motion_turn_rate_up_to(remaining_rad, LOOK_TURN_RADPS));
        }
        break;
    }
    case IDLE:
        break;
    }
}
