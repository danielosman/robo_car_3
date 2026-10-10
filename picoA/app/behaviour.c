#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "geom.h"
#include "stamp.h"
#include "body.h"
#include "motion.h"
#include "pose.h"
#include "motion_sense.h"
#include "recorder.h"
#include "behaviour.h"

#define SCAN_RAD         (390 * RAD_PER_DEG) // a full turn plus 30° of overlap (§5.4)
#define WATCH_US         60000000  // watching ends after a minute
#define RECORD_US        5000000   // record 5 s
#define START_TIMEOUT_US 3000000   // the motors switching on and the robot standing still
#define SETTLE_MIN_US    300000    // done: standing still this long at least...
#define SETTLE_MAX_US    3000000   // ...and at most this long, before the motors go off
#define FOLLOW_START_RAD (8 * RAD_PER_DEG) // the movement this far off straight ahead: turn towards it...
#define FOLLOW_STOP_RAD  (3 * RAD_PER_DEG) // ...until it is this close, and
#define FOLLOW_STILL_RADPS (5 * RAD_PER_DEG) // no faster than this
#define FOLLOW_GAIN      2.0f      // turn rate per angle still to go (1/s), on top of the movement's own angular speed
#define FOLLOW_FIT_US    500000    // its angular speed: a line through its directions of the last 0.5 s
#define FOLLOW_HISTORY   10
#define FOLLOW_GAP_US    300000    // no sighting this long, or a jump this big: a new movement
#define FOLLOW_JUMP_RAD  (15 * RAD_PER_DEG)
// printf takes doubles; the (double) casts below are for printing only.

typedef enum { SCAN, MOVE, TURN, WATCH, RECORD } action_t;
static const char *const action_name[] = {"Scan", "Move", "Turn", "Watch", "Record"};
static const char *const take_name[] = {"scan", "move", "turn", "watch", "record"}; // in recordings

// STARTING: the motors switching on; RUNNING: the action; SETTLING: done, standing
// still before the motors go off.
static enum { IDLE, STARTING, RUNNING, SETTLING } state;
static action_t action;
static float goal;              // MOVE: metres (+ = forward); TURN: radians (+ = left)
static odom_report_t start;     // the robot when the action began running
static absolute_time_t deadline, settle_min_end, run_start;
static bool following;          // watching: turning towards a movement
// The movement's recent directions in the world (the map's yaw, unwrapped), when seen.
static float seen_rad[FOLLOW_HISTORY];
static uint32_t seen_us[FOLLOW_HISTORY];
static int seen_n;
static uint32_t last_seen_us;

// Ends the action early, saying why; the motors go off unless another action follows.
static void end_early(rec_end_t reason, const char *why, bool motors_off) {
    if (state == IDLE) return;
    body_drive(0, 0);
    if (motors_off) body_motors(false);
    motion_sense_log(false);
    printf("%s stopped: %s\n", action_name[action], why);
    state = IDLE;
    recorder_take_end(reason);
}

static void begin(action_t a, float g) {
    end_early(REC_END_REPLACED, "another action started", false);
    action = a;
    goal = g;
    body_drive(0, 0);
    body_motors(true);
    state = STARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
    recorder_take_start(take_name[a], a == WATCH ? WATCH_US * 1e-6f : g);
}

void behaviour_scan(void) {
    begin(SCAN, SCAN_RAD);
    printf("Scan: turning %.0f deg left in place (~%.0f s)\n", (double)(SCAN_RAD * DEG_PER_RAD),
           (double)(SCAN_RAD / MOTION_TURN_RADPS));
}

void behaviour_move(float distance_m) {
    begin(MOVE, distance_m);
    printf("Move: %.0f cm %s\n", (double)fabsf(distance_m * 100), distance_m > 0 ? "forward" : "back");
}

void behaviour_turn(float angle_rad) {
    begin(TURN, angle_rad);
    printf("Turn: %.0f deg %s\n", (double)fabsf(angle_rad * DEG_PER_RAD), angle_rad > 0 ? "left" : "right");
}

void behaviour_watch(void) {
    begin(WATCH, 0);
    printf("Watch: for %d s the robot turns towards the biggest movement (space stops)\n", WATCH_US / 1000000);
}

// Recording only: the robot stands still with the motors off, and nothing is
// checked (it runs without PicoB too).
void behaviour_record(void) {
    if (!recorder_on()) { printf("Record: recording is off (R switches it on)\n"); return; }
    if (!recorder_ready()) { printf("Record: not recorded: no recording connection\n"); return; }
    end_early(REC_END_REPLACED, "another action started", true);
    action = RECORD;
    goal = 0;
    start = *body_odom();
    run_start = get_absolute_time();
    state = RUNNING;
    recorder_take_start(take_name[RECORD], RECORD_US * 1e-6f);
    printf("Record: %d s, the robot stands still\n", RECORD_US / 1000000);
}

void behaviour_stop(void) { end_early(REC_END_STOPPED, "by command", true); }

bool behaviour_busy(void) { return state != IDLE; }

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

// What the action did, by odometry, once the robot stands still.
static void say_done(const odom_report_t *o) {
    if (action == RECORD) {
        printf("Record done: %.1f s\n", (double)(absolute_time_diff_us(run_start, get_absolute_time()) * 1e-6f));
        return;
    }
    float dx = o->x_m - start.x_m, dy = o->y_m - start.y_m;
    float c = cosf(start.yaw_rad), s = sinf(start.yaw_rad);
    printf("%s done: %.1f s; odometry: %+.1f cm forward, %+.1f cm left, turned %+.1f deg\n", action_name[action],
           (double)(absolute_time_diff_us(run_start, get_absolute_time()) * 1e-6f), (double)((c * dx + s * dy) * 100),
           (double)((-s * dx + c * dy) * 100), (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD));
}

// Runs the action; returns true when it is done.
static bool step(const odom_report_t *o) {
    switch (action) {
    case SCAN:
    case TURN: {
        float dir = goal < 0 ? -1.0f : 1.0f;
        float remaining_rad = fabsf(goal) - dir * (o->yaw_rad - start.yaw_rad);
        if (remaining_rad <= 0) return true;
        body_drive(0, motion_turn_rate(dir * remaining_rad));
        return false;
    }
    case MOVE: {
        float dir = goal < 0 ? -1.0f : 1.0f;
        float remaining_m = fabsf(goal) - hypotf(o->x_m - start.x_m, o->y_m - start.y_m);
        if (remaining_m <= 0) return true;
        body_drive(motion_speed(dir * remaining_m), 0);
        return false;
    }
    case WATCH:
        if (absolute_time_diff_us(run_start, get_absolute_time()) >= WATCH_US) return true;
        watch();
        return false;
    case RECORD:
        return absolute_time_diff_us(run_start, get_absolute_time()) >= RECORD_US;
    }
    return true;
}

static void finish(const odom_report_t *o) {
    motion_sense_log(false);
    say_done(o);
    body_motors(false);
    state = IDLE;
    recorder_take_end(REC_END_DONE);
}

void behaviour_update(void) {
    if (state == IDLE) return;
    const odom_report_t *o = body_odom();
    if (action == RECORD) {
        if (step(o)) finish(o);
        return;
    }
    if (!body_connected()) { end_early(REC_END_PICOB_LOST, "PicoB not connected", true); return; }
    if (state != STARTING && !o->motors_on) {
        printf("%s stopped: PicoB switched the motors off (%s)\n", action_name[action], stop_reason_text(o->stop_reason));
        body_drive(0, 0);
        motion_sense_log(false);
        state = IDLE;
        recorder_take_end(REC_END_SAFETY_STOP);
        return;
    }
    switch (state) {
    case STARTING:
        if (o->motors_on && o->stationary) {
            start = *o;
            run_start = get_absolute_time();
            following = false;
            seen_n = 0;
            motion_sense_log(action == WATCH); // movement lines only while watching
            state = RUNNING;
        } else if (time_reached(deadline)) {
            end_early(REC_END_NO_START, o->motors_on ? "the robot isn't standing still" : "the motors didn't switch on", true);
        }
        break;
    case RUNNING:
        if (step(o)) {
            body_drive(0, 0);
            state = SETTLING;
            settle_min_end = make_timeout_time_us(SETTLE_MIN_US);
            deadline = make_timeout_time_us(SETTLE_MAX_US);
        }
        break;
    case SETTLING:
        if ((time_reached(settle_min_end) && o->stationary) || time_reached(deadline)) finish(o);
        break;
    case IDLE:
        break;
    }
}
