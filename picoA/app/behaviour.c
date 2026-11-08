#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "geom.h"
#include "stamp.h"
#include "body.h"
#include "motion.h"
#include "pose.h"
#include "cell_map.h"
#include "watch.h"
#include "move.h"
#include "behaviour.h"

#define FOLLOW_START_RAD (8 * RAD_PER_DEG) // the movement this far off straight ahead: turn towards it...
#define FOLLOW_STOP_RAD  (3 * RAD_PER_DEG) // ...until it is this close, and
#define FOLLOW_STILL_RADPS (5 * RAD_PER_DEG) // no faster than this
#define FOLLOW_GAIN      2.0f      // turn rate per angle still to go (1/s), on top of the movement's own angular speed
#define FOLLOW_FIT_US    500000    // its angular speed: a line through its directions of the last 0.5 s
#define FOLLOW_HISTORY   10
#define FOLLOW_GAP_US    300000    // no sighting this long, or a jump this big: a new movement
#define FOLLOW_JUMP_RAD  (15 * RAD_PER_DEG)
#define GOTO_AIM_RAD     (1.5f * RAD_PER_DEG) // turned to within this of the goal's bearing: drive
#define GOTO_STEER       2.0f      // while driving: turn rate per heading error (1/s)...
#define GOTO_STEER_RADPS 0.3f      // ...at most this
// printf takes doubles; the (double) casts below are for printing only.

typedef enum { DRIVE, TURN, WATCH, GOTO } action_t;
static const char *const action_name[] = {"Move", "Turn", "Watch", "Go to"};
static const move_kind_t kind_of[] = {MOVE_DRIVE, MOVE_TURN, MOVE_WATCH, MOVE_GOTO};

// The action of the running move (its act phase: move.h).
static action_t action;
static float goal;              // DRIVE: metres (+ = forward); TURN: radians (+ = left); WATCH: seconds
static odom_report_t start;     // the robot when the act phase began
static absolute_time_t run_start;
static bool following;          // watching: turning towards a movement
static bool obstacle_stop;      // moving: stopped short, before an obstacle on the map
static float goal_x, goal_y;    // GOTO: the point (map frame) and how far before it to stop
static float goal_stop_m;
static bool goal_driving;       // GOTO: turned towards it, now driving
// The movement's recent directions in the world (the map's yaw, unwrapped), when seen.
static float seen_rad[FOLLOW_HISTORY];
static uint32_t seen_us[FOLLOW_HISTORY];
static int seen_n;
static uint32_t last_seen_us;

static void act_begin(const odom_report_t *o);
static move_act_t act_step(const odom_report_t *o);
static void act_end(const odom_report_t *o, move_end_t how);
static const move_actor_t behaviour_actor = {act_begin, act_step, act_end};

// Starts the move of action a; the action's own state is set after (the move
// replaced, if any, has ended by then).
static void begin(action_t a, float g, float param, uint32_t number) {
    move_start(kind_of[a], param, number, &behaviour_actor);
    action = a;
    goal = g;
}

// A move along dir (+1 forward, -1 back) from o: how far the edge on that side is from
// the nearest obstacle on the map in its path (blocked or overhang), looking at most
// look_m past the stop point; unknown cells count as free.
static float obstacle_gap_m(const odom_report_t *o, float dir, float look_m) {
    return cell_map_obstacle_along(o->x_m, o->y_m, dir * cosf(o->yaw_rad), dir * sinf(o->yaw_rad), PATH_HALF_M,
                                   EDGE_M + STOP_BEFORE_M + look_m) - EDGE_M;
}

void behaviour_move(float distance_m, uint32_t number) {
    const char *side = distance_m > 0 ? "ahead" : "behind";
    float gap_m = obstacle_gap_m(body_odom(), distance_m > 0 ? 1.0f : -1.0f, fabsf(distance_m));
    if (gap_m <= STOP_BEFORE_M + 0.01f) {
        printf("Move: not moving: obstacle %.0f cm %s (map), closer than %.0f cm\n", (double)(gap_m * 100), side,
               (double)(STOP_BEFORE_M * 100));
        move_refused(MOVE_DRIVE, distance_m, number, MOVE_END_OBSTACLE);
        return;
    }
    begin(DRIVE, distance_m, distance_m, number);
    printf("Move: %.0f cm %s%s\n", (double)fabsf(distance_m * 100), distance_m > 0 ? "forward" : "back",
           distance_m > 0 ? "" : " (the VL53 can't see behind: it stops only for what the map already has there)");
}

void behaviour_turn(float angle_rad, uint32_t number) {
    begin(TURN, angle_rad, angle_rad, number);
    printf("Turn: %.0f deg %s\n", (double)fabsf(angle_rad * DEG_PER_RAD), angle_rad > 0 ? "left" : "right");
}

void behaviour_goto(float x_m, float y_m, float stop_m, uint32_t number) {
    const odom_report_t *o = body_odom();
    float dx = x_m - o->x_m, dy = y_m - o->y_m, d = hypotf(dx, dy), goal_m = d;
    if (d - stop_m < 0.05f) {
        printf("Go to: already there (%.0f cm away)\n", (double)(d * 100));
        move_refused(MOVE_GOTO, goal_m, number, MOVE_END_DONE);
        return;
    }
    if (d - stop_m > GOTO_MAX_M) { // only so far towards it
        float k = (GOTO_MAX_M + stop_m) / d;
        printf("Go to: the goal is %.1f m away; going %.1f m towards it\n", (double)d, (double)GOTO_MAX_M);
        x_m = o->x_m + k * dx;
        y_m = o->y_m + k * dy;
        d = GOTO_MAX_M + stop_m;
    }
    begin(GOTO, d, goal_m, number);
    goal_x = x_m;
    goal_y = y_m;
    goal_stop_m = stop_m;
    goal_driving = false;
    printf("Go to: (%+.0f, %+.0f) cm, %.0f cm away at %+.0f deg (map frame)%s\n", (double)(x_m * 100), (double)(y_m * 100),
           (double)(d * 100), (double)(atan2f(dy, dx) * DEG_PER_RAD), stop_m > 0 ? ", stopping before it" : "");
}

void behaviour_watch(float seconds, uint32_t number) {
    begin(WATCH, seconds, seconds, number);
    printf("Watch: for %.0f s the robot turns towards the biggest movement\n", (double)seconds);
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
    bool moving = watch_strongest(&bearing_rad, &t_us) && pose_now(&now);
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
        watch_stamp();
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
    float dx = o->x_m - start.x_m, dy = o->y_m - start.y_m;
    if (action == DRIVE && obstacle_stop) {
        float dir = goal < 0 ? -1.0f : 1.0f;
        printf("Move stopped: obstacle %.0f cm %s (map), moved %.0f cm\n", (double)(obstacle_gap_m(o, dir, 0.1f) * 100),
               dir > 0 ? "ahead" : "behind", (double)(hypotf(dx, dy) * 100));
        return;
    }
    if (action == GOTO) {
        float off = hypotf(goal_x - o->x_m, goal_y - o->y_m);
        if (obstacle_stop)
            printf("Go to stopped: obstacle %.0f cm ahead (map), %.0f cm from the goal\n",
                   (double)(obstacle_gap_m(o, 1.0f, 0.1f) * 100), (double)(off * 100));
        else printf("Go to done: %.1f s; at (%+.0f, %+.0f) cm, %.0f cm from the goal\n",
                    (double)(absolute_time_diff_us(run_start, get_absolute_time()) * 1e-6f), (double)(o->x_m * 100),
                    (double)(o->y_m * 100), (double)(off * 100));
        return;
    }
    float c = cosf(start.yaw_rad), s = sinf(start.yaw_rad);
    printf("%s done: %.1f s; odometry: %+.1f cm forward, %+.1f cm left, turned %+.1f deg\n", action_name[action],
           (double)(absolute_time_diff_us(run_start, get_absolute_time()) * 1e-6f), (double)((c * dx + s * dy) * 100),
           (double)((-s * dx + c * dy) * 100), (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD));
}

// Going to a point: turn in place towards it, then drive, steering a little to
// keep it ahead, and stop goal_stop_m before it (measured along the way) or 15 cm
// before an obstacle on the map. Returns true when done.
static bool go_to(const odom_report_t *o) {
    float dx = goal_x - o->x_m, dy = goal_y - o->y_m;
    float error_rad = wrap_pi(atan2f(dy, dx) - o->yaw_rad);
    if (!goal_driving) {
        if (fabsf(error_rad) > GOTO_AIM_RAD) { body_drive(0, motion_turn_rate(error_rad)); return false; }
        goal_driving = true;
    }
    float c = cosf(o->yaw_rad), s = sinf(o->yaw_rad);
    float remaining_m = c * dx + s * dy - goal_stop_m; // along the way: never circles around it
    float room_m = obstacle_gap_m(o, 1.0f, fmaxf(remaining_m, 0)) - STOP_BEFORE_M;
    obstacle_stop = room_m < remaining_m;
    if (obstacle_stop) remaining_m = room_m;
    if (remaining_m <= 0) return true;
    float w = fmaxf(-GOTO_STEER_RADPS, fminf(GOTO_STEER_RADPS, GOTO_STEER * error_rad));
    body_drive(motion_speed(remaining_m), w);
    return false;
}

// The act phase begins: the robot stands still, motors on.
static void act_begin(const odom_report_t *o) {
    start = *o;
    run_start = get_absolute_time();
    following = false;
    obstacle_stop = false;
    seen_n = 0;
    watch_log(action == WATCH); // movement lines only while watching
}

// Runs the action; returns true when it is done.
static bool step(const odom_report_t *o) {
    switch (action) {
    case TURN: {
        float dir = goal < 0 ? -1.0f : 1.0f;
        float remaining_rad = fabsf(goal) - dir * (o->yaw_rad - start.yaw_rad);
        if (remaining_rad <= 0) return true;
        body_drive(0, motion_turn_rate(dir * remaining_rad));
        return false;
    }
    case DRIVE: {
        float dir = goal < 0 ? -1.0f : 1.0f;
        // Checked every time: the map grows as the robot drives.
        float remaining_m = fabsf(goal) - hypotf(o->x_m - start.x_m, o->y_m - start.y_m);
        float room_m = obstacle_gap_m(o, dir, fmaxf(remaining_m, 0)) - STOP_BEFORE_M;
        obstacle_stop = room_m < remaining_m;
        if (obstacle_stop) remaining_m = room_m;
        if (remaining_m <= 0) return true;
        body_drive(motion_speed(dir * remaining_m), 0);
        return false;
    }
    case WATCH:
        if (absolute_time_diff_us(run_start, get_absolute_time()) >= (int64_t)(goal * 1e6f)) return true;
        watch();
        return false;
    case GOTO:
        return go_to(o);
    }
    return true;
}

static move_act_t act_step(const odom_report_t *o) {
    if (!step(o)) return MOVE_ACTING;
    return (action == DRIVE || action == GOTO) && obstacle_stop ? MOVE_ACT_OBSTACLE : MOVE_ACT_DONE;
}

// The move has ended: done or before an obstacle, what odometry measured; the
// movement lines stop.
static void act_end(const odom_report_t *o, move_end_t how) {
    watch_log(false);
    if (how == MOVE_END_DONE || how == MOVE_END_OBSTACLE) say_done(o);
}
