#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "units.h"
#include "body.h"
#include "motion.h"
#include "robot_test.h"

#define FULL_TURN_RAD    (2 * PI_F)
#define SQUARE_SIDE_M    0.50f
#define STRAIGHT_M       2.0f
#define TURNS            10
#define DRIFT_S          600
#define DRIFT_REPORT_US  15000000
#define PAUSE_US         700000     // stand still between steps (PicoB re-measures gyro bias)
#define PAUSE_MAX_US     3000000
#define START_TIMEOUT_US 3000000    // motors switching on, or the robot settling for the drift test
// printf takes doubles; the (double) casts below are for printing only.

typedef enum { MOVE_FORWARD, MOVE_TURN, HOLD_STILL } step_kind_t;
typedef struct {
    step_kind_t kind;
    union {
        float distance_m;    // MOVE_FORWARD, + = forward
        float angle_rad;     // MOVE_TURN, + = left
        float duration_s;    // HOLD_STILL
    };
} step_t;

// One test: what it does (steps), what it watches while running (progress) and
// what it reports (result, also after an early stop: complete = false).
typedef struct {
    const char *name;
    bool needs_motors;
    const step_t *steps;
    int n_steps;
    void (*intro)(void);
    void (*begin)(const odom_report_t *o);    // optional
    void (*progress)(const odom_report_t *o); // optional, every loop iteration while moving
    void (*result)(const odom_report_t *o, bool complete);
} test_def_t;

static enum { IDLE, STARTING, MOVING, PAUSED } state;
static const test_def_t *test;
static int step_i;
static odom_report_t start, step_start;     // the robot at the start of the test and of the step
static absolute_time_t test_start_time, step_start_time, step_end_time;
static absolute_time_t deadline, pause_min_end;
static char result[1024];
static size_t result_len;

static float seconds_between(absolute_time_t from, absolute_time_t to) {
    return (float)absolute_time_diff_us(from, to) * 1e-6f;
}

static float seconds_since(absolute_time_t t) { return seconds_between(t, get_absolute_time()); }

// Prints and appends to the result, so it can be shown again later.
static void say(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(result + result_len, sizeof result - result_len, fmt, args);
    va_end(args);
    if (n < 0) return;
    printf("%s", result + result_len);
    result_len += (size_t)n;
    if (result_len > sizeof result - 1) result_len = sizeof result - 1;
}

// Where the robot is relative to the start of the test, in the frame it had
// then: x forward, y left.
static void offset_from_start(const odom_report_t *o, float *forward_m, float *left_m) {
    float dx = o->x_m - start.x_m, dy = o->y_m - start.y_m;
    float c = cosf(start.yaw_rad), s = sinf(start.yaw_rad);
    *forward_m = c * dx + s * dy;
    *left_m = -s * dx + c * dy;
}

static void say_wheels(const odom_report_t *o) {
    say("Wheels: left %+.1f cm, right %+.1f cm.\n",
        (double)((o->wheel_left_m - start.wheel_left_m) * 100),
        (double)((o->wheel_right_m - start.wheel_right_m) * 100));
}

// --- Square ---

static const step_t square_steps[] = {
    {MOVE_FORWARD, .distance_m = SQUARE_SIDE_M}, {MOVE_TURN, .angle_rad = FULL_TURN_RAD / 4},
    {MOVE_FORWARD, .distance_m = SQUARE_SIDE_M}, {MOVE_TURN, .angle_rad = FULL_TURN_RAD / 4},
    {MOVE_FORWARD, .distance_m = SQUARE_SIDE_M}, {MOVE_TURN, .angle_rad = FULL_TURN_RAD / 4},
    {MOVE_FORWARD, .distance_m = SQUARE_SIDE_M}, {MOVE_TURN, .angle_rad = FULL_TURN_RAD / 4},
};

static void square_intro(void) {
    printf("Square test: %.0f cm sides, turning left\n", (double)(SQUARE_SIDE_M * 100));
}

static void square_result(const odom_report_t *o, bool complete) {
    float forward_m, left_m;
    offset_from_start(o, &forward_m, &left_m);
    say("Odometry says the robot is %.1f cm forward, %.1f cm left of the start,\n"
        "having turned %.1f deg (a perfect square: 0 cm, 0 cm, 360 deg).\n",
        (double)(forward_m * 100), (double)(left_m * 100),
        (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD));
    if (complete) say("Measure where it really is to see the odometry error.\n");
}

// --- Turns: gyro scale and effective track width ---

static const step_t turns_steps[] = {{MOVE_TURN, .angle_rad = TURNS * FULL_TURN_RAD}};
static int turns_done;

static void turns_intro(void) {
    printf("Turn test: %d turns left in place (~%.0f min), stopping at %d deg by the gyro\n", TURNS,
           (double)(TURNS * FULL_TURN_RAD / MOTION_TURN_RADPS / 60), TURNS * 360);
}

static void turns_begin(const odom_report_t *o) { (void)o; turns_done = 0; }

static void turns_progress(const odom_report_t *o) {
    if (o->yaw_rad - start.yaw_rad >= (float)(turns_done + 1) * FULL_TURN_RAD)
        printf("Turn %d of %d, %.0f s\n", ++turns_done, TURNS, (double)seconds_since(test_start_time));
}

static void turns_result(const odom_report_t *o, bool complete) {
    float turned_rad = o->yaw_rad - start.yaw_rad;
    float s = seconds_between(test_start_time, complete ? step_end_time : get_absolute_time());
    float forward_m, left_m;
    offset_from_start(o, &forward_m, &left_m);
    say("The gyro says %.1f deg (%d full turns) in %.0f s (%.1f deg/s on average).\n",
        (double)(turned_rad * DEG_PER_RAD), turns_done, (double)s,
        (double)(turned_rad * DEG_PER_RAD / fmaxf(s, 1)));
    if (complete)
        say("Look at the mark: if the robot stopped N deg short of it, the gyro reads N/%d %% too much;\n"
            "past it, too little.\n", TURNS * 360 / 100);
    say_wheels(o);
    // The wheels' distance difference over the angle turned; no PicoB constant needed.
    if (turned_rad > FULL_TURN_RAD / 4)
        say("Effective track width on this floor: %.1f cm. The centre moved %.1f cm.\n",
            (double)((o->wheel_right_m - start.wheel_right_m - (o->wheel_left_m - start.wheel_left_m)) /
                     turned_rad * 100),
            (double)(hypotf(forward_m, left_m) * 100));
}

// --- Straight: encoder distance ---

static const step_t forward_steps[] = {{MOVE_FORWARD, .distance_m = STRAIGHT_M}};
static const step_t back_steps[] = {{MOVE_FORWARD, .distance_m = -STRAIGHT_M}};

static void straight_intro(void) {
    float d_m = test->steps[0].distance_m;
    printf("%s: %.0f m straight %s (~%.0f s). You can unplug the USB now.\n", test->name,
           (double)fabsf(d_m), d_m > 0 ? "ahead" : "back", (double)(fabsf(d_m) / MOTION_SPEED_MPS));
}

static void straight_result(const odom_report_t *o, bool complete) {
    float forward_m, left_m;
    offset_from_start(o, &forward_m, &left_m);
    say("Odometry says %.1f cm forward, %.1f cm left, heading changed %+.1f deg.\n",
        (double)(forward_m * 100), (double)(left_m * 100),
        (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD));
    say_wheels(o);
    if (complete) say("Measure how far the robot really moved: real / odometry - 1 is the distance error.\n");
}

// --- Drift: gyro bias over time, standing still with the motors off ---

static const step_t drift_steps[] = {{HOLD_STILL, .duration_s = DRIFT_S}};
static float bias_start_radps, bias_min_radps, bias_max_radps;
static bool moved;
static absolute_time_t next_drift_report;

static void drift_intro(void) {
    printf("Drift test: motors off; don't touch the robot for %d min. Yaw and gyro bias every %d s.\n",
           DRIFT_S / 60, DRIFT_REPORT_US / 1000000);
}

static void drift_begin(const odom_report_t *o) {
    (void)o;
    bias_start_radps = bias_min_radps = bias_max_radps = body_status()->gyro_bias_radps;
    moved = false;
    next_drift_report = make_timeout_time_us(DRIFT_REPORT_US);
}

static void drift_progress(const odom_report_t *o) {
    float bias_radps = body_status()->gyro_bias_radps;
    bias_min_radps = fminf(bias_min_radps, bias_radps);
    bias_max_radps = fmaxf(bias_max_radps, bias_radps);
    if (!o->stationary) moved = true;
    if (!time_reached(next_drift_report)) return;
    next_drift_report = delayed_by_us(next_drift_report, DRIFT_REPORT_US);
    int s = (int)(seconds_since(test_start_time) + 0.5f);
    printf("Drift %d:%02d  yaw change %+.2f deg  gyro bias %+.4f deg/s (%+.4f since start)%s\n",
           s / 60, s % 60, (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD),
           (double)(bias_radps * DEG_PER_RAD), (double)((bias_radps - bias_start_radps) * DEG_PER_RAD),
           moved ? "  (robot moved)" : "");
}

static void drift_result(const odom_report_t *o, bool complete) {
    (void)complete;
    float spread_dps = (bias_max_radps - bias_min_radps) * DEG_PER_RAD;
    say("Standing still for %.1f min. Yaw changed %+.2f deg (heading is frozen while still, so ~0 is expected).\n"
        "Gyro bias %+.4f deg/s at the start, %+.4f at the end, %+.4f to %+.4f in between.\n"
        "Even if all of that %.4f deg/s change happened during one minute of driving,\n"
        "the heading would be off by only %.2f deg.\n",
        (double)(seconds_since(test_start_time) / 60), (double)((o->yaw_rad - start.yaw_rad) * DEG_PER_RAD),
        (double)(bias_start_radps * DEG_PER_RAD), (double)(body_status()->gyro_bias_radps * DEG_PER_RAD),
        (double)(bias_min_radps * DEG_PER_RAD), (double)(bias_max_radps * DEG_PER_RAD),
        (double)spread_dps, (double)(spread_dps * 60));
    if (moved) say("The robot moved during the test: repeat it without touching the robot.\n");
}

// --- The tests ---

#define STEPS(a) a, (int)(sizeof a / sizeof a[0])
static const test_def_t tests[] = {
    [ROBOT_TEST_SQUARE] = {"Square test", true, STEPS(square_steps), square_intro, NULL, NULL, square_result},
    [ROBOT_TEST_DRIFT] = {"Drift test", false, STEPS(drift_steps), drift_intro, drift_begin, drift_progress, drift_result},
    [ROBOT_TEST_TURNS] = {"Turn test", true, STEPS(turns_steps), turns_intro, turns_begin, turns_progress, turns_result},
    [ROBOT_TEST_FORWARD] = {"Forward test", true, STEPS(forward_steps), straight_intro, NULL, NULL, straight_result},
    [ROBOT_TEST_BACK] = {"Back test", true, STEPS(back_steps), straight_intro, NULL, NULL, straight_result},
};

// --- Running a test ---

// Ends the test: complete, or stopped early (`why`), with what it measured so far.
static void end_test(const char *why) {
    if (state == IDLE) return;
    body_drive(0, 0);
    const odom_report_t *o = body_odom();
    if (!why) say("%s done.\n", test->name);
    else if (state == STARTING) say("%s didn't start: %s\n", test->name, why);
    else say("%s stopped after %.0f s: %s\n", test->name, (double)seconds_since(test_start_time), why);
    if (state != STARTING) test->result(o, !why);
    state = IDLE;
}

// "PicoB switched the motors off (tilted too far)"
static const char *with_stop_reason(const char *what, const odom_report_t *o) {
    static char text[96];
    if (o->stop_reason == STOP_NONE) return what;
    snprintf(text, sizeof text, "%s (%s)", what, stop_reason_text(o->stop_reason));
    return text;
}

static void begin_step(void) {
    step_start = *body_odom();
    step_start_time = get_absolute_time();
    state = MOVING;
}

static void begin(void) {
    start = *body_odom();
    test_start_time = get_absolute_time();
    if (test->begin) test->begin(&start);
    step_i = 0;
    begin_step();
}

// Runs the current step; returns true when it's complete.
static bool run_step(const odom_report_t *o) {
    const step_t *s = &test->steps[step_i];
    switch (s->kind) {
    case MOVE_FORWARD: {
        float dir = s->distance_m < 0 ? -1.0f : 1.0f;
        float remaining_m = fabsf(s->distance_m) - hypotf(o->x_m - step_start.x_m, o->y_m - step_start.y_m);
        if (remaining_m <= 0) return true;
        body_drive(motion_speed(dir * remaining_m), 0);
        return false;
    }
    case MOVE_TURN: {
        float dir = s->angle_rad < 0 ? -1.0f : 1.0f;
        float remaining_rad = fabsf(s->angle_rad) - dir * (o->yaw_rad - step_start.yaw_rad);
        if (remaining_rad <= 0) return true;
        body_drive(0, motion_turn_rate(dir * remaining_rad));
        return false;
    }
    case HOLD_STILL:
        return seconds_since(step_start_time) >= s->duration_s;
    }
    return true;
}

void robot_test_start(robot_test_t t) {
    end_test("another test started");
    test = &tests[t];
    result_len = 0;
    result[0] = '\0';
    test->intro();
    body_drive(0, 0);
    body_motors(test->needs_motors);
    state = STARTING;
    deadline = make_timeout_time_us(START_TIMEOUT_US);
}

void robot_test_stop(void) { end_test("by command"); }

bool robot_test_running(void) { return state != IDLE; }

const char *robot_test_result(void) { return result; }

void robot_test_update(void) {
    if (state == IDLE) return;
    const odom_report_t *o = body_odom();
    if (!body_connected()) { end_test("PicoB not connected"); return; }
    if (state != STARTING && test->needs_motors && !o->motors_on) {
        end_test(with_stop_reason("PicoB switched the motors off", o));
        return;
    }

    switch (state) {
    case STARTING:
        // The drift test starts once the robot has settled (motors off, standing still).
        if (test->needs_motors ? o->motors_on : o->stationary && !o->motors_on) begin();
        else if (time_reached(deadline))
            end_test(test->needs_motors ? with_stop_reason("the motors didn't switch on", o)
                                        : "the robot isn't standing still");
        break;
    case MOVING:
        if (test->progress) test->progress(o);
        if (run_step(o)) {
            body_drive(0, 0);
            step_end_time = get_absolute_time();
            state = PAUSED;
            pause_min_end = make_timeout_time_us(PAUSE_US);
            deadline = make_timeout_time_us(PAUSE_MAX_US);
        }
        break;
    case PAUSED:
        if ((time_reached(pause_min_end) && o->stationary) || time_reached(deadline)) {
            if (++step_i == test->n_steps) end_test(NULL);
            else begin_step();
        }
        break;
    case IDLE:
        break;
    }
}
