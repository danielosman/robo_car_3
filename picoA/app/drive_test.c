#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "body.h"
#include "drive_test.h"

#define SIDE_M        0.50f
#define SPEED_MPS     0.10f
#define MIN_SPEED_MPS 0.03f
#define TURN_RADPS    0.50f
#define MIN_TURN_RADPS 0.15f
#define QUARTER_TURN  1.5707963f
#define LEGS          8          // forward, turn, forward, turn, ...
#define PAUSE_US      700000     // stand still between legs (PicoB re-measures gyro bias)
#define PAUSE_MAX_US  3000000
#define MOTORS_ON_TIMEOUT_US 1000000
#define RAD_TO_DEG    57.29578f

static enum { IDLE, WAITING_FOR_MOTORS, MOVING, PAUSED } state;
static int leg;
static float start_x, start_y, start_yaw;
static float leg_x, leg_y, leg_yaw;
static absolute_time_t deadline, pause_min_end;

static void stop(const char *why) {
    body_drive(0, 0);
    if (state != IDLE && why) printf("Square test stopped: %s\n", why);
    state = IDLE;
}

static void begin_leg(const odom_report_t *o) {
    leg_x = o->x_m;
    leg_y = o->y_m;
    leg_yaw = o->yaw_rad;
    state = MOVING;
}

static void finish(const odom_report_t *o) {
    // End pose in the frame the robot had at the start: x forward, y left.
    float dx = o->x_m - start_x, dy = o->y_m - start_y;
    float c = cosf(start_yaw), s = sinf(start_yaw);
    printf("Square done. Odometry says the robot is %.1f cm forward, %.1f cm left of the start,\n"
           "having turned %.1f deg (a perfect square: 0 cm, 0 cm, 360 deg).\n"
           "Measure where it really is to see the odometry error.\n",
           (double)((c * dx + s * dy) * 100), (double)((-s * dx + c * dy) * 100),
           (double)((o->yaw_rad - start_yaw) * RAD_TO_DEG)); // printf takes doubles
    stop(NULL);
}

// Drives the current leg; returns true when it's complete. Slows down near the
// end so the robot stops close to the target.
static bool drive_leg(const odom_report_t *o) {
    if (leg % 2 == 0) {
        float remaining = SIDE_M - hypotf(o->x_m - leg_x, o->y_m - leg_y);
        if (remaining <= 0) return true;
        body_drive(fminf(SPEED_MPS, fmaxf(MIN_SPEED_MPS, remaining)), 0);
    } else {
        float remaining = QUARTER_TURN - (o->yaw_rad - leg_yaw);
        if (remaining <= 0) return true;
        body_drive(0, fminf(TURN_RADPS, fmaxf(MIN_TURN_RADPS, 2 * remaining)));
    }
    return false;
}

void drive_test_start(void) {
    body_drive(0, 0);
    body_motors(true);
    state = WAITING_FOR_MOTORS;
    deadline = make_timeout_time_us(MOTORS_ON_TIMEOUT_US);
    printf("Square test: 50 cm sides, turning left\n");
}

void drive_test_stop(void) { stop("by command"); }

bool drive_test_running(void) { return state != IDLE; }

void drive_test_update(void) {
    if (state == IDLE) return;
    const odom_report_t *o = body_odom();
    if (!body_connected()) { stop("PicoB not connected"); return; }
    if (state != WAITING_FOR_MOTORS && !o->motors_on) { stop("PicoB switched the motors off"); return; }

    switch (state) {
    case WAITING_FOR_MOTORS:
        if (o->motors_on) {
            start_x = o->x_m;
            start_y = o->y_m;
            start_yaw = o->yaw_rad;
            leg = 0;
            begin_leg(o);
        } else if (time_reached(deadline)) {
            stop("motors didn't switch on");
        }
        break;
    case MOVING:
        if (drive_leg(o)) {
            body_drive(0, 0);
            state = PAUSED;
            pause_min_end = make_timeout_time_us(PAUSE_US);
            deadline = make_timeout_time_us(PAUSE_MAX_US);
        }
        break;
    case PAUSED:
        if ((time_reached(pause_min_end) && o->stationary) || time_reached(deadline)) {
            if (++leg == LEGS) finish(o);
            else begin_leg(o);
        }
        break;
    case IDLE:
        break;
    }
}
