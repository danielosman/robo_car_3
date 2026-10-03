#include <math.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "units.h"
#include "drive.h"

#define TRACK_M          0.225f // wheel centre to wheel centre; skid steering needs more (gyro trim covers it, M1 measures it)
#define MAX_V_MPS        0.20f  // full power is ~0.24 m/s on the battery; leave control headroom
#define MAX_W_RADPS      1.5f
#define ACCEL_MPS2       0.4f
#define TURN_ACCEL_RADPS2 3.0f

#define POWER_PER_MPS    4.0f   // feedforward: full power ~ 0.25 m/s
#define WHEEL_KP         2.0f   // power per m/s of wheel speed error
#define WHEEL_KI         8.0f   // power per metre of accumulated wheel error
#define WHEEL_I_MAX      0.5f   // integral term limit, in power
#define TURN_KI          0.5f   // m/s of wheel speed difference per rad of accumulated turn-rate error
#define TURN_TRIM_MAX    0.1f   // m/s
#define FOLLOW_MIN_MPS   0.02f  // a wheel is judged only when asked for at least this
#define MAX_TILT_RAD     (DRIVE_MAX_TILT_DEG * RAD_PER_DEG)

static bool enabled;
static float v_target_mps, w_target_radps; // commanded
static float v_mps, w_radps;               // ramped toward the command
static float left_integral_m, right_integral_m; // wheel speed error, integrated
static float turn_trim_mps;                // wheel speed difference added to correct the turn rate
static float left_lag_s, right_lag_s;
static drive_fault_t fault;
static uint64_t last_us;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static float approach(float x, float target, float step) {
    return x < target ? fminf(x + step, target) : fmaxf(x - step, target);
}

static void reset_control(void) {
    v_mps = w_radps = 0;
    left_integral_m = right_integral_m = turn_trim_mps = 0;
    left_lag_s = right_lag_s = 0;
}

void drive_init(void) {
    motor_init();
    reset_control();
}

void drive_enable(bool on) {
    if (on == enabled) return;
    enabled = on;
    if (on) fault = DRIVE_OK;
    v_target_mps = w_target_radps = 0;
    reset_control();
    motor_enable(on); // off also zeroes the outputs
    last_us = time_us_64();
}

bool drive_enabled(void) { return enabled; }

drive_fault_t drive_fault(void) { return fault; }

void drive_set(float v_cmd_mps, float w_cmd_radps) {
    v_target_mps = clampf(v_cmd_mps, -MAX_V_MPS, MAX_V_MPS);
    w_target_radps = clampf(w_cmd_radps, -MAX_W_RADPS, MAX_W_RADPS);
}

// Feedforward plus PI on one side's wheel speed; returns motor power.
static float wheel_power(float *integral_m, float target_mps, float measured_mps, float dt) {
    float err_mps = target_mps - measured_mps;
    *integral_m = clampf(*integral_m + err_mps * dt, -WHEEL_I_MAX / WHEEL_KI, WHEEL_I_MAX / WHEEL_KI);
    return POWER_PER_MPS * target_mps + WHEEL_KP * err_mps + WHEEL_KI * *integral_m;
}

// Counts how long a wheel hasn't followed its target: stopped, far too slow, or
// turning the wrong way (a reversed encoder makes the speed loop run away).
static bool not_following(float *seconds, float target_mps, float measured_mps, float dt) {
    bool lagging = fabsf(target_mps) >= FOLLOW_MIN_MPS &&
                   measured_mps * copysignf(1, target_mps) < 0.25f * fabsf(target_mps);
    *seconds = lagging ? *seconds + dt : 0;
    return *seconds >= DRIVE_FOLLOW_TIME_S;
}

void drive_update(const odom_t *odom) {
    uint64_t now = time_us_64();
    float dt = fminf((float)(now - last_us) * 1e-6f, 0.05f);
    last_us = now;
    if (!enabled) return;
    if (fabsf(odom->pitch_rad) > MAX_TILT_RAD || fabsf(odom->roll_rad) > MAX_TILT_RAD) {
        fault = DRIVE_TILTED;
        drive_enable(false);
        return;
    }

    v_mps = approach(v_mps, v_target_mps, ACCEL_MPS2 * dt);
    w_radps = approach(w_radps, w_target_radps, TURN_ACCEL_RADPS2 * dt);
    if (v_mps == 0 && w_radps == 0) { // at rest: coast, and start the next move fresh
        reset_control();
        motor_set(0, 0);
        return;
    }
    // Keeps the measured turn rate on the command; with w = 0 it also keeps the robot straight.
    turn_trim_mps = clampf(turn_trim_mps + TURN_KI * (w_radps - odom->w_radps) * dt, -TURN_TRIM_MAX, TURN_TRIM_MAX);
    float half_diff_mps = w_radps * TRACK_M * 0.5f + turn_trim_mps;
    float left_target_mps = v_mps - half_diff_mps, right_target_mps = v_mps + half_diff_mps;
    bool left_bad = not_following(&left_lag_s, left_target_mps, odom->wheel_left_mps, dt);
    bool right_bad = not_following(&right_lag_s, right_target_mps, odom->wheel_right_mps, dt);
    if (left_bad || right_bad) {
        fault = left_bad ? DRIVE_LEFT_NOT_FOLLOWING : DRIVE_RIGHT_NOT_FOLLOWING;
        drive_enable(false);
        return;
    }
    motor_set(wheel_power(&left_integral_m, left_target_mps, odom->wheel_left_mps, dt),
              wheel_power(&right_integral_m, right_target_mps, odom->wheel_right_mps, dt));
}
