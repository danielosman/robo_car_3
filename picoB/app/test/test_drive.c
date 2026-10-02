// Host test for picoB/app/drive.c: ramping, speed limits, wheel speed control,
// gyro turn trim and stopping, against a simple simulated robot. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoB/app/test/stubs -o build/test_drive picoB/app/test/test_drive.c -lm && build/test_drive
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../drive.c"

#define DT_S        0.01f
#define FULL_SPEED  0.25f        // m/s at full power
#define MOTOR_TAU_S 0.05f
#define SKID        1.6f         // turns are 1.6x slower than the track width says

static float wheel_l, wheel_r;   // simulated wheel speeds
static float left_gain = 1, right_gain = 1;
static odom_t odom;

// One 10 ms step: the controller runs, then the simulated robot responds.
static void step(void) {
    fake_now_us += (uint64_t)(DT_S * 1e6f);
    drive_update(&odom);
    float k = DT_S / (MOTOR_TAU_S + DT_S);
    wheel_l += k * (fake_left_power * left_gain * FULL_SPEED - wheel_l);
    wheel_r += k * (fake_right_power * right_gain * FULL_SPEED - wheel_r);
    odom.wheel_left_mps = wheel_l;
    odom.wheel_right_mps = wheel_r;
    odom.v_mps = 0.5f * (wheel_l + wheel_r);
    odom.w_radps = (wheel_r - wheel_l) / (TRACK_M * SKID);
}

static void run(float seconds) { for (int i = 0; i < (int)(seconds / DT_S + 0.5f); i++) step(); }

static void check_ramp(float seconds) {
    for (int i = 0; i < (int)(seconds / DT_S + 0.5f); i++) {
        float v0 = v, w0 = w, l0 = fake_left_power, r0 = fake_right_power;
        step();
        assert(fabsf(v - v0) <= ACCEL_MPS2 * DT_S + 1e-6f);
        assert(fabsf(w - w0) <= TURN_ACCEL_RADPS2 * DT_S + 1e-6f);
        assert(fabsf(fake_left_power - l0) <= 0.12f);   // no power jumps
        assert(fabsf(fake_right_power - r0) <= 0.12f);
    }
}

int main(void) {
    drive_init();
    fake_now_us = 1000000;
    drive_update(&odom);
    assert(fake_left_power == 0 && !fake_standby_off);     // starts off

    // Straight at 0.1 m/s: ramps up (0.1 m/s after 0.25 s), then holds the speed.
    drive_enable(true);
    drive_set(0.1f, 0);
    check_ramp(0.25f);
    assert(fabsf(v - 0.1f) < 1e-6f);
    check_ramp(1.75f);
    assert(fabsf(wheel_l - 0.1f) < 0.005f && fabsf(wheel_r - 0.1f) < 0.005f);
    printf("straight: wheels %.3f / %.3f m/s (target 0.100), powers %.2f / %.2f\n",
           (double)wheel_l, (double)wheel_r, (double)fake_left_power, (double)fake_right_power);

    // Stop: ramps down, then coasts with zero power.
    drive_set(0, 0);
    check_ramp(0.25f);
    run(0.1f);
    assert(v == 0 && fake_left_power == 0 && fake_right_power == 0);

    // A weaker left motor: the wheel loops and the gyro trim keep it straight.
    left_gain = 0.8f;
    drive_set(0.1f, 0);
    run(2);
    assert(fabsf(odom.w_radps) < 0.02f && fabsf(wheel_l - 0.1f) < 0.01f);
    printf("weak left motor: turn rate %.3f rad/s (target 0), wheels %.3f / %.3f\n",
           (double)odom.w_radps, (double)wheel_l, (double)wheel_r);
    left_gain = 1;
    drive_set(0, 0);
    run(1);

    // Turning in place with skid: the gyro trim brings the real turn rate to the command.
    drive_set(0, 0.5f);
    check_ramp(0.2f);
    run(2.8f);
    assert(fabsf(odom.w_radps - 0.5f) < 0.025f);
    printf("turn with %.1fx skid: %.3f rad/s (target 0.500; untrimmed it would be %.3f)\n",
           (double)SKID, (double)odom.w_radps, (double)(0.5f / SKID));

    assert(drive_fault() == DRIVE_OK);   // no false stall alarms above

    // A jammed left wheel (or a missing encoder signal): stopped within ~1 s, not run away.
    drive_set(0, 0);
    run(1);
    left_gain = 0;
    drive_set(0.1f, 0);
    int steps = 0;
    while (drive_enabled() && steps < 300) { step(); steps++; }
    assert(!drive_enabled() && drive_fault() == DRIVE_LEFT_NOT_FOLLOWING);
    assert(fake_left_power == 0 && fake_right_power == 0 && !fake_standby_off);
    printf("jammed left wheel: motors off after %.2f s\n", (double)(steps * DT_S));
    left_gain = 1;
    wheel_l = wheel_r = 0;
    drive_enable(true);                   // switching on again clears the fault
    assert(drive_fault() == DRIVE_OK);

    // Reversed encoders (what the first square test on the robot found): the speed
    // loop runs away the wrong way; caught within ~1 s.
    left_gain = right_gain = -1;
    drive_set(0.1f, 0);
    steps = 0;
    while (drive_enabled() && steps < 300) { step(); steps++; }
    assert(!drive_enabled() && drive_fault() != DRIVE_OK);
    printf("reversed encoders: motors off after %.2f s\n", (double)(steps * DT_S));
    left_gain = right_gain = 1;
    wheel_l = wheel_r = 0;
    drive_enable(true);

    // Commands are limited to safe speeds.
    drive_set(5, -9);
    assert(v_target == MAX_V_MPS && w_target == -MAX_W_RADPS);

    // Switching off stops at once, no ramp.
    run(0.5f);
    drive_enable(false);
    assert(fake_left_power == 0 && fake_right_power == 0 && !fake_standby_off);
    drive_update(&odom);
    assert(fake_left_power == 0);
    printf("OK: drive ramps, limits, controls speed and turn rate, stops\n");
    return 0;
}
