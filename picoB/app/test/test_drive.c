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
static float rear_left_gain = 1; // on top of left_gain: 0 = the rear-left motor is stuck
static odom_t odom;

// One 10 ms step: the controller runs, then the simulated robot responds.
static void step(void) {
    fake_now_us += (uint64_t)(DT_S * 1e6f);
    drive_update(&odom);
    float k = DT_S / (MOTOR_TAU_S + DT_S);
    wheel_l += k * (fake_left_power * left_gain * FULL_SPEED - wheel_l);
    wheel_r += k * (fake_right_power * right_gain * FULL_SPEED - wheel_r);
    // Each side's wheels roll together, except a stuck rear-left wheel, which skids.
    odom.wheel_mps[ENC_LEFT_FRONT] = wheel_l;
    odom.wheel_mps[ENC_LEFT_REAR] = wheel_l * rear_left_gain;
    odom.wheel_mps[ENC_RIGHT_FRONT] = odom.wheel_mps[ENC_RIGHT_REAR] = wheel_r;
    odom.wheel_left_mps = 0.5f * (odom.wheel_mps[ENC_LEFT_FRONT] + odom.wheel_mps[ENC_LEFT_REAR]);
    odom.wheel_right_mps = wheel_r;
    odom.v_mps = 0.5f * (wheel_l + wheel_r);
    odom.w_radps = (wheel_r - wheel_l) / (TRACK_M * SKID);
}

static void run(float seconds) { for (int i = 0; i < (int)(seconds / DT_S + 0.5f); i++) step(); }

static void check_ramp(float seconds) {
    for (int i = 0; i < (int)(seconds / DT_S + 0.5f); i++) {
        float v0 = v_mps, w0 = w_radps, l0 = fake_left_power, r0 = fake_right_power;
        step();
        assert(fabsf(v_mps - v0) <= ACCEL_MPS2 * DT_S + 1e-6f);
        assert(fabsf(w_radps - w0) <= TURN_ACCEL_RADPS2 * DT_S + 1e-6f);
        assert(fabsf(fake_left_power - l0) <= 0.12f);   // no power jumps
        assert(fabsf(fake_right_power - r0) <= 0.12f);
    }
}

int main(void) {
    drive_init();
    fake_now_us = FAKE_CLOCK_WRAP_US - 7000000; // the clock wraps during the test
    drive_update(&odom);
    assert(fake_left_power == 0 && !fake_standby_off);     // starts off

    // Straight at 0.1 m/s: ramps up (0.1 m/s after 0.25 s), then holds the speed.
    drive_enable(true);
    drive_set(0.1f, 0);
    check_ramp(0.25f);
    assert(fabsf(v_mps - 0.1f) < 1e-6f);
    check_ramp(1.75f);
    assert(fabsf(wheel_l - 0.1f) < 0.005f && fabsf(wheel_r - 0.1f) < 0.005f);
    printf("straight: wheels %.3f / %.3f m/s (target 0.100), powers %.2f / %.2f\n",
           (double)wheel_l, (double)wheel_r, (double)fake_left_power, (double)fake_right_power);

    // Stop: ramps down, then coasts with zero power.
    drive_set(0, 0);
    check_ramp(0.25f);
    run(0.1f);
    assert(v_mps == 0 && fake_left_power == 0 && fake_right_power == 0);

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

    // A stuck rear-left wheel (or its encoder unplugged) while the front-left one
    // turns: the side's average is still half the target, but the wheel is caught.
    rear_left_gain = 0;
    drive_set(0.1f, 0);
    steps = 0;
    while (drive_enabled() && steps < 300) { step(); steps++; }
    assert(!drive_enabled() && drive_fault() == DRIVE_LEFT_NOT_FOLLOWING);
    printf("stuck rear-left wheel: motors off after %.2f s\n", (double)(steps * DT_S));
    rear_left_gain = 1;
    wheel_l = wheel_r = 0;
    drive_enable(true);

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

    // Tilted past 15° (lifted, tipping over): motors off at once; 10° is fine.
    drive_set(0.1f, 0);
    run(1);
    odom.roll_rad = -0.17f;
    run(0.5f);
    assert(drive_enabled());
    odom.pitch_rad = 0.27f;
    step();
    assert(!drive_enabled() && drive_fault() == DRIVE_TILTED && fake_left_power == 0);
    printf("tilted 15.5 deg: motors off\n");
    odom.pitch_rad = odom.roll_rad = 0;
    wheel_l = wheel_r = 0;
    drive_enable(true);
    assert(drive_fault() == DRIVE_OK);

    // Commands are limited to safe speeds.
    drive_set(5, -9);
    assert(v_target_mps == MAX_V_MPS && w_target_radps == -MAX_W_RADPS);

    // Switching off stops at once, no ramp.
    run(0.5f);
    drive_enable(false);
    assert(fake_left_power == 0 && fake_right_power == 0 && !fake_standby_off);
    drive_update(&odom);
    assert(fake_left_power == 0);
    printf("OK: drive ramps, limits, controls speed and turn rate, stops\n");
    return 0;
}
