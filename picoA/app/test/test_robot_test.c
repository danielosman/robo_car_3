// Host test for picoA/app/robot_test.c: each test drives a simulated robot the
// right way, ends where it should and reports what it measured; safety stops,
// motors that don't switch on and a lost PicoB end a test, and the result says
// why and what was measured so far. The test provides body.h's functions as the
// simulated robot. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -o build/test_robot_test picoA/app/test/test_robot_test.c -lm && build/test_robot_test
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../robot_test.c"

#define DT_US       1000
#define TRACK_M     0.225f
#define SKID        1.5f      // the wheels travel 1.5x the geometric turn distance
#define LAG_US      30000     // MOTORS reaching PicoB and its ODOM coming back
#define ACCEL_MPS2  0.4f      // PicoB's ramps (picoB/app/drive.c)
#define TURN_ACCEL_RADPS2 3.0f
#define RAD(d)      ((d) * RAD_PER_DEG)
#define DEG(r)      ((r) * DEG_PER_RAD)

// The simulated robot: PicoB's motor state shows up in its reports LAG_US after
// body_motors(); speeds ramp like PicoB's; "stationary" 0.5 s after stopping.
static odom_report_t sim;
static status_report_t sim_status;
static bool connected = true, motors_wanted, motors_refused;
static uint64_t motors_changed_us;
static float cmd_v, cmd_w, v, w, still_s;
static float bias_drift_radps2;  // how fast the gyro bias wanders

bool body_connected(void) { return connected; }
const odom_report_t *body_odom(void) { return &sim; }
const status_report_t *body_status(void) { return &sim_status; }
void body_motors(bool on) {
    motors_wanted = on;
    motors_changed_us = fake_now_us;
    if (!on) cmd_v = cmd_w = 0;
}
void body_drive(float v_mps, float w_radps) { cmd_v = v_mps; cmd_w = w_radps; }

static float approach(float x, float target, float step) {
    return x < target ? fminf(x + step, target) : fmaxf(x - step, target);
}

static void tick(void) {
    float dt = DT_US * 1e-6f;
    fake_now_us += DT_US;
    if (fake_now_us - motors_changed_us >= LAG_US) sim.motors_on = motors_wanted && !motors_refused;
    v = sim.motors_on ? approach(v, cmd_v, ACCEL_MPS2 * dt) : 0;
    w = sim.motors_on ? approach(w, cmd_w, TURN_ACCEL_RADPS2 * dt) : 0;
    sim.x_m += v * cosf(sim.yaw_rad) * dt;
    sim.y_m += v * sinf(sim.yaw_rad) * dt;
    sim.yaw_rad += w * dt;
    sim.wheel_left_m += (v - w * TRACK_M * SKID / 2) * dt;
    sim.wheel_right_m += (v + w * TRACK_M * SKID / 2) * dt;
    still_s = v == 0 && w == 0 ? still_s + dt : 0;
    sim.stationary = still_s >= 0.5f;
    sim_status.gyro_bias_radps += bias_drift_radps2 * dt;
    robot_test_update();
}

// Runs until the test ends (or `max_s` passes); returns the seconds it took.
static float run_test(float max_s) {
    float s = 0;
    while (robot_test_running() && s < max_s) { tick(); s += DT_US * 1e-6f; }
    return s;
}

static void run_for(float seconds) {
    for (int i = 0; i < (int)(seconds * 1e6f / DT_US); i++) tick();
}

static bool result_has(const char *text) { return strstr(robot_test_result(), text) != NULL; }

int main(void) {
    // Square: back where it started, facing the same way, and it says so.
    robot_test_start(ROBOT_TEST_SQUARE);
    run_test(120);
    assert(!robot_test_running() && motors_wanted);
    assert(hypotf(sim.x_m, sim.y_m) < 0.02f && fabsf(DEG(sim.yaw_rad) - 360) < 3);
    assert(result_has("Square test done.") && result_has("Measure where it really is"));

    // Ten turns: stops at 3600° by the gyro, finds the effective track width.
    float yaw0 = sim.yaw_rad;
    robot_test_start(ROBOT_TEST_TURNS);
    float s = run_test(300);
    assert(!robot_test_running());
    assert(fabsf(DEG(sim.yaw_rad - yaw0) - 3600) < 2);
    assert(s > 110 && s < 150);
    char expected[64];
    snprintf(expected, sizeof expected, "track width on this floor: %.1f cm", (double)(TRACK_M * SKID * 100));
    assert(result_has("Turn test done.") && result_has("(10 full turns)") && result_has("N/36 %"));
    assert(result_has(expected));
    printf("10 turns took %.0f s; %s\n", (double)s, expected);

    // 2 m forward, then 2 m back.
    float x0 = sim.x_m, y0 = sim.y_m;
    robot_test_start(ROBOT_TEST_FORWARD);
    run_test(60);
    assert(fabsf(hypotf(sim.x_m - x0, sim.y_m - y0) - 2.0f) < 0.01f);
    assert(result_has("Forward test done.\nOdometry says 200."));
    robot_test_start(ROBOT_TEST_BACK);
    run_test(60);
    assert(hypotf(sim.x_m - x0, sim.y_m - y0) < 0.02f);
    assert(result_has("Back test done.\nOdometry says -200."));

    // Drift: switches the motors off and reports the bias wandering over 10 minutes.
    bias_drift_radps2 = RAD(0.01f) / 600;   // 0.01 deg/s over the test
    robot_test_start(ROBOT_TEST_DRIFT);
    s = run_test(700);
    assert(!robot_test_running() && !motors_wanted);
    assert(s > 600 && s < 605);
    assert(result_has("Drift test done.") && result_has("+0.0000 to +0.0100 in between"));
    assert(!result_has("robot moved"));
    bias_drift_radps2 = 0;

    // Drift test with the robot nudged halfway: the result says to repeat it.
    robot_test_start(ROBOT_TEST_DRIFT);
    run_for(300);
    body_motors(true); cmd_v = 0.05f;
    run_for(0.2f);
    body_motors(false);
    run_test(400);
    assert(result_has("repeat it without touching"));

    // A safety stop on PicoB ends a driving test; the result keeps why and how far it got.
    robot_test_start(ROBOT_TEST_FORWARD);
    run_for(5);
    assert(robot_test_running() && cmd_v > 0);
    sim.motors_on = false;                   // PicoB stopped by itself...
    sim.stop_reason = STOP_TILT;
    motors_wanted = false;                   // ...and body respects it
    tick();
    assert(!robot_test_running());
    assert(result_has("Forward test stopped after 5 s: PicoB switched the motors off (tilted too far)"));
    assert(result_has("Odometry says 4"));   // ~40-something cm so far
    sim.stop_reason = STOP_NONE;

    // Ten turns cut short by a lost PicoB: partial turns and the wheels are kept.
    robot_test_start(ROBOT_TEST_TURNS);
    run_for(30);
    connected = false;
    tick();
    assert(!robot_test_running() && cmd_w == 0);
    assert(result_has("stopped after 30 s: PicoB not connected") && result_has("(2 full turns)"));
    assert(!result_has("Look at the mark"));
    connected = true;

    // The motors don't switch on (PicoB without an IMU refuses; they were off
    // before): the test gives up after 3 s and says so.
    body_motors(false);
    run_for(0.1f);
    motors_refused = true;
    robot_test_start(ROBOT_TEST_SQUARE);
    s = run_test(10);
    assert(!robot_test_running() && s > 2.9f && s < 3.1f);
    assert(result_has("Square test didn't start: the motors didn't switch on"));
    motors_refused = false;

    // Starting a test while one runs replaces it; stop ends it with the robot standing.
    robot_test_start(ROBOT_TEST_FORWARD);
    run_for(1);
    robot_test_start(ROBOT_TEST_SQUARE);
    assert(robot_test_running());
    run_for(1);
    robot_test_stop();
    assert(!robot_test_running() && cmd_v == 0 && cmd_w == 0);
    assert(result_has("Square test stopped after"));

    printf("OK: robot tests drive the square, turns, straight lines and drift test, and report early stops\n");
    return 0;
}
