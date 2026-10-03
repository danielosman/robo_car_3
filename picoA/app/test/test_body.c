// Host test for picoA/app/body.c: the greeting and version check, repeating
// DRIVE, re-sending MOTORS until PicoB has acted on it (also after a safety stop),
// respecting PicoB's safety stops, PicoB restarting, and the connection timeout,
// against a fake link with the test playing PicoB. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -o build/test_body picoA/app/test/test_body.c && build/test_body
#include <assert.h>
#include <stdio.h>
#include "../body.c"

// The fake PicoB.
static bool b_alive;             // sends ODOM and STATUS
static bool b_hears = true;      // false: MOTORS messages are lost on the way
static bool b_has_imu = true;
static odom_report_t b_odom;
static status_report_t b_status;
static int sent[16];             // messages PicoA sent, by type, since the last reset
static motors_msg_t last_motors;
static drive_msg_t last_drive;
static hello_msg_t last_hello;

static void reset_counts(void) { memset(sent, 0, sizeof sent); }

static void b_hello(uint8_t version, bool is_reply) {
    hello_msg_t h = {.version = version, .is_reply = is_reply};
    fake_link_deliver(MSG_HELLO, &h, sizeof h);
}

// Runs PicoA for `ms` milliseconds, 1 ms per loop iteration, with PicoB answering.
static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        body_update();
        for (int k = 0; k < fake_sent_n; k++) {
            const link_msg_t *m = &fake_sent[k];
            sent[m->type]++;
            if (m->type == MSG_MOTORS) {
                memcpy(&last_motors, m->body, sizeof last_motors);
                // PicoB acts on each request once (see picoB/app/brain.c).
                if (b_hears && last_motors.request != b_odom.motors_request) {
                    b_odom.motors_request = last_motors.request;
                    if (b_has_imu) { b_odom.motors_on = last_motors.on; b_odom.stop_reason = STOP_NONE; }
                }
            }
            if (m->type == MSG_DRIVE) memcpy(&last_drive, m->body, sizeof last_drive);
            if (m->type == MSG_HELLO) memcpy(&last_hello, m->body, sizeof last_hello);
        }
        fake_sent_n = 0;
        if (b_alive && fake_now_us % LINK_ODOM_PERIOD_US < 1000)
            fake_link_deliver(MSG_ODOM, &b_odom, sizeof b_odom);
        if (b_alive && fake_now_us % LINK_STATUS_PERIOD_US < 1000)
            fake_link_deliver(MSG_STATUS, &b_status, sizeof b_status);
    }
}

int main(void) {
    fake_now_us = 1000;
    body_init();

    // Until PicoB answers: HELLO once a second, nothing else.
    run(2500);
    assert(sent[MSG_HELLO] == 3 && !last_hello.is_reply);
    assert(sent[MSG_DRIVE] == 0 && sent[MSG_MOTORS] == 0 && !body_connected());

    // PicoB with the wrong protocol version: not greeted, still no DRIVE.
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION + 1, false);
    run(500);
    assert(sent[MSG_DRIVE] == 0 && !body_connected());

    // The right version: PicoA replies; connected once reports arrive.
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION, false);
    run(10);
    assert(sent[MSG_HELLO] == 1 && last_hello.is_reply);
    assert(!body_connected());
    b_alive = true;
    run(100);
    assert(body_connected());

    // DRIVE repeats often enough for PicoB's 250 ms timeout, with the latest command.
    body_drive(0.1f, -0.2f);
    reset_counts();
    run(1000);
    assert(sent[MSG_DRIVE] >= 1000000 / LINK_DRIVE_TIMEOUT_US * 4);
    assert(last_drive.v_mps == 0.1f && last_drive.w_radps == -0.2f);

    // Motors on: re-sent while PicoB hasn't acted on it (a lost message), then quiet.
    b_hears = false;
    reset_counts();
    body_motors(true);
    run(1000);
    assert(sent[MSG_MOTORS] >= 4 && last_motors.on);
    b_hears = true;
    run(300);
    assert(b_odom.motors_on);
    reset_counts();
    run(1000);
    assert(sent[MSG_MOTORS] == 0);

    // A safety stop on PicoB: PicoA doesn't switch the motors back on, and the drive command is cleared.
    b_odom.motors_on = false;
    b_odom.stop_reason = STOP_TILT;
    reset_counts();
    run(1000);
    assert(sent[MSG_MOTORS] == 0 && !b_odom.motors_on);
    assert(last_drive.v_mps == 0 && last_drive.w_radps == 0);
    // Only an explicit request switches them on again, even if its first MOTORS
    // is lost: the stale "off + stop reason" reports must not cancel it.
    b_hears = false;
    body_motors(true);
    run(150);
    b_hears = true;
    run(300);
    assert(b_odom.motors_on && b_odom.stop_reason == STOP_NONE);

    // PicoB restarts: reports stop during its ~1 s gyro calibration, then it
    // greets again with the motors off and no request seen. Motors restored.
    b_alive = false;
    run(1200);
    assert(!body_connected());
    b_odom = (odom_report_t){0};
    b_alive = true;
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION, false);
    run(500);
    assert(sent[MSG_HELLO] == 1 && last_hello.is_reply);
    assert(body_connected() && b_odom.motors_on);

    // No IMU on PicoB: it acts on the request but refuses, and PicoA stops asking.
    b_has_imu = false;
    b_odom.motors_on = false;
    b_odom.imu_error = true;
    body_motors(true);
    run(100);
    reset_counts();
    run(1000);
    assert(sent[MSG_MOTORS] == 0 && !b_odom.motors_on);
    b_odom.imu_error = false;
    b_has_imu = true;

    // STATUS and ODOM are decoded.
    b_status.gyro_bias_radps = 0.0123f;
    b_odom.yaw_rad = 1.5f;
    run(600);
    assert(body_status()->gyro_bias_radps == 0.0123f && body_odom()->yaw_rad == 1.5f);

    // PicoB goes quiet: disconnected within 10 report periods.
    b_alive = false;
    run(10 * LINK_ODOM_PERIOD_US / 1000 + 20);
    assert(!body_connected());

    printf("OK: body greets, repeats DRIVE, re-sends MOTORS, respects safety stops, times out\n");
    return 0;
}
