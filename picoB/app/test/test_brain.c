// Host test for picoB/app/brain.c: the greeting and version lock, MOTORS and
// DRIVE, the safety stops (no DRIVE, drive faults) reported once, repeated MOTORS
// requests not undoing a stop, no IMU, and the report rates, against a fake link
// with the test playing PicoA and a fake `drive`. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoB/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_brain picoB/app/test/test_brain.c && build/test_brain
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../brain.c"

// Fake drive and odometry: what brain.c uses of them.
static bool drive_on;
static drive_fault_t fault_now;
static float drive_v, drive_w;
static odom_t odom;
void drive_enable(bool on) { drive_on = on; if (on) fault_now = DRIVE_OK; }
bool drive_enabled(void) { return drive_on; }
void drive_set(float v_mps, float w_radps) { drive_v = v_mps; drive_w = w_radps; }
drive_fault_t drive_fault(void) { return fault_now; }
const odom_t *odom_get(void) { return &odom; }

// What PicoB sent since the last reset.
static int sent[16];
static odom_report_t last_odom;
static char last_log[LINK_MAX_BODY + 1];
static hello_msg_t last_hello;
static int hello_replies;
static uint8_t request;

static void reset_counts(void) { memset(sent, 0, sizeof sent); hello_replies = 0; }

static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        brain_update();
        for (int k = 0; k < fake_sent_n; k++) {
            const link_msg_t *m = &fake_sent[k];
            sent[m->type]++;
            if (m->type == MSG_ODOM) memcpy(&last_odom, m->body, sizeof last_odom);
            if (m->type == MSG_HELLO) { memcpy(&last_hello, m->body, sizeof last_hello); hello_replies += last_hello.is_reply; }
            if (m->type == MSG_LOG) { memcpy(last_log, m->body, m->len); last_log[m->len] = '\0'; }
        }
        fake_sent_n = 0;
    }
}

static void a_hello(uint8_t version, bool is_reply) {
    hello_msg_t h = {.version = version, .is_reply = is_reply};
    fake_link_deliver(MSG_HELLO, &h, sizeof h);
}
static void a_motors(bool on, uint8_t req) {
    motors_msg_t m = {.on = on, .request = req};
    fake_link_deliver(MSG_MOTORS, &m, sizeof m);
}
static void a_drive(float v, float w) {
    drive_msg_t d = {.v_mps = v, .w_radps = w};
    fake_link_deliver(MSG_DRIVE, &d, sizeof d);
}
// PicoA keeps driving: DRIVE every 50 ms for `ms`.
static void run_driving(int ms, float v) {
    for (int t = 0; t < ms; t += 50) { a_drive(v, 0); run(50); }
}

int main(void) {
    fake_now_us = 1000;
    brain_init(true);

    // Before PicoA greets: HELLO once a second, ODOM at 50 Hz, STATUS at 2 Hz; MOTORS ignored.
    a_motors(true, ++request);
    run(2000);
    assert(sent[MSG_HELLO] == 2 && hello_replies == 0);
    assert(sent[MSG_ODOM] >= 99 && sent[MSG_ODOM] <= 101);
    assert(sent[MSG_STATUS] >= 4 && sent[MSG_STATUS] <= 5);
    assert(!drive_on && last_odom.motors_request == 0);

    // The wrong protocol version: answered, but the motors stay off.
    reset_counts();
    a_hello(LINK_PROTOCOL_VERSION + 1, false);
    a_motors(true, ++request);
    run(10);
    assert(hello_replies == 1 && strstr(last_log, "motors stay off"));
    assert(!drive_on);

    // The right version: greeted, stops its own HELLOs; MOTORS on is applied and echoed.
    a_hello(LINK_PROTOCOL_VERSION, false);
    run(10);
    reset_counts();
    run(2000);
    assert(sent[MSG_HELLO] == 0);
    a_motors(true, ++request);
    run(30);
    assert(drive_on && last_odom.motors_on && last_odom.motors_request == request);
    assert(strcmp(last_log, "Motors on") == 0);

    // DRIVE is applied; without it for 250 ms the motors go off, reported once.
    run_driving(500, 0.1f);
    assert(drive_on && drive_v == 0.1f);
    reset_counts();
    run(LINK_DRIVE_TIMEOUT_US / 1000 + 10);
    assert(!drive_on && last_odom.stop_reason == STOP_NO_DRIVE && sent[MSG_LOG] == 1);

    // A drive fault (tilt): motors off, reported once with the matching stop reason.
    a_motors(true, ++request);
    run_driving(200, 0.1f);
    assert(drive_on && last_odom.stop_reason == STOP_NONE);
    drive_on = false;
    fault_now = DRIVE_TILTED;            // what drive_update() does
    reset_counts();
    run_driving(500, 0.1f);
    assert(last_odom.stop_reason == STOP_TILT && sent[MSG_LOG] == 1);
    assert(strcmp(last_log, "Motors off: tilted too far") == 0);

    // A repeat of the request that was already acted on (PicoA hadn't seen our
    // ODOM yet) doesn't undo the stop.
    a_motors(true, request);
    run_driving(200, 0.1f);
    assert(!drive_on && last_odom.stop_reason == STOP_TILT);

    // MOTORS off after the stop: clears the reason, and the old fault isn't reported again.
    a_motors(false, ++request);
    reset_counts();
    run(500);
    assert(!drive_on && last_odom.stop_reason == STOP_NONE && sent[MSG_LOG] == 1);
    assert(strcmp(last_log, "Motors off") == 0);

    // A wheel fault after switching on again maps to its own stop reason.
    a_motors(true, ++request);
    run_driving(200, 0.1f);
    drive_on = false;
    fault_now = DRIVE_RIGHT_NOT_FOLLOWING;
    run_driving(100, 0.1f);
    assert(last_odom.stop_reason == STOP_RIGHT_WHEELS);

    // PicoA restarts: its request numbers start again at 1, and that's not ignored.
    a_hello(LINK_PROTOCOL_VERSION, false);
    request = 0;
    a_motors(true, ++request);
    run_driving(100, 0.1f);
    assert(drive_on && last_odom.motors_request == 1);

    // No IMU: MOTORS on is acknowledged but refused.
    a_motors(false, ++request);
    run(10);
    imu_ok = false;
    a_motors(true, ++request);
    run_driving(100, 0.1f);
    assert(!drive_on && last_odom.motors_request == request && last_odom.imu_error);
    assert(strcmp(last_log, "No IMU: motors stay off") == 0);

    printf("OK: brain greets, applies MOTORS and DRIVE, stops safely and reports each stop once\n");
    return 0;
}
