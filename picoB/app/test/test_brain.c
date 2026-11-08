// Host test for picoB/app/brain.c: the greeting and version lock, MOTORS and
// DRIVE, the safety stops (no DRIVE, drive faults) reported once, repeated MOTORS
// requests not undoing a stop, no IMU, the report rates, and the imu records (each
// sent as made, in order, only once greeted; the odom report without the peaks),
// against a fake link with the test playing PicoA and a fake `drive`. Run from the repo root:
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
// Odometry's imu records: one made every 10 ms of run(), numbered in accel[0].
static imu_record_t imu_made[64];
static int n_made, n_taken;
uint32_t odom_imu_lost(void) { return 7; }
uint32_t odom_imu_skipped(void) { return 5; }
bool odom_imu_take(imu_record_t *r) {
    if (n_taken == n_made) return false;
    *r = imu_made[n_taken++ % 64];
    return true;
}
static void make_imu(void) {
    assert(n_made - n_taken < 64);
    imu_record_t *r = &imu_made[n_made % 64];
    r->t_us = time_us_32();
    r->accel[0] = (int16_t)n_made; r->accel[1] = -1; r->accel[2] = 16384;
    r->gyro[0] = -32768; r->gyro[1] = 32767; r->gyro[2] = (int16_t)-n_made;
    n_made++;
}

// What PicoB sent since the last reset.
static int sent[16];
static odom_report_t last_odom;
static char last_log[LINK_MAX_BODY + 1];
static hello_msg_t last_hello;
static imu_record_t last_imu;
static status_report_t last_status;
static int imu_out_of_order; // a record sent not right after the one before
static int hello_replies;
static uint8_t request;

static void reset_counts(void) { memset(sent, 0, sizeof sent); hello_replies = 0; }

static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        static int ms_run;
        if (++ms_run % 10 == 0) make_imu();
        brain_update();
        for (int k = 0; k < fake_sent_n; k++) {
            const link_msg_t *m = &fake_sent[k];
            sent[m->type]++;
            if (m->type == MSG_ODOM) memcpy(&last_odom, m->body, sizeof last_odom);
            if (m->type == MSG_STATUS) memcpy(&last_status, m->body, sizeof last_status);
            if (m->type == MSG_HELLO) { memcpy(&last_hello, m->body, sizeof last_hello); hello_replies += last_hello.is_reply; }
            if (m->type == MSG_LOG) { memcpy(last_log, m->body, m->len); last_log[m->len] = '\0'; }
            if (m->type == MSG_IMU) {
                assert(m->len == sizeof(imu_record_t));
                int16_t before = last_imu.accel[0];
                memcpy(&last_imu, m->body, sizeof last_imu);
                if (sent[MSG_IMU] > 1 && last_imu.accel[0] != (int16_t)(before + 1)) imu_out_of_order++;
            }
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
    fake_now_us = FAKE_CLOCK_WRAP_US - 3000000; // the clock wraps during the test
    brain_init(true);

    // The odom report lost the peaks (jolt and tilt, 16 B); an imu record is 16 B.
    assert(sizeof(odom_report_t) == 45 && sizeof(imu_record_t) == 16);

    // Before PicoA greets: HELLO once a second, ODOM at 50 Hz, STATUS at 2 Hz; MOTORS
    // ignored; the imu records are taken (none pile up) but not sent.
    a_motors(true, ++request);
    run(2000);
    assert(sent[MSG_HELLO] == 2 && hello_replies == 0);
    assert(sent[MSG_ODOM] >= 99 && sent[MSG_ODOM] <= 101);
    assert(sent[MSG_STATUS] >= 4 && sent[MSG_STATUS] <= 5);
    assert(last_status.imu_lost == 7); // odometry's queue overflow count reaches PicoA
    assert(last_status.imu_skipped == 5); // and the records dropped after a skipped sample
    assert(!drive_on && last_odom.motors_request == 0);
    assert(sent[MSG_IMU] == 0 && n_made == 200 && n_taken == n_made);

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
    // Greeted: each imu record goes out in the loop it was made, unchanged, in order.
    assert(sent[MSG_IMU] == 200 && imu_out_of_order == 0 && n_taken == n_made);
    assert(last_imu.accel[0] == (int16_t)(n_made - 1) && last_imu.accel[2] == 16384 && last_imu.gyro[0] == -32768 &&
           last_imu.gyro[1] == 32767 && last_imu.gyro[2] == (int16_t)-(n_made - 1) && last_imu.t_us == imu_made[(n_made - 1) % 64].t_us);
    // Several made in one loop (a long iteration): all sent, oldest first.
    reset_counts();
    make_imu(); make_imu(); make_imu();
    run(1);
    assert(sent[MSG_IMU] == 3 && imu_out_of_order == 0 && last_imu.accel[0] == (int16_t)(n_made - 1));
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

    // The wrong version again: no more imu records (PicoA can't take them).
    a_hello(LINK_PROTOCOL_VERSION + 1, false);
    run(10);
    reset_counts();
    run(500);
    assert(sent[MSG_IMU] == 0 && n_taken == n_made);

    printf("OK: brain greets, applies MOTORS and DRIVE, stops safely and reports each stop once, sends the imu records once greeted\n");
    return 0;
}
