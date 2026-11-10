// Host test for picoA/app/body.c: the greeting and version check, repeating
// DRIVE, re-sending MOTORS until PicoB has acted on it (also after a safety stop),
// respecting PicoB's safety stops, PicoB restarting, the connection timeout,
// keeping every report (several can arrive in one loop iteration), and PicoB's imu
// records on PicoA's clock (dropped until the clock is known, both clocks wrapping,
// the newest kept, the lost counted),
// against a fake link with the test playing PicoB. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -o build/test_body picoA/app/test/test_body.c && build/test_body
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../body.c"

// The fake PicoB.
static bool b_alive;             // sends ODOMETRY and STATUS
static bool b_hears = true;      // false: MOTORS messages are lost on the way
static bool b_has_imu = true;
static odometry_report_t b_odometry;
static status_report_t b_status;
static int sent[16];             // messages PicoA sent, by type, since the last reset
static motors_message_t last_motors;
static drive_message_t last_drive;
static hello_message_t last_hello;

static void reset_counts(void) { memset(sent, 0, sizeof sent); }

static void b_hello(uint8_t version, bool is_reply) {
    hello_message_t h = {.version = version, .is_reply = is_reply};
    fake_link_deliver(MESSAGE_HELLO, &h, sizeof h);
}

// Runs PicoA for `ms` milliseconds, 1 ms per loop iteration, with PicoB answering.
static void run(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        body_update();
        for (int k = 0; k < fake_sent_n; k++) {
            const link_message_t *m = &fake_sent[k];
            sent[m->type]++;
            if (m->type == MESSAGE_MOTORS) {
                memcpy(&last_motors, m->body, sizeof last_motors);
                // PicoB acts on each request once (see picoB/app/brain.c).
                if (b_hears && last_motors.request != b_odometry.motors_request) {
                    b_odometry.motors_request = last_motors.request;
                    if (b_has_imu) { b_odometry.motors_on = last_motors.on; b_odometry.stop_reason = STOP_NONE; }
                }
            }
            if (m->type == MESSAGE_DRIVE) memcpy(&last_drive, m->body, sizeof last_drive);
            if (m->type == MESSAGE_HELLO) memcpy(&last_hello, m->body, sizeof last_hello);
        }
        fake_sent_n = 0;
        if (b_alive && fake_now_us % LINK_ODOMETRY_PERIOD_US < 1000)
            fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
        if (b_alive && fake_now_us % LINK_STATUS_PERIOD_US < 1000)
            fake_link_deliver(MESSAGE_STATUS, &b_status, sizeof b_status);
    }
}

int main(void) {
    fake_now_us = FAKE_CLOCK_WRAP_US - 10000000; // the clock wraps during the test
    body_init();

    // Until PicoB answers: HELLO once a second, nothing else.
    run(2500);
    assert(sent[MESSAGE_HELLO] == 3 && !last_hello.is_reply);
    assert(sent[MESSAGE_DRIVE] == 0 && sent[MESSAGE_MOTORS] == 0 && !body_connected());

    // PicoB with the wrong protocol version: not greeted, still no DRIVE.
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION + 1, false);
    run(500);
    assert(sent[MESSAGE_DRIVE] == 0 && !body_connected());

    // The right version: PicoA replies; connected once reports arrive.
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION, false);
    run(10);
    assert(sent[MESSAGE_HELLO] == 1 && last_hello.is_reply);
    assert(!body_connected());
    b_alive = true;
    run(100);
    assert(body_connected());

    // DRIVE repeats often enough for PicoB's 250 ms timeout, with the latest command.
    body_drive(0.1f, -0.2f);
    reset_counts();
    run(1000);
    assert(sent[MESSAGE_DRIVE] >= 1000000 / LINK_DRIVE_TIMEOUT_US * 4);
    assert(last_drive.v_mps == 0.1f && last_drive.w_radps == -0.2f);

    // Motors on: re-sent while PicoB hasn't acted on it (a lost message), then quiet.
    b_hears = false;
    reset_counts();
    body_motors(true);
    run(1000);
    assert(sent[MESSAGE_MOTORS] >= 4 && last_motors.on);
    b_hears = true;
    run(300);
    assert(b_odometry.motors_on);
    reset_counts();
    run(1000);
    assert(sent[MESSAGE_MOTORS] == 0);

    // A safety stop on PicoB: PicoA doesn't switch the motors back on, and the drive command is cleared.
    b_odometry.motors_on = false;
    b_odometry.stop_reason = STOP_TILT;
    reset_counts();
    run(1000);
    assert(sent[MESSAGE_MOTORS] == 0 && !b_odometry.motors_on);
    assert(last_drive.v_mps == 0 && last_drive.w_radps == 0);
    // Only an explicit request switches them on again, even if its first MOTORS
    // is lost: the stale "off + stop reason" reports must not cancel it.
    b_hears = false;
    body_motors(true);
    run(150);
    b_hears = true;
    run(300);
    assert(b_odometry.motors_on && b_odometry.stop_reason == STOP_NONE);

    // PicoB restarts: reports stop during its ~1 s gyro calibration, then it
    // greets again with the motors off and no request seen. Motors restored.
    b_alive = false;
    run(1200);
    assert(!body_connected());
    b_odometry = (odometry_report_t){0};
    b_alive = true;
    reset_counts();
    b_hello(LINK_PROTOCOL_VERSION, false);
    run(500);
    assert(sent[MESSAGE_HELLO] == 1 && last_hello.is_reply);
    assert(body_connected() && b_odometry.motors_on);

    // No IMU on PicoB: it acts on the request but refuses, and PicoA stops asking.
    b_has_imu = false;
    b_odometry.motors_on = false;
    b_odometry.imu_error = true;
    body_motors(true);
    run(100);
    reset_counts();
    run(1000);
    assert(sent[MESSAGE_MOTORS] == 0 && !b_odometry.motors_on);
    b_odometry.imu_error = false;
    b_has_imu = true;

    // STATUS and ODOMETRY are decoded.
    b_status.gyro_bias_radps = 0.0123f;
    b_odometry.yaw_rad = 1.5f;
    run(600);
    assert(body_status()->gyro_bias_radps == 0.0123f && body_odometry()->yaw_rad == 1.5f);

    // PicoB goes quiet: disconnected within 10 report periods.
    b_alive = false;
    run(10 * LINK_ODOMETRY_PERIOD_US / 1000 + 20);
    assert(!body_connected());

    // Clock sync: PicoB's clock is far off (and wraps soon), runs 50 ppm fast, and
    // each report arrives 0-3 ms late on top of its time on the wire. After a few
    // seconds, the report's time on PicoA's clock is within 0.3 ms.
    uint64_t b_start_us = fake_now_us;
    uint32_t b_clock0 = 0xFFF00000u;       // wraps after ~1 s
    srand(1);
    float worst_ms = 0;
    for (int i = 0; i < 500; i++) {        // 10 s of reports
        uint64_t sent_us = fake_now_us;
        b_odometry.t_us = b_clock0 + (uint32_t)((sent_us - b_start_us) * 1000050ull / 1000000ull);
        fake_now_us += ODOMETRY_WIRE_US + (uint64_t)(rand() % 3000);
        fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
        body_update();
        float err_ms = (float)(int32_t)(body_odometry_time_us() - (uint32_t)sent_us) * 1e-3f;
        if (i >= 150) worst_ms = fmaxf(worst_ms, fabsf(err_ms));
        fake_now_us = sent_us + LINK_ODOMETRY_PERIOD_US;
    }
    assert(worst_ms < 0.3f);
    printf("clock sync: report times within %.2f ms after 3 s (0-3 ms random delay, 50 ppm drift)\n", (double)worst_ms);

    // PicoB restarts (its clock starts again from 0): right again within a second.
    b_hello(LINK_PROTOCOL_VERSION, false);
    b_start_us = fake_now_us;
    for (int i = 0; i < 50; i++) {
        uint64_t sent_us = fake_now_us;
        b_odometry.t_us = (uint32_t)(sent_us - b_start_us);
        fake_now_us += ODOMETRY_WIRE_US + (uint64_t)(rand() % 3000);
        fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
        body_update();
        if (i >= 40) assert(fabsf((float)(int32_t)(body_odometry_time_us() - (uint32_t)sent_us)) < 1000);
        fake_now_us = sent_us + LINK_ODOMETRY_PERIOD_US;
    }

    // A long loop iteration: three reports arrive before one body_update(); all are
    // kept, numbered, with their times. More than BODY_ODOMETRY_KEPT: the oldest go.
    uint32_t before = body_odometry_count();
    uint32_t b0 = b_odometry.t_us;
    for (int i = 1; i <= 3; i++) {
        b_odometry.t_us = b0 + (uint32_t)i * 20000u;
        b_odometry.x_m = (float)i;
        fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
    }
    fake_now_us += 60000;
    body_update();
    assert(body_odometry_count() == before + 3);
    for (uint32_t i = 1; i <= 3; i++) {
        odometry_report_t r;
        uint32_t t_us, t_next;
        assert(body_odometry_get(before + i, &r, &t_us) && r.x_m == (float)i);
        if (i < 3) assert(body_odometry_get(before + i + 1, &r, &t_next) && t_next - t_us == 20000);
    }
    odometry_report_t r;
    uint32_t t_us;
    assert(!body_odometry_get(before + 4, &r, &t_us) && !body_odometry_get(0, &r, &t_us));
    for (int i = 0; i < BODY_ODOMETRY_KEPT; i++) fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
    body_update();
    assert(!body_odometry_get(before + 3, &r, &t_us) && body_odometry_get(before + 4, &r, &t_us));
    printf("ok: every report kept and numbered, also several in one loop iteration\n");

    // imu records. PicoB restarts: its clock isn't known until its first report, so a
    // record before that is dropped.
    imu_record_t rec = {.t_us = 5, .acceleration = {1, 2, 3}, .gyro = {4, 5, 6}}, got;
    while (body_imu_take(&got)) {}
    b_hello(LINK_PROTOCOL_VERSION, false);
    body_update();
    uint32_t received0 = body_imu_received();
    fake_link_deliver(MESSAGE_IMU, &rec, sizeof rec);
    body_update();
    assert(!body_imu_take(&got) && body_imu_received() == received0);
    // Then 4 s of reports and two records each (measured 14 and 4 ms before the
    // report was sent), with PicoB's clock far off and 50 ppm fast, 0-3 ms random
    // delays; PicoA's clock wraps 2.5 s in. Each record's time on PicoA's clock is
    // within 0.3 ms of when it was measured once the clock has settled (1.5 s).
    uint64_t next_wrap = (fake_now_us | 0xFFFFFFFFull) + 1;
    assert(next_wrap - 2500000 > fake_now_us);
    fake_now_us = next_wrap - 2500000;
    b_start_us = fake_now_us;
    const uint32_t b0_us = 0x40000000u, before_us[2] = {14000, 4000};
    float worst_imu_ms = 0;
    int checked = 0, dropped_first = 0;
    bool crossed = false;
    uint32_t prev_t = 0;
    for (int i = 0; i < 200; i++) {
        uint64_t sent_us = fake_now_us;
        for (int k = 0; k < 2; k++) {
            uint64_t measured = sent_us - before_us[k];
            rec.t_us = b0_us + (uint32_t)((measured - b_start_us) * 1000050ull / 1000000ull);
            rec.acceleration[0] = (int16_t)i;
            rec.gyro[0] = (int16_t)k;
            fake_link_deliver(MESSAGE_IMU, &rec, sizeof rec);
        }
        b_odometry.t_us = b0_us + (uint32_t)((sent_us - b_start_us) * 1000050ull / 1000000ull);
        fake_now_us += ODOMETRY_WIRE_US + (uint64_t)(rand() % 3000);
        fake_link_deliver(MESSAGE_ODOMETRY, &b_odometry, sizeof b_odometry);
        body_update();
        if (i == 0) dropped_first = !body_imu_take(&got); // they came before the first report
        while (body_imu_take(&got)) {
            assert(got.acceleration[1] == 2 && got.acceleration[2] == 3 && got.gyro[1] == 5 && got.gyro[2] == 6); // unchanged
            uint32_t measured = (uint32_t)(b_start_us + (uint64_t)got.acceleration[0] * LINK_ODOMETRY_PERIOD_US - before_us[got.gyro[0]]);
            float err_ms = (float)(int32_t)(got.t_us - measured) * 1e-3f;
            if (got.acceleration[0] >= 75) { worst_imu_ms = fmaxf(worst_imu_ms, fabsf(err_ms)); checked++; }
            if (prev_t && got.t_us < prev_t) crossed = true;
            prev_t = got.t_us;
        }
        fake_now_us = sent_us + LINK_ODOMETRY_PERIOD_US;
    }
    assert(dropped_first && crossed && checked == 2 * 125 && worst_imu_ms < 0.3f);
    printf("imu records on PicoA's clock: within %.2f ms, also across its wrap; dropped before the clock is known\n",
           (double)worst_imu_ms);

    // A record of the wrong length is dropped; 40 not taken: the newest 32 wait, 8 lost.
    fake_link_deliver(MESSAGE_IMU, &rec, sizeof rec - 1);
    body_update();
    assert(!body_imu_take(&got));
    uint32_t lost0 = body_imu_lost();
    received0 = body_imu_received();
    for (int n = 0; n < 40; n++) {
        rec.acceleration[0] = (int16_t)n;
        fake_link_deliver(MESSAGE_IMU, &rec, sizeof rec);
        if (n % 10 == 9) body_update(); // the fake link holds 64
    }
    assert(body_imu_received() == received0 + 40 && body_imu_lost() == lost0 + 8);
    for (int n = 8; n < 40; n++) assert(body_imu_take(&got) && got.acceleration[0] == n);
    assert(!body_imu_take(&got));
    printf("ok: imu records kept until taken (the newest %d), the lost counted\n", BODY_IMU_KEPT);

    // PicoB with another version: its clock is still known, but its records are dropped.
    b_hello(LINK_PROTOCOL_VERSION + 1, false);
    body_update();
    received0 = body_imu_received();
    fake_link_deliver(MESSAGE_IMU, &rec, sizeof rec);
    body_update();
    assert(!body_imu_take(&got) && body_imu_received() == received0);
    printf("ok: imu records dropped from a PicoB with another version\n");

    printf("OK: body greets, repeats DRIVE, re-sends MOTORS, respects safety stops, times out, syncs clocks, keeps imu records\n");
    return 0;
}
