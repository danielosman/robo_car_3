// Host test of the IMU pipeline end to end: PicoB's real odometry and brain
// (test_imu_path_picob.c, its own clock and fake link) -> the link messages passed
// over -> PicoA's real body, main.c's pipeline, move and wire -> the imu batches as
// the PC gets them, decoded. PicoB's clock is far from PicoA's and both wrap during
// the test. Checks: every record of the move arrives once, in order, with its counts
// unchanged; its time on PicoA's clock is within 1.5 ms of when its samples were read
// (the link here passes messages within 1 ms); the batches' periods are 9.6 ms; the
// move end's counts match. Run from the repo root (run_tests.sh builds both files):
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -IpicoB/app/test/stubs -Icommon -Ibuild/host \
//      -o build/test_imu_path picoA/app/test/test_imu_path.c picoA/app/test/test_imu_path_picob.c -lm && build/test_imu_path
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../tof_format.c"
#include "../wire.c"
#include "../body.c"
#include "../move.c"
#include "imu_path.h"

// --- link_pc.h: the imu batches and the move end as the PC gets them ---
static wire_imu_batch_t batches[256];
static int n_batches;
static wire_move_end_t move_end;
static bool ended_seen;
bool link_send_control(const uint8_t *msg, uint32_t len) {
    wire_msg_t m;
    assert(wire_take(msg, len, &m) == (int32_t)len);
    if (m.type == WIRE_MOVE_END) { assert(wire_decode_move_end(&m, &move_end)); ended_seen = true; }
    return true;
}
bool link_send_bulk(const uint8_t *msg, uint32_t len) {
    wire_msg_t m;
    assert(wire_take(msg, len, &m) == (int32_t)len && m.type == WIRE_IMU_BATCH && n_batches < 256);
    assert(wire_decode_imu_batch(&m, &batches[n_batches]));
    n_batches++;
    return true;
}

// --- the action: stands still 2 s ---
static int act_steps;
static void act_begin(const odom_report_t *o) { (void)o; }
static move_act_t act_step(const odom_report_t *o) { (void)o; return ++act_steps >= 2000 ? MOVE_ACT_DONE : MOVE_ACTING; }
static void act_end(const odom_report_t *o, move_end_t how) { (void)o; assert(how == MOVE_END_DONE); }
static const move_actor_t actor = {act_begin, act_step, act_end};

// PicoB's clock: this far ahead of PicoA's (mod 2^32), so it wraps at another moment.
#define B_AHEAD_US 0x9abc0123ull
#define SAMPLE_US  2404
// PicoB's samples: sample s belongs to record s / 4 (odometry starts a record with
// the first sample); all four carry the record's number, so the average is exact.
static uint64_t b_next_sample_us; // PicoA's clock
static uint32_t samples;
static uint64_t record_mid_us[20000]; // when record j's samples were read (PicoA's clock), middle

static void run_ms(int ms, bool move_running) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        while (b_next_sample_us <= fake_now_us) {
            uint32_t j = samples / 4;
            if (samples % 4 == 0) record_mid_us[j % 20000] = b_next_sample_us + (3 * SAMPLE_US + 1) / 2;
            const int16_t accel[3] = {(int16_t)(j % 30000), (int16_t)-(int16_t)(j % 30000), 16384};
            const int16_t gyro[3] = {-32768, 32767, (int16_t)(j % 7)};
            b_sample(b_next_sample_us + B_AHEAD_US, accel, gyro);
            samples++;
            b_next_sample_us += SAMPLE_US;
        }
        link_msg_t m[64];
        int n = b_sent(m, 64);
        for (int k = 0; k < n; k++) fake_link_deliver(m[k].type, m[k].body, m[k].len); // PicoB -> PicoA within 1 ms
        body_update();
        imu_record_t r;
        while (body_imu_take(&r)) move_imu(&r); // main.c's imu_pipeline()
        if (move_running) move_update();
        for (int k = 0; k < fake_sent_n; k++) b_deliver(&fake_sent[k]); // PicoA -> PicoB
        fake_sent_n = 0;
    }
}

int main(void) {
    fake_now_us = FAKE_CLOCK_WRAP_US - 1500000; // PicoA's clock wraps 1.5 s in
    b_next_sample_us = fake_now_us + 1000;
    body_init();
    b_start(fake_now_us + B_AHEAD_US);
    run_ms(1000, false); // greeting, ODOM, the clocks matched; nothing batched while idle
    assert(body_connected() && n_batches == 0);
    uint32_t received_idle = body_imu_received();
    assert(received_idle > 90); // ~104 a second once greeted

    move_start(MOVE_WATCH, 2, 1, &actor);
    run_ms(3500, true);
    assert(!*move_running() && ended_seen && move_end.end == WIRE_END_OK);

    // Every batch is the move's, the records consecutive and unchanged.
    assert(n_batches > 20 && (uint32_t)n_batches == move_end.imu_sent && move_end.imu_dropped == 0);
    int records = 0;
    float worst_ms = 0;
    int32_t prev = -1;
    for (int b = 0; b < n_batches; b++) {
        assert(batches[b].move == 1);
        for (int i = 0; i < batches[b].n; i++) {
            const imu_record_t *r = &batches[b].records[i];
            int32_t j = r->accel[0];
            assert(prev < 0 || j == prev + 1);
            prev = j;
            assert(r->accel[1] == -j && r->accel[2] == 16384 && r->gyro[0] == -32768 && r->gyro[1] == 32767 && r->gyro[2] == j % 7);
            uint64_t t_us = batches[b].t_us + (uint64_t)i * batches[b].period_us; // since power-up
            float err_ms = (float)((int64_t)t_us - (int64_t)(record_mid_us[j] - CLOCK_START_US)) * 1e-3f;
            if (err_ms < 0) err_ms = -err_ms;
            if (err_ms > worst_ms) worst_ms = err_ms;
            records++;
        }
        if (batches[b].n > 1) assert(batches[b].period_us >= 9612 && batches[b].period_us <= 9620); // 4 samples of 2404 µs
    }
    // The move: ~0.2 s start, 0.2 s still, 2 s acting, 0.2 s still: ~2.6 s, ~270 records.
    printf("imu path: %d records in %d batches, all in order and unchanged, times since power-up within %.2f ms\n",
           records, n_batches, (double)worst_ms);
    assert(records > 230 && records < 300 && worst_ms < 1.5f);
    assert(body_imu_lost() == 0 && n_batches == (records + 9) / 10);
    printf("OK: the IMU from PicoB's odometry to the PC's batches\n");
    return 0;
}
