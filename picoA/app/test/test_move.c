// Host test for picoA/app/move.c: a move's phases with the fake clock (start: the
// motors on, waiting for the robot to stand still; pre: still pre_ms before the
// action drives; act; post: still post_ms, and that is all the settling there is;
// end: motors off), the still times as settings (0 acts at once; out of range
// refused), a stop in pre or post, a new move replacing one, PicoB's safety stop,
// PicoB lost, a move that doesn't start, one refused before it starts (a running
// one goes on), a watch as a move; the move start and move end messages (number,
// command, kind, parameter, times since power-up, still times; how it ended, the
// frames sent and dropped, how far it went; a refused move's flag; each safety stop
// named by its reason); tof frames go to the bulk lane only during a move (start,
// act and post), in the map use's format, under their move's number and with their
// time since power-up, counted, and dropped and counted when the lane is full; imu records go in batches of 10 only during a move,
// each batch under its move's number, the last (partial) batch before the move end,
// a gap or a step back in time starting a new batch, batches dropped and counted, the
// counts in the move end. The test plays body.h, link_pc.h and the action.
// Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon/test/fakes -Icommon -Ibuild/host -o build/test_move picoA/app/test/test_move.c -lm && build/test_move
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../tof_format.c"
#include "../wire.c"
#include "../move.c"

// --- body.h: the robot as PicoB reports it ---
static odometry_report_t odometry;
static bool connected = true, motors;
static float drive_v, drive_w;
static int drives_nonzero_before_act; // the action drove before its act phase: must stay 0
bool body_connected(void) { return connected; }
const odometry_report_t *body_odometry(void) { return &odometry; }
void body_motors(bool on) { motors = on; }
void body_drive(float v, float w) { drive_v = v; drive_w = w; }

// --- link_pc.h: what goes to the PC ---
static uint8_t control[64][WIRE_MESSAGE_MAX];
static int n_control;
static int bulk_room = 1000;  // messages the bulk lane still takes
static int n_bulk;
static uint8_t last_bulk[WIRE_MESSAGE_MAX];
static uint32_t last_tof_move;       // the move number the last tof frame went under
static wire_imu_batch_t batches[64]; // the imu batches the lane took
static int n_batches;
bool link_pc_send_control(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len && n_control < 64);
    memcpy(control[n_control++], message, len);
    return true;
}
bool link_pc_send_bulk(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len);
    if (bulk_room <= 0) return false;
    bulk_room--;
    n_bulk++;
    if (m.type == WIRE_IMU_BATCH) {
        assert(n_batches < 64 && wire_decode_imu_batch(&m, &batches[n_batches]));
        n_batches++;
        return true;
    }
    memcpy(last_bulk, message, len);
    last_tof_move = read32(m.payload);
    return true;
}
static wire_message_t control_message(int i) { wire_message_t m; assert(i < n_control && wire_take(control[i], WIRE_MESSAGE_MAX, &m) > 0); return m; }
static wire_move_start_t start_message(int i) {
    wire_message_t m = control_message(i);
    wire_move_start_t s;
    assert(wire_decode_move_start(&m, &s));
    return s;
}
static wire_move_end_t end_message(int i) {
    wire_message_t m = control_message(i);
    wire_move_end_t e;
    assert(wire_decode_move_end(&m, &e));
    return e;
}

// --- the action: drives until told it is done ---
static int begins, ends;
static uint64_t begun_at_us, ended_at_us;
static move_act_t result = MOVE_ACTING;
static move_end_t ended_how;
static void fake_begin(const odometry_report_t *o) { (void)o; begins++; begun_at_us = fake_now_us; }
static move_act_t fake_step(const odometry_report_t *o) {
    (void)o;
    if (result == MOVE_ACTING) body_drive(0.1f, 0);
    return result;
}
static void fake_end(const odometry_report_t *o, move_end_t how) { (void)o; ends++; ended_how = how; ended_at_us = fake_now_us; }
static const move_actor_t fake_actor = {fake_begin, fake_step, fake_end};

// A posed frame numbered n.
static void frame(uint32_t n) {
    static posed_frame_t f;
    f.frame.number = n;
    f.frame.period_us = 100000;
    f.frame.beyond_mm = TOF_EXACT;
    move_tof(&f);
}

static void loop_ms(int ms) {
    for (int i = 0; i < ms; i++) {
        fake_now_us += 1000;
        move_update();
        if (!begins && (drive_v != 0 || drive_w != 0)) drives_nonzero_before_act++;
    }
}

static void reset(void) {
    begins = ends = 0;
    result = MOVE_ACTING;
    n_control = 0;
    drive_v = drive_w = 0;
}

static void test_phases(void) {
    reset();
    odometry = (odometry_report_t){.stationary = false, .motors_on = false, .x_m = 1, .yaw_rad = 0.5f};
    frame(1); // idle: nothing goes
    assert(n_bulk == 0);
    move_start(MOVE_DRIVE, 0.5f, 17, &fake_actor);
    assert(motors && !strcmp(move_running(), "drive") && n_control == 1);
    wire_move_start_t s = start_message(0);
    assert(s.move == 1 && s.command == 17 && s.kind == WIRE_MOVE_DRIVE && s.parameter == 0.5f && s.t_us == clock_since_start_us() &&
           s.pre_ms == 200 && s.post_ms == 200);
    frame(2); // from the start on, frames go
    assert(n_bulk == 1);
    // The motors come on, the robot settles: only then the pre phase counts.
    loop_ms(100);
    odometry.motors_on = true;
    loop_ms(400);
    assert(begins == 0);
    odometry.stationary = true;
    uint64_t still_from = fake_now_us + 1000;
    loop_ms(150);
    assert(begins == 0 && drive_v == 0);
    loop_ms(100);
    assert(begins == 1 && begun_at_us - still_from == 200000); // pre: 200 ms of standing still, motors on
    assert(drives_nonzero_before_act == 0 && drive_v == 0.1f);
    odometry.stationary = false;
    frame(3);
    frame(4);
    odometry.x_m = 1.3f;
    odometry.yaw_rad = 0.6f;
    loop_ms(500);
    // Done: drive 0 at once; post: 200 ms still, and only once PicoB says still.
    result = MOVE_ACT_DONE;
    loop_ms(1);
    uint64_t done_at = fake_now_us;
    assert(drive_v == 0 && motors && *move_running());
    frame(5); // during post: still the move's
    loop_ms(300);
    assert(*move_running()); // not still yet
    odometry.stationary = true;
    loop_ms(1);
    assert(!*move_running() && !motors && ends == 1 && ended_how == MOVE_END_DONE && fake_now_us - done_at == 301000);
    move_record_t r;
    assert(move_endings(&r) == 1 && r.move == 1 && r.command == 17 && r.kind == MOVE_DRIVE && r.how == MOVE_END_DONE &&
           move_wire_end(&r) == WIRE_END_OK && r.frames_sent == 4 && r.frames_dropped == 0 && r.goal_m == 0);
    assert(r.moved_m > 0.29f && r.moved_m < 0.31f && r.rotated_rad > 0.099f && r.rotated_rad < 0.101f);
    wire_move_end_t e = end_message(1);
    assert(n_control == 2 && e.move == 1 && e.end == WIRE_END_OK && e.t_us == clock_since_start_us() && e.frames_sent == 4 &&
           e.frames_dropped == 0 && e.moved_m == r.moved_m);
    frame(6); // ended: no more frames
    assert(n_bulk == 4 && strcmp(move_running(), "") == 0);
    // The frames went in the map use's format.
    wire_message_t m;
    assert(wire_take(last_bulk, WIRE_MESSAGE_MAX, &m) > 0 && m.type == tof_format_message_type(tof_format_of(TOF_USE_MAP)));
    posed_frame_t back;
    assert(tof_format_unpack(tof_format_of(TOF_USE_MAP), m.payload + WIRE_TOF_HEADER, m.len - WIRE_TOF_HEADER, &back) &&
           back.frame.number == 5);
    // ... under the move's number, its time since power-up (the frame's own time_us_32()).
    assert(read32(m.payload) == 1 && read64(m.payload + 4) == clock_since_start_of(back.frame.t_us));
    printf("ok: start, pre 200 ms still before it drives, act, post 200 ms (the only settling), end; frames only during it\n");

    // The robot still at once after the act: the post is exactly post_ms, not post + a settle.
    reset();
    move_start(MOVE_ROTATE, 0.5f, 18, &fake_actor);
    loop_ms(1);
    assert(begins == 0);
    loop_ms(200);
    assert(begins == 1);
    result = MOVE_ACT_DONE;
    loop_ms(1);
    done_at = fake_now_us;
    loop_ms(199);
    assert(*move_running());
    loop_ms(1);
    assert(!*move_running() && fake_now_us - done_at == 200000);
    printf("ok: post_ms replaces the settle: ends %d ms after the action when still\n", move_post_ms());
}

static void test_still_settings(void) {
    reset();
    assert(!move_set_still(-1, 0) && !move_set_still(0, MOVE_STILL_MAX_MS + 1) && move_pre_ms() == 200);
    assert(move_set_still(0, 0) && move_pre_ms() == 0 && move_post_ms() == 0);
    odometry.stationary = odometry.motors_on = true;
    move_start(MOVE_ROTATE, -0.5f, 19, &fake_actor);
    assert(start_message(0).pre_ms == 0 && start_message(0).post_ms == 0);
    loop_ms(1);
    assert(begins == 1); // pre 0: acts at once
    result = MOVE_ACT_OBSTACLE;
    loop_ms(2); // post 0: ends on the next loop
    assert(!*move_running() && ended_how == MOVE_END_OBSTACLE && end_message(1).end == WIRE_END_OBSTACLE);
    assert(move_set_still(500, 1000));
    reset();
    move_start(MOVE_DRIVE, 1, 20, &fake_actor);
    loop_ms(500); // the first loop finds it still; pre counts from there
    assert(begins == 0);
    loop_ms(1);
    assert(begins == 1);
    result = MOVE_ACT_DONE;
    loop_ms(1);
    loop_ms(999);
    assert(*move_running());
    loop_ms(1);
    assert(!*move_running());
    assert(move_set_still(MOVE_PRE_MS_DEFAULT, MOVE_POST_MS_DEFAULT));
    printf("ok: the still times are settings (0 acts at once; 500 / 1000 ms; out of range refused)\n");
}

static void test_stops(void) {
    // Stop during pre: ends at once, motors off, the action never began.
    reset();
    odometry.stationary = odometry.motors_on = true;
    move_start(MOVE_DRIVE, 0.5f, 21, &fake_actor);
    loop_ms(100);
    move_stop();
    assert(!*move_running() && !motors && begins == 0 && ends == 1 && ended_how == MOVE_END_STOPPED);
    move_record_t r;
    move_endings(&r);
    assert(r.how == MOVE_END_STOPPED && move_wire_end(&r) == WIRE_END_STOPPED && r.moved_m == 0 && end_message(1).end == WIRE_END_STOPPED);
    // Stop during post.
    reset();
    move_start(MOVE_DRIVE, 0.5f, 22, &fake_actor);
    loop_ms(250);
    result = MOVE_ACT_DONE;
    odometry.stationary = false;
    loop_ms(50);
    assert(*move_running());
    move_stop();
    assert(!*move_running() && !motors && ended_how == MOVE_END_STOPPED);
    move_stop(); // nothing runs: nothing happens
    assert(ends == 1 && n_control == 2);
    printf("ok: a stop in pre or post ends the move at once, motors off\n");

    // A new move replaces the running one: the motors stay on.
    reset();
    odometry.stationary = true;
    move_start(MOVE_ROTATE, 1, 23, &fake_actor);
    loop_ms(300);
    move_start(MOVE_WATCH, 60, 24, &fake_actor);
    assert(motors && *move_running() && ended_how == MOVE_END_REPLACED && n_control == 3);
    assert(end_message(1).end == WIRE_END_REPLACED && start_message(2).kind == WIRE_MOVE_WATCH && start_message(2).parameter == 60 &&
           start_message(2).move == end_message(1).move + 1);
    assert(!strcmp(move_running(), "watch")); // a watch is a move like the others
    // PicoB's safety stop while acting.
    loop_ms(300);
    odometry.motors_on = false;
    odometry.stop_reason = STOP_TILT;
    loop_ms(1);
    move_record_t t;
    move_endings(&t);
    assert(!*move_running() && t.how == MOVE_END_SAFETY_STOP && t.stop_reason == STOP_TILT && move_wire_end(&t) == WIRE_END_TILT);
    // Each safety stop named by its reason.
    const struct { uint8_t stop; wire_end_t end; } named[] = {{STOP_TILT, WIRE_END_TILT}, {STOP_LEFT_WHEELS, WIRE_END_WHEELS},
        {STOP_RIGHT_WHEELS, WIRE_END_WHEELS}, {STOP_NO_DRIVE, WIRE_END_NO_DRIVE}, {STOP_NONE, WIRE_END_SAFETY}};
    for (int i = 0; i < 5; i++) {
        move_record_t x = {.how = MOVE_END_SAFETY_STOP, .stop_reason = named[i].stop};
        assert(move_wire_end(&x) == named[i].end);
    }
    assert(end_message(3).end == WIRE_END_TILT && t.kind == MOVE_WATCH);
    odometry.stop_reason = STOP_NONE;
    // PicoB lost.
    reset();
    odometry.motors_on = true;
    move_start(MOVE_DRIVE, 0.2f, 25, &fake_actor);
    connected = false;
    loop_ms(1);
    connected = true;
    assert(!*move_running() && !motors && ended_how == MOVE_END_PICOB_LOST && end_message(1).end == WIRE_END_PICOB_LOST);
    // Never still: no start after 3 s.
    reset();
    odometry.stationary = false;
    move_start(MOVE_DRIVE, 0.2f, 26, &fake_actor);
    loop_ms(2999);
    assert(*move_running());
    loop_ms(1);
    assert(!*move_running() && !motors && ended_how == MOVE_END_NO_START && end_message(1).end == WIRE_END_NO_START);
    odometry.stationary = true;
    printf("ok: replaced (motors stay on), a watch is a move, safety stop named, PicoB lost, no start after 3 s\n");
}

static void test_refused(void) {
    // Refused before it starts: numbered, start and end sent, no phases; a running move goes on.
    reset();
    uint32_t endings_before = move_endings(NULL);
    move_start(MOVE_ROTATE, 1, 30, &fake_actor);
    move_refused(MOVE_DRIVE, 0.5f, 31, MOVE_END_OBSTACLE);
    assert(!strcmp(move_running(), "rotate") && n_control == 3);
    wire_move_start_t a = start_message(0), b = start_message(1);
    wire_move_end_t e = end_message(2);
    assert(a.flags == 0 && b.flags == WIRE_MOVE_REFUSED); // the PC keeps the turn as the open move
    assert(b.move == a.move + 1 && b.command == 31 && e.move == b.move && e.end == WIRE_END_OBSTACLE && e.frames_sent == 0);
    move_record_t r;
    assert(move_endings(&r) == endings_before + 1 && r.command == 31 && r.moved_m == 0);
    frame(100); // the turn's frame
    loop_ms(300);
    result = MOVE_ACT_DONE;
    loop_ms(300);
    assert(!*move_running() && end_message(3).move == a.move && end_message(3).frames_sent == 1 && last_tof_move == a.move);
    // Taken one by one, oldest first: the refused drive, then the turn (both in one loop for commands.c).
    move_record_t q;
    while (move_next_ended(&q) && q.command != 31) {}
    assert(q.command == 31 && move_next_ended(&q) && q.command == 30 && !move_next_ended(&q));
    // A refused move and the running one ending in the same loop: both kept, in order.
    move_start(MOVE_WATCH, 5, 33, &fake_actor);
    move_refused(MOVE_DRIVE, 0.5f, 34, MOVE_END_OBSTACLE);
    move_stop();
    assert(move_next_ended(&q) && q.command == 34 && move_next_ended(&q) && q.command == 33 && !move_next_ended(&q));
    // A go to already there: done at once, its goal in the record.
    move_refused(MOVE_GOTO, 0.03f, 32, MOVE_END_DONE);
    assert(move_endings(&r) && move_wire_end(&r) == WIRE_END_OK && r.kind == MOVE_GOTO && r.goal_m == 0.03f);
    printf("ok: a move refused before it starts: start and end sent, a running move goes on\n");
}

// The time the PC gets: a time_us_32() time as µs since power-up, right across the
// 32-bit wrap (before and after it), a little ahead of now, and after several wraps.
static void test_since_start(void) {
    uint64_t saved = fake_now_us;
    fake_now_us = FAKE_CLOCK_WRAP_US + 1000;                                    // 1 ms after the wrap
    assert(clock_since_start_of((uint32_t)(FAKE_CLOCK_WRAP_US - 2000)) == 30000000u - 2000);
    assert(clock_since_start_of((uint32_t)fake_now_us) == 30000000u + 1000);
    assert(clock_since_start_of((uint32_t)(fake_now_us + 500)) == 30000000u + 1500);
    fake_now_us = FAKE_CLOCK_WRAP_US * 3 + 5;                                   // ~3.6 h after power-up
    assert(clock_since_start_of((uint32_t)fake_now_us - 100) == fake_now_us - 100 - CLOCK_START_US);
    fake_now_us = saved;
    printf("ok: times to the PC: since power-up across the 32-bit wrap, slightly ahead, after several wraps\n");
}

static void test_frames_dropped(void) {
    reset();
    odometry.stationary = true;
    move_start(MOVE_WATCH, 5, 40, &fake_actor);
    bulk_room = 2;
    for (uint32_t n = 200; n < 205; n++) frame(n);
    loop_ms(300);
    move_stop();
    move_record_t r;
    move_endings(&r);
    wire_move_end_t e = end_message(1);
    assert(r.frames_sent == 2 && r.frames_dropped == 3 && e.frames_sent == 2 && e.frames_dropped == 3);
    bulk_room = 1000;
    printf("ok: the bulk lane full: frames dropped and counted per move\n");
}

// imu record k of a run: measured every IMU_RECORD_PERIOD_US from t0, its counts from k.
static imu_record_t imu_rec(uint32_t t0, int k) {
    imu_record_t r = {.t_us = t0 + (uint32_t)k * IMU_RECORD_PERIOD_US};
    for (int a = 0; a < 3; a++) { r.acceleration[a] = (int16_t)(k * 3 + a); r.gyro[a] = (int16_t)(-k * 3 - a); }
    if (k == 0) { r.acceleration[0] = 32767; r.gyro[0] = -32768; } // the limits go through
    return r;
}

// Checks batch b holds records first..first+n-1 of a run from t0 (times since
// power-up within 5 µs).
static void check_batch(int b, uint32_t move, uint32_t t0, int first, int n) {
    assert(b < n_batches && batches[b].move == move && batches[b].n == n);
    for (int i = 0; i < n; i++) {
        imu_record_t want = imu_rec(t0, first + i), got = batches[b].records[i];
        int64_t got_us = (int64_t)(batches[b].t_us + (uint64_t)i * batches[b].period_us);
        assert(llabs(got_us - (int64_t)clock_since_start_of(want.t_us)) <= 5);
        assert(!memcmp(got.acceleration, want.acceleration, sizeof got.acceleration) && !memcmp(got.gyro, want.gyro, sizeof got.gyro));
    }
}

static void test_imu(void) {
    reset();
    n_batches = 0;
    odometry.stationary = odometry.motors_on = true;
    uint32_t t0 = time_us_32();
    imu_record_t r = imu_rec(t0, 0);
    move_imu(&r); // idle: nothing goes
    assert(n_batches == 0);
    // A move: 25 records make two batches of 10 while it runs, and the last 5 go
    // as a batch of their own when it ends, before the move end.
    move_start(MOVE_ROTATE, 1, 50, &fake_actor);
    uint32_t move = start_message(0).move;
    t0 = time_us_32() + 0xfffff000u; // the records' times cross the clock's wrap
    for (int k = 0; k < 25; k++) {
        r = imu_rec(t0, k);
        move_imu(&r);
        assert(n_batches == (k + 1) / 10);
    }
    loop_ms(250);
    result = MOVE_ACT_DONE;
    loop_ms(300);
    assert(!*move_running() && n_batches == 3);
    check_batch(0, move, t0, 0, 10);
    check_batch(1, move, t0, 10, 10);
    check_batch(2, move, t0, 20, 5);
    wire_move_end_t e = end_message(1);
    move_record_t rec;
    move_endings(&rec);
    assert(e.imu_sent == 3 && e.imu_dropped == 0 && rec.imu_sent == 3 && rec.imu_dropped == 0);
    r = imu_rec(t0, 26);
    move_imu(&r); // ended: no more
    assert(n_batches == 3);
    printf("ok: imu records in batches of 10 during a move, the last 5 sent at its end, under its number\n");

    // A record lost on the way (a gap of two periods), and a record from before the
    // last: each starts a new batch, so a batch's records are evenly spaced.
    reset();
    n_batches = 0;
    move_start(MOVE_DRIVE, 0.2f, 51, &fake_actor);
    move = start_message(0).move;
    t0 = time_us_32();
    for (int k = 0; k < 3; k++) { r = imu_rec(t0, k); move_imu(&r); }
    for (int k = 4; k < 7; k++) { r = imu_rec(t0, k); move_imu(&r); }  // record 3 lost
    assert(n_batches == 1);
    r = imu_rec(t0, 2);
    move_imu(&r);                                                       // back in time
    assert(n_batches == 2);
    r = imu_rec(t0, 7);
    r.t_us = imu_rec(t0, 2).t_us + IMU_RECORD_PERIOD_US + 1200;         // a sample missed on PicoB: still the next
    move_imu(&r);
    move_stop();
    assert(n_batches == 3 && batches[2].n == 2);
    check_batch(0, move, t0, 0, 3);
    check_batch(1, move, t0, 4, 3);
    assert(end_message(1).imu_sent == 3);
    printf("ok: a lost record or a step back in time starts a new batch\n");

    // Replaced: the running move's records go under its own number; then the new one's.
    reset();
    n_batches = 0;
    move_start(MOVE_WATCH, 5, 52, &fake_actor);
    uint32_t first = start_message(0).move;
    t0 = time_us_32();
    for (int k = 0; k < 4; k++) { r = imu_rec(t0, k); move_imu(&r); }
    move_start(MOVE_ROTATE, 1, 53, &fake_actor);
    for (int k = 4; k < 6; k++) { r = imu_rec(t0, k); move_imu(&r); }
    move_refused(MOVE_DRIVE, 1, 54, MOVE_END_OBSTACLE); // a refused move: no batches, the turn goes on
    move_stop();
    assert(n_batches == 2 && end_message(1).imu_sent == 1 && end_message(4).imu_sent == 0 && end_message(5).imu_sent == 1);
    check_batch(0, first, t0, 0, 4);
    check_batch(1, first + 1, t0, 4, 2);
    printf("ok: a replaced move's last batch under its own number; a refused move sends none\n");

    // The gap's edges: half a period and one and a half still belong to the batch;
    // a µs less or more starts a new one.
    reset();
    n_batches = 0;
    move_start(MOVE_WATCH, 5, 56, &fake_actor);
    t0 = time_us_32();
    const int32_t steps[] = {IMU_RECORD_PERIOD_US / 2, IMU_RECORD_PERIOD_US * 3 / 2,      // 4807, 14422: kept
                             IMU_RECORD_PERIOD_US / 2 - 1, IMU_RECORD_PERIOD_US * 3 / 2 + 1}; // new batch each
    r = imu_rec(t0, 0);
    move_imu(&r);
    for (int k = 0; k < 4; k++) { r.t_us += (uint32_t)steps[k]; move_imu(&r); }
    move_stop();
    assert(n_batches == 3 && batches[0].n == 3 && batches[1].n == 1 && batches[2].n == 1);
    printf("ok: the gap's edges: 0.5 and 1.5 periods keep the batch, beyond them a new one\n");

    // A sensor 2 %% slow (9807 µs between records): still one batch of 10, and the
    // period says so; records while the move is starting (not yet still) go too.
    reset();
    n_batches = 0;
    odometry.stationary = false;
    move_start(MOVE_ROTATE, 1, 57, &fake_actor);
    t0 = time_us_32();
    for (int k = 0; k < 10; k++) {
        r = imu_rec(t0, k);
        r.t_us = t0 + (uint32_t)k * 9807u;
        move_imu(&r);
        loop_ms(9);
    }
    assert(begins == 0 && n_batches == 1 && batches[0].n == 10 && batches[0].period_us == 9807);
    odometry.stationary = true;
    move_stop();
    printf("ok: a sensor 2 %% slow stays one batch with its own period; records while starting go too\n");

    // Off the even spacing: the period from the first to the last record, rounded;
    // one record gets the nominal period.
    reset();
    n_batches = 0;
    move_start(MOVE_WATCH, 5, 58, &fake_actor);
    t0 = time_us_32();
    for (int k = 0; k < 3; k++) {
        r = imu_rec(t0, k);
        r.t_us += k == 1 ? 40u : (uint32_t)(k == 2); // PicoB read record 1 40 µs late, record 2 1 µs late
        move_imu(&r);
    }
    move_stop();
    move_start(MOVE_WATCH, 5, 59, &fake_actor);
    r = imu_rec(t0, 5);
    move_imu(&r);
    move_stop();
    assert(n_batches == 2 && batches[0].period_us == 9616 && batches[1].n == 1 && batches[1].period_us == IMU_RECORD_PERIOD_US);
    printf("ok: the period: first to last over n - 1, rounded (9616 for 2 × 9615 + 1); the nominal for one record\n");

    // The bulk lane full: batches dropped and counted.
    reset();
    n_batches = 0;
    move_start(MOVE_WATCH, 5, 55, &fake_actor);
    t0 = time_us_32();
    bulk_room = 1;
    for (int k = 0; k < 35; k++) { r = imu_rec(t0, k); move_imu(&r); }
    move_stop();
    move_endings(&rec);
    e = end_message(1);
    assert(n_batches == 1 && e.imu_sent == 1 && e.imu_dropped == 3 && rec.imu_dropped == 3);
    bulk_room = 1000;
    printf("ok: imu batches dropped and counted when the lane is full\n");
}

int main(void) {
    fake_now_us = FAKE_CLOCK_START_US + 29000000; // the first move crosses the clock's wrap
    test_phases();
    test_still_settings();
    test_stops();
    test_refused();
    test_since_start();
    test_frames_dropped();
    test_imu();
    printf("move: all tests passed\n");
    return 0;
}
