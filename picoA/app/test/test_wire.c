// Host test for picoA/app/wire.c: the message table (every type's lane and lengths;
// the ToF frames' types from tof_format, a collision found), every message encoded
// and decoded again (move start and end, a tof frame in its message too, imu batches
// of 1 and 10 records: the int16 limits, a time past 2^32 µs, the period), the
// envelope's errors (not whole yet, unknown type, too long, too short), what the
// encoders and decoders refuse, and the golden bytes both sides check against
// (picoA/app/test/data/wire_golden.hex; pc/robot/src/wire.test.ts reads the same
// file): each sample encoded here must give its line's bytes, and each line
// must decode to its sample. After a change to the wire, write the file again with
//   build/test_wire --write
// (and raise WIRE_VERSION). Run from the repo root:
//   cc -std=c11 -Wall -Wextra -Icommon -o build/test_wire picoA/app/test/test_wire.c && build/test_wire
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../tof_format.c"
// wire.c asks tof_format for the ToF types through this, so the test can make one collide.
static bool collide;
static uint8_t test_tof_type(tof_format_t f);
#define tof_format_message_type test_tof_type
#include "../wire.c"
#undef tof_format_message_type
static uint8_t test_tof_type(tof_format_t f) { return collide ? WIRE_ANSWER : tof_format_message_type(f); }

#define GOLDEN "picoA/app/test/data/wire_golden.hex"

static wire_message_t take_whole(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len);
    return m;
}

static void test_table(void) {
    assert(wire_check());
    wire_type_info_t i;
    assert(wire_info(WIRE_HELLO, &i) && i.lane == WIRE_LANE_CONTROL);
    assert(wire_info(WIRE_CONSOLE, &i) && i.lane == WIRE_LANE_CONTROL && i.max == 240);
    assert(wire_info(WIRE_ANSWER, &i) && i.min == 34 && i.max == 34);
    assert(wire_info(WIRE_STATUS, &i) && i.min == 33 && i.max == 33);
    assert(wire_info(WIRE_MAP_CLEARED, &i) && i.max == 0);
    // A move's start and end: control; their times u64.
    assert(wire_info(WIRE_MOVE_START, &i) && i.lane == WIRE_LANE_CONTROL && i.min == 26 && i.max == 26);
    assert(wire_info(WIRE_MOVE_END, &i) && i.lane == WIRE_LANE_CONTROL && i.min == 37 && i.max == 37);
    assert(wire_info(WIRE_SNAPSHOT_PART, &i) && i.lane == WIRE_LANE_BULK && i.max == 1400);
    // The imu batch: bulk; 1-10 records of 12 B after 15 B.
    assert(wire_info(WIRE_IMU_BATCH, &i) && i.lane == WIRE_LANE_BULK && i.min == 27 && i.max == 135);
    assert(wire_info(WIRE_COMMAND, &i) && i.lane == WIRE_LANE_TO_ROBOT);
    assert(wire_info(WIRE_HEARTBEAT, &i) && i.lane == WIRE_LANE_TO_ROBOT);
    // The ToF frame: its type and size are tof_format's, after the move and the time.
    uint8_t tof = tof_format_message_type(TOF_FORMAT_COMPACT);
    assert(wire_info(tof, &i) && i.lane == WIRE_LANE_BULK && i.min == WIRE_TOF_HEADER + TOF_COMPACT_BYTES &&
           i.max == WIRE_TOF_HEADER + TOF_COMPACT_BYTES);
    assert(!wire_info(0x00, &i) && !wire_info(0xff, &i) && !wire_info('W', &i)); // 'W': an old robot's text
    collide = true; // tof_format gives a type the table has
    assert(!wire_check());
    collide = false;
    assert(wire_check());
    printf("ok: the table, the ToF's types from tof_format, a collision found\n");
}

static bool same_answer(const wire_answer_t *a, const wire_answer_t *b) {
    return a->n == b->n && a->end == b->end && a->has == b->has && a->x_m == b->x_m && a->y_m == b->y_m &&
           a->yaw_rad == b->yaw_rad && a->moved_m == b->moved_m && a->rotated_rad == b->rotated_rad &&
           a->goal_m == b->goal_m && a->snapshot == b->snapshot;
}

static bool same_status(const wire_status_t *a, const wire_status_t *b) {
    return a->x_m == b->x_m && a->y_m == b->y_m && a->yaw_rad == b->yaw_rad && !strcmp(a->mode, b->mode) &&
           !strcmp(a->action, b->action) && a->rssi_dbm == b->rssi_dbm;
}

// The golden file's imu batches: ten records 9615 µs apart, the first past 2^32 µs
// since power-up, the int16 limits and both signs; and one record.
#define SAMPLE_T_US 5000000123ull
static wire_imu_batch_t sample_batch(int n) {
    wire_imu_batch_t b = {.move = 12, .t_us = SAMPLE_T_US, .period_us = IMU_RECORD_PERIOD_US, .n = (uint8_t)n};
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            b.records[i].acceleration[k] = (int16_t)(i * 1000 - k * 7);
            b.records[i].gyro[k] = (int16_t)(-i * 300 + k);
        }
    }
    b.records[0].acceleration[2] = 16384;                       // 1 g up
    b.records[n - 1].acceleration[0] = 32767; b.records[n - 1].gyro[1] = -32768;
    return b;
}

static void test_round_trips(void) {
    uint8_t message[WIRE_MESSAGE_MAX] = {0};
    wire_message_t m;

    wire_hello_t h = {.version = WIRE_VERSION, .boot_id = 0x7a587226, .build = "Nov  7 2026 18:00:00"}, h2;
    m = take_whole(message, wire_encode_hello(&h, message));
    assert(wire_decode_hello(&m, &h2) && h2.version == WIRE_VERSION && h2.boot_id == h.boot_id && !strcmp(h2.build, h.build));
    wire_hello_t long_build = {.version = 1, .build = "123456789012345678901234"}; // 24: fills the field, no NUL
    m = take_whole(message, wire_encode_hello(&long_build, message));
    assert(wire_decode_hello(&m, &h2) && !strcmp(h2.build, long_build.build));

    char text[WIRE_CONSOLE_MAX + 1];
    m = take_whole(message, wire_encode_console("Map cleared", 11, message));
    assert(m.len == 11 && wire_decode_console(&m, text) && !strcmp(text, "Map cleared"));
    m = take_whole(message, wire_encode_console("", 0, message));
    assert(m.len == 0 && wire_decode_console(&m, text) && !strcmp(text, ""));
    char longer[300];
    memset(longer, 'x', sizeof longer);
    m = take_whole(message, wire_encode_console(longer, sizeof longer, message)); // cut at 240
    assert(m.len == WIRE_CONSOLE_MAX && wire_decode_console(&m, text) && strlen(text) == WIRE_CONSOLE_MAX);

    wire_answer_t a = {.n = 17, .end = WIRE_END_OBSTACLE, .has = WIRE_HAS_MOVE | WIRE_HAS_GOAL, .x_m = 0.43f, .y_m = -0.04f,
                       .yaw_rad = 0.5236f, .moved_m = 0.31f, .rotated_rad = -0.125f, .goal_m = 4.2f}, a2;
    m = take_whole(message, wire_encode_answer(&a, message));
    assert(wire_decode_answer(&m, &a2) && same_answer(&a, &a2));
    for (int end = 0; end < WIRE_ENDS; end++) {
        a.end = (uint8_t)end;
        m = take_whole(message, wire_encode_answer(&a, message));
        assert(wire_decode_answer(&m, &a2) && a2.end == end);
    }

    wire_status_t s = {.x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 7.0f, .mode = "manual", .action = "goto", .rssi_dbm = -61}, s2;
    m = take_whole(message, wire_encode_status(&s, message));
    assert(wire_decode_status(&m, &s2) && same_status(&s, &s2));
    wire_status_t idle = {.mode = "idle", .action = ""};
    m = take_whole(message, wire_encode_status(&idle, message));
    assert(wire_decode_status(&m, &s2) && !strcmp(s2.mode, "idle") && !strcmp(s2.action, "") && s2.rssi_dbm == 0);

    m = take_whole(message, wire_encode_map_cleared(message));
    assert(m.type == WIRE_MAP_CLEARED && m.len == 0);

    wire_move_start_t ms = {.move = 12, .command = 17, .kind = WIRE_MOVE_GOTO, .parameter = 4.2f, .t_us = 4300000000ull,
                            .pre_ms = 200, .post_ms = 2000, .flags = WIRE_MOVE_REFUSED}, ms2;
    m = take_whole(message, wire_encode_move_start(&ms, message));
    assert(wire_decode_move_start(&m, &ms2) && ms2.move == 12 && ms2.command == 17 && ms2.kind == WIRE_MOVE_GOTO &&
           ms2.parameter == 4.2f && ms2.t_us == 4300000000ull && ms2.pre_ms == 200 && ms2.post_ms == 2000 && ms2.flags == WIRE_MOVE_REFUSED);
    wire_move_end_t me = {.move = 12, .end = WIRE_END_TILT, .t_us = 0xffffffffffffull, .frames_sent = 41, .frames_dropped = 2,
                          .moved_m = 0.31f, .rotated_rad = -0.125f, .imu_sent = 44, .imu_dropped = 0xffffffffu}, me2;
    m = take_whole(message, wire_encode_move_end(&me, message));
    assert(wire_decode_move_end(&m, &me2) && me2.move == 12 && me2.end == WIRE_END_TILT && me2.t_us == 0xffffffffffffull &&
           me2.frames_sent == 41 && me2.frames_dropped == 2 &&
           me2.moved_m == 0.31f && me2.rotated_rad == -0.125f && me2.imu_sent == 44 && me2.imu_dropped == 0xffffffffu);
    assert(!wire_decode_move_start(&m, &ms2)); // another type

    // imu batches of 1 and 10 records come back with their move, time, period and counts.
    for (int n = 1; n <= WIRE_IMU_BATCH_MAX; n += WIRE_IMU_BATCH_MAX - 1) {
        wire_imu_batch_t b = sample_batch(n), b2;
        m = take_whole(message, wire_encode_imu_batch(&b, message));
        assert(m.type == WIRE_IMU_BATCH && m.len == WIRE_IMU_BATCH_HEADER + n * WIRE_IMU_RECORD);
        assert(wire_decode_imu_batch(&m, &b2) && b2.move == 12 && b2.n == n && b2.t_us == SAMPLE_T_US &&
               b2.period_us == IMU_RECORD_PERIOD_US);
        for (int i = 0; i < n; i++)
            assert(!memcmp(b.records[i].acceleration, b2.records[i].acceleration, 6) && !memcmp(b.records[i].gyro, b2.records[i].gyro, 6));
    }
    wire_imu_batch_t none = {.n = 0}, many = {.n = WIRE_IMU_BATCH_MAX + 1};
    assert(wire_encode_imu_batch(&none, message) == 0 && wire_encode_imu_batch(&many, message) == 0);

    // A tof frame in its message: its move and time, then tof_format's bytes, read back whole.
    static posed_frame_t f, back;
    f.frame.number = 77; f.frame.t_us = 123456; f.frame.period_us = 100000; f.frame.beyond_mm = TOF_EXACT;
    f.x_m = 1.5f; f.yaw_rad = 0.5f; f.pose_known = true;
    for (int z = 0; z < TOF_ZONES; z++) f.frame.zone[z] = (tof_zone_t){.distance_mm = (uint16_t)(100 + 10 * z), .signal = 30,
                                                                      .sigma_mm = 4, .kind = TOF_SURE, .targets = 1};
    static uint8_t tof_message[WIRE_HEADER + WIRE_TOF_HEADER + TOF_FORMAT_MAX];
    uint32_t n = wire_encode_tof(TOF_USE_MAP, 9, 4300000000ull, &f, tof_message);
    m = take_whole(tof_message, n);
    assert(m.type == tof_format_message_type(tof_format_of(TOF_USE_MAP)) && m.len == WIRE_TOF_HEADER + TOF_COMPACT_BYTES);
    assert(read32(m.payload) == 9 && read64(m.payload + 4) == 4300000000ull);
    assert(tof_format_unpack(TOF_FORMAT_COMPACT, m.payload + WIRE_TOF_HEADER, TOF_COMPACT_BYTES, &back) && back.frame.number == 77 &&
           back.frame.zone[63].distance_mm == 730 && back.pose_known && back.x_m == 1.5f);

    uint8_t part[WIRE_PART_MAX];
    for (int i = 0; i < WIRE_PART_MAX; i++) part[i] = (uint8_t)(i * 7);
    m = take_whole(message, wire_encode_snapshot_part(part, WIRE_PART_MAX, message));
    assert(m.type == WIRE_SNAPSHOT_PART && m.len == WIRE_PART_MAX && !memcmp(m.payload, part, WIRE_PART_MAX));
    assert(wire_encode_snapshot_part(part, 0, message) == 0 && wire_encode_snapshot_part(part, WIRE_PART_MAX + 1, message) == 0);

    wire_command_t c = {.n = 17, .text = "drive 50"}, c2;
    m = take_whole(message, wire_encode_command(&c, message));
    assert(wire_decode_command(&m, &c2) && c2.n == 17 && !strcmp(c2.text, "drive 50"));
    wire_command_t longest = {.n = 0xffffffff};
    memset(longest.text, 'a', WIRE_COMMAND_TEXT_MAX);
    m = take_whole(message, wire_encode_command(&longest, message));
    assert(wire_decode_command(&m, &c2) && c2.n == 0xffffffff && strlen(c2.text) == WIRE_COMMAND_TEXT_MAX);
    wire_command_t empty = {.n = 1, .text = ""}, newline = {.n = 1, .text = "drive\n1"};
    assert(wire_encode_command(&empty, message) == 0 && wire_encode_command(&newline, message) == 0);

    m = take_whole(message, wire_encode_heartbeat(message));
    assert(m.type == WIRE_HEARTBEAT && m.len == 0);
    printf("ok: every message encoded and decoded again\n");
}

static void test_envelope(void) {
    uint8_t stream[128];
    wire_answer_t a = {.n = 3};
    uint32_t n1 = wire_encode_answer(&a, stream), n2 = wire_encode_console("hi", 2, stream + n1);
    wire_message_t m;
    for (uint32_t len = 0; len < n1; len++) assert(wire_take(stream, len, &m) == 0); // not whole yet
    assert(wire_take(stream, n1 + n2, &m) == (int32_t)n1 && m.type == WIRE_ANSWER && m.len == 34);
    assert(wire_take(stream + n1, n2, &m) == (int32_t)n2 && m.type == WIRE_CONSOLE && !memcmp(m.payload, "hi", 2));
    // Errors: an unknown type at once, before its length; too long; too short.
    const uint8_t unknown[] = {0x99};
    assert(wire_take(unknown, 1, &m) == WIRE_UNKNOWN_TYPE);
    const uint8_t text[] = "WiFi: connected\n"; // an old robot's console text
    assert(wire_take(text, sizeof text - 1, &m) == WIRE_UNKNOWN_TYPE);
    uint8_t bad[3];
    wire_header(bad, WIRE_CONSOLE, WIRE_CONSOLE_MAX + 1);
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_SNAPSHOT_PART, 1401);
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_ANSWER, 33);
    assert(wire_take(bad, 3, &m) == WIRE_TOO_SHORT);
    wire_header(bad, WIRE_STATUS, 49); // the old status, with PicoB's peaks
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_MOVE_END, 41); // version 3's move end, with frame numbers and a u32 time
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_IMU_BATCH, 26); // no record
    assert(wire_take(bad, 3, &m) == WIRE_TOO_SHORT);
    wire_header(bad, WIRE_IMU_BATCH, 136); // more than 10
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_HEARTBEAT, 1);
    assert(wire_take(bad, 3, &m) == WIRE_TOO_LONG);
    wire_header(bad, WIRE_COMMAND, 4); // a number and no text
    assert(wire_take(bad, 3, &m) == WIRE_TOO_SHORT);
    assert(!strcmp(wire_error_text(WIRE_UNKNOWN_TYPE), "unknown message type"));
    // A hello of any length passes the envelope (another version's), but this
    // version reads only its own.
    uint8_t hello[WIRE_HEADER + 40] = {0};
    wire_header(hello, WIRE_HELLO, 40);
    hello[3] = 9;
    m = take_whole(hello, sizeof hello);
    wire_hello_t h;
    assert(m.payload[0] == 9 && !wire_decode_hello(&m, &h));
    // An imu batch whose count doesn't match its length, or is 0: refused.
    wire_imu_batch_t b = sample_batch(3), b2;
    uint8_t batch[WIRE_HEADER + WIRE_IMU_BATCH_HEADER + WIRE_IMU_BATCH_MAX * WIRE_IMU_RECORD];
    uint32_t nb = wire_encode_imu_batch(&b, batch);
    m = take_whole(batch, nb);
    assert(wire_decode_imu_batch(&m, &b2));
    batch[WIRE_HEADER + 14] = 2;
    m = take_whole(batch, nb);
    assert(!wire_decode_imu_batch(&m, &b2));
    batch[WIRE_HEADER + 14] = 0;
    m = take_whole(batch, nb);
    assert(!wire_decode_imu_batch(&m, &b2));
    // Decoders refuse another type or a wrong length; a command with a control character.
    wire_answer_t got;
    m = take_whole(stream + n1, n2);
    assert(!wire_decode_answer(&m, &got));
    uint8_t cmd[16];
    wire_header(cmd, WIRE_COMMAND, 6);
    memcpy(cmd + 3, "\1\0\0\0a\n", 6);
    m = take_whole(cmd, 9);
    wire_command_t c;
    assert(!wire_decode_command(&m, &c));
    printf("ok: the envelope: not whole yet, unknown type, too long, too short; decoders check\n");
}

// --- the golden bytes ---

typedef struct { const char *name; uint8_t bytes[WIRE_HEADER + WIRE_TOF_HEADER + TOF_FORMAT_MAX]; uint32_t len; } sample_t;

static uint32_t samples(sample_t *out) {
    uint32_t n = 0;
    wire_hello_t h = {.version = WIRE_VERSION, .boot_id = 0x7a587226, .build = "Nov  7 2026 18:00:00"};
    out[n] = (sample_t){.name = "hello"}; out[n].len = wire_encode_hello(&h, out[n].bytes); n++;
    out[n] = (sample_t){.name = "console"}; out[n].len = wire_encode_console("Turn: done, 90.0 deg", 20, out[n].bytes); n++;
    out[n] = (sample_t){.name = "console_empty"}; out[n].len = wire_encode_console("", 0, out[n].bytes); n++;
    wire_answer_t move = {.n = 17, .end = WIRE_END_OBSTACLE, .has = WIRE_HAS_MOVE | WIRE_HAS_GOAL, .x_m = 0.43f,
                          .y_m = -0.04f, .yaw_rad = 0.5236f, .moved_m = 0.31f, .rotated_rad = -0.125f, .goal_m = 4.2f};
    out[n] = (sample_t){.name = "answer_move"}; out[n].len = wire_encode_answer(&move, out[n].bytes); n++;
    wire_answer_t snap = {.n = 4, .end = WIRE_END_OK, .has = WIRE_HAS_SNAPSHOT, .x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 0.75f,
                          .snapshot = 41};
    out[n] = (sample_t){.name = "answer_snapshot"}; out[n].len = wire_encode_answer(&snap, out[n].bytes); n++;
    wire_answer_t refused = {.n = 9, .end = WIRE_END_REFUSED};
    out[n] = (sample_t){.name = "answer_refused"}; out[n].len = wire_encode_answer(&refused, out[n].bytes); n++;
    wire_status_t s = {.x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 7.0f, .mode = "manual", .action = "goto", .rssi_dbm = -61};
    out[n] = (sample_t){.name = "status"}; out[n].len = wire_encode_status(&s, out[n].bytes); n++;
    out[n] = (sample_t){.name = "map_cleared"}; out[n].len = wire_encode_map_cleared(out[n].bytes); n++;
    wire_move_start_t ms = {.move = 12, .command = 17, .kind = WIRE_MOVE_GOTO, .parameter = 4.2f, .t_us = 4300000000ull,
                            .pre_ms = 200, .post_ms = 300, .flags = WIRE_MOVE_REFUSED};
    out[n] = (sample_t){.name = "move_start"}; out[n].len = wire_encode_move_start(&ms, out[n].bytes); n++;
    wire_move_end_t me = {.move = 12, .end = WIRE_END_TILT, .t_us = 0xffffffffffffull, .frames_sent = 41, .frames_dropped = 2,
                          .moved_m = 0.31f, .rotated_rad = -0.125f, .imu_sent = 44, .imu_dropped = 3};
    out[n] = (sample_t){.name = "move_end"}; out[n].len = wire_encode_move_end(&me, out[n].bytes); n++;
    wire_imu_batch_t ten = sample_batch(10), one = sample_batch(1);
    out[n] = (sample_t){.name = "imu_batch"}; out[n].len = wire_encode_imu_batch(&ten, out[n].bytes); n++;
    out[n] = (sample_t){.name = "imu_batch_one"}; out[n].len = wire_encode_imu_batch(&one, out[n].bytes); n++;
    // A tof frame at the format's limits: every kind, beyond, the caps, a negative pose.
    static posed_frame_t f;
    memset(&f, 0, sizeof f);
    f.frame = (tof_frame_t){.number = 4000000000u, .t_us = 0xfffffff0u, .period_us = 66667, .beyond_mm = TOF_EXACT};
    f.x_m = -1.234f; f.y_m = 32.0f; f.yaw_rad = -3.0f; f.pitch_rad = -0.0157f; f.turn_rad = 0.1f; f.still = true; f.pose_known = true;
    for (int z = 0; z < TOF_ZONES; z++)
        f.frame.zone[z] = (tof_zone_t){.distance_mm = (uint16_t)(z * 47), .signal = (uint16_t)(z * 9), .sigma_mm = (uint16_t)(z * 5),
                                       .kind = (uint8_t)(z % 4), .targets = (uint8_t)(z % 5)};
    f.frame.zone[63].distance_mm = 3000; // beyond 2.54 m
    out[n] = (sample_t){.name = "tof_compact"}; out[n].len = wire_encode_tof(TOF_USE_MAP, 12, 4300000000ull, &f, out[n].bytes); n++;
    const uint8_t part[] = {'S', 'N', 'A', 'P', 1, 2, 0xfe, 0xff};
    out[n] = (sample_t){.name = "snapshot_part"}; out[n].len = wire_encode_snapshot_part(part, sizeof part, out[n].bytes); n++;
    wire_command_t c = {.n = 17, .text = "drive 50"};
    out[n] = (sample_t){.name = "command"}; out[n].len = wire_encode_command(&c, out[n].bytes); n++;
    out[n] = (sample_t){.name = "heartbeat"}; out[n].len = wire_encode_heartbeat(out[n].bytes); n++;
    return n;
}

// The table as one line: each known type's number, lane, min and max length.
static void table_line(char *out, size_t size) {
    size_t at = 0;
    for (int t = 0; t < 256; t++) {
        wire_type_info_t i;
        if (!wire_info((uint8_t)t, &i)) continue;
        at += (size_t)snprintf(out + at, size - at, "%s%02x:%d:%u:%u", at ? " " : "", t, (int)i.lane, (unsigned)i.min,
                               (unsigned)i.max);
    }
}

static void hex(const uint8_t *b, uint32_t n, char *out) {
    for (uint32_t i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", b[i]);
    out[2 * n] = 0;
}

static void write_golden(void) {
    FILE *f = fopen(GOLDEN, "w");
    assert(f);
    fprintf(f, "# The wire's golden bytes (picoA/app/wire.h, pc/robot/src/wire.ts): both sides encode\n"
               "# the same samples and must give these bytes, and decode these bytes to them.\n"
               "# Written by build/test_wire --write; read by test_wire.c and wire.test.ts.\n");
    fprintf(f, "version %d\n", WIRE_VERSION);
    char line[2048];
    table_line(line, sizeof line);
    fprintf(f, "table %s\n", line);
    sample_t s[24];
    uint32_t n = samples(s);
    for (uint32_t i = 0; i < n; i++) { hex(s[i].bytes, s[i].len, line); fprintf(f, "%s %s\n", s[i].name, line); }
    fclose(f);
    printf("wrote %s\n", GOLDEN);
}

static uint32_t unhex(const char *text, uint8_t *out) {
    uint32_t n = 0;
    for (; text[0] && text[1] && text[0] != '\n'; text += 2) { unsigned v; sscanf(text, "%2x", &v); out[n++] = (uint8_t)v; }
    return n;
}

static void test_golden(void) {
    FILE *f = fopen(GOLDEN, "r");
    assert(f && "picoA/app/test/data/wire_golden.hex missing: build/test_wire --write");
    sample_t s[24];
    uint32_t n = samples(s), seen = 0;
    char line[4096], name[64], table[2048];
    table_line(table, sizeof table);
    bool version_ok = false, table_ok = false;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        char *rest = strchr(line, ' ');
        assert(rest);
        *rest++ = 0;
        rest[strcspn(rest, "\n")] = 0;
        snprintf(name, sizeof name, "%s", line);
        if (!strcmp(name, "version")) { version_ok = atoi(rest) == WIRE_VERSION; continue; }
        if (!strcmp(name, "table")) { table_ok = !strcmp(rest, table); continue; }
        uint8_t bytes[WIRE_HEADER + TOF_FORMAT_MAX];
        uint32_t len = unhex(rest, bytes);
        bool found = false;
        for (uint32_t i = 0; i < n; i++) {
            if (strcmp(s[i].name, name)) continue;
            found = true;
            if (len != s[i].len || memcmp(bytes, s[i].bytes, len)) {
                printf("%s: the bytes differ from %s (changed the wire? then raise WIRE_VERSION and write it again)\n", name, GOLDEN);
                assert(0);
            }
            seen++;
        }
        assert(found);
        // The command, as the PC encodes it, decodes here.
        if (!strcmp(name, "command")) {
            wire_message_t m = take_whole(bytes, len);
            wire_command_t c;
            assert(wire_decode_command(&m, &c) && c.n == 17 && !strcmp(c.text, "drive 50"));
        }
    }
    fclose(f);
    if (!table_ok) printf("the table differs from %s:\n  here: %s\n", GOLDEN, table);
    assert(version_ok && table_ok && seen == n);
    printf("ok: the golden bytes (%u messages and the table) match %s\n", (unsigned)n, GOLDEN);
}

int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "--write")) { write_golden(); return 0; }
    test_table();
    test_round_trips();
    test_envelope();
    test_golden();
    printf("wire: all tests passed\n");
    return 0;
}
