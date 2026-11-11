// Host test for picoA/app/snapshot.c: nothing is built without a server; a server
// connecting gets one at once; then one when a command asks (with the last action's
// name and how it ended), its command told once it has wholly gone out; one asked
// for while the last is going out comes after it.
// Each is read back here as the PC reads it: "SNAP", version, length, the sections
// (the head with the boot id and build given, the pose, the ToF frame as a compact
// frame of the snapshot use with the pose it was measured at, the camera frame, its
// features, the colour reading), the CRC. The camera features of the frame in the
// snapshot: it waits for vision to process that frame (asking it to), at most 0.3 s.
// The parts go to the bulk lane (snapshot part messages) and wait for room there; a
// snapshot has gone once its last part is in the lane; one cut by a lost connection
// is not sent again (its command is still told; the new server gets its own). The test plays body.h, the
// stamped ToF frame, the camera, the colour sensor, move and the bulk lane; the
// snapshots go to build/test_snapshot.snap for the PC's parser test. Run from the
// repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -IpicoA/drivers -Icommon/test/fakes -Icommon -o build/test_snapshot picoA/app/test/test_snapshot.c -lm && build/test_snapshot
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../tof.c"
#include "../mode.c"
#include "../tof_format.c"
#include "../wire.c"
#include "../snapshot.c"
#include "robot_mounting.h"

// --- body.h, pose.h ---
static odometry_report_t odometry = {.x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 0.75f, .pitch_rad = -0.01f, .stationary = true};
static status_report_t status;
bool body_connected(void) { return true; }
const odometry_report_t *body_odometry(void) { return &odometry; }
uint32_t body_odometry_time_us(void) { return 4242; }
uint32_t body_odometry_count(void) { return 0; }
bool body_odometry_get(uint32_t n, odometry_report_t *r, uint32_t *t_us) { (void)n; (void)r; (void)t_us; return false; }
const status_report_t *body_status(void) { return &status; }

// --- the sensors ---
static posed_frame_t tof_frame; // as main.c stamps it
static uint8_t pixels[CAMERA_WIDTH * CAMERA_HEIGHT];
bool camera_last_frame(camera_frame_t *f) {
    *f = (camera_frame_t){.pixels = pixels, .number = 77, .last_row_us = 1000, .line_us = 42.5f,
                          .exposure_us = 20000, .gain = 2.5f};
    return true;
}
bool opt4048_latest(opt4048_reading_t *r) {
    *r = (opt4048_reading_t){.sequence = 9, .t_us = 555};
    for (int i = 0; i < 16; i++) r->raw[i] = (uint8_t)(0xa0 + i);
    return true;
}

// --- vision: the features of a frame ---
static vision_frame_t vis = {.sequence = 5, .camera_number = 77, .n_features = 2,
                             .features = {{.id = 9, .age = 3, .x = 12.5f, .y = 100.25f, .dx = -0.5f, .dy = 0.75f, .strength = 0.5f, .match = 0.9f},
                                          {.id = 10, .flags = FEATURE_NEW, .x = 150, .y = 9, .strength = 1, .match = 1}}};
static bool have_vis = true;
static int vision_requests;
const vision_frame_t *vision_latest(void) { return have_vis ? &vis : NULL; }
void vision_request(void) { vision_requests++; }

// --- move ---
static uint32_t endings;
static const char *running = "";
uint32_t move_endings(move_record_t *last) {
    if (last) *last = (move_record_t){.kind = MOVE_DRIVE, .how = MOVE_END_OBSTACLE};
    return endings;
}
uint8_t move_wire_end(const move_record_t *r) { return r->how == MOVE_END_OBSTACLE ? WIRE_END_OBSTACLE : WIRE_END_ENDED; }
const char *move_kind_name(move_kind_t k) { return k == MOVE_DRIVE ? "drive" : "?"; }
const char *move_running(void) { return running; }

// --- the network: the bulk lane (link_pc.h) as the PC reads it ---
static uint32_t conn;
static uint8_t sent[1 << 20];         // the snapshot parts' bytes, one after another
static uint32_t sent_len;
static uint32_t room = 1000000;       // parts the lane takes before it is full
uint32_t link_pc_connection(void) { return conn; }
bool link_pc_send_bulk(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(conn && wire_take(message, len, &m) == (int32_t)len && m.type == WIRE_SNAPSHOT_PART);
    if (!room) return false;
    room--;
    assert(sent_len + m.len <= sizeof sent);
    memcpy(sent + sent_len, m.payload, m.len);
    sent_len += m.len;
    return true;
}

// The whole snapshots in the stream.
static uint32_t sent_count(void) {
    uint32_t n = 0, at = 0, total;
    while (at + 9 <= sent_len && (memcpy(&total, sent + at + 5, 4), at + total <= sent_len)) { at += total; n++; }
    return n;
}
#define n_sent sent_count()

// --- reading a snapshot back ---
typedef struct { const uint8_t *at[16]; uint32_t len[16]; uint32_t total; } parsed_t;

static parsed_t parse(const uint8_t *s) {
    parsed_t p = {0};
    assert(memcmp(s, "SNAP", 4) == 0 && s[4] == SNAPSHOT_VERSION);
    memcpy(&p.total, s + 5, 4);
    uint32_t crc;
    memcpy(&crc, s + p.total - 4, 4);
    assert(crc == crc32_of(s, p.total - 4));
    for (uint32_t i = 9; i < p.total - 4;) {
        uint8_t type = s[i];
        uint32_t n;
        memcpy(&n, s + i + 1, 4);
        assert(type > 0 && type < 16 && !p.at[type] && i + 5 + n <= p.total - 4);
        p.at[type] = s + i + 5;
        p.len[type] = n;
        i += 5 + n;
    }
    return p;
}

// The n-th snapshot sent (0 = the first).
static parsed_t nth(uint32_t n) {
    uint32_t at = 0;
    for (uint32_t k = 0; k < n; k++) { uint32_t t; memcpy(&t, sent + at + 5, 4); at += t; }
    return parse(sent + at);
}

static void loop_ms(int ms) {
    for (int i = 0; i < ms; i++) { fake_now_us += 1000; snapshot_update(); }
}

#define BOOT_ID 0x7a587226u

int main(void) {
    snapshot_init(BOOT_ID, "Nov  7 2026 19:00:00");
    assert(tof_init(&ROBOT_MOUNTING.tof, TOF_DEFAULT_HZ));
    for (int i = 0; i < CAMERA_WIDTH * CAMERA_HEIGHT; i++) pixels[i] = (uint8_t)(i * 13);
    // A ToF frame: zone z has z % 3 targets (the first one counts), 100 * z mm away.
    for (int z = 0; z < 64; z++) {
        fake_tof.nb_target_detected[z] = (uint8_t)(z % 3);
        for (int t = 0; t < 4; t++) { fake_tof.distance_mm[z * 4 + t] = (int16_t)(100 * z + t); fake_tof.target_status[z * 4 + t] = 5; }
    }
    fake_tof_fresh = true;
    assert(tof_poll(&tof_frame.frame));
    tof_frame.x_m = 1.2f, tof_frame.y_m = -0.45f, tof_frame.yaw_rad = 0.7f, tof_frame.pitch_rad = -0.02f;
    tof_frame.pose_known = true;
    snapshot_tof(&tof_frame);
    tof_frame.frame.number = 999; // the snapshot holds the packed frame, not a pointer to this one

    // Nothing without a server, even when asked.
    snapshot_request(SNAPSHOT_COMMAND, 0);
    loop_ms(1000);
    assert(n_sent == 0 && sent_len == 0);

    // A server: one snapshot at once.
    conn = 1;
    loop_ms(1);
    assert(n_sent == 1);
    parsed_t p = nth(0);
    snapshot_head_t h;
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(h.boot_id == BOOT_ID && h.sequence == 1 && h.why == SNAPSHOT_SERVER && h.mode == MODE_MANUAL);
    assert(!strcmp(h.build, "Nov  7 2026 19:00:00"));
    assert(h.actions_ended == 0 && h.action[0] == 0 && h.end_reason == NO_REASON && h.running[0] == 0);
    snapshot_pose_t pose;
    memcpy(&pose, p.at[SEC_POSE], sizeof pose);
    assert(pose.x_m == 1.25f && pose.y_m == -0.5f && pose.yaw_rad == 0.75f && pose.t_us == 4242 && pose.stationary);
    assert(!p.at[3] && !p.at[7] && !p.at[8]); // no sensor geometry, map window or tiles in snapshots
    // The ToF section: a compact frame (the snapshot use), its pose in it.
    static posed_frame_t tof;
    assert(p.len[SEC_TOF] == tof_format_size(tof_format_of(TOF_USE_SNAPSHOT)));
    assert(tof_format_unpack(tof_format_of(TOF_USE_SNAPSHOT), p.at[SEC_TOF], p.len[SEC_TOF], &tof));
    assert(tof.frame.number == 1 && tof.pose_known && tof.x_m == 1.2f && tof.y_m == -0.45f && fabsf(tof.yaw_rad - 0.7f) < 1e-4f);
    for (int z = 0; z < 64; z++) {
        const tof_zone_t *zone = &tof.frame.zone[z];
        int sz = sensor_zone(z); // the robot's zone z reads this sensor zone
        assert(zone->targets == sz % 3);
        assert(sz % 3 == 0 ? zone->kind == TOF_NOTHING && zone->distance_mm == 0
                           : zone->kind == TOF_SURE && zone->distance_mm == (100 * sz < 2545 ? 100 * sz : 2550)); // 2550: beyond
    }
    snapshot_camera_t cam;
    memcpy(&cam, p.at[SEC_CAMERA], sizeof cam);
    assert(cam.number == 77 && cam.width == 160 && cam.height == 120 && cam.gain == 2.5f);
    assert(memcmp(p.at[SEC_CAMERA] + sizeof cam, pixels, sizeof pixels) == 0);
    snapshot_color_t color;
    memcpy(&color, p.at[SEC_COLOR], sizeof color);
    assert(color.sequence == 9 && color.raw[15] == 0xaf);
    snapshot_features_t fh;
    memcpy(&fh, p.at[SEC_FEATURES], sizeof fh);
    assert(fh.camera_number == 77 && fh.n == 2 && p.len[SEC_FEATURES] == sizeof fh + 2 * sizeof(snapshot_feature_t));
    snapshot_feature_t f0;
    memcpy(&f0, p.at[SEC_FEATURES] + sizeof fh, sizeof f0);
    assert(f0.id == 9 && f0.age == 3 && f0.x16 == 200 && f0.y16 == 1604 && f0.dx64 == -32 && f0.dy64 == 48 && f0.match == 230);
    printf("ok: a server gets a snapshot at once: %lu bytes, every section as written\n", (unsigned long)p.total);

    // An action ending asks for nothing. Asked for by a command: built, its command
    // told once it has gone (every part in the lane).
    snapshot_sent_t told;
    endings = 1; // an action ended
    running = "";
    loop_ms(100);
    assert(n_sent == 1);
    snapshot_request(SNAPSHOT_COMMAND, 17);
    loop_ms(1);
    assert(n_sent == 2);
    p = nth(1);
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(h.sequence == 2 && h.why == SNAPSHOT_COMMAND && strcmp(h.action, "drive") == 0 && h.end_reason == WIRE_END_OBSTACLE);
    assert(snapshot_sent(&told) && told.command == 17 && told.sequence == 2 && told.x_m == 1.25f && told.yaw_rad == 0.75f);
    assert(!snapshot_sent(&told));
    // Asked again, with the robot driving now.
    running = "goto";
    mode_set(MODE_IDLE);
    snapshot_request(SNAPSHOT_COMMAND, 18);
    loop_ms(1);
    p = nth(2);
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(n_sent == 3 && h.why == SNAPSHOT_COMMAND && strcmp(h.running, "goto") == 0 && h.mode == MODE_IDLE);
    assert(snapshot_sent(&told) && told.command == 18 && told.sequence == 3);
    printf("ok: on a command, the command told once it has gone\n");

    FILE *f = fopen("build/test_snapshot.snap", "wb");
    assert(f);
    fwrite(sent, 1, sent_len, f);
    fclose(f);

    // Vision behind the camera's frame: the snapshot waits for it (asking for that
    // frame), then goes; without vision catching up, after 0.3 s.
    uint32_t before = n_sent;
    vis.camera_number = 76;
    snapshot_request(SNAPSHOT_COMMAND, 0);
    loop_ms(100);
    assert(n_sent == before && vision_requests > 0);
    vis.camera_number = 77;
    loop_ms(1);
    assert(n_sent == before + 1);
    vis.camera_number = 70;
    snapshot_request(SNAPSHOT_COMMAND, 0);
    loop_ms(250);
    assert(n_sent == before + 1);
    loop_ms(100);
    assert(n_sent == before + 2);
    printf("ok: a snapshot waits for the features of its frame, at most 0.3 s\n");

    // The lane full: the parts wait for room, none dropped; the snapshot has gone
    // once its last part is in the lane, and only then is asked for one built.
    uint32_t start = sent_len;
    vis.camera_number = 77; // vision has caught up
    room = 5;
    snapshot_request(SNAPSHOT_COMMAND, 21);
    loop_ms(1);
    assert(sent_len == start + 5 * WIRE_PART_MAX && !snapshot_sent(&told));
    snapshot_request(SNAPSHOT_COMMAND, 0); // asked while it goes out: waits
    loop_ms(10);
    assert(sent_len == start + 5 * WIRE_PART_MAX); // still full
    room = 1000000;
    loop_ms(1);
    uint32_t total;
    memcpy(&total, sent + start + 5, 4);
    assert(memcmp(sent + start, "SNAP", 4) == 0 && sent_len == start + total);
    assert(snapshot_sent(&told) && told.command == 21);
    loop_ms(1);
    assert(sent_len > start + total); // then the next one
    printf("ok: parts wait for room in the bulk lane, none dropped; the next waits for it\n");

    // The connection lost half-way: not sent again; its command is told (the PC's
    // runner says it wasn't stored); the new server gets its own snapshot.
    room = 3;
    snapshot_request(SNAPSHOT_COMMAND, 22);
    loop_ms(1);
    assert(!snapshot_sent(&told));
    conn = 0;
    loop_ms(10);
    assert(snapshot_sent(&told) && told.command == 22);
    room = 1000000;
    conn = 3;
    sent_len = 0; // a new connection: the PC's stream starts again
    loop_ms(1);
    memcpy(&total, sent + 5, 4);
    assert(memcmp(sent, "SNAP", 4) == 0 && sent_len == total && parse(sent).total == total);
    snapshot_head_t again;
    memcpy(&again, parse(sent).at[SEC_HEAD], sizeof again);
    assert(again.why == SNAPSHOT_SERVER && !snapshot_sent(&told));
    printf("ok: a snapshot cut by a lost connection isn't sent again; the new server gets its own\n");
    printf("snapshot: all tests passed\n");
    return 0;
}
