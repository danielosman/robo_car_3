// Host test for picoA/app/snapshot.c: nothing is built without a server; a server
// connecting gets one at once; then one when a command asks (with the last action's
// name and how it ended), its command told once it has wholly gone out; one asked
// for while the last is going out comes after it.
// Each is read back here as the PC reads it: "SNAP", version, length, the sections
// (the head, the pose, the sensor geometry, the ToF frame with every target and the
// pose it was measured at, the camera frame, the colour reading, the map window
// with its anchor, the dirty tiles of the whole-house map), the CRC. Tiles go once
// each, then only when changed, all again for a new server; more than fit go with
// the next snapshot. The camera features of the frame in the snapshot: it waits for
// vision to process that frame (asking it to), at most 0.3 s. The pose line every
// 0.25 s, with recording and PicoB's largest jolt and tilt. The test plays body.h,
// pose_at(), the camera, the colour sensor, behaviour and the network; the
// snapshots go to build/test_snapshot.snap for the PC's parser test. Run from the
// repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -IpicoA/drivers -Icommon/test/fakes -Icommon -o build/test_snapshot picoA/app/test/test_snapshot.c -lm && build/test_snapshot
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../rangefinder.c"
#include "../cell_map.c"
#include "../house_map.c"
#include "../recorder.c"
#include "../mode.c"
#include "../snapshot.c"

// --- body.h, pose.h ---
static odom_report_t odom = {.x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 0.75f, .pitch_rad = -0.01f, .stationary = true};
static status_report_t status;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &odom; }
uint32_t body_odom_time_us(void) { return 4242; }
uint32_t body_odom_count(void) { return 0; }
bool body_odom_get(uint32_t n, odom_report_t *r, uint32_t *t_us) { (void)n; (void)r; (void)t_us; return false; }
const status_report_t *body_status(void) { return &status; }
uint32_t body_drive_sent(float *v, float *w, uint32_t *t_us) { *v = *w = 0; *t_us = 0; return 0; }
void body_peaks_take(body_peaks_t *p) { *p = (body_peaks_t){.jolt_g = {0.25f, 0.5f, 1.25f}, .tilt_rad = 0.1f}; }
bool pose_at(uint32_t t_us, pose_t *p) { (void)t_us; *p = (pose_t){1.2f, -0.45f, 0.7f, -0.02f}; return true; }

// --- the sensors ---
static range_frame_t tof_frame;
static bool have_tof;
const range_frame_t *surroundings_last_frame(void) { return have_tof ? &tof_frame : NULL; }
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
static vision_frame_t vis = {.seq = 5, .camera_number = 77, .n_features = 2,
                             .features = {{.id = 9, .age = 3, .x = 12.5f, .y = 100.25f, .dx = -0.5f, .dy = 0.75f, .strength = 0.5f, .match = 0.9f},
                                          {.id = 10, .flags = FEATURE_NEW, .x = 150, .y = 9, .strength = 1, .match = 1}}};
static bool have_vis = true;
static int vision_requests;
const vision_frame_t *vision_latest(void) { return have_vis ? &vis : NULL; }
void vision_request(void) { vision_requests++; }

// --- behaviour ---
static uint32_t endings;
static const char *running = "";
uint32_t behaviour_endings(behaviour_end_t *last) {
    if (last) *last = (behaviour_end_t){.action = "move", .reason = REC_END_OBSTACLE};
    return endings;
}
const char *behaviour_action(void) { return running; }

// --- the network ---
static uint32_t conn;
static bool busy;                     // the last snapshot is still going out
static uint8_t sent[1 << 20];         // the snapshots, one after another
static uint32_t sent_len, n_sent;
static char records[4096];
uint32_t wifi_console_data_connection(void) { return conn; }
bool wifi_console_data_send(const void *b, uint32_t n) { (void)b; (void)n; return false; }
void wifi_console_link_status(wifi_link_status_t *s) { memset(s, 0, sizeof *s); }
int8_t wifi_console_signal_dbm(void) { return -61; }
bool wifi_console_snapshot_busy(void) { return busy; }
bool wifi_console_snapshot_send(const uint8_t *b, uint32_t n) {
    assert(!busy && conn && n <= SNAP_MAX && sent_len + n <= sizeof sent);
    memcpy(sent + sent_len, b, n);
    sent_len += n;
    n_sent++;
    busy = true;
    return true;
}
void wifi_console_record(const char *text) { strcat(records, text); strcat(records, "\n"); }

// --- reading a snapshot back ---
typedef struct { const uint8_t *at[16]; uint32_t len[16]; uint32_t total; } parsed_t;

static parsed_t parse(const uint8_t *s) {
    parsed_t p = {0};
    assert(memcmp(s, "SNAP", 4) == 0 && s[4] == SNAP_VERSION);
    memcpy(&p.total, s + 5, 4);
    uint32_t crc;
    memcpy(&crc, s + p.total - 4, 4);
    assert(crc == recorder_crc32(s, p.total - 4));
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

static uint16_t tiles_in(const parsed_t *p) { uint16_t n; memcpy(&n, p->at[SEC_TILES], 2); return n; }

static void run(int ms) {
    for (int i = 0; i < ms; i++) { fake_now_us += 1000; snapshot_update(); }
}

int main(void) {
    recorder_init();
    assert(rangefinder_init());
    cell_map_clear();
    for (int i = 0; i < CAMERA_WIDTH * CAMERA_HEIGHT; i++) pixels[i] = (uint8_t)(i * 13);
    // A ToF frame: zone z has z % 3 targets.
    for (int z = 0; z < 64; z++) {
        fake_tof.nb_target_detected[z] = (uint8_t)(z % 3);
        for (int t = 0; t < 4; t++) { fake_tof.distance_mm[z * 4 + t] = (int16_t)(100 * z + t); fake_tof.target_status[z * 4 + t] = 5; }
    }
    fake_tof_fresh = true;
    assert(rangefinder_poll(&tof_frame));
    have_tof = true;
    // The map: a blocked column in the window, and columns in two tiles of the whole-house map.
    grid[1][30][31] = C_PACK(CELL_STATE_OCCUPIED, 0, 12);
    house_map_put(-45, 0, COLUMN_BLOCKED);
    house_map_put(50, 50, COLUMN_DRIVABLE);

    // Nothing without a server, even when asked.
    snapshot_request(SNAP_COMMAND, 0);
    run(1000);
    assert(n_sent == 0 && records[0] == 0);

    // A server: one snapshot at once.
    conn = 1;
    run(1);
    assert(n_sent == 1);
    parsed_t p = nth(0);
    snap_head_t h;
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(h.boot_id == recorder_boot_id() && h.seq == 1 && h.why == SNAP_SERVER && h.mode == MODE_MANUAL);
    assert(h.actions_ended == 0 && h.action[0] == 0 && h.end_reason == NO_REASON && h.running[0] == 0);
    snap_pose_t pose;
    memcpy(&pose, p.at[SEC_POSE], sizeof pose);
    assert(pose.x_m == 1.25f && pose.y_m == -0.5f && pose.yaw_rad == 0.75f && pose.t_us == 4242 && pose.stationary);
    assert(p.len[SEC_GEOMETRY] == RECORDER_GEOMETRY_SIZE);
    snap_tof_t tof;
    memcpy(&tof, p.at[SEC_TOF], sizeof tof);
    assert(tof.pose_ok && tof.x_m == 1.2f && tof.yaw_rad == 0.7f);
    rec_tof_head_t th;
    memcpy(&th, p.at[SEC_TOF] + sizeof tof, sizeof th);
    assert(th.first_zone == 0 && th.zones == 64 && th.frame_no == tof_frame.number);
    uint32_t targets = 0;
    for (int z = 0; z < 64; z++) targets += (uint32_t)(z % 3);
    assert(p.len[SEC_TOF] == sizeof tof + sizeof th + 64 * sizeof(rec_zone_t) + targets * sizeof(rec_target_t));
    snap_camera_t cam;
    memcpy(&cam, p.at[SEC_CAMERA], sizeof cam);
    assert(cam.number == 77 && cam.width == 160 && cam.height == 120 && cam.gain == 2.5f);
    assert(memcmp(p.at[SEC_CAMERA] + sizeof cam, pixels, sizeof pixels) == 0);
    snap_color_t color;
    memcpy(&color, p.at[SEC_COLOR], sizeof color);
    assert(color.sequence == 9 && color.raw[15] == 0xaf);
    snap_window_t win;
    memcpy(&win, p.at[SEC_WINDOW], sizeof win);
    assert(win.org_cx == -30 && win.org_cy == -30 && win.cells_x == 60 && win.cells_y == 60 && win.layers == 3);
    const uint8_t *cells = p.at[SEC_WINDOW] + sizeof win;
    assert(cells[(1 * 60 + 30) * 60 + 31] == C_PACK(CELL_STATE_OCCUPIED, 0, 12));
    assert(tiles_in(&p) == 2);
    snap_tile_t tile;
    memcpy(&tile, p.at[SEC_TILES] + 2, sizeof tile);
    assert(tile.tx == -2 && tile.ty == 0); // column -45: tile -2 holds -50 … -31
    snap_features_t fh;
    memcpy(&fh, p.at[SEC_FEATURES], sizeof fh);
    assert(fh.camera_number == 77 && fh.n == 2 && p.len[SEC_FEATURES] == sizeof fh + 2 * sizeof(snap_feature_t));
    snap_feature_t f0;
    memcpy(&f0, p.at[SEC_FEATURES] + sizeof fh, sizeof f0);
    assert(f0.id == 9 && f0.age == 3 && f0.x16 == 200 && f0.y16 == 1604 && f0.dx64 == -32 && f0.dy64 == 48 && f0.match == 230);
    assert((tile.columns[(5 * 20 + 10) / 2] & 15) == COLUMN_BLOCKED);
    printf("ok: a server gets a snapshot at once: %lu bytes, every section as written\n", (unsigned long)p.total);

    // An action ending asks for nothing. While one goes out, nothing else; asked for
    // meanwhile: built when it has gone; its command told only once it has wholly gone.
    snapshot_sent_t told;
    endings = 1; // an action ended
    running = "";
    run(100);
    assert(n_sent == 1);
    snapshot_request(SNAP_COMMAND, 17);
    run(100);
    assert(n_sent == 1 && !snapshot_sent(&told));
    busy = false;
    run(1);
    assert(n_sent == 2 && !snapshot_sent(&told));
    p = nth(1);
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(h.seq == 2 && h.why == SNAP_COMMAND && strcmp(h.action, "move") == 0 && h.end_reason == REC_END_OBSTACLE);
    assert(tiles_in(&p) == 0); // all sent already
    busy = false;
    assert(snapshot_sent(&told) && told.command == 17 && told.seq == 2 && told.x_m == 1.25f && told.yaw_rad == 0.75f);
    assert(!snapshot_sent(&told));
    // Asked again, with a tile changed and the robot driving now.
    house_map_put(-45, 0, COLUMN_OPEN);
    running = "goto";
    mode_set(MODE_IDLE);
    snapshot_request(SNAP_COMMAND, 18);
    run(1);
    p = nth(2);
    memcpy(&h, p.at[SEC_HEAD], sizeof h);
    assert(n_sent == 3 && h.why == SNAP_COMMAND && strcmp(h.running, "goto") == 0 && h.mode == MODE_IDLE);
    assert(tiles_in(&p) == 1);
    busy = false;
    assert(snapshot_sent(&told) && told.command == 18 && told.seq == 3);
    printf("ok: on a command, one at a time, the command told once it has gone; a changed tile goes again\n");

    // A new server: all the tiles again. More dirty tiles than fit: the rest next time.
    busy = false;
    for (int t = 0; t < 40; t++) house_map_put(t * 20, 0, COLUMN_DRIVABLE);
    conn = 2;
    run(1);
    p = nth(3);
    int first = tiles_in(&p);
    assert(n_sent == 4 && first < house_map_tiles() && p.total <= SNAP_MAX);
    busy = false;
    snapshot_request(SNAP_COMMAND, 0);
    run(1);
    p = nth(4);
    printf("a new server, %d tiles: %d in the first snapshot (%lu bytes), %d in the next\n", house_map_tiles(), first,
           (unsigned long)nth(3).total, tiles_in(&p));
    assert(first + tiles_in(&p) == house_map_tiles());
    printf("ok: a new server gets every tile; what doesn't fit goes with the next\n");

    // Where the robot is: every 0.25 s.
    records[0] = 0;
    run(1000);
    int poses = 0;
    for (const char *r = records; (r = strstr(r, "pose ")); r++) poses++;
    assert(poses == 4 && strstr(records, "pose 125.0 -50.0 43.0 idle goto -61 0 0.25 0.50 1.25 5.7\n"));
    printf("ok: the pose line every 0.25 s, with the WiFi signal, recording, jolt and tilt\n");

    FILE *f = fopen("build/test_snapshot.snap", "wb");
    assert(f);
    fwrite(sent, 1, sent_len, f);
    fclose(f);

    // Vision behind the camera's frame: the snapshot waits for it (asking for that
    // frame), then goes; without vision catching up, after 0.3 s.
    uint32_t before = n_sent;
    busy = false;
    vis.camera_number = 76;
    snapshot_request(SNAP_COMMAND, 0);
    run(100);
    assert(n_sent == before && vision_requests > 0);
    vis.camera_number = 77;
    run(1);
    assert(n_sent == before + 1);
    busy = false;
    vis.camera_number = 70;
    snapshot_request(SNAP_COMMAND, 0);
    run(250);
    assert(n_sent == before + 1);
    run(100);
    assert(n_sent == before + 2);
    printf("ok: a snapshot waits for the features of its frame, at most 0.3 s\n");
    printf("snapshot: all tests passed\n");
    return 0;
}
