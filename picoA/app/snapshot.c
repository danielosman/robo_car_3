#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "body.h"
#include "pose.h"
#include "surroundings.h"
#include "cell_map.h"
#include "house_map.h"
#include "camera.h"
#include "opt4048.h"
#include "behaviour.h"
#include "mode.h"
#include "recorder.h"
#include "wifi_console.h"
#include "units.h"
#include "vision.h"
#include "snapshot.h"

#define SNAP_VERSION 1
#define SNAP_MAX     38912  // bytes: the fixed sections ~34.5 KB (with 96 features), the rest for tiles
#define FEATURES_WAIT_US 300000 // a snapshot waits this long at most for vision to process the frame it shows
#define POSE_US      250000 // "\001pose ..." this often
#define NO_REASON    255

enum { SEC_HEAD = 1, SEC_POSE, SEC_GEOMETRY, SEC_TOF, SEC_CAMERA, SEC_COLOR, SEC_WINDOW, SEC_TILES, SEC_FEATURES };

typedef struct __attribute__((packed)) {
    uint32_t boot_id;       // as the recordings'
    uint32_t seq;           // counts the snapshots since power-up, from 1
    uint32_t t_us;          // when it was built
    uint8_t why;            // snapshot_why_t
    char action[12];        // the action that ended last ("scan", "goto"...), NUL-padded; "" if none yet
    uint8_t end_reason;     // how it ended (rec_end_t), 255 if none yet
    uint32_t actions_ended; // since power-up
    char running[12];       // the action running now, "" if none
    uint8_t mode;           // robot_mode_t
    uint32_t map_clears;    // the map was cleared this many times (C): the PC starts the whole-house map again
    char build[24];         // when the firmware was built
} snap_head_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;          // when PicoB measured it (PicoA's clock)
    float x_m, y_m, yaw_rad, pitch_rad, roll_rad, v_mps, w_radps;
    uint8_t stationary, motors_on, stop_reason, picob_connected;
} snap_pose_t;

// Then the TOF_RAW payload of all 64 zones (recorder.h).
typedef struct __attribute__((packed)) {
    uint8_t pose_ok;        // where the robot was when the frame was measured (pose_at())
    float x_m, y_m, yaw_rad, pitch_rad;
} snap_tof_t;

// Then width x height pixels, top row first, as the robot sees it.
typedef struct __attribute__((packed)) {
    uint32_t number;        // counts the frames captured
    uint32_t last_row_us;   // when its bottom row was read out
    float line_us, exposure_us, gain;
    uint16_t width, height;
} snap_camera_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;          // when it was read
    uint32_t sequence;
    uint8_t overload;
    uint8_t raw[16];        // the OPT4048's registers 0x00-0x07 (pc/bringup/src/opt4048.ts)
} snap_color_t;

// Then layers x cells_x x cells_y bytes, [layer][x][y] (cell_map.h).
typedef struct __attribute__((packed)) {
    int16_t org_cx, org_cy; // the window's corner: world cell (10 cm) indexes
    uint8_t cells_x, cells_y, layers;
} snap_window_t;

// Then n x snap_tile_t.
typedef struct __attribute__((packed)) {
    int16_t tx, ty;
    uint8_t columns[HOUSE_TILE_BYTES]; // house_map.h's layout
} snap_tile_t;

// The camera features of the frame in the snapshot (vision.h), then n x snap_feature_t.
typedef struct __attribute__((packed)) {
    uint32_t seq;           // vision's frame count
    uint32_t camera_number; // the frame they were found in: the camera section's
    uint8_t n;
} snap_features_t;

typedef struct __attribute__((packed)) {
    uint16_t id, age;
    uint8_t flags;          // FEATURE_NEW, _MISSED, _LOST, _LEFT_IMAGE
    int16_t x16, y16;       // place, 1/16 px
    int16_t dx64, dy64;     // measured move, 1/64 px
    uint8_t strength, match; // 0-1 as 0-255
} snap_feature_t;

static uint8_t buf[SNAP_MAX];
static uint32_t len;
static bool pending;
static snapshot_why_t pending_why;
static uint32_t seq, sent_bytes, last_len, tiles_sent, connection, endings_seen;
static uint32_t last_pose_us, wait_since_us;
static bool waiting;

// Starts a section: its header, and room for `size` bytes after it; NULL if it doesn't fit.
static uint8_t *section(uint8_t type, uint32_t size) {
    if (len + 5 + size + 4 > SNAP_MAX) return NULL;
    buf[len] = type;
    memcpy(buf + len + 1, &size, 4);
    len += 5;
    uint8_t *p = buf + len;
    len += size;
    return p;
}

static void add(uint8_t type, const void *payload, uint32_t size) {
    uint8_t *p = section(type, size);
    if (p) memcpy(p, payload, size);
}

static void add_tof(void) {
    const range_frame_t *f = surroundings_last_frame();
    if (!f) return;
    static uint8_t raw[RECORDER_TOF_MAX];
    uint32_t n = recorder_tof_payload(raw, f, 0, RANGEFINDER_RAYS, 0);
    if (!n) return;
    pose_t p;
    snap_tof_t t = {.pose_ok = pose_at(f->t_us, &p)};
    if (t.pose_ok) { t.x_m = p.x_m; t.y_m = p.y_m; t.yaw_rad = p.yaw_rad; t.pitch_rad = p.pitch_rad; }
    uint8_t *out = section(SEC_TOF, sizeof t + n);
    if (!out) return;
    memcpy(out, &t, sizeof t);
    memcpy(out + sizeof t, raw, n);
}

static void add_camera(void) {
    camera_frame_t f;
    if (!camera_last_frame(&f)) return;
    snap_camera_t c = {.number = f.number, .last_row_us = f.last_row_us, .line_us = f.line_us,
                       .exposure_us = f.exposure_us, .gain = f.gain, .width = CAMERA_WIDTH, .height = CAMERA_HEIGHT};
    uint8_t *out = section(SEC_CAMERA, sizeof c + CAMERA_WIDTH * CAMERA_HEIGHT);
    if (!out) return;
    memcpy(out, &c, sizeof c);
    memcpy(out + sizeof c, f.pixels, CAMERA_WIDTH * CAMERA_HEIGHT);
}

static void add_window(void) {
    int ox, oy;
    const uint8_t *cells = cell_map_window(&ox, &oy);
    snap_window_t w = {.org_cx = (int16_t)ox, .org_cy = (int16_t)oy, .cells_x = CELL_MAP_GRID,
                       .cells_y = CELL_MAP_GRID, .layers = CELL_LAYERS};
    uint32_t n = CELL_LAYERS * CELL_MAP_GRID * CELL_MAP_GRID;
    uint8_t *out = section(SEC_WINDOW, sizeof w + n);
    if (!out) return;
    memcpy(out, &w, sizeof w);
    memcpy(out + sizeof w, cells, n);
}

// The dirty tiles, as many as fit; the others go with the next snapshot.
static void add_tiles(void) {
    uint32_t room = SNAP_MAX - len - 5 - 4 - 2;
    uint16_t n = 0;
    uint32_t at = len + 5 + 2; // past the section's header and the count
    for (int i = house_map_next_dirty(0); i >= 0 && (uint32_t)(n + 1) * sizeof(snap_tile_t) <= room;
         i = house_map_next_dirty(i + 1)) {
        int16_t tx, ty;
        const uint8_t *columns;
        house_map_tile(i, &tx, &ty, &columns);
        snap_tile_t t = {.tx = tx, .ty = ty};
        memcpy(t.columns, columns, HOUSE_TILE_BYTES);
        memcpy(buf + at, &t, sizeof t);
        at += sizeof t;
        house_map_sent(i);
        n++;
    }
    uint8_t *out = section(SEC_TILES, 2 + n * (uint32_t)sizeof(snap_tile_t));
    memcpy(out, &n, 2);
    tiles_sent += n;
}

static int16_t fixed(float v, float scale) {
    float q = v * scale;
    return (int16_t)(q > 32767 ? 32767 : q < -32768 ? -32768 : q);
}

static void add_features(void) {
    const vision_frame_t *v = vision_latest();
    if (!v) return;
    snap_features_t h = {.seq = v->seq, .camera_number = v->camera_number, .n = v->n_features};
    uint8_t *out = section(SEC_FEATURES, sizeof h + v->n_features * (uint32_t)sizeof(snap_feature_t));
    if (!out) return;
    memcpy(out, &h, sizeof h);
    out += sizeof h;
    for (int k = 0; k < v->n_features; k++) {
        const vision_feature_t *f = &v->features[k];
        snap_feature_t p = {.id = f->id, .age = f->age, .flags = f->flags, .x16 = fixed(f->x, 16), .y16 = fixed(f->y, 16),
                            .dx64 = fixed(f->dx, 64), .dy64 = fixed(f->dy, 64),
                            .strength = (uint8_t)(f->strength * 255 + 0.5f), .match = (uint8_t)(f->match * 255 + 0.5f)};
        memcpy(out, &p, sizeof p);
        out += sizeof p;
    }
}

// The camera's last frame hasn't been through vision yet: ask for it (once a
// camera is working), and wait a little.
static bool features_behind(void) {
    camera_frame_t f;
    if (!camera_last_frame(&f)) return false;
    const vision_frame_t *v = vision_latest();
    if (v && v->camera_number == f.number) { waiting = false; return false; }
    if (!waiting) { waiting = true; wait_since_us = time_us_32(); }
    if (time_us_32() - wait_since_us >= FEATURES_WAIT_US) { waiting = false; return false; }
    vision_request();
    return true;
}

static void build(snapshot_why_t why) {
    len = 9; // "SNAP", version, length
    snap_head_t h = {.boot_id = recorder_boot_id(), .seq = ++seq, .t_us = time_us_32(), .why = (uint8_t)why,
                     .end_reason = NO_REASON, .mode = (uint8_t)mode_get(), .map_clears = house_map_clears()};
    const char *ended;
    rec_end_t reason;
    h.actions_ended = behaviour_endings(&ended, &reason);
    if (h.actions_ended) { snprintf(h.action, sizeof h.action, "%s", ended); h.end_reason = (uint8_t)reason; }
    snprintf(h.running, sizeof h.running, "%s", behaviour_action());
    snprintf(h.build, sizeof h.build, "%s", __DATE__ " " __TIME__);
    add(SEC_HEAD, &h, sizeof h);
    if (body_connected()) {
        const odom_report_t *o = body_odom();
        snap_pose_t p = {.t_us = body_odom_time_us(), .x_m = o->x_m, .y_m = o->y_m, .yaw_rad = o->yaw_rad,
                         .pitch_rad = o->pitch_rad, .roll_rad = o->roll_rad, .v_mps = o->v_mps, .w_radps = o->w_radps,
                         .stationary = o->stationary, .motors_on = o->motors_on, .stop_reason = o->stop_reason,
                         .picob_connected = 1};
        add(SEC_POSE, &p, sizeof p);
    }
    uint8_t geometry[RECORDER_GEOMETRY_SIZE];
    uint32_t geometry_len = recorder_geometry_payload(geometry);
    add(SEC_GEOMETRY, geometry, geometry_len);
    add_tof();
    add_camera();
    add_features();
    opt4048_reading_t r;
    if (opt4048_latest(&r)) {
        snap_color_t c = {.t_us = r.t_us, .sequence = r.sequence, .overload = r.overload};
        memcpy(c.raw, r.raw, sizeof c.raw);
        add(SEC_COLOR, &c, sizeof c);
    }
    add_window();
    add_tiles();
    memcpy(buf, "SNAP", 4);
    buf[4] = SNAP_VERSION;
    uint32_t total = len + 4;
    memcpy(buf + 5, &total, 4);
    uint32_t crc = recorder_crc32(buf, len);
    memcpy(buf + len, &crc, 4);
    len = total;
}

void snapshot_request(snapshot_why_t why) {
    pending = true;
    pending_why = why;
}

void snapshot_update(void) {
    uint32_t now = wifi_console_data_connection();
    if (now != connection) { // a new server: it has none of the tiles
        connection = now;
        if (now) { house_map_all_dirty(); snapshot_request(SNAP_SERVER); }
    }
    uint32_t endings = behaviour_endings(NULL, NULL);
    if (endings != endings_seen) { endings_seen = endings; snapshot_request(SNAP_ACTION_END); }
    if (!now) return;
    if (pending && !wifi_console_snapshot_busy() && !features_behind()) {
        pending = false;
        build(pending_why);
        wifi_console_snapshot_send(buf, len);
        sent_bytes += len;
        last_len = len;
    }
    if (time_us_32() - last_pose_us >= POSE_US) {
        last_pose_us = time_us_32();
        const odom_report_t *o = body_odom();
        const char *action = behaviour_action();
        char text[64];
        snprintf(text, sizeof text, "pose %.1f %.1f %.1f %s %s", (double)(o->x_m * 100), (double)(o->y_m * 100),
                 (double)(o->yaw_rad * DEG_PER_RAD), mode_name(mode_get()), *action ? action : "-");
        wifi_console_record(text);
    }
}

void snapshot_print_status(void) {
    printf("Snapshots: %lu sent (the last %lu KB), %lu KB in all, %lu map tiles; %d tiles of the whole-house map%s\n",
           (unsigned long)seq, (unsigned long)(last_len / 1024), (unsigned long)(sent_bytes / 1024),
           (unsigned long)tiles_sent, house_map_tiles(),
           wifi_console_snapshot_busy() ? "; one going out" : "");
}
