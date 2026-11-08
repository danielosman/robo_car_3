#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "body.h"
#include "pose.h"
#include "tof_frame.h"
#include "cell_map.h"
#include "house_map.h"
#include "camera.h"
#include "opt4048.h"
#include "move.h"
#include "mode.h"
#include "tof_format.h"
#include "crc32.h"
#include "link_pc.h"
#include "wire.h"
#include "vision.h"
#include "snapshot.h"

#define SNAP_VERSION 2 // 2: the ToF section a compact frame, no geometry section, the end a wire_end_t
#define SNAP_MAX     38912  // bytes: the fixed sections ~34.5 KB (with 96 features), the rest for tiles
#define FEATURES_WAIT_US 300000 // a snapshot waits this long at most for vision to process the frame it shows
#define NO_REASON    255

enum { SEC_HEAD = 1, SEC_POSE, SEC_TOF = 4, SEC_CAMERA, SEC_COLOR, SEC_WINDOW, SEC_TILES, SEC_FEATURES }; // 3 was the sensor geometry

typedef struct __attribute__((packed)) {
    uint32_t boot_id;       // as hello's
    uint32_t seq;           // counts the snapshots since power-up, from 1
    uint32_t t_us;          // when it was built
    uint8_t why;            // snapshot_why_t
    char action[12];        // the move that ended last ("drive", "turn", "goto", "watch"), NUL-padded; "" if none yet
    uint8_t end_reason;     // how it ended (wire_end_t), 255 if none yet
    uint32_t actions_ended; // moves ended since power-up
    char running[12];       // the move running now, "" if none
    uint8_t mode;           // robot_mode_t
    uint32_t map_clears;    // the map was cleared this many times (C): the PC starts the whole-house map again
    char build[24];         // when the firmware was built
} snap_head_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;          // when PicoB measured it (PicoA's clock)
    float x_m, y_m, yaw_rad, pitch_rad, roll_rad, v_mps, w_radps;
    uint8_t stationary, motors_on, stop_reason, picob_connected;
} snap_pose_t;

// The ToF section: the latest frame packed for the snapshot use (tof_format.h), its
// pose in it.

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

static uint32_t boot_id;
static const char *build_text = "";
static uint8_t buf[SNAP_MAX];
static uint32_t len;
static bool pending;
static snapshot_why_t pending_why;
static uint32_t pending_command;  // the command asking for the pending snapshot, 0 if none
static snapshot_sent_t going;     // the snapshot going out, if a command asked for it (command 0: none)
static uint32_t seq, sent_bytes, last_len, tiles_sent, connection;
static uint32_t wait_since_us;
static bool waiting;
// The snapshot going out: its parts handed to the bulk lane up to `at`.
static bool sending;
static uint32_t at;

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

// The latest ToF frame, packed for the snapshot use.
static uint8_t tof_bytes[TOF_FORMAT_MAX];
static uint32_t tof_len;

void snapshot_tof(const posed_frame_t *f) { tof_len = tof_pack(TOF_USE_SNAPSHOT, f, tof_bytes); }

void snapshot_init(uint32_t boot, const char *build) {
    boot_id = boot;
    build_text = build;
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
    snap_head_t h = {.boot_id = boot_id, .seq = ++seq, .t_us = time_us_32(), .why = (uint8_t)why,
                     .end_reason = NO_REASON, .mode = (uint8_t)mode_get(), .map_clears = house_map_clears()};
    move_record_t last;
    h.actions_ended = move_endings(&last);
    if (h.actions_ended) { snprintf(h.action, sizeof h.action, "%s", move_kind_name(last.kind)); h.end_reason = move_wire_end(&last); }
    snprintf(h.running, sizeof h.running, "%s", move_running());
    snprintf(h.build, sizeof h.build, "%s", build_text);
    add(SEC_HEAD, &h, sizeof h);
    if (body_connected()) {
        const odom_report_t *o = body_odom();
        snap_pose_t p = {.t_us = body_odom_time_us(), .x_m = o->x_m, .y_m = o->y_m, .yaw_rad = o->yaw_rad,
                         .pitch_rad = o->pitch_rad, .roll_rad = o->roll_rad, .v_mps = o->v_mps, .w_radps = o->w_radps,
                         .stationary = o->stationary, .motors_on = o->motors_on, .stop_reason = o->stop_reason,
                         .picob_connected = 1};
        add(SEC_POSE, &p, sizeof p);
    }
    if (tof_len) add(SEC_TOF, tof_bytes, tof_len);
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
    uint32_t crc = crc32_of(buf, len);
    memcpy(buf + len, &crc, 4);
    len = total;
}

void snapshot_request(snapshot_why_t why, uint32_t command) {
    pending = true;
    pending_why = why;
    if (command) pending_command = command;
}

bool snapshot_sent(snapshot_sent_t *s) {
    if (!going.command || sending) return false;
    *s = going;
    going.command = 0;
    return true;
}

// Hands the bulk lane the next parts, as many as it takes now (a part it has no room
// for waits); the snapshot has gone once every part is in the lane.
static void send_parts(void) {
    static uint8_t part[WIRE_HEADER + WIRE_PART_MAX];
    while (at < len) {
        uint32_t n = len - at > WIRE_PART_MAX ? WIRE_PART_MAX : len - at;
        uint32_t n_part = wire_encode_snapshot_part(buf + at, n, part);
        if (!link_send_bulk(part, n_part)) return;
        at += n;
    }
    sending = false;
}

void snapshot_update(void) {
    uint32_t now = link_pc_connection();
    if (now != connection) { // a new server: it has none of the tiles; one cut by a lost connection isn't sent again
        connection = now;
        sending = false;
        if (now) { house_map_all_dirty(); snapshot_request(SNAP_SERVER, 0); }
    }
    if (!now) return;
    if (sending) send_parts();
    if (pending && !sending && !going.command && !features_behind()) {
        pending = false;
        build(pending_why);
        if (pending_command) {
            const odom_report_t *o = body_odom();
            going = (snapshot_sent_t){.command = pending_command, .seq = seq, .x_m = o->x_m, .y_m = o->y_m,
                                      .yaw_rad = o->yaw_rad};
            pending_command = 0;
        }
        sending = true;
        at = 0;
        send_parts();
        sent_bytes += len;
        last_len = len;
    }
}

void snapshot_print_status(void) {
    printf("Snapshots: %lu sent (the last %lu KB), %lu KB in all, %lu map tiles; %d tiles of the whole-house map%s\n",
           (unsigned long)seq, (unsigned long)(last_len / 1024), (unsigned long)(sent_bytes / 1024),
           (unsigned long)tiles_sent, house_map_tiles(),
           sending ? "; one going out" : "");
}
