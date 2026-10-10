#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/rand.h"
#include "pico/stdio/driver.h"
#include "tof.h"
#include "body.h"
#include "wifi_console.h"
#include "recorder.h"

#define REC_VERSION  3      // 3: UDP datagrams, compact ToF halves; 2: WIFI records
#define DGRAM_MAX    1400   // bytes per datagram: under the 1472 that fit in one WiFi frame
#define FLUSH_US     50000  // small records wait at most this long for a datagram to fill
#define MARK_MAX     160    // longer console lines are split
#define WIFI_US      500000 // a WIFI record this often during a take
#define HALF_ZONES   32     // a ToF frame goes as two records of 32 zones (one alone won't fit)
#define ODOM_COPIES  4      // ODOM records of a datagram repeated in the next one
#define START_SENDS  2      // the datagram with TAKE_START and GEOMETRY is sent this often
#define END_SENDS    3      // the one with TAKE_END too

const VL53L8CX_ResultsData *rangefinder_raw(void); // rangefinder.c

// The records (doc/TELEMETRY_PLAN.md §2.1): [type u8][length u16][payload][CRC-32 u32]
// of type, length and payload; little-endian, packed. They travel in UDP datagrams:
// a header (rec_dgram_t), then whole records.
enum { REC_TAKE_START = 1, REC_GEOMETRY, REC_TOF_RAW, REC_ODOM, REC_DRIVE, REC_MARK, REC_TAKE_END, REC_WIFI };
#define REC_HEAD 3
#define REC_CRC  4

typedef struct __attribute__((packed)) {
    uint8_t version;     // REC_VERSION
    uint32_t boot_id;
    uint32_t seq;        // counts the datagrams since power-up; a repeat keeps its number
} rec_dgram_t;

typedef struct __attribute__((packed)) {
    uint8_t version;     // REC_VERSION
    uint32_t boot_id;    // random at power-up
    uint16_t take_no;    // from 1 at power-up
    uint32_t t_us;       // PicoA's clock, as every t_us here
    char key;            // the key that started it
    char action[12];     // "scan", "move", "turn", "watch", "record"; NUL-padded
    float param;         // scan, turn: rad (+ = left); move: m (+ = forward); watch, record: s
    char build[24];      // when the firmware was built ("Oct 10 2026 12:34:56")
} rec_take_start_t;

typedef struct __attribute__((packed)) {
    float sensor_m[3];   // where the sensor is: forward, left, up from the floor (robot frame)
    float zone_rad;      // the angle between neighbouring rays
    uint8_t rows, cols;
    uint8_t zone_of_ray[RANGEFINDER_RAYS]; // ray row * cols + col reads this sensor zone
} rec_geometry_t;

typedef struct __attribute__((packed)) {
    uint32_t frame_no;   // counts every frame the sensor gave, sent or not
    uint32_t t_us;       // the middle of the measurement
    uint16_t skipped;    // frames not sent since the last one sent in this take
    int8_t temp_c;       // the sensor's silicon
    uint8_t first_zone;  // this record holds zones first_zone .. first_zone + zones - 1
    uint8_t zones;
    // then per zone, in the sensor's order (zone_of_ray): rec_zone_t, then its
    // targets (at most 4, closest first): rec_target_t each
} rec_tof_head_t;

typedef struct __attribute__((packed)) {
    uint8_t targets;
    uint16_t ambient_per_spad; // kcps/SPAD (the ULD's u32, capped at 65535)
    uint16_t spads;            // SPADs enabled (capped too)
} rec_zone_t;

typedef struct __attribute__((packed)) {
    int16_t distance_mm;
    uint16_t sigma_mm;
    uint16_t signal_per_spad;  // kcps/SPAD (capped at 65535)
    uint8_t reflectance;       // percent
    uint8_t status;            // 5, 6, 9: valid
} rec_target_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;             // when PicoB measured it (body_odom_get())
    odom_report_t report;      // as PicoB sent it (link_msgs.h), its own t_us included
    float gyro_bias_radps;
} rec_odom_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;             // when it was sent
    float v_mps, w_radps;
} rec_drive_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint8_t reason;            // rec_end_t
    uint32_t records;          // records sent in the take before this one
    uint32_t records_failed;   // records in datagrams the WiFi refused
    uint32_t frames;           // ToF frames sent (both halves)
    uint32_t frames_failed;    // ToF frames with a half refused
    uint32_t bytes;            // sent in the take before this record, datagram headers included
    uint32_t datagrams;        // sent in the take (repeats included)
    uint32_t datagrams_failed; // refused by lwIP / the WiFi chip
} rec_take_end_t;

typedef struct __attribute__((packed)) {
    uint32_t waiting, unacked;
    uint16_t segments;
    uint8_t retries;
    uint16_t rto_ms;
    uint32_t cwnd, peer_window, write_errors, output_errors;
} rec_conn_t; // 29 bytes

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    int8_t rssi_dbm;
    uint32_t console_silent_ms; // since the server was last heard on the console
    rec_conn_t console;         // the console's TCP connection
    uint32_t datagrams, datagrams_failed; // recordings since power-up
} rec_wifi_t; // 46 bytes

#define REC_MAX(payload) (REC_HEAD + (payload) + REC_CRC)
#define TOF_HALF_MAX (sizeof(rec_tof_head_t) + HALF_ZONES * (sizeof(rec_zone_t) + \
                      VL53L8CX_NB_TARGET_PER_ZONE * sizeof(rec_target_t)))
#define ODOM_REC REC_MAX(sizeof(rec_odom_t))

static bool enabled, in_take, cut;
static uint32_t boot_id, connection; // connection: the console connection the take goes with
static uint16_t take_no;
static char key = '?';
static rec_take_end_t counts;
static uint16_t skipped;
static uint32_t odom_taken, drives_seen;
static uint32_t last_wifi_us;
static char line[MARK_MAX];
static int line_len;
// The datagram being filled; its ODOM records, to repeat in the next one.
static uint8_t dgram[DGRAM_MAX];
static uint32_t dgram_len, dgram_records, dgram_since_us, seq;
static uint8_t odom_copies[ODOM_COPIES][ODOM_REC], last_odoms[ODOM_COPIES][ODOM_REC];
static int n_odoms, n_last_odoms;
static bool dgram_has_frame_half; // for counting frames_failed
static uint32_t datagrams_total, datagrams_failed_total;

static uint32_t crc32(const uint8_t *p, uint32_t n) {
    static const uint32_t nibble[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    uint32_t c = 0xffffffffu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        c = (c >> 4) ^ nibble[c & 15];
        c = (c >> 4) ^ nibble[c & 15];
    }
    return ~c;
}

// Sends the datagram being filled `times` times (one number: the PC keeps one), with
// the previous datagram's ODOM records repeated where they fit. Never prints: it
// runs inside printf for the marks.
static void flush(int times) {
    if (dgram_records == 0) return;
    for (int i = 0; i < n_last_odoms && dgram_len + ODOM_REC <= DGRAM_MAX; i++) {
        memcpy(dgram + dgram_len, last_odoms[i], ODOM_REC);
        dgram_len += ODOM_REC;
    }
    rec_dgram_t h = {.version = REC_VERSION, .boot_id = boot_id, .seq = seq++};
    memcpy(dgram, &h, sizeof h);
    bool ok = false;
    for (int i = 0; i < times; i++) {
        bool sent = wifi_console_data_send(dgram, dgram_len);
        ok |= sent;
        counts.datagrams++;
        datagrams_total++;
        if (sent) counts.bytes += dgram_len;
        else { counts.datagrams_failed++; datagrams_failed_total++; }
    }
    if (!ok) counts.records_failed += dgram_records;
    if (!ok && dgram_has_frame_half) counts.frames_failed++;
    memcpy(last_odoms, odom_copies, sizeof odom_copies);
    n_last_odoms = n_odoms;
    n_odoms = 0;
    dgram_len = sizeof(rec_dgram_t);
    dgram_records = 0;
    dgram_has_frame_half = false;
}

// Frames the payload already at rec + REC_HEAD and adds it to the datagram (sent
// first if it is full), if the take's console connection is still there. Never
// prints (see flush()).
static bool put(uint8_t *rec, uint8_t type, uint32_t len) {
    if (wifi_console_data_connection() != connection) { in_take = false; cut = true; return false; }
    rec[0] = type;
    rec[1] = (uint8_t)len;
    rec[2] = (uint8_t)(len >> 8);
    uint32_t crc = crc32(rec, REC_HEAD + len), n = REC_HEAD + len + REC_CRC;
    memcpy(rec + REC_HEAD + len, &crc, REC_CRC);
    if (dgram_len + n > DGRAM_MAX) flush(1);
    if (dgram_records == 0) dgram_since_us = time_us_32();
    memcpy(dgram + dgram_len, rec, n);
    dgram_len += n;
    dgram_records++;
    counts.records++;
    if (type == REC_ODOM && n_odoms < ODOM_COPIES) memcpy(odom_copies[n_odoms++], rec, ODOM_REC);
    return true;
}

static void put_small(uint8_t type, const void *payload, uint32_t len) {
    uint8_t rec[REC_MAX(MARK_MAX + 8)];
    memcpy(rec + REC_HEAD, payload, len);
    put(rec, type, len);
}

static void mark(const char *text, int len) {
    uint8_t payload[4 + MARK_MAX];
    uint32_t t_us = time_us_32();
    memcpy(payload, &t_us, 4);
    memcpy(payload + 4, text, (size_t)len);
    put_small(REC_MARK, payload, 4 + (uint32_t)len);
}

// The console's lines during a take become marks: a third stdio output beside USB
// and WiFi.
static void capture(const char *buf, int len) {
    if (!in_take) return;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n' || line_len == MARK_MAX) {
            if (line_len) mark(line, line_len);
            line_len = 0;
        }
        if (buf[i] != '\n' && buf[i] != '\r') line[line_len++] = buf[i];
    }
}

static stdio_driver_t capture_driver = {
    .out_chars = capture,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = false,
#endif
};

static void put_odom(uint32_t n) {
    odom_report_t report;
    uint32_t t_us;
    if (!body_odom_get(n, &report, &t_us)) return;
    rec_odom_t o = {.t_us = t_us, .report = report, .gyro_bias_radps = body_status()->gyro_bias_radps};
    put_small(REC_ODOM, &o, sizeof o);
}

static void put_wifi(void) {
    wifi_link_status_t s;
    wifi_console_link_status(&s);
    const wifi_conn_status_t *c = &s.console;
    rec_wifi_t w = {.t_us = time_us_32(), .rssi_dbm = s.rssi_dbm, .console_silent_ms = s.console_silent_ms,
                    .console = {.waiting = c->waiting, .unacked = c->unacked, .segments = c->segments,
                                .retries = c->retries, .rto_ms = c->rto_ms, .cwnd = c->cwnd,
                                .peer_window = c->peer_window, .write_errors = c->write_errors,
                                .output_errors = c->output_errors},
                    .datagrams = datagrams_total, .datagrams_failed = datagrams_failed_total};
    last_wifi_us = w.t_us;
    put_small(REC_WIFI, &w, sizeof w);
}

static uint16_t cap16(uint32_t v) { return v > 0xffffu ? 0xffffu : (uint16_t)v; }

// Zones first .. first + HALF_ZONES - 1 of the latest frame.
static void put_tof_half(const range_frame_t *frame, const VL53L8CX_ResultsData *r, int first) {
    static uint8_t rec[REC_MAX(TOF_HALF_MAX)];
    uint8_t *p = rec + REC_HEAD;
    rec_tof_head_t h = {.frame_no = frame->number, .t_us = frame->t_us, .skipped = skipped,
                        .temp_c = r->silicon_temp_degc, .first_zone = (uint8_t)first, .zones = HALF_ZONES};
    memcpy(p, &h, sizeof h);
    p += sizeof h;
    for (int z = first; z < first + HALF_ZONES; z++) {
        int n = r->nb_target_detected[z];
        if (n > (int)VL53L8CX_NB_TARGET_PER_ZONE) n = VL53L8CX_NB_TARGET_PER_ZONE;
        rec_zone_t zone = {.targets = (uint8_t)n, .ambient_per_spad = cap16(r->ambient_per_spad[z]),
                           .spads = cap16(r->nb_spads_enabled[z])};
        memcpy(p, &zone, sizeof zone);
        p += sizeof zone;
        for (int t = 0; t < n; t++) {
            int i = z * (int)VL53L8CX_NB_TARGET_PER_ZONE + t;
            rec_target_t target = {.distance_mm = r->distance_mm[i], .sigma_mm = r->range_sigma_mm[i],
                                   .signal_per_spad = cap16(r->signal_per_spad[i]),
                                   .reflectance = r->reflectance[i], .status = r->target_status[i]};
            memcpy(p, &target, sizeof target);
            p += sizeof target;
        }
    }
    uint32_t len = (uint32_t)(p - (rec + REC_HEAD));
    if (dgram_len + REC_MAX(len) > DGRAM_MAX) flush(1); // each half starts a datagram: a lost one costs one half
    if (put(rec, REC_TOF_RAW, len)) dgram_has_frame_half = true;
}

void recorder_init(void) {
    boot_id = get_rand_32();
    dgram_len = sizeof(rec_dgram_t);
    stdio_set_driver_enabled(&capture_driver, true);
}

void recorder_set(bool on) {
    if (!on) recorder_take_end(REC_END_RECORDING_OFF);
    enabled = on;
}

bool recorder_on(void) { return enabled; }

bool recorder_ready(void) { return enabled && wifi_console_data_connection() != 0; }

void recorder_key(char k) {
    key = k;
    if (!in_take) return;
    char text[12];
    int n = k == ' ' ? snprintf(text, sizeof text, "key space") : snprintf(text, sizeof text, "key %c", k);
    mark(text, n);
}

void recorder_take_start(const char *action, float param) {
    recorder_take_end(REC_END_REPLACED);
    if (!enabled) return;
    connection = wifi_console_data_connection();
    if (!connection) { printf("Not recorded: no connection to the server\n"); return; }
    memset(&counts, 0, sizeof counts);
    skipped = 0;
    line_len = 0;
    cut = false;
    n_odoms = n_last_odoms = 0;
    dgram_len = sizeof(rec_dgram_t); // what a cut take left goes
    dgram_records = 0;
    dgram_has_frame_half = false;
    take_no++;
    in_take = true;
    rec_take_start_t s = {.version = REC_VERSION, .boot_id = boot_id, .take_no = take_no,
                          .t_us = time_us_32(), .key = key, .param = param};
    snprintf(s.action, sizeof s.action, "%s", action);
    snprintf(s.build, sizeof s.build, "%s", __DATE__ " " __TIME__);
    put_small(REC_TAKE_START, &s, sizeof s);
    float origin[3];
    rangefinder_origin(origin);
    rec_geometry_t g = {.sensor_m = {origin[0], origin[1], origin[2]}, .zone_rad = rangefinder_zone_rad(),
                        .rows = RANGEFINDER_ROWS, .cols = RANGEFINDER_COLS};
    for (int i = 0; i < RANGEFINDER_RAYS; i++) g.zone_of_ray[i] = (uint8_t)rangefinder_zone(i);
    put_small(REC_GEOMETRY, &g, sizeof g);
    odom_taken = body_odom_count();
    put_odom(odom_taken); // the pose before the first frame
    put_wifi();
    flush(START_SENDS);
    float v, w;
    uint32_t t_us;
    drives_seen = body_drive_sent(&v, &w, &t_us);
}

void recorder_take_end(rec_end_t reason) {
    if (!in_take) return;
    if (line_len) mark(line, line_len);
    line_len = 0;
    flush(1);
    rec_take_end_t e = counts;
    e.t_us = time_us_32();
    e.reason = (uint8_t)reason;
    put_small(REC_TAKE_END, &e, sizeof e);
    if (!in_take) return; // cut just now
    flush(END_SENDS);
    in_take = false;
    printf("Recorded take %u: %lu ToF frames sent, %lu failed; %lu datagrams, %lu refused; %lu KB\n",
           (unsigned)take_no, (unsigned long)counts.frames, (unsigned long)counts.frames_failed,
           (unsigned long)counts.datagrams, (unsigned long)counts.datagrams_failed,
           (unsigned long)(counts.bytes / 1024));
}

void recorder_tof(const range_frame_t *frame) {
    const VL53L8CX_ResultsData *r = rangefinder_raw();
    if (!in_take || !r) return;
    uint32_t failed = counts.frames_failed;
    put_tof_half(frame, r, 0);
    if (in_take) put_tof_half(frame, r, HALF_ZONES);
    if (!in_take) return;
    flush(1); // whole halves go at once; the next records start a new datagram
    if (counts.frames_failed == failed) { counts.frames++; skipped = 0; }
    else if (skipped < UINT16_MAX) skipped++;
    // A frame whose both halves failed counted twice: once is enough.
    if (counts.frames_failed > failed + 1) counts.frames_failed = failed + 1;
}

void recorder_update(void) {
    if (cut) {
        cut = false;
        printf("Recording: the connection to the server was lost, take %u cut\n", (unsigned)take_no);
    }
    if (!in_take) return;
    // Every report since the last call (several when a loop iteration took long).
    uint32_t n = body_odom_count();
    if (n - odom_taken > BODY_ODOM_KEPT) odom_taken = n - BODY_ODOM_KEPT; // the older ones are gone
    while (odom_taken != n && in_take) put_odom(++odom_taken);
    if (in_take && time_us_32() - last_wifi_us >= WIFI_US) put_wifi();
    float v, w;
    uint32_t t_us, sent = body_drive_sent(&v, &w, &t_us);
    if (sent != drives_seen && in_take) {
        drives_seen = sent;
        rec_drive_t d = {.t_us = t_us, .v_mps = v, .w_radps = w};
        put_small(REC_DRIVE, &d, sizeof d);
    }
    if (in_take && dgram_records && time_us_32() - dgram_since_us >= FLUSH_US) flush(1);
}

void recorder_print_status(void) {
    printf("Recording: %s, server %s; %u takes since power-up (boot %08lx), %lu datagrams, %lu refused%s\n",
           enabled ? "on" : "off (R)", wifi_console_data_connection() ? "connected" : "not connected",
           (unsigned)take_no, (unsigned long)boot_id, (unsigned long)datagrams_total,
           (unsigned long)datagrams_failed_total, in_take ? "; a take running" : "");
}
