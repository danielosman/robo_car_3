// Host test for picoA/app/recorder.c: takes only with recording on and the server
// connected; the datagrams a take sends (decoded here, every record's CRC
// checked): at most 1400 B each, numbered, the first (TAKE_START, GEOMETRY, the ODOM
// before the first frame, WIFI) and the last (TAKE_END) repeated; each ToF frame as
// two records of 32 zones with every output in the compact format (values capped at
// 65535) and only the targets found; every ODOM, also several per loop iteration,
// repeated in the next datagram; DRIVE when new; the console's lines and keys as
// marks; small records sent within 50 ms; WIFI every 0.5 s; refused datagrams
// counted, the frame's number gap saying so, the lost ODOM found in the next
// datagram; a take cut when the connection is lost. The test plays body.h and the
// network (wifi_console.h). The first take's datagrams go to build/test_recorder.rec
// (the PC's file format) for the PC's decoder test. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_recorder picoA/app/test/test_recorder.c -lm && build/test_recorder
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../rangefinder.c"
#include "../recorder.c"

// --- body.h, played by the test ---
static odom_report_t odom, odoms[BODY_ODOM_KEPT + 1]; // odoms[n % ...]: report n
static uint32_t odom_count;
static status_report_t status = {.gyro_bias_radps = 0.001f};
static uint32_t drives;
static float drive_v, drive_w;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &odom; }
uint32_t body_odom_time_us(void) { return odom.t_us + 1000; }
uint32_t body_odom_count(void) { return odom_count; }
bool body_odom_get(uint32_t n, odom_report_t *r, uint32_t *t_us) {
    if (n == 0 || n > odom_count || odom_count - n >= BODY_ODOM_KEPT) return false;
    *r = odoms[n % (BODY_ODOM_KEPT + 1)];
    *t_us = r->t_us + 1000;
    return true;
}
const status_report_t *body_status(void) { return &status; }
uint32_t body_drive_sent(float *v, float *w, uint32_t *t_us) { *v = drive_v; *w = drive_w; *t_us = 77; return drives; }

// --- the network, played by the test ---
static uint32_t conn = 1;
static bool refuse;                      // the WiFi chip refuses datagrams
static uint8_t got[1 << 20];             // the datagrams sent: [len u16][datagram], one after another
static uint32_t got_len, sent_dgrams;
uint32_t wifi_console_data_connection(void) { return conn; }
bool wifi_console_data_send(const void *buf, uint32_t len) {
    assert(len <= DGRAM_MAX);
    if (!conn || refuse) return false;
    got[got_len] = (uint8_t)len;
    got[got_len + 1] = (uint8_t)(len >> 8);
    memcpy(got + got_len + 2, buf, len);
    got_len += 2 + len;
    sent_dgrams++;
    return true;
}
void wifi_console_link_status(wifi_link_status_t *s) {
    memset(s, 0, sizeof *s);
    s->rssi_dbm = -61;
    s->console.retries = 2;
    s->console.rto_ms = 1500;
    s->console_silent_ms = 300;
}

// --- reading the datagrams back ---
typedef struct { uint8_t type; uint16_t len; const uint8_t *payload; uint32_t seq; } record_t;
static record_t records[4096];
static int n_records, n_dgrams, n_repeats, n_lost; // n_lost: numbers missing between datagrams

static uint32_t crc_bitwise(const uint8_t *p, uint32_t n) {
    uint32_t c = 0xffffffffu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0xedb88320u : c >> 1;
    }
    return ~c;
}

// Splits what was sent since the last call into datagrams and records, checking
// each CRC; a repeated datagram (same number) is read once, a repeated ODOM (same
// PicoB time) too, as the PC does.
static void decode(void) {
    n_records = n_dgrams = n_repeats = n_lost = 0;
    uint32_t last_seq = 0;
    bool any = false;
    uint32_t seen_odom[512];
    int n_seen = 0;
    for (uint32_t at = 0; at < got_len;) {
        uint32_t len = got[at] | got[at + 1] << 8u;
        const uint8_t *d = got + at + 2;
        at += 2 + len;
        rec_dgram_t h;
        memcpy(&h, d, sizeof h);
        assert(h.version == REC_VERSION && h.boot_id == 0x7f3a5c01);
        if (any && h.seq == last_seq) { n_repeats++; continue; }
        assert(!any || h.seq > last_seq);
        if (any) n_lost += (int)(h.seq - last_seq - 1);
        any = true;
        last_seq = h.seq;
        n_dgrams++;
        for (uint32_t p = sizeof h; p < len;) {
            record_t r = {d[p], (uint16_t)(d[p + 1] | d[p + 2] << 8), d + p + 3, h.seq};
            uint32_t crc;
            memcpy(&crc, d + p + 3 + r.len, 4);
            assert(crc == crc_bitwise(d + p, 3u + r.len));
            p += 3u + r.len + 4u;
            assert(p <= len);
            if (r.type == REC_ODOM) {
                rec_odom_t o;
                memcpy(&o, r.payload, sizeof o);
                bool dup = false;
                for (int k = 0; k < n_seen; k++) dup |= seen_odom[k] == o.report.t_us;
                if (dup) continue;
                seen_odom[n_seen++] = o.report.t_us;
            }
            records[n_records++] = r;
        }
    }
    got_len = 0;
}

static int count(uint8_t type) {
    int n = 0;
    for (int i = 0; i < n_records; i++) n += records[i].type == type;
    return n;
}

static const record_t *find(uint8_t type, int nth) {
    for (int i = 0; i < n_records; i++)
        if (records[i].type == type && nth-- == 0) return &records[i];
    return NULL;
}

static bool has_mark(const char *text) {
    for (int i = 0; i < n_records; i++)
        if (records[i].type == REC_MARK && records[i].len == 4 + strlen(text) &&
            memcmp(records[i].payload + 4, text, strlen(text)) == 0) return true;
    return false;
}

// A ToF frame: zone z has (z % 3) targets at 100 * z + 10 * t mm; zone 1's ambient
// and signal are beyond 65535.
static void tof_frame(void) {
    for (int z = 0; z < 64; z++) {
        fake_tof.nb_target_detected[z] = (uint8_t)(z % 3);
        fake_tof.ambient_per_spad[z] = z == 1 ? 70000u : 1000u + (uint32_t)z;
        fake_tof.nb_spads_enabled[z] = 2000u + (uint32_t)z;
        for (int t = 0; t < 4; t++) {
            int i = z * 4 + t;
            fake_tof.distance_mm[i] = (int16_t)(100 * z + 10 * t);
            fake_tof.range_sigma_mm[i] = (uint16_t)(t + 1);
            fake_tof.signal_per_spad[i] = z == 1 ? 100000u : 5000u + (uint32_t)i;
            fake_tof.reflectance[i] = (uint8_t)(z % 100);
            fake_tof.target_status[i] = 5;
        }
    }
    fake_tof.silicon_temp_degc = 31;
    fake_tof_fresh = true;
    fake_now_us += 66667;
    range_frame_t f;
    assert(rangefinder_poll(&f));
    recorder_tof(&f);
}

static void odom_report(void) {
    odom.t_us += 20000;
    odom.x_m += 0.001f;
    odoms[++odom_count % (BODY_ODOM_KEPT + 1)] = odom;
}
static void print(const char *text) { fake_stdio->out_chars(text, (int)strlen(text)); }

static void test_off_or_no_connection(void) {
    recorder_take_start("scan", 6.8f); // off
    tof_frame();
    recorder_update();
    assert(got_len == 0 && !recorder_ready());
    recorder_set(true);
    conn = 0;
    assert(!recorder_ready());
    recorder_take_start("scan", 6.8f);
    tof_frame();
    assert(got_len == 0);
    conn = 1;
    assert(recorder_ready());
    printf("ok: nothing sent with recording off or no server\n");
}

// The PC's file format (pc/robot/src/recording.ts): "RCDGRAM3", then per datagram
// [len u16][arrival ms f64][datagram].
static void save_for_pc(void) {
    FILE *f = fopen("build/test_recorder.rec", "wb");
    assert(f);
    fwrite("RCDGRAM3", 1, 8, f);
    for (uint32_t at = 0; at < got_len;) {
        uint32_t len = got[at] | got[at + 1] << 8u;
        double ms = 1000.0 * at;
        fwrite(got + at, 1, 2, f);
        fwrite(&ms, 1, 8, f);
        fwrite(got + at + 2, 1, len, f);
        at += 2 + len;
    }
    fclose(f);
}

static void test_take(void) {
    recorder_key('s');
    odom_report();
    recorder_take_start("scan", 6.8f);
    print("Scan: turning 390 deg\n");
    tof_frame();
    tof_frame();
    recorder_update();      // nothing new
    odom_report();
    drives++;
    drive_w = 0.5f;
    recorder_update();      // a new report and a drive command
    recorder_update();
    recorder_key('p');
    print("x +1.0 cm  ");   // a line in two parts
    print("y +2.0 cm\n");
    recorder_take_end(REC_END_DONE);
    tof_frame();            // after the take: nothing
    recorder_update();
    save_for_pc();
    decode();
    assert(records[0].type == REC_TAKE_START && records[1].type == REC_GEOMETRY && records[2].type == REC_ODOM);
    assert(records[3].type == REC_WIFI && records[n_records - 1].type == REC_TAKE_END);
    assert(n_repeats == (START_SENDS - 1) + (END_SENDS - 1) && n_lost == 0);
    assert(count(REC_TOF_RAW) == 4 && count(REC_ODOM) == 2 && count(REC_DRIVE) == 1 && count(REC_MARK) == 3);
    assert(has_mark("Scan: turning 390 deg") && has_mark("key p") && has_mark("x +1.0 cm  y +2.0 cm"));

    rec_take_start_t s;
    memcpy(&s, records[0].payload, sizeof s);
    assert(records[0].len == sizeof s && s.version == 3 && s.boot_id == 0x7f3a5c01 && s.take_no == 1);
    assert(s.key == 's' && strcmp(s.action, "scan") == 0 && s.param == 6.8f && s.build[0]);
    rec_geometry_t g;
    memcpy(&g, records[1].payload, sizeof g);
    assert(g.rows == 8 && g.cols == 8 && g.zone_rad == ZONE_RAD && g.sensor_m[2] == SENSOR_Z_M);
    assert(g.zone_of_ray[0] == 56 && g.zone_of_ray[7] == 0 && g.zone_of_ray[8] == 57); // row 0 col 0: zone 56
    rec_wifi_t wifi;
    memcpy(&wifi, records[3].payload, sizeof wifi);
    assert(records[3].len == 46 && wifi.rssi_dbm == -61 && wifi.console.retries == 2 && wifi.console_silent_ms == 300);
    rec_odom_t o;
    memcpy(&o, find(REC_ODOM, 1)->payload, sizeof o);
    assert(o.t_us == odom.t_us + 1000 && o.report.t_us == odom.t_us && o.gyro_bias_radps == 0.001f);
    rec_drive_t d;
    memcpy(&d, find(REC_DRIVE, 0)->payload, sizeof d);
    assert(d.w_radps == 0.5f && d.t_us == 77);

    // The second frame: two halves, every zone, only its targets, capped values.
    int targets = 0;
    for (int half = 0; half < 2; half++) {
        const record_t *t = find(REC_TOF_RAW, 2 + half);
        rec_tof_head_t h;
        memcpy(&h, t->payload, sizeof h);
        uint32_t read_us = time_us_32() - 66667; // the frame after the take came later
        assert(h.skipped == 0 && h.temp_c == 31 && h.t_us == read_us - 1000000 / 15 / 2 - POLL_US / 2);
        assert(h.first_zone == 32 * half && h.zones == 32);
        const uint8_t *p = t->payload + sizeof h;
        for (int z = h.first_zone; z < h.first_zone + 32; z++) {
            rec_zone_t zone;
            memcpy(&zone, p, sizeof zone);
            p += sizeof zone;
            assert(zone.targets == z % 3 && zone.spads == 2000 + z);
            assert(zone.ambient_per_spad == (z == 1 ? 65535 : 1000 + z));
            for (int k = 0; k < zone.targets; k++, targets++) {
                rec_target_t tg;
                memcpy(&tg, p, sizeof tg);
                p += sizeof tg;
                assert(tg.distance_mm == 100 * z + 10 * k && tg.sigma_mm == k + 1 && tg.status == 5);
                assert(tg.signal_per_spad == (z == 1 ? 65535 : 5000 + z * 4 + k) && tg.reflectance == z % 100);
            }
        }
        assert(p == t->payload + t->len);
        printf("ToF half %d: %u bytes\n", half, (unsigned)t->len + 7);
    }
    assert(targets == 63);
    rec_tof_head_t first, second;
    memcpy(&first, find(REC_TOF_RAW, 0)->payload, sizeof first);
    memcpy(&second, find(REC_TOF_RAW, 2)->payload, sizeof second);
    assert(second.frame_no == first.frame_no + 1);

    rec_take_end_t e;
    memcpy(&e, records[n_records - 1].payload, sizeof e);
    assert(e.reason == REC_END_DONE && e.records == (uint32_t)n_records - 1 && e.frames == 2);
    assert(e.frames_failed == 0 && e.records_failed == 0 && e.datagrams_failed == 0);
    assert(e.datagrams == (uint32_t)(n_dgrams - 1 + START_SENDS - 1)); // before TAKE_END's
    printf("ok: a take: start, geometry, odometry, frames in halves, drive, marks, end; %d datagrams\n", n_dgrams);
}

static void test_refused(void) {
    recorder_key('a');
    recorder_take_start("watch", 60);
    tof_frame();
    odom_report();
    recorder_update();
    refuse = true; // the frame and the ODOM before it are refused
    tof_frame();
    refuse = false;
    tof_frame();   // the ODOM comes again with this one
    tof_frame();
    recorder_take_end(REC_END_STOPPED);
    decode();
    assert(count(REC_TOF_RAW) == 6 && count(REC_ODOM) == 2);
    rec_tof_head_t a, b;
    memcpy(&a, find(REC_TOF_RAW, 0)->payload, sizeof a);
    memcpy(&b, find(REC_TOF_RAW, 2)->payload, sizeof b);
    assert(b.frame_no == a.frame_no + 2 && b.skipped == 1);
    rec_take_end_t e;
    memcpy(&e, records[n_records - 1].payload, sizeof e);
    assert(e.reason == REC_END_STOPPED && e.frames == 3 && e.frames_failed == 1 && e.datagrams_failed == 1);
    assert(n_lost == 1); // the PC sees the refused datagram as a missing number
    printf("ok: refused datagrams: counted, the frame gap says so, the ODOM comes again\n");
}

// Small records don't wait for a datagram to fill: out within 50 ms.
static void test_small_records_flushed(void) {
    recorder_take_start("record", 5);
    decode();
    odom_report();
    recorder_update();
    assert(got_len == 0); // waits
    fake_now_us += 49000;
    recorder_update();
    assert(got_len == 0);
    fake_now_us += 2000;
    recorder_update();
    assert(got_len > 0);
    recorder_take_end(REC_END_DONE);
    decode();
    printf("ok: small records out within 50 ms\n");
}

// A loop iteration longer than 20 ms: several reports arrived since the last call,
// all are recorded, in order.
static void test_odom_all_kept(void) {
    recorder_take_start("watch", 60);
    for (int i = 0; i < 3; i++) odom_report();
    recorder_update();
    odom_report();
    recorder_update();
    for (int i = 0; i < BODY_ODOM_KEPT + 5; i++) odom_report(); // more than body keeps
    recorder_update();
    recorder_take_end(REC_END_DONE);
    decode();
    assert(count(REC_ODOM) == 1 + 3 + 1 + BODY_ODOM_KEPT);
    uint32_t last = 0;
    for (int i = 0; i < count(REC_ODOM); i++) {
        rec_odom_t o;
        memcpy(&o, find(REC_ODOM, i)->payload, sizeof o);
        assert(i == 0 || o.report.t_us == last + 20000 || (i == 5 && o.report.t_us == last + 6 * 20000));
        last = o.report.t_us;
    }
    printf("ok: every odometry report recorded, also several per loop iteration\n");
}

// WIFI records every 0.5 s during a take, none outside.
static void test_wifi_every_half_second(void) {
    recorder_take_start("record", 5);
    for (int i = 0; i < 2000; i++) { fake_now_us += 1000; recorder_update(); }
    recorder_take_end(REC_END_DONE);
    for (int i = 0; i < 2000; i++) { fake_now_us += 1000; recorder_update(); }
    decode();
    assert(count(REC_WIFI) == 5); // at 0, 0.5, 1, 1.5, 2 s
    printf("ok: the WiFi's status every 0.5 s during a take\n");
}

static void test_cut(void) {
    recorder_take_start("move", 0.5f);
    tof_frame();
    conn = 2; // lost and found again: the take is cut
    tof_frame();
    odom_report();
    recorder_update();
    print("after the cut\n");
    recorder_take_end(REC_END_DONE);
    decode();
    assert(count(REC_TAKE_START) == 1 && count(REC_TOF_RAW) == 2 && count(REC_TAKE_END) == 0);
    // The next take goes to the new connection; R off ends a running take.
    recorder_take_start("turn", 0.5f);
    recorder_set(false);
    tof_frame();
    decode();
    assert(count(REC_TAKE_START) == 1 && records[n_records - 1].type == REC_TAKE_END);
    rec_take_end_t e;
    memcpy(&e, records[n_records - 1].payload, sizeof e);
    assert(e.reason == REC_END_RECORDING_OFF);
    rec_take_start_t s;
    memcpy(&s, records[0].payload, sizeof s);
    assert(s.take_no == 7);
    printf("ok: a lost connection cuts the take; R off ends it\n");
}

int main(void) {
    assert(rangefinder_init());
    recorder_init();
    assert(fake_stdio && fake_stdio->out_chars);
    test_off_or_no_connection();
    test_take();
    test_refused();
    test_small_records_flushed();
    test_odom_all_kept();
    test_wifi_every_half_second();
    test_cut();
    printf("recorder: all tests passed\n");
    return 0;
}
