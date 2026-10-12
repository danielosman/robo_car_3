// Snapshots from a recorded drive (doc/ARCHITECTURE.md, the PC): the takes' ToF frames
// (pc/robot: npm run frames, one file per take, in order) through the robot's own map
// (picoA/app/cell_map.c, the window following the robot) and its own snapshot code
// (picoA/app/snapshot.c): one snapshot at the end of each take, as the robot sends
// them, written as a raw snapshot file the server imports (npm run import -- <file>).
// There are no camera pictures or colour readings in recordings: those sections are
// left out; each frame's ToF zones have their nearest sure target only. Built and run
// by pc/robot/src/snapreplay.ts:
//   replay_snapshots <out.snap> <boot id hex> <start, ms since 1970> <action>=<frames file> ...
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rangefinder.c"
#include "cell_map.c"
#include "house_map.c"
#include "recorder.c"
#include "mode.c"
#include "snapshot.c"

// --- what the robot would know: the frame being replayed and the pose then ---
static range_frame_t frame;
static pose_t pose;
static odom_report_t odom;
static status_report_t status;
static bool have_frame;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &odom; }
uint32_t body_odom_time_us(void) { return frame.t_us; }
uint32_t body_odom_count(void) { return 0; }
bool body_odom_get(uint32_t n, odom_report_t *r, uint32_t *t_us) { (void)n; (void)r; (void)t_us; return false; }
const status_report_t *body_status(void) { return &status; }
uint32_t body_drive_sent(float *v, float *w, uint32_t *t_us) { *v = *w = 0; *t_us = 0; return 0; }
bool pose_at(uint32_t t_us, pose_t *p) { (void)t_us; *p = pose; return have_frame; }
const range_frame_t *surroundings_last_frame(void) { return have_frame ? &frame : NULL; }
bool camera_last_frame(camera_frame_t *f) { (void)f; return false; }
bool opt4048_latest(opt4048_reading_t *r) { (void)r; return false; }
const vision_frame_t *vision_latest(void) { return NULL; }
void vision_request(void) {}
static uint32_t endings;
static const char *ended = "";
uint32_t behaviour_endings(const char **action, rec_end_t *reason) {
    if (action) *action = ended;
    if (reason) *reason = REC_END_DONE;
    return endings;
}
const char *behaviour_action(void) { return ""; }

// --- the server: the snapshot file ---
static FILE *out;
static double arrival_ms = 1.7e12;
uint32_t wifi_console_data_connection(void) { return 1; }
bool wifi_console_data_send(const void *b, uint32_t n) { (void)b; (void)n; return false; }
void wifi_console_link_status(wifi_link_status_t *s) { memset(s, 0, sizeof *s); }
bool wifi_console_snapshot_busy(void) { return false; }
bool wifi_console_snapshot_send(const uint8_t *b, uint32_t n) {
    fwrite(&arrival_ms, 8, 1, out);
    fwrite(b, 1, n, out);
    return true;
}
void wifi_console_record(const char *text) { (void)text; }

// The frame as the sensor's raw output: the nearest sure target per zone.
static void raw_of(const range_frame_t *f) {
    memset(&fake_tof, 0, sizeof fake_tof);
    for (int ray = 0; ray < RANGEFINDER_RAYS; ray++) {
        int z = rangefinder_zone(ray), i = z * (int)VL53L8CX_NB_TARGET_PER_ZONE;
        if (f->range_mm[ray] == RANGE_NO_TARGET) continue;
        fake_tof.nb_target_detected[z] = 1;
        fake_tof.distance_mm[i] = f->range_mm[ray] == RANGE_INVALID ? 0 : (int16_t)f->range_mm[ray];
        fake_tof.target_status[i] = f->range_mm[ray] == RANGE_INVALID ? f->status[ray] : 5;
    }
    fake_tof_fresh = true;
    range_frame_t ignored;
    rangefinder_poll(&ignored); // makes it rangefinder_raw()
}

// One take's frames into the map; false if the file can't be read.
static bool replay(const char *path, float *t0) {
    FILE *in = fopen(path, "r");
    if (!in) { perror(path); return false; }
    static char line[4096];
    while (fgets(line, sizeof line, in)) {
        float t;
        int n;
        char *s = line;
        range_frame_t f = {0};
        pose_t p = {0};
        if (sscanf(s, "%u %f %f %f %f %f%n", &f.number, &t, &p.x_m, &p.y_m, &p.yaw_rad, &p.pitch_rad, &n) != 6) continue;
        s += n;
        for (int i = 0; i < RANGEFINDER_RAYS; i++) {
            unsigned mm, st;
            if (sscanf(s, "%u %u%n", &mm, &st, &n) != 2) { fprintf(stderr, "%s: bad line\n", path); exit(1); }
            s += n;
            f.range_mm[i] = (uint16_t)mm;
            f.status[i] = (uint8_t)st;
        }
        *t0 += 1.0f / 15;
        f.t_us = (uint32_t)(*t0 * 1e6f);
        frame = f;
        pose = p;
        have_frame = true;
        odom = (odom_report_t){.x_m = p.x_m, .y_m = p.y_m, .yaw_rad = p.yaw_rad, .pitch_rad = p.pitch_rad, .stationary = true};
        raw_of(&f);
        cell_map_follow(p.x_m, p.y_m);
        cell_map_add(&f, &p);
        arrival_ms += 1000.0 / 15;
    }
    fclose(in);
    return true;
}

int main(int argc, char **argv) {
    if (argc < 5) { fprintf(stderr, "usage: replay_snapshots <out.snap> <boot id hex> <start ms> <action>=<frames file> ...\n"); return 2; }
    out = fopen(argv[1], "wb");
    if (!out) { perror(argv[1]); return 1; }
    fwrite("RCSNAP01", 1, 8, out);
    boot_id = (uint32_t)strtoul(argv[2], NULL, 16);
    arrival_ms = strtod(argv[3], NULL);
    assert(rangefinder_init());
    cell_map_clear();
    mode_set(MODE_MANUAL);
    float t = 0;
    snapshot_update(); // the server connects: a first snapshot
    for (int a = 4; a < argc; a++) {
        char *eq = strchr(argv[a], '=');
        if (!eq) { fprintf(stderr, "%s: not <action>=<file>\n", argv[a]); return 2; }
        *eq = 0;
        if (!replay(eq + 1, &t)) return 1;
        ended = argv[a];
        endings++;
        snapshot_update();
        int ox, oy;
        cell_map_window(&ox, &oy);
        printf("take %d (%s): snapshot %lu, robot at (%.2f, %.2f) m, window from (%.1f, %.1f) m, %d tiles\n", a - 3, argv[a],
               (unsigned long)seq, (double)pose.x_m, (double)pose.y_m, (double)(ox * 0.1f), (double)(oy * 0.1f), house_map_tiles());
    }
    fclose(out);
    return 0;
}
