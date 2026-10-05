#include <stdio.h>
#include "pico/stdlib.h"
#include "pose.h"
#include "world_map.h"
#include "surroundings.h"

#define KEPT_FRAMES 300 // 20 s at 15 Hz: a 390° turn at ~30°/s takes ~14 s

typedef struct { range_frame_t frame; pose_t pose; } placed_frame_t;

static placed_frame_t kept[KEPT_FRAMES];
static int n_kept;
static bool learning, mapping, have_frame;
static range_frame_t last;

bool surroundings_init(void) {
    map_clear();
    return rangefinder_init();
}

static void map_frame(const range_frame_t *frame, const pose_t *pose) {
    scan_t scan;
    rangefinder_scan(frame, pose->pitch_rad, &scan);
    map_add_scan(&scan, pose);
}

void surroundings_update(void) {
    range_frame_t frame;
    if (!rangefinder_poll(&frame)) return;
    last = frame;
    have_frame = true;
    pose_t pose;
    if (!pose_at(frame.t_us, &pose)) return; // no odometry for that moment (PicoB not connected)
    if (learning && n_kept < KEPT_FRAMES) kept[n_kept++] = (placed_frame_t){frame, pose};
    if (mapping) map_frame(&frame, &pose);
}

void surroundings_learn_start(void) {
    pose_set_origin(); // the map's x axis: where the robot faces now
    map_clear();
    rangefinder_forget_floor();
    n_kept = 0;
    learning = true;
    mapping = false;
}

int surroundings_learn_finish(void) {
    for (int i = 0; i < n_kept; i++) rangefinder_learn_floor(&kept[i].frame, kept[i].pose.pitch_rad);
    int assumed;
    int learned = rangefinder_finish_floor(&assumed);
    for (int i = 0; i < n_kept; i++) map_frame(&kept[i].frame, &kept[i].pose);
    printf("Floor learned in %d of %d zones from %d frames", learned, rangefinder_floor_zones(), n_kept);
    if (assumed) printf(" (%d more assumed from the sensor's height)", assumed);
    printf("; rows %d-%d see it at", rangefinder_first_floor_row() + 1, RANGEFINDER_ROWS);
    for (int row = rangefinder_first_floor_row(); row < RANGEFINDER_ROWS; row++)
        printf(" %.0f", (double)(rangefinder_floor_distance(row) * 100));
    printf(" cm\n");
    learning = false;
    mapping = true;
    return learned;
}

bool surroundings_mapping(void) { return mapping; }

const range_frame_t *surroundings_last_frame(void) { return have_frame ? &last : NULL; }
