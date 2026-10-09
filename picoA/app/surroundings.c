#include "pico/stdlib.h"
#include "pose.h"
#include "cell_map.h"
#include "surroundings.h"

static bool have_frame;
static range_frame_t last;

bool surroundings_init(void) {
    cell_map_clear();
    return rangefinder_init();
}

void surroundings_update(void) {
    range_frame_t frame;
    if (!rangefinder_poll(&frame)) return;
    last = frame;
    have_frame = true;
    pose_t pose;
    if (!pose_at(frame.t_us, &pose)) return; // no odometry for that moment (PicoB not connected)
    cell_map_add(&frame, &pose);
}

void surroundings_restart(void) {
    pose_set_origin(); // the map's x axis: where the robot faces now
    cell_map_clear();
}

const range_frame_t *surroundings_last_frame(void) { return have_frame ? &last : NULL; }
