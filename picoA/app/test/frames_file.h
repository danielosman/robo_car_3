#pragma once
// Recorded ToF frames as text (picoA/app/test/data; exported from the old recordings,
// tag data-v1, with its pc/robot `npm run frames`), one
// line per frame: frame number, t (s), x, y, yaw, pitch, then per zone (row order)
// the first valid distance in mm (0 nothing in range, 65535 not sure) and its status
// (if not sure, the first target's). Lines from before 7 Nov also hold a second
// target per zone; it is dropped, as tof.c drops it. Signal and sigma weren't
// exported: 0. For test_map_replay.c, which keeps them (the 7 cm data, its own mounting).
#include <stdio.h>
#include <string.h>
#include "../tof_frame.h"
#include "geometry.h"

#define FRAMES_FILE_HZ 15 // every frames file so far was recorded at 15 Hz

// One line into *f (its pose as recorded, the turn unknown: 0); false if it isn't a
// frame line.
static bool frames_file_line(const char *line, posed_frame_t *f) {
    memset(f, 0, sizeof *f);
    unsigned number;
    float t_s, x, y, yaw, pitch;
    int used;
    if (sscanf(line, "%u %f %f %f %f %f%n", &number, &t_s, &x, &y, &yaw, &pitch, &used) != 6) return false;
    const char *s = line + used;
    f->frame.number = number;
    f->frame.t_us = (uint32_t)(t_s * 1e6f);
    f->frame.period_us = 1000000 / FRAMES_FILE_HZ;
    f->frame.beyond_mm = TOF_EXACT;
    f->x_m = x;
    f->y_m = y;
    f->yaw_rad = wrap_pi(yaw);
    f->pitch_rad = pitch;
    f->pose_known = true;
    for (int i = 0; i < TOF_ZONES; i++) {
        unsigned mm, status;
        if (sscanf(s, "%u %u%n", &mm, &status, &used) != 2) return false;
        s += used;
        tof_zone_t *z = &f->frame.zone[i];
        z->targets = mm != 0;
        z->kind = mm == 0 ? TOF_NOTHING : mm == 65535 ? TOF_UNSURE : status == 5 ? TOF_SURE : TOF_LESS_SURE;
        z->distance_mm = mm == 65535 ? 0 : (uint16_t)mm;
    }
    return true;
}
