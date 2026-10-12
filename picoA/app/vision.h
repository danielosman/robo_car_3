#pragma once
// Camera features (doc/ARCHITECTURE.md, vision; doc/archive/CAMERA_MOTION_PLAN.md §5-6,
// milestone V1): corners found in the camera image and followed from frame to
// frame. Each processed frame gives one record (vision_frame_t) of what was
// measured: every feature's place (sub-pixel), its measured move since the
// previous processed frame, how sure the match was, and per 20 x 20 px region the
// texture, the brightness and the features' median move. Nothing in it is
// interpreted: the robot's own turn is removed by readers (V2's `ego`).
//
// Finding: FAST-9 corners in each 20 x 20 px cell with fewer than 2 features (the
// threshold lowered until a few come up, never below 3 x the pixel noise at the
// gain in use), the best by Shi-Tomasi score (7 x 7) that are 6 px from other
// features; at most 96, none within 8 px of the image's edge (the half-size
// template needs it).
// Following: each feature's 8 x 8 template is searched ±8 px around its last place
// moved by its own last measured move (V1; V2's search hint adds the robot's turn:
// the window's place only, the measured move comes from the match), coarse on the
// half-size image, then at full size with
// a parabola and Lucas-Kanade steps for the sub-pixel part. A match is taken when its
// zero-mean normalised correlation (ZNCC: blind to gain and offset, so exposure
// steps don't break tracks) is ≥ 0.8 and the best place is clearly better than the
// second best (≥ 0.05: repeated patterns fail this). Not found: MISSED (its last
// place kept), searched again; the third time in a row LOST. One that would leave
// the image: LOST and LEFT_IMAGE. A feature's templates are taken again from the new
// frame once it has moved 1 px from where they were taken or matches less than 0.9;
// kept otherwise, so a still scene's features don't wander. LOST is reported once,
// then the feature is gone.
//
// V1 runs on core 0 from the main loop, on every 3rd camera frame (and on the
// next one when a snapshot asks); V3 moves it to core 1.
#include <stdbool.h>
#include <stdint.h>
#include "pose.h"

#define VISION_MAX_FEATURES 96
#define VISION_REGION_COLS  8    // regions of 20 x 20 px
#define VISION_REGION_ROWS  6

enum { // vision_feature_t.flags
    FEATURE_NEW        = 1, // found in this frame: no measured move yet
    FEATURE_MISSED     = 2, // not found in this frame; x, y its last measured place, no measured move
    FEATURE_LOST       = 4, // given up in this frame; reported once, then gone
    FEATURE_LEFT_IMAGE = 8, // ...because it would be outside the image (with LOST)
};

typedef struct {
    uint16_t id;     // unique while it lives (wraps after 65 535 births)
    uint16_t age;    // frames since it was found; 0 when NEW
    uint8_t flags;
    float x, y;      // where it was measured in this frame, px (as the robot sees it: x right, y down)
    float dx, dy;    // measured move since the previous processed frame, px (0 when NEW or MISSED;
                     // after a miss: since its last measurement)
    float strength;  // Shi-Tomasi score over the patch's gradient energy, 0-1 (at finding)
    float match;     // ZNCC of this frame's match, 0-1 (1 when NEW)
    uint32_t t_us;   // its row's time: row0_us + y * line_us
} vision_feature_t;

typedef struct {
    uint8_t features, lost, born, missed;
    float texture;    // mean gradient magnitude, grey levels per px: low = nothing to find here
    float brightness; // mean, 0-255
    float dx, dy;     // median measured move of its found features, px (0 if none)
} vision_region_t;

typedef struct {     // the robot at the frame's rows, from pose_at()
    bool ok;         // false: no odometry
    pose_t first_row, last_row;
} vision_pose_t;

typedef struct {
    uint32_t seq;           // counts processed frames (readers see gaps)
    uint32_t camera_number; // the driver's frame number
    uint32_t row0_us;       // the middle of row 0's exposure
    float line_us;          // row r at row0_us + r * line_us
    uint32_t t_us;          // the middle row's time: row0_us + 59.5 * line_us
    float dt_s;             // since the previous processed frame
    float exposure_us, gain;
    float brightness;       // mean over unsaturated samples, 0-255
    float saturated;        // share of samples >= 250
    vision_pose_t pose, prev_pose; // the robot during this and the previous processed frame
    uint8_t n_features, n_found;   // in the array; of which measured in this frame
    vision_feature_t features[VISION_MAX_FEATURES]; // NEW, MISSED and LOST included
    vision_region_t regions[VISION_REGION_ROWS][VISION_REGION_COLS];
} vision_frame_t;

// One camera image as vision takes it.
typedef struct {
    const uint8_t *pixels;  // 160 x 120, top row first
    uint32_t number;
    uint32_t row0_us;
    float line_us, exposure_us, gain;
} vision_image_t;

typedef struct {
    uint32_t frames;        // processed since power-up
    float ms_mean, ms_max;  // per processed frame, over the last second's
    float per_s;            // processed frames per second, over the last second
    uint32_t born_1s, lost_1s; // features born and lost in the last second
    float jitter_px;        // σ of the found features' measured moves (still scene: noise), last frame
} vision_status_t;

void vision_reset(void);   // forgets every feature
// The core, on one image (host tests call it directly); pose and prev_pose may be NULL.
const vision_frame_t *vision_process(const vision_image_t *image, const vision_pose_t *pose);
void vision_update(void);  // call every loop iteration, after motion_sense (it takes the camera's frames)
void vision_request(void); // process the next frame, whatever the rate (a snapshot wants its features)
const vision_frame_t *vision_latest(void); // NULL before the first processed frame
void vision_status(vision_status_t *status);
void vision_print_status(void); // the `v` key
