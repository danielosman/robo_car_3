#pragma once
// Movement in front of the camera while the robot stands still (doc/archive/ROBOT_PLAN.md §6.3).
// Each 8 x 8-pixel block of the 160 x 120 image is a cell (20 x 15); its value is
// its mean brightness. After watch_camera_restart() the camera's exposure is let
// reach its target and is then held, and each block learns over 1 s what it
// normally reads (the middle value) and how much it wobbles (its noise); a block
// at its background follows it slowly (dusk). A block differs when it
// is off its background by clearly more than its noise; it has moved when it
// differs in 2 of the last 4 frames next to another such block, or in 3 alone;
// moved blocks that touch are reported as observations, with a direction and no
// range. A block that differs but holds steady for 1 s has stopped (a box put
// down, a light switched on): that is its background from then on. Movement is
// change: a light switched and taking something away count too, until steady.
// When the camera wants a new exposure, it gets it only when calm (nothing moved
// for 5 s, or after 30 s of waiting): the camera stops watching, the exposure is
// adjusted and the view learned again.
// Frames are used at most ~15 times a second, whatever the camera's frame rate.
#include <stdbool.h>
#include <stdint.h>
#include "camera.h"
#include "watch_obs.h"
#include "mounting.h"

#define WATCH_CAMERA_COLS 20
#define WATCH_CAMERA_ROWS 15
#define WATCH_CAMERA_BLOCKS (WATCH_CAMERA_COLS * WATCH_CAMERA_ROWS)

// The camera's focal length, for the blocks' directions (main.c: ROBOT_MOUNTING).
// Call first; false if the mounting isn't for this picture (160 × 120, f > 0): then
// every frame is unused.
bool watch_camera_init(const mounting_t *m);
// The view changed (the robot moved), or the robot starts moving: frees the
// exposure to adjust; the next frames learn the view again once it is on target.
void watch_camera_restart(void);
bool watch_camera_ready(void); // the backgrounds are learned
// Feeds one frame; fills up to max observations, the most blocks first, and returns
// how many; -1 when the frame wasn't used (skipped to keep ~15 a second, the
// exposure adjusting, the view being learned): it says nothing about movement.
int watch_camera_add(const camera_frame_t *frame, watch_obs_t *obs, int max);
uint32_t watch_camera_exposure_adjustments(void); // while still, since power-up

// For printing, per block (row by row, as the robot sees it): its background and
// noise (brightness 0-255), whether it differs now, whether it moved.
float watch_camera_background(int block);
float watch_camera_noise(int block);
bool watch_camera_differs(int block);
bool watch_camera_moved(int block);
