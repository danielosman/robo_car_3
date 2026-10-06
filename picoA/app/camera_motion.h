#pragma once
// Movement in front of the camera while the robot stands still (ROBOT_PLAN.md §6.3).
// Each 8 x 8-pixel block of the 160 x 120 image is a cell (20 x 15); its value is
// its mean brightness. After camera_motion_restart() the camera's exposure is let
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
#include "motion_obs.h"

#define CAMERA_MOTION_COLS 20
#define CAMERA_MOTION_ROWS 15
#define CAMERA_MOTION_BLOCKS (CAMERA_MOTION_COLS * CAMERA_MOTION_ROWS)

// The view changed (the robot moved), or the robot starts moving: frees the
// exposure to adjust; the next frames learn the view again once it is on target.
void camera_motion_restart(void);
bool camera_motion_ready(void); // the backgrounds are learned
// Feeds one frame; fills up to max observations, the most blocks first, and returns
// how many; -1 when the frame wasn't used (skipped to keep ~15 a second, the
// exposure adjusting, the view being learned): it says nothing about movement.
int camera_motion_add(const camera_frame_t *frame, motion_obs_t *obs, int max);
uint32_t camera_motion_exposure_adjustments(void); // while still, since power-up

// For printing, per block (row by row, as the robot sees it): its background and
// noise (brightness 0-255), whether it differs now, whether it moved.
float camera_motion_background(int block);
float camera_motion_noise(int block);
bool camera_motion_differs(int block);
bool camera_motion_moved(int block);
