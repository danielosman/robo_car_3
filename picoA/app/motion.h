#pragma once
// Speed commands that finish a move on target: full speed, slowing down near the
// end so the robot stops close to it. Shared by the robot tests and behaviour.
#include <math.h>

#define MOTION_SPEED_MPS      0.10f
#define MOTION_MIN_SPEED_MPS  0.03f
#define MOTION_TURN_RADPS     0.50f // ~29°/s
#define MOTION_MIN_TURN_RADPS 0.15f

// Forward speed (+) or back (-) with remaining_m still to go in that direction's sign.
static inline float motion_speed(float remaining_m) {
    return copysignf(fminf(MOTION_SPEED_MPS, fmaxf(MOTION_MIN_SPEED_MPS, fabsf(remaining_m))), remaining_m);
}

// Turn rate (+ = left) with remaining_rad still to turn (+ = left).
static inline float motion_turn_rate(float remaining_rad) {
    return copysignf(fminf(MOTION_TURN_RADPS, fmaxf(MOTION_MIN_TURN_RADPS, 2 * fabsf(remaining_rad))), remaining_rad);
}
