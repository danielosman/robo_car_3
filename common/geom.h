#pragma once
// Angles and directions, robot frame (x forward, y left, z up; + = left).
#include <math.h>
#include "units.h"

// The bearing of a direction (any length): 0 ahead, + to the left, −π…π.
static inline float bearing_rad(const float v[3]) { return atan2f(v[1], v[0]); }

// The same angle in −π < a ≤ π.
static inline float wrap_pi(float a) {
    while (a > PI_F) a -= 2 * PI_F;
    while (a <= -PI_F) a += 2 * PI_F;
    return a;
}
