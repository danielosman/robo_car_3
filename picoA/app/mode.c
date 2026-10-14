#include <stdio.h>
#include "mode.h"

static robot_mode_t mode = MODE_MANUAL;

const char *mode_name(robot_mode_t m) { return m == MODE_MANUAL ? "manual" : "idle"; }

void mode_set(robot_mode_t m) {
    if (m == mode) return;
    mode = m;
    printf("Mode: %s\n", m == MODE_MANUAL ? "manual (commands steer the robot)"
                                         : "idle (the robot stands, motors off)");
}

robot_mode_t mode_get(void) { return mode; }
