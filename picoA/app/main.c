// PicoA robot firmware, the brain. M2: talks to PicoB over the link, maps the
// surroundings with the ToF sensor; a serial monitor on USB starts the start-up
// scan, shows the map and odometry, and runs the calibration tests.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "body.h"
#include "pose.h"
#include "surroundings.h"
#include "behaviour.h"
#include "robot_test.h"
#include "debug_console.h"

int main(void) {
    stdio_init_all();
    body_init();
    bool tof_ok = surroundings_init(); // ~2 s: uploads the ToF sensor's firmware
    for (;;) {
        body_update();
        pose_update();
        surroundings_update();
        behaviour_update();
        robot_test_update();
        debug_console_update();
        static bool warned;
        if (!tof_ok && !warned && stdio_usb_connected()) { warned = true; printf("ToF sensor not working: no map\n"); }
    }
}
