// PicoA robot firmware, the brain. M1: talks to PicoB over the link; a serial
// monitor on USB shows the link and odometry and runs the calibration tests.
#include "pico/stdlib.h"
#include "body.h"
#include "robot_test.h"
#include "debug_console.h"

int main(void) {
    stdio_init_all();
    body_init();
    for (;;) {
        body_update();
        robot_test_update();
        debug_console_update();
    }
}
