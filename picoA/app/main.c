// PicoA robot firmware, the brain. M0: talks to PicoB over the link; a serial
// monitor on USB shows the link and odometry and can run the square test.
#include "pico/stdlib.h"
#include "body.h"
#include "drive_test.h"
#include "debug_console.h"

int main(void) {
    stdio_init_all();
    body_init();
    for (;;) {
        body_update();
        drive_test_update();
        debug_console_update();
    }
}
