// PicoA robot firmware, the brain. M2: talks to PicoB over the link, maps the
// surroundings with the ToF sensor; a serial monitor on USB or the robot server
// over WiFi (ROBOT_WIFI.md) shows the map and odometry, starts the start-up scan
// and runs the calibration tests.
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "body.h"
#include "pose.h"
#include "surroundings.h"
#include "behaviour.h"
#include "robot_test.h"
#include "debug_console.h"
#include "wifi_console.h"

#define USB_CHECK_US 3000000 // after power-up: time for a computer to enumerate the USB

// On USB (a computer, not a charger), wait for the serial monitor and its keys.
// Without USB, connect to WiFi, then do the start-up scan whether that worked or not.
static void start_up(absolute_time_t usb_check) {
    static enum { CHECK_USB, CONNECT_WIFI, DONE } step;
    if (step == CHECK_USB) {
        if (tud_mounted()) step = DONE;
        else if (time_reached(usb_check)) { step = CONNECT_WIFI; wifi_console_start(); }
    } else if (step == CONNECT_WIFI && wifi_console_settled()) {
        step = DONE;
        behaviour_scan();
    }
}

int main(void) {
    stdio_init_all();
    absolute_time_t usb_check = make_timeout_time_us(USB_CHECK_US);
    body_init();
    bool tof_ok = surroundings_init(); // ~2 s: uploads the ToF sensor's firmware
    for (;;) {
        start_up(usb_check);
        body_update();
        pose_update();
        surroundings_update();
        behaviour_update();
        robot_test_update();
        wifi_console_update();
        debug_console_update();
        static bool warned;
        if (!tof_ok && !warned && (stdio_usb_connected() || wifi_console_connected())) {
            warned = true;
            printf("ToF sensor not working: no map\n");
        }
    }
}
