// PicoA robot firmware, the brain. Talks to PicoB over the link, maps the
// surroundings with the ToF sensor, watches for movement and turns after it; a
// serial monitor on USB or the robot server over WiFi (ROBOT_WIFI.md) shows the map
// and odometry, starts the start-up scan and watching, and runs the calibration tests.
#include <stdio.h>
#include "pico/stdlib.h"
#include "clock_start.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "body.h"
#include "pose.h"
#include "surroundings.h"
#include "behaviour.h"
#include "robot_test.h"
#include "debug_console.h"
#include "wifi_console.h"
#include "camera.h"
#include "motion_sense.h"
#include "loop_stats.h"

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

static void stage_done(const char *stage) { loop_stats_stage(stage, time_us_32()); }

int main(void) {
    clock_start();
    stdio_init_all();
    absolute_time_t usb_check = make_timeout_time_us(USB_CHECK_US);
    body_init();
    bool tof_ok = surroundings_init(); // ~2 s: uploads the ToF sensor's firmware
    bool camera_ok = camera_init();
    for (;;) {
        loop_stats_begin(time_us_32());
        start_up(usb_check);
        body_update();
        pose_update();
        stage_done("body");
        surroundings_update();
        stage_done("surroundings");
        camera_update();
        stage_done("camera");
        motion_sense_update();
        stage_done("motion_sense");
        behaviour_update();
        stage_done("behaviour");
        robot_test_update();
        stage_done("robot_test");
        wifi_console_update();
        stage_done("wifi");
        debug_console_update();
        stage_done("console");
        static bool warned;
        if (!warned && (stdio_usb_connected() || wifi_console_connected())) {
            warned = true;
            if (!tof_ok) printf("ToF sensor not working: no map\n");
            if (!camera_ok) printf("Camera not working\n");
        }
    }
}
