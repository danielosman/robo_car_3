#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "link.h"
#include "body.h"
#include "drive_test.h"
#include "debug_console.h"

#define STATUS_PERIOD_US 500000
#define RAD_TO_DEG 57.29578f
// printf takes doubles; the (double) casts below are for printing only.

static bool was_connected;
static absolute_time_t next_status;

static void help(void) {
    printf("Keys: g = motors on, s = stop (motors off), q = square test, l = link counters, h = help\n");
}

static void print_link(void) {
    link_stats_t s = link_stats();
    printf("Link: rx %lu bytes, %lu msgs, %lu bad, %lu lost; tx %lu msgs, %lu dropped\n",
           (unsigned long)s.rx_bytes, (unsigned long)s.rx_msgs, (unsigned long)s.rx_bad,
           (unsigned long)s.rx_lost, (unsigned long)s.tx_msgs, (unsigned long)s.tx_dropped);
}

static void print_status(void) {
    if (!body_connected()) {
        link_stats_t s = link_stats();
        printf("PicoB not connected (received %lu bytes so far%s)\n", (unsigned long)s.rx_bytes,
               s.rx_bytes ? "" : ": check that PicoB runs picoB_app and the UART wiring");
        return;
    }
    const odom_report_t *o = body_odom();
    printf("x %+6.1f cm  y %+6.1f cm  yaw %+7.1f deg  v %+5.1f cm/s  w %+6.1f deg/s  "
           "pitch %+5.1f  roll %+5.1f  %s  motors %s%s%s%s\n",
           (double)(o->x_m * 100), (double)(o->y_m * 100), (double)(o->yaw_rad * RAD_TO_DEG),
           (double)(o->v_mps * 100), (double)(o->w_radps * RAD_TO_DEG),
           (double)(o->pitch_rad * RAD_TO_DEG), (double)(o->roll_rad * RAD_TO_DEG),
           o->stationary ? "still " : "moving", o->motors_on ? "on" : "off",
           o->stop_reason != STOP_NONE ? " (safety stop)" : "", o->imu_error ? "  IMU ERROR" : "",
           drive_test_running() ? "  [square test]" : "");
}

void debug_console_update(void) {
    bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
        printf("\nPicoA robot firmware (M0)\n");
        help();
        next_status = get_absolute_time();
    }
    // Everything that moves the robot is started from here for now, so losing
    // the monitor (cable pulled, window closed) stops it. Revisit when the
    // behaviour drives the robot on its own (M4).
    if (!connected && was_connected) {
        drive_test_stop();
        body_motors(false);
    }
    was_connected = connected;
    if (!connected) return;

    switch (getchar_timeout_us(0)) {
    case 'g': body_motors(true); printf("Motors on\n"); break;
    case 's': drive_test_stop(); body_motors(false); printf("Stopped, motors off\n"); break;
    case 'q': drive_test_start(); break;
    case 'l': print_link(); break;
    case 'h': case '?': help(); break;
    default: break;
    }
    if (time_reached(next_status)) {
        next_status = make_timeout_time_us(STATUS_PERIOD_US);
        print_status();
    }
}
