#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "units.h"
#include "link.h"
#include "body.h"
#include "robot_test.h"
#include "debug_console.h"

#define STATUS_PERIOD_US  15000000 // not while a test runs: tests print their own progress
#define GREETING_DELAY_US 1000000  // the Mac drops what arrives the moment the port opens
// printf takes doubles; the (double) casts below are for printing only.

static bool was_connected, greeted;
static absolute_time_t next_status, greeting_time;

static void help(void) {
    printf("Keys: g = motors on, s = stop (motors off), p = print status now, l = link counters, h = help\n"
           "Tests: q = square, d = drift (still), r = turns, f / b = straight forward / back,\n"
           "       t = print the last test result again\n");
}

static void print_last_result(void) {
    if (robot_test_running()) printf("A test is running\n");
    else if (*robot_test_result()) printf("Last test result:\n%s", robot_test_result());
    else printf("No test result yet\n");
}

static void greet(void) {
    printf("\nPicoA robot firmware (M1)\n");
    help();
    print_last_result();
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
           "pitch %+5.1f  roll %+5.1f  bias %+.4f deg/s  %s  motors %s%s%s\n",
           (double)(o->x_m * 100), (double)(o->y_m * 100), (double)(o->yaw_rad * DEG_PER_RAD),
           (double)(o->v_mps * 100), (double)(o->w_radps * DEG_PER_RAD),
           (double)(o->pitch_rad * DEG_PER_RAD), (double)(o->roll_rad * DEG_PER_RAD),
           (double)(body_status()->gyro_bias_radps * DEG_PER_RAD),
           o->stationary ? "still " : "moving", o->motors_on ? "on" : "off",
           o->stop_reason != STOP_NONE ? " (safety stop)" : "", o->imu_error ? "  IMU ERROR" : "");
}

void debug_console_update(void) {
    bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
        greeted = false;
        greeting_time = make_timeout_time_us(GREETING_DELAY_US);
    }
    // Losing the monitor doesn't stop the robot: a test can run with the cable
    // pulled. PicoB's safety stops (wheels, tilt) still apply.
    was_connected = connected;
    if (!connected) return;
    if (!greeted) {
        if (!time_reached(greeting_time)) return;
        greeted = true;
        greet();
        next_status = get_absolute_time();
    }

    switch (getchar_timeout_us(0)) {
    case 'g': body_motors(true); printf("Motors on\n"); break;
    case 's': robot_test_stop(); body_motors(false); printf("Stopped, motors off\n"); break;
    case 'q': robot_test_start(ROBOT_TEST_SQUARE); break;
    case 'd': robot_test_start(ROBOT_TEST_DRIFT); break;
    case 'r': robot_test_start(ROBOT_TEST_TURNS); break;
    case 'f': robot_test_start(ROBOT_TEST_FORWARD); break;
    case 'b': robot_test_start(ROBOT_TEST_BACK); break;
    case 'p': print_status(); break;
    case 't': print_last_result(); break;
    case 'l': print_link(); break;
    case 'h': case '?': help(); break;
    default: break;
    }
    if (time_reached(next_status)) {
        next_status = make_timeout_time_us(STATUS_PERIOD_US);
        if (!robot_test_running()) print_status();
    }
}
