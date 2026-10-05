#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "units.h"
#include "link.h"
#include "body.h"
#include "robot_test.h"
#include "behaviour.h"
#include "surroundings.h"
#include "world_map.h"
#include "pose.h"
#include "debug_console.h"

#define STATUS_PERIOD_US  15000000 // not while a test or the scan runs: they print their own progress
#define GREETING_DELAY_US 1000000  // the Mac drops what arrives the moment the port opens
// printf takes doubles; the (double) casts below are for printing only.

static bool was_connected, greeted, scan_started;
static absolute_time_t next_status, greeting_time;

static void help(void) {
    printf("Keys: g = motors on, s = stop (motors off), p = print status now, l = link counters, h = help\n"
           "Map: n = start-up scan again (390 deg turn), m = print the map, z = one ToF frame\n"
           "Tests: q = square, d = drift (still), r = turns, f / b = straight forward / back,\n"
           "       t = print the last test result again\n");
}

static void print_last_result(void) {
    if (robot_test_running()) printf("A test is running\n");
    else if (*robot_test_result()) printf("Last test result:\n%s", robot_test_result());
    else printf("No test result yet\n");
}

static void greet(void) {
    printf("\nPicoA robot firmware (M2)\n");
    help();
    print_last_result();
}

static void print_link(void) {
    link_stats_t s = link_stats();
    printf("Link: rx %lu bytes, %lu msgs, %lu bad, %lu lost; tx %lu msgs, %lu dropped\n",
           (unsigned long)s.rx_bytes, (unsigned long)s.rx_msgs, (unsigned long)s.rx_bad,
           (unsigned long)s.rx_lost, (unsigned long)s.tx_msgs, (unsigned long)s.tx_dropped);
}

// One ToF frame as the robot sees it (top row up, left column left): the distances,
// what each zone makes of them, and what the floor zones learned.
static void print_frame(void) {
    const range_frame_t *f = surroundings_last_frame();
    if (!f) { printf("No ToF frame yet\n"); return; }
    printf("ToF frame, cm along each ray (-- nothing in range, ?N unsure: VL53 status N); as the robot sees it:\n");
    for (int row = 0; row < RANGEFINDER_ROWS; row++) {
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            int i = row * RANGEFINDER_COLS + col;
            uint16_t mm = f->range_mm[i];
            if (mm == RANGE_NO_TARGET) printf("   --");
            else if (mm == RANGE_INVALID) printf(" ?%-3u", (unsigned)f->status[i]);
            else printf(" %4u", (unsigned)((mm + 5) / 10));
        }
        printf("\n");
    }
    pose_t p = {0};
    pose_now(&p);
    scan_t scan;
    rangefinder_scan(f, p.pitch_rad, &scan);
    printf("What each zone makes of it: #N obstacle N cm above the floor, . free, ? floor not seen, blank unused%s:\n",
           surroundings_mapping() ? "" : " (floor not learned yet)");
    for (int row = 0; row < RANGEFINDER_ROWS; row++) {
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            const ray_t *r = &scan.ray[row * RANGEFINDER_COLS + col];
            if (r->kind == RAY_HIT) printf("  #%-2d", (int)(r->z_m * 100 + 0.5f));
            else if (r->kind == RAY_CLEAR || r->kind == RAY_FLOOR) printf("    .");
            else if (r->kind == RAY_NO_FLOOR) printf("    ?");
            else printf("     ");
        }
        printf("\n");
    }
    printf("Learned floor, cm along each ray / obstacle from cm above it:\n");
    for (int row = rangefinder_first_floor_row(); row < RANGEFINDER_ROWS; row++) {
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            float floor_m, min_m;
            if (rangefinder_zone_floor(row * RANGEFINDER_COLS + col, &floor_m, &min_m))
                printf(" %4.0f/%-2.0f", (double)(floor_m * 100), (double)(min_m * 100));
            else printf("    -   ");
        }
        printf("\n");
    }
}

static void print_map(void) {
    pose_t p;
    if (!pose_now(&p)) { printf("No odometry yet\n"); return; }
    if (body_odom()->motors_on) {
        // Printing takes longer than PicoB waits for drive commands.
        behaviour_stop();
        robot_test_stop();
        body_motors(false);
        printf("Motors off for printing the map (g switches them on)\n");
    }
    map_print(&p);
    printf("Map changes so far: %u%s\n", map_changes(), surroundings_mapping() ? "" : " (not mapping: the floor isn't learned yet, press n)");
}

static void start_test(robot_test_t t) {
    behaviour_stop();
    robot_test_start(t);
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
        next_status = make_timeout_time_us(STATUS_PERIOD_US);
        // The first time a serial monitor opens after power-up, the robot looks around.
        if (!scan_started && !robot_test_running()) {
            scan_started = true;
            behaviour_scan();
        }
    }

    switch (getchar_timeout_us(0)) {
    case 'g': body_motors(true); printf("Motors on\n"); break;
    case 's': behaviour_stop(); robot_test_stop(); body_motors(false); printf("Stopped, motors off\n"); break;
    case 'n': robot_test_stop(); scan_started = true; behaviour_scan(); break;
    case 'm': print_map(); break;
    case 'z': print_frame(); break;
    case 'q': start_test(ROBOT_TEST_SQUARE); break;
    case 'd': start_test(ROBOT_TEST_DRIFT); break;
    case 'r': start_test(ROBOT_TEST_TURNS); break;
    case 'f': start_test(ROBOT_TEST_FORWARD); break;
    case 'b': start_test(ROBOT_TEST_BACK); break;
    case 'p': print_status(); break;
    case 't': print_last_result(); break;
    case 'l': print_link(); break;
    case 'h': case '?': help(); break;
    default: break;
    }
    if (time_reached(next_status)) {
        next_status = make_timeout_time_us(STATUS_PERIOD_US);
        if (!robot_test_running() && !behaviour_busy()) print_status();
    }
}
