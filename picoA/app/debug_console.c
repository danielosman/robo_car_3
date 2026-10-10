#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "units.h"
#include "link.h"
#include "body.h"
#include "behaviour.h"
#include "surroundings.h"
#include "cell_map.h"
#include "pose.h"
#include "wifi_console.h"
#include "recorder.h"
#include "camera.h"
#include "tof_motion.h"
#include "camera_motion.h"
#include "motion_sense.h"
#include "loop_stats.h"
#include "debug_console.h"

#define GREETING_DELAY_US 1000000  // the Mac drops what arrives the moment the port opens
#define MOVE_M            0.5f
#define TURN_RAD          (30 * RAD_PER_DEG)
// printf takes doubles; the (double) casts below are for printing only.

// The same keys on USB and over WiFi (both are stdio). One key, one operation
// (doc/COMMANDS_PLAN.md); the robot does nothing by itself.
static bool usb_was_connected, wifi_was_connected, greeted;
static absolute_time_t greeting_time;
static bool backwards; // the direction setting: f moves back, t turns right

static const char *direction(void) {
    return backwards ? "back (f moves back, t turns right)" : "forward (f moves forward, t turns left)";
}

static void help(void) {
    printf("Keys: space = stop (motors off), p = status, h = help; the robot does nothing by itself\n"
           "Actions (motors on while one runs): s = scan (390 deg left), f = move %.0f cm, t = turn %.0f deg,\n"
           "       a = watch movement for 1 min, 5 = record 5 s standing still (motors off)\n"
           "Settings: r = direction forward / back (now %s), R = recording on / off (now %s)\n"
           "Map: C = clear the map, m = print the map, z = one ToF frame\n"
           "Camera: c = one frame as 20 x 15 blocks, with its exposure\n"
           "Movement backgrounds now: o = each ToF zone, k = each camera block\n"
           "WiFi: W = connect (or show how it is connected)\n",
           (double)(MOVE_M * 100), (double)(TURN_RAD * DEG_PER_RAD), backwards ? "back" : "forward",
           recorder_on() ? "on" : "off");
}

static void greet(void) {
    printf("\nPicoA robot firmware (M3)\n");
    help();
}

static void print_link(void) {
    link_stats_t s = link_stats();
    printf("Link: rx %lu bytes, %lu msgs, %lu bad, %lu lost; tx %lu msgs, %lu dropped\n",
           (unsigned long)s.rx_bytes, (unsigned long)s.rx_msgs, (unsigned long)s.rx_bad,
           (unsigned long)s.rx_lost, (unsigned long)s.tx_msgs, (unsigned long)s.tx_dropped);
}

// One ToF frame as the robot sees it (top row up, left column left): the distances.
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
}

#define BLOCK 8 // pixels per block side: 160 x 120 -> 20 x 15

// One camera frame as block brightness 0-99, as the robot sees it, so the numbers
// can be compared between presses (a hand in front of it, the light changing).
static void print_camera(void) {
    static uint32_t last_number;
    camera_frame_t f;
    absolute_time_t deadline = make_timeout_time_ms(500);
    while (!camera_frame(&f)) {
        if (time_reached(deadline)) {
            printf("No camera frame (%lu captured so far)\n", (unsigned long)camera_frames_captured());
            return;
        }
    }
    camera_exposure_t e;
    camera_exposure(&e);
    printf("Camera: %.1f frames/s, line %.1f us; exposure %.1f ms, gain x%.2f, %s%s\n",
           e.frame_us > 0.0f ? (double)(1e6f / e.frame_us) : 0.0, (double)e.line_us,
           (double)(e.exposure_us / 1000.0f), (double)e.gain, e.held ? "held" : "free",
           e.settling ? " (changing now)" : e.wants_change ? (e.held ? " (wants a change)" : " (adjusting)") : "");
    printf("Frame %lu (%lu since the last c), its middle row taken %.0f ms ago; block brightness 0-99:\n",
           (unsigned long)f.number, (unsigned long)(f.number - last_number),
           (double)((time_us_32() - camera_row_time(&f, CAMERA_HEIGHT / 2)) / 1000.0f));
    last_number = f.number;
    uint32_t sum = 0;
    for (int by = 0; by < CAMERA_HEIGHT / BLOCK; by++) {
        for (int bx = 0; bx < CAMERA_WIDTH / BLOCK; bx++) {
            uint32_t block = 0;
            for (int y = 0; y < BLOCK; y++)
                for (int x = 0; x < BLOCK; x++) block += f.pixels[(by * BLOCK + y) * CAMERA_WIDTH + bx * BLOCK + x];
            sum += block;
            printf(" %2lu", (unsigned long)(block * 100 / (BLOCK * BLOCK * 256)));
        }
        printf("\n");
    }
    printf("Mean brightness %.1f of 255\n", (double)((float)sum / (float)(CAMERA_WIDTH * CAMERA_HEIGHT)));
}

// What the VL53's movement detection makes of each zone now: its background
// (cm; -- nothing), * = moved (reads clearly closer).
static void print_tof_motion(void) {
    printf("ToF movement, %s: each zone's background, cm (-- nothing), * = moved:\n",
           tof_motion_ready() ? (motion_sense_still() ? "watching" : "watching while turning") : "filling the background");
    for (int row = 0; row < RANGEFINDER_ROWS; row++) {
        for (int col = 0; col < RANGEFINDER_COLS; col++) {
            int i = row * RANGEFINDER_COLS + col;
            uint16_t mm = tof_motion_background_mm(i);
            if (mm) printf(" %c%4u", tof_motion_moved(i) ? '*' : ' ', (unsigned)((mm + 5) / 10));
            else printf(" %c  --", tof_motion_moved(i) ? '*' : ' ');
        }
        printf("\n");
    }
}

// What the camera's movement detection makes of each block now (as the robot sees
// it): its background brightness 0-99, + = differs from it now, * = moved; then
// each block's noise (brightness 0-255, tenths).
static void print_camera_motion(void) {
    printf("Camera movement, %s; exposure adjusted %lu times while still so far.\n",
           !motion_sense_still() ? "robot moving" : camera_motion_ready() ? "watching" : "adjusting the exposure or learning the view",
           (unsigned long)camera_motion_exposure_adjustments());
    if (!camera_motion_ready()) return;
    printf("Each block's background, brightness 0-99 (+ = differs now, * = moved):\n");
    for (int row = 0; row < CAMERA_MOTION_ROWS; row++) {
        for (int col = 0; col < CAMERA_MOTION_COLS; col++) {
            int i = row * CAMERA_MOTION_COLS + col;
            char mark = camera_motion_moved(i) ? '*' : camera_motion_differs(i) ? '+' : ' ';
            printf(" %c%2d", mark, (int)(camera_motion_background(i) * 100.0f / 256.0f));
        }
        printf("\n");
    }
    printf("Each block's noise, brightness 0-255 x 10 (differs beyond 4 x this):\n");
    for (int row = 0; row < CAMERA_MOTION_ROWS; row++) {
        for (int col = 0; col < CAMERA_MOTION_COLS; col++)
            printf(" %3d", (int)(camera_motion_noise(row * CAMERA_MOTION_COLS + col) * 10.0f + 0.5f));
        printf("\n");
    }
}

static void print_map(void) {
    pose_t p;
    if (!pose_now(&p)) { printf("No odometry yet\n"); return; }
    if (behaviour_busy() || body_odom()->motors_on) {
        // Printing takes longer than PicoB waits for drive commands.
        behaviour_stop();
        body_motors(false);
        printf("Stopped for printing the map, motors off\n");
    }
    cell_map_print(&p, 0);
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

// The RAM no static variable uses (the linker's heap: nothing calls malloc but lwIP's
// own pools, which are static too). Fixed at build time.
static void print_ram(void) {
    extern char __end__, __HeapLimit;
    printf("RAM free %lu KB\n", (unsigned long)((uintptr_t)&__HeapLimit - (uintptr_t)&__end__) / 1024);
}

void debug_console_update(void) {
    bool usb = stdio_usb_connected(), wifi = wifi_console_connected();
    if ((usb && !usb_was_connected) || (wifi && !wifi_was_connected)) {
        greeted = false;
        greeting_time = make_timeout_time_us(GREETING_DELAY_US);
    }
    // Losing the monitor doesn't stop the robot: a test can run with the cable
    // pulled. PicoB's safety stops (wheels, tilt) still apply.
    usb_was_connected = usb;
    wifi_was_connected = wifi;
    if (!usb && !wifi) return;
    if (!greeted) {
        if (!time_reached(greeting_time)) return;
        greeted = true;
        greet();
    }

    int key = getchar_timeout_us(0);
    if (key == PICO_ERROR_TIMEOUT) return;
    recorder_key((char)key);
    switch (key) {
    case ' ': behaviour_stop(); body_motors(false); printf("Stopped, motors off\n"); break;
    case 's': behaviour_scan(); break;
    case 'f': behaviour_move(backwards ? -MOVE_M : MOVE_M); break;
    case 't': behaviour_turn(backwards ? -TURN_RAD : TURN_RAD); break;
    case 'a': behaviour_watch(); break;
    case '5': behaviour_record(); break;
    case 'R':
        recorder_set(!recorder_on());
        if (!recorder_on()) printf("Recording: off\n");
        else if (recorder_ready()) printf("Recording: on; each action is recorded\n");
        else printf("Recording: on, but no recording connection: actions run unrecorded until it is up\n");
        break;
    case 'r': backwards = !backwards; printf("Direction: %s\n", direction()); break;
    case 'C': surroundings_clear(); printf("Map cleared\n"); break;
    case 'm': print_map(); break;
    case 'z': print_frame(); break;
    case 'c': print_camera(); break;
    case 'o': print_tof_motion(); break;
    case 'k': print_camera_motion(); break;
    case 'p':
        print_status();
        print_link();
        wifi_console_print_status();
        loop_stats_print();
        print_ram();
        recorder_print_status();
        printf("Direction: %s\n", direction());
        break;
    case 'W': wifi_console_start(); break;
    case 'h': case '?': help(); break;
    default: break;
    }
}
