// PicoB robot firmware, the body: drives the wheels as PicoA commands over the
// link, reports odometry back at 50 Hz, and switches the motors off on its own if
// PicoA's drive commands stop arriving. The motors stay off until PicoA has
// greeted us with the same protocol version and switched them on.
// Keep the robot still for ~1 s after power-up (gyro bias).
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "link.h"
#include "link_msgs.h"
#include "odometry.h"
#include "drive.h"

#define DRIVE_PERIOD_US 10000 // wheel control loop

static bool imu_ok;
static bool greeted;          // PicoA's HELLO with our protocol version has arrived
static uint8_t stop_reason;   // STOP_*: why we switched the motors off ourselves
static absolute_time_t drive_deadline;

// Prints on PicoB's USB serial and in PicoA's debug log.
static void report(const char *fmt, ...) {
    char text[LINK_MAX_BODY + 1];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(text, sizeof text, fmt, args);
    va_end(args);
    if (n < 0) return;
    if (n > LINK_MAX_BODY) n = LINK_MAX_BODY;
    printf("%s\n", text);
    link_send(MSG_LOG, text, (size_t)n);
}

// The motors are already off; record why and tell PicoA.
static void stop_safely(uint8_t reason) {
    stop_reason = reason;
    report("Motors off: %s", stop_reason_text(reason));
}

static void send_hello(bool is_reply) {
    hello_msg_t h = {.version = LINK_PROTOCOL_VERSION, .is_reply = is_reply};
    link_send(MSG_HELLO, &h, sizeof h);
}

static void on_hello(const hello_msg_t *h) {
    bool ok = h->version == LINK_PROTOCOL_VERSION;
    if (ok && !greeted) report("PicoA connected");
    if (!ok) {
        report("PicoA protocol v%u, mine v%u: motors stay off", h->version, LINK_PROTOCOL_VERSION);
        drive_enable(false);
    }
    greeted = ok;
    if (!h->is_reply) send_hello(true);
}

static void on_motors(const motors_msg_t *m) {
    if (!greeted) return;
    if (m->on && !imu_ok) { report("No IMU: motors stay off"); return; }
    if (m->on == drive_enabled() && stop_reason == STOP_NONE) return;
    drive_enable(m->on);
    stop_reason = STOP_NONE;
    drive_deadline = make_timeout_time_us(LINK_DRIVE_TIMEOUT_US);
    report("Motors %s", m->on ? "on" : "off");
}

static void on_drive(const drive_msg_t *d) {
    if (!greeted) return;
    drive_set(d->v_mps, d->w_radps);
    drive_deadline = make_timeout_time_us(LINK_DRIVE_TIMEOUT_US);
}

static void on_message(const link_msg_t *m) {
    switch (m->type) {
    case MSG_HELLO:
        if (m->len == sizeof(hello_msg_t)) on_hello((const hello_msg_t *)m->body);
        break;
    case MSG_MOTORS:
        if (m->len == sizeof(motors_msg_t)) on_motors((const motors_msg_t *)m->body);
        break;
    case MSG_DRIVE:
        if (m->len == sizeof(drive_msg_t)) on_drive((const drive_msg_t *)m->body);
        break;
    default:
        break;
    }
}

static void send_odom(void) {
    const odom_t *o = odom_get();
    odom_report_t r = {
        .t_us = time_us_32(),
        .x_m = o->x_m, .y_m = o->y_m, .yaw_rad = o->yaw_rad,
        .v_mps = o->v_mps, .w_radps = o->w_radps,
        .pitch_rad = o->pitch_rad, .roll_rad = o->roll_rad,
        .stationary = o->stationary,
        .motors_on = drive_enabled(),
        .stop_reason = stop_reason,
        .imu_error = !imu_ok,
    };
    link_send(MSG_ODOM, &r, sizeof r);
}

int main(void) {
    stdio_init_all();
    link_init();
    drive_init();
    imu_ok = odom_init();
    if (!imu_ok) report("IMU error: WHO_AM_I is not 0x6B; check the ISM330DHCX wiring");

    absolute_time_t next_drive = get_absolute_time();
    absolute_time_t next_report = next_drive, next_hello = next_drive;
    for (;;) {
        odom_update();
        link_msg_t m;
        while (link_receive(&m)) on_message(&m);

        if (drive_enabled() && time_reached(drive_deadline)) {
            drive_enable(false);
            stop_safely(STOP_NO_DRIVE);
        }
        if (time_reached(next_drive)) {
            next_drive = make_timeout_time_us(DRIVE_PERIOD_US);
            drive_update(odom_get());
            drive_fault_t fault = drive_fault();
            if (fault != DRIVE_OK && stop_reason == STOP_NONE)
                stop_safely(fault == DRIVE_LEFT_NOT_FOLLOWING ? STOP_LEFT_WHEELS : STOP_RIGHT_WHEELS);
        }
        if (time_reached(next_report)) {
            next_report = make_timeout_time_us(LINK_ODOM_PERIOD_US);
            send_odom();
        }
        if (!greeted && time_reached(next_hello)) {
            next_hello = make_timeout_time_us(LINK_HELLO_PERIOD_US);
            send_hello(false);
        }
    }
}
