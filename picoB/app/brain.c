#include <stdarg.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "link.h"
#include "link_msgs.h"
#include "odometry.h"
#include "drive.h"
#include "brain.h"

static bool imu_ok;
static bool greeted;          // PicoA's HELLO with our protocol version has arrived
static uint8_t stop_reason;   // STOP_*: why we switched the motors off ourselves
static uint8_t motors_request; // the last MOTORS request acted on; 0 = none yet
static drive_fault_t reported_fault; // so a fault is reported once, not again after MOTORS off
static absolute_time_t drive_deadline, next_odom, next_status, next_hello;

// What PicoA is told when `drive` switches the motors off by itself.
static const uint8_t stop_for_fault[] = {
    [DRIVE_OK] = STOP_NONE,
    [DRIVE_LEFT_NOT_FOLLOWING] = STOP_LEFT_WHEELS,
    [DRIVE_RIGHT_NOT_FOLLOWING] = STOP_RIGHT_WHEELS,
    [DRIVE_TILTED] = STOP_TILT,
};

void brain_log(const char *fmt, ...) {
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
    brain_log("Motors off: %s", stop_reason_text(reason));
}

static void send_hello(bool is_reply) {
    hello_msg_t h = {.version = LINK_PROTOCOL_VERSION, .is_reply = is_reply};
    link_send(MSG_HELLO, &h, sizeof h);
}

static void on_hello(const hello_msg_t *h) {
    bool ok = h->version == LINK_PROTOCOL_VERSION;
    if (ok && !greeted) brain_log("PicoA connected");
    if (!ok) {
        brain_log("PicoA protocol v%u, mine v%u: motors stay off", h->version, LINK_PROTOCOL_VERSION);
        drive_enable(false);
    }
    greeted = ok;
    if (!h->is_reply) {
        motors_request = 0; // PicoA (re)started: its request numbers start again
        send_hello(true);
    }
}

static void on_motors(const motors_msg_t *m) {
    // PicoA repeats a request until our ODOM shows we acted on it; a repeat
    // arriving after a safety stop must not undo the stop.
    if (!greeted || m->request == motors_request) return;
    motors_request = m->request;
    if (m->on && !imu_ok) { brain_log("No IMU: motors stay off"); return; }
    if (m->on == drive_enabled() && stop_reason == STOP_NONE) return;
    drive_enable(m->on);
    if (m->on) reported_fault = DRIVE_OK; // drive_enable(true) cleared the fault
    stop_reason = STOP_NONE;
    drive_deadline = make_timeout_time_us(LINK_DRIVE_TIMEOUT_US);
    brain_log("Motors %s", m->on ? "on" : "off");
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
        .wheel_left_m = o->wheel_left_m, .wheel_right_m = o->wheel_right_m,
        .stationary = o->stationary,
        .motors_on = drive_enabled(),
        .stop_reason = stop_reason,
        .motors_request = motors_request,
        .imu_error = !imu_ok,
    };
    link_send(MSG_ODOM, &r, sizeof r);
}

static void send_status(void) {
    status_report_t r = {.gyro_bias_radps = odom_get()->gyro_bias_radps};
    link_send(MSG_STATUS, &r, sizeof r);
}

void brain_init(bool imu_present) {
    imu_ok = imu_present;
    link_init();
    next_odom = next_status = next_hello = get_absolute_time();
    if (!imu_ok) brain_log("IMU error: WHO_AM_I is not 0x6B; check the ISM330DHCX wiring");
}

void brain_update(void) {
    link_msg_t m;
    while (link_receive(&m)) on_message(&m);

    if (drive_enabled() && time_reached(drive_deadline)) {
        drive_enable(false);
        stop_safely(STOP_NO_DRIVE);
    }
    drive_fault_t fault = drive_fault();
    if (fault != DRIVE_OK && fault != reported_fault) {
        reported_fault = fault;
        stop_safely(stop_for_fault[fault]);
    }
    if (time_reached(next_odom)) {
        next_odom = make_timeout_time_us(LINK_ODOM_PERIOD_US);
        send_odom();
    }
    if (time_reached(next_status)) {
        next_status = make_timeout_time_us(LINK_STATUS_PERIOD_US);
        send_status();
    }
    if (!greeted && time_reached(next_hello)) {
        next_hello = make_timeout_time_us(LINK_HELLO_PERIOD_US);
        send_hello(false);
    }
}
