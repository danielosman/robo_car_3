#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "link.h"
#include "body.h"

#define DRIVE_PERIOD_US   (LINK_DRIVE_TIMEOUT_US / 5) // a lost DRIVE or two doesn't stop the robot
#define MOTORS_RETRY_US   200000
#define REPORT_TIMEOUT_US (10 * LINK_ODOM_PERIOD_US)

static bool greeted;
static bool motors_wanted;
static uint8_t motors_request; // numbers our MOTORS requests; PicoB echoes the last one it acted on
static drive_msg_t drive;
static odom_report_t odom;
static status_report_t status;
static absolute_time_t last_report, next_drive, next_motors, next_hello;

static void send_hello(bool is_reply) {
    hello_msg_t h = {.version = LINK_PROTOCOL_VERSION, .is_reply = is_reply};
    link_send(MSG_HELLO, &h, sizeof h);
}

static void send_motors(void) {
    motors_msg_t m = {.on = motors_wanted, .request = motors_request};
    link_send(MSG_MOTORS, &m, sizeof m);
    next_motors = make_timeout_time_us(MOTORS_RETRY_US);
}

static void on_hello(const hello_msg_t *h) {
    if (h->version != LINK_PROTOCOL_VERSION) {
        printf("PicoB protocol v%u, mine v%u: reflash both Picos\n", h->version, LINK_PROTOCOL_VERSION);
        greeted = false;
        return;
    }
    if (!h->is_reply && greeted) printf("PicoB restarted\n");
    greeted = true;
    if (!h->is_reply) send_hello(true);
}

static void on_message(const link_msg_t *m) {
    switch (m->type) {
    case MSG_HELLO:
        if (m->len == sizeof(hello_msg_t)) on_hello((const hello_msg_t *)m->body);
        break;
    case MSG_ODOM:
        if (m->len == sizeof odom) {
            memcpy(&odom, m->body, sizeof odom);
            last_report = get_absolute_time();
        }
        break;
    case MSG_STATUS:
        if (m->len == sizeof status) memcpy(&status, m->body, sizeof status);
        break;
    case MSG_LOG:
        printf("B: %.*s\n", m->len, (const char *)m->body);
        break;
    default:
        break;
    }
}

void body_init(void) {
    link_init();
    last_report = nil_time;
    next_drive = next_motors = next_hello = get_absolute_time();
}

void body_update(void) {
    link_msg_t m;
    while (link_receive(&m)) on_message(&m);

    if (!greeted) {
        if (time_reached(next_hello)) {
            next_hello = make_timeout_time_us(LINK_HELLO_PERIOD_US);
            send_hello(false);
        }
        return;
    }
    if (time_reached(next_drive)) {
        next_drive = make_timeout_time_us(DRIVE_PERIOD_US);
        link_send(MSG_DRIVE, &drive, sizeof drive);
    }
    // PicoB acted on our last request, then switched the motors off by itself
    // (its LOG says why): respect that. Switching them on again is a decision for
    // whoever called body_motors().
    bool acted_on = odom.motors_request == motors_request;
    if (motors_wanted && acted_on && !odom.motors_on && odom.stop_reason != STOP_NONE) {
        motors_wanted = false;
        memset(&drive, 0, sizeof drive);
    }
    // Covers a lost MOTORS message and PicoB restarting (it then echoes 0): say
    // it again until PicoB has acted on it. PicoB ignores repeats it has seen.
    if (body_connected() && !acted_on && time_reached(next_motors)) send_motors();
}

bool body_connected(void) {
    return greeted && !is_nil_time(last_report) &&
           absolute_time_diff_us(last_report, get_absolute_time()) < REPORT_TIMEOUT_US;
}

const odom_report_t *body_odom(void) { return &odom; }

const status_report_t *body_status(void) { return &status; }

void body_motors(bool on) {
    motors_wanted = on;
    if (++motors_request == 0) motors_request = 1; // 0 is PicoB's "none yet"
    if (!on) memset(&drive, 0, sizeof drive);
    if (greeted) send_motors();
}

void body_drive(float v_mps, float w_radps) {
    drive.v_mps = v_mps;
    drive.w_radps = w_radps;
}
