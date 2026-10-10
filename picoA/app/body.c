#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "link.h"
#include "body.h"

#define DRIVE_PERIOD_US   (LINK_DRIVE_TIMEOUT_US / 5) // a lost DRIVE or two doesn't stop the robot
#define MOTORS_RETRY_US   200000
#define REPORT_TIMEOUT_US (10 * LINK_ODOM_PERIOD_US)
// Clock offset (PicoA - PicoB): the smallest arrival delay over the last few
// seconds is the one with no waiting in buffers or loops. Blocks of 1 s, so the
// estimate follows the clocks drifting apart.
#define OFFSET_BLOCK_US   1000000
#define OFFSET_BLOCKS     4
#define OFFSET_JUMP_US    100000  // a bigger change means PicoB restarted its clock: start over
// The shortest possible delay: an ODOM frame on the wire (COBS, type, sequence,
// CRC and the delimiter add 6 bytes).
#define ODOM_WIRE_US      ((uint32_t)((sizeof(odom_report_t) + 6) * 10 * 1000000ull / LINK_BAUD))

static bool greeted;
static bool motors_wanted;
static uint8_t motors_request; // numbers our MOTORS requests; PicoB echoes the last one it acted on
static drive_msg_t drive, drive_sent;
static uint32_t drives_sent, drive_sent_us;
static odom_report_t odom;
static struct { odom_report_t report; uint32_t t_us; } kept[BODY_ODOM_KEPT]; // report n at n % BODY_ODOM_KEPT
static uint32_t odom_count;
static status_report_t status;
static absolute_time_t last_report, next_drive, next_motors, next_hello;
static uint32_t block_min[OFFSET_BLOCKS], block_start_us, offset_us;
static int blocks_full;    // completed blocks in block_min, up to OFFSET_BLOCKS
static bool clock_known;

static void restart_clock(void) { clock_known = false; blocks_full = 0; }

// d = arrival on PicoA's clock - PicoB's clock at sending. The two clocks wrap at
// the same time, so differences of uint32 values stay right across wraps.
static void track_clock(uint32_t d, uint32_t now_us) {
    if (clock_known && (uint32_t)abs((int32_t)(d - offset_us)) > OFFSET_JUMP_US) restart_clock();
    if (!clock_known) {
        clock_known = true;
        block_start_us = now_us;
        block_min[0] = offset_us = d;
        return;
    }
    uint32_t *current = &block_min[blocks_full % OFFSET_BLOCKS];
    if (now_us - block_start_us >= OFFSET_BLOCK_US) {
        block_start_us = now_us;
        blocks_full++;
        current = &block_min[blocks_full % OFFSET_BLOCKS];
        *current = d;
    } else if ((int32_t)(d - *current) < 0) {
        *current = d;
    }
    int n = blocks_full < OFFSET_BLOCKS ? blocks_full + 1 : OFFSET_BLOCKS;
    offset_us = *current;
    for (int i = 0; i < n; i++)
        if ((int32_t)(block_min[i] - offset_us) < 0) offset_us = block_min[i];
}

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
    if (!h->is_reply) restart_clock();
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
            track_clock(time_us_32() - odom.t_us, time_us_32());
            odom_count++;
            kept[odom_count % BODY_ODOM_KEPT].report = odom;
            kept[odom_count % BODY_ODOM_KEPT].t_us = body_odom_time_us();
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
        drive_sent = drive;
        drive_sent_us = time_us_32();
        drives_sent++;
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

uint32_t body_odom_time_us(void) { return odom.t_us + offset_us - ODOM_WIRE_US; }

uint32_t body_odom_count(void) { return odom_count; }

bool body_odom_get(uint32_t n, odom_report_t *report, uint32_t *t_us) {
    if (n == 0 || n > odom_count || odom_count - n >= BODY_ODOM_KEPT) return false;
    *report = kept[n % BODY_ODOM_KEPT].report;
    *t_us = kept[n % BODY_ODOM_KEPT].t_us;
    return true;
}

const status_report_t *body_status(void) { return &status; }

uint32_t body_drive_sent(float *v_mps, float *w_radps, uint32_t *t_us) {
    *v_mps = drive_sent.v_mps;
    *w_radps = drive_sent.w_radps;
    *t_us = drive_sent_us;
    return drives_sent;
}

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
