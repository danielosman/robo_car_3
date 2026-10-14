#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "units.h"
#include "link.h"
#include "body.h"
#include "behaviour.h"
#include "surroundings.h"
#include "wifi_console.h"
#include "recorder.h"
#include "loop_stats.h"
#include "mode.h"
#include "snapshot.h"
#include "commands.h"

#define GREETING_DELAY_US 1000000  // the Mac drops what arrives the moment the port opens
#define LINE_MAX          48
// printf takes doubles; the (double) casts below are for printing only.

static bool usb_was_connected, wifi_was_connected, greeted;
static absolute_time_t greeting_time;
static uint32_t endings_seen;

static void greet(void) {
    printf("\nPicoA robot firmware. Commands, one per line: drive <cm>, rotate <deg>, goto <x cm> <y cm>,\n"
           "watch <s>, snapshot, clear_map, recording on|off, mode idle|manual, stop, status\n");
}

static void print_link(void) {
    link_stats_t s = link_stats();
    printf("Link: rx %lu bytes, %lu msgs, %lu bad, %lu lost; tx %lu msgs, %lu dropped\n",
           (unsigned long)s.rx_bytes, (unsigned long)s.rx_msgs, (unsigned long)s.rx_bad,
           (unsigned long)s.rx_lost, (unsigned long)s.tx_msgs, (unsigned long)s.tx_dropped);
}

static void print_body(void) {
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

static void print_status(void) {
    print_body();
    print_link();
    wifi_console_print_status();
    loop_stats_print();
    print_ram();
    printf("Mode: %s\n", mode_name(mode_get()));
    snapshot_print_status();
    recorder_print_status();
}

// The done record: how command n ended, where the robot is (x, y in m, yaw in rad),
// and what else the end says (" moved=31", or "").
static void answer_at(uint32_t n, const char *end, float x_m, float y_m, float yaw_rad, const char *more) {
    char text[112];
    snprintf(text, sizeof text, "done n=%lu end=%s x=%.1f y=%.1f yaw=%.1f%s", (unsigned long)n, end,
             (double)(x_m * 100), (double)(y_m * 100), (double)(yaw_rad * DEG_PER_RAD), more);
    wifi_console_record(text);
}

static void answer(uint32_t n, const char *end) {
    const odom_report_t *o = body_odom();
    answer_at(n, end, o->x_m, o->y_m, o->yaw_rad, "");
}

static const char *end_word(const behaviour_end_t *e) {
    switch (e->reason) {
    case REC_END_DONE: return "ok";
    case REC_END_OBSTACLE: return "obstacle";
    case REC_END_STOPPED: return "stopped";
    case REC_END_REPLACED: return "replaced";
    case REC_END_PICOB_LOST: return "picob_lost";
    case REC_END_NO_START: return "no_start";
    case REC_END_SAFETY_STOP:
        switch (e->stop_reason) {
        case STOP_TILT: return "tilt";
        case STOP_LEFT_WHEELS: case STOP_RIGHT_WHEELS: return "wheels";
        case STOP_NO_DRIVE: return "no_drive";
        default: return "safety";
        }
    default: return "ended";
    }
}

// A move in idle: the robot is now steered by hand.
static void manual(void) { if (mode_get() == MODE_IDLE) mode_set(MODE_MANUAL); }

static void stop(void) {
    behaviour_stop();
    body_motors(false);
    printf("Stopped, motors off\n");
    mode_set(MODE_IDLE);
}

static bool within(int v, int lo, int hi) { return v >= lo && v <= hi; }

// One command line, its number already taken off. Moves and snapshots are answered
// when they end (commands_update); the rest at once.
static void run(const char *line, uint32_t n) {
    int a, b, end = -1;
    char word[8];
#define IS(fmt, count, ...) (end = -1, sscanf(line, fmt "%n", __VA_ARGS__, &end) == (count) && end >= 0 && line[end] == 0)
#define IS_WORD(name) (strcmp(line, name) == 0)
    if (IS("drive %d", 1, &a) && within(a, -COMMAND_DRIVE_MAX_CM, COMMAND_DRIVE_MAX_CM)) {
        manual();
        recorder_key('d');
        behaviour_move((float)a / 100, n);
    } else if (IS("rotate %d", 1, &a) && within(a, -COMMAND_ROTATE_MAX_DEG, COMMAND_ROTATE_MAX_DEG)) {
        manual();
        recorder_key('r');
        behaviour_turn((float)a * RAD_PER_DEG, n);
    } else if (IS("goto %d %d", 2, &a, &b) && within(a, -COMMAND_GOTO_MAX_CM, COMMAND_GOTO_MAX_CM) &&
               within(b, -COMMAND_GOTO_MAX_CM, COMMAND_GOTO_MAX_CM)) {
        manual();
        recorder_key('g');
        behaviour_goto((float)a / 100, (float)b / 100, 0, n);
    } else if (IS("watch %d", 1, &a) && within(a, COMMAND_WATCH_MIN_S, COMMAND_WATCH_MAX_S)) {
        manual();
        recorder_key('w');
        behaviour_watch((float)a, n);
    } else if (IS_WORD("snapshot")) {
        snapshot_request(SNAP_COMMAND, n);
    } else if (IS_WORD("clear_map")) {
        surroundings_clear();
        printf("Map cleared\n");
        answer(n, "ok");
    } else if (IS("recording %7s", 1, word) && (!strcmp(word, "on") || !strcmp(word, "off"))) {
        recorder_set(!strcmp(word, "on"));
        if (!recorder_on()) printf("Recording: off\n");
        else if (recorder_ready()) printf("Recording: on; each action is recorded\n");
        else printf("Recording: on, but no recording connection: actions run unrecorded until it is up\n");
        answer(n, "ok");
    } else if (IS("mode %7s", 1, word) && (!strcmp(word, "idle") || !strcmp(word, "manual"))) {
        if (!strcmp(word, "idle")) stop();
        else mode_set(MODE_MANUAL);
        answer(n, "ok");
    } else if (IS_WORD("stop")) {
        stop();
        answer(n, "ok");
    } else if (IS_WORD("status")) {
        print_status();
        answer(n, "ok");
    } else {
        printf("Command refused: \"%s\" (not a command, or a number out of range)\n", line);
        answer(n, "refused");
    }
#undef IS
#undef IS_WORD
}

// A whole line: the command, and its number after '#' if there is one.
static void line_done(char *line) {
    uint32_t n = 0;
    char *hash = strchr(line, '#');
    if (hash) {
        n = (uint32_t)strtoul(hash + 1, NULL, 10);
        *hash = 0;
    }
    size_t len = strlen(line);
    while (len && line[len - 1] == ' ') line[--len] = 0;
    while (*line == ' ') line++;
    if (*line) run(line, n);
}

// Answers the moves and snapshots that ended since the last call.
static void answer_ended(void) {
    behaviour_end_t e;
    uint32_t endings = behaviour_endings(&e);
    if (endings != endings_seen) {
        endings_seen = endings;
        char more[48];
        int len = snprintf(more, sizeof more, " moved=%.0f rotated=%.0f", (double)(e.moved_m * 100),
                           (double)(e.rotated_rad * DEG_PER_RAD));
        if (e.goal_m > 0) snprintf(more + len, sizeof more - (size_t)len, " goal=%.0f", (double)(e.goal_m * 100));
        const odom_report_t *o = body_odom();
        answer_at(e.command, end_word(&e), o->x_m, o->y_m, o->yaw_rad, more);
    }
    snapshot_sent_t s;
    if (snapshot_sent(&s)) {
        char more[24];
        snprintf(more, sizeof more, " snapshot=%lu", (unsigned long)s.seq);
        answer_at(s.command, "ok", s.x_m, s.y_m, s.yaw_rad, more);
    }
}

void commands_update(void) {
    answer_ended();
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
    // Everything that has arrived; a line may come over several loop iterations.
    static char line[LINE_MAX + 1];
    static int len;
    for (int c; (c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT;) {
        if (c == '\n' || c == '\r') {
            line[len] = 0;
            len = 0;
            line_done(line);
        } else if (c >= ' ' && len < LINE_MAX) {
            line[len++] = (char)c; // the server's heartbeat ('\0') and other control characters are skipped
        }
    }
}
