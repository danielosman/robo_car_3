#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "units.h"
#include "link.h"
#include "body.h"
#include "behaviour.h"
#include "move.h"
#include "tof.h"
#include "cell_map.h"
#include "link_pc.h"
#include "wire.h"
#include "loop_stats.h"
#include "mode.h"
#include "snapshot.h"
#include "commands.h"

#define GREETING_DELAY_US 1000000  // the Mac drops what arrives the moment the port opens
#define LINE_MAX          48
// printf takes doubles; the (double) casts below are for printing only.

static bool usb_was_connected, wifi_was_connected, greeted;
static absolute_time_t greeting_time;

static void greet(void) {
    printf("\nPicoA robot firmware. Commands, one per line: drive <cm>, rotate <deg>, goto <x cm> <y cm>,\n"
           "watch <s>, snap, clear_map, config pre_ms|post_ms <ms>, mode idle|manual, stop, status\n");
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
    link_pc_print_status();
    loop_stats_print();
    printf("ToF: %d Hz set, %.1f frames a second\n", tof_hz(), (double)tof_frames_per_s());
    print_ram();
    printf("Mode: %s\n", mode_name(mode_get()));
    snapshot_print_status();
    printf("Moves: %lu ended; still %d ms before and %d ms after each (config pre_ms, post_ms)\n",
           (unsigned long)move_endings(NULL), move_pre_ms(), move_post_ms());
    printf("IMU: %lu records from PicoB (104 a second), %lu not taken in time; on PicoB %lu lost (its queue full), %lu dropped (a sample skipped)\n",
           (unsigned long)body_imu_received(), (unsigned long)body_imu_lost(), (unsigned long)body_status()->imu_lost,
           (unsigned long)body_status()->imu_skipped);
}

// The answer: how command n ended, where the robot is (x, y in m, yaw in rad);
// for a move also how far it went, for a snapshot its number (a->has says which).
static void answer_with(wire_answer_t *a) {
    uint8_t msg[WIRE_HEADER + WIRE_ANSWER_SIZE];
    uint32_t n = wire_encode_answer(a, msg);
    link_send_control(msg, n);
}

static void answer(uint32_t n, wire_end_t end) {
    const odom_report_t *o = body_odom();
    wire_answer_t a = {.n = n, .end = (uint8_t)end, .x_m = o->x_m, .y_m = o->y_m, .yaw_rad = o->yaw_rad};
    answer_with(&a);
}

// A move in idle: the robot is now steered by hand.
static void manual(void) { if (mode_get() == MODE_IDLE) mode_set(MODE_MANUAL); }

static void stop(void) {
    move_stop();
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
        behaviour_drive((float)a / 100, n);
    } else if (IS("rotate %d", 1, &a) && within(a, -COMMAND_ROTATE_MAX_DEG, COMMAND_ROTATE_MAX_DEG)) {
        manual();
        behaviour_rotate((float)a * RAD_PER_DEG, n);
    } else if (IS("goto %d %d", 2, &a, &b) && within(a, -COMMAND_GOTO_MAX_CM, COMMAND_GOTO_MAX_CM) &&
               within(b, -COMMAND_GOTO_MAX_CM, COMMAND_GOTO_MAX_CM)) {
        manual();
        behaviour_goto((float)a / 100, (float)b / 100, 0, n);
    } else if (IS("watch %d", 1, &a) && within(a, COMMAND_WATCH_MIN_S, COMMAND_WATCH_MAX_S)) {
        manual();
        behaviour_watch((float)a, n);
    } else if (IS_WORD("snap")) {
        snapshot_request(SNAP_COMMAND, n);
    } else if (IS_WORD("clear_map")) {
        cell_map_clear();
        printf("Map cleared\n");
        uint8_t msg[WIRE_HEADER];
        uint32_t n_msg = wire_encode_map_cleared(msg);
        link_send_control(msg, n_msg);
        answer(n, WIRE_END_OK);
    } else if (IS("config %7s %d", 2, word, &a) && (!strcmp(word, "pre_ms") || !strcmp(word, "post_ms")) &&
               within(a, 0, COMMAND_STILL_MAX_MS)) {
        bool pre = !strcmp(word, "pre_ms");
        move_set_still(pre ? a : move_pre_ms(), pre ? move_post_ms() : a);
        printf("Config: still %d ms before each move, %d ms after (from the next move on)\n", move_pre_ms(), move_post_ms());
        answer(n, WIRE_END_OK);
    } else if (IS("mode %7s", 1, word) && (!strcmp(word, "idle") || !strcmp(word, "manual"))) {
        if (!strcmp(word, "idle")) stop();
        else mode_set(MODE_MANUAL);
        answer(n, WIRE_END_OK);
    } else if (IS_WORD("stop")) {
        stop();
        answer(n, WIRE_END_OK);
    } else if (IS_WORD("status")) {
        print_status();
        answer(n, WIRE_END_OK);
    } else {
        printf("Command refused: \"%s\" (not a command, or a number out of range)\n", line);
        answer(n, WIRE_END_REFUSED);
    }
#undef IS
#undef IS_WORD
}

// A command's text: spaces at either end taken off.
static void run_text(char *text, uint32_t n) {
    size_t len = strlen(text);
    while (len && text[len - 1] == ' ') text[--len] = 0;
    while (*text == ' ') text++;
    if (*text) run(text, n);
}

// A whole line typed on USB: the command, and its number after '#' if there is one.
static void line_done(char *line) {
    uint32_t n = 0;
    char *hash = strchr(line, '#');
    if (hash) {
        n = (uint32_t)strtoul(hash + 1, NULL, 10);
        *hash = 0;
    }
    run_text(line, n);
}

// Answers the moves and snapshots that ended since the last call.
static void answer_ended(void) {
    move_record_t e;
    while (move_next_ended(&e)) { // every one, oldest first: a refused move and the running one can end together
        const odom_report_t *o = body_odom();
        wire_answer_t a = {.n = e.command, .end = move_wire_end(&e), .has = WIRE_HAS_MOVE, .x_m = o->x_m, .y_m = o->y_m,
                           .yaw_rad = o->yaw_rad, .moved_m = e.moved_m, .rotated_rad = e.rotated_rad, .goal_m = e.goal_m};
        if (e.goal_m > 0) a.has |= WIRE_HAS_GOAL;
        answer_with(&a);
    }
    snapshot_sent_t s;
    if (snapshot_sent(&s)) {
        wire_answer_t a = {.n = s.command, .end = WIRE_END_OK, .has = WIRE_HAS_SNAPSHOT, .x_m = s.x_m, .y_m = s.y_m,
                           .yaw_rad = s.yaw_rad, .snapshot = s.seq};
        answer_with(&a);
    }
}

void commands_status(wire_status_t *s) {
    const odom_report_t *o = body_odom();
    s->x_m = o->x_m;
    s->y_m = o->y_m;
    s->yaw_rad = o->yaw_rad;
    snprintf(s->mode, sizeof s->mode, "%s", mode_name(mode_get()));
    snprintf(s->action, sizeof s->action, "%s", move_running());
}

void commands_update(void) {
    answer_ended();
    bool usb = stdio_usb_connected(), wifi = link_pc_connected();
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
    // The PC's commands, each a message.
    wire_command_t c;
    while (link_received(&c)) {
        run_text(c.text, c.n);
        answer_ended(); // each command's move answered before the next can end another
    }
    // Everything typed on USB; a line may come over several loop iterations.
    static char line[LINE_MAX + 1];
    static int len;
    for (int ch; (ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT;) {
        if (ch == '\n' || ch == '\r') {
            line[len] = 0;
            len = 0;
            line_done(line);
            answer_ended();
        } else if (ch >= ' ' && len < LINE_MAX) {
            line[len++] = (char)ch; // control characters are skipped
        }
    }
}
