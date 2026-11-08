// Host test for picoA/app/commands.c over the link to the PC: command messages in
// (each with its number), answer messages out (how it ended, the pose, how far a
// move went, a go to's goal, a snapshot's number), "map cleared" when clear_map
// runs, a command refused, the safety stops named, the commands waiting for the
// greeting after a server connects, a line typed on USB still taken, config pre_ms
// and post_ms (the moves' still times; out of range refused), and the status the PC
// gets (pose, mode, the running move; no peaks since step 4c). The test plays link_pc.h,
// body, behaviour, move, the map and snapshot. Run from the
// repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -Ibuild/host -o build/test_commands picoA/app/test/test_commands.c -lm && build/test_commands
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdio_usb.h"
#include "../tof_format.c"
#include "../wire.c"
#include "../mode.c"
#include "../commands.c"

// --- link_pc.h: the PC's side ---
static bool connected;
static wire_command_t inbox[32];
static int inbox_n, inbox_at;
static uint8_t out[64][WIRE_MSG_MAX]; // the control messages sent
static int out_n;
bool link_pc_connected(void) { return connected; }
void link_pc_print_status(void) {}
bool link_send_control(const uint8_t *msg, uint32_t len) {
    wire_msg_t m;
    assert(wire_take(msg, len, &m) == (int32_t)len && out_n < 64);
    memcpy(out[out_n++], msg, len);
    return true;
}
bool link_received(wire_command_t *c) {
    if (inbox_at == inbox_n) return false;
    *c = inbox[inbox_at++];
    return true;
}
static void pc_sends(uint32_t n, const char *text) {
    inbox[inbox_n] = (wire_command_t){.n = n};
    snprintf(inbox[inbox_n].text, sizeof inbox[inbox_n].text, "%s", text);
    inbox_n++;
}
static wire_msg_t sent(int i) { wire_msg_t m; assert(i < out_n && wire_take(out[i], WIRE_MSG_MAX, &m) > 0); return m; }
static wire_answer_t answer_sent(int i) {
    wire_msg_t m = sent(i);
    wire_answer_t a;
    assert(wire_decode_answer(&m, &a));
    return a;
}

// --- body ---
static odom_report_t odom = {.x_m = 0.43f, .y_m = -0.04f, .yaw_rad = 0.5f};
static status_report_t body_st;
static bool motors;
bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &odom; }
const status_report_t *body_status(void) { return &body_st; }
void body_motors(bool on) { motors = on; }
uint32_t body_imu_received(void) { return 0; }
uint32_t body_imu_lost(void) { return 0; }

// --- behaviour, move ---
static uint32_t started_n, endings;
static float started_value;
static char started[8];
static move_record_t last_end;
static const char *running = "";
static int pre = 200, post = 200, stops;
static void start(const char *what, float v, uint32_t n) { snprintf(started, sizeof started, "%s", what); started_value = v; started_n = n; }
void behaviour_drive(float m, uint32_t n) { start("drive", m, n); }
void behaviour_rotate(float rad, uint32_t n) { start("rotate", rad, n); }
void behaviour_watch(float s, uint32_t n) { start("watch", s, n); }
void behaviour_goto(float x, float y, float stop_m, uint32_t n) { (void)y; (void)stop_m; start("goto", x, n); }
void move_stop(void) { stops++; }
uint32_t move_endings(move_record_t *last) { if (last) *last = last_end; return endings; }
static move_record_t queue[8];
static wire_end_t queue_end[8];
static uint32_t queued_n, taken_n;
static wire_end_t taken_end;
// The test ends a move: it waits to be taken (endings, last_end as move.c keeps them).
static void move_ended(move_record_t r, wire_end_t end) { last_end = r; endings++; queue[queued_n % 8] = r; queue_end[queued_n++ % 8] = end; }
bool move_next_ended(move_record_t *r) {
    if (taken_n == queued_n) return false;
    taken_end = queue_end[taken_n % 8];
    *r = queue[taken_n++ % 8];
    return true;
}
uint8_t move_wire_end(const move_record_t *r) { (void)r; return (uint8_t)taken_end; }
const char *move_running(void) { return running; }
bool move_set_still(int pre_ms, int post_ms) { pre = pre_ms; post = post_ms; return true; }
int move_pre_ms(void) { return pre; }
int move_post_ms(void) { return post; }

// --- the rest ---
char __end__, __HeapLimit; // the linker's, for "RAM free"
static int clears;
static uint32_t snap_asked;
static snapshot_sent_t snap_gone;
static bool snap_has_gone;
void cell_map_clear(void) { clears++; }
int tof_hz(void) { return 10; }
float tof_frames_per_s(void) { return 10; }
void loop_stats_print(void) {}
void snapshot_request(snapshot_why_t why, uint32_t command) { assert(why == SNAP_COMMAND); snap_asked = command; }
bool snapshot_sent(snapshot_sent_t *s) { if (!snap_has_gone) return false; snap_has_gone = false; *s = snap_gone; return true; }
void snapshot_print_status(void) {}

static void loop(int ms) {
    for (int i = 0; i < ms; i++) { fake_now_us += 1000; commands_update(); }
}

int main(void) {
    // A server connects: its commands wait for the greeting (1 s), then run.
    connected = true;
    pc_sends(1, "clear_map");
    loop(500);
    assert(clears == 0 && out_n == 0);
    loop(600);
    assert(clears == 1 && out_n == 2);
    assert(sent(0).type == WIRE_MAP_CLEARED);
    wire_answer_t a = answer_sent(1);
    assert(a.n == 1 && a.end == WIRE_END_OK && a.has == 0 && a.x_m == 0.43f && a.yaw_rad == 0.5f);
    printf("ok: commands wait for the greeting; clear_map: map cleared, then its answer\n");

    // A move: started with its number, answered when it ends, with how far it went.
    out_n = 0;
    pc_sends(3, " drive 50 ");
    loop(1);
    assert(!strcmp(started, "drive") && started_value == 0.5f && started_n == 3 && out_n == 0);
    move_ended((move_record_t){.command = 3, .kind = MOVE_DRIVE, .how = MOVE_END_OBSTACLE, .moved_m = 0.31f, .rotated_rad = 0.01f},
               WIRE_END_OBSTACLE);
    loop(1);
    a = answer_sent(0);
    assert(out_n == 1 && a.n == 3 && a.end == WIRE_END_OBSTACLE && a.has == WIRE_HAS_MOVE && a.moved_m == 0.31f &&
           a.rotated_rad == 0.01f);
    // A go to: its goal too.
    pc_sends(4, "goto 120 -35");
    loop(1);
    assert(!strcmp(started, "goto") && started_value == 1.2f && started_n == 4);
    move_ended((move_record_t){.command = 4, .kind = MOVE_GOTO, .how = MOVE_END_DONE, .moved_m = 1.25f, .goal_m = 4.2f}, WIRE_END_OK);
    loop(1);
    a = answer_sent(1);
    assert(a.n == 4 && a.end == WIRE_END_OK && a.has == (WIRE_HAS_MOVE | WIRE_HAS_GOAL) && a.goal_m == 4.2f);
    // The safety stops, named as move_wire_end() names them.
    const wire_end_t safety[] = {WIRE_END_TILT, WIRE_END_WHEELS, WIRE_END_NO_DRIVE};
    for (int i = 0; i < 3; i++) {
        move_ended((move_record_t){.command = 10, .kind = MOVE_DRIVE, .how = MOVE_END_SAFETY_STOP}, safety[i]);
        loop(1);
        assert(answer_sent(out_n - 1).end == safety[i]);
    }
    // Two moves ending in one loop (a refused drive while a watch ends): both answered, in order.
    int before = out_n;
    move_ended((move_record_t){.command = 20, .kind = MOVE_DRIVE, .how = MOVE_END_OBSTACLE}, WIRE_END_OBSTACLE);
    move_ended((move_record_t){.command = 21, .kind = MOVE_WATCH, .how = MOVE_END_DONE}, WIRE_END_OK);
    loop(1);
    assert(out_n == before + 2 && answer_sent(before).n == 20 && answer_sent(before).end == WIRE_END_OBSTACLE &&
           answer_sent(before + 1).n == 21 && answer_sent(before + 1).end == WIRE_END_OK);
    printf("ok: a move answered when it ends, how far it went; a go to's goal; the safety stops named; two in one loop both\n");

    // A snapshot: answered once it has gone, with its number and its pose.
    out_n = 0;
    pc_sends(5, "snap");
    loop(1);
    assert(snap_asked == 5 && out_n == 0);
    snap_gone = (snapshot_sent_t){.command = 5, .seq = 41, .x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 0.75f};
    snap_has_gone = true;
    loop(1);
    a = answer_sent(0);
    assert(a.n == 5 && a.has == WIRE_HAS_SNAPSHOT && a.snapshot == 41 && a.x_m == 1.25f && a.yaw_rad == 0.75f);
    // Refused: not a command, or out of range.
    pc_sends(6, "fly 3");
    pc_sends(7, "drive 301");
    loop(1);
    assert(answer_sent(1).n == 6 && answer_sent(1).end == WIRE_END_REFUSED);
    assert(answer_sent(2).n == 7 && answer_sent(2).end == WIRE_END_REFUSED);
    // config, mode, stop, status: answered at once. recording is gone (refused).
    pc_sends(8, "config pre_ms 500");
    pc_sends(9, "mode idle");
    pc_sends(10, "status");
    pc_sends(11, "config post_ms 0");
    pc_sends(12, "config post_ms 2001");
    pc_sends(13, "config wait_ms 10");
    pc_sends(14, "recording on");
    loop(1);
    assert(pre == 500 && post == 0 && mode_get() == MODE_IDLE && stops == 1 && out_n == 10);
    assert(answer_sent(3).n == 8 && answer_sent(3).end == WIRE_END_OK && answer_sent(5).n == 10 && answer_sent(5).end == WIRE_END_OK);
    assert(answer_sent(6).n == 11 && answer_sent(6).end == WIRE_END_OK);
    for (int i = 7; i < 10; i++) assert(answer_sent(i).end == WIRE_END_REFUSED);
    printf("ok: a snapshot answered once it has gone; refused; config pre_ms / post_ms; the others at once\n");

    // A line typed on USB still works, its number after '#'.
    out_n = 0;
    fake_usb_connected = true;
    fake_usb_type("rotate 30 #12\n");
    loop(1100); // a monitor connected: after the greeting
    assert(!strcmp(started, "rotate") && fabsf(started_value - 30 * RAD_PER_DEG) < 1e-6f && started_n == 12);
    printf("ok: a line typed on USB is taken too\n");

    // The status the PC gets.
    running = "rotate";
    wire_status_t s;
    memset(&s, 0, sizeof s);
    commands_status(&s);
    assert(s.x_m == 0.43f && s.y_m == -0.04f && s.yaw_rad == 0.5f && !strcmp(s.mode, "manual") && !strcmp(s.action, "rotate"));
    printf("ok: the status: pose, mode, the running move\n");
    printf("commands: all tests passed\n");
    return 0;
}
