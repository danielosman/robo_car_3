// Host test for picoA/app/commands.c over the link to the PC: command messages in
// (each with its number), answer messages out (how it ended, the pose, how far a
// move went, a go to's goal, a snapshot's number), "map cleared" when clear_map
// runs, a command refused with its reason (not a command, out of range, busy), the safety stops named, the commands waiting for the
// greeting after a server connects, a line typed on USB still taken, config pre_ms
// and post_ms (the moves' still times; out of range refused), and the status the PC
// gets (pose, mode, the running move; no peaks since step 4c); correct: the pose
// corrected, "corrected" sent before the answer, refused during a move or out of range;
// a go to's range counts from where the robot is, not from where it was switched on;
// "corrected" again on a new connection. The test plays link_pc.h,
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
static uint8_t out[64][WIRE_MESSAGE_MAX]; // the control messages sent
static int out_n;
static uint8_t bulk[16][WIRE_MESSAGE_MAX];
static int bulk_n, bulk_room = 100;
bool link_pc_connected(void) { return connected; }
static uint32_t bulk_waiting;
uint32_t link_pc_bulk_waiting(void) { return bulk_waiting; }
void link_pc_print_status(void) {}
bool link_pc_send_control(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len && out_n < 64);
    memcpy(out[out_n++], message, len);
    return true;
}
bool link_pc_send_bulk(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len && bulk_n < 16);
    if (!bulk_room) return false;
    bulk_room--;
    memcpy(bulk[bulk_n++], message, len);
    return true;
}
bool link_pc_received(wire_command_t *c) {
    if (inbox_at == inbox_n) return false;
    *c = inbox[inbox_at++];
    return true;
}
static void pc_sends(uint32_t n, const char *text) {
    inbox[inbox_n] = (wire_command_t){.n = n};
    snprintf(inbox[inbox_n].text, sizeof inbox[inbox_n].text, "%s", text);
    inbox_n++;
}
static wire_message_t sent(int i) { wire_message_t m; assert(i < out_n && wire_take(out[i], WIRE_MESSAGE_MAX, &m) > 0); return m; }
static wire_answer_t answer_sent(int i) {
    wire_message_t m = sent(i);
    wire_answer_t a;
    assert(wire_decode_answer(&m, &a));
    return a;
}

// --- body ---
static odometry_report_t odometry = {.x_m = 0.43f, .y_m = -0.04f, .yaw_rad = 0.5f};
static status_report_t body_st;
static bool motors;
bool body_connected(void) { return true; }
const odometry_report_t *body_odometry(void) { return &odometry; }
const status_report_t *body_status(void) { return &body_st; }
void body_motors(bool on) { motors = on; }
uint32_t body_imu_received(void) { return 0; }
static float corrected_to[3];
static int corrections;
void body_correction(float *x, float *y, float *yaw) { *x = corrected_to[0]; *y = corrected_to[1]; *yaw = corrected_to[2]; }
void pose_correct(float x, float y, float yaw) { corrected_to[0] = x; corrected_to[1] = y; corrected_to[2] = yaw; corrections++; }
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
static float clear_x, clear_y;
static uint8_t fake_window[CELL_LAYERS * CELL_MAP_GRID * CELL_MAP_GRID];
static uint32_t snapshot_asked;
static snapshot_sent_t snapshot_gone;
static bool snapshot_has_gone;
void cell_map_clear(float x_m, float y_m) { clears++; clear_x = x_m; clear_y = y_m; }
const uint8_t *cell_map_window(int *ox, int *oy) { *ox = -10; *oy = 20; return fake_window; }
int tof_hz(void) { return 10; }
float tof_frames_per_s(void) { return 10; }
void loop_statistics_print(void) {}
void snapshot_request(snapshot_why_t why, uint32_t command) { assert(why == SNAPSHOT_COMMAND); snapshot_asked = command; }
bool snapshot_sent(snapshot_sent_t *s) { if (!snapshot_has_gone) return false; snapshot_has_gone = false; *s = snapshot_gone; return true; }
void snapshot_print_status(void) {}
static bool snapshot_is_busy;
bool snapshot_busy(void) { return snapshot_is_busy; }

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
    assert(clears == 1 && clear_x == 0.43f && clear_y == -0.04f && out_n == 2);
    assert(sent(0).type == WIRE_MAP_CLEARED);
    wire_answer_t a = answer_sent(1);
    assert(a.n == 1 && a.end == WIRE_END_OK && a.has == 0 && a.x_m == 0.43f && a.yaw_rad == 0.5f);
    // "map": sends the 9 parts of the window on the bulk lane, then answers ok (or busy_bulk if the lane is full).
    out_n = 0;
    fake_window[0] = 0x55;
    fake_window[sizeof fake_window - 1] = 0xaa;
    pc_sends(2, "map");
    loop(1);
    assert(bulk_n == WIRE_MAP_PARTS && out_n == 1 && answer_sent(0).n == 2 && answer_sent(0).end == WIRE_END_OK);
    wire_message_t bm;
    wire_map_part_t mp;
    assert(wire_take(bulk[0], WIRE_MESSAGE_MAX, &bm) > 0 && wire_decode_map_part(&bm, &mp) && mp.part == 0 &&
           mp.origin_cx == -10 && mp.origin_cy == 20 && mp.cells[0] == 0x55);
    assert(wire_take(bulk[8], WIRE_MESSAGE_MAX, &bm) > 0 && wire_decode_map_part(&bm, &mp) && mp.part == 8 &&
           mp.cells[WIRE_MAP_PART_CELLS - 1] == 0xaa);
    bulk_n = 0;
    bulk_room = 3;
    out_n = 0;
    pc_sends(29, "map");
    loop(1);
    assert(out_n == 1 && answer_sent(0).n == 29 && answer_sent(0).end == WIRE_END_BUSY_BULK);
    printf("ok: commands wait for the greeting; clear_map: map cleared with pose; map: 9 parts on bulk\n");

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
    // A go to's range: each of x and y at most 300 cm from where the robot is (43, -4 cm), not from 0, 0.
    started_n = 0;
    pc_sends(30, "goto 340 290");  // 297 and 294 cm from the robot: taken, though past 300 from the start
    loop(1);
    assert(!strcmp(started, "goto") && started_n == 30);
    move_ended((move_record_t){.command = 30, .kind = MOVE_GOTO, .how = MOVE_END_DONE}, WIRE_END_OK);
    loop(1);
    int at = out_n;
    pc_sends(31, "goto -260 0");   // 303 cm from the robot in x: refused, out of range
    pc_sends(32, "goto 0 297");    // 301 cm in y
    loop(1);
    assert(started_n == 30 && out_n == at + 2 && answer_sent(at).n == 31 && answer_sent(at).end == WIRE_END_OUT_OF_RANGE &&
           answer_sent(at + 1).end == WIRE_END_OUT_OF_RANGE);
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
    assert(snapshot_asked == 5 && out_n == 0);
    snapshot_gone = (snapshot_sent_t){.command = 5, .sequence = 41, .x_m = 1.25f, .y_m = -0.5f, .yaw_rad = 0.75f};
    snapshot_has_gone = true;
    loop(1);
    a = answer_sent(0);
    assert(a.n == 5 && a.has == WIRE_HAS_SNAPSHOT && a.snapshot == 41 && a.x_m == 1.25f && a.yaw_rad == 0.75f);
    // Refused: not a command, or out of range (each its own end, so the PC can say which).
    pc_sends(6, "fly 3");
    pc_sends(7, "drive 301");
    loop(1);
    assert(answer_sent(1).n == 6 && answer_sent(1).end == WIRE_END_REFUSED);
    assert(answer_sent(2).n == 7 && answer_sent(2).end == WIRE_END_OUT_OF_RANGE);
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
    assert(answer_sent(7).end == WIRE_END_OUT_OF_RANGE && answer_sent(8).end == WIRE_END_REFUSED &&
           answer_sent(9).end == WIRE_END_REFUSED);
    printf("ok: a snapshot answered once it has gone; refused; config pre_ms / post_ms; the others at once\n");

    // correct: mm and 0.01°; "corrected" first, then the answer; refused out of range or during a move.
    out_n = 0;
    pc_sends(20, "correct 120 -45 -1050");
    loop(1);
    assert(corrections == 1 && corrected_to[0] == 0.12f && corrected_to[1] == -0.045f &&
           fabsf(corrected_to[2] + 10.5f * RAD_PER_DEG) < 1e-6f && out_n == 2);
    wire_message_t cm = sent(0);
    wire_corrected_t k;
    assert(wire_decode_corrected(&cm, &k) && k.x_m == 0.12f && k.y_m == -0.045f && k.yaw_rad == corrected_to[2]);
    assert(answer_sent(1).n == 20 && answer_sent(1).end == WIRE_END_OK);
    pc_sends(21, "correct 20001 0 0");
    pc_sends(22, "correct 0 0 18001");
    pc_sends(23, "correct 1 2");
    running = "drive";
    pc_sends(24, "correct 0 0 0");
    loop(1);
    running = "";
    bulk_waiting = 289;      // a frame still waiting in the bulk lane
    pc_sends(25, "correct 0 0 0");
    loop(1);
    bulk_waiting = 0;
    snapshot_is_busy = true; // a snapshot asked for or going out
    pc_sends(26, "correct 0 0 0");
    loop(1);
    snapshot_is_busy = false;
    assert(corrections == 1 && out_n == 8);
    assert(answer_sent(2).end == WIRE_END_OUT_OF_RANGE && answer_sent(3).end == WIRE_END_OUT_OF_RANGE &&
           answer_sent(4).end == WIRE_END_REFUSED && answer_sent(5).end == WIRE_END_BUSY_MOVE &&
           answer_sent(6).end == WIRE_END_BUSY_BULK && answer_sent(7).end == WIRE_END_BUSY_SNAPSHOT);
    printf("ok: correct: the pose corrected, \"corrected\" before the answer; refused out of range, during a move, "
           "while bulk waits or a snapshot is on its way\n");
    // A new connection: "corrected" again, the correction the robot has.
    out_n = 0;
    connected = false;
    loop(1);
    connected = true;
    loop(1);
    assert(out_n == 1);
    wire_message_t again = sent(0);
    assert(wire_decode_corrected(&again, &k) && k.x_m == 0.12f && k.y_m == -0.045f);
    out_n = 0;
    printf("ok: \"corrected\" sent again on a new connection\n");

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
