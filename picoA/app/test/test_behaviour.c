// End-to-end host test of the actions (doc/ARCHITECTURE.md, actions): picoA/app/behaviour.c
// with the real ToF pipeline (tof, pose's stamp, tof_format, tof_rays, cell_map: as
// main.c runs it) and pose, on a simulated robot
// in a simulated room seen by a simulated ToF sensor (rays cast at the floor, the
// walls and a box; like the real sensor, each zone reports the nearest point of its
// 5.6° tall patch, with noise; the robot sits 0.9° nose-down).
// The robot does nothing until asked. A 390° turn (what the scan was) maps the
// walls with an empty floor between them (the low box: KNOWN, found about half the time); move and turn go 50 cm and
// 30° each way; stop and a new action end the running one; the motors are off
// after each action. Watching: a person (a fake watch, which reports them
// while they are in the VL53's view) walks in and stops; the robot turns towards
// them and stops facing them; one walking faster than it turns gets away and it
// stops; watching ends after the seconds asked. Each action is one move (the real
// move.c: still before and after, its start and end messages, ended with the right
// reason, once, with its command's number, how far it moved; its frames sent only
// while it runs, none dropped), also a move blocked from the start and a goal
// already reached (a start and an end, no frames). Go to a point turns towards it
// and drives there (or stops before an obstacle, or short of it where asked).
// The test provides body.h's and link_pc.h's functions. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_behaviour picoA/app/test/test_behaviour.c -lm && build/test_behaviour
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../pose.c"
#include "../tof.c"
#include "../tof_rays.c"
#include "../tof_format.c"
#include "../cell_map.c"
#include "../house_map.c"
#include "../wire.c"
#include "../move.c"
#include "../behaviour.c"
#include "robot_mounting.h"

#define DT_US       1000
#define SCAN_RAD    (390 * RAD_PER_DEG) // a full turn plus 30° of overlap
#ifndef TOF_HZ
#define TOF_HZ      TOF_DEFAULT_HZ // run_tests.sh also builds it with -DTOF_HZ=15: nothing here counts frames
#endif
#define TOF_US      (1000000 / TOF_HZ)
#define TOF_DELAY_US (TOF_US / 2 + 2500) // a frame is read this long after the middle of its measurement (half a period + half a poll)
#define ACCEL_MPS2  0.4f
#define TURN_ACCEL_RADPS2 3.0f
#define LAG_US      30000
#define DEG(r)      ((r) * DEG_PER_RAD)

// The room: walls 1.6 m ahead, 1.0 m behind, 0.8 m to each side; an 8 cm tall box
// 40 cm ahead-left. (Much farther, a box this low reads like the floor behind it.)
#define WALL_FRONT  1.6f
#define WALL_BACK   (-1.0f)
#define WALL_SIDE   0.8f
#define WALL_H      0.5f
#define BOX_X       0.4f
#define BOX_Y       0.25f
#define BOX_HALF    0.05f
#define BOX_H       0.08f
// A 20 cm box put down after the scan (a chair's base), for moves to stop before.
#define CHAIR_X     0.0f
#define CHAIR_Y     (-0.65f) // its near face mid-cell, 55 cm from the robot's centre
#define CHAIR_HALF  0.1f
#define CHAIR_H     0.2f
static bool chair_there;

// --- The simulated robot ---
static odometry_report_t sim, report;
static uint32_t report_time_us, report_count;
static bool motors_wanted;
static uint64_t motors_changed_us;
static float cmd_v, cmd_w, v, w, still_s, max_yaw_rad;

bool body_connected(void) { return true; }
const odometry_report_t *body_odometry(void) { return &report; }
uint32_t body_odometry_time_us(void) { return report_time_us; }
uint32_t body_odometry_count(void) { return report_count; }
bool body_odometry_get(uint32_t n, odometry_report_t *r, uint32_t *t_us) {
    if (n != report_count) return false; // the fake keeps only the latest
    *r = report;
    *t_us = report_time_us;
    return true;
}
void body_motors(bool on) { motors_wanted = on; motors_changed_us = fake_now_us; if (!on) cmd_v = cmd_w = 0; }
void body_drive(float v_mps, float w_radps) { cmd_v = v_mps; cmd_w = w_radps; }

// --- A fake watch: a person walking around the robot ---
// Reported while in the VL53's view (±22.5°); one who has stopped is still movement
// while the robot turns, and part of the view once it has stood still for 1 s.
static float person_rad = 99.0f; // world bearing; 99: nobody
static float person_radps;       // walking (+ = left); 0: stopped
static float stopped_still_s;    // how long the robot has stood still since the person stopped
static float walk_sign = 1;      // the way the person last walked (+ = left)

// As the VL53 gives it: a frame every TOF_US, the direction measured then (±2°),
// read TOF_DELAY_US later.
static bool seen_moving;
static float seen_bearing_rad;
static uint32_t seen_at_us;
static uint64_t next_sight_us;
static struct { bool moving; float bearing_rad; uint32_t t_us; uint64_t ready_us; } sight_pending;

static void sight(void) { // every tick
    if (fake_now_us >= next_sight_us) {
        next_sight_us = fake_now_us + TOF_US;
        float rel = wrap_pi(person_rad - sim.yaw_rad);
        bool in_view = person_rad < 90.0f && fabsf(rel) <= 22.5f * RAD_PER_DEG;
        bool moving = in_view && !(person_radps == 0 && stopped_still_s >= 1.0f);
        float jitter = 2.0f * RAD_PER_DEG * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        sight_pending.moving = moving;
        sight_pending.bearing_rad = rel + jitter;
        sight_pending.t_us = (uint32_t)fake_now_us;
        sight_pending.ready_us = fake_now_us + TOF_DELAY_US;
    }
    if (sight_pending.ready_us && fake_now_us >= sight_pending.ready_us) {
        seen_moving = sight_pending.moving;
        seen_bearing_rad = sight_pending.bearing_rad;
        seen_at_us = sight_pending.t_us;
        sight_pending.ready_us = 0;
    }
}

bool watch_strongest(float *bearing_rad, uint32_t *t_us) {
    if (!seen_moving) return false;
    *bearing_rad = seen_bearing_rad;
    *t_us = seen_at_us;
    return true;
}
bool watch_watching(void) { return still_s >= 1.5f; }
void watch_stamp(void) {}
static bool movement_log;
void watch_log(bool on) { movement_log = on; }

// --- link_pc.h: the moves as the PC gets them (start, end, frames in between) ---
static bool move_open;            // a move has started and not ended
static int moves_started, frames_in_move, frames_sent;
static wire_move_start_t last_start;
static wire_move_end_t last_end;
bool link_pc_send_control(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len);
    if (wire_decode_move_start(&m, &last_start)) {
        assert(!move_open); // the one before has ended
        move_open = true;
        moves_started++;
        frames_in_move = 0;
    } else if (wire_decode_move_end(&m, &last_end)) {
        assert(move_open && last_end.move == last_start.move && last_end.frames_sent == (uint32_t)frames_in_move);
        assert(last_end.frames_dropped == 0);
        move_open = false;
    }
    return true;
}
bool link_pc_send_bulk(const uint8_t *message, uint32_t len) {
    wire_message_t m;
    assert(wire_take(message, len, &m) == (int32_t)len && move_open); // frames only during a move
    assert(read32(m.payload) == last_start.move);                  // under the running move's number
    frames_in_move++;
    frames_sent++;
    return true;
}

// The ToF pipeline into the map, as main.c runs it (watch is faked above).
static void tof_pipeline(void) {
    static tof_frame_t frame;
    static posed_frame_t posed, used;
    static uint8_t bytes[TOF_FORMAT_MAX];
    if (!tof_poll(&frame)) return;
    pose_stamp(&frame, &posed);
    move_tof(&posed);
    uint32_t n_bytes = tof_format_pack(TOF_USE_MAP, &posed, bytes);
    assert(tof_format_unpack(tof_format_of(TOF_USE_MAP), bytes, n_bytes, &used));
    cell_map_add(&used);
}

static float approach_to(float x, float target, float step) {
    return x < target ? fminf(x + step, target) : fmaxf(x - step, target);
}

// --- The simulated ToF sensor ---

// A box on the floor (slabs): the distance to it if nearer than best.
static float cast_box(const float p[3], const float d[3], float best, float x, float y, float half, float h) {
    float lo[3] = {x - half, y - half, 0}, hi[3] = {x + half, y + half, h};
    float t0 = 0, t1 = best;
    for (int a = 0; a < 3 && t0 <= t1; a++) {
        if (fabsf(d[a]) < 1e-9f) { if (p[a] < lo[a] || p[a] > hi[a]) t0 = t1 + 1; continue; }
        float ta = (lo[a] - p[a]) / d[a], tb = (hi[a] - p[a]) / d[a];
        if (ta > tb) { float x = ta; ta = tb; tb = x; }
        t0 = fmaxf(t0, ta);
        t1 = fminf(t1, tb);
    }
    return t0 <= t1 && t0 > 0 ? t0 : best;
}

// Distance from p along unit d to the room (floor, walls, box), or 0 beyond 4 m.
static float cast(const float p[3], const float d[3]) {
    float best = 4.0f;
    if (d[2] < 0) best = fminf(best, -p[2] / d[2]);
    const float walls[4][2] = {{WALL_FRONT, 0}, {WALL_BACK, 0}, {WALL_SIDE, 1}, {-WALL_SIDE, 1}};
    for (int k = 0; k < 4; k++) {
        int axis = (int)walls[k][1];
        if (d[axis] == 0) continue;
        float t = (walls[k][0] - p[axis]) / d[axis];
        if (t > 0 && p[2] + t * d[2] < WALL_H) best = fminf(best, t);
    }
    best = cast_box(p, d, best, BOX_X, BOX_Y, BOX_HALF, BOX_H);
    if (chair_there) best = cast_box(p, d, best, CHAIR_X, CHAIR_Y, CHAIR_HALF, CHAIR_H);
    return best < 4.0f ? best : 0;
}

static VL53L8CX_ResultsData pending;
static uint64_t pending_at_us, next_frame_us;

static void measure_frame(void) {
    float c = cosf(sim.yaw_rad), s = sinf(sim.yaw_rad);
    float p[3] = {sim.x_m + c * mount.x_m - s * mount.y_m, sim.y_m + s * mount.x_m + c * mount.y_m, mount.z_m};
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++) {
            float range = 0;
            for (int k = -1; k <= 1; k++) { // the zone's centre and its upper and lower edges
                float r[3], d[3];
                int i = row * 8 + col;
                ray_direction(zone_down_rad[i] + (float)k * zone_rad / 2, zone_left_rad[i], sim.pitch_rad, r);
                d[0] = c * r[0] - s * r[1]; d[1] = s * r[0] + c * r[1]; d[2] = r[2];
                float t = cast(p, d);
                if (t > 0 && (range == 0 || t < range)) range = t;
            }
            if (range > 0) range += 0.01f * (2 * (float)rand() / (float)RAND_MAX - 1);
            int zone = (7 - col) * 8 + row, first = zone * (int)VL53L8CX_NB_TARGET_PER_ZONE;
            pending.nb_target_detected[zone] = range > 0;
            pending.distance_mm[first] = (int16_t)(range * 1000 + 0.5f);
            pending.target_status[first] = 5;
        }
    pending_at_us = fake_now_us + TOF_DELAY_US;
}

static void tick(void) {
    float dt = DT_US * 1e-6f;
    fake_now_us += DT_US;
    if (fake_now_us - motors_changed_us >= LAG_US) sim.motors_on = motors_wanted;
    v = sim.motors_on ? approach_to(v, cmd_v, ACCEL_MPS2 * dt) : 0;
    w = sim.motors_on ? approach_to(w, cmd_w, TURN_ACCEL_RADPS2 * dt) : 0;
    sim.x_m += v * cosf(sim.yaw_rad) * dt;
    sim.y_m += v * sinf(sim.yaw_rad) * dt;
    sim.yaw_rad += w * dt;
    max_yaw_rad = fmaxf(max_yaw_rad, sim.yaw_rad);
    sim.v_mps = v;
    sim.w_radps = w;
    still_s = v == 0 && w == 0 ? still_s + dt : 0;
    person_rad += person_radps * dt;
    stopped_still_s = person_radps == 0 && w == 0 ? stopped_still_s + dt : 0;
    sight();
    sim.stationary = still_s >= 0.5f;
    if ((fake_now_us - FAKE_CLOCK_START_US) % LINK_ODOMETRY_PERIOD_US == 0) { // PicoB's report arrives
        report = sim;
        report.t_us = (uint32_t)fake_now_us;
        report_time_us = (uint32_t)fake_now_us;
        report_count++;
    }
    if (fake_now_us >= next_frame_us) { next_frame_us += TOF_US; measure_frame(); }
    if (pending_at_us && fake_now_us >= pending_at_us) { fake_tof = pending; fake_tof_fresh = true; pending_at_us = 0; }
    pose_update();
    tof_pipeline();
    move_update();
}

// A wall at x (axis 0) or y (axis 1): blocked or an overhang, in the column on either side of it.
static bool wall_at(float x, float y, int axis) {
    for (int side = -1; side <= 1; side += 2) {
        float cx = axis == 0 ? x + 0.05f * (float)side : x, cy = axis == 1 ? y + 0.05f * (float)side : y;
        column_t c = cell_map_column(cx, cy);
        if (c == COLUMN_BLOCKED || c == COLUMN_OVERHANG) return true;
    }
    return false;
}

// Runs for `seconds`; returns the fastest turn on the way (deg/s). Counts the
// times the robot starts turning, and the farthest it got past the person in the
// direction they walk (overshoot, deg).
static int starts;
static float overshoot_deg;
static float run(float seconds) {
    float fastest = 0;
    for (float t = 0; t < seconds; t += DT_US * 1e-6f) {
        bool was_turning = cmd_w != 0;
        tick();
        starts += !was_turning && cmd_w != 0;
        fastest = fmaxf(fastest, fabsf(DEG(w)));
        if (person_rad < 90.0f) overshoot_deg = fmaxf(overshoot_deg, DEG(walk_sign * (sim.yaw_rad - person_rad)));
    }
    return fastest;
}

// Runs until the action ends (at most max_s); returns how long it ran (s).
static float run_action(float max_s) {
    float seconds = 0;
    while (*move_running() && seconds < max_s) { tick(); seconds += DT_US * 1e-6f; }
    assert(!*move_running());
    run(0.1f); // PicoB switches the motors off
    assert(!motors_wanted && !sim.motors_on && v == 0 && w == 0);
    return seconds;
}

// One watching trial with the person's sighting jitter of `seed`: a 15 deg/s walker
// who stops, a 25 deg/s one, a 40 deg/s one who gets away. The watch keeps running.
typedef struct { int starts15, starts25; float lag15, over15, off15, lag25, over25; } trial_t;
static void watch_trial(unsigned seed, trial_t *t) {
    // Nothing moving: the robot stays put, motors on.
    srand(seed);
    behaviour_watch(60, 0);
    assert(move_open && last_start.kind == WIRE_MOVE_WATCH && last_start.parameter == 60); // a watch is a move
    float yaw = sim.yaw_rad;
    run(5);
    assert(*move_running() && sim.yaw_rad == yaw && sim.motors_on && movement_log);
    // Someone comes in on the right (-15 deg) walking right at 15 deg/s for 4 s, then
    // stops: the robot turns towards them and follows at their speed (one start, not
    // stop-and-go), lagging a few degrees; when they stop it doesn't run past them,
    // and stops facing them.
    person_rad = yaw - 15 * RAD_PER_DEG;
    person_radps = -15 * RAD_PER_DEG;
    walk_sign = -1;
    starts = 0;
    float fastest = run(4);
    float lag_deg = DEG(wrap_pi(person_rad - sim.yaw_rad));
    int walking_starts = starts;
    overshoot_deg = -99;
    person_radps = 0;
    fastest = fmaxf(fastest, run(3));
    float off_deg = DEG(wrap_pi(person_rad - sim.yaw_rad));
    *t = (trial_t){.starts15 = walking_starts, .lag15 = fabsf(lag_deg), .over15 = fmaxf(overshoot_deg, 0), .off15 = fabsf(off_deg)};
    assert(w == 0 && fastest <= 30.0f);
    yaw = sim.yaw_rad;
    run(5);
    assert(sim.yaw_rad == yaw); // they stand there: part of the view, no more turning

    // Walking the other way at 25 deg/s (close to the robot's 29 deg/s): still one start.
    person_rad = yaw + 10 * RAD_PER_DEG;
    person_radps = 25 * RAD_PER_DEG;
    walk_sign = 1;
    starts = 0;
    run(4);
    lag_deg = DEG(wrap_pi(person_rad - sim.yaw_rad));
    person_radps = 0;
    overshoot_deg = -99;
    run(3);
    t->starts25 = starts;
    t->lag25 = fabsf(lag_deg);
    t->over25 = fmaxf(overshoot_deg, 0);
    yaw = sim.yaw_rad;
    run(3);

    // Someone walking left at 40 deg/s, faster than the robot turns: it turns after
    // them until they are out of view, then stops.
    person_rad = yaw + 10 * RAD_PER_DEG;
    person_radps = 40 * RAD_PER_DEG;
    walk_sign = 1;
    fastest = run(3);
    person_rad = 99.0f; // gone
    person_radps = 0;
    run(2);
    assert(sim.yaw_rad > yaw + 10 * RAD_PER_DEG && fastest <= 30.0f && w == 0);
    yaw = sim.yaw_rad;
    run(5);
    assert(sim.yaw_rad == yaw);
}

static float median_f(float *v, int n) {
    for (int i = 1; i < n; i++)
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { float x = v[j]; v[j] = v[j - 1]; v[j - 1] = x; }
    return (v[(n - 1) / 2] + v[n / 2]) / 2;
}

int main(void) {
    fake_now_us = FAKE_CLOCK_START_US + 1000000;
    next_frame_us = fake_now_us;
    sim.pitch_rad = -0.9f * RAD_PER_DEG;
    srand(3);
    tof_rays_init(&ROBOT_MOUNTING);
    cell_map_clear();
    assert(tof_init(&ROBOT_MOUNTING.tof, TOF_HZ));
    printf("(the ToF at %d Hz)\n", TOF_HZ);
    run(1); // standing still: PicoB reports "stationary"

    // Idle: nothing happens until asked, motors off.
    run(5);
    assert(!*move_running() && !motors_wanted && !sim.motors_on && sim.yaw_rad == 0 && sim.x_m == 0);
    printf("ok: idle after start-up, motors off\n");

    // A scan: 390° left, then it stands still with the motors off (no turn to the
    // most open direction, no watching).
    behaviour_rotate(SCAN_RAD, 7);
    assert(move_open && last_start.kind == WIRE_MOVE_ROTATE && last_start.command == 7 && fabsf(last_start.parameter - SCAN_RAD) < 1e-6f);
    assert(last_start.pre_ms == 200 && last_start.post_ms == 200);
    float seconds = run_action(40);
    assert(!move_open && last_end.end == WIRE_END_OK && moves_started == 1);
    move_record_t ended;
    assert(move_endings(&ended) == 1 && ended.command == 7 && ended.how == MOVE_END_DONE && ended.moved_m < 0.03f);
    // Its frames: every one while it ran (~10 a second), none dropped; none while idle.
    printf("scan: %lu ToF frames sent, none dropped\n", (unsigned long)last_end.frames_sent);
    assert(last_end.frames_sent >= (uint32_t)(seconds * TOF_HZ) - 2 && last_end.frames_sent <= (uint32_t)(seconds * TOF_HZ) + 2);
    int sent_before = frames_sent;
    run(2);
    assert(frames_sent == sent_before);
    assert(fabsf(DEG(ended.rotated_rad) - DEG(sim.yaw_rad)) < 0.5f);
    printf("scan done in %.0f s; turned %.1f deg\n", (double)seconds, (double)DEG(sim.yaw_rad));
    assert(seconds > 14 && seconds < 25);
    assert(fabsf(DEG(sim.yaw_rad) - 390) < 3 && fabsf(DEG(max_yaw_rad) - DEG(sim.yaw_rad)) < 0.5f);
    float yaw = sim.yaw_rad;
    run(3);
    assert(sim.yaw_rad == yaw);

    // The walls are on the map where they are, the open floor between them has no
    // obstacle, the floor around the robot is free.
    int wall_cells = 0, wall_checked = 0;
    for (float y = -0.65f; y <= 0.65f; y += 0.1f) {
        wall_checked++;
        wall_cells += wall_at(WALL_FRONT, y, 0);
    }
    for (float x = -0.85f; x <= 1.45f; x += 0.1f) {
        wall_checked += 2;
        wall_cells += wall_at(x, WALL_SIDE, 1) + wall_at(x, -WALL_SIDE, 1);
    }
    int false_obstacles = 0, near_cells = 0, near_free = 0;
    for (float x = -0.75f; x <= 1.35f; x += 0.1f)
        for (float y = -0.55f; y <= 0.55f; y += 0.1f) {
            if (x > BOX_X - 0.2f && fabsf(y - BOX_Y) < 0.2f) continue; // the box hides what's behind it
            false_obstacles += cell_map_column(x, y) == COLUMN_BLOCKED;
            if (hypotf(x, y) < 0.6f) {
                near_cells++;
                column_t c = cell_map_column(x, y);
                near_free += c == COLUMN_DRIVABLE || c == COLUMN_OPEN;
            }
        }
    bool box_seen = cell_map_column(BOX_X - 0.05f, BOX_Y) == COLUMN_BLOCKED ||
                    cell_map_column(BOX_X - 0.05f, BOX_Y - 0.05f) == COLUMN_BLOCKED ||
                    cell_map_column(BOX_X - 0.05f, BOX_Y + 0.05f) == COLUMN_BLOCKED;
    printf("walls: %d of %d wall columns found; %d false obstacles on the floor; %d of %d columns within 60 cm free\n",
           wall_cells, wall_checked, false_obstacles, near_free, near_cells);
    // KNOWN (doc/archive/MAP_DESIGN.md §8.1): the 8 cm box at 40 cm is found with about half of
    // the random seeds; not asserted until the map fix tested on recordings.
    printf("box (8 cm, 40 cm away): %s (KNOWN: found about half the time)\n", box_seen ? "seen" : "missed");
    assert(wall_cells >= wall_checked * 8 / 10);
    assert(false_obstacles == 0);
    assert(near_free >= near_cells * 9 / 10);
    printf("OK: the scan turns 390 deg, maps the room and stops, motors off\n");

    // Turn 30° left and right, move 50 cm forward and back: on target, motors off after each.
    for (int k = 0; k < 2; k++) {
        float turn_rad = k ? -30 * RAD_PER_DEG : 30 * RAD_PER_DEG, before = sim.yaw_rad;
        behaviour_rotate(turn_rad, 0);
        run_action(10);
        printf("turn %+.0f deg: turned %+.1f deg\n", (double)DEG(turn_rad), (double)DEG(sim.yaw_rad - before));
        assert(fabsf(DEG(sim.yaw_rad - before - turn_rad)) < 2);
    }
    // Face the right-hand wall (-y), away from the low box; 30 cm each way (the wall
    // is 80 cm away, and a move stops 15 cm before the first cell it shows in).
    behaviour_rotate(-wrap_pi(sim.yaw_rad) - 90 * RAD_PER_DEG, 0);
    run_action(15);
    for (int k = 0; k < 2; k++) {
        float move_m = k ? -0.3f : 0.3f, x0 = sim.x_m, y0 = sim.y_m, c = cosf(sim.yaw_rad), sn = sinf(sim.yaw_rad);
        behaviour_drive(move_m, 0);
        run_action(15);
        float forward_m = c * (sim.x_m - x0) + sn * (sim.y_m - y0);
        printf("move %+.0f cm: moved %+.1f cm\n", (double)(move_m * 100), (double)(forward_m * 100));
        assert(fabsf(forward_m - move_m) < 0.02f);
    }
    printf("OK: turn and move go where asked, motors off after each\n");

    // A move stops 15 cm (front edge to it) before an obstacle on the map, up to a
    // cell (10 cm) earlier: the nearest corner of the cell it shows in. A 20 cm box is
    // put down 45 cm ahead after the scan: the map finds it while driving.
    chair_there = true;
    float y0 = sim.y_m;
    behaviour_drive(0.5f, 0);
    run_action(15);
    float gap_cm = (sim.y_m - EDGE_M - (CHAIR_Y + CHAIR_HALF)) * 100; // facing -y
    printf("move +50 cm towards a 20 cm box 45 cm ahead: moved %.1f cm, front edge %.1f cm from the box\n",
           (double)((y0 - sim.y_m) * 100), (double)gap_cm);
    assert(last_end.end == WIRE_END_OBSTACLE && gap_cm > 14 && gap_cm < 26);
    assert(move_endings(&ended) && ended.how == MOVE_END_OBSTACLE && fabsf(ended.moved_m - (y0 - sim.y_m)) < 0.005f);
    // Within 15 cm already: it doesn't move, but it ends (blocked) at once, so its
    // command is answered: a move start and end, no frames.
    int moves_before = moves_started;
    uint32_t endings_before = move_endings(NULL);
    float x0 = sim.x_m;
    y0 = sim.y_m;
    behaviour_drive(0.5f, 12);
    assert(!*move_running() && !move_open && moves_started == moves_before + 1 && last_end.frames_sent == 0);
    assert(move_endings(&ended) == endings_before + 1 && ended.command == 12 && ended.how == MOVE_END_OBSTACLE);
    assert(ended.moved_m == 0 && ended.kind == MOVE_DRIVE && last_end.end == WIRE_END_OBSTACLE);
    run(1);
    assert(sim.x_m == x0 && sim.y_m == y0);
    // Back: only what the map has (the left-hand wall at +0.8 m, from the scan): the
    // first 50 cm is free, the second stops 15 cm (back edge to it) before the wall.
    behaviour_drive(-0.5f, 0);
    run_action(15);
    assert(last_end.end == WIRE_END_OK && fabsf(sim.y_m - y0 - 0.5f) < 0.02f);
    behaviour_drive(-0.5f, 0);
    run_action(15);
    gap_cm = (WALL_SIDE - sim.y_m - EDGE_M) * 100;
    printf("then back 50 cm twice: the second stopped with the back edge %.1f cm from the wall\n", (double)gap_cm);
    assert(last_end.end == WIRE_END_OBSTACLE && gap_cm > 14 && gap_cm < 26);
    chair_there = false;
    printf("OK: a move stops 15 cm before an obstacle on the map, ahead or behind, or doesn't start\n");


    // Stop ends a scan at once; a new action replaces the running one (the motors
    // stay on in between) and runs whole.
    yaw = sim.yaw_rad;
    behaviour_rotate(SCAN_RAD, 0);
    run(3);
    assert(sim.yaw_rad > yaw + 10 * RAD_PER_DEG);
    move_stop();
    assert(!*move_running() && !motors_wanted && !move_open && last_end.end == WIRE_END_STOPPED);
    run(1);
    assert(!sim.motors_on && w == 0);
    yaw = sim.yaw_rad;
    behaviour_rotate(30 * RAD_PER_DEG, 0);
    run(0.5f);
    int before = moves_started;
    behaviour_rotate(-30 * RAD_PER_DEG, 0);
    assert(motors_wanted && last_end.end == WIRE_END_REPLACED && moves_started == before + 1 && move_open);
    assert(last_start.kind == WIRE_MOVE_ROTATE && last_start.parameter < 0);
    run_action(10);
    assert(last_end.end == WIRE_END_OK);
    float after_replace = DEG(sim.yaw_rad - yaw);
    printf("turn +30 replaced by turn -30 after 0.5 s: turned %+.1f deg in all\n", (double)after_replace);
    assert(after_replace < -15 && after_replace > -30);
    printf("ok: stop, and a new action replacing the running one\n");

    // The map is cleared by C (cell_map_clear), not by a scan.
    cell_map_clear();
    assert(cell_map_column(WALL_FRONT - 0.05f, 0) == COLUMN_UNKNOWN);

    // Watching, with the person's sighting jitter of several seeds, each trial from
    // where the one before left the robot (the overshoot hangs on the seed and the
    // start to the 0.1 deg: KNOWN, so the limits are on the median and the worst of 8).
    // The last trial's watch runs on to its end.
    enum { TRIALS = 8 };
    trial_t trials[TRIALS];
    for (int k = 0; k < TRIALS; k++) {
        watch_trial((unsigned)(k + 1), &trials[k]);
        if (k < TRIALS - 1) { move_stop(); run(1); }
    }
    float over15[TRIALS], over25[TRIALS], worst15 = 0, worst25 = 0, worst_lag15 = 0, worst_lag25 = 0, worst_off = 0;
    for (int k = 0; k < TRIALS; k++) {
        const trial_t *t = &trials[k];
        assert(t->starts15 == 1 && t->starts25 == 1); // one start, not stop-and-go
        over15[k] = t->over15;
        over25[k] = t->over25;
        worst15 = fmaxf(worst15, t->over15);
        worst25 = fmaxf(worst25, t->over25);
        worst_lag15 = fmaxf(worst_lag15, t->lag15);
        worst_lag25 = fmaxf(worst_lag25, t->lag25);
        worst_off = fmaxf(worst_off, t->off15);
    }
    float median15 = median_f(over15, TRIALS), median25 = median_f(over25, TRIALS);
    printf("walkers over %d seeds: at 15 deg/s %.1f deg behind at most, %.1f deg past them at most (median %.1f), stopped within"
           " %.1f deg;\n    at 25 deg/s %.1f deg behind at most, %.1f deg past them at most (median %.1f)\n",
           TRIALS, (double)worst_lag15, (double)worst15, (double)median15, (double)worst_off, (double)worst_lag25, (double)worst25,
           (double)median25);
    // The medians keep the old single-seed limits (3 and 4 deg); single seeds go up to
    // ~3.5 and ~5.5 deg (8 Nov, both rates), hence the worst-case limits.
    assert(worst_lag15 < 5.0f && median15 < 3.0f && worst15 < 4.0f && worst_off <= 3.5f);
    assert(worst_lag25 < 8.0f && median25 < 4.0f && worst25 < 6.0f);
    printf("ok: follows a steady walker smoothly, doesn't overshoot when they stop, stops facing them; a faster walker too;\n"
           "    someone faster than the robot gets away; it stops when nothing moves\n");

    float watched_s = 37 + run_action(30);
    printf("watching ended by itself after %.1f s\n", (double)watched_s);
    assert(watched_s > 59.5f && watched_s < 61 && !movement_log);
    printf("OK: watching, the robot turns towards movement and stops facing it; it ends after a minute\n");

    // Go to a point: turn towards it, drive there, motors off; each one a move and an
    // ending. Back to the middle first, then 1 m ahead-right; then towards a point
    // beyond the front wall: stops 15 cm (front edge) before it.
    const float goals[2][2] = {{0, 0}, {1.0f, -0.3f}};
    for (int k = 0; k < 2; k++) {
        endings_before = move_endings(NULL);
        behaviour_goto(goals[k][0], goals[k][1], 0, 20 + (uint32_t)k);
        assert(move_open && last_start.kind == WIRE_MOVE_GOTO); // the turn and the drive: one move
        float took = run_action(30);
        assert(move_endings(&ended) == endings_before + 1 && ended.command == 20 + (uint32_t)k);
        assert(ended.kind == MOVE_GOTO && ended.how == MOVE_END_DONE && last_end.end == WIRE_END_OK);
        float off_cm = hypotf(sim.x_m - goals[k][0], sim.y_m - goals[k][1]) * 100;
        printf("go to (%+.0f, %+.0f) cm: there in %.1f s, %.1f cm off\n", (double)(goals[k][0] * 100),
               (double)(goals[k][1] * 100), (double)took, (double)off_cm);
        assert(off_cm < 3);
    }
    float x0g = sim.x_m, y0g = sim.y_m;
    behaviour_goto(3.0f, -0.3f, 0, 0);
    run_action(30);
    gap_cm = (WALL_FRONT - sim.x_m - EDGE_M) * 100;
    printf("go to a point 1.4 m beyond the front wall: stopped with the front edge %.1f cm from it\n", (double)gap_cm);
    assert(last_end.end == WIRE_END_OBSTACLE && gap_cm > 14 && gap_cm < 26);
    assert(move_endings(&ended) && fabsf(ended.goal_m - hypotf(3.0f - x0g, -0.3f - y0g)) < 0.01f);
    // Already there: nothing starts, and it ends done at once.
    moves_before = moves_started;
    endings_before = move_endings(NULL);
    behaviour_goto(sim.x_m + 0.02f, sim.y_m, 0, 30);
    assert(!*move_running() && !move_open && moves_started == moves_before + 1 && last_end.frames_sent == 0);
    assert(move_endings(&ended) == endings_before + 1 && ended.command == 30 && ended.how == MOVE_END_DONE);
    // Stopping before it: 50 cm short of the middle, measured along the way.
    behaviour_goto(0, -0.3f, 0.5f, 0);
    run_action(30);
    float short_cm = hypotf(sim.x_m, sim.y_m + 0.3f) * 100;
    printf("go to the middle, stopping 50 cm before it: stopped %.1f cm from it\n", (double)short_cm);
    assert(fabsf(short_cm - 50) < 3);
    printf("OK: go to a point turns, drives there, stops before obstacles and where asked\n");
    return 0;
}
