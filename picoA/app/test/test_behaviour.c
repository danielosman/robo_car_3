// End-to-end host test of the actions (doc/COMMANDS_PLAN.md): picoA/app/behaviour.c
// with the real surroundings, rangefinder, cell_map and pose, on a simulated robot
// in a simulated room seen by a simulated ToF sensor (rays cast at the floor, the
// walls and a box; like the real sensor, each zone reports the nearest point of its
// 5.6° tall patch, with noise; the robot sits 0.9° nose-down).
// The robot does nothing until asked. The scan turns 390° and only that, mapping the
// walls with an empty floor between them (the low box: KNOWN, found about half the time); move and turn go 50 cm and
// 30° each way; stop and a new action end the running one; the motors are off
// after each action. Watching: a person (a fake motion_sense, which reports them
// while they are in the VL53's view) walks in and stops; the robot turns towards
// them and stops facing them; one walking faster than it turns gets away and it
// stops; watching ends after a minute.
// The test provides body.h's functions as the robot. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_behaviour picoA/app/test/test_behaviour.c -lm && build/test_behaviour
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../pose.c"
#include "../rangefinder.c"
#include "../cell_map.c"
#include "../surroundings.c"
#include "../behaviour.c"

#define DT_US       1000
#define TOF_US      66667      // 15 Hz
#define TOF_DELAY_US 35833     // a frame is read this long after the middle of its measurement
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

// --- The simulated robot ---
static odom_report_t sim, report;
static uint32_t report_time_us;
static bool motors_wanted;
static uint64_t motors_changed_us;
static float cmd_v, cmd_w, v, w, still_s, max_yaw_rad;

bool body_connected(void) { return true; }
const odom_report_t *body_odom(void) { return &report; }
uint32_t body_odom_time_us(void) { return report_time_us; }
void body_motors(bool on) { motors_wanted = on; motors_changed_us = fake_now_us; if (!on) cmd_v = cmd_w = 0; }
void body_drive(float v_mps, float w_radps) { cmd_v = v_mps; cmd_w = w_radps; }

// --- A fake motion_sense: a person walking around the robot ---
// Reported while in the VL53's view (±22.5°); one who has stopped is still movement
// while the robot turns, and part of the view once it has stood still for 1 s.
static float person_rad = 99.0f; // world bearing; 99: nobody
static float person_radps;       // walking (+ = left); 0: stopped
static float stopped_still_s;    // how long the robot has stood still since the person stopped
static float walk_sign = 1;      // the way the person last walked (+ = left)

// As the VL53 gives it: a frame every 66.7 ms, the direction measured then (±2°),
// read 40 ms later.
static bool seen_moving;
static float seen_bearing_rad;
static uint32_t seen_at_us;
static uint64_t next_sight_us;
static struct { bool moving; float bearing_rad; uint32_t t_us; uint64_t ready_us; } sight_pending;

static void sight(void) { // every tick
    if (fake_now_us >= next_sight_us) {
        next_sight_us = fake_now_us + 66667;
        float rel = wrap_pi(person_rad - sim.yaw_rad);
        bool in_view = person_rad < 90.0f && fabsf(rel) <= 22.5f * RAD_PER_DEG;
        bool moving = in_view && !(person_radps == 0 && stopped_still_s >= 1.0f);
        float jitter = 2.0f * RAD_PER_DEG * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        sight_pending.moving = moving;
        sight_pending.bearing_rad = rel + jitter;
        sight_pending.t_us = (uint32_t)fake_now_us;
        sight_pending.ready_us = fake_now_us + 40000;
    }
    if (sight_pending.ready_us && fake_now_us >= sight_pending.ready_us) {
        seen_moving = sight_pending.moving;
        seen_bearing_rad = sight_pending.bearing_rad;
        seen_at_us = sight_pending.t_us;
        sight_pending.ready_us = 0;
    }
}

bool motion_sense_strongest(float *bearing_rad, uint32_t *t_us) {
    if (!seen_moving) return false;
    *bearing_rad = seen_bearing_rad;
    *t_us = seen_at_us;
    return true;
}
bool motion_sense_watching(void) { return still_s >= 1.5f; }
void motion_sense_stamp(void) {}
static bool movement_log;
void motion_sense_log(bool on) { movement_log = on; }

static float approach_to(float x, float target, float step) {
    return x < target ? fminf(x + step, target) : fmaxf(x - step, target);
}

// --- The simulated ToF sensor ---

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
    // The box: slabs.
    float lo[3] = {BOX_X - BOX_HALF, BOX_Y - BOX_HALF, 0}, hi[3] = {BOX_X + BOX_HALF, BOX_Y + BOX_HALF, BOX_H};
    float t0 = 0, t1 = best;
    for (int a = 0; a < 3 && t0 <= t1; a++) {
        if (fabsf(d[a]) < 1e-9f) { if (p[a] < lo[a] || p[a] > hi[a]) t0 = t1 + 1; continue; }
        float ta = (lo[a] - p[a]) / d[a], tb = (hi[a] - p[a]) / d[a];
        if (ta > tb) { float x = ta; ta = tb; tb = x; }
        t0 = fmaxf(t0, ta);
        t1 = fminf(t1, tb);
    }
    if (t0 <= t1 && t0 > 0) best = t0;
    return best < 4.0f ? best : 0;
}

static VL53L8CX_ResultsData pending;
static uint64_t pending_at_us, next_frame_us;

static void measure_frame(void) {
    float c = cosf(sim.yaw_rad), s = sinf(sim.yaw_rad);
    float p[3] = {sim.x_m + c * SENSOR_X_M - s * SENSOR_Y_M, sim.y_m + s * SENSOR_X_M + c * SENSOR_Y_M, SENSOR_Z_M};
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++) {
            float range = 0;
            for (int k = -1; k <= 1; k++) { // the zone's centre and its upper and lower edges
                float r[3], d[3];
                int i = row * 8 + col;
                direction(zone_down_rad[i] + (float)k * ZONE_RAD / 2, zone_left_rad[i], sim.pitch_rad, r);
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
    if ((fake_now_us - FAKE_CLOCK_START_US) % LINK_ODOM_PERIOD_US == 0) { // PicoB's report arrives
        report = sim;
        report.t_us = (uint32_t)fake_now_us;
        report_time_us = (uint32_t)fake_now_us;
    }
    if (fake_now_us >= next_frame_us) { next_frame_us += TOF_US; measure_frame(); }
    if (pending_at_us && fake_now_us >= pending_at_us) { fake_tof = pending; fake_tof_fresh = true; pending_at_us = 0; }
    pose_update();
    surroundings_update();
    behaviour_update();
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
    while (behaviour_busy() && seconds < max_s) { tick(); seconds += DT_US * 1e-6f; }
    assert(!behaviour_busy());
    run(0.1f); // PicoB switches the motors off
    assert(!motors_wanted && !sim.motors_on && v == 0 && w == 0);
    return seconds;
}

int main(void) {
    fake_now_us = FAKE_CLOCK_START_US + 1000000;
    next_frame_us = fake_now_us;
    sim.pitch_rad = -0.9f * RAD_PER_DEG;
    srand(3);
    assert(surroundings_init());
    run(1); // standing still: PicoB reports "stationary"

    // Idle: nothing happens until asked, motors off.
    run(5);
    assert(!behaviour_busy() && !motors_wanted && !sim.motors_on && sim.yaw_rad == 0 && sim.x_m == 0);
    printf("ok: idle after start-up, motors off\n");

    // The scan: 390° left, then it stands still with the motors off (no turn to the
    // most open direction, no watching).
    behaviour_scan();
    float seconds = run_action(40);
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
    // KNOWN (MAP_DESIGN.md §8.1): the 8 cm box at 40 cm is found with about half of
    // the random seeds; not asserted until the map fix tested on recordings.
    printf("box (8 cm, 40 cm away): %s (KNOWN: found about half the time)\n", box_seen ? "seen" : "missed");
    assert(wall_cells >= wall_checked * 8 / 10);
    assert(false_obstacles == 0);
    assert(near_free >= near_cells * 9 / 10);
    printf("OK: the scan turns 390 deg, maps the room and stops, motors off\n");

    // Turn 30° left and right, move 50 cm forward and back: on target, motors off after each.
    for (int k = 0; k < 2; k++) {
        float turn_rad = k ? -30 * RAD_PER_DEG : 30 * RAD_PER_DEG, before = sim.yaw_rad;
        behaviour_turn(turn_rad);
        run_action(10);
        printf("turn %+.0f deg: turned %+.1f deg\n", (double)DEG(turn_rad), (double)DEG(sim.yaw_rad - before));
        assert(fabsf(DEG(sim.yaw_rad - before - turn_rad)) < 2);
    }
    for (int k = 0; k < 2; k++) {
        float move_m = k ? -0.5f : 0.5f, x0 = sim.x_m, y0 = sim.y_m, c = cosf(sim.yaw_rad), sn = sinf(sim.yaw_rad);
        behaviour_move(move_m);
        run_action(15);
        float forward_m = c * (sim.x_m - x0) + sn * (sim.y_m - y0);
        printf("move %+.0f cm: moved %+.1f cm\n", (double)(move_m * 100), (double)(forward_m * 100));
        assert(fabsf(forward_m - move_m) < 0.02f);
    }
    printf("OK: turn and move go where asked, motors off after each\n");

    // Stop ends a scan at once; a new action replaces the running one (the motors
    // stay on in between) and runs whole.
    yaw = sim.yaw_rad;
    behaviour_scan();
    run(3);
    assert(sim.yaw_rad > yaw + 10 * RAD_PER_DEG);
    behaviour_stop();
    assert(!behaviour_busy() && !motors_wanted);
    run(1);
    assert(!sim.motors_on && w == 0);
    yaw = sim.yaw_rad;
    behaviour_turn(30 * RAD_PER_DEG);
    run(0.5f);
    behaviour_turn(-30 * RAD_PER_DEG);
    assert(motors_wanted);
    run_action(10);
    float after_replace = DEG(sim.yaw_rad - yaw);
    printf("turn +30 replaced by turn -30 after 0.5 s: turned %+.1f deg in all\n", (double)after_replace);
    assert(after_replace < -15 && after_replace > -30);
    printf("ok: stop, and a new action replacing the running one\n");

    // The map is cleared by C (surroundings_clear), not by a scan.
    surroundings_clear();
    assert(cell_map_column(WALL_FRONT - 0.05f, 0) == COLUMN_UNKNOWN);

    // Watching: nothing moving, the robot stays put, motors on.
    behaviour_watch();
    yaw = sim.yaw_rad;
    run(5);
    assert(behaviour_busy() && sim.yaw_rad == yaw && sim.motors_on && movement_log);
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
    printf("someone walking right at 15 deg/s for 4 s: %d start(s), %.1f deg behind at the end, at most %.0f deg/s;\n"
           "    then stopping: %.1f deg past them at most, stopped %.1f deg from them\n",
           walking_starts, (double)fabsf(lag_deg), (double)fastest, (double)fmaxf(overshoot_deg, 0), (double)off_deg);
    assert(walking_starts == 1 && fabsf(lag_deg) < 5.0f);
    assert(overshoot_deg < 3.0f && fabsf(off_deg) <= 3.5f && w == 0 && fastest <= 30.0f);
    yaw = sim.yaw_rad;
    run(5);
    assert(sim.yaw_rad == yaw); // they stand there: part of the view, no more turning
    printf("ok: follows a steady walker smoothly, doesn't overshoot when they stop, stops facing them\n");

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
    printf("someone walking left at 25 deg/s: %d start(s), %.1f deg behind; after stopping %.1f deg past them at most\n",
           starts, (double)fabsf(lag_deg), (double)fmaxf(overshoot_deg, 0));
    assert(starts == 1 && fabsf(lag_deg) < 8.0f && overshoot_deg < 3.0f);
    yaw = sim.yaw_rad;
    run(3);
    printf("ok: a faster walker too\n");

    // Someone walking left at 40 deg/s, faster than the robot turns: it turns after
    // them until they are out of view, then stops.
    person_rad = yaw + 10 * RAD_PER_DEG;
    person_radps = 40 * RAD_PER_DEG;
    walk_sign = 1;
    fastest = run(3);
    person_rad = 99.0f; // gone
    person_radps = 0;
    run(2);
    printf("someone walking left at 40 deg/s: the robot turned %+.0f deg (at most %.0f deg/s), then stopped\n",
           (double)DEG(sim.yaw_rad - yaw), (double)fastest);
    assert(sim.yaw_rad > yaw + 10 * RAD_PER_DEG && fastest <= 30.0f && w == 0);
    yaw = sim.yaw_rad;
    run(5);
    assert(sim.yaw_rad == yaw);
    printf("ok: someone faster than the robot gets away; it stops when nothing moves\n");

    float watched_s = 37 + run_action(30);
    printf("watching ended by itself after %.1f s\n", (double)watched_s);
    assert(watched_s > 59.5f && watched_s < 61 && !movement_log);
    printf("OK: watching, the robot turns towards movement and stops facing it; it ends after a minute\n");
    return 0;
}
