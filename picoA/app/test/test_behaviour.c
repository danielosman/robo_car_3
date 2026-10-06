// End-to-end host test of the start-up scan: picoA/app/behaviour.c with the real
// surroundings, rangefinder, world_map and pose, on a simulated robot in a
// simulated room seen by a simulated ToF sensor (rays cast at the floor, the walls
// and a box; like the real sensor, each zone reports the nearest point of its
// 5.6° tall patch, with noise; the robot sits 0.9° nose-down). The robot must turn 390°, learn the floor, map the walls and the box
// with an empty floor between them, and turn to face the most open direction.
// Then it watches: told that a target is leaving the view (a fake motion_sense), it
// turns to where the target will be when the turn ends, and on after it when the
// target is seen leaving again while the view is learned.
// The test provides body.h's functions as the robot. Run from the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_behaviour picoA/app/test/test_behaviour.c -lm && build/test_behaviour
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../pose.c"
#include "../rangefinder.c"
#include "../world_map.c"
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

// --- The simulated robot (as in test_robot_test.c) ---
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

// --- A fake motion_sense: the leaving reports the test hands out ---
static leaving_t fake_leaving;
static bool fake_have_leaving, leave_while_learning;
static float learning_leave_bearing_deg;

bool motion_sense_leaving(leaving_t *l) {
    if (!fake_have_leaving) return false;
    *l = fake_leaving;
    fake_have_leaving = false;
    return true;
}
bool motion_sense_watching(void) { return still_s >= 1.5f; } // stationary after 0.5 s, then 1 s learning
void motion_sense_stamp(void) {}

static void report_leaving(float bearing_deg, float rate_degps, uint32_t t_us) {
    fake_leaving = (leaving_t){.bearing_rad = bearing_deg * RAD_PER_DEG, .rate_radps = rate_degps * RAD_PER_DEG, .t_us = t_us};
    fake_have_leaving = true;
}

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
    float was_still_s = still_s;
    still_s = v == 0 && w == 0 ? still_s + dt : 0;
    if (leave_while_learning && was_still_s < 1.5f && still_s >= 1.5f) { // the view just learned
        leave_while_learning = false;
        report_leaving(learning_leave_bearing_deg, 0, (uint32_t)fake_now_us - 200000);
    }
    sim.stationary = still_s >= 0.5f;
    if (fake_now_us % LINK_ODOM_PERIOD_US == 0) { // PicoB's report arrives
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

// A wall at x (axis 0) or y (axis 1): occupied in any layer, in the cell on either side of it.
static bool wall_at(float x, float y, int axis) {
    for (int side = -1; side <= 1; side += 2)
        for (int l = 0; l < MAP_LAYERS; l++) {
            float cx = axis == 0 ? x + 0.05f * (float)side : x, cy = axis == 1 ? y + 0.05f * (float)side : y;
            if (map_cell(cx, cy, l) == CELL_OCCUPIED) return true;
        }
    return false;
}

// Runs until the robot has turned and stands still, the view not learned yet; checks the turn.
static void turn_and_learn(float *yaw, float expected_deg, const char *what) {
    for (int i = 0; i < 1000 && state == WATCHING; i++) tick();
    assert(state == LOOKING);
    for (int i = 0; i < 20000 && !(state == WATCHING && still_s >= 1.4f); i++) tick();
    assert(state == WATCHING && still_s >= 1.4f);
    float turned_deg = DEG(sim.yaw_rad - *yaw);
    printf("ok: %s: turned %+.1f deg (expected %+.1f)\n", what, (double)turned_deg, (double)expected_deg);
    assert(fabsf(turned_deg - expected_deg) < 3.0f);
    *yaw = sim.yaw_rad;
}

int main(void) {
    fake_now_us = 1000000;
    next_frame_us = fake_now_us;
    sim.pitch_rad = -0.9f * RAD_PER_DEG;
    srand(3);
    assert(surroundings_init());
    for (int i = 0; i < 1000; i++) tick(); // standing still: PicoB reports "stationary"

    behaviour_scan();
    float seconds = 0;
    while (!behaviour_watching() && seconds < 60) { tick(); seconds += DT_US * 1e-6f; }
    assert(behaviour_watching());
    printf("start-up scan done in %.0f s; yaw %.1f deg\n", (double)seconds, (double)DEG(sim.yaw_rad));
    assert(seconds > 14 && seconds < 25);

    // It turned the full 390° first, then to face the most open direction: ahead
    // (+x, 1.6 m to the wall), a little right of the box.
    assert(max_yaw_rad > 389 * RAD_PER_DEG);
    float heading = wrap_angle(sim.yaw_rad);
    assert(DEG(heading) >= -20.5f && DEG(heading) <= 0.5f);

    // The floor was learned in every zone; the walls are on the map where they are
    // (in some layer: beyond ~1.4 m only their part above 12 cm is seen), and the
    // floor between them is free except at the box.
    assert(surroundings_mapping());
    int wall_cells = 0, wall_checked = 0;
    for (float y = -0.65f; y <= 0.65f; y += 0.1f) {
        wall_checked++;
        wall_cells += wall_at(WALL_FRONT, y, 0);
    }
    for (float x = -0.85f; x <= 1.45f; x += 0.1f) {
        wall_checked += 2;
        wall_cells += wall_at(x, WALL_SIDE, 1) + wall_at(x, -WALL_SIDE, 1);
    }
    int false_obstacles = 0, free_floor = 0, floor_cells = 0, near_cells = 0, near_floor_seen = 0;
    // Layer 0 (2-12 cm) is only seen within ~1 m: farther, the rays above the
    // horizon are higher than 12 cm and the one below has reached the floor. The box
    // hides the floor behind it.
    for (float x = -0.75f; x <= 1.35f; x += 0.1f)
        for (float y = -0.55f; y <= 0.55f; y += 0.1f) {
            if (hypotf(x, y) > 0.95f) continue;
            if (x > BOX_X - 0.2f && fabsf(y - BOX_Y) < 0.2f) continue;
            floor_cells++;
            false_obstacles += map_cell(x, y, 0) == CELL_OCCUPIED;
            free_floor += map_cell(x, y, 0) == CELL_FREE;
            // Within ~50 cm the floor rows see the floor; closer, the robot stands on it.
            if (hypotf(x, y) < 0.45f) near_cells++, near_floor_seen += map_floor_seen(x, y);
        }
    bool box_seen = map_cell(BOX_X - 0.05f, BOX_Y, 0) == CELL_OCCUPIED || map_cell(BOX_X - 0.05f, BOX_Y - 0.05f, 0) == CELL_OCCUPIED ||
                    map_cell(BOX_X - 0.05f, BOX_Y + 0.05f, 0) == CELL_OCCUPIED || map_cell(BOX_X + 0.05f, BOX_Y, 0) == CELL_OCCUPIED;
    printf("walls: %d of %d wall cells occupied; floor: %d of %d cells free, %d false obstacles; box %s\n",
           wall_cells, wall_checked, free_floor, floor_cells, false_obstacles, box_seen ? "seen" : "missed");
    assert(wall_cells >= wall_checked * 9 / 10);
    assert(false_obstacles == 0);
    assert(free_floor >= floor_cells * 9 / 10);
    printf("floor seen in %d of %d cells within 45 cm\n", near_floor_seen, near_cells);
    assert(near_floor_seen >= near_cells * 9 / 10);
    assert(box_seen);

    printf("OK: the start-up scan learns the floor, maps the room and faces the most open direction\n");

    // Watching: nothing leaving, the robot stays put.
    float yaw = sim.yaw_rad;
    for (int i = 0; i < 3000; i++) tick();
    assert(behaviour_watching() && sim.yaw_rad == yaw && sim.motors_on);
    // A target leaving on the right at -19 deg, going right at 30 deg/s: the turn
    // ends where it will be then: a = -19 - 30 (|a| / 57.3 + 0.5) = -71 deg.
    report_leaving(-19, -30, (uint32_t)fake_now_us);
    turn_and_learn(&yaw, -71.4f, "a target leaving on the right at 30 deg/s");
    // It is seen leaving again on the right while the view is learned (0.2 s before
    // the view is learned): on after it at the same speed: a = -19.7 - 30 (0.2 + |a| / 57.3 + 0.5) = -85 deg.
    leave_while_learning = true;
    learning_leave_bearing_deg = -19.7f;
    report_leaving(-19, -30, (uint32_t)fake_now_us);
    turn_and_learn(&yaw, -71.4f, "again");
    turn_and_learn(&yaw, -85.4f, "it left the view again on the right while learning");
    // Seen leaving while learning, but the other way, or not after a turn: nothing to follow.
    leave_while_learning = true;
    learning_leave_bearing_deg = 19.7f;
    report_leaving(-19, -30, (uint32_t)fake_now_us);
    turn_and_learn(&yaw, -71.4f, "a target leaving on the right");
    for (int i = 0; i < 3000; i++) tick();
    assert(state == WATCHING && sim.yaw_rad == yaw);
    report_leaving(19.7f, 0, (uint32_t)fake_now_us);
    for (int i = 0; i < 3000; i++) tick();
    assert(state == WATCHING && sim.yaw_rad == yaw);
    printf("ok: left the view while learning the other way, or not after a turn: no turn\n");
    // Leaving on the left at 20 deg/s: a = 18 + 20 (|a| / 57.3 + 0.5) = 43 deg; fast: at most 90 deg.
    report_leaving(18, 20, (uint32_t)fake_now_us);
    turn_and_learn(&yaw, 43.0f, "a target leaving on the left at 20 deg/s");
    report_leaving(-19, -120, (uint32_t)fake_now_us);
    turn_and_learn(&yaw, -90.0f, "a target leaving on the right at 120 deg/s (at most 90 deg)");

    behaviour_stop();
    assert(!behaviour_busy());
    printf("OK: watching, the robot turns after a target leaving the view, and on when it leaves again while learning\n");
    return 0;
}
