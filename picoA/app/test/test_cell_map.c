// Host test for picoA/app/cell_map.c (doc/MAP_DESIGN.md), both variants (VOTES; with
// -DTEST_READINGS, READINGS, which must also block the low box), in simulated rooms: each
// VL53 zone reads the nearest surface over 5 x 5 directions across its cone, ±1.5 %
// noise. After a start-up turn: an empty room has its walls and no obstacle on the
// open floor, a box is blocked, a table top at 15-20 cm is an overhang, nothing beyond
// stairs down is drivable, a hallway with nothing in range shows no obstacles. A chair
// leg seen from 30 cm and then moved away is erased seen from 1 m; standing still
// 10 min 2 m from a box keeps it; someone walking across is gone 3 frames after.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_cell_map picoA/app/test/test_cell_map.c -lm && build/test_cell_map
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"
#include "../cell_map.c"

// --- The scene: the floor (z = 0) with an optional hole, and boxes ---
typedef struct { float x0, x1, y0, y1, z0, z1; bool on; } box_t;
#define MAX_BOXES 8
static box_t boxes[MAX_BOXES];
static box_t hole; // where the floor isn't (x, y only)
static float drop_x = 1e9f; // beyond this x the floor is DROP_M lower (stairs down)
#define DROP_M 0.25f
static int n_boxes;

static int add_box(float x0, float x1, float y0, float y1, float z0, float z1) {
    boxes[n_boxes] = (box_t){x0, x1, y0, y1, z0, z1, true};
    return n_boxes++;
}

static void walls(float half_m) { // a square room, walls 1 m tall
    add_box(half_m, half_m + 0.1f, -half_m, half_m, 0, 1);
    add_box(-half_m - 0.1f, -half_m, -half_m, half_m, 0, 1);
    add_box(-half_m, half_m, half_m, half_m + 0.1f, 0, 1);
    add_box(-half_m, half_m, -half_m - 0.1f, -half_m, 0, 1);
}

static void reset_scene(void) { n_boxes = 0; hole.on = false; drop_x = 1e9f; cell_map_clear(); }

static float hit_box(const box_t *b, const float o[3], const float d[3]) {
    float lo[3] = {b->x0, b->y0, b->z0}, hi[3] = {b->x1, b->y1, b->z1}, t0 = 0, t1 = 1e9f;
    for (int k = 0; k < 3; k++) {
        if (fabsf(d[k]) < 1e-9f) { if (o[k] < lo[k] || o[k] > hi[k]) return 1e9f; continue; }
        float a = (lo[k] - o[k]) / d[k], b2 = (hi[k] - o[k]) / d[k];
        if (a > b2) { float t = a; a = b2; b2 = t; }
        t0 = fmaxf(t0, a);
        t1 = fminf(t1, b2);
        if (t0 > t1) return 1e9f;
    }
    return t0;
}

static float trace(const float o[3], const float d[3]) {
    float best = 1e9f;
    if (d[2] < 0) {
        float t = -o[2] / d[2], x = o[0] + t * d[0], y = o[1] + t * d[1];
        bool in_hole = hole.on && x > hole.x0 && x < hole.x1 && y > hole.y0 && y < hole.y1;
        if (x > drop_x) { // over the drop: the step's face, or the lower floor
            float t_low = (-DROP_M - o[2]) / d[2];
            best = o[0] + t_low * d[0] > drop_x ? t_low : (drop_x - o[0]) / d[0];
        } else if (!in_hole) best = t;
    }
    for (int i = 0; i < n_boxes; i++) if (boxes[i].on) best = fminf(best, hit_box(&boxes[i], o, d));
    return best;
}

static float jitter(float v) { return v * 0.015f * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f); }

// One frame from `pose`: each zone the nearest surface in its cone, up to 4 m.
static void measure(range_frame_t *f, const pose_t *pose) {
    float origin[3], c = cosf(pose->yaw_rad), s = sinf(pose->yaw_rad);
    rangefinder_origin(origin);
    float o[3] = {pose->x_m + c * origin[0] - s * origin[1], pose->y_m + s * origin[0] + c * origin[1], origin[2]};
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float nearest = 1e9f;
        for (int a = 0; a < 5; a++)
            for (int b = 0; b < 5; b++) {
                float r[3];
                rangefinder_sub_ray_direction(i, -0.5f + 0.25f * (float)a, -0.5f + 0.25f * (float)b, pose->pitch_rad, r);
                float d[3] = {c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]};
                nearest = fminf(nearest, trace(o, d));
            }
        f->range_mm[i] = nearest > 4.0f ? RANGE_NO_TARGET : (uint16_t)(1000.0f * (nearest + jitter(nearest)));
        f->status[i] = 5;
    }
}

static void frame_at(pose_t pose) {
    range_frame_t f;
    measure(&f, &pose);
    cell_map_add(&f, &pose);
}

static void scan_at(float x, float y) { // the start-up turn: 390° at 30°/s, 15 frames a second
    for (float yaw = 0; yaw < 390 * RAD_PER_DEG; yaw += 2 * RAD_PER_DEG) frame_at((pose_t){x, y, yaw, 0});
}

static int count(float x0, float x1, float y0, float y1, column_t kind) {
    int n = 0;
    for (float x = x0 + 0.05f; x < x1; x += 0.1f)
        for (float y = y0 + 0.05f; y < y1; y += 0.1f) n += cell_map_column(x, y) == kind;
    return n;
}

int main(void) {
#ifdef TEST_READINGS // run_tests.sh runs this test with each variant
    cell_map_set_variant(CELL_MAP_READINGS, 0.9f);
    printf("variant READINGS\n");
#else
    printf("variant VOTES\n");
#endif
    srand(1);
    pose_t home = {0, 0, 0, 0};

    // An empty 4 x 4 m room: walls blocked, no obstacle on the open floor, the floor
    // around the robot drivable.
    reset_scene();
    walls(2.0f);
    scan_at(0, 0);
    cell_map_print(&home, 0);
    int false_blocked = count(-1.7f, 1.7f, -1.7f, 1.7f, COLUMN_BLOCKED);
    // Standing still, the floor is seen where rays end on it: in rings at each row's
    // distance (about 20, 30, 45 and 70 cm); between them it is free, floor not seen.
    int drivable = count(-0.8f, 0.8f, -0.8f, 0.8f, COLUMN_DRIVABLE);
    int free_near = drivable + count(-0.8f, 0.8f, -0.8f, 0.8f, COLUMN_OPEN);
    int wall_blocked = count(1.9f, 2.0f, -1.0f, 1.0f, COLUMN_BLOCKED);
    printf("empty room: %d blocked columns on the open floor; within 0.8 m %d of 256 free, %d of them drivable;"
           " %d of 20 wall columns blocked\n", false_blocked, free_near, drivable, wall_blocked);
    assert(false_blocked == 0);
    assert(free_near >= 250 && drivable >= 100);
    assert(wall_blocked >= 16);
    printf("ok: empty room\n");

    // A box 20 x 20 x 10 cm 60 cm ahead; a table top at 15-20 cm 60-90 cm to the left.
    reset_scene();
    walls(2.0f);
    add_box(0.6f, 0.8f, -0.1f, 0.1f, 0, 0.1f);
    add_box(-0.3f, 0.3f, 0.6f, 0.9f, 0.15f, 0.2f);
    scan_at(0, 0);
    cell_map_print(&home, 0);
    // The box's front face is seen (its back half hides behind it). Under the table,
    // KNOWN: the 4th row's cone touches the table's underside ~0.8 m away, so all its
    // rays stop there and the lower ones end in L1: false "blocked" that no ray from
    // here contradicts. Driving closer, rays pass under it and erase it.
    int table_blocked = count(-0.3f, 0.3f, 0.6f, 0.9f, COLUMN_BLOCKED);
    printf("box: %d of 2 front columns blocked; table: %d of 18 columns overhang, %d blocked (known, see the test)\n",
           count(0.6f, 0.7f, -0.1f, 0.1f, COLUMN_BLOCKED), count(-0.3f, 0.3f, 0.6f, 0.9f, COLUMN_OVERHANG), table_blocked);
    assert(count(0.6f, 0.7f, -0.1f, 0.1f, COLUMN_BLOCKED) == 2);
    assert(count(-0.3f, 0.3f, 0.6f, 0.9f, COLUMN_OVERHANG) >= 4);
    assert(table_blocked <= 8);
    printf("ok: a box is blocked, a table top at 15-20 cm an overhang\n");

    // A low box (7 cm tall, under the sensor's height) 50 cm to the right: its front
    // face blocked. Rays from the row above pass over it inside the same L1 cell
    // (3-13 cm); they must not erase it (found on the robot, 9 Oct).
    reset_scene();
    walls(2.0f);
    add_box(-0.2f, 0.2f, -0.6f, -0.5f, 0, 0.07f);
    scan_at(0, 0);
    int low_front = count(-0.2f, 0.2f, -0.6f, -0.5f, COLUMN_BLOCKED);
#ifdef TEST_READINGS
    printf("low box (7 cm) 50 cm to the right: %d of 4 front columns blocked\n", low_front);
    assert(low_front >= 3);
#else
    printf("low box (7 cm) 50 cm to the right: %d of 4 front columns blocked (KNOWN: should be 4; READINGS fixes it)\n", low_front);
#endif

    // A hole 30 x 40 cm in the floor 50-80 cm ahead. KNOWN: zones that see the floor
    // beside it stop all their rays at that distance, so a few rays into the hole mark
    // floor there; printed, not checked.
    reset_scene();
    walls(2.0f);
    hole = (box_t){0.5f, 0.8f, -0.2f, 0.2f, 0, 0, true};
    scan_at(0, 0);
    printf("hole 30 x 40 cm: %d of 12 columns drivable (known), %d no floor\n",
           count(0.5f, 0.8f, -0.2f, 0.2f, COLUMN_DRIVABLE), count(0.5f, 0.8f, -0.2f, 0.2f, COLUMN_NO_FLOOR));

    // Stairs down: the floor drops 25 cm along a line 50 cm ahead. Nothing beyond the
    // edge is drivable (drivable needs the floor seen, and the floor down there is
    // below the ground layer).
    reset_scene();
    walls(2.0f);
    drop_x = 0.5f;
    scan_at(0, 0);
    cell_map_print(&home, 0);
    int beyond = count(0.6f, 1.5f, -1.0f, 1.0f, COLUMN_DRIVABLE);
    printf("stairs down 50 cm ahead: %d drivable columns beyond the edge, %d marked no floor\n", beyond,
           count(0.5f, 1.9f, -1.9f, 1.9f, COLUMN_NO_FLOOR));
    assert(beyond == 0);
    printf("ok: nothing beyond a drop is drivable\n");

    // A hallway: nothing within 4 m but the floor. No obstacles anywhere.
    reset_scene();
    scan_at(0, 0);
    cell_map_print(&home, 0);
    printf("hallway: %d blocked columns\n", count(-3, 3, -3, 3, COLUMN_BLOCKED));
    assert(count(-3, 3, -3, 3, COLUMN_BLOCKED) == 0);
    printf("ok: a far floor with nothing in range shows no obstacles\n");

    // A chair leg (4 x 4 cm) seen from 30 cm, then moved away: seen from 1 m it is
    // erased within a few frames.
    reset_scene();
    walls(2.0f);
    int leg = add_box(0.42f, 0.46f, 0.02f, 0.06f, 0, 0.45f); // in the cell 40-50 cm ahead, 0-10 cm left
    for (int k = 0; k < 15; k++) frame_at(home);
    assert(cell_map_column(0.45f, 0.05f) == COLUMN_BLOCKED);
    float confidence;
    cell_map_cell(0.45f, 0.05f, 1, &confidence);
    printf("chair leg seen from 30 cm: blocked, confidence %.2f\n", (double)confidence);
    boxes[leg].on = false;
    pose_t back = {-0.6f, 0, 0, 0}; // 1 m from where the leg was
    int frames = 0;
    while (cell_map_column(0.45f, 0.05f) == COLUMN_BLOCKED && frames < 30) { frame_at(back); frames++; }
    printf("moved away, seen from 1 m: erased after %d frames\n", frames);
    assert(frames <= 5);
    printf("ok: a chair leg seen up close is erased from 1 m\n");

    // Standing still 10 min 2 m from a box seen up close: the box stays.
    reset_scene();
    walls(2.0f);
    add_box(0.6f, 0.8f, -0.1f, 0.1f, 0, 0.1f);
    scan_at(0, 0);
    int box_before = count(0.6f, 0.8f, -0.1f, 0.1f, COLUMN_BLOCKED);
    pose_t far = {-1.4f, 0, 0, 0};
    for (int k = 0; k < 15 * 600; k++) frame_at(far);
    int box_after = count(0.6f, 0.8f, -0.1f, 0.1f, COLUMN_BLOCKED);
    printf("standing 10 min 2 m away: box columns blocked %d before, %d after\n", box_before, box_after);
    assert(box_after == box_before);
    printf("ok: standing still far away doesn't erase what was explored\n");

    // Someone (30 cm wide, 1.7 m tall) walks across 1 m ahead at 1 m/s: blocked
    // while there, free again 3 frames after.
    reset_scene();
    walls(2.0f);
    for (int k = 0; k < 15; k++) frame_at(home);
    int person = add_box(0.9f, 1.1f, -0.9f, -0.6f, 0, 1.7f);
    bool seen = false;
    for (float y = -0.9f; y < 0.9f; y += 1.0f / 15) {
        boxes[person].y0 = y;
        boxes[person].y1 = y + 0.3f;
        frame_at(home);
        seen |= count(0.9f, 1.1f, y, y + 0.3f, COLUMN_BLOCKED) > 0;
    }
    boxes[person].on = false;
    int left = -1;
    for (int k = 0; k < 10 && left < 0; k++) {
        frame_at(home);
        if (count(0.8f, 1.2f, -1.0f, 1.0f, COLUMN_BLOCKED) == 0) left = k + 1;
    }
    printf("someone walking across at 1 m: seen %s, gone %d frames after they left\n", seen ? "yes" : "no", left);
    assert(seen && left >= 1 && left <= 4);
    printf("ok: someone walking through is erased once they're gone\n");

    printf("cell_map: all tests passed\n");
    return 0;
}
