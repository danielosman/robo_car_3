// Host test for picoA/app/cell_map.c (doc/ARCHITECTURE.md, the map) in simulated rooms: each
// VL53 zone reads the nearest surface over 5 x 5 directions across its cone, ±1.5 %
// noise. After a start-up turn: an empty room has its walls and no obstacle on the
// open floor, a box is blocked, a table top at 15-20 cm is an overhang, nothing beyond
// stairs down is drivable, a hallway with nothing in range shows no obstacles. A chair
// leg seen from 30 cm and then moved away is erased seen from 1 m; standing still
// 10 min 2 m from a box keeps it; someone walking across is gone 3 frames after. A
// thin leg driven at is blocked, on a cell line or not; a wall seen up close doesn't
// thicken again when the robot drives away.
// The whole-house map: columns in tiles, a known class overwrites, unknown keeps,
// dirty tiles. The window follows the robot down a 7 m hallway by whole tiles (not
// before it is 0.5 m out of the middle tile): a box seen at the start is kept in
// the whole-house map once it scrolls out, one 5.5 m away is mapped; back at the
// start the box comes back from the whole-house map, and a scan erases it once it
// is gone.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_cell_map picoA/app/test/test_cell_map.c -lm && build/test_cell_map
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../rangefinder.c"
#include "../cell_map.c"
#include "../house_map.c"

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

#define MERGE_M 0.6f // surfaces closer together than this are one target (ST's forum)

static uint16_t range_of(float m) { return m > 4.0f ? RANGE_NO_TARGET : (uint16_t)(1000.0f * (m + jitter(m))); }

// One frame from `pose`: each zone the nearest surface in its cone, up to 4 m, and
// the nearest one more than MERGE_M behind it.
static void measure(range_frame_t *f, const pose_t *pose) {
    float origin[3], c = cosf(pose->yaw_rad), s = sinf(pose->yaw_rad);
    rangefinder_origin(origin);
    float o[3] = {pose->x_m + c * origin[0] - s * origin[1], pose->y_m + s * origin[0] + c * origin[1], origin[2]};
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        float seen[25];
        float nearest = 1e9f, behind = 1e9f;
        for (int k = 0; k < 25; k++) {
            float r[3];
            rangefinder_sub_ray_direction(i, -0.5f + 0.25f * (float)(k / 5), -0.5f + 0.25f * (float)(k % 5), pose->pitch_rad, r);
            float d[3] = {c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]};
            seen[k] = trace(o, d);
            nearest = fminf(nearest, seen[k]);
        }
        for (int k = 0; k < 25; k++) if (seen[k] > nearest + MERGE_M) behind = fminf(behind, seen[k]);
        f->range_mm[i] = range_of(nearest);
        f->status[i] = 5;
        f->range2_mm[i] = f->range_mm[i] == RANGE_NO_TARGET ? RANGE_NO_TARGET : range_of(behind);
        f->status2[i] = 5;
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
    // The wall's face is on a cell boundary (x = 2.0 m): a hit lands on either side of it
    // with the range's noise, so a wall column counts as blocked if either cell is.
    int wall_blocked = 0;
    for (float y = -0.95f; y < 1.0f; y += 0.1f)
        wall_blocked += cell_map_column(1.95f, y) == COLUMN_BLOCKED || cell_map_column(2.05f, y) == COLUMN_BLOCKED;
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
    printf("low box (7 cm) 50 cm to the right: %d of 4 front columns blocked\n", low_front);
    assert(low_front >= 3);

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

    // A thin leg (2 x 2 cm, 70 cm tall) with the wall behind it, driven at from 2 m:
    // on a cell line and in a cell's middle. The other zones' rays pass beside it in
    // its 10 cm cell; it must still be blocked, and `f` stop before it (snapshot 35,
    // 12 Oct: the robot drove into such legs).
    for (int on_line = 1; on_line >= 0; on_line--) {
        reset_scene();
        walls(2.0f);
        float y0 = on_line ? -0.01f : 0.04f;
        add_box(0.55f, 0.57f, y0, y0 + 0.02f, 0, 0.7f);
        scan_at(-1.5f, 0);
        float x = -1.5f;
        for (; x <= 0; x += 0.02f) frame_at((pose_t){x, 0, 0, 0});
        float stop = cell_map_obstacle_along(0, y0 + 0.01f, 1, 0, 0.235f / 2 + 0.05f, 1.5f);
        printf("thin leg %s, driven at from 2 m to 55 cm: column %s, f stops at %.0f cm\n",
               on_line ? "on a cell line" : "in a cell's middle",
               cell_map_column(0.56f, y0 + 0.01f) == COLUMN_BLOCKED ? "blocked" : "NOT blocked", (double)(stop * 100));
        assert(cell_map_column(0.56f, y0 + 0.01f) == COLUMN_BLOCKED);
        assert(stop > 0.4f && stop <= 0.55f);
    }
    printf("ok: a thin leg is blocked\n");

    // A wall seen from 2 m (thick: a zone is 20 cm wide there), then from 40 cm, then
    // from 2 m again: what the close look cleared in front of the wall stays free.
    reset_scene();
    walls(2.0f);
    for (float x = -0.4f; x <= 1.6f; x += 0.02f) frame_at((pose_t){x, 0, 0, 0});
    int near_front = count(1.7f, 2.0f, -0.4f, 0.4f, COLUMN_BLOCKED);
    for (float x = 1.6f; x >= -0.4f; x -= 0.02f) frame_at((pose_t){x, 0, 0, 0});
    int far_front = count(1.7f, 2.0f, -0.4f, 0.4f, COLUMN_BLOCKED);
    printf("wall: blocked columns 0-30 cm in front of it: %d after driving to 40 cm, %d after driving back to 2.4 m\n",
           near_front, far_front);
    assert(far_front <= near_front);
    printf("ok: driving away doesn't thicken a wall seen up close\n");

    // The whole-house map by itself.
    house_map_clear();
    assert(house_map_tile_of(-10) == 0 && house_map_tile_of(9) == 0 && house_map_tile_of(10) == 1);
    assert(house_map_tile_of(-11) == -1 && house_map_tile_of(-30) == -1 && house_map_tile_of(-31) == -2);
    assert(house_map_get(-55, 3) == COLUMN_UNKNOWN && house_map_tiles() == 0);
    assert(house_map_put(-55, 3, COLUMN_UNKNOWN) && house_map_tiles() == 0); // unknown: nothing to keep
    house_map_put(-55, 3, COLUMN_BLOCKED);
    house_map_put(-54, 3, COLUMN_DRIVABLE);
    assert(house_map_get(-55, 3) == COLUMN_BLOCKED && house_map_get(-54, 3) == COLUMN_DRIVABLE);
    house_map_put(-55, 3, COLUMN_UNKNOWN); // keeps
    assert(house_map_get(-55, 3) == COLUMN_BLOCKED);
    house_map_put(-55, 3, COLUMN_OPEN);    // a known class overwrites
    assert(house_map_get(-55, 3) == COLUMN_OPEN && house_map_get(-55, 4) == COLUMN_UNKNOWN);
    int16_t tx, ty;
    const uint8_t *cols;
    assert(house_map_tiles() == 1);
    house_map_tile(0, &tx, &ty, &cols);
    assert(tx == -3 && ty == 0); // tile -3 holds columns -70 … -51
    // Column (-55, 3) is (15, 13) in its tile: byte (15 * 20 + 13) / 2, the high half.
    assert(cols[(15 * 20 + 13) / 2] >> 4 == COLUMN_OPEN && (cols[(16 * 20 + 13) / 2] >> 4) == COLUMN_DRIVABLE);
    assert(house_map_next_dirty(0) == 0);
    house_map_sent(0);
    assert(house_map_next_dirty(0) == -1);
    house_map_put(-55, 3, COLUMN_OPEN); // the same: still clean
    assert(house_map_next_dirty(0) == -1);
    house_map_put(-55, 3, COLUMN_BLOCKED);
    assert(house_map_next_dirty(0) == 0);
    house_map_sent(0);
    house_map_all_dirty();
    assert(house_map_next_dirty(0) == 0);
    uint32_t clears = house_map_clears();
    cell_map_clear(); // clears the whole-house map too
    assert(house_map_tiles() == 0 && house_map_clears() == clears + 1);
    printf("ok: whole-house map: tiles, known overwrites, unknown keeps, dirty tiles\n");

    // The window follows the robot down a hallway (walls 1 m to each side, an end
    // wall behind the start: rays reading nothing clear only 1 m).
    reset_scene();
    add_box(-3, 8, 1.0f, 1.1f, 0, 1);
    add_box(-3, 8, -1.1f, -1.0f, 0, 1);
    add_box(-3.1f, -3, -1, 1, 0, 1);
    int near_box = add_box(-1.8f, -1.6f, -0.1f, 0.1f, 0, 0.1f);
    add_box(5.4f, 5.6f, -0.1f, 0.1f, 0, 0.1f);
    scan_at(0, 0);
    int ox, oy;
    cell_map_window(&ox, &oy);
    assert(ox == -30 && oy == -30);
    assert(cell_map_column(-1.65f, 0.05f) == COLUMN_BLOCKED);
    cell_map_follow(1.45f, 0);
    cell_map_window(&ox, &oy);
    assert(ox == -30); // 0.45 m out of the middle tile: stays
    for (float x = 0; x <= 4.5f; x += 0.05f) {
        cell_map_follow(x, 0);
        frame_at((pose_t){x, 0, 0, 0});
    }
    cell_map_window(&ox, &oy);
    printf("hallway: driven to 4.5 m, the window starts at %.1f m; the box 5.4 m ahead: %d of 2 columns blocked;"
           " the one behind the start (now outside): %s, %d tiles kept\n", (double)(ox * 0.1f),
           count(5.4f, 5.5f, -0.1f, 0.1f, COLUMN_BLOCKED),
           cell_map_column(-1.65f, 0.05f) == COLUMN_BLOCKED ? "blocked" : "lost", house_map_tiles());
    assert(ox == 10 && oy == -30); // middle tile 2 (3-5 m)
    assert(count(5.4f, 5.5f, -0.1f, 0.1f, COLUMN_BLOCKED) == 2);
    assert(cell_map_column(-1.65f, 0.05f) == COLUMN_BLOCKED); // from the whole-house map
    assert(cell_map_column(0.05f, 1.05f) == COLUMN_BLOCKED);  // the wall at the start
    // Back at the start (no frames on the way): the box comes back from the
    // whole-house map, barely confident.
    cell_map_follow(0, 0);
    cell_map_window(&ox, &oy);
    assert(ox == -30);
    float conf;
    assert(cell_map_cell(-1.65f, 0.05f, 1, &conf) == CELL_STATE_OCCUPIED && conf < 0.1f);
    assert(cell_map_column(-1.65f, 0.05f) == COLUMN_BLOCKED && count(5.4f, 5.5f, -0.1f, 0.1f, COLUMN_BLOCKED) == 2);
    // The box taken away: a scan erases it, and the whole-house map gets that too
    // once it scrolls out again.
    boxes[near_box].on = false;
    scan_at(0, 0);
    assert(cell_map_column(-1.65f, 0.05f) != COLUMN_BLOCKED);
    cell_map_follow(4.5f, 0);
    assert(cell_map_column(-1.65f, 0.05f) != COLUMN_BLOCKED && cell_map_column(-1.65f, 0.05f) != COLUMN_UNKNOWN);
    printf("ok: the window follows the robot by tiles; what scrolls out is kept, and comes back\n");

    return 0;
}
