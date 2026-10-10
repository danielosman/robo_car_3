#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cell_map.h"

#define GRID_M        0.1f
#define GRID          60        // grid across: 6 m
#define G_BOTTOM_M    (-0.07f)  // the ground layer's bottom; layers are GRID_M tall
#define NOTHING_M     1.0f      // a zone with nothing in range: free this far
#define NEAR_M        1.5f      // the ray_closeness sigmoid: 0.5 here...
#define NEAR_SLOPE_M  0.25f     // ...falling over about this much
#define FULL_RAYS     9.0f      // rays' worth of agreement for full confidence
#define ERASE_Q       0.3f      // a frame this confident may overwrite, given...
#define DOUBT_FRAMES  3         // ...this many in a row
#define FOOT_M        0.15f     // the robot stands and turns here: floor, L1 free
#define L2_BLIND_M    0.25f     // no zone sees L2 this close to the robot
#define WEIGHT_ONE    64        // a ray of weight 1 in the per-frame sums
#define G_TOP_M       (G_BOTTOM_M + GRID_M) // 3 cm: above this an end point is no floor
#define Z_STEP_M      0.002f    // hit heights kept in 2 mm steps above G_BOTTOM_M
#define NO_HIT_Z      255
#define PASS_BELOW_M  0.01f     // READINGS: a pass counts if this close above the highest hit, or lower

// A cell: state (2 bits), doubt (2 bits), confidence 0-15 (4 bits).
#define C_STATE(c)  ((c) & 3)
#define C_DOUBT(c)  (((c) >> 2) & 3)
#define C_CONF(c)   ((c) >> 4)
#define C_PACK(state, doubt, conf) ((uint8_t)((state) | (doubt) << 2 | (conf) << 4))

static uint8_t grid[CELL_LAYERS][GRID][GRID];
static uint16_t hit_sum[CELL_LAYERS][GRID][GRID], pass_sum[CELL_LAYERS][GRID][GRID]; // this frame
// READINGS: the highest point hit in an occupied cell (kept), and in this frame: what
// stands there reaches at least that high.
static uint8_t hit_z[CELL_LAYERS][GRID][GRID], frame_hit_z[CELL_LAYERS][GRID][GRID];
static cell_map_variant_t variant = CELL_MAP_VOTES;
static float floor_margin = 0.9f;

void cell_map_set_variant(cell_map_variant_t v, float margin) {
    variant = v;
    floor_margin = margin;
}

void cell_map_clear(void) {
    memset(grid, 0, sizeof grid);
    memset(hit_sum, 0, sizeof hit_sum);
    memset(pass_sum, 0, sizeof pass_sum);
    memset(hit_z, NO_HIT_Z, sizeof hit_z);
}

static uint8_t z_code(float z_m) {
    float q = (z_m - G_BOTTOM_M) / Z_STEP_M;
    return q <= 0 ? 0 : q >= NO_HIT_Z - 1 ? NO_HIT_Z - 1 : (uint8_t)q;
}

// The higher of two hit heights (NO_HIT_Z: none).
static uint8_t higher(uint8_t a, uint8_t b) { return a == NO_HIT_Z ? b : b == NO_HIT_Z ? a : a > b ? a : b; }

// READINGS: a pass at or below this height in the cell counts against it (it would
// have hit what stands there); any pass if nothing was hit there.
static float pass_limit_m(int layer, int ix, int iy) {
    uint8_t z = higher(frame_hit_z[layer][ix][iy], hit_z[layer][ix][iy]);
    return z == NO_HIT_Z ? 1e9f : G_BOTTOM_M + (float)z * Z_STEP_M + PASS_BELOW_M;
}

static int grid_index(float m) { return (int)floorf(m / GRID_M) + GRID / 2; }
static bool in_grid(int ix, int iy, int layer) {
    return ix >= 0 && ix < GRID && iy >= 0 && iy < GRID && layer >= 0 && layer < CELL_LAYERS;
}

static float ray_closeness(float d_m) { return 1.0f / (1.0f + expf((d_m - NEAR_M) / NEAR_SLOPE_M)); }

static void add_weight(uint16_t *sum, float w) {
    uint32_t v = *sum + (uint32_t)(w * WEIGHT_ONE + 0.5f);
    *sum = v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

// What a ray's end cell gets: a hit, a pass (nothing in range: it went on), or
// nothing (READINGS: an unsure reading, or an obstacle's hit placed before); LIFTED
// (READINGS, an obstacle): a hit, in L1 if the ray ends in the ground layer, and its
// height noted for this frame's passes.
typedef enum { END_HIT, END_PASS, END_NONE, END_LIFTED } ray_end_t;

// Walks one ray (3D DDA over the grid): every cell it passes gets a pass, the cell
// it ends in what `end` says. A ground cell gets a pass only from a ray leaving it
// through its bottom. Below the ground layer it stops (it went through the floor's
// place: a hole, or noise); above L2 going up it stops too. READINGS: a pass above
// the highest point hit in a cell doesn't count there (it wouldn't have hit it).
static void walk_ray(const float o[3], const float d[3], float length_m, ray_end_t end, float v, bool passes) {
    float p[3] = {o[0] / GRID_M, o[1] / GRID_M, (o[2] - G_BOTTOM_M) / GRID_M};
    int c[3], step[3];
    float t_max[3], t_delta[3];
    for (int k = 0; k < 3; k++) {
        c[k] = (int)floorf(p[k]);
        step[k] = d[k] > 0 ? 1 : -1;
        float dk = fabsf(d[k]) > 1e-6f ? d[k] : 1e-6f;
        t_delta[k] = fabsf(GRID_M / dk);
        float boundary = (float)(d[k] > 0 ? c[k] + 1 : c[k]);
        t_max[k] = (boundary - p[k]) * GRID_M / dk;
    }
    float t = 0;
    for (;;) {
        int axis = t_max[0] < t_max[1] ? (t_max[0] < t_max[2] ? 0 : 2) : (t_max[1] < t_max[2] ? 1 : 2);
        bool last = t_max[axis] >= length_m;
        int ix = c[0] + GRID / 2, iy = c[1] + GRID / 2, layer = c[2];
        if (layer < 0 || (layer >= CELL_LAYERS && d[2] >= 0)) return;
        if (in_grid(ix, iy, layer)) {
            float w = ray_closeness(t) * v;
            // The ground layer is passed only by going through it: a ray crossing its
            // top (air above whatever is there) tells nothing about it.
            bool through = layer > 0 || (!last && axis == 2 && step[2] < 0);
            if (last && end == END_HIT) add_weight(&hit_sum[layer][ix][iy], w);
            else if (last && end == END_LIFTED) {
                int to = layer < 1 ? 1 : layer;
                float z = fmaxf(o[2] + length_m * d[2], G_TOP_M + 0.001f);
                if (to < CELL_LAYERS) {
                    add_weight(&hit_sum[to][ix][iy], w);
                    frame_hit_z[to][ix][iy] = higher(frame_hit_z[to][ix][iy], z_code(z));
                }
            } else if (last || !passes) {}
            else if (through) {
                float t_out = last ? length_m : t_max[axis];
                float low_m = fminf(o[2] + t * d[2], o[2] + t_out * d[2]); // the ray's lowest point in the cell
                if (variant == CELL_MAP_VOTES || low_m <= pass_limit_m(layer, ix, iy)) add_weight(&pass_sum[layer][ix][iy], w);
            }
        }
        if (last) return;
        t = t_max[axis];
        t_max[axis] += t_delta[axis];
        c[axis] += step[axis];
    }
}

// One frame's verdict on a cell, against what it keeps (MAP_DESIGN.md §4).
static void decide_cell(uint8_t *cell, float h, float p) {
    if (h == p) return;
    int verdict = h > p ? CELL_STATE_OCCUPIED : CELL_STATE_FREE;
    float q = fabsf(h - p) / fmaxf(h + p, FULL_RAYS);
    int qc = (int)(q * 15 + 0.5f);
    uint8_t c = *cell;
    if (C_STATE(c) == CELL_STATE_UNKNOWN || (float)C_CONF(c) / 15 < q) *cell = C_PACK(verdict, 0, qc);
    else if (C_STATE(c) == verdict) *cell = C_PACK(verdict, 0, C_CONF(c) > qc ? C_CONF(c) : qc);
    else if (q >= ERASE_Q) {
        int doubt = C_DOUBT(c) + 1;
        *cell = doubt >= DOUBT_FRAMES ? C_PACK(verdict, 0, qc) : C_PACK(C_STATE(c), doubt, C_CONF(c));
    }
}

// The robot stands and turns on floor; L2 close to it is never seen by any zone.
static void around_the_robot(const pose_t *pose) {
    for (int ix = grid_index(pose->x_m - L2_BLIND_M); ix <= grid_index(pose->x_m + L2_BLIND_M); ix++)
        for (int iy = grid_index(pose->y_m - L2_BLIND_M); iy <= grid_index(pose->y_m + L2_BLIND_M); iy++) {
            if (!in_grid(ix, iy, 0)) continue;
            float dx = ((float)(ix - GRID / 2) + 0.5f) * GRID_M - pose->x_m;
            float dy = ((float)(iy - GRID / 2) + 0.5f) * GRID_M - pose->y_m;
            float r = sqrtf(dx * dx + dy * dy);
            if (r <= FOOT_M) {
                grid[0][ix][iy] = C_PACK(CELL_STATE_OCCUPIED, 0, 15);
                grid[1][ix][iy] = C_PACK(CELL_STATE_FREE, 0, 15);
            }
            if (r <= L2_BLIND_M && C_STATE(grid[2][ix][iy]) == CELL_STATE_UNKNOWN)
                grid[2][ix][iy] = C_PACK(CELL_STATE_FREE, 0, 1);
        }
}

// READINGS: how a zone's reading is judged as a whole.
typedef enum { READ_OBSTACLE, READ_FLOOR, READ_UNSURE } reading_t;

// Closer than where the zone's cone first meets the floor (its lower edge, with the
// robot's pitch): no floor can be there, so an obstacle. Otherwise floor if all its
// rays end at floor height, unsure if they straddle it (grazing the floor, or a low
// thing on it: its rays at floor height mark the floor, the others nothing). Beyond
// the floor patch's far end its rays end below the floor: floor rules (a drop).
static reading_t judge(int zone, float length_m, float sensor_z_m, float pitch_rad, const float frac[3]) {
    float edge[3];
    rangefinder_sub_ray_direction(zone, 0.5f, 0, pitch_rad, edge);
    float floor_from_m = edge[2] < 0 ? sensor_z_m / -edge[2] : 1e9f;
    if (length_m < floor_margin * floor_from_m) return READ_OBSTACLE;
    for (int a = 0; a < 3; a++) {
        float r[3];
        rangefinder_sub_ray_direction(zone, frac[a], 0, pitch_rad, r);
        if (sensor_z_m + length_m * r[2] >= G_TOP_M) return READ_UNSURE;
    }
    return READ_FLOOR;
}

void cell_map_add(const range_frame_t *frame, const pose_t *pose) {
    memset(hit_sum, 0, sizeof hit_sum);
    memset(pass_sum, 0, sizeof pass_sum);
    memset(frame_hit_z, NO_HIT_Z, sizeof frame_hit_z);
    float origin[3], c = cosf(pose->yaw_rad), s = sinf(pose->yaw_rad);
    rangefinder_origin(origin);
    float o[3] = {pose->x_m + c * origin[0] - s * origin[1], pose->y_m + s * origin[0] + c * origin[1], origin[2]};
    static const float frac[3] = {-1.0f / 3, 0, 1.0f / 3}; // 1/6, 1/2, 5/6 of the zone
    reading_t judged_as[RANGEFINDER_RAYS];
    // READINGS: first every reading's judgement and the obstacles' hits, so that this
    // frame's passes already know where things were hit.
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        uint16_t mm = frame->range_mm[i];
        judged_as[i] = READ_FLOOR; // VOTES: every reading as the floor rules have it
        if (variant == CELL_MAP_VOTES || mm == RANGE_INVALID || mm == RANGE_NO_TARGET) continue;
        float length_m = (float)mm / 1000.0f, v = frame->status[i] == 5 ? 1.0f : 0.5f;
        judged_as[i] = judge(i, length_m, origin[2], pose->pitch_rad, frac);
        if (judged_as[i] != READ_OBSTACLE) continue;
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++) {
                float r[3];
                rangefinder_sub_ray_direction(i, frac[a], frac[b], pose->pitch_rad, r);
                float d[3] = {c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]};
                walk_ray(o, d, length_m, END_LIFTED, v, false); // its hit only: the whole reading is the obstacle
            }
    }
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        uint16_t mm = frame->range_mm[i];
        if (mm == RANGE_INVALID) continue;
        bool ends = mm != RANGE_NO_TARGET;
        float length_m = ends ? (float)mm / 1000.0f : NOTHING_M;
        float v = !ends || frame->status[i] == 5 ? 1.0f : 0.5f; // sure, or valid with less confidence
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++) {
                float r[3];
                rangefinder_sub_ray_direction(i, frac[a], frac[b], pose->pitch_rad, r);
                float d[3] = {c * r[0] - s * r[1], s * r[0] + c * r[1], r[2]};
                ray_end_t end = !ends ? END_PASS : judged_as[i] == READ_FLOOR ? END_HIT
                              : judged_as[i] == READ_OBSTACLE ? END_NONE // its hit placed above
                              : o[2] + length_m * d[2] < G_TOP_M ? END_HIT : END_NONE; // unsure
                walk_ray(o, d, length_m, end, v, true);
            }
    }
    for (int l = 0; l < CELL_LAYERS; l++)
        for (int ix = 0; ix < GRID; ix++)
            for (int iy = 0; iy < GRID; iy++)
                if (hit_sum[l][ix][iy] || pass_sum[l][ix][iy]) {
                    decide_cell(&grid[l][ix][iy], (float)hit_sum[l][ix][iy] / WEIGHT_ONE,
                                (float)pass_sum[l][ix][iy] / WEIGHT_ONE);
                    // READINGS: an occupied cell keeps the highest point hit in it; a freed one forgets it.
                    if (C_STATE(grid[l][ix][iy]) != CELL_STATE_OCCUPIED) hit_z[l][ix][iy] = NO_HIT_Z;
                    else hit_z[l][ix][iy] = higher(hit_z[l][ix][iy], frame_hit_z[l][ix][iy]);
                }
    around_the_robot(pose);
}

cell_state3_t cell_map_cell(float x_m, float y_m, int layer, float *confidence) {
    int ix = grid_index(x_m), iy = grid_index(y_m);
    uint8_t c = in_grid(ix, iy, layer) ? grid[layer][ix][iy] : 0;
    if (confidence) *confidence = (float)C_CONF(c) / 15;
    return (cell_state3_t)C_STATE(c);
}

static int layer_state(int ix, int iy, int layer, float min_confidence) {
    uint8_t c = grid[layer][ix][iy];
    return (float)C_CONF(c) / 15 + 1e-3f < min_confidence ? CELL_STATE_UNKNOWN : C_STATE(c);
}

static column_t column_of(int ix, int iy, float min_confidence) {
    if (!in_grid(ix, iy, 0)) return COLUMN_UNKNOWN;
    int g = layer_state(ix, iy, 0, min_confidence), l1 = layer_state(ix, iy, 1, min_confidence);
    int l2 = layer_state(ix, iy, 2, min_confidence);
    if (l1 == CELL_STATE_OCCUPIED) return COLUMN_BLOCKED;
    if (l2 == CELL_STATE_OCCUPIED) return COLUMN_OVERHANG;
    if (g == CELL_STATE_FREE) return COLUMN_NO_FLOOR;
    if (l1 != CELL_STATE_FREE || l2 != CELL_STATE_FREE) return COLUMN_UNKNOWN;
    return g == CELL_STATE_OCCUPIED ? COLUMN_DRIVABLE : COLUMN_OPEN;
}

column_t cell_map_column(float x_m, float y_m) { return column_of(grid_index(x_m), grid_index(y_m), 0); }

float cell_map_free_distance(float x_m, float y_m, float dx, float dy, float max_m) {
    for (float d = 0; d < max_m; d += GRID_M / 2) {
        column_t c = cell_map_column(x_m + d * dx, y_m + d * dy);
        if (c != COLUMN_OPEN && c != COLUMN_DRIVABLE) return d;
    }
    return max_m;
}

#define PRINT_GRID 40 // 4 m

void cell_map_print(const pose_t *robot, float min_confidence) {
    int rx = grid_index(robot->x_m), ry = grid_index(robot->y_m);
    int ax = grid_index(robot->x_m + 0.3f * cosf(robot->yaw_rad)), ay = grid_index(robot->y_m + 0.3f * sinf(robot->yaw_rad));
    printf("Map (9 rays per zone), 4 x 4 m around the robot, 10 cm cells, cells under %.0f %% confidence hidden;\n"
           "up = where the robot faced when the scan started. ## blocked (3-13 cm), '' overhang (13-23 cm),\n"
           "? no floor (rays went through its place), . drivable (floor seen, free above), : free, floor not seen,\n"
           "blank unknown, () robot, ** 30 cm ahead of it.\n", (double)(min_confidence * 100));
    printf("+");
    for (int i = 0; i < PRINT_GRID; i++) printf("--");
    printf("+\n");
    for (int ix = rx + PRINT_GRID / 2 - 1; ix >= rx - PRINT_GRID / 2; ix--) {
        char line[2 * PRINT_GRID + 3];
        int n = 0;
        line[n++] = '|';
        for (int iy = ry + PRINT_GRID / 2 - 1; iy >= ry - PRINT_GRID / 2; iy--) {
            static const char *const symbol[] = {"  ", "##", "''", "? ", ". ", ": "};
            const char *s = ix == rx && iy == ry ? "()" : ix == ax && iy == ay ? "**"
                          : symbol[column_of(ix, iy, min_confidence)];
            line[n++] = s[0];
            line[n++] = s[1];
        }
        line[n++] = '|';
        line[n] = '\0';
        printf("%s\n", line);
    }
    printf("+");
    for (int i = 0; i < PRINT_GRID; i++) printf("--");
    printf("+\n");
}
