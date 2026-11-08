#include <math.h>
#include <stdio.h>
#include <string.h>
#include "tof_rays.h"
#include "cell_map.h"
#include "house_map.h"

#define GRID_M        0.1f
#define GRID          CELL_MAP_GRID // grid across: 6 m, three tiles of the whole-house map
#define TILE          HOUSE_TILE_COLS
#define TILE_M        (TILE * GRID_M)
#define FOLLOW_M      0.5f      // the robot this far out of the window's middle tile: it scrolls by a tile
#define RESTORED_CONF 1         // cells started from the whole-house map: any measurement overrides them
_Static_assert(GRID == 3 * TILE, "the window is 3 x 3 tiles");
#define G_TOP_M       TOF_FLOOR_TOP_M       // 3 cm: a ray ending lower is on the floor (tof_rays)
#define G_BOTTOM_M    (G_TOP_M - GRID_M)    // −7 cm: the ground layer's bottom; layers are GRID_M tall
#define NOTHING_M     1.0f      // a zone with nothing in range: free this far
#define NEAR_M        1.5f      // the ray_closeness sigmoid: 0.5 here...
#define NEAR_SLOPE_M  0.25f     // ...falling over about this much
#define ERASE_Q       0.3f      // a frame this confident may overwrite, given...
#define DOUBT_FRAMES  3         // ...this many in a row
#define FOOT_M        0.15f     // the robot stands and turns here: floor, L1 free
#define L2_BLIND_M    0.25f     // no zone sees L2 this close to the robot
#define WEIGHT_ONE    64        // weight 1 in the per-frame hit_w, pass_w
#define FULL_FLOOR_RAYS 9.0f    // the ground layer: rays' worth of agreement for full confidence
#define Z_STEP_M      0.002f    // hit heights kept in 2 mm steps above G_BOTTOM_M
#define NO_HIT_Z      255
#define PASS_BELOW_M  0.01f     // a pass counts if this close above the highest hit, or lower

// A cell: state (2 bits), doubt (2 bits), confidence 0-15 (4 bits).
#define C_STATE(c)  ((c) & 3)
#define C_DOUBT(c)  (((c) >> 2) & 3)
#define C_CONF(c)   ((c) >> 4)
#define C_PACK(state, doubt, conf) ((uint8_t)((state) | (doubt) << 2 | (conf) << 4))

static uint8_t grid[CELL_LAYERS][GRID][GRID];
// This frame, × WEIGHT_ONE: in L1 and L2 the strongest hit and pass in each cell (a
// reading's weight); in the ground layer the rays' weights added up.
static uint16_t hit_w[CELL_LAYERS][GRID][GRID], pass_w[CELL_LAYERS][GRID][GRID];
// The highest point a detection found in a cell (kept while it is occupied, or until
// a pass below it finds nothing), and in this frame (NO_HIT_Z: no detection marked
// it): what stands there reaches at least that high.
static uint8_t hit_z[CELL_LAYERS][GRID][GRID], frame_hit_z[CELL_LAYERS][GRID][GRID];
// The window: GRID x GRID columns from world cell (org_x, org_y) on (cell (cx, cy)
// covers cx * 10 cm … (cx + 1) * 10 cm), the middle tile (tile_x, tile_y) of 3 x 3
// tiles of the whole-house map. At power-up it is centred on the map's origin.
static int tile_x, tile_y, org_x = -GRID / 2, org_y = -GRID / 2;

void cell_map_clear(void) {
    memset(grid, 0, sizeof grid);
    memset(hit_w, 0, sizeof hit_w);
    memset(pass_w, 0, sizeof pass_w);
    memset(hit_z, NO_HIT_Z, sizeof hit_z);
    house_map_clear();
}

static uint8_t z_code(float z_m) {
    float q = (z_m - G_BOTTOM_M) / Z_STEP_M;
    return q <= 0 ? 0 : q >= NO_HIT_Z - 1 ? NO_HIT_Z - 1 : (uint8_t)q;
}

// The higher of two hit heights (NO_HIT_Z: none).
static uint8_t higher(uint8_t a, uint8_t b) { return a == NO_HIT_Z ? b : b == NO_HIT_Z ? a : a > b ? a : b; }

// A pass at or below this height in the cell counts against it (it would
// have hit what stands there); any pass if nothing was hit there.
static float pass_limit_m(int layer, int ix, int iy) {
    uint8_t z = hit_z[layer][ix][iy];
    return z == NO_HIT_Z ? 1e9f : G_BOTTOM_M + (float)z * Z_STEP_M + PASS_BELOW_M;
}

static int cell_of(float m) { return (int)floorf(m / GRID_M); } // the world cell
static bool in_window(int cx, int cy) { return cx >= org_x && cx < org_x + GRID && cy >= org_y && cy < org_y + GRID; }
static bool in_grid(int ix, int iy, int layer) { // window indexes
    return ix >= 0 && ix < GRID && iy >= 0 && iy < GRID && layer >= 0 && layer < CELL_LAYERS;
}

static float ray_closeness(float d_m) { return 1.0f / (1.0f + expf((d_m - NEAR_M) / NEAR_SLOPE_M)); }

static uint16_t weight_code(float w) { return (uint16_t)(w * WEIGHT_ONE + 0.5f); }
static void keep_max(uint16_t *kept, float w) { if (weight_code(w) > *kept) *kept = weight_code(w); }
static void add_weight(uint16_t *sum, float w) {
    uint32_t v = *sum + weight_code(w);
    *sum = v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

// What a ray's end cell gets: a hit, a pass (nothing in range: it went on), or
// nothing (an unsure reading's ray above the floor, or an obstacle: detect() marks it).
typedef enum { END_HIT, END_PASS, END_NONE } ray_end_t;

// Walks one ray (3D DDA over the grid): every cell it passes gets a pass, the cell
// it ends in what `end` says. A ground cell gets a pass only from a ray leaving it
// through its bottom. Below the ground layer it stops (it went through the floor's
// place: a hole, or noise); above L2 going up it stops too. A pass above
// the highest point hit in a cell doesn't count there (it wouldn't have hit it), and
// none counts in a cell a detection marked this frame: the other zones' rays pass
// beside a thin leg in the same 10 cm cell.
static void walk_ray(const float o[3], const float d[3], float length_m, ray_end_t end, float v) {
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
        int ix = c[0] - org_x, iy = c[1] - org_y, layer = c[2];
        if (layer < 0 || (layer >= CELL_LAYERS && d[2] >= 0)) return;
        if (in_grid(ix, iy, layer)) {
            // The ground layer is passed only by going through it: a ray crossing its
            // top (air above whatever is there) tells nothing about it.
            bool through = layer > 0 || (!last && axis == 2 && step[2] < 0);
            // The floor is a surface that fills a zone: its rays add up, so that the few
            // steep rays of a reading that end a little below the floor (one length for
            // all 9) don't outweigh the ones that hit it (taking the strongest ray,
            // floor the robot had stood on turned "no floor" later, 13 Oct).
            if (last && end == END_HIT && layer == 0) add_weight(&hit_w[0][ix][iy], ray_closeness(t) * v);
            else if (last && end == END_HIT) keep_max(&hit_w[layer][ix][iy], ray_closeness(length_m) * v);
            else if (last) {}
            else if (through && frame_hit_z[layer][ix][iy] == NO_HIT_Z) {
                float t_out = t_max[axis];
                float low_m = fminf(o[2] + t * d[2], o[2] + t_out * d[2]); // the ray's lowest point in the cell
                // As sure as a reading from where the ray leaves the cell: it is free that far.
                if (low_m > pass_limit_m(layer, ix, iy)) {}
                else if (layer == 0) add_weight(&pass_w[0][ix][iy], ray_closeness(t) * v);
                else keep_max(&pass_w[layer][ix][iy], ray_closeness(t_out) * v);
            }
        }
        if (last) return;
        t = t_max[axis];
        t_max[axis] += t_delta[axis];
        c[axis] += step[axis];
    }
}

// One frame's verdict on a cell, against what it keeps (doc/ARCHITECTURE.md, the map).
// `full`: the agreement that gives full confidence (one reading in L1 and L2, 9 rays
// in the ground layer).
static void decide_cell(uint8_t *cell, float h, float p, float full) {
    if (h == p) return;
    int verdict = h > p ? CELL_STATE_OCCUPIED : CELL_STATE_FREE;
    float q = fabsf(h - p) / fmaxf(h + p, full);
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
static void around_the_robot(float x_m, float y_m) {
    for (int cx = cell_of(x_m - L2_BLIND_M); cx <= cell_of(x_m + L2_BLIND_M); cx++)
        for (int cy = cell_of(y_m - L2_BLIND_M); cy <= cell_of(y_m + L2_BLIND_M); cy++) {
            if (!in_window(cx, cy)) continue;
            int ix = cx - org_x, iy = cy - org_y;
            float dx = ((float)cx + 0.5f) * GRID_M - x_m;
            float dy = ((float)cy + 0.5f) * GRID_M - y_m;
            float r = sqrtf(dx * dx + dy * dy);
            if (r <= FOOT_M) {
                grid[0][ix][iy] = C_PACK(CELL_STATE_OCCUPIED, 0, 15);
                grid[1][ix][iy] = C_PACK(CELL_STATE_FREE, 0, 15);
            }
            if (r <= L2_BLIND_M && C_STATE(grid[2][ix][iy]) == CELL_STATE_UNKNOWN)
                grid[2][ix][iy] = C_PACK(CELL_STATE_FREE, 0, 1);
        }
}

// A detection: a zone's sure target judged an obstacle, `length_m` along rays `d`
// (its 9, in the world) from `o`. Which of the cells they end in holds it isn't
// known, so all of them are marked (in L1 if they end in G), each as sure as the
// whole reading: closeness × v, so that a far detection is weak and can't overturn
// what was seen from closer (q < 0.3 beyond ~1.7 m).
typedef struct { uint8_t layer, ix, iy, z; } footprint_t; // a cell (window indexes), the highest point in it

static void detect(const float o[3], const float d[TOF_RAYS_N][3], float length_m, float v) {
    footprint_t cell[TOF_RAYS_N];
    int n = 0;
    for (int k = 0; k < TOF_RAYS_N; k++) {
        float z = o[2] + length_m * d[k][2];
        int ix = cell_of(o[0] + length_m * d[k][0]) - org_x, iy = cell_of(o[1] + length_m * d[k][1]) - org_y;
        int layer = (int)floorf((z - G_BOTTOM_M) / GRID_M);
        if (layer < 1) { layer = 1; z = G_TOP_M + 0.001f; }
        if (!in_grid(ix, iy, layer)) continue;
        int j = 0;
        while (j < n && !(cell[j].layer == layer && cell[j].ix == ix && cell[j].iy == iy)) j++;
        if (j == n) cell[n++] = (footprint_t){(uint8_t)layer, (uint8_t)ix, (uint8_t)iy, NO_HIT_Z};
        cell[j].z = higher(cell[j].z, z_code(z));
    }
    float w = ray_closeness(length_m) * v;
    if (weight_code(w) == 0) return; // too far to mark anything (beyond ~2.7 m): nor to keep passes out
    for (int j = 0; j < n; j++) {
        int l = cell[j].layer, ix = cell[j].ix, iy = cell[j].iy;
        keep_max(&hit_w[l][ix][iy], w); // the surest detection there
        frame_hit_z[l][ix][iy] = higher(frame_hit_z[l][ix][iy], cell[j].z);
    }
}

// How a zone's reading goes into the map.
typedef enum { ZONE_NONE, ZONE_PASS, ZONE_READING } zone_use_t;

// A zone with nothing in range is free NOTHING_M along its rays; one beyond the
// frame's limit free up to it; an unsure one tells nothing.
static zone_use_t zone_use(const tof_frame_t *f, int zone, float *length_m) {
    const tof_zone_t *z = &f->zone[zone];
    if (z->kind == TOF_NOTHING) { *length_m = NOTHING_M; return ZONE_PASS; }
    if (!tof_valid(z)) return ZONE_NONE;
    if (tof_beyond(f, zone)) { *length_m = (float)f->beyond_mm / 1000.0f; return ZONE_PASS; }
    *length_m = (float)z->distance_mm / 1000.0f;
    return ZONE_READING;
}

void cell_map_add(const posed_frame_t *f) {
    if (!f->pose_known) return; // no odometry for that moment (PicoB not connected)
    cell_map_follow(f->x_m, f->y_m);
    memset(hit_w, 0, sizeof hit_w);
    memset(pass_w, 0, sizeof pass_w);
    memset(frame_hit_z, NO_HIT_Z, sizeof frame_hit_z);
    tof_verdict_t verdict[TOF_ZONES];
    // First every reading's verdict and the detections, so that this frame's passes
    // already know where things were detected.
    for (int i = 0; i < TOF_ZONES; i++) {
        float length_m;
        verdict[i] = TOF_VERDICT_FLOOR;
        if (zone_use(&f->frame, i, &length_m) != ZONE_READING) continue;
        tof_rays_t r;
        tof_rays_of(f, i, length_m, &r);
        verdict[i] = r.verdict;
        if (r.verdict == TOF_VERDICT_OBSTACLE) detect(r.origin, r.dir, length_m, f->frame.zone[i].kind == TOF_SURE ? 1.0f : 0.5f);
    }
    for (int i = 0; i < TOF_ZONES; i++) {
        float length_m;
        zone_use_t use = zone_use(&f->frame, i, &length_m);
        if (use == ZONE_NONE) continue;
        tof_kind_t kind = (tof_kind_t)f->frame.zone[i].kind; // nothing in range counts fully, as a sure reading
        float v = kind == TOF_NOTHING || kind == TOF_SURE ? 1.0f : 0.5f; // also when "beyond" only passes
        tof_rays_t r;
        tof_rays_of(f, i, length_m, &r);
        for (int k = 0; k < TOF_RAYS_N; k++) {
            ray_end_t end = use == ZONE_PASS ? END_PASS : verdict[i] == TOF_VERDICT_FLOOR ? END_HIT
                          : verdict[i] == TOF_VERDICT_OBSTACLE ? END_NONE // detected above
                          : r.origin[2] + length_m * r.dir[k][2] < G_TOP_M ? END_HIT : END_NONE; // unsure
            walk_ray(r.origin, r.dir[k], length_m, end, v);
        }
    }
    for (int l = 0; l < CELL_LAYERS; l++)
        for (int ix = 0; ix < GRID; ix++)
            for (int iy = 0; iy < GRID; iy++)
                if (hit_w[l][ix][iy] || pass_w[l][ix][iy]) {
                    decide_cell(&grid[l][ix][iy], (float)hit_w[l][ix][iy] / WEIGHT_ONE,
                                (float)pass_w[l][ix][iy] / WEIGHT_ONE, l == 0 ? FULL_FLOOR_RAYS : 1.0f);
                    // A cell keeps the highest point a detection found in it, also while
                    // still free (doubted): rays passing over it don't count. A pass at or
                    // below it that found nothing forgets it, unless the cell stays occupied.
                    if (frame_hit_z[l][ix][iy] != NO_HIT_Z) hit_z[l][ix][iy] = higher(hit_z[l][ix][iy], frame_hit_z[l][ix][iy]);
                    else if (C_STATE(grid[l][ix][iy]) != CELL_STATE_OCCUPIED) hit_z[l][ix][iy] = NO_HIT_Z;
                }
    around_the_robot(f->x_m, f->y_m);
}

static column_t column_of(int ix, int iy, float min_confidence);

// A column from the whole-house map as cells, so confident they lose to any measurement.
static void restore(int ix, int iy, column_t c) {
    uint8_t occupied = C_PACK(CELL_STATE_OCCUPIED, 0, RESTORED_CONF), free = C_PACK(CELL_STATE_FREE, 0, RESTORED_CONF);
    switch (c) {
    case COLUMN_BLOCKED: grid[1][ix][iy] = occupied; break;
    case COLUMN_OVERHANG: grid[1][ix][iy] = free; grid[2][ix][iy] = occupied; break;
    case COLUMN_NO_FLOOR: grid[0][ix][iy] = free; break;
    case COLUMN_DRIVABLE: grid[0][ix][iy] = occupied; grid[1][ix][iy] = grid[2][ix][iy] = free; break;
    case COLUMN_OPEN: grid[1][ix][iy] = grid[2][ix][iy] = free; break;
    case COLUMN_UNKNOWN: break;
    }
}

// Moves a [layer][x][y] array by one tile along x (dx = ±1) or y (dy = ±1): what was
// at +d tiles comes to 0; what comes in is `empty`.
static void shift(uint8_t a[CELL_LAYERS][GRID][GRID], int dx, int dy, uint8_t empty) {
    const int keep = GRID - TILE;
    for (int l = 0; l < CELL_LAYERS; l++) {
        if (dx > 0) { memmove(a[l][0], a[l][TILE], (size_t)keep * GRID); memset(a[l][keep], empty, (size_t)TILE * GRID); }
        if (dx < 0) { memmove(a[l][TILE], a[l][0], (size_t)keep * GRID); memset(a[l][0], empty, (size_t)TILE * GRID); }
        for (int ix = 0; ix < GRID && dy; ix++) {
            if (dy > 0) { memmove(&a[l][ix][0], &a[l][ix][TILE], keep); memset(&a[l][ix][keep], empty, TILE); }
            else { memmove(&a[l][ix][TILE], &a[l][ix][0], keep); memset(&a[l][ix][0], empty, TILE); }
        }
    }
}

// One tile along x or y: the tiles leaving go into the whole-house map, the ones
// coming in start from it.
static void scroll(int dx, int dy) {
    for (int ix = 0; ix < GRID; ix++)
        for (int iy = 0; iy < GRID; iy++) {
            bool leaving = (dx > 0 && ix < TILE) || (dx < 0 && ix >= GRID - TILE) ||
                           (dy > 0 && iy < TILE) || (dy < 0 && iy >= GRID - TILE);
            if (leaving) house_map_put(org_x + ix, org_y + iy, column_of(ix, iy, 0));
        }
    shift(grid, dx, dy, 0);
    shift(hit_z, dx, dy, NO_HIT_Z);
    tile_x += dx;
    tile_y += dy;
    org_x = tile_x * TILE - GRID / 2;
    org_y = tile_y * TILE - GRID / 2;
    for (int ix = 0; ix < GRID; ix++)
        for (int iy = 0; iy < GRID; iy++) {
            bool coming = (dx < 0 && ix < TILE) || (dx > 0 && ix >= GRID - TILE) ||
                          (dy < 0 && iy < TILE) || (dy > 0 && iy >= GRID - TILE);
            if (coming) restore(ix, iy, house_map_get(org_x + ix, org_y + iy));
        }
}

void cell_map_follow(float x_m, float y_m) {
    // The middle tile covers 2 tile_x ± 1 m.
    while (x_m >= ((float)tile_x + 0.5f) * TILE_M + FOLLOW_M) scroll(1, 0);
    while (x_m < ((float)tile_x - 0.5f) * TILE_M - FOLLOW_M) scroll(-1, 0);
    while (y_m >= ((float)tile_y + 0.5f) * TILE_M + FOLLOW_M) scroll(0, 1);
    while (y_m < ((float)tile_y - 0.5f) * TILE_M - FOLLOW_M) scroll(0, -1);
}

const uint8_t *cell_map_window(int *org_cx, int *org_cy) {
    *org_cx = org_x;
    *org_cy = org_y;
    return &grid[0][0][0];
}

cell_state3_t cell_map_cell(float x_m, float y_m, int layer, float *confidence) {
    int ix = cell_of(x_m) - org_x, iy = cell_of(y_m) - org_y;
    uint8_t c = in_grid(ix, iy, layer) ? grid[layer][ix][iy] : 0;
    if (confidence) *confidence = (float)C_CONF(c) / 15;
    return (cell_state3_t)C_STATE(c);
}

static int layer_state(int ix, int iy, int layer, float min_confidence) {
    uint8_t c = grid[layer][ix][iy];
    return (float)C_CONF(c) / 15 + 1e-3f < min_confidence ? CELL_STATE_UNKNOWN : C_STATE(c);
}

// A column of the window (window indexes).
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

// A world column: the window's inside it, the whole-house map's outside.
static column_t column_at(int cx, int cy, float min_confidence) {
    return in_window(cx, cy) ? column_of(cx - org_x, cy - org_y, min_confidence) : house_map_get(cx, cy);
}

column_t cell_map_column(float x_m, float y_m) { return column_at(cell_of(x_m), cell_of(y_m), 0); }

float cell_map_obstacle_along(float x_m, float y_m, float dx, float dy, float half_width_m, float max_m) {
    float nearest = max_m;
    float ends_x[2] = {x_m, x_m + max_m * dx}, ends_y[2] = {y_m, y_m + max_m * dy};
    float wx = fabsf(dy) * half_width_m, wy = fabsf(dx) * half_width_m; // the path's half width along x and y
    for (int ix = cell_of(fminf(ends_x[0], ends_x[1]) - wx); ix <= cell_of(fmaxf(ends_x[0], ends_x[1]) + wx); ix++)
        for (int iy = cell_of(fminf(ends_y[0], ends_y[1]) - wy); iy <= cell_of(fmaxf(ends_y[0], ends_y[1]) + wy); iy++) {
            column_t c = column_at(ix, iy, 0);
            if (c != COLUMN_BLOCKED && c != COLUMN_OVERHANG) continue;
            float along_lo = 1e9f, along_hi = -1e9f, side_lo = 1e9f, side_hi = -1e9f;
            for (int k = 0; k < 4; k++) { // the cell's corners in the path's frame
                float cx = (float)(ix + (k & 1)) * GRID_M - x_m;
                float cy = (float)(iy + (k >> 1)) * GRID_M - y_m;
                float along = cx * dx + cy * dy, side = -cx * dy + cy * dx;
                along_lo = fminf(along_lo, along); along_hi = fmaxf(along_hi, along);
                side_lo = fminf(side_lo, side); side_hi = fmaxf(side_hi, side);
            }
            if (side_hi <= -half_width_m || side_lo >= half_width_m || along_hi <= 0) continue;
            nearest = fminf(nearest, fmaxf(along_lo, 0));
        }
    return nearest;
}

#define PRINT_GRID 40 // 4 m

void cell_map_print(const pose_t *robot, float min_confidence) {
    int rx = cell_of(robot->x_m), ry = cell_of(robot->y_m);
    int ax = cell_of(robot->x_m + 0.3f * cosf(robot->yaw_rad)), ay = cell_of(robot->y_m + 0.3f * sinf(robot->yaw_rad));
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
                          : symbol[column_at(ix, iy, min_confidence)];
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
