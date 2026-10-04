#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "world_map.h"

#define CELL_M         0.1f
#define SIZE           40          // cells per side: 4 m
#define FIRST_LAYER_M  0.02f       // below: the floor, driven over
#define LAYER_M        0.1f
#define RECENTRE_CELLS 5           // re-centre when the robot is 0.5 m from the window's centre
#define STEP_M         (CELL_M / 2) // ray walking
#define SEEN_FOR_S     60          // a sighting
#define MAX_OCCUPIED_S 240
#define REMEMBER_S     240         // unseen this long: unknown again
#define SOLID_S        120         // occupancy that counts as solid for change detection
#define SWEEP_US       60000000    // keeps the 16-bit times from wrapping (18 h)

typedef struct {
    uint16_t occupied_until; // robot seconds; occupied while in the future
    uint16_t seen_at;        // robot seconds; 0 = never (or forgotten)
    uint16_t frame;          // the last scan that updated it: one update per scan
    bool solid;              // has been occupied for SOLID_S or more since it was last free
} cell_t;

static cell_t cells[MAP_LAYERS][SIZE][SIZE]; // [layer][x mod SIZE][y mod SIZE]
static int origin_x, origin_y;               // world cell index of the window's lower corner
static bool placed;                          // the window has been centred on the robot
static uint16_t frame_id;
static unsigned changes;
static uint64_t next_sweep_us;

// Robot time in whole seconds, never 0 (0 means "never seen").
static uint16_t now_s(void) { return (uint16_t)(time_us_64() / 1000000u % 65535u + 1u); }
static int age_s(uint16_t t, uint16_t now) { return (int16_t)(uint16_t)(now - t); }
static int remaining_s(const cell_t *c, uint16_t now) {
    int r = (int16_t)(uint16_t)(c->occupied_until - now);
    return r > 0 ? r : 0;
}

static int cell_index(float m) { return (int)floorf(m / CELL_M); }
static int wrap(int i) { return ((i % SIZE) + SIZE) % SIZE; }
static bool in_window(int ix, int iy) {
    return ix >= origin_x && ix < origin_x + SIZE && iy >= origin_y && iy < origin_y + SIZE;
}
static cell_t *cell_at(int layer, int ix, int iy) { return &cells[layer][wrap(ix)][wrap(iy)]; }

static cell_state_t state_of(const cell_t *c, uint16_t now) {
    if (c->seen_at == 0 || age_s(c->seen_at, now) > REMEMBER_S) return CELL_UNKNOWN;
    return remaining_s(c, now) > 0 ? CELL_OCCUPIED : CELL_FREE;
}

static void forget(cell_t *c) { memset(c, 0, sizeof *c); }

void map_clear(void) {
    memset(cells, 0, sizeof cells);
    placed = false;
    frame_id = 0;
    changes = 0;
}

// Moves the window so the robot is at its centre, forgetting the cells that leave it.
static void recentre(float x_m, float y_m) {
    int want_x = cell_index(x_m) - SIZE / 2, want_y = cell_index(y_m) - SIZE / 2;
    if (placed && abs(want_x - origin_x) <= RECENTRE_CELLS && abs(want_y - origin_y) <= RECENTRE_CELLS) return;
    if (placed) {
        // A cell stays if it's in both the old and the new window; the others are forgotten.
        for (int ix = want_x; ix < want_x + SIZE; ix++)
            for (int iy = want_y; iy < want_y + SIZE; iy++)
                if (!in_window(ix, iy))
                    for (int l = 0; l < MAP_LAYERS; l++) forget(cell_at(l, ix, iy));
    }
    origin_x = want_x;
    origin_y = want_y;
    placed = true;
}

static int layer_of(float z_m) {
    if (z_m < FIRST_LAYER_M) return -1;
    int l = (int)((z_m - FIRST_LAYER_M) / LAYER_M);
    return l < MAP_LAYERS ? l : -1;
}

static void hit(cell_t *c, uint16_t now) {
    if (state_of(c, now) == CELL_FREE) changes++;
    int r = remaining_s(c, now) + SEEN_FOR_S;
    if (r > MAX_OCCUPIED_S) r = MAX_OCCUPIED_S;
    c->occupied_until = (uint16_t)(now + r);
    if (r >= SOLID_S) c->solid = true;
    c->seen_at = now;
}

static void miss(cell_t *c, uint16_t now) {
    int r = remaining_s(c, now) - SEEN_FOR_S;
    if (r <= 0) {
        if (c->solid) changes++;
        c->solid = false;
        r = 0;
    }
    c->occupied_until = (uint16_t)(now + r);
    c->seen_at = now;
}

typedef struct { float x, y, z; } point_t;

static point_t to_world(const pose_t *p, float x_m, float y_m, float z_m) {
    float c = cosf(p->yaw_rad), s = sinf(p->yaw_rad);
    return (point_t){p->x_m + c * x_m - s * y_m, p->y_m + s * x_m + c * y_m, z_m};
}

// Every cell the line from a to b passes through, before b, gets a miss, unless
// this scan already updated it (a hit always wins).
static void clear_along(point_t a, point_t b, uint16_t now) {
    float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    float length = sqrtf(dx * dx + dy * dy + dz * dz);
    int steps = (int)(length / STEP_M);
    for (int k = 0; k < steps; k++) {
        float f = (float)k * STEP_M / length;
        int ix = cell_index(a.x + f * dx), iy = cell_index(a.y + f * dy), l = layer_of(a.z + f * dz);
        if (l < 0 || !in_window(ix, iy)) continue;
        cell_t *c = cell_at(l, ix, iy);
        if (c->frame == frame_id) continue;
        c->frame = frame_id;
        miss(c, now);
    }
}

void map_add_scan(const scan_t *scan, const pose_t *pose) {
    uint16_t now = now_s();
    if (time_us_64() >= next_sweep_us) {
        // Forget what's too old, so 16-bit times never wrap into looking fresh.
        next_sweep_us = time_us_64() + SWEEP_US;
        for (int l = 0; l < MAP_LAYERS; l++)
            for (int i = 0; i < SIZE; i++)
                for (int j = 0; j < SIZE; j++) {
                    cell_t *c = &cells[l][i][j];
                    if (c->seen_at && age_s(c->seen_at, now) > REMEMBER_S) forget(c);
                    else if (!remaining_s(c, now)) c->occupied_until = now;
                }
    }
    recentre(pose->x_m, pose->y_m);
    if (++frame_id == 0) { // the frame stamps wrapped: none may look current
        for (int l = 0; l < MAP_LAYERS; l++)
            for (int i = 0; i < SIZE; i++)
                for (int j = 0; j < SIZE; j++) cells[l][i][j].frame = 0;
        frame_id = 1;
    }
    point_t origin = to_world(pose, scan->origin_x_m, scan->origin_y_m, scan->origin_z_m);
    // Hits first, so a ray passing through a cell another ray ends in doesn't clear it.
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        const ray_t *r = &scan->ray[i];
        if (r->kind != RAY_HIT) continue;
        point_t end = to_world(pose, r->x_m, r->y_m, r->z_m);
        int ix = cell_index(end.x), iy = cell_index(end.y), l = layer_of(end.z);
        if (l < 0 || !in_window(ix, iy)) continue;
        cell_t *c = cell_at(l, ix, iy);
        if (c->frame == frame_id) continue;
        c->frame = frame_id;
        hit(c, now);
    }
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        const ray_t *r = &scan->ray[i];
        if (r->kind == RAY_UNUSED) continue;
        clear_along(origin, to_world(pose, r->x_m, r->y_m, r->z_m), now);
    }
}

cell_state_t map_cell(float x_m, float y_m, int layer) {
    int ix = cell_index(x_m), iy = cell_index(y_m);
    if (!placed || layer < 0 || layer >= MAP_LAYERS || !in_window(ix, iy)) return CELL_UNKNOWN;
    return state_of(cell_at(layer, ix, iy), now_s());
}

float map_free_distance(float x_m, float y_m, float dx, float dy, float max_m) {
    for (float d = 0; d < max_m; d += STEP_M)
        if (map_cell(x_m + d * dx, y_m + d * dy, 0) != CELL_FREE) return d;
    return max_m;
}

unsigned map_changes(void) { return changes; }

void map_print(const pose_t *robot) {
    if (!placed) { printf("The map is empty\n"); return; }
    uint16_t now = now_s();
    int rx = cell_index(robot->x_m), ry = cell_index(robot->y_m);
    int ax = cell_index(robot->x_m + 0.3f * cosf(robot->yaw_rad));
    int ay = cell_index(robot->y_m + 0.3f * sinf(robot->yaw_rad));
    printf("Map %d x %d m, 10 cm cells; up = where the robot faced when the scan started.\n"
           "## obstacle (2-12 cm), '' obstacle only seen above 12 cm, . free, blank unknown,\n"
           "() robot, ** 30 cm ahead of it.\n",
           SIZE / 10, SIZE / 10);
    printf("+");
    for (int i = 0; i < SIZE; i++) printf("--");
    printf("+\n");
    for (int ix = origin_x + SIZE - 1; ix >= origin_x; ix--) {
        char line[2 * SIZE + 3];
        int n = 0;
        line[n++] = '|';
        for (int iy = origin_y + SIZE - 1; iy >= origin_y; iy--) {
            const char *s = "  ";
            if (ix == rx && iy == ry) s = "()";
            else if (ix == ax && iy == ay) s = "**";
            else {
                cell_state_t ground = state_of(cell_at(0, ix, iy), now);
                bool overhang = false;
                for (int l = 1; l < MAP_LAYERS; l++)
                    overhang |= state_of(cell_at(l, ix, iy), now) == CELL_OCCUPIED;
                if (ground == CELL_OCCUPIED) s = "##";
                else if (overhang) s = "''";
                else if (ground == CELL_FREE) s = ". ";
            }
            line[n++] = s[0];
            line[n++] = s[1];
        }
        line[n++] = '|';
        line[n] = '\0';
        printf("%s\n", line);
    }
    printf("+");
    for (int i = 0; i < SIZE; i++) printf("--");
    printf("+\n");
}
