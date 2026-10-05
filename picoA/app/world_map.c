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
#define MISSES_TO_CLEAR 6          // empty readings in a row that clear an occupied cell
#define SOLID_SIGHTINGS 2          // occupied in this many scans: clearing it is a change
#define NO_FLOOR_SCANS  3          // "no floor" this many times, the floor not seen in between: "?"
                                   // (a shiny floor sometimes reflects a zone's light far away)
#define SWEEP_US       60000000    // keeps the 16-bit times from wrapping (18 h)
#define OLDEST_S       30000       // ages are kept up to ~8 h; older cells count as this old

typedef struct {
    uint16_t seen_at;        // robot seconds of the last measurement; 0 = never
    uint16_t frame;          // the last scan that updated it: one update per scan
    uint8_t misses_left;     // empty readings still needed to clear it; 0 = not occupied
    uint8_t sightings;       // scans that saw it occupied since it was last free
    uint8_t no_floor;        // layer 0: scans in a row that expected the floor here and didn't see it
    bool floor_seen;         // layer 0: the floor was seen here (and not missed since)
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

static int cell_index(float m) { return (int)floorf(m / CELL_M); }
static int wrap(int i) { return ((i % SIZE) + SIZE) % SIZE; }
static bool in_window(int ix, int iy) {
    return ix >= origin_x && ix < origin_x + SIZE && iy >= origin_y && iy < origin_y + SIZE;
}
static cell_t *cell_at(int layer, int ix, int iy) { return &cells[layer][wrap(ix)][wrap(iy)]; }

static cell_state_t state_of(const cell_t *c) {
    if (c->seen_at == 0) return CELL_UNKNOWN;
    if (c->misses_left > 0) return CELL_OCCUPIED;
    return c->no_floor >= NO_FLOOR_SCANS ? CELL_NO_FLOOR : CELL_FREE;
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
    if (state_of(c) == CELL_FREE) changes++;
    c->misses_left = MISSES_TO_CLEAR;
    if (c->sightings < UINT8_MAX) c->sightings++;
    c->seen_at = now;
}

static void miss(cell_t *c, uint16_t now) {
    if (c->misses_left > 0 && --c->misses_left == 0) {
        if (c->sightings >= SOLID_SIGHTINGS) changes++;
        c->sightings = 0;
    }
    c->seen_at = now;
}

typedef struct { float x, y, z; } point_t;

static point_t to_world(const pose_t *p, float x_m, float y_m, float z_m) {
    float c = cosf(p->yaw_rad), s = sinf(p->yaw_rad);
    return (point_t){p->x_m + c * x_m - s * y_m, p->y_m + s * x_m + c * y_m, z_m};
}

// One update per cell per scan: true the first time this scan reaches the cell.
static bool first_this_scan(cell_t *c) {
    if (c->frame == frame_id) return false;
    c->frame = frame_id;
    return true;
}

// The cell of layer 0 at a point, or NULL outside the window.
static cell_t *ground_at(point_t p) {
    int ix = cell_index(p.x), iy = cell_index(p.y);
    return in_window(ix, iy) ? cell_at(0, ix, iy) : NULL;
}

static void clear_at(point_t p, uint16_t now) {
    int ix = cell_index(p.x), iy = cell_index(p.y);
    // Under 2 cm the ray passed below anything standing on the floor tall enough to
    // block the robot, so layer 0 is free there too.
    int l = p.z < FIRST_LAYER_M ? 0 : layer_of(p.z);
    if (l < 0 || !in_window(ix, iy)) return;
    cell_t *c = cell_at(l, ix, iy);
    if (first_this_scan(c)) miss(c, now);
}

// Every cell the line from a to b passes through, before b (and b's if
// with_end), gets a miss, unless this scan already updated it (a hit always wins).
static void clear_along(point_t a, point_t b, bool with_end, uint16_t now) {
    float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    float length = sqrtf(dx * dx + dy * dy + dz * dz);
    int steps = (int)(length / STEP_M);
    for (int k = 0; k < steps; k++) {
        float f = (float)k * STEP_M / length;
        clear_at((point_t){a.x + f * dx, a.y + f * dy, a.z + f * dz}, now);
    }
    if (with_end) clear_at(b, now);
}

void map_add_scan(const scan_t *scan, const pose_t *pose) {
    uint16_t now = now_s();
    if (time_us_64() >= next_sweep_us) {
        // Very old times move up to OLDEST_S, so 16-bit times never wrap into looking fresh.
        next_sweep_us = time_us_64() + SWEEP_US;
        for (int l = 0; l < MAP_LAYERS; l++)
            for (int i = 0; i < SIZE; i++)
                for (int j = 0; j < SIZE; j++) {
                    cell_t *c = &cells[l][i][j];
                    if (c->seen_at && age_s(c->seen_at, now) > OLDEST_S) {
                        c->seen_at = (uint16_t)(now - OLDEST_S);
                        if (c->seen_at == 0) c->seen_at = UINT16_MAX; // 0 means never seen
                    }
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
        if (first_this_scan(c)) hit(c, now);
    }
    // Where the floor should be: seen there starts the count again, not seen there
    // counts (both in one scan: it counts).
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        const ray_t *r = &scan->ray[i];
        cell_t *c = r->kind == RAY_FLOOR ? ground_at(to_world(pose, r->x_m, r->y_m, r->z_m)) : NULL;
        if (c) c->no_floor = 0, c->floor_seen = true;
    }
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        const ray_t *r = &scan->ray[i];
        cell_t *c = r->kind == RAY_NO_FLOOR ? ground_at(to_world(pose, r->x_m, r->y_m, r->z_m)) : NULL;
        if (c) {
            if (c->no_floor < NO_FLOOR_SCANS && ++c->no_floor == NO_FLOOR_SCANS) c->floor_seen = false;
            c->seen_at = now;
        }
    }
    for (int i = 0; i < RANGEFINDER_RAYS; i++) {
        const ray_t *r = &scan->ray[i];
        if (r->kind == RAY_CLEAR || r->kind == RAY_FLOOR || r->kind == RAY_HIT)
            clear_along(origin, to_world(pose, r->x_m, r->y_m, r->z_m), r->kind != RAY_HIT, now);
    }
}

cell_state_t map_cell(float x_m, float y_m, int layer) {
    int ix = cell_index(x_m), iy = cell_index(y_m);
    if (!placed || layer < 0 || layer >= MAP_LAYERS || !in_window(ix, iy)) return CELL_UNKNOWN;
    return state_of(cell_at(layer, ix, iy));
}

bool map_floor_seen(float x_m, float y_m) {
    int ix = cell_index(x_m), iy = cell_index(y_m);
    return placed && in_window(ix, iy) && cell_at(0, ix, iy)->floor_seen;
}

float map_free_distance(float x_m, float y_m, float dx, float dy, float max_m) {
    for (float d = 0; d < max_m; d += STEP_M)
        if (map_cell(x_m + d * dx, y_m + d * dy, 0) != CELL_FREE) return d;
    return max_m;
}

unsigned map_changes(void) { return changes; }

void map_print(const pose_t *robot) {
    if (!placed) { printf("The map is empty\n"); return; }
    int rx = cell_index(robot->x_m), ry = cell_index(robot->y_m);
    int ax = cell_index(robot->x_m + 0.3f * cosf(robot->yaw_rad));
    int ay = cell_index(robot->y_m + 0.3f * sinf(robot->yaw_rad));
    printf("Map %d x %d m, 10 cm cells; up = where the robot faced when the scan started.\n"
           "## obstacle (2-12 cm), ? no floor where expected (a drop?), '' obstacle only seen above 12 cm,\n"
           ". floor seen, nothing on it, : nothing in the way, floor not seen (too far, or a drop),\n"
           "blank never seen, () robot, ** 30 cm ahead of it.\n",
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
                cell_state_t ground = state_of(cell_at(0, ix, iy));
                bool overhang = false;
                for (int l = 1; l < MAP_LAYERS; l++)
                    overhang |= state_of(cell_at(l, ix, iy)) == CELL_OCCUPIED;
                if (ground == CELL_OCCUPIED) s = "##";
                else if (ground == CELL_NO_FLOOR) s = "? ";
                else if (overhang) s = "''";
                else if (ground == CELL_FREE) s = cell_at(0, ix, iy)->floor_seen ? ". " : ": ";
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
