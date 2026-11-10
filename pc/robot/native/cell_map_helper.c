// The PC's cell map (doc/DATA_FLOW_PLAN.md, step 5): the robot's own map code
// (picoA/app/cell_map.c, tof_rays.c, house_map.c, tof_format.c, room.c) in a program on the
// Mac, so the PC's map is built exactly as the robot builds its own. One process per
// boot, started with that boot's ToF mounting; pc/robot/src/maps/cell_map.ts builds
// it (cc) and talks to it.
//
//   cell_map_helper <tof x_m> <y_m> <z_m> <zone_deg> <zones_turn_deg>
// (the boot's mounting_tof_t, in this order: cell_map.ts passes the same five)
//
// Requests on stdin, one byte saying which, then its bytes; every request gets one
// answer on stdout: a u32 length, then that many bytes. Little-endian, floats f32.
//   'F' frame (277 B, compact)  add it to the map (cell_map_add)        -> nothing
//   'C'                         clear the map (window and tiles)         -> nothing
//   'D' x y yaw distance        a drive on the map now (room_drive)      -> f32 moved, u8 obstacle
//   'W' x y r                   a blocked or overhang column within r?   -> u8 (0, 1)
//   'V'                         the map: the window's origin cell (i32 x, y), its
//                               cells (3 x 60 x 60 B, cell_map_window), the tile count
//                               (u16), each tile's tx, ty (i16) and columns (200 B)
//   'R' frame (277 B, compact)  each zone's reading in the world (tof_rays_of): 64 x
//                               verdict u8 (255: no valid reading or no pose), the
//                               point x, y, z, the lowest and highest ray end (f32)
// The map code's own console lines (printf) go to stderr. EOF ends it.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "cell_map.h"
#include "house_map.h"
#include "room.h"
#include "tof_format.h"
#include "tof_rays.h"

#define NO_VERDICT 255

static int out_fd; // the answers; stdout itself goes to stderr

static void read_all(void *buf, size_t n) {
    if (n && fread(buf, 1, n, stdin) != n) exit(0); // EOF: the server is gone
}

static void write_all(const void *buf, size_t n) {
    const uint8_t *p = buf;
    while (n) {
        ssize_t k = write(out_fd, p, n);
        if (k <= 0) exit(1);
        p += k;
        n -= (size_t)k;
    }
}

// The answer being built: its bytes, then sent with its length first.
static uint8_t answer[CELL_LAYERS * CELL_MAP_GRID * CELL_MAP_GRID + 16 + HOUSE_TILES * (4 + HOUSE_TILE_BYTES)];
static uint32_t answer_len;

static void put(const void *bytes, size_t n) {
    memcpy(answer + answer_len, bytes, n);
    answer_len += (uint32_t)n;
}

static void send_answer(void) {
    write_all(&answer_len, 4);
    write_all(answer, answer_len);
    answer_len = 0;
}

static void read_floats(float *v, int n) { read_all(v, (size_t)n * sizeof(float)); }

static bool read_frame(posed_frame_t *f) {
    uint8_t bytes[TOF_COMPACT_BYTES];
    read_all(bytes, sizeof bytes);
    return tof_format_unpack(TOF_FORMAT_COMPACT, bytes, sizeof bytes, f);
}

// For robot check's turns (pc/robot/src/api/check.ts): a blocked or overhang column
// with a point closer than r to (x, y).
static bool obstacle_within(float x, float y, float r) {
    const float cell = 0.1f; // cell_map.c's GRID_M
    for (int cx = (int)floorf((x - r) / cell); cx <= (int)floorf((x + r) / cell); cx++)
        for (int cy = (int)floorf((y - r) / cell); cy <= (int)floorf((y + r) / cell); cy++) {
            float mx = ((float)cx + 0.5f) * cell, my = ((float)cy + 0.5f) * cell;
            column_t c = cell_map_column(mx, my);
            if (c != COLUMN_BLOCKED && c != COLUMN_OVERHANG) continue;
            float nx = fmaxf((float)cx * cell, fminf(x, (float)(cx + 1) * cell));
            float ny = fmaxf((float)cy * cell, fminf(y, (float)(cy + 1) * cell));
            if (hypotf(nx - x, ny - y) < r) return true;
        }
    return false;
}

static void view(void) {
    int ox, oy;
    const uint8_t *cells = cell_map_window(&ox, &oy);
    int32_t origin[2] = {ox, oy};
    put(origin, sizeof origin);
    put(cells, CELL_LAYERS * CELL_MAP_GRID * CELL_MAP_GRID);
    uint16_t n = (uint16_t)house_map_tiles();
    put(&n, 2);
    for (int i = 0; i < n; i++) {
        int16_t t[2];
        const uint8_t *columns;
        house_map_tile(i, &t[0], &t[1], &columns);
        put(t, sizeof t);
        put(columns, HOUSE_TILE_BYTES);
    }
}

static void rays(const posed_frame_t *f) {
    for (int i = 0; i < TOF_ZONES; i++) {
        uint8_t verdict = NO_VERDICT;
        float v[5] = {0};
        if (f->pose_known && tof_valid(&f->frame.zone[i])) {
            tof_rays_t r;
            tof_rays_of(f, i, (float)f->frame.zone[i].distance_mm / 1000.0f, &r);
            verdict = (uint8_t)r.verdict;
            v[0] = r.point[0]; v[1] = r.point[1]; v[2] = r.point[2]; v[3] = r.low_m; v[4] = r.high_m;
        }
        put(&verdict, 1);
        put(v, sizeof v);
    }
}

int main(int argc, char **argv) {
    if (argc != 6) {
        fprintf(stderr, "usage: cell_map_helper <tof x_m> <y_m> <z_m> <zone_deg> <zones_turn_deg>\n");
        return 2;
    }
    out_fd = dup(1);
    dup2(2, 1);
    setvbuf(stdout, NULL, _IOLBF, 0); // the map code's lines reach the server's log as they come
    mounting_t m = {.tof = {strtof(argv[1], NULL), strtof(argv[2], NULL), strtof(argv[3], NULL), strtof(argv[4], NULL), atoi(argv[5])}};
    tof_rays_init(&m);
    cell_map_clear();
    for (;;) {
        uint8_t op;
        read_all(&op, 1);
        posed_frame_t f;
        float a[4];
        switch (op) {
        case 'F': if (read_frame(&f)) cell_map_add(&f); break;
        case 'C': cell_map_clear(); break;
        case 'D': {
            read_floats(a, 4);
            room_drive_t d = room_drive(a[0], a[1], a[2], a[3]);
            uint8_t obstacle = d.obstacle;
            put(&d.moved_m, 4);
            put(&obstacle, 1);
            break;
        }
        case 'W': {
            read_floats(a, 3);
            uint8_t yes = obstacle_within(a[0], a[1], a[2]);
            put(&yes, 1);
            break;
        }
        case 'V': view(); break;
        case 'R': if (read_frame(&f)) rays(&f); break;
        default: fprintf(stderr, "cell_map_helper: unknown request %d\n", op); return 1;
        }
        send_answer();
    }
}
