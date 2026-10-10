// Replay (doc/TELEMETRY_PLAN.md §5): runs picoA/app/cell_map.c on a recorded take's
// frames (pc/robot: npm run frames) in both variants, VOTES (the robot's today) and
// READINGS (MAP_DESIGN.md §9), and prints both maps and every column where they
// differ, with its distance and bearing from where the take started. Built and run
// by tools/replay.sh:
//   tools/replay.sh <take_id> [floor margin, default 0.9]
// With --json (the viewer, pc/robot/src/replay.ts) it prints both maps' cells as
// JSON instead: {"frames": n, "votes": [...], "readings": [...]}, each cell
// [x, y, column, ground, L1, L2, confidence %] (column: column_t; layers: cell_state3_t,
// a layer counting as unknown below MIN_CONFIDENCE; confidence: the highest of the three).
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rangefinder.c"
#include "cell_map.c"

#define MAX_FRAMES 4000
#define MIN_CONFIDENCE 0.1f // compared and printed: cells set by a measurement this confident at least
                            // (a single far hit sets a cell with confidence ~0)

typedef struct { range_frame_t f; pose_t p; } frame_t;
static frame_t frames[MAX_FRAMES];
static int n_frames;

static void load(const char *path) {
    FILE *in = fopen(path, "r");
    if (!in) { perror(path); exit(1); }
    static char line[4096];
    while (n_frames < MAX_FRAMES && fgets(line, sizeof line, in)) {
        frame_t *fr = &frames[n_frames];
        memset(fr, 0, sizeof *fr);
        float t;
        int n;
        char *s = line;
        if (sscanf(s, "%u %f %f %f %f %f%n", &fr->f.number, &t, &fr->p.x_m, &fr->p.y_m, &fr->p.yaw_rad, &fr->p.pitch_rad, &n) != 6) continue;
        s += n;
        for (int i = 0; i < RANGEFINDER_RAYS; i++) {
            unsigned mm, st;
            if (sscanf(s, "%u %u%n", &mm, &st, &n) != 2) { fprintf(stderr, "bad line\n"); exit(1); }
            s += n;
            fr->f.range_mm[i] = (uint16_t)mm;
            fr->f.status[i] = (uint8_t)st;
        }
        n_frames++;
    }
    fclose(in);
}

static column_t result[2][GRID][GRID];
static bool json;

static void json_cells(void) {
    bool first = true;
    printf("[");
    for (int ix = 0; ix < GRID; ix++)
        for (int iy = 0; iy < GRID; iy++) {
            int st[CELL_LAYERS], conf = 0;
            bool known = false;
            for (int l = 0; l < CELL_LAYERS; l++) {
                st[l] = layer_state(ix, iy, l, MIN_CONFIDENCE);
                known |= st[l] != CELL_STATE_UNKNOWN;
                int c = C_CONF(grid[l][ix][iy]) * 100 / 15;
                if (st[l] != CELL_STATE_UNKNOWN && c > conf) conf = c;
            }
            if (!known) continue;
            float x = ((float)ix - GRID / 2.0f + 0.5f) * GRID_M, y = ((float)iy - GRID / 2.0f + 0.5f) * GRID_M;
            printf("%s[%.2f,%.2f,%d,%d,%d,%d,%d]", first ? "" : ",", (double)x, (double)y,
                   column_of(ix, iy, MIN_CONFIDENCE), st[0], st[1], st[2], conf);
            first = false;
        }
    printf("]");
}

static void run(int v, cell_map_variant_t variant, float margin, const pose_t *home) {
    cell_map_set_variant(variant, margin);
    cell_map_clear();
    for (int i = 0; i < n_frames; i++) cell_map_add(&frames[i].f, &frames[i].p);
    for (int ix = 0; ix < GRID; ix++)
        for (int iy = 0; iy < GRID; iy++) result[v][ix][iy] = column_of(ix, iy, MIN_CONFIDENCE);
    if (json) {
        printf("%s\"%s\":", v ? "," : "", variant == CELL_MAP_VOTES ? "votes" : "readings");
        json_cells();
        return;
    }
    printf("\n=== %s ===\n", variant == CELL_MAP_VOTES ? "VOTES (the robot today)" : "READINGS");
    cell_map_print(home, MIN_CONFIDENCE);
}

static bool solid(column_t c) { return c == COLUMN_BLOCKED || c == COLUMN_OVERHANG; }
static const char *name(column_t c) {
    static const char *const names[] = {"unknown", "BLOCKED", "overhang", "no floor", "drivable", "open"};
    return names[c];
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: replay_map <frames file> [floor margin] [--json]\n"); return 2; }
    float margin = 0.9f;
    for (int a = 2; a < argc; a++) {
        if (!strcmp(argv[a], "--json")) json = true;
        else margin = strtof(argv[a], NULL);
    }
    load(argv[1]);
    pose_t home = {0, 0, 0, 0}; // where the take's odometry frame starts: up = facing at power-up
    if (json) {
        printf("{\"frames\":%d,\"margin\":%.2f,", n_frames, (double)margin);
        run(0, CELL_MAP_VOTES, margin, &home);
        run(1, CELL_MAP_READINGS, margin, &home);
        printf("}\n");
        return 0;
    }
    if (!n_frames) { fprintf(stderr, "no frames\n"); return 1; }
    printf("%d frames; floor margin %.2f\n", n_frames, (double)margin);
    run(0, CELL_MAP_VOTES, margin, &home);
    run(1, CELL_MAP_READINGS, margin, &home);
    int solid_n[2] = {0}, diff = 0;
    printf("\nColumns that differ (x, y of the cell's centre; distance and bearing from the start, + = left):\n");
    for (int ix = 0; ix < GRID; ix++)
        for (int iy = 0; iy < GRID; iy++) {
            for (int v = 0; v < 2; v++) solid_n[v] += solid(result[v][ix][iy]);
            if (solid(result[0][ix][iy]) == solid(result[1][ix][iy])) continue;
            float x = ((float)ix - GRID / 2.0f + 0.5f) * GRID_M, y = ((float)iy - GRID / 2.0f + 0.5f) * GRID_M;
            printf("  %+5.2f %+5.2f  %4.0f cm %+4.0f deg: VOTES %-8s READINGS %s\n", (double)x, (double)y,
                   (double)(100 * hypotf(x, y)), (double)(atan2f(y, x) * DEG_PER_RAD), name(result[0][ix][iy]),
                   name(result[1][ix][iy]));
            diff++;
        }
    printf("%d columns differ; blocked or overhang: VOTES %d, READINGS %d (cells at least %.0f %% confident)\n", diff,
           solid_n[0], solid_n[1], (double)(MIN_CONFIDENCE * 100));
    return 0;
}
