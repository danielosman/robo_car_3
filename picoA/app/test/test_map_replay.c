// Regression test on real recordings (doc/archive/MAP_DESIGN.md §9): the five scans of 10 Oct
// (picoA/app/test/data/*.frames, exported with pc/robot `npm run frames`: each frame
// as the robot's map received it) through picoA/app/cell_map.c. Daniel's room: a cup ~45 cm ahead, a box to the left, his office chair
// on wheels to the right, a cupboard behind; from scan 3 on the robot started 30 cm
// further left (the cup then ahead-right). Each object must have a blocked or
// overhang column in its region; nothing may be blocked within 30 cm of the robot
// (floor only). A move (`f`) pointed at the cup would stop before it: the first
// obstacle on its path (the robot's width plus 5 cm each side) is the cup's cell.
// Run from the repo root (run_tests.sh does):
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_map_replay picoA/app/test/test_map_replay.c -lm && build/test_map_replay
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../rangefinder.c"
#include "../cell_map.c"

#define MAX_FRAMES 400

typedef struct { const char *name; float x0, x1, y0, y1; } region_t;
typedef struct { const char *take; region_t objects[4]; } scan_t;

// Regions in each scan's own frame (x forward at power-up, y left), m.
static const scan_t scans[] = {
    {"8f3159b3-2", {{"cup", 0.4f, 0.6f, 0.1f, 0.2f}, {"box", -0.1f, 0.2f, 0.5f, 0.6f},
                    {"chair", -0.3f, 0.3f, -0.8f, -0.5f}, {"cupboard", -0.5f, -0.4f, -0.3f, 0.1f}}},
    {"ace8afeb-2", {{"cup", 0.4f, 0.6f, 0.1f, 0.2f}, {"box", -0.1f, 0.1f, 0.5f, 0.6f}}}, // half lost (TCP stall)
    {"123eef20-2", {{"cup", 0.4f, 0.5f, -0.3f, -0.2f}, {"box", 0.0f, 0.2f, 0.3f, 0.5f},
                    {"chair", -0.4f, 0.3f, -1.0f, -0.5f}, {"cupboard", -0.7f, -0.4f, -0.6f, -0.1f}}},
    {"0558a0ee-2", {{"cup", 0.4f, 0.5f, -0.3f, -0.2f}, {"box", 0.0f, 0.2f, 0.3f, 0.5f},
                    {"chair", -0.1f, 0.1f, -0.7f, -0.6f}, {"cupboard", -0.7f, -0.4f, -0.6f, -0.1f}}},
    {"0b9377b7-1", {{"cup", 0.4f, 0.5f, -0.3f, -0.2f}, {"box", 0.0f, 0.2f, 0.3f, 0.5f},
                    {"chair", -0.1f, 0.1f, -0.7f, -0.6f}, {"cupboard", -0.7f, -0.4f, -0.6f, -0.1f}}},
};

typedef struct { range_frame_t f; pose_t p; } frame_t;
static frame_t frames[MAX_FRAMES];

static int load(const char *take) {
    char path[128];
    snprintf(path, sizeof path, "picoA/app/test/data/%s.frames", take);
    FILE *in = fopen(path, "r");
    assert(in);
    static char line[4096];
    int n = 0;
    while (n < MAX_FRAMES && fgets(line, sizeof line, in)) {
        frame_t *fr = &frames[n];
        float t;
        int used;
        char *s = line;
        int got = sscanf(s, "%u %f %f %f %f %f%n", &fr->f.number, &t, &fr->p.x_m, &fr->p.y_m, &fr->p.yaw_rad, &fr->p.pitch_rad, &used);
        assert(got == 6);
        s += used;
        for (int i = 0; i < RANGEFINDER_RAYS; i++) {
            unsigned mm, st;
            got = sscanf(s, "%u %u%n", &mm, &st, &used);
            assert(got == 2);
            s += used;
            fr->f.range_mm[i] = (uint16_t)mm;
            fr->f.status[i] = (uint8_t)st;
        }
        n++;
    }
    fclose(in);
    return n;
}

static bool solid_at(float x, float y) {
    column_t c = column_of(grid_index(x), grid_index(y), 0.1f); // as the viewer and replay compare: ≥ 10 % confident
    return c == COLUMN_BLOCKED || c == COLUMN_OVERHANG;
}

static int solid_in(const region_t *r) {
    int n = 0;
    for (float x = r->x0 + 0.05f; x < r->x1; x += 0.1f)
        for (float y = r->y0 + 0.05f; y < r->y1; y += 0.1f) n += solid_at(x, y);
    return n;
}

static int solid_near(float radius_m) {
    int n = 0;
    for (float x = -radius_m + 0.05f; x < radius_m; x += 0.1f)
        for (float y = -radius_m + 0.05f; y < radius_m; y += 0.1f)
            if (hypotf(x, y) <= radius_m) n += solid_at(x, y);
    return n;
}

static void run(int n) {
    cell_map_clear();
    for (int i = 0; i < n; i++) cell_map_add(&frames[i].f, &frames[i].p);
}

int main(void) {
    for (size_t k = 0; k < sizeof scans / sizeof scans[0]; k++) {
        const scan_t *s = &scans[k];
        int n = load(s->take);
        assert(n > 50);
        int found[4];
        run(n);
        printf("%s (%d frames):", s->take, n);
        for (int o = 0; o < 4 && s->objects[o].name; o++) {
            found[o] = solid_in(&s->objects[o]);
            printf(" %s %d", s->objects[o].name, found[o]);
        }
        int near = solid_near(0.3f);
        // f towards the cup: its first obstacle is within a cell of the cup's region.
        const region_t *cup = &s->objects[0];
        float cx = (cup->x0 + cup->x1) / 2, cy = (cup->y0 + cup->y1) / 2, r = hypotf(cx, cy);
        float to_cup = cell_map_obstacle_along(0, 0, cx / r, cy / r, 0.235f / 2 + 0.05f, 1.5f);
        printf("; blocked within 30 cm: %d; f towards the cup (%.0f cm away): obstacle at %.0f cm\n", near,
               (double)(r * 100), (double)(to_cup * 100));
        assert(to_cup > r - 0.2f && to_cup < r);
        for (int o = 0; o < 4 && s->objects[o].name; o++) assert(found[o] > 0);
        assert(near == 0);
    }
    printf("map_replay: all tests passed\n");
    return 0;
}
