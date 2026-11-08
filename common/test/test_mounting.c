// Host test for the mounting: robot_mounting.h, which common/mounting.cmake makes from
// common/mounting.json, holds the file's numbers, each in its own field. The file is
// read here on its own (a few lines that find a key inside a section), not by CMake.
// Run from the repo root (run_tests.sh does; it makes build/host/robot_mounting.h first).
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot_mounting.h"

static char json[8192];

// The number after "key": inside the section "section": {...}.
static double number(const char *section, const char *key) {
    char name[64];
    snprintf(name, sizeof name, "\"%s\"", section);
    const char *s = strstr(json, name);
    assert(s);
    const char *end = strchr(s, '}');
    snprintf(name, sizeof name, "\"%s\"", key);
    const char *k = strstr(s, name);
    if (!k || k > end) { fprintf(stderr, "mounting.json: %s.%s not found\n", section, key); exit(1); }
    return strtod(strchr(k, ':') + 1, NULL);
}

// The header's value is the file's, rounded to a float.
static void same(const char *section, const char *key, float built) {
    double want = number(section, key);
    printf("  %s.%s %g\n", section, key, (double)built);
    if (built != (float)want) { fprintf(stderr, "%s.%s: header %g, file %g\n", section, key, (double)built, want); exit(1); }
}

int main(void) {
    FILE *in = fopen("common/mounting.json", "r");
    assert(in);
    size_t n = fread(json, 1, sizeof json - 1, in);
    fclose(in);
    json[n] = 0;
    const mounting_t *m = &ROBOT_MOUNTING;
    printf("robot_mounting.h:\n");
    same("tof", "x_m", m->tof.x_m);
    same("tof", "y_m", m->tof.y_m);
    same("tof", "z_m", m->tof.z_m);
    same("tof", "zone_deg", m->tof.zone_deg);
    same("tof", "zones_turn_deg", (float)m->tof.zones_turn_deg);
    same("camera", "x_m", m->camera.x_m);
    same("camera", "y_m", m->camera.y_m);
    same("camera", "z_m", m->camera.z_m);
    same("camera", "f_px", m->camera.f_px);
    same("camera", "tilt_deg", m->camera.tilt_deg);
    same("camera", "width_px", (float)m->camera.width_px);
    same("camera", "height_px", (float)m->camera.height_px);
    // The IMU's matrix: its nine numbers in order, row by row.
    const char *p = strstr(json, "\"robot_from_sensor\"");
    assert(p);
    p = strchr(p, '[');
    for (int i = 0; i < 9; i++) {
        while (*p && !(*p == '-' || (*p >= '0' && *p <= '9'))) p++;
        char *after;
        long v = strtol(p, &after, 10);
        if (v != m->imu.robot_from_sensor[i / 3][i % 3]) { fprintf(stderr, "imu.robot_from_sensor[%d][%d]: header %d, file %ld\n", i / 3, i % 3, m->imu.robot_from_sensor[i / 3][i % 3], v); exit(1); }
        p = after;
    }
    printf("  imu.robot_from_sensor as in the file\nmounting: all tests passed\n");
    return 0;
}
