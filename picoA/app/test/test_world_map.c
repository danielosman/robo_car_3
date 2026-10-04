// Host test for picoA/app/world_map.c: the occupancy timers (R2), one update per
// cell per scan with hits winning, rays clearing what they pass, the window
// moving with the robot, change detection, free distance and forgetting. Run from
// the repo root:
//   cc -std=c11 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -o build/test_world_map picoA/app/test/test_world_map.c -lm && build/test_world_map
#include <assert.h>
#include <stdio.h>
#include "units.h"
#include "../world_map.c"

static scan_t scan;
static const pose_t at_origin = {0, 0, 0, 0};

// A scan with one ray from the sensor at (0, 0, 7 cm) to (x, y, z); the others unused.
static void one_ray(float x, float y, float z, ray_kind_t kind) {
    memset(&scan, 0, sizeof scan);
    scan.origin_z_m = 0.07f;
    scan.ray[0] = (ray_t){x, y, z, kind};
}

static void seconds(float s) { fake_now_us += (uint64_t)(s * 1e6f); }

int main(void) {
    fake_now_us = 5000000;
    map_clear();
    assert(map_cell(1.05f, 0.05f, 0) == CELL_UNKNOWN);

    // One sighting: occupied for 60 s, the cells before it free.
    one_ray(1.05f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    assert(map_cell(0.55f, 0.05f, 0) == CELL_FREE);
    seconds(59);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    seconds(2);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_FREE);

    // Four sightings make 240 s (capped); one miss leaves 180 (still occupied),
    // four in a row clear it.
    for (int i = 0; i < 6; i++) { one_ray(1.05f, 0.05f, 0.07f, RAY_HIT); map_add_scan(&scan, &at_origin); }
    one_ray(1.55f, 0.05f, 0.07f, RAY_CLEAR); // passes through the cell
    map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    seconds(179);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    seconds(-179);
    unsigned changes_before = map_changes();
    for (int i = 0; i < 3; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_FREE);
    assert(map_changes() == changes_before + 1); // a solid obstacle cleared: a change

    // Within one scan a cell is updated once, and a hit wins over a ray passing through.
    memset(&scan, 0, sizeof scan);
    scan.origin_z_m = 0.07f;
    for (int i = 0; i < 10; i++) scan.ray[i] = (ray_t){0.75f, -0.45f, 0.07f, RAY_HIT};
    scan.ray[10] = (ray_t){1.5f, -0.9f, 0.07f, RAY_CLEAR}; // through (0.75, -0.45)
    map_add_scan(&scan, &at_origin);
    seconds(61);
    assert(map_cell(0.75f, -0.45f, 0) == CELL_FREE); // 60 s, not 10 x 60 s
    seconds(-61);

    // A hit where the map had seen free space is a change.
    changes_before = map_changes();
    one_ray(0.55f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    assert(map_changes() == changes_before + 1);

    // Layers: 2-12 cm is layer 0, 12-22 cm layer 1; under 2 cm is the floor (no cell).
    one_ray(0.35f, 0.95f, 0.15f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.35f, 0.95f, 1) == CELL_OCCUPIED && map_cell(0.35f, 0.95f, 0) != CELL_OCCUPIED);

    // Free distance in layer 0: up to the first occupied cell (0.55 m ahead).
    float d = map_free_distance(0.05f, 0.05f, 1, 0, 2);
    assert(d > 0.4f && d <= 0.5f);

    // Rotation and translation: a robot at (1, 1) facing +y sees a hit 0.5 m ahead at (1, 1.5).
    pose_t p = {1.0f, 1.0f, PI_F / 2, 0};
    one_ray(0.5f, 0, 0.07f, RAY_HIT);
    map_add_scan(&scan, &p);
    assert(map_cell(1.0f, 1.5f, 0) == CELL_OCCUPIED);

    // The window follows the robot: 0.6 m away it re-centres; cells in both windows
    // stay, cells left behind are forgotten.
    one_ray(-1.85f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(-1.85f, 0.05f, 0) == CELL_OCCUPIED);
    pose_t moved = {0.6f, 0, 0, 0};
    one_ray(0.1f, 0, 0.07f, RAY_UNUSED);
    map_add_scan(&scan, &moved);
    assert(map_cell(-1.85f, 0.05f, 0) == CELL_UNKNOWN);  // fell off the back
    assert(map_cell(1.0f, 1.5f, 0) == CELL_OCCUPIED);    // kept
    assert(map_cell(2.55f, 0.05f, 0) == CELL_UNKNOWN);   // new at the front

    // Not seen for 240 s: unknown again (also across the sweeps that keep times from wrapping).
    for (int i = 0; i < 5; i++) { seconds(60); map_add_scan(&scan, &moved); }
    assert(map_cell(1.0f, 1.5f, 0) == CELL_UNKNOWN);

    printf("OK: world map timers, one update per scan, layers, free distance, moving window, changes\n");
    return 0;
}
