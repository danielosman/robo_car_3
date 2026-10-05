// Host test for picoA/app/world_map.c: occupied until 6 empty readings in a row
// (nothing changes with time), one update per
// cell per scan with hits winning, rays clearing what they pass, the window
// moving with the robot, change detection, free distance, floor rays clearing
// layer 0 to the floor, no-floor marks, and cells staying known. Run from
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

    // One sighting: occupied, the cells before it free. Time alone changes nothing.
    one_ray(1.05f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    assert(map_cell(0.55f, 0.05f, 0) == CELL_FREE);
    seconds(3600);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);

    // Seen through: free after 6 readings in a row, not 5; a sighting in between
    // starts the count again. Clearing what was seen in 2 or more scans is a change.
    one_ray(1.05f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    one_ray(1.55f, 0.05f, 0.07f, RAY_CLEAR); // passes through the cell
    for (int i = 0; i < 5; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    one_ray(1.05f, 0.05f, 0.07f, RAY_HIT);
    map_add_scan(&scan, &at_origin);
    one_ray(1.55f, 0.05f, 0.07f, RAY_CLEAR);
    for (int i = 0; i < 5; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_OCCUPIED);
    unsigned changes_before = map_changes();
    map_add_scan(&scan, &at_origin);
    assert(map_cell(1.05f, 0.05f, 0) == CELL_FREE);
    assert(map_changes() == changes_before + 1); // a solid obstacle cleared: a change

    // Within one scan a cell is updated once, and a hit wins over a ray passing
    // through: 10 rays ending in it are one sighting, so clearing it is no change.
    memset(&scan, 0, sizeof scan);
    scan.origin_z_m = 0.07f;
    for (int i = 0; i < 10; i++) scan.ray[i] = (ray_t){0.75f, -0.45f, 0.07f, RAY_HIT};
    scan.ray[10] = (ray_t){1.5f, -0.9f, 0.07f, RAY_CLEAR}; // through (0.75, -0.45)
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.75f, -0.45f, 0) == CELL_OCCUPIED);
    assert(cell_at(0, cell_index(0.75f), cell_index(-0.45f))->sightings == 1);
    one_ray(1.5f, -0.9f, 0.07f, RAY_CLEAR);
    changes_before = map_changes();
    for (int i = 0; i < 6; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(0.75f, -0.45f, 0) == CELL_FREE && map_changes() == changes_before);

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

    // A cell stays as last measured, also after hours (across the sweeps that keep
    // the 16-bit times from wrapping), and remembers how old the measurement is.
    for (int i = 0; i < 20 * 60; i++) { seconds(60); map_add_scan(&scan, &moved); }
    assert(map_cell(1.0f, 1.5f, 0) == CELL_OCCUPIED);
    const cell_t *old = cell_at(0, cell_index(1.0f), cell_index(1.5f));
    assert(old->seen_at != 0 && age_s(old->seen_at, now_s()) >= OLDEST_S - 60);

    // A floor ray: free all the way to where it meets the floor, also where it's
    // under 2 cm (a ray from 7 cm to the floor 0.3 m ahead is under 2 cm from 0.21 m).
    map_clear();
    one_ray(0.30f, 0.05f, 0, RAY_FLOOR);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.15f, 0.05f, 0) == CELL_FREE);
    assert(map_cell(0.25f, 0.05f, 0) == CELL_FREE && map_cell(0.30f, 0.05f, 0) == CELL_FREE);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_UNKNOWN);
    // The floor is seen where the ray ends, not along it; a clear ray (nothing within
    // 1 m, or over a drop) sees no floor.
    assert(map_floor_seen(0.30f, 0.05f) && !map_floor_seen(0.15f, 0.05f));
    one_ray(0.95f, 0.55f, 0.07f, RAY_CLEAR);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.85f, 0.45f, 0) == CELL_FREE && !map_floor_seen(0.85f, 0.45f));

    // No floor where it should be, 3 times with no floor seen in between: "?" until
    // the floor is seen there (a single reflection doesn't make one). A ray passing
    // over it doesn't clear it; free distance stops at it.
    one_ray(0.45f, 0.05f, 0, RAY_NO_FLOOR);
    for (int i = 0; i < 2; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_FREE);
    one_ray(0.45f, 0.05f, 0, RAY_FLOOR);
    map_add_scan(&scan, &at_origin);
    one_ray(0.45f, 0.05f, 0, RAY_NO_FLOOR);
    for (int i = 0; i < 2; i++) map_add_scan(&scan, &at_origin);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_FREE); // the floor seen in between
    assert(map_floor_seen(0.45f, 0.05f));
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_NO_FLOOR && !map_floor_seen(0.45f, 0.05f));
    assert(map_cell(0.35f, 0.05f, 0) == CELL_FREE); // the no-floor ray clears nothing new
    one_ray(0.95f, 0.05f, 0.07f, RAY_CLEAR);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_NO_FLOOR);
    d = map_free_distance(0.05f, 0.05f, 1, 0, 2);
    assert(d > 0.3f && d <= 0.4f);
    one_ray(0.45f, 0.05f, 0, RAY_FLOOR);
    map_add_scan(&scan, &at_origin);
    assert(map_cell(0.45f, 0.05f, 0) == CELL_FREE);

    printf("OK: world map 6 misses clear, no change with time, one update per scan, layers, free distance, moving window, changes, floor rays, no-floor marks, cells stay known\n");
    return 0;
}
