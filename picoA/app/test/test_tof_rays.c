// Host test for picoA/app/tof_rays.c, the geometry from the mounting it is given:
// where the floor rows meet the floor with the ToF at 9 cm and at 7 cm, the map's
// fixed numbers at both, the floor limits, another mounting altogether; a zone's
// reading placed in the world for a posed frame (yaw, pitch, where the robot is):
// its rays, point and height span; the verdicts (obstacle, floor, unsure) at 9 and
// 7 cm, with the robot pitched, and the floor margin.
// Run from the repo root (run_tests.sh does; it makes build/host/robot_mounting.h first).
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../tof_rays.c"
#include "robot_mounting.h"
#include "mounting_before_lift.h"

// Where a ray meets a flat floor, along the ray (m); 0 if it doesn't look down.
static float floor_along_m(int ray) {
    float dir[3], origin[3];
    tof_rays_direction(ray, dir);
    tof_rays_origin(origin);
    return dir[2] < 0 ? origin[2] / -dir[2] : 0;
}

// Rows 5-8 (README: rows 1-8): where their centre rays meet the floor, cm.
static void floor_rings(const mounting_t *m, const float want_cm[4]) {
    tof_rays_init(m);
    printf("ToF %.0f cm up: rows 5-8 meet the floor at", (double)(m->tof.z_m * 100));
    for (int row = 5; row <= 8; row++) {
        float cm = floor_along_m((row - 1) * 8 + 3) * 100;
        printf(" %.0f", (double)cm);
        assert(fabsf(cm - want_cm[row - 5]) < 1);
    }
    printf(" cm\n");
    for (int row = 1; row <= 4; row++) assert(floor_along_m((row - 1) * 8 + 3) == 0); // rows 1-4 look up
}

// The map's fixed numbers hold at this height (doc/CHANGELOG.md, 7 Nov): every ray
// reaching L2's bottom (13 cm) does so ahead of the robot, inside the ToF's 45°, never
// beside or behind it (L2_BLIND_M); where row 8's lower edge first meets the floor.
static void map_numbers(const mounting_t *m, float l2_from_cm, float floor_from_cm) {
    tof_rays_init(m);
    float origin[3], nearest = 1e9f;
    tof_rays_origin(origin);
    for (int ray = 0; ray < 4 * 8; ray++) // rows 1-4 look up
        for (int k = 0; k < 4; k++) {
            float d[3];
            tof_rays_sub_direction(ray, -0.5f + (float)(k / 2), -0.5f + (float)(k % 2), 0, d);
            if (d[2] <= 0) continue;
            float t = (0.13f - origin[2]) / d[2], x = origin[0] + t * d[0], y = origin[1] + t * d[1];
            assert(x > origin[0] && fabsf(atan2f(y - origin[1], x - origin[0])) <= 22.5f * RAD_PER_DEG + 1e-4f);
            nearest = fminf(nearest, hypotf(x, y));
        }
    float d[3];
    tof_rays_sub_direction(7 * 8 + 3, 0.5f, 0, 0, d); // row 8's lower edge, ahead
    float floor_x = origin[0] + origin[2] / -d[2] * d[0];
    printf("ToF %.0f cm up: L2 first reached %.1f cm from the centre (ahead); the floor first seen %.1f cm ahead\n",
           (double)(m->tof.z_m * 100), (double)(nearest * 100), (double)(floor_x * 100));
    assert(fabsf(nearest * 100 - l2_from_cm) < 0.5f && fabsf(floor_x * 100 - floor_from_cm) < 0.5f);
}

static void geometry(void) {
    mounting_t before = MOUNTING_BEFORE_LIFT, lifted = MOUNTING_BEFORE_LIFT;
    lifted.tof.z_m = 0.09f;
    floor_rings(&before, (const float[4]){143, 48, 29, 21});
    floor_rings(&lifted, (const float[4]){183, 61, 37, 27});
    map_numbers(&before, 16.8f, 20.1f);
    map_numbers(&lifted, 12.2f, 24.9f);

    // The floor rows' limit: beyond where a zone's upper edge meets a flat floor (+15 %),
    // a reading is a reflection; the rows above have none. It grows with the height.
    for (int k = 0; k < 2; k++) {
        const mounting_t *m = k ? &lifted : &before;
        tof_rays_init(m);
        float row7_m = tof_rays_floor_limit_m(6 * 8 + 3), row8_m = tof_rays_floor_limit_m(7 * 8 + 3);
        printf("ToF %.0f cm up: floor limits row 7 %.0f cm, row 8 %.0f cm\n", (double)(m->tof.z_m * 100),
               (double)(row7_m * 100), (double)(row8_m * 100));
        assert(fabsf(row7_m - 1.15f * m->tof.z_m / sinf(11.25f * RAD_PER_DEG)) < 0.001f);
        assert(fabsf(row8_m - 1.15f * m->tof.z_m / sinf(16.875f * RAD_PER_DEG)) < 0.001f);
        assert(tof_rays_floor_limit_m(4 * 8 + 3) == 0 && tof_rays_floor_limit_m(3 * 8 + 3) == 0); // row 5's upper edge is level
    }

    // The origin and the zone angle are the mounting's.
    float origin[3];
    tof_rays_origin(origin);
    assert(origin[0] == lifted.tof.x_m && origin[1] == lifted.tof.y_m && origin[2] == 0.09f);
    assert(fabsf(tof_rays_zone_rad() - 5.625f * RAD_PER_DEG) < 1e-6f);

    // Another mounting altogether: 5 cm ahead, 1 cm left, zones of 6°.
    mounting_t other = MOUNTING_BEFORE_LIFT;
    other.tof.x_m = 0.05f, other.tof.y_m = 0.01f, other.tof.zone_deg = 6.0f;
    tof_rays_init(&other);
    tof_rays_origin(origin);
    assert(origin[0] == 0.05f && origin[1] == 0.01f && origin[2] == 0.07f);
    assert(fabsf(tof_rays_zone_rad() - 6.0f * RAD_PER_DEG) < 1e-6f);
    float dir[3];
    tof_rays_direction(7 * 8 + 0, dir); // row 8, column 1: 3.5 zones down, 3.5 left
    float down = 21.0f * RAD_PER_DEG, left = 21.0f * RAD_PER_DEG;
    assert(fabsf(dir[0] - cosf(down) * cosf(left)) < 1e-6f && fabsf(dir[1] - cosf(down) * sinf(left)) < 1e-6f);
    assert(fabsf(dir[2] + sinf(down)) < 1e-6f);
    assert(fabsf(tof_rays_floor_limit_m(6 * 8 + 3) - 1.15f * 0.07f / sinf(12.0f * RAD_PER_DEG)) < 0.001f);
    printf("ok: the geometry follows the mounting\n");
}

// A posed frame at (x, y), yaw, pitch.
static posed_frame_t at(float x, float y, float yaw_deg, float pitch_deg) {
    posed_frame_t f = {.x_m = x, .y_m = y, .yaw_rad = yaw_deg * RAD_PER_DEG, .pitch_rad = pitch_deg * RAD_PER_DEG,
                       .pose_known = true};
    return f;
}

static bool near(float a, float b) { return fabsf(a - b) < 1e-5f; }

// A zone's reading in the world: the sensor where the mounting puts it on the robot,
// the rays turned with the robot, the point at the reading's distance along the
// centre ray, the heights of the rays' ends.
static void placed(void) {
    tof_rays_init(&ROBOT_MOUNTING);
    tof_rays_t r;
    posed_frame_t f = at(1.0f, 2.0f, 90, 0); // facing +y
    int zone = 4 * 8 + 3;                     // row 5, column 4: just left of centre, half a zone down
    tof_rays_of(&f, zone, 1.0f, &r);
    const mounting_tof_t *m = &ROBOT_MOUNTING.tof;
    assert(near(r.origin[0], 1.0f - m->y_m) && near(r.origin[1], 2.0f + m->x_m) && near(r.origin[2], m->z_m));
    float d[3];
    tof_rays_direction(zone, d);
    assert(near(r.dir[4][0], -d[1]) && near(r.dir[4][1], d[0]) && near(r.dir[4][2], d[2])); // turned 90° left
    for (int k = 0; k < 3; k++) assert(near(r.point[k], r.origin[k] + r.dir[4][k]));
    assert(r.length_m == 1.0f);
    float half = 2.8125f * RAD_PER_DEG, third = 1.875f * RAD_PER_DEG; // its centre half a zone below level
    assert(near(r.high_m, m->z_m - sinf(half - third)) && near(r.low_m, m->z_m - sinf(half + third)));
    for (int k = 0; k < TOF_RAYS_N; k++) assert(near(hypotf(hypotf(r.dir[k][0], r.dir[k][1]), r.dir[k][2]), 1));
    // Nose up 2°: every ray 2° higher.
    posed_frame_t up = at(1.0f, 2.0f, 90, 2);
    tof_rays_t ru;
    tof_rays_of(&up, zone, 1.0f, &ru);
    assert(near(asinf(ru.dir[4][2]) - asinf(r.dir[4][2]), 2 * RAD_PER_DEG));
    printf("ok: a zone's reading in the world: origin, rays, point, height span; pitch raises them\n");
}

// The verdict of a reading `cm` long in row `row` (1-8), column 4.
static tof_verdict_t verdict_of(int row, float cm, float pitch_deg) {
    tof_rays_t r;
    posed_frame_t f = at(0, 0, 0, pitch_deg);
    tof_rays_of(&f, (row - 1) * 8 + 3, cm / 100, &r);
    return r.verdict;
}

// Where a row's cone first meets the floor (its lower edge), cm.
static float floor_from_cm(const mounting_t *m, int row) {
    return m->tof.z_m / sinf(((float)row - 4.5f + 0.5f) * 5.625f * RAD_PER_DEG) * 100;
}

static void verdicts(void) {
    mounting_t lifted = ROBOT_MOUNTING, before = MOUNTING_BEFORE_LIFT;
    assert(lifted.tof.z_m == 0.09f && before.tof.z_m == 0.07f);
    for (int k = 0; k < 2; k++) {
        const mounting_t *m = k ? &before : &lifted;
        tof_rays_init(m);
        float row8 = floor_from_cm(m, 8), row6 = floor_from_cm(m, 6);
        // Closer than 90 % of where the cone meets the floor: an obstacle.
        assert(verdict_of(8, 0.89f * row8, 0) == TOF_VERDICT_OBSTACLE && verdict_of(8, 0.91f * row8, 0) == TOF_VERDICT_FLOOR);
        assert(verdict_of(6, 0.89f * row6, 0) == TOF_VERDICT_OBSTACLE);
        // Row 6 at its floor: floor; just past the obstacle limit its top rays end above 3 cm: unsure.
        float row6_centre = m->tof.z_m / sinf(1.5f * 5.625f * RAD_PER_DEG) * 100;
        assert(verdict_of(6, row6_centre, 0) == TOF_VERDICT_FLOOR);
        assert(verdict_of(6, 0.91f * row6, 0) == TOF_VERDICT_UNSURE);
        // Rows 1-4 never see the floor: anything they read is an obstacle, however far.
        assert(verdict_of(4, 400, 0) == TOF_VERDICT_OBSTACLE && verdict_of(1, 50, 0) == TOF_VERDICT_OBSTACLE);
        // Beyond the floor (a drop): floor rules.
        assert(verdict_of(8, 3 * row8, 0) == TOF_VERDICT_FLOOR);
        // Nose down 3°: the cone meets the floor sooner, so a reading at 90 % of the level
        // limit is floor (or unsure), no longer an obstacle.
        assert(verdict_of(6, 0.85f * row6, -3) != TOF_VERDICT_OBSTACLE && verdict_of(6, 0.85f * row6, 0) == TOF_VERDICT_OBSTACLE);
        printf("ok: ToF %.0f cm up: row 8 an obstacle closer than %.0f cm, row 6 closer than %.0f cm, unsure"
               " just beyond; rows 1-4 always; pitch moves it\n", (double)(m->tof.z_m * 100), (double)(0.9f * row8),
               (double)(0.9f * row6));
    }
    // The floor margin (replay only): 0.8 makes 85 % floor-ish, not an obstacle.
    tof_rays_init(&lifted);
    float row8 = floor_from_cm(&lifted, 8);
    tof_rays_set_floor_margin(0.8f);
    assert(verdict_of(8, 0.85f * row8, 0) != TOF_VERDICT_OBSTACLE);
    tof_rays_set_floor_margin(TOF_FLOOR_MARGIN);
    assert(verdict_of(8, 0.85f * row8, 0) == TOF_VERDICT_OBSTACLE);
    printf("ok: the floor margin\n");
}

// The judge cell_map.c had until 7 Nov (3 rays down the zone's middle column), for
// comparing: the verdict must agree with it for every zone, length and pitch.
static tof_verdict_t old_judge(int zone, float length_m, float z_m, float pitch_rad) {
    static const float frac[3] = {-1.0f / 3, 0, 1.0f / 3};
    float edge[3];
    tof_rays_sub_direction(zone, 0.5f, 0, pitch_rad, edge);
    float floor_from_m = edge[2] < 0 ? z_m / -edge[2] : 1e9f;
    if (length_m < 0.9f * floor_from_m) return TOF_VERDICT_OBSTACLE;
    for (int a = 0; a < 3; a++) {
        float r[3];
        tof_rays_sub_direction(zone, frac[a], 0, pitch_rad, r);
        if (z_m + length_m * r[2] >= 0.03f) return TOF_VERDICT_UNSURE;
    }
    return TOF_VERDICT_FLOOR;
}

static void same_as_old_judge(void) {
    const mounting_t *ms[2] = {&ROBOT_MOUNTING, &MOUNTING_BEFORE_LIFT};
    const float pitches_deg[] = {-4, -1.5f, -0.9f, 0, 0.7f, 2, 5};
    int n = 0, seen[3] = {0};
    for (int k = 0; k < 2; k++) {
        tof_rays_init(ms[k]);
        for (size_t p = 0; p < sizeof pitches_deg / sizeof pitches_deg[0]; p++)
            for (int zone = 0; zone < TOF_ZONES; zone++)
                for (float len = 0.05f; len < 3.0f; len += 0.013f, n++) {
                    tof_rays_t r;
                    posed_frame_t f = at(0.3f, -0.2f, 33, pitches_deg[p]);
                    tof_rays_of(&f, zone, len, &r);
                    assert(r.verdict == old_judge(zone, len, ms[k]->tof.z_m, f.pitch_rad));
                    seen[r.verdict]++;
                }
    }
    assert(seen[0] && seen[1] && seen[2]);
    printf("ok: the verdict is the old judge's in %d cases (7 pitches, 9 and 7 cm): %d floor, %d obstacle, %d unsure\n",
           n, seen[0], seen[1], seen[2]);
}

int main(void) {
    geometry();
    placed();
    verdicts();
    same_as_old_judge();
    printf("tof_rays: all tests passed\n");
    return 0;
}
