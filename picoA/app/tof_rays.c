#include <assert.h>
#include <math.h>
#include "units.h"
#include "tof_rays.h"

#define FIRST_FLOOR_ROW 4     // index: rows 5-8 see the floor
#define DROP_SHARE      1.15f // a floor zone's reading this much farther than where its upper
                              // edge meets a flat floor is a reflection (or a drop)

static mounting_tof_t mount;
static bool mounted;
static float zone_rad;
static float floor_margin = TOF_FLOOR_MARGIN;
// Each zone's centre ray with the robot level: how far below level (rad) and how far left.
static float zone_down_rad[TOF_ZONES], zone_left_rad[TOF_ZONES];
static const float third[3] = {-1.0f / 3, 0, 1.0f / 3}; // 1/6, 1/2, 5/6 of the zone

void tof_rays_init(const mounting_t *m) {
    mount = m->tof;
    mounted = mount.zone_deg > 0 && mount.z_m > 0;
    assert(mounted);
    zone_rad = mount.zone_deg * RAD_PER_DEG;
    for (int zone = 0; zone < TOF_ZONES; zone++) {
        zone_down_rad[zone] = ((float)(zone / TOF_COLS) - 3.5f) * zone_rad;
        zone_left_rad[zone] = (3.5f - (float)(zone % TOF_COLS)) * zone_rad;
    }
}

void tof_rays_set_floor_margin(float margin) { floor_margin = margin; }

// Unit direction of a ray down_rad below level and left_rad to the left, with the
// robot pitched nose up by pitch_rad.
static void direction(float down_rad, float left_rad, float pitch_rad, float out[3]) {
    float a = down_rad - pitch_rad;
    out[0] = cosf(a) * cosf(left_rad);
    out[1] = cosf(a) * sinf(left_rad);
    out[2] = -sinf(a);
}

void tof_rays_origin(float origin[3]) {
    assert(mounted);
    origin[0] = mount.x_m;
    origin[1] = mount.y_m;
    origin[2] = mount.z_m;
}

void tof_rays_direction(int zone, float dir[3]) {
    assert(mounted);
    direction(zone_down_rad[zone], zone_left_rad[zone], 0, dir);
}

void tof_rays_sub_direction(int zone, float down_frac, float left_frac, float pitch_rad, float dir[3]) {
    assert(mounted);
    direction(zone_down_rad[zone] + down_frac * zone_rad, zone_left_rad[zone] + left_frac * zone_rad, pitch_rad, dir);
}

float tof_rays_zone_rad(void) { return zone_rad; }

int tof_rays_first_floor_row(void) { return FIRST_FLOOR_ROW; }

float tof_rays_floor_limit_m(int zone) {
    assert(mounted);
    float far_down_rad = zone_down_rad[zone] - zone_rad / 2; // where its upper edge meets the floor
    if (zone / TOF_COLS < FIRST_FLOOR_ROW || far_down_rad <= 0) return 0;
    return DROP_SHARE * mount.z_m / sinf(far_down_rad);
}

// Closer than where the zone's cone first meets the floor (its lower edge, with the
// robot's pitch): no floor can be there, so an obstacle. Otherwise floor if all its
// rays end at floor height, unsure if they straddle it (grazing the floor, or a low
// thing on it). Beyond the floor patch's far end its rays end below the floor: floor
// (a drop).
static tof_verdict_t verdict(int zone, float length_m, float pitch_rad, float high_m) {
    float edge[3];
    tof_rays_sub_direction(zone, 0.5f, 0, pitch_rad, edge);
    float floor_from_m = edge[2] < 0 ? mount.z_m / -edge[2] : 1e9f;
    if (length_m < floor_margin * floor_from_m) return TOF_VERDICT_OBSTACLE;
    return high_m >= TOF_FLOOR_TOP_M ? TOF_VERDICT_UNSURE : TOF_VERDICT_FLOOR;
}

void tof_rays_of(const posed_frame_t *f, int zone, float length_m, tof_rays_t *out) {
    assert(mounted);
    float c = cosf(f->yaw_rad), s = sinf(f->yaw_rad);
    out->origin[0] = f->x_m + c * mount.x_m - s * mount.y_m;
    out->origin[1] = f->y_m + s * mount.x_m + c * mount.y_m;
    out->origin[2] = mount.z_m;
    out->length_m = length_m;
    out->low_m = 1e9f;
    out->high_m = -1e9f;
    for (int k = 0; k < TOF_RAYS_N; k++) {
        float r[3];
        tof_rays_sub_direction(zone, third[k / 3], third[k % 3], f->pitch_rad, r);
        out->dir[k][0] = c * r[0] - s * r[1];
        out->dir[k][1] = s * r[0] + c * r[1];
        out->dir[k][2] = r[2];
        float z = mount.z_m + length_m * r[2];
        out->low_m = fminf(out->low_m, z);
        out->high_m = fmaxf(out->high_m, z);
    }
    for (int a = 0; a < 3; a++) out->point[a] = out->origin[a] + length_m * out->dir[4][a];
    out->verdict = verdict(zone, length_m, f->pitch_rad, out->high_m);
}
