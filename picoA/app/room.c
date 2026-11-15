#include <math.h>
#include <stddef.h>
#include "cell_map.h"
#include "room.h"

// From the edge to the nearest obstacle within STOP_BEFORE_M + look_m; false if none.
// unseen_floor (may be NULL): that stop is floor not seen yet, not a thing.
static bool gap(float x_m, float y_m, float yaw_rad, float direction, float look_m, float *gap_m, bool *unseen_floor) {
    float look = EDGE_M + STOP_BEFORE_M + look_m;
    float along = cell_map_stop_along(x_m, y_m, direction * cosf(yaw_rad), direction * sinf(yaw_rad), PATH_HALF_M, HALF_WIDTH_M,
                                      look, direction > 0, unseen_floor);
    *gap_m = along - EDGE_M;
    return along < look;
}

float room_m(float x_m, float y_m, float yaw_rad, float direction, float look_m) {
    float g;
    return gap(x_m, y_m, yaw_rad, direction, look_m, &g, NULL) ? g - STOP_BEFORE_M : look_m;
}

float room_gap_m(float x_m, float y_m, float yaw_rad, float direction, float look_m) {
    float g;
    return gap(x_m, y_m, yaw_rad, direction, look_m, &g, NULL) ? g : STOP_BEFORE_M + look_m;
}

bool room_unseen_floor(float x_m, float y_m, float yaw_rad, float direction, float look_m) {
    float g;
    bool unseen = false;
    return gap(x_m, y_m, yaw_rad, direction, look_m, &g, &unseen) && unseen;
}

room_drive_t room_drive(float x_m, float y_m, float yaw_rad, float distance_m) {
    float want = fabsf(distance_m), room = room_m(x_m, y_m, yaw_rad, distance_m < 0 ? -1.0f : 1.0f, want);
    bool unseen = room < want && room_unseen_floor(x_m, y_m, yaw_rad, distance_m < 0 ? -1.0f : 1.0f, want);
    if (room <= ROOM_START_M) return (room_drive_t){0, true, unseen};
    return room < want ? (room_drive_t){room, true, unseen} : (room_drive_t){want, false, false};
}
