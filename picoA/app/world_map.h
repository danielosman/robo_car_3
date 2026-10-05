#pragma once
// What is around the robot (§4): 10 cm cells in four 10 cm layers from 2 cm above
// the floor (layer 0 blocks the robot), 4 x 4 m around the robot. The window moves
// with the robot in whole cells; what falls off its edge is forgotten. Cells change
// only when measured, never with time, and remember when they were last measured.
// A sighting makes a cell occupied; it is free again after 6 readings in a row
// that see through it (a sighting in between starts the count again).
// Layer 0 also remembers where the floor should have been seen and wasn't, 3 times
// with the floor not seen in between (a drop?), until the floor is seen there.
// World frame: the map frame of pose.h.
#include <stdbool.h>
#include "pose.h"
#include "rangefinder.h"

#define MAP_LAYERS 4

typedef enum {
    CELL_UNKNOWN,  // never seen
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_NO_FLOOR, // layer 0: free as far as seen, but the floor wasn't seen where it should be
} cell_state_t;

void map_clear(void);                                     // everything unknown
void map_add_scan(const scan_t *scan, const pose_t *pose); // the robot's pose when the scan was measured
cell_state_t map_cell(float x_m, float y_m, int layer);   // UNKNOWN outside the window
// Whether the floor was seen in the cell at (x, y), and not missed there since. Free
// space (nothing in the way) far away or over a drop isn't floor seen: only the floor
// rows see the floor, up to ~50 cm from the sensor. Under the robot (within 15 cm of
// where a scan was measured) counts as floor seen: it stands there.
bool map_floor_seen(float x_m, float y_m);
// How far from (x, y) along the unit direction (dx, dy) layer 0 stays free:
// up to the first cell that isn't free (occupied, no floor, unknown), the window's edge, or max_m.
float map_free_distance(float x_m, float y_m, float dx, float dy, float max_m);
// Changes since the map started (§4.6): something appeared where it was seen free,
// or a solid obstacle was seen through until it cleared.
unsigned map_changes(void);
// The whole window as text, x (pose.h's map frame: forward at the scan's start) up and y left:
// occupied in layer 0 "##", no floor "? ", occupied only above it "''" (an overhang,
// or a wall whose low part wasn't seen: layer 0 is only seen within ~1.5 m), free
// with the floor seen ". ", free without ": ", unknown "  ", the robot "()" with "**"
// 30 cm ahead of it.
void map_print(const pose_t *robot);
