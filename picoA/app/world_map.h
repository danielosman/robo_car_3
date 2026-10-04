#pragma once
// What is around the robot (§4): 10 cm cells in four 10 cm layers from 2 cm above
// the floor (layer 0 blocks the robot), 4 x 4 m around the robot. The window moves
// with the robot in whole cells; what falls off its edge is forgotten. Each cell
// remembers when it was last seen and stays occupied for up to 240 s: a sighting
// adds 60 s, seeing through it takes 60 s away. Not seen for 240 s: unknown again.
// World frame: the map frame of pose.h.
#include <stdbool.h>
#include "pose.h"
#include "rangefinder.h"

#define MAP_LAYERS 4

typedef enum { CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED } cell_state_t;

void map_clear(void);                                     // everything unknown
void map_add_scan(const scan_t *scan, const pose_t *pose); // the robot's pose when the scan was measured
cell_state_t map_cell(float x_m, float y_m, int layer);   // UNKNOWN outside the window
// How far from (x, y) along the unit direction (dx, dy) layer 0 stays free:
// up to the first occupied or unknown cell, the window's edge, or max_m.
float map_free_distance(float x_m, float y_m, float dx, float dy, float max_m);
// Changes since the map started (§4.6): something appeared where it was seen free,
// or a solid obstacle was seen through until it cleared.
unsigned map_changes(void);
// The whole window as text, x (pose.h's map frame: forward at the scan's start) up and y left:
// occupied in layer 0 "##", occupied only above it "''" (an overhang, or a wall
// whose low part wasn't seen: layer 0 is only seen within ~1 m), free ". ",
// unknown "  ", the robot "()" with "**" 30 cm ahead of it.
void map_print(const pose_t *robot);
