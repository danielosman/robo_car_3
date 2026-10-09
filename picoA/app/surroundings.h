#pragma once
// What the robot knows about its surroundings: the map (cell_map.h, doc/MAP_DESIGN.md),
// kept up to date from every ToF frame, placed where the robot was when the frame was
// measured. Nothing has to be learned first: the start-up scan's frames are mapped
// as they come.
#include <stdbool.h>
#include "rangefinder.h"

bool surroundings_init(void);   // starts the ToF sensor (~2 s); false if it doesn't answer
void surroundings_update(void); // call every loop iteration
void surroundings_restart(void); // forgets the map; its x axis becomes the robot's heading now
const range_frame_t *surroundings_last_frame(void); // NULL before the first one
