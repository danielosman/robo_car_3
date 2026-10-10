#pragma once
// What the robot knows about its surroundings: the map (cell_map.h, doc/MAP_DESIGN.md),
// kept up to date from every ToF frame, placed where the robot was when the frame was
// measured. Nothing has to be learned first: every frame is mapped as it comes,
// in odometry's frame (from where PicoB started).
#include <stdbool.h>
#include "rangefinder.h"

bool surroundings_init(void);   // starts the ToF sensor (~2 s); false if it doesn't answer
void surroundings_update(void); // call every loop iteration
void surroundings_clear(void);  // forgets the map (what it keeps and the frame being added)
const range_frame_t *surroundings_last_frame(void); // NULL before the first one
