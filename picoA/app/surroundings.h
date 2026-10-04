#pragma once
// What the robot knows about its surroundings: the map (world_map.h), kept up to
// date from every ToF frame, placed where the robot was when the frame was
// measured. The floor has to be learned first, during a full turn: from
// surroundings_learn_start() the frames are kept, and surroundings_learn_finish()
// learns the floor from them and puts them on the map, so even the first turn
// maps low obstacles. Until then nothing is mapped.
#include <stdbool.h>
#include "rangefinder.h"

bool surroundings_init(void);   // starts the ToF sensor (~2 s); false if it doesn't answer
void surroundings_update(void); // call every loop iteration
void surroundings_learn_start(void); // forgets the map and the floor; the map's x axis becomes the robot's heading now
// Learns the floor, maps the kept frames and maps every frame from then on.
// Returns how many floor zones were learned (of rangefinder_floor_zones()) and
// prints a line about it.
int surroundings_learn_finish(void);
bool surroundings_mapping(void);       // the floor is learned and frames go on the map
const range_frame_t *surroundings_last_frame(void); // NULL before the first one
