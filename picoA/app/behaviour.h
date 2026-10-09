#pragma once
// What the robot decides to do (§7). The start-up scan: the robot turns 390° in
// place (a full turn plus 30° of overlap), mapping what's around it (no map is
// printed: m does), turns to face the most open direction and prints its
// heading. Then it watches: motors on, it turns towards the biggest movement the
// VL53 sees (still or turning), following it at its own angular speed plus a
// correction, at most ~29°/s, never past where it was last seen; it starts when the
// movement is 8° off and stops within 3° once it is about still, or when nothing
// moves. A safety stop on PicoB or losing PicoB ends it, saying why.
#include <stdbool.h>

void behaviour_scan(void);   // starts the start-up scan (again), then watches; switches the motors on
void behaviour_watch(void);  // starts watching from where the robot faces; switches the motors on
void behaviour_stop(void);   // ends it; the robot stands still, the motors stay as they are
bool behaviour_busy(void);
bool behaviour_watching(void);
void behaviour_update(void); // call every loop iteration
