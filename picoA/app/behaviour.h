#pragma once
// What the robot decides to do (§7). The start-up scan: the robot turns 390° in
// place (a full turn plus 30° of overlap), learning the floor and mapping what's
// around it, prints the map, turns to face the most open direction and prints its
// heading. Then it watches (§6.6): it stands still, motors on, and when the target
// is leaving the view it turns to where the target will be when the turn ends (if
// it keeps its angular speed; at most 90°), stops and learns the view again. If
// the target is seen leaving the view while it learns, it turns after it again
// with the same angular speed. A safety stop on PicoB or losing PicoB ends it,
// saying why.
#include <stdbool.h>

void behaviour_scan(void);   // starts the start-up scan (again), then watches; switches the motors on
void behaviour_watch(void);  // starts watching from where the robot faces; switches the motors on
void behaviour_stop(void);   // ends it; the robot stands still, the motors stay as they are
bool behaviour_busy(void);
bool behaviour_watching(void);
void behaviour_update(void); // call every loop iteration
