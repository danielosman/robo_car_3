#pragma once
// What the robot decides to do (§7). So far the first need, knowing its
// surroundings: the start-up scan. The robot turns 390° in place (a full turn
// plus 30° of overlap), learning the floor and mapping what's around it, prints
// the map, turns to face the most open direction and prints its heading. Then it
// stands still with the motors off. A safety stop on PicoB or losing PicoB ends
// it, saying why.
#include <stdbool.h>

void behaviour_scan(void);   // starts the start-up scan (again); switches the motors on
void behaviour_stop(void);   // ends it; the robot stands still, the motors stay as they are
bool behaviour_busy(void);
void behaviour_update(void); // call every loop iteration
