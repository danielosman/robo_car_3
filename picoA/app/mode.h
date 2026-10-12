#pragma once
// Modes (doc/ARCHITECTURE.md, modes): who chooses the robot's actions.
//   Idle:   nobody. The robot stands with the motors off. After power-up, and after
//           a stop (space).
//   Manual: the operator, by keys and by goals clicked on the viewer's map. An
//           action key in idle switches to manual; a goal is taken only in manual
//           (a click is easy to make by accident).
// Later modes (sentry, explore...) will choose actions by themselves.
typedef enum { MODE_IDLE, MODE_MANUAL } robot_mode_t;

void mode_set(robot_mode_t mode); // says the change on the console
robot_mode_t mode_get(void);
const char *mode_name(robot_mode_t mode);
