#pragma once
// Modes (doc/ARCHITECTURE.md, modes): who chooses the robot's actions.
//   Idle:   nobody. The robot stands with the motors off. After a stop.
//   Manual: the operator (a person or an agent), by commands (commands.h). After
//           power-up; a move in idle switches to manual.
// Later modes (sentry, explore...) will choose actions by themselves.
typedef enum { MODE_IDLE, MODE_MANUAL } robot_mode_t;

void mode_set(robot_mode_t mode); // says the change on the console
robot_mode_t mode_get(void);
const char *mode_name(robot_mode_t mode);
