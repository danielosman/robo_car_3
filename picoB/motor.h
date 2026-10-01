#pragma once
#include <stdbool.h>

void motor_init(void);         // both drivers in standby, outputs off
void motor_enable(bool on);    // STBY pin (both drivers); off also zeroes the outputs
void motor_set(float left, float right); // -1..+1 per side, + = forward; 0 = coast
