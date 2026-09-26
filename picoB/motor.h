#pragma once
#include <stdbool.h>

void motor_init(void);         // driver in standby, outputs off
void motor_enable(bool on);    // STBY pin; off also zeroes the output
void motor_set(float power);   // -1..+1; 0 = coast
