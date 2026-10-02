#pragma once
// Fake motor driver: records what drive.c asks for; using it before motor_init() fails the test.
#include <assert.h>
#include <stdbool.h>
static float fake_left_power, fake_right_power;
static bool fake_standby_off, fake_motors_started;
static inline float fake_clamp(float p) { return p > 1 ? 1 : p < -1 ? -1 : p; }
static inline void motor_init(void) { fake_motors_started = true; }
static inline void motor_enable(bool on) { assert(fake_motors_started); fake_standby_off = on; if (!on) fake_left_power = fake_right_power = 0; }
static inline void motor_set(float l, float r) { assert(fake_motors_started); fake_left_power = fake_clamp(l); fake_right_power = fake_clamp(r); }
