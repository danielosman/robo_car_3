// Both TB6612FNG drivers: left (channels A/B = left-front/left-rear) and right
// (right-front/right-rear). The PCB ties AIN/BIN together per driver, so both
// motors on a side always share direction; each has its own PWM pin.
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor.h"

#define PIN_STBY 14 // M_STBY, both drivers
#define PWM_WRAP 7499 // 150 MHz / 7500 = 20 kHz

// The right motors are mounted mirrored, so the same IN1/IN2 polarity turns
// them the other way. Flip a side's sign if "forward" drives it backwards.
#define LEFT_FORWARD   1
#define RIGHT_FORWARD -1

typedef struct {
    uint pwm_front, pwm_rear, in1, in2;
    int forward;
} side_t;

static const side_t left  = {15, 11, 13, 12, LEFT_FORWARD};  // ML_PWMF 7B, ML_PWMB 5B, ML_IN1, ML_IN2
static const side_t right = {22, 28, 27, 26, RIGHT_FORWARD}; // MR_PWMF 3A, MR_PWMB 6A, MR_IN1, MR_IN2

static void out(uint pin, bool v) { gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, v); }

static void pwm_out(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

static void side_init(const side_t *s) {
    out(s->in1, 0);
    out(s->in2, 0);
    pwm_out(s->pwm_front);
    pwm_out(s->pwm_rear);
}

static void side_set(const side_t *s, float power) {
    if (power > 1) power = 1;
    if (power < -1) power = -1;
    power *= s->forward;
    gpio_put(s->in1, power > 0); // IN1 H, IN2 L = CW (driver truth table)
    gpio_put(s->in2, power < 0);
    uint16_t level = (uint16_t)(fabsf(power) * (PWM_WRAP + 1));
    pwm_set_gpio_level(s->pwm_front, level);
    pwm_set_gpio_level(s->pwm_rear, level);
}

void motor_init(void) {
    out(PIN_STBY, 0);
    side_init(&left);
    side_init(&right);
}

void motor_enable(bool on) {
    if (!on) motor_set(0, 0);
    gpio_put(PIN_STBY, on);
}

void motor_set(float left_power, float right_power) {
    side_set(&left, left_power);
    side_set(&right, right_power);
}
