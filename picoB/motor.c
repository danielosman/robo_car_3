// Left TB6612FNG, channels A and B (both left motors). The PCB ties AIN/BIN
// together, so both channels always share direction; each has its own PWM pin.
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor.h"

#define PIN_PWMB 11 // ML_PWMB, slice 5B
#define PIN_IN2  12 // ML_IN2
#define PIN_IN1  13 // ML_IN1
#define PIN_STBY 14 // M_STBY, both drivers
#define PIN_PWMA 15 // ML_PWMF, slice 7B
#define PWM_WRAP 7499 // 150 MHz / 7500 = 20 kHz

static void out(uint pin, bool v) { gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, v); }

void motor_init(void) {
    out(PIN_STBY, 0);
    out(PIN_IN1, 0);
    out(PIN_IN2, 0);
    const uint pins[] = {PIN_PWMA, PIN_PWMB};
    for (int i = 0; i < 2; i++) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_gpio_level(pins[i], 0);
        pwm_set_enabled(slice, true);
    }
}

void motor_enable(bool on) {
    if (!on) motor_set(0);
    gpio_put(PIN_STBY, on);
}

void motor_set(float power) {
    if (power > 1) power = 1;
    if (power < -1) power = -1;
    gpio_put(PIN_IN1, power > 0); // IN1 H, IN2 L = CW (driver truth table)
    gpio_put(PIN_IN2, power < 0);
    uint16_t level = (uint16_t)(fabsf(power) * (PWM_WRAP + 1));
    pwm_set_gpio_level(PIN_PWMA, level);
    pwm_set_gpio_level(PIN_PWMB, level);
}
