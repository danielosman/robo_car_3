// Left-front Pololu magnetic encoder (J6 pins 1-4). 12 counts per motor-shaft
// revolution counting both edges of both channels, decoded in a GPIO IRQ.
#include "pico/stdlib.h"
#include "encoder.h"

#define PIN_A 9 // ENC_LF_1
#define PIN_B 8 // ENC_LF_2
#define DIRECTION 1 // flip to -1 if counts go negative when the motor runs forward

static volatile int32_t count;
static uint8_t state;
// Index = previous state << 2 | new state; invalid (double) steps count 0.
static const int8_t step[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static uint8_t read_state(void) { return gpio_get(PIN_A) << 1 | gpio_get(PIN_B); }

static void on_edge(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    uint8_t s = read_state();
    count += DIRECTION * step[state << 2 | s];
    state = s;
}

void encoder_init(void) {
    gpio_init(PIN_A);
    gpio_init(PIN_B); // outputs have 10 k pull-ups on the encoder board
    state = read_state();
    gpio_set_irq_enabled_with_callback(PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, on_edge);
    gpio_set_irq_enabled(PIN_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

int32_t encoder_count(void) { return count; }
void encoder_reset(void) { count = 0; }
