// Front-wheel Pololu magnetic encoders (only the front wheels have them).
// 12 counts per motor-shaft revolution counting both edges of both channels,
// decoded in a GPIO IRQ.
// Wiring differs from the PCB labels: the left-front encoder is on J6 pins
// 9-12 (ENC_RF_*, GP5/GP4) and the right-front one on J6 pins 1-4 (ENC_LF_*,
// GP9/GP8), matching the motors being on the opposite drivers (see motor.c).
#include "pico/stdlib.h"
#include "encoder.h"

typedef struct {
    uint pin_a, pin_b;
    int direction; // flip if counts go negative when that wheel runs forward
} enc_pins_t;

// Same signs as LEFT_FORWARD / RIGHT_FORWARD in motor.c: the encoder turns
// with its motor, so a mirrored side counts the other way.
static const enc_pins_t pins[ENC_COUNT] = {
    [ENC_LEFT_FRONT]  = {5, 4, -1}, // J6 9/11: ENC_RF_1, ENC_RF_2
    [ENC_RIGHT_FRONT] = {9, 8,  1}, // J6 1/3:  ENC_LF_1, ENC_LF_2
};

static volatile int32_t count[ENC_COUNT];
static uint8_t state[ENC_COUNT];
// Index = previous state << 2 | new state; invalid (double) steps count 0.
static const int8_t step[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static uint8_t read_state(const enc_pins_t *p) { return gpio_get(p->pin_a) << 1 | gpio_get(p->pin_b); }

static void on_edge(uint gpio, uint32_t events) {
    (void)events;
    for (int i = 0; i < ENC_COUNT; i++) {
        const enc_pins_t *p = &pins[i];
        if (gpio != p->pin_a && gpio != p->pin_b) continue;
        uint8_t s = read_state(p);
        count[i] += p->direction * step[state[i] << 2 | s];
        state[i] = s;
    }
}

void encoder_init(void) {
    for (int i = 0; i < ENC_COUNT; i++) {
        const enc_pins_t *p = &pins[i];
        gpio_init(p->pin_a);
        gpio_init(p->pin_b); // outputs have 10 k pull-ups on the encoder board
        state[i] = read_state(p);
        gpio_set_irq_enabled_with_callback(p->pin_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, on_edge);
        gpio_set_irq_enabled(p->pin_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }
}

int32_t encoder_count(encoder_id_t e) { return count[e]; }
void encoder_reset(void) { for (int i = 0; i < ENC_COUNT; i++) count[i] = 0; }
