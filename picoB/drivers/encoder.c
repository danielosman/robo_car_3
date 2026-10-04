// Hall encoders on all four wheels (GA46-N20E-0043 gearmotors), counted by one
// PIO state machine each (quadrature.pio), so fast wheels cost no CPU time.
// Wiring differs from the PCB labels, matching the motors being on the opposite
// drivers (see motor.c): J6 pins 1-8 are the right wheels (front 1-4 on ENC_LF_*,
// rear 5-8 on ENC_LB_*) and pins 9-16 the left wheels (front 9-12 on ENC_RF_*,
// rear 13-16 on ENC_RB_*).
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "encoder.h"
#include "quadrature.pio.h"

#define ENC_PIO pio0 // all four state machines; the program fills most of its memory

typedef struct {
    uint pin_b;    // channel A is on pin_b + 1 (the PIO program reads both as a pair)
    int direction; // flip if counts go negative when that wheel runs forward
} enc_pins_t;

// Signs: all four checked with the new motors in picoB_bringup (4 Oct). Recheck
// each wheel with picoB_bringup (rpm must be + while the wheel drives forward).
static const enc_pins_t pins[ENC_COUNT] = {
    [ENC_LEFT_FRONT]  = {4,  1}, // J6 9/11:  ENC_RF_1 = A (GP5), ENC_RF_2 = B (GP4)
    [ENC_RIGHT_FRONT] = {8, -1}, // J6 1/3:   ENC_LF_1 = A (GP9), ENC_LF_2 = B (GP8)
    [ENC_LEFT_REAR]   = {2,  1}, // J6 13/15: ENC_RB_1 = A (GP3), ENC_RB_2 = B (GP2)
    [ENC_RIGHT_REAR]  = {6, -1}, // J6 5/7:   ENC_LB_1 = A (GP7), ENC_LB_2 = B (GP6)
};

static uint sm[ENC_COUNT];
static uint32_t zero[ENC_COUNT]; // raw count at the last encoder_reset()

static void start_sm(uint s, uint pin_b) {
    for (uint pin = pin_b; pin <= pin_b + 1; pin++) {
        pio_gpio_init(ENC_PIO, pin);
        gpio_pull_up(pin); // harmless if the encoder board has its own pull-ups
    }
    pio_sm_set_consecutive_pindirs(ENC_PIO, s, pin_b, 2, false);
    pio_sm_config c = quadrature_program_get_default_config(0);
    sm_config_set_in_pins(&c, pin_b);
    sm_config_set_in_shift(&c, false, false, 32);  // shift left, no autopush
    sm_config_set_out_shift(&c, true, false, 32);  // shift right, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(ENC_PIO, s, quadrature_offset_update, &c);
    // Count 0, and the current pin state as the previous one, so starting doesn't count a step.
    pio_sm_exec(ENC_PIO, s, pio_encode_set(pio_y, 0));
    pio_sm_exec(ENC_PIO, s, pio_encode_in(pio_pins, 2));
    pio_sm_exec(ENC_PIO, s, pio_encode_mov(pio_osr, pio_isr));
    pio_sm_set_enabled(ENC_PIO, s, true);
}

// The state machine pushes its count every loop (~50 ns) and drops pushes while
// the FIFO is full, so the FIFO holds old counts: drain it, then wait for a fresh one.
static uint32_t raw_count(encoder_id_t e) {
    uint n = pio_sm_get_rx_fifo_level(ENC_PIO, sm[e]) + 1;
    uint32_t c = 0;
    while (n--) c = pio_sm_get_blocking(ENC_PIO, sm[e]);
    return c;
}

void encoder_init(void) {
    pio_add_program_at_offset(ENC_PIO, &quadrature_program, 0); // the jump table must be at 0
    for (int i = 0; i < ENC_COUNT; i++) {
        sm[i] = (uint)pio_claim_unused_sm(ENC_PIO, true);
        start_sm(sm[i], pins[i].pin_b);
        zero[i] = 0;
    }
}

int32_t encoder_count(encoder_id_t e) { return pins[e].direction * (int32_t)(raw_count(e) - zero[e]); }
void encoder_reset(void) { for (int i = 0; i < ENC_COUNT; i++) zero[i] = raw_count(i); }
