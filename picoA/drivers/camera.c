#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hm0360_regs.h"
#include "hm0360.pio.h"
#include "camera.h"

#define HM_I2C i2c1
#define HM_ADDR 0x24
#define SDA_PIN 14
#define SCL_PIN 15
#define DATA_BASE 7 // GP7..10 = D3..D0 (descending); reversed in PIO
#define VSYNC_PIN 13
#define DMA_IRQ_INDEX 1 // DMA_IRQ_1: the WiFi chip's driver waits on its DMA without interrupts

// Registers (HM0360 datasheet V01 §9, §10). Exposure, gains and frame length are
// double-buffered: written, then latched together by COMMAND_UPDATE, they take
// effect at the next frame boundary (N_PLUS_MODE).
#define REG_EXPOSURE     0x0202 // 16 bits, in line periods; at most frame length - 4 (§9.3)
#define REG_ANALOG_GAIN  0x0205 // [6:4] = N: x2^N, N 0-4 (§9.2)
#define REG_DIGITAL_GAIN 0x020E // 0x020E[1:0] and 0x020F[7:2]: 2.6 fixed point, 64 = x1
#define REG_FRAME_LENGTH 0x0340 // 16 bits, lines per frame
#define REG_N_PLUS_MODE  0x3032 // [0] = 1: changes take effect at the next frame (§9.1)

#define MIN_FRAME_LINES 144 // 120 rows + blanking: the shortest frame known to read out cleanly
#define FLICKER_US 10000.0f // lights flicker at 100 Hz on 50 Hz mains (§9.3.1)
#define MAX_STEPS 4         // longest exposure 40 ms: at least 25 frames/s, blur 1.6° at 40°/s
#define PREFERRED_GAIN 4.0f // more gain before a longer exposure only beyond this
#define MAX_GAIN (16.0f * 255.0f / 64.0f)
#define TARGET_BRIGHTNESS 100.0f // of 255: room above for highlights
#define SETTLE_FRAMES 3     // after a change: the frame being read, the one it applies to, one spare
#define MAX_STEP 1.25f      // one adjustment changes exposure x gain by at most this
#define WAIT_MAX_US 5000000 // off target by 25 %: wait this long before adjusting (a shadow passes)...
#define WAIT_MIN_US 500000  // ...far off: at least this long
#define SAMPLE_STEP 4       // brightness from every 4th pixel of every 4th row

struct senosr_reg { uint16_t addr; uint8_t value; };
#include "hm0360_init.h"

static uint8_t buffer[3][CAMERA_WIDTH * CAMERA_HEIGHT];
static PIO pio;
static uint sm, offset;
static int dma = -1;

// The settings the sensor has (or will have from frame settle_until on).
static uint16_t exposure_lines, frame_lines;
static uint8_t analog_code, digital_gain; // digital: 64 = x1
static bool held;
static bool off_target;      // brightness outside the band since off_since_us
static uint32_t off_since_us;
static volatile bool wants_change; // off target for its wait: adjusting, or would be if not held

// Shared with the interrupt. One buffer is being filled, one may hold the newest
// complete frame, one may be held by the caller of camera_frame().
static volatile int capturing, ready = -1, held_buffer = -1;
static volatile uint32_t frames, settle_until, last_done_us;
static volatile float line_us, brightness, saturated; // saturated: share of samples >= 250
static volatile camera_frame_t ready_frame;

bool camera_read_reg(uint16_t reg, uint8_t *value) {
    uint8_t a[] = {reg >> 8, reg & 255};
    return i2c_write_timeout_us(HM_I2C, HM_ADDR, a, 2, true, 10000) == 2 &&
           i2c_read_timeout_us(HM_I2C, HM_ADDR, value, 1, false, 10000) == 1;
}

bool camera_write_reg(uint16_t reg, uint8_t value) {
    uint8_t b[] = {reg >> 8, reg & 255, value};
    bool ok = i2c_write_timeout_us(HM_I2C, HM_ADDR, b, 3, false, 10000) == 3;
    if (!ok) printf("Camera: I2C write failed: %04x\n", reg);
    return ok;
}

static float gain(void) { return (float)(1u << analog_code) * (float)digital_gain / 64.0f; }

// Arms the PIO and DMA for the next frame into buffer[capturing]; the PIO waits
// for VSYNC itself, so this can come at any moment.
static void arm(void) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_put(pio, sm, CAMERA_WIDTH - 1u);
    dma_channel_set_write_addr(dma, buffer[capturing], false);
    dma_channel_set_trans_count(dma, CAMERA_WIDTH * CAMERA_HEIGHT, true);
    pio_sm_set_enabled(pio, sm, true);
}

static void measure_brightness(const uint8_t *pixels) {
    uint32_t sum = 0, bright = 0, n = 0;
    for (int y = SAMPLE_STEP / 2; y < CAMERA_HEIGHT; y += SAMPLE_STEP)
        for (int x = SAMPLE_STEP / 2; x < CAMERA_WIDTH; x += SAMPLE_STEP, n++) {
            uint8_t p = pixels[y * CAMERA_WIDTH + x];
            sum += p;
            bright += p >= 250;
        }
    brightness = (float)sum / (float)n;
    saturated = (float)bright / (float)n;
}

static void frame_done(void) {
    if (!dma_irqn_get_channel_status(DMA_IRQ_INDEX, dma)) return; // shared: another channel's
    dma_irqn_acknowledge_channel(DMA_IRQ_INDEX, dma);
    uint32_t now = time_us_32();
    uint32_t number = ++frames;
    int done = capturing;
    for (int i = 0; i < 3; i++)
        if (i != done && i != held_buffer) { capturing = i; break; }
    arm(); // first, so the next frame isn't missed

    // The line period, from frames all taken with the same frame length (it doesn't
    // depend on the exposure: §9.4). Skips the first frames and gaps.
    if (number > settle_until + 1 && number > 2) {
        float per_line = (float)(now - last_done_us) / (float)frame_lines;
        if (line_us == 0.0f) line_us = per_line;
        else if (per_line < 1.5f * line_us) line_us += 0.1f * (per_line - line_us);
    }
    last_done_us = now;
    measure_brightness(buffer[done]);
    ready = done;
    ready_frame.pixels = buffer[done];
    ready_frame.number = number;
    ready_frame.last_row_us = now;
    ready_frame.line_us = line_us;
    ready_frame.exposure_us = (float)exposure_lines * line_us;
    ready_frame.gain = gain();
    ready_frame.settling = number <= settle_until;
    ready_frame.wants_exposure_change = wants_change;
}

static bool write16(uint16_t reg, uint16_t value) {
    return camera_write_reg(reg, value >> 8) && camera_write_reg(reg + 1, value & 255);
}

// Sends the settings in exposure_lines, analog_code and digital_gain, with a frame
// just long enough for the exposure.
static bool apply_settings(void) {
    frame_lines = exposure_lines + 4 > MIN_FRAME_LINES ? exposure_lines + 4 : MIN_FRAME_LINES;
    bool ok = write16(REG_FRAME_LENGTH, frame_lines) && write16(REG_EXPOSURE, exposure_lines) &&
              camera_write_reg(REG_ANALOG_GAIN, (uint8_t)(analog_code << 4)) &&
              camera_write_reg(REG_DIGITAL_GAIN, digital_gain >> 6) &&
              camera_write_reg(REG_DIGITAL_GAIN + 1, (uint8_t)((digital_gain & 63) << 2)) &&
              camera_write_reg(HM_REG_COMMAND_UPDATE, 1);
    settle_until = frames + SETTLE_FRAMES;
    return ok;
}

// The exposure and gain for an amount of light (exposure x gain, in us).
static void choose_settings(float light_us) {
    float exposure_us, g;
    if (light_us < FLICKER_US) { // brighter than 10 ms at x1: daylight, which doesn't flicker
        exposure_us = light_us;
        g = 1.0f;
    } else {
        int steps = (int)(light_us / (FLICKER_US * PREFERRED_GAIN) + 0.999f);
        if (steps < 1) steps = 1;
        if (steps > MAX_STEPS) steps = MAX_STEPS;
        exposure_us = (float)steps * FLICKER_US;
        g = light_us / exposure_us;
    }
    if (g > MAX_GAIN) g = MAX_GAIN;
    float lines = exposure_us / line_us + 0.5f;
    exposure_lines = lines < 1.0f ? 1 : lines > 65000.0f ? 65000 : (uint16_t)lines;
    analog_code = 0;
    while (analog_code < 4 && (float)(2u << analog_code) <= g) analog_code++;
    float digital = g / (float)(1u << analog_code) * 64.0f + 0.5f;
    digital_gain = digital < 64.0f ? 64 : digital > 255.0f ? 255 : (uint8_t)digital;
}

// Slow, like an auto-exposure: off target, it waits first (5 s when 25 % off, less
// the further off, 0.5 s at least), then steps by at most x1.25 per adjustment, one
// after the other, until on target. Held, it only says it wants to.
void camera_update(void) {
    if (dma < 0 || line_us == 0.0f || frames <= settle_until) return;
    float b = brightness < 1.0f ? 1.0f : brightness;
    float change = TARGET_BRIGHTNESS / b;
    if (saturated > 0.05f && change > 0.7f) change = 0.7f; // large areas burnt out
    if (change > 0.8f && change < 1.25f) {
        off_target = wants_change = false;
        return;
    }
    uint32_t now = time_us_32();
    if (!off_target) { off_target = true; off_since_us = now; }
    float factor = change > 1.0f ? change : 1.0f / change;
    float wait_us = (float)WAIT_MAX_US * (MAX_STEP - 1.0f) / (factor - 1.0f);
    if (wait_us < (float)WAIT_MIN_US) wait_us = (float)WAIT_MIN_US;
    if (!wants_change && (float)(now - off_since_us) < wait_us) return;
    wants_change = true; // from now on it steps until on target
    if (held) return;
    if (change < 1.0f / MAX_STEP) change = 1.0f / MAX_STEP;
    if (change > MAX_STEP) change = MAX_STEP;
    float light = (float)exposure_lines * line_us * gain() * change;
    uint16_t old_lines = exposure_lines;
    uint8_t old_analog = analog_code, old_digital = digital_gain;
    choose_settings(light);
    if (exposure_lines != old_lines || analog_code != old_analog || digital_gain != old_digital)
        apply_settings();
}

static bool setup_capture(void) {
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&hm0360_capture_program, &pio, &sm, &offset,
                                                          DATA_BASE, VSYNC_PIN - DATA_BASE + 1, true)) {
        printf("Camera: no free PIO state machine\n");
        return false;
    }
    for (uint pin = DATA_BASE; pin <= VSYNC_PIN; ++pin) pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, VSYNC_PIN - DATA_BASE + 1, false);
    pio_sm_config c = hm0360_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, DATA_BASE);
    sm_config_set_in_shift(&c, false, false, 8);
    pio_sm_init(pio, sm, offset, &c);
    dma = dma_claim_unused_channel(true);
    dma_channel_config d = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_8);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma, &d, buffer[0], (const volatile uint8_t *)&pio->rxf[sm] + 3, 0, false);
    irq_add_shared_handler(dma_get_irq_num(DMA_IRQ_INDEX), frame_done, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(dma_get_irq_num(DMA_IRQ_INDEX), true);
    dma_irqn_set_channel_enabled(DMA_IRQ_INDEX, dma, true);
    capturing = 0;
    arm();
    return true;
}

bool camera_init(void) {
    i2c_init(HM_I2C, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    uint8_t hi, lo;
    if (!camera_read_reg(0, &hi) || !camera_read_reg(1, &lo) || hi != 3 || lo != 0x60) {
        printf("Camera: no HM0360 (expected model 0360): check its power and I2C1 GP14/15\n");
        return false;
    }
    bool reset = false;
    for (int i = 0; i < 10 && !reset; ++i) {
        if (!camera_write_reg(HM_REG_SW_RESET, 1)) return false;
        sleep_ms(10);
        reset = camera_read_reg(HM_REG_MODE_SELECT, &lo) && lo == HM0360_MODE_STANDBY;
    }
    if (!reset) { printf("Camera: reset failed\n"); return false; }
    for (const struct senosr_reg *r = hm0360_320x240_trim_2; r->addr != 0xffff; ++r) {
        if (r->addr == 0xfffe) sleep_ms(r->value);
        else if (!camera_write_reg(r->addr, r->value)) return false;
    }
    // Until the line period is measured: the longest exposure the shortest frame
    // allows, no gain. camera_update() takes over from there.
    exposure_lines = MIN_FRAME_LINES - 4;
    analog_code = 0;
    digital_gain = 64;
    if (!camera_write_reg(REG_N_PLUS_MODE, 1) || !apply_settings()) return false;
    return setup_capture();
}

bool camera_frame(camera_frame_t *frame) {
    uint32_t irq = save_and_disable_interrupts();
    bool fresh = ready >= 0;
    if (fresh) {
        held_buffer = ready;
        ready = -1;
        *frame = *(const camera_frame_t *)&ready_frame;
    }
    restore_interrupts(irq);
    return fresh;
}

void camera_hold_exposure(bool hold) { held = hold; }

void camera_exposure(camera_exposure_t *e) {
    e->line_us = line_us;
    e->frame_us = line_us * (float)frame_lines;
    e->exposure_us = (float)exposure_lines * line_us;
    e->gain = gain();
    e->brightness = brightness;
    e->held = held;
    e->settling = frames <= settle_until;
    e->wants_change = wants_change;
}

uint32_t camera_frames_captured(void) { return frames; }
