#pragma once
// HM0360 camera (Arducam B0319) on PicoA: I2C1 GP14/15, 4-bit pixel data GP7-10,
// PCLK GP11, HREF GP12, VSYNC GP13. Monochrome, 8 bits per pixel, 160 x 120.
// After camera_init() frames are captured continuously in the background (PIO and
// DMA, one interrupt per frame) and come as fast as the exposure allows;
// camera_frame() hands out the newest complete one.
//
// The driver sets the exposure itself (the sensor's auto-exposure is off): whole
// 10 ms steps against the flicker of lights on 50 Hz mains, the shortest one that
// needs at most x4 gain, then more gain; shorter than 10 ms only when even that is
// too bright. camera_hold_exposure() freezes it, so frames stay comparable.
//
// The sensor has a rolling shutter: each row is exposed one line period after the
// row above it; camera_row_time() gives each row's moment.
// Everything runs on the main core except the frame interrupt. Problems are
// printed on stdout.
#include <stdbool.h>
#include <stdint.h>

#define CAMERA_WIDTH 160
#define CAMERA_HEIGHT 120

typedef struct {
    const uint8_t *pixels; // CAMERA_WIDTH x CAMERA_HEIGHT, top row first, as the robot sees it
    uint32_t number;       // counts every frame captured: a gap means frames were skipped
    uint32_t last_row_us;  // time_us_32() when the bottom row was read out
    float line_us;         // time from one row to the next; 0 until measured
    float exposure_us;     // the same for every row
    float gain;            // analog x digital
    bool settling;         // the exposure or gain changed in the last few frames: brightness
                           // not comparable with earlier frames, exposure_us and gain unsure
} camera_frame_t;

typedef struct {
    float exposure_us, gain;
    float frame_us, line_us; // measured; 0 until measured
    float brightness;        // mean of the newest frame, 0-255
    bool held, settling;
} camera_exposure_t;

bool camera_init(void);   // finds the sensor, loads its settings and starts capturing
void camera_update(void); // call every loop iteration: adjusts the exposure
// The newest complete frame, if one arrived since the last call; its pixels stay
// valid until the next call.
bool camera_frame(camera_frame_t *frame);
void camera_hold_exposure(bool hold);
void camera_exposure(camera_exposure_t *exposure);
uint32_t camera_frames_captured(void);

// The middle of a row's exposure, on time_us_32()'s clock.
static inline uint32_t camera_row_time(const camera_frame_t *f, int row) {
    return f->last_row_us - (uint32_t)(f->line_us * (float)(CAMERA_HEIGHT - 1 - row) + f->exposure_us / 2.0f);
}

// Raw register access, for the bring-up firmware's register commands.
bool camera_read_reg(uint16_t reg, uint8_t *value);
bool camera_write_reg(uint16_t reg, uint8_t value);
