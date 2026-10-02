#pragma once
// Fake encoders: the test sets the counts. Reading them before encoder_init() fails
// the test, as the real encoders would silently read 0.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define ENCODER_COUNTS_PER_REV (12.0f * 297.9238f)
typedef enum { ENC_LEFT_FRONT, ENC_RIGHT_FRONT, ENC_COUNT } encoder_id_t;
static int32_t fake_counts[ENC_COUNT];
static bool fake_encoders_started;
static inline void encoder_init(void) { fake_encoders_started = true; }
static inline int32_t encoder_count(encoder_id_t e) { assert(fake_encoders_started); return fake_counts[e]; }
