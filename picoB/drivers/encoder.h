#pragma once
#include <stdint.h>

// 7 pulses per channel per motor turn, counted on every edge of both channels
// (28 per motor turn), x 298:1 gearbox (nominal; to check with 10 wheel turns by hand)
#define ENCODER_COUNTS_PER_REV (28.0f * 298.0f)

typedef enum { ENC_LEFT_FRONT, ENC_RIGHT_FRONT, ENC_LEFT_REAR, ENC_RIGHT_REAR, ENC_COUNT } encoder_id_t;

void encoder_init(void);
int32_t encoder_count(encoder_id_t e); // + = that wheel turning forward
void encoder_reset(void); // zeroes all four
