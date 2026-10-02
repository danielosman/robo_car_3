#pragma once
#include <stdint.h>

#define ENCODER_COUNTS_PER_REV (12.0f * 297.9238f) // 12 CPR x 298:1 gearbox

typedef enum { ENC_LEFT_FRONT, ENC_RIGHT_FRONT, ENC_COUNT } encoder_id_t;

void encoder_init(void);
int32_t encoder_count(encoder_id_t e);
void encoder_reset(void); // zeroes both
