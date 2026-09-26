#pragma once
#include <stdint.h>

#define ENCODER_COUNTS_PER_REV (12.0f * 297.9238f) // 12 CPR x 298:1 gearbox

void encoder_init(void);
int32_t encoder_count(void);
void encoder_reset(void);
