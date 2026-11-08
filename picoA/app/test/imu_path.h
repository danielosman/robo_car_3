#pragma once
// test_imu_path.c's PicoB (test_imu_path_picob.c): its clock is set by each call.
#include <stdint.h>
#include "link.h"

void b_start(uint64_t now_us);
// One IMU sample read at now_us (PicoB's clock) with these raw counts; then brain runs.
void b_sample(uint64_t now_us, const int16_t accel[3], const int16_t gyro[3]);
int b_sent(link_msg_t *out, int max);    // what PicoB sent since the last call
void b_deliver(const link_msg_t *m);     // a message from PicoA
