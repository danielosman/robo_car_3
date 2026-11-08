// PicoB's half of test_imu_path.c: the real odometry and brain on PicoB's own fake
// clock and fake link (this file is its own translation unit, so its statics stay
// apart from PicoA's). The test drives it through the functions below.
#include "../../../picoB/app/odometry.c"
#include "../../../picoB/app/brain.c"
#include "robot_mounting.h"
#include "imu_path.h"

// The wheels, as brain.c uses them: the motors switch at once, nothing faults.
static bool drive_on;
void drive_enable(bool on) { drive_on = on; }
bool drive_enabled(void) { return drive_on; }
void drive_set(float v_mps, float w_radps) { (void)v_mps; (void)w_radps; }
drive_fault_t drive_fault(void) { return DRIVE_OK; }

void b_start(uint64_t now_us) {
    fake_now_us = now_us;
    bool ok = odom_init(&ROBOT_MOUNTING);
    brain_init(ok);
}

void b_sample(uint64_t now_us, const int16_t accel[3], const int16_t gyro[3]) {
    fake_now_us = now_us;
    for (int i = 0; i < 3; i++) { fake_imu_counts.accel[i] = accel[i]; fake_imu_counts.gyro[i] = gyro[i]; }
    fake_imu_fresh = true; // the float sample stays still: 1 g up, no turn
    odom_update();
    brain_update();
}

int b_sent(link_msg_t *out, int max) {
    int n = fake_sent_n < max ? fake_sent_n : max;
    memcpy(out, fake_sent, sizeof(link_msg_t) * (size_t)n);
    fake_sent_n = 0;
    return n;
}

void b_deliver(const link_msg_t *m) { fake_link_deliver(m->type, m->body, m->len); }
