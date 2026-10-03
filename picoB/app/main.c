// PicoB robot firmware, the body: drives the wheels as PicoA commands over the
// link and reports odometry back (see brain.h for the protocol side and the
// safety stops). Keep the robot still for ~1 s after power-up (gyro bias).
#include "pico/stdlib.h"
#include "odometry.h"
#include "drive.h"
#include "brain.h"

#define CONTROL_PERIOD_US 10000 // wheel control loop

int main(void) {
    stdio_init_all();
    drive_init();
    bool imu_ok = odom_init();
    brain_init(imu_ok);

    absolute_time_t next_control = get_absolute_time();
    for (;;) {
        odom_update();
        if (time_reached(next_control)) {
            next_control = make_timeout_time_us(CONTROL_PERIOD_US);
            drive_update(odom_get());
        }
        brain_update();
    }
}
