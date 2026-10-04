// PicoB bring-up: tilt the base about the IMU's X axis to drive one side
// forward. Breakout's +Y end tipped up (tilt X > 0) runs the right wheels,
// tipped down (tilt X < 0) the left wheels; the other side coasts. +-15 deg
// dead zone, then power ramps linearly to 100 % at 90 deg.
// Nothing is printed and the drivers stay in standby until a serial monitor
// is open; closing it stops the motors. Each wheel's revolutions and rpm are
// printed with the sign encoder.c gives them: + must mean forward.
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"

#define DEAD_ZONE_DEG 15.0f
#define FULL_DEG      90.0f
#define FILTER_TAU_S  0.5f  // complementary filter: gyro below this time scale, gravity above
#define RAD_TO_DEG    57.29578f
#define PRINT_US      100000

static float tilt_to_power(float deg) {
    float a = fabsf(deg);
    if (a < DEAD_ZONE_DEG) return 0;
    float p = (a - DEAD_ZONE_DEG) / (FULL_DEG - DEAD_ZONE_DEG);
    return p > 1 ? 1 : p;
}

static void help(void) {
    printf("Commands: g = go (enable motors), s = stop, z = zero revolutions\n");
}

int main(void) {
    stdio_init_all();
    motor_init();
    encoder_init();
    bool imu_ok = imu_init();
    if (imu_ok) imu_calibrate_gyro(400); // ~1 s at 416 Hz: keep the robot still at power-up

    for (;;) {
        motor_enable(false);
        while (!stdio_usb_connected()) sleep_ms(100);
        sleep_ms(200); // let the monitor finish opening before the first line
        printf("\nPicoB tilt -> motor test\n");
        if (!imu_ok) {
            printf("IMU error: WHO_AM_I is not 0x6B. Check the ISM330DHCX wiring, then reset.\n");
            while (stdio_usb_connected()) sleep_ms(100);
            continue;
        }
        printf("Tilt about IMU X axis: +Y up = right wheels, +Y down = left wheels (forward);\n"
               "dead zone +-%.0f deg, full power at +-%.0f deg\n", DEAD_ZONE_DEG, FULL_DEG);
        help();
        printf("Units: tilt deg, accel g, gyro deg/s, L/R power; per wheel (LF LR RF RR = left/right front/rear)\n"
               "output-shaft revolutions / rpm, + = forward\n");
        printf("Motors are STOPPED. Press g to enable.\n");

        bool running = false;
        float tilt_x = 0, tilt_y = 0;
        bool have_tilt = false;
        absolute_time_t last_sample = get_absolute_time();
        absolute_time_t next_print = make_timeout_time_us(PRINT_US);
        int32_t last_count[ENC_COUNT];
        for (int i = 0; i < ENC_COUNT; i++) last_count[i] = encoder_count(i);

        while (stdio_usb_connected()) {
            int c = getchar_timeout_us(0);
            if (c == 'g') { running = true; motor_enable(true); printf("Motors ENABLED\n"); }
            else if (c == 's') { running = false; motor_enable(false); printf("Motors STOPPED\n"); }
            else if (c == 'z') { encoder_reset(); for (int i = 0; i < ENC_COUNT; i++) last_count[i] = 0; printf("Revolutions zeroed\n"); }
            else if (c == 'h' || c == '?') help();

            static imu_sample_t s;
            if (imu_read(&s)) {
                absolute_time_t now = get_absolute_time();
                float dt = absolute_time_diff_us(last_sample, now) * 1e-6f;
                last_sample = now;
                // Gravity direction from the accelerometer (right-hand rotations about X and Y).
                float accel_x = atan2f(s.ay, s.az) * RAD_TO_DEG;
                float accel_y = atan2f(-s.ax, s.az) * RAD_TO_DEG;
                if (have_tilt) {
                    float w = dt / (FILTER_TAU_S + dt);
                    tilt_x = (1 - w) * (tilt_x + s.gx * dt) + w * accel_x;
                    tilt_y = (1 - w) * (tilt_y + s.gy * dt) + w * accel_y;
                } else {
                    tilt_x = accel_x;
                    tilt_y = accel_y;
                }
                have_tilt = true;
            }
            // At rest the accelerometer reads +1 g along whichever axis points up,
            // so ay (and tilt X) is positive when the +Y end is raised.
            float power = running ? tilt_to_power(tilt_x) : 0;
            float left = tilt_x < 0 ? power : 0;
            float right = tilt_x > 0 ? power : 0;
            motor_set(left, right);

            if (time_reached(next_print)) {
                next_print = delayed_by_us(next_print, PRINT_US);
                float revs[ENC_COUNT], rpm[ENC_COUNT];
                for (int i = 0; i < ENC_COUNT; i++) {
                    int32_t count = encoder_count(i);
                    revs[i] = count / ENCODER_COUNTS_PER_REV;
                    rpm[i] = (count - last_count[i]) / ENCODER_COUNTS_PER_REV * (60e6f / PRINT_US);
                    last_count[i] = count;
                }
                printf("tiltX %+6.1f tiltY %+6.1f  ax %+4.1f ay %+4.1f az %+4.1f  gx %+6.1f gy %+6.1f gz %+6.1f  "
                       "L %3.0f%% R %3.0f%%  LF %+5.0f/%+4.0f  LR %+5.0f/%+4.0f  RF %+5.0f/%+4.0f  RR %+5.0f/%+4.0f  %s\n",
                       tilt_x, tilt_y, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                       left * 100, right * 100,
                       revs[ENC_LEFT_FRONT], rpm[ENC_LEFT_FRONT], revs[ENC_LEFT_REAR], rpm[ENC_LEFT_REAR],
                       revs[ENC_RIGHT_FRONT], rpm[ENC_RIGHT_FRONT], revs[ENC_RIGHT_REAR], rpm[ENC_RIGHT_REAR],
                       running ? "RUN" : "STOP");
            }
        }
    }
}
