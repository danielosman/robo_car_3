#pragma once
// Messages between PicoA (the brain) and PicoB (the body), sent with link.h.
// Bodies are packed little-endian structs in SI units (m, rad, s); both Picos
// build from this one header, so the layout can't drift between them.
// Bump LINK_PROTOCOL_VERSION whenever a message changes: PicoB keeps the motors
// off until it has heard a HELLO carrying its own version.
// Robot frame: x forward, y left, z up. Angles: + = turned left, nose up, left side up.
#include <stdbool.h>
#include <stdint.h>

#define LINK_PROTOCOL_VERSION 6

// Timing both sides rely on.
#define LINK_HELLO_PERIOD_US   1000000 // HELLO repeats until answered
#define LINK_DRIVE_TIMEOUT_US  250000  // PicoB stops the motors without a DRIVE this long
#define LINK_ODOM_PERIOD_US    20000   // PicoB sends ODOM at 50 Hz
#define LINK_STATUS_PERIOD_US  500000  // PicoB sends STATUS at 2 Hz
// The IMU measures 416 times a second; PicoB averages each 4 of its samples into one
// imu_record_t (104 a second) and sends each as soon as it is made.
#define IMU_RECORD_SAMPLES     4
#define IMU_RECORD_PERIOD_US   9615    // 4 / 416 Hz, on the sensor's own clock (it may run a little off)

enum {
    MSG_HELLO = 1, // both ways, hello_msg_t: each side repeats it until answered
    MSG_MOTORS,    // A -> B, motors_msg_t
    MSG_DRIVE,     // A -> B, drive_msg_t, repeated well within LINK_DRIVE_TIMEOUT_US
    MSG_ODOM,      // B -> A, odom_report_t, every LINK_ODOM_PERIOD_US
    MSG_LOG,       // B -> A, text without terminator; PicoA prints it in its debug log
    MSG_STATUS,    // B -> A, status_report_t, every LINK_STATUS_PERIOD_US
    MSG_IMU,       // B -> A, imu_record_t, one each IMU_RECORD_PERIOD_US (once PicoA has greeted)
};

typedef struct __attribute__((packed)) {
    uint8_t version;  // LINK_PROTOCOL_VERSION
    bool is_reply;    // answering the other side's HELLO; replies are not answered
} hello_msg_t;

typedef struct __attribute__((packed)) {
    bool on;          // on also clears a safety stop
    uint8_t request;  // numbers each request; PicoB echoes it in odom_report_t.motors_request
} motors_msg_t;

typedef struct __attribute__((packed)) {
    float v_mps;      // forward speed
    float w_radps;    // turn rate, + = left
} drive_msg_t;

// Why PicoB switched the motors off by itself. The motors stay off until PicoA
// switches them on again, which clears the reason. Texts fit in a LOG message.
// The wheel stops: a wheel on that side didn't follow its target (jammed, no encoder
// signal, or turning the wrong way). Tilt: pitch or roll too large (lifted,
// tipping over, climbing something). The limits are in picoB/app/drive.h.
enum { STOP_NONE, STOP_NO_DRIVE, STOP_LEFT_WHEELS, STOP_RIGHT_WHEELS, STOP_TILT };

static inline const char *stop_reason_text(uint8_t reason) {
    switch (reason) {
    case STOP_NONE: return "none";
    case STOP_NO_DRIVE: return "drive commands stopped arriving";
    case STOP_LEFT_WHEELS: return "left wheels not following (jam/encoder)";
    case STOP_RIGHT_WHEELS: return "right wheels not following (jam/encoder)";
    case STOP_TILT: return "tilted too far";
    default: return "unknown";
    }
}

// How PicoB has moved since it started. t_us: when PicoB sent it, on its clock (PicoA
// learns the clocks' offset from it, body.c). The odometry in it is from PicoB's latest
// IMU sample, up to one sample period (2.4 ms) earlier: ~0.1° at a fast turn, ignored.
typedef struct __attribute__((packed)) {
    uint32_t t_us;
    float x_m, y_m;          // x = forward at PicoB's start
    float yaw_rad;           // not wrapped: a full left turn adds 2π
    float v_mps, w_radps;
    float pitch_rad, roll_rad;
    float wheel_left_m, wheel_right_m; // each side's total distance (front and rear averaged), + = forward
    bool stationary;         // wheels and gyro still for 0.5 s
    bool motors_on;
    uint8_t stop_reason;     // STOP_*: why PicoB switched the motors off by itself
    uint8_t motors_request;  // the last MOTORS request PicoB acted on (0 = none since it started)
    bool imu_error;          // no IMU: odometry is frozen and the motors stay off
} odom_report_t;

// The IMU as it measured (doc/DATA_FLOW_PLAN.md, the imu record): IMU_RECORD_SAMPLES
// samples averaged (rounded to the nearest count, halves away from zero), in the
// sensor's own int16 counts and axes, nothing taken out (the gyro's bias neither):
// raw, for the PC to do its own. Scale: accel 0.061 mg a count (±2 g), gyro 8.75 mdps
// a count (±250 °/s) (picoB/drivers/imu.h). The sensor's axes turn into the robot's by
// imu.robot_from_sensor in common/mounting.json. t_us: the middle of the times PicoB
// read its samples (each a small, steady lag after the sensor measured it, with up to
// a loop's jitter), on the sender's clock (PicoB's on the link; PicoA's once body.h
// has turned it). The 4 samples are always consecutive: a skipped one starts a new
// record, so it shows as a gap in the times.
typedef struct __attribute__((packed)) {
    uint32_t t_us;
    int16_t accel[3];        // x, y, z
    int16_t gyro[3];         // x, y, z
} imu_record_t;

// PicoB's internals, for calibration and diagnosis.
typedef struct __attribute__((packed)) {
    float gyro_bias_radps;   // what PicoB currently subtracts from the yaw gyro; re-measured while stationary
    uint32_t imu_lost;       // imu records made but never sent since PicoB started (its queue full: a slow loop)
    uint32_t imu_skipped;    // partial records dropped since PicoB started (a sample skipped: a slow loop)
} status_report_t;
