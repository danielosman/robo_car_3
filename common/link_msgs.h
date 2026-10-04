#pragma once
// Messages between PicoA (the brain) and PicoB (the body), sent with link.h.
// Bodies are packed little-endian structs in SI units (m, rad, s); both Picos
// build from this one header, so the layout can't drift between them.
// Bump LINK_PROTOCOL_VERSION whenever a message changes: PicoB keeps the motors
// off until it has heard a HELLO carrying its own version.
// Robot frame: x forward, y left, z up. Angles: + = turned left, nose up, left side up.
#include <stdbool.h>
#include <stdint.h>

#define LINK_PROTOCOL_VERSION 4

// Timing both sides rely on.
#define LINK_HELLO_PERIOD_US   1000000 // HELLO repeats until answered
#define LINK_DRIVE_TIMEOUT_US  250000  // PicoB stops the motors without a DRIVE this long
#define LINK_ODOM_PERIOD_US    20000   // PicoB sends ODOM at 50 Hz
#define LINK_STATUS_PERIOD_US  500000  // PicoB sends STATUS at 2 Hz

enum {
    MSG_HELLO = 1, // both ways, hello_msg_t: each side repeats it until answered
    MSG_MOTORS,    // A -> B, motors_msg_t
    MSG_DRIVE,     // A -> B, drive_msg_t, repeated well within LINK_DRIVE_TIMEOUT_US
    MSG_ODOM,      // B -> A, odom_report_t, every LINK_ODOM_PERIOD_US
    MSG_LOG,       // B -> A, text without terminator; PicoA prints it in its debug log
    MSG_STATUS,    // B -> A, status_report_t, every LINK_STATUS_PERIOD_US
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

// How PicoB has moved since it started, measured at t_us on PicoB's clock.
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

// PicoB's internals, for calibration and diagnosis.
typedef struct __attribute__((packed)) {
    float gyro_bias_radps;   // what PicoB currently subtracts from the yaw gyro; re-measured while stationary
} status_report_t;
