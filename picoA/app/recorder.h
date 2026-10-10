#pragma once
// Recording (doc/TELEMETRY_PLAN.md): with recording on, each action is a take, and
// while it runs the raw data goes to the PC over the recording connection
// (wifi_console.h): every ToF frame with all of the sensor's outputs, PicoB's
// odometry reports, the drive commands, and the console's lines and keys as marks.
// Nothing is sent when no action runs. When the connection can't keep up, whole
// ToF frames are left out (counted, and each frame says how many were left out
// before it); the rest is kept. Without the connection the action runs unrecorded.
// The record format is in doc/TELEMETRY_PLAN.md §2.1.
#include <stdbool.h>
#include <stdint.h>
#include "rangefinder.h"

// How a take ended (TAKE_END).
typedef enum {
    REC_END_DONE,          // the action finished
    REC_END_STOPPED,       // stop (space, or printing the map)
    REC_END_REPLACED,      // another action started
    REC_END_SAFETY_STOP,   // PicoB switched the motors off
    REC_END_PICOB_LOST,
    REC_END_NO_START,      // the motors didn't switch on, or the robot didn't stand still
    REC_END_RECORDING_OFF, // R during the take
} rec_end_t;

void recorder_init(void);           // the boot id; starts taking the console's lines
void recorder_set(bool on);         // off ends a running take
bool recorder_on(void);
bool recorder_ready(void);          // on, and the recording connection is up
void recorder_key(char key);        // a key arrived (the next take's key; a mark during a take)
// An action starts (or ends): a take starts (ends) if recording is on. param: scan,
// turn: rad (+ = left); move: m (+ = forward); watch, record: s.
void recorder_take_start(const char *action, float param);
void recorder_take_end(rec_end_t reason);
void recorder_tof(const range_frame_t *frame); // every frame, right after rangefinder_poll()
void recorder_update(void);         // call every loop iteration: odometry, drive commands
void recorder_print_status(void);   // one line for the status key
