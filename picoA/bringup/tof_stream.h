#pragma once
// Bring-up: streams every VL53L8CX frame to pc/bringup over USB (TOF3 packets)
// and takes the viewer's ToF commands. All functions run on the main core,
// between complete USB frame writes.
void tof_stream_init(void);
void tof_stream_poll(void);
void tof_stream_command(const char *line);
