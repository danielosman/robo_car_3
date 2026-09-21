#pragma once
// All functions run on the main core, between complete USB frame writes.
void tof_init(void);
void tof_poll(void);
void tof_command(const char *line);
