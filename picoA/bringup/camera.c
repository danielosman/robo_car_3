// PicoA bring-up: streams the camera, VL53L8CX and OPT4048 to pc/bringup over USB
// (PCB_REVIEW.md, A1 sensors). The camera image is the robot's: the same driver,
// the same 160 x 120 and its own exposure control. MLX90640 and UART unconfigured.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hm0360_regs.h"
#include "camera.h"
#include "tof_stream.h"
#include "opt4048.h"

#define CAMERA_STATE_PERIOD_MS 1000

// The exposure as the viewer shows it (pc/bringup parses this line).
static void camera_state(void) {
    camera_exposure_t e;
    camera_exposure(&e);
    printf("CAMERA_STATE {\"type\":\"cameraSettings\",\"held\":%s,\"settling\":%s,\"exposureMs\":%.1f,"
           "\"gain\":%.2f,\"fps\":%.1f,\"brightness\":%.0f}\n",
           e.held ? "true" : "false", e.settling ? "true" : "false", (double)(e.exposure_us / 1000.0f),
           (double)e.gain, e.frame_us > 0.0f ? (double)(1e6f / e.frame_us) : 0.0, (double)e.brightness);
}

static bool send_frame(void) {
    camera_frame_t f;
    absolute_time_t deadline = make_timeout_time_ms(2000);
    while (!camera_frame(&f)) {
        camera_update();
        if (time_reached(deadline)) {
            printf("Capture timeout: no camera frame in 2 s (%lu so far): check its power and sync wires\n",
                   (unsigned long)camera_frames_captured());
            return false;
        }
    }
    uint8_t header[] = {'H','M','0','3',CAMERA_WIDTH & 255,CAMERA_WIDTH >> 8,CAMERA_HEIGHT,0};
    fwrite(header, 1, sizeof header, stdout);
    fwrite(f.pixels, 1, CAMERA_WIDTH * CAMERA_HEIGHT, stdout);
    fflush(stdout);
    return true;
}

static void read_line(char *buf, unsigned size) {
    unsigned n = 0;
    while (n + 1 < size) {
        int c = getchar_timeout_us(2000000);
        if (c < 0 || c == '\n' || c == '\r') break;
        buf[n++] = c;
    }
    buf[n] = 0;
}

int main(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(500);
    printf("\nRoboCar PicoA HM0360 + VL53L8CX + OPT4048 test\n");
    bool camera_ready = camera_init();
    opt4048_init();
    tof_stream_init(); // Other sensors remain usable even if the camera is absent.
    printf("Ready: S/X/C camera stream/stop/one frame, H hold|release|status exposure, P+byte test pattern,\n"
           "w/r camera register, T ToF, O init OPT4048\n");
    bool streaming = false;
    absolute_time_t next_camera_state = make_timeout_time_ms(CAMERA_STATE_PERIOD_MS);
    while (true) {
        tof_stream_poll();
        opt4048_poll();
        if (camera_ready) {
            camera_update();
            if (time_reached(next_camera_state)) {
                camera_state();
                next_camera_state = make_timeout_time_ms(CAMERA_STATE_PERIOD_MS);
            }
        }
        int c = getchar_timeout_us(streaming ? 0 : 1000);
        switch (c) {
        case 'O': {
            char line[24]; read_line(line, sizeof line);
            if (!strcmp(line, " init")) opt4048_init();
            break;
        }
        case 'H': {
            char line[24]; read_line(line, sizeof line);
            if (!camera_ready) printf("CAMERA unavailable\n");
            else if (!strcmp(line, " hold")) { camera_hold_exposure(true); camera_state(); }
            else if (!strcmp(line, " release")) { camera_hold_exposure(false); camera_state(); }
            else if (!strcmp(line, " status")) camera_state();
            else printf("CAMERA invalid command\n");
            break;
        }
        case 'T': {
            char line[80]; read_line(line, sizeof line); tof_stream_command(line); break;
        }
        case 'S':
            streaming = camera_ready;
            if (!camera_ready) printf("CAMERA unavailable; check power/I2C and reboot\n");
            break;
        case 'X': streaming = false; break;
        case 'C': if (camera_ready && !send_frame()) streaming = false; break;
        case 'P': {
            int p = getchar_timeout_us(100000);
            if (p >= 0 && p <= 5) {
                camera_write_reg(HM_REG_TEST_PATTERN, p ? ((p-1)<<4)|1 : 0);
                camera_write_reg(HM_REG_COMMAND_UPDATE, 1);
            }
            break;
        }
        case 'w': case 'r': {
            char line[40]; unsigned addr, value;
            read_line(line, sizeof line);
            int n = sscanf(line, "%x %x", &addr, &value);
            if (n < 1 || addr > 0xffff || (c == 'w' && (n != 2 || value > 255))) {
                printf("Invalid register command\n"); break;
            }
            if (c == 'w') {
                camera_write_reg(addr, value);
                camera_write_reg(HM_REG_COMMAND_UPDATE, 1);
            }
            uint8_t rb = 0;
            bool ok = camera_read_reg(addr, &rb);
            printf("reg %04x = %02x %s\n", addr, rb, ok ? "OK" : "FAIL");
            break;
        }
        default: break;
        }
        if (streaming && !send_frame()) streaming = false;
    }
}
