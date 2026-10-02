// PicoA camera + VL53L8CX + OPT4048 test. PCB_REVIEW.md, A1 sensors.
// MLX90640 and UART remain unconfigured.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hm0360_regs.h"
#include "hm0360.pio.h"
#include "tof.h"
#include "opt4048.h"
#include "hm0360_curves.h"

#define HM_I2C i2c1
#define HM_ADDR 0x24
#define SDA_PIN 14
#define SCL_PIN 15
#define DATA_BASE 7  // GP7..10 = D3..D0 (descending); reversed in PIO
#define VSYNC_PIN 13
#define HEIGHT 120
struct senosr_reg { uint16_t addr; uint8_t value; };
#include "hm0360_init.h"
static uint8_t capture[320 * HEIGHT], frame[160 * HEIGHT];
static PIO pio = pio0;
static uint sm, offset;
static int dma;
static unsigned capture_width = 160;
static bool even_columns = false;

static bool read_reg(uint16_t reg, uint8_t *value) {
    uint8_t a[] = {reg >> 8, reg & 255};
    return i2c_write_timeout_us(HM_I2C, HM_ADDR, a, 2, true, 10000) == 2 &&
           i2c_read_timeout_us(HM_I2C, HM_ADDR, value, 1, false, 10000) == 1;
}
static bool write_reg(uint16_t reg, uint8_t value) {
    uint8_t b[] = {reg >> 8, reg & 255, value};
    bool ok = i2c_write_timeout_us(HM_I2C, HM_ADDR, b, 3, false, 10000) == 3;
    if (!ok) printf("I2C write failed: %04x\n", reg);
    return ok;
}

// Mode 0 = legacy workaround; 1 = direct Sub4 + H/V bin experiment;
// 2 = direct Sub4 + V bin baseline; 3 = direct Sub4 without binning;
// 4 = raw legacy 320x120 (inspect even/odd columns in the viewer).
// Subsampling and binning are distinct controls: mode 1 is an experiment,
// not a claim that the sensor performs a verified 16-pixel average.
static bool set_mode(unsigned mode) {
    if (mode > 4) return false;
    unsigned width = (mode == 0 || mode == 4) ? 320 : 160;
    uint8_t bin = mode == 2 ? 1 : mode == 3 ? 0 : 3;
    bool ok = write_reg(0x0100, 0);
    sleep_ms(20);
    ok = write_reg(0x0380, width == 320 ? 1 : 2) && ok;
    ok = write_reg(0x0381, 2) && ok;
    ok = write_reg(0x0382, bin) && ok;
    ok = write_reg(HM_REG_COMMAND_UPDATE, 1) && ok;
    ok = write_reg(0x0100, 1) && ok;
    sleep_ms(50);
    uint8_t h = 255, v = 255, b = 255;
    ok = read_reg(0x0380, &h) && ok;
    ok = read_reg(0x0381, &v) && ok;
    ok = read_reg(0x0382, &b) && ok;
    ok = ok && h == (width == 320 ? 1 : 2) && v == 2 && b == bin;
    if (ok) { capture_width = width; even_columns = mode == 0; }
    printf("MODE %u: H_SUB=%u V_SUB=%u BIN=%u capture=%ux120 %s\n",
           mode, h, v, b, width, ok ? "OK" : "FAILED (stream stopped)");
    return ok;
}

// Exposure is in line periods, not milliseconds: camera clocks differ from
// the old example. Keep the fixed frame length and clamp to FLL - 4 (spec 9.3).
static absolute_time_t next_camera_status;
static bool read16_reg(uint16_t reg, uint16_t *value) {
    uint8_t hi, lo;
    if (!read_reg(reg, &hi) || !read_reg(reg + 1, &lo)) return false;
    *value = ((uint16_t)hi << 8) | lo; return true;
}
static void camera_status(void) {
    uint8_t ae, ag, dh, dl, curve[16]; uint16_t lines, fll;
    if (!read_reg(0x2000, &ae) || !read_reg(0x0205, &ag) ||
        !read_reg(0x020e, &dh) || !read_reg(0x020f, &dl) ||
        !read16_reg(0x0202, &lines) || !read16_reg(0x0340, &fll)) {
        printf("CAMERA settings read failed\n"); return;
    }
    for (unsigned i = 0; i < 16; ++i) if (!read_reg(0x1030 + i, &curve[i])) return;
    int tone = -1;
    for (int i = 0; i < 4; ++i) if (!memcmp(curve, hm_tone_curves[i], 16)) tone = i;
    printf("CAMERA_STATE {\"type\":\"cameraSettings\",\"ae\":%s,\"lines\":%u,\"maxLines\":%u,\"analog\":%u,\"digital\":%u,\"tone\":%d}\n",
        ae & 1 ? "true" : "false", lines, fll > 4 ? fll - 4 : 1,
        (ag >> 4) & 7, ((dh & 3) << 6) | (dl >> 2), tone);
}
static void camera_command(const char *line) {
    while (*line == ' ') ++line;
    uint8_t ae; unsigned lines, analog, digital, tone; char extra;
    bool ok = true;
    if (!strcmp(line, "status")) { camera_status(); return; }
    if (!strcmp(line, "auto")) {
        ok = read_reg(0x2000, &ae) && write_reg(0x2000, ae | 1);
    } else if (sscanf(line, "manual %u %u %u %c", &lines, &analog, &digital, &extra) == 3) {
        if (!lines || lines > 65535 || analog > 4 || digital < 64 || digital > 255) {
            printf("CAMERA invalid manual exposure/gain\n"); return;
        }
        uint16_t fll;
        if (!read16_reg(0x0340, &fll) || fll <= 4 || !read_reg(0x2000, &ae)) {
            printf("CAMERA cannot read exposure limits\n"); return;
        }
        if (lines > fll - 4u) lines = fll - 4u;
        ok = write_reg(0x2000, ae & ~1u) &&
            write_reg(0x0202, lines >> 8) && write_reg(0x0203, lines & 255) &&
            write_reg(0x0205, analog << 4) &&
            write_reg(0x020e, digital >> 6) && write_reg(0x020f, (digital & 63) << 2) &&
            write_reg(HM_REG_COMMAND_UPDATE, 1);
    } else if (sscanf(line, "tone %u %c", &tone, &extra) == 1 && tone < 4) {
        for (unsigned i = 0; i < 16; ++i) {
            if (!write_reg(0x1030 + i, hm_tone_curves[tone][i])) { ok = false; break; }
        }
    } else { printf("CAMERA invalid settings command\n"); return; }
    printf("CAMERA settings %s\n", ok ? "written; readback follows" : "WRITE FAILED (may be partially applied)");
    next_camera_status = make_timeout_time_ms(150); // Allow CMU to latch over subsequent frames.
}

static void setup_capture(void) {
    offset = pio_add_program(pio, &hm0360_capture_program);
    sm = pio_claim_unused_sm(pio, true);
    for (uint pin = DATA_BASE; pin <= VSYNC_PIN; ++pin) pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, 7, false);
    pio_sm_config c = hm0360_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, DATA_BASE);
    sm_config_set_in_shift(&c, false, false, 8);
    pio_sm_init(pio, sm, offset, &c);
    dma = dma_claim_unused_channel(true);
    dma_channel_config d = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_8);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma, &d, capture,
        (const volatile uint8_t *)&pio->rxf[sm] + 3, sizeof capture, false);
}
static bool capture_frame(void) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_put(pio, sm, capture_width - 1);
    // Wait for a fresh VSYNC edge, rather than arming halfway through a frame.
    absolute_time_t deadline = make_timeout_time_ms(2000);
    while (gpio_get(VSYNC_PIN)) if (time_reached(deadline)) goto timeout;
    while (!gpio_get(VSYNC_PIN)) if (time_reached(deadline)) goto timeout;
    dma_channel_set_write_addr(dma, capture, false);
    dma_channel_set_trans_count(dma, capture_width * HEIGHT, false);
    dma_channel_start(dma);
    pio_sm_set_enabled(pio, sm, true);
    while (dma_channel_is_busy(dma)) if (time_reached(deadline)) goto timeout;
    pio_sm_set_enabled(pio, sm, false);
    return true;
timeout:
    pio_sm_set_enabled(pio, sm, false);
    dma_channel_abort(dma);
    printf("Capture timeout: check camera sync/power or select another mode.\n");
    return false;
}
static bool send_frame(void) {
    if (!capture_frame()) return false;
    unsigned width = even_columns ? 160 : capture_width;
    const uint8_t *pixels = capture;
    if (even_columns) {
        for (unsigned i = 0; i < sizeof frame; ++i) frame[i] = capture[2*i];
        pixels = frame;
    }
    uint8_t header[] = {'H','M','0','3',width & 255,width >> 8,HEIGHT,0};
    fwrite(header, 1, sizeof header, stdout);
    fwrite(pixels, 1, width * HEIGHT, stdout);
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
static bool camera_init(void) {
    i2c_init(HM_I2C, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); gpio_pull_up(SCL_PIN);
    uint8_t hi, lo;
    if (!read_reg(0, &hi) || !read_reg(1, &lo) || hi != 3 || lo != 0x60) {
        printf("FATAL: expected MODEL_ID 0360. Check camera power/I2C1 GP14/15.\n");
        return false;
    }
    bool reset = false;
    for (int i = 0; i < 10 && !reset; ++i) {
        if (!write_reg(0x0103, 1)) return false;
        sleep_ms(10);
        reset = read_reg(0x0100, &lo) && lo == 0;
    }
    if (!reset) { printf("CAMERA reset failed\n"); return false; }
    for (const struct senosr_reg *r = hm0360_320x240_trim_2; r->addr != 0xffff; ++r) {
        if (r->addr == 0xfffe) sleep_ms(r->value);
        else if (!write_reg(r->addr, r->value)) return false;
    }
    setup_capture();
    return set_mode(3); // Boot default: native 160x120 Sub4, NO binning.
}
int main(void) {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(500);
    printf("\nRoboCar PicoA HM0360 + VL53L8CX + OPT4048 test\n");
    bool camera_ready = camera_init();
    opt4048_init();
    tof_init(); // Other sensors remain usable even if the camera is absent.
    printf("Ready: S/X/C camera, M+byte mode, H camera settings, T ToF, O init OPT4048\n");
    bool streaming = false;
    while (true) {
        tof_poll();
        opt4048_poll();
        if (camera_ready && time_reached(next_camera_status)) {
            camera_status(); next_camera_status = make_timeout_time_ms(1500);
        }
        int c = getchar_timeout_us(streaming ? 0 : 1000);
        switch (c) {
        case 'O': {
            char line[24]; read_line(line, sizeof line);
            if (!strcmp(line, " init")) opt4048_init();
            break;
        }
        case 'H': {
            char line[80]; read_line(line, sizeof line);
            if (camera_ready) camera_command(line);
            else printf("CAMERA unavailable\n");
            break;
        }
        case 'T': {
            char line[80]; read_line(line, sizeof line); tof_command(line); break;
        }
        case 'S':
            streaming = camera_ready;
            if (!camera_ready) printf("CAMERA unavailable; check power/I2C and reboot\n");
            break;
        case 'X': streaming = false; break;
        case 'C': if (camera_ready && !send_frame()) streaming = false; break;
        case 'M': {
            int mode = getchar_timeout_us(100000);
            if (!camera_ready || mode < 0 || !set_mode((unsigned)mode)) streaming = false;
            break;
        }
        case 'P': {
            int p = getchar_timeout_us(100000);
            if (p >= 0 && p <= 5) {
                write_reg(HM_REG_TEST_PATTERN, p ? ((p-1)<<4)|1 : 0);
                write_reg(HM_REG_COMMAND_UPDATE, 1);
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
                write_reg(addr, value);
                write_reg(HM_REG_COMMAND_UPDATE, 1);
            }
            uint8_t rb = 0;
            bool ok = read_reg(addr, &rb);
            printf("reg %04x = %02x %s\n", addr, rb, ok ? "OK" : "FAIL");
            break;
        }
        default: break;
        }
        if (streaming && !send_frame()) streaming = false;
    }
}
