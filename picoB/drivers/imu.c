// ISM330DHCX on PicoB SPI0 (Adafruit breakout, 4-wire SPI, mode 3).
// Register values from ST DS13012 Rev 7, sections 9.11-9.20.
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "imu.h"

#define IMU_SPI  spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define WHO_AM_I   0x0F // fixed 0x6B
#define CTRL1_XL   0x10
#define CTRL2_G    0x11
#define CTRL3_C    0x12
#define CTRL4_C    0x13
#define CTRL9_XL   0x18
#define STATUS_REG 0x1E
#define OUTX_L_G   0x22 // gyro X..Z, then accel X..Z at 0x28

#define ACCEL_G_PER_LSB  0.000061f // +-2 g: 0.061 mg/LSB
#define GYRO_DPS_PER_LSB 0.00875f  // +-250 dps: 8.75 mdps/LSB

static float bias_x, bias_y, bias_z;

static void read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t cmd = reg | 0x80; // bit 7 = read
    gpio_put(PIN_CS, 0);
    spi_write_blocking(IMU_SPI, &cmd, 1);
    spi_read_blocking(IMU_SPI, 0, buf, len);
    gpio_put(PIN_CS, 1);
}
static uint8_t read_reg(uint8_t reg) { uint8_t v; read_regs(reg, &v, 1); return v; }
static void write_reg(uint8_t reg, uint8_t v) {
    uint8_t buf[2] = {reg & 0x7F, v};
    gpio_put(PIN_CS, 0);
    spi_write_blocking(IMU_SPI, buf, 2);
    gpio_put(PIN_CS, 1);
}

bool imu_init(void) {
    spi_init(IMU_SPI, 1000 * 1000); // datasheet max 10 MHz
    spi_set_format(IMU_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    sleep_ms(50); // turn-on time 35 ms

    if (read_reg(WHO_AM_I) != 0x6B) return false;
    write_reg(CTRL3_C, 0x01);                   // SW_RESET
    for (int i = 0; i < 100 && (read_reg(CTRL3_C) & 0x01); i++) sleep_ms(1);
    write_reg(CTRL3_C, 0x44);                   // BDU, IF_INC
    write_reg(CTRL4_C, 0x04);                   // I2C_disable
    write_reg(CTRL9_XL, 0xE2);                  // default 0xE0 + DEVICE_CONF
    write_reg(CTRL1_XL, 0x60);                  // 416 Hz, +-2 g
    write_reg(CTRL2_G, 0x60);                   // 416 Hz, +-250 dps
    sleep_ms(100);                              // let filters settle
    return true;
}

static void read_raw(imu_sample_t *s) {
    uint8_t b[12];
    read_regs(OUTX_L_G, b, sizeof b);
    int16_t v[6];
    for (int i = 0; i < 6; i++) v[i] = (int16_t)(b[2 * i] | b[2 * i + 1] << 8);
    s->gx = v[0] * GYRO_DPS_PER_LSB;
    s->gy = v[1] * GYRO_DPS_PER_LSB;
    s->gz = v[2] * GYRO_DPS_PER_LSB;
    s->ax = v[3] * ACCEL_G_PER_LSB;
    s->ay = v[4] * ACCEL_G_PER_LSB;
    s->az = v[5] * ACCEL_G_PER_LSB;
}

bool imu_read(imu_sample_t *s) {
    if (!(read_reg(STATUS_REG) & 0x02)) return false; // GDA
    read_raw(s);
    s->gx -= bias_x; s->gy -= bias_y; s->gz -= bias_z;
    return true;
}

void imu_calibrate_gyro(int samples) {
    float x = 0, y = 0, z = 0;
    imu_sample_t s;
    for (int n = 0; n < samples; ) {
        if (!(read_reg(STATUS_REG) & 0x02)) continue;
        read_raw(&s);
        x += s.gx; y += s.gy; z += s.gz; n++;
    }
    bias_x = x / samples; bias_y = y / samples; bias_z = z / samples;
}

float imu_gyro_bias_z(void) { return bias_z; }
