#ifndef HM0360_REGS_H
#define HM0360_REGS_H

#include <stdint.h>
#include <stdbool.h>

/*
 * HM0360 named registers for the Arducam B0319 module. The actual boot table lives
 * in doc/hm0360_init_trim_2.h (160x120, monochrome, 1-bit serial on D0); this
 * header only names the few registers the firmware pokes at runtime.
 *
 * 16-bit register address, 8-bit data. Module self-clocks (no XCLK pin) and has no
 * PWDN / POWER_EN pins (same connector as ../../hm01b0).
 *
 * The earlier hand-written "minimal global-only" init (flat-gray real image, now
 * superseded) is archived in doc/legacy-minimal-init.md.
 */

// --- named registers we reference by name ---
#define HM_REG_MODE_SELECT     0x0100  // 0x00 standby, 0x01 continuous streaming
#define HM_REG_SW_RESET        0x0103  // write 0x01 -> soft reset
#define HM_REG_COMMAND_UPDATE  0x0104  // 0x01 -> latch double-buffered (CMU) regs
#define HM_REG_TEST_PATTERN    0x0601  // [6:4] mode (0=color bar), [0] enable

#define HM0360_MODE_STANDBY    0x00
#define HM0360_MODE_STREAMING  0x01

#define HM0360_TESTPAT_COLORBAR  0x01  // 0x0601: [6:4]=000 color bar, [0]=1 enable
#define HM0360_TESTPAT_OFF       0x00

#endif // HM0360_REGS_H
