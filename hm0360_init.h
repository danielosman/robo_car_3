// hm0360_init_trim_2.h  -- minimal HM0360 boot set for the Arducam B0319.
//
// "trim_2" = the GLOBAL registers (addr < 0x3500) read back AFTER applying
// ArduCAM's full doc/hm0360_init.h verbatim and starting streaming, MINUS every
// register that was already at its power-on/soft-reset default. Apply this after a
// soft reset to reproduce the full init's effective global state: 320x240 (Sub2),
// monochrome, 1-bit serial on D0. Context A/B (0x35xx) are NOT used.
//
// Derivation: diff(doc/dump_afterinit.txt, doc/dump_defaults.txt) -- 195 of the
// init's 323 globals actually change something; the other 128 were already default.
//
// PLUS 4 hand-added GLOBAL registers the diff could NOT find, because ArduCAM only
// ever set them inside Context A (so they were never written as globals):
//   0x3024=0x08 (disable context, globals authoritative), 0x0300=0x03 (8 PCLK/px),
//   0x3030=0x01 (640x480 window), and 0x0380/0x0381=0x01 (Sub2) carried from the
//   dump. HW-VERIFIED: 2560 active PCLK/line, clean 320x240, ~4 fps.
//
// COMMENTS: each line has a short note on the register's function. Descriptions for
// the 0x10xx/0x20xx blocks (BLC/AE/tone/motion-detect) are confident; the 0x30xx
// "analog core / readout sequencer" registers are largely undocumented in the
// public datasheet -- treat those notes as best-effort and the values as load-
// bearing (changing them risks the analog readout / fps).
//
//   [rm?] = I believe this line can be removed with no effect on the working image
//           (AE overwrites it at runtime, or it configures a feature this app never
//           uses). LEFT IN deliberately -- remove only after testing on hardware.
//
// This file is generated+annotated; the firmware boots from it (see arducam_b0319.c).

struct senosr_reg hm0360_320x240_trim_2[] = {
    // --- Analog / mono front-end -------------------------------------------
    {0x0350,0xE0},   // analog control (datasheet-undocumented); load-bearing
    {0x0370,0x01},   // MONO_MODE: monochrome sensor path on
    {0x0372,0x00},   // MONO_MODE_SEL: mono source select

    // --- Black-level calibration (BLC) -------------------------------------
    {0x1000,0x43},   // BLC_CTRL: black-level calibration enable/config
    {0x1001,0x80},   // BLC target / offset
    {0x100A,0x07},   // BLC tuning
    {0x1014,0x01},   // OPFM_CTRL: [3]=0 PCLKO non-gated, [0]=1 VSYNC shift (needed for 1-bit serial)
    {0x101D,0xCF},   // BLC / digital offset tuning
    {0x1021,0x5D},   // BLC / digital offset tuning
    {0x102F,0x08},   // BLC tuning

    // --- Gamma / tone-mapping curve (16 knee points, monotonic) ------------
    {0x1030,0x09},   // gamma LUT point 0
    {0x1031,0x12},   // gamma LUT point 1
    {0x1032,0x23},   // gamma LUT point 2
    {0x1033,0x31},   // gamma LUT point 3
    {0x1034,0x3E},   // gamma LUT point 4
    {0x1035,0x4B},   // gamma LUT point 5
    {0x1036,0x56},   // gamma LUT point 6
    {0x1037,0x5E},   // gamma LUT point 7
    {0x1038,0x65},   // gamma LUT point 8
    {0x1039,0x72},   // gamma LUT point 9
    {0x103A,0x7F},   // gamma LUT point 10
    {0x103B,0x8C},   // gamma LUT point 11
    {0x103C,0x98},   // gamma LUT point 12
    {0x103D,0xB2},   // gamma LUT point 13
    {0x103E,0xCC},   // gamma LUT point 14
    {0x103F,0xE6},   // gamma LUT point 15

    // --- Auto-exposure (AE) ------------------------------------------------
    {0x2000,0x3F},   // AE_CTRL: bit[0] AE en, bit[4] AE_update, bit[7] ALC kept;
                     //   bit[6] FR_ctrl_en CLEARED (was 0x7F) -> auto-framerate OFF
                     //   so the frame_length below HOLDS instead of AE stretching it
                     //   to lengthen exposure in low light (that was a big fps sink).
    {0x202C,0x1F},   // AE max gain / convergence
    {0x2031,0x18},   // [rm?] AE integration-time seed (AE overwrites at runtime)
    {0x2032,0x01},   // [rm?] AE integration-time seed (AE overwrites at runtime)
    {0x2036,0x20},   // AE target / gain config
    {0x2038,0x14},   // AE config
    {0x203C,0x00},   // [rm?] AE runtime state (analog gain, AE-driven)
    {0x203D,0x01},   // [rm?] AE runtime state (digital gain, AE-driven)
    {0x203E,0x00},   // [rm?] AE runtime state (AE-driven)
    {0x203F,0x01},   // [rm?] AE runtime state (AE-driven)

    // --- AE metering windows / zone weighting ------------------------------
    {0x2049,0x10},   // AE window config
    {0x204C,0x08},   // AE window config
    {0x204D,0x20},   // AE window config
    {0x204F,0x38},   // AE window config
    {0x2050,0xE0},   // AE window config
    {0x2052,0x1C},   // AE window config
    {0x2053,0x70},   // AE window config
    {0x2055,0x1A},   // AE window config
    {0x2056,0xC0},   // AE window config
    {0x2058,0x06},   // AE window config
    {0x2059,0xB0},   // AE window config
    {0x2062,0x00},   // AE window config
    {0x2063,0xC8},   // AE window config

    // --- AE convergence / weighting table ----------------------------------
    {0x2080,0x41},   // AE control table
    {0x2081,0xE0},   // AE control table
    {0x208A,0x1C},   // AE control table
    {0x208B,0x30},   // AE control table
    {0x208D,0x02},   // AE control table
    {0x208E,0x08},   // AE control table
    {0x208F,0x0D},   // AE control table
    {0x2090,0x14},   // AE control table
    {0x2091,0x1D},   // AE control table
    {0x2092,0x30},   // AE control table
    {0x2094,0x0A},   // AE control table
    {0x2095,0x0F},   // AE control table
    {0x2096,0x14},   // AE control table
    {0x2097,0x18},   // AE control table
    {0x2098,0x20},   // AE control table
    {0x2099,0x10},   // AE control table
    {0x209A,0x00},   // AE control table
    {0x209F,0x20},   // AE control table
    {0x20A0,0x10},   // AE control table

    // --- Motion detection (this app never reads MD output) -----------------
    {0x2590,0x01},   // [rm?] motion-detect enable -- unused feature
    {0x2800,0x00},   // [rm?] motion-detect control -- unused feature
    {0x2804,0x02},   // [rm?] motion-detect config -- unused feature
    {0x2805,0x03},   // [rm?] motion-detect config -- unused feature
    {0x2806,0x03},   // [rm?] motion-detect config -- unused feature
    {0x2808,0x04},   // [rm?] motion-detect config -- unused feature
    {0x2809,0x0C},   // [rm?] motion-detect config -- unused feature
    {0x280A,0x03},   // [rm?] motion-detect config -- unused feature
    {0x280F,0x03},   // [rm?] motion-detect config -- unused feature
    {0x2810,0x03},   // [rm?] motion-detect config -- unused feature
    {0x2812,0x09},   // [rm?] motion-detect config -- unused feature
    {0x282A,0x0F},   // [rm?] motion-detect config -- unused feature
    {0x282B,0x08},   // [rm?] motion-detect config -- unused feature
    {0x282E,0x2F},   // [rm?] motion-detect config -- unused feature

    // --- Sensor core: PMU / timing / analog readout ------------------------
    {0x301C,0xFF},   // analog core timing
    {0x3024,0x08},   // PMU_CFG_3 [3]=1 CXT DISABLE (datasheet 10.18). Default 0x02 =
                     //   AUTO context enable, which makes the sensor take geometry/
                     //   PLL/AE from the Context A/B regs (0x35xx) -- the ones this
                     //   globals-only table omits. 0x08 forces the GLOBAL registers
                     //   to be authoritative instead.
    {0x3026,0x03},   // PMU / context timing
    {0x3027,0x81},   // PMU / context timing
    {0x302A,0x30},   // analog core timing
    {0x3031,0x01},   // analog core timing
    {0x3035,0x01},   // analog core timing
    {0x3094,0x02},   // analog readout timing
    {0x3095,0x02},   // analog readout timing
    {0x3097,0x02},   // analog readout timing
    {0x3099,0x02},   // analog readout timing
    {0x309E,0x05},   // PCLKO_GATED_EN -- required for 1-bit serial output
    {0x30A2,0x00},   // PCLKO_LINE_FRONT_L (clock-adj porch). Was 0x08 (carried from
                     //   1-bit clk_tb/8); at 4-bit clk_tb/2 that fixed clock offset is
                     //   mis-scaled (~4x more pixels). Zeroed to the spec default as a
                     //   correct-for-4-bit value + H-BIN FIX CANDIDATE. Test H-bin.
    {0x30A4,0x00},   // PCLKO_LINE_END_L (clock-adj porch). Was 0x20 (carried from 1-bit);
                     //   spec default 0x00. Zeroing it didn't move the ACTIVE count, but
                     //   it's the correct-for-4-bit value (the fixed clock offset is
                     //   mis-scaled at clk_tb/2) -- KEPT, not reverted, and part of the
                     //   H-bin fix hunt (test H-bin with both porches zeroed).
    {0x30A5,0x04},   // analog readout timing
    {0x30A6,0x02},   // analog readout timing
    {0x30A7,0x02},   // analog readout timing
    {0x30B0,0x03},   // analog readout timing
    {0x30C4,0x10},   // analog readout timing
    {0x30C5,0x01},   // analog readout timing
    {0x30C6,0xBF},   // analog readout timing
    {0x30CB,0xFF},   // analog readout timing
    {0x30CC,0xFF},   // analog readout timing
    {0x30CD,0x7F},   // analog readout timing
    {0x30CE,0x7F},   // analog readout timing

    // --- Analog readout sequencer / ramp (5-byte repeating groups) ---------
    // Sensor-core column/ramp timing. Undocumented but load-bearing: this block is
    // what engages the real pixel-array readout (and sets the halved PCLK / ~8 fps).
    {0x30D3,0x01},   {0x30D4,0xFF},   {0x30D5,0x00},   {0x30D6,0x40},   {0x30D7,0x00},
    {0x30D8,0xA7},   {0x30D9,0x05},   {0x30DA,0x01},   {0x30DB,0x40},   {0x30DC,0x00},
    {0x30DD,0x27},   {0x30DE,0x05},   {0x30DF,0x07},   {0x30E0,0x40},   {0x30E1,0x00},
    {0x30E2,0x27},   {0x30E3,0x05},   {0x30E4,0x47},   {0x30E5,0x30},   {0x30E6,0x00},
    {0x30E7,0x27},   {0x30E8,0x05},   {0x30E9,0x87},   {0x30EA,0x30},   {0x30EB,0x00},
    {0x30EC,0x27},   {0x30ED,0x05},   {0x30EF,0x40},   {0x30F0,0x00},   {0x30F1,0xA7},
    {0x30F2,0x05},   {0x30F4,0x40},   {0x30F5,0x00},   {0x30F6,0x27},   {0x30F7,0x05},
    {0x30F9,0x40},   {0x30FA,0x00},   {0x30FB,0x27},   {0x30FC,0x05},   {0x30FD,0x47},
    {0x30FE,0x30},   {0x30FF,0x00},   {0x3100,0x27},   {0x3101,0x05},   {0x3102,0x87},
    {0x3103,0x30},   {0x3104,0x00},   {0x3105,0x27},   {0x3106,0x05},

    // --- Output interface / format -----------------------------------------
    {0x310B,0x10},   // output driver / pad control
    {0x3112,0x04},   // OUTPUT bit order/polarity: [3]=0 MSB-first, [2]=1 PCLKO inverted.
                     //   HW-VERIFIED for 4-bit + descending wiring + PIO bit-reverse: 0x0C
                     //   ([3]=1) gave noise with visible structure (nibbles swapped); [3]=0
                     //   (0x04) is the clean combo -- empirical, as with the 1-bit build the
                     //   Fig 6.7 MSB/LSB naming did not predict the working state.

    // --- More analog core --------------------------------------------------
    {0x3113,0xA0},   // analog core
    {0x3114,0x67},   // analog core
    {0x3115,0x42},   // analog core
    {0x3116,0x10},   // analog core
    {0x3117,0x0A},   // analog core
    {0x311C,0x10},   // analog core
    {0x311D,0x06},   // analog core
    {0x311E,0x0F},   // analog core
    {0x311F,0x0E},   // analog core
    {0x3120,0x0D},   // analog core
    {0x3121,0x0F},   // analog core
    {0x3122,0x00},   // analog core
    {0x3126,0x03},   // analog core
    {0x3128,0x57},   // analog core
    {0x312A,0x11},   // analog core
    {0x312B,0x41},   // analog core
    {0x312E,0x00},   // analog core
    {0x3141,0x2A},   // analog core
    {0x3142,0x9F},   // analog core
    {0x3147,0x18},   // analog core
    {0x3149,0x18},   // analog core
    {0x314B,0x01},   // analog core
    {0x3150,0x50},   // analog core
    {0x3152,0x00},   // analog core
    {0x3156,0x2C},   // analog core
    {0x315A,0x0A},   // analog core
    {0x315C,0xE0},   // analog core
    {0x3160,0x1F},   // analog core
    {0x3163,0x1F},   // analog core
    {0x3164,0x7F},   // analog core
    {0x3165,0x7F},   // analog core
    {0x317D,0x02},   // analog core
    {0x318C,0x00},   // analog core

    // --- Explicit GLOBAL geometry (replicates Context A's working values) ---
    // ArduCAM configured geometry inside Context A; with context disabled (0x3024
    // =0x08) we must set the GLOBAL copies to the SAME values the working init used
    // in Context A, or the window/readout-clock are wrong and the frame tiles.
    {0x0300,0x01},   // PLL1CFG (10.4): [1:0]=01 clk_tb /2 -> 2 PCLK per pixel, which
                     //   4-bit serial needs (8-bit pixel = two 4-bit nibbles on D0-D3).
                     //   (1-bit serial used 0x03 = clk_tb /8 = 8 PCLK/px; archived in
                     //   doc/legacy-1bit-serial-init.md.) Halving the timebase here is
                     //   also the suspected fix for the broken H-bin column tap.
    {0x3030,0x01},   // WIN_MODE (10.20) [0]=1 -> 640x480 active window (Context A: 0x350D=0x01)

    // --- Blanking trim (10.5): pure fps win, image-neutral ------------------
    // !!! 4-BIT NOTE: the values below are CARRIED OVER from the 1-bit build and are
    // NOT yet re-tuned for 4-bit. They are in pixel-period units (160 active + blank)
    // so they remain SAFE, but the ~23 fps 1-bit tune was set by the serial-DRAIN
    // shear at 8 PCLK/px; 4-bit drains 4x faster (2 PCLK/px, clk_tb/2) so LINE_LENGTH
    // can likely drop for more fps. Re-trim on HW: geometry probe should now read
    // ~320 active PCLK/line (was 1280). Full 1-bit tuning archived in
    // doc/legacy-1bit-serial-init.md.
    // Defaults are LINE_LENGTH=0x0300 (768 px-clk) and FRAME_LENGTH=0x0214 (532
    // lines), but only 320 px-clk and 240 lines are ACTIVE -> >50% of every line
    // and >290 lines/frame are wasted blanking. fps ~ 1/(LINE_LENGTH * FRAME_LENGTH),
    // so trimming both toward the active size multiplies fps. KEEP a margin above
    // active (sync/porch overhead): watch the on-boot geometry probe -- active must
    // stay 2560 PCLK/line and ~240 clean rows; only the blanking shrinks. If lines
    // shear/clip raise LINE_LENGTH; if the frame tears raise FRAME_LENGTH.
    // To push further (toward the ~12 fps 320x240 ceiling): LINE 0x0180, FRAME 0x0108.
    {0x0342,0x02},   // LINE_LENGTH_PCK_H }= 0x0200 (512 px: 320 active + 192 hblank).
    {0x0343,0x00},   // LINE_LENGTH_PCK_L }  We now capture 320 cols (Sub2, H-binned down
                     //   to 160), so active doubled 160->320 px; LINE_LENGTH bumped
                     //   256->512 to keep comfortable right-edge drain (too tight SHEARS:
                     //   last cols slip into the next row). Master clock pinned 7.62 MHz,
                     //   so this is the fps lever: lower toward 0x01C0(448)/0x01A0(416)
                     //   for more fps until the right edge just starts to shear, back off.
    {0x0340,0x00},   // FRAME_LENGTH_LINES_H }= 0x0090 (144: 120 active + 24 vblank)
    {0x0341,0x90},   // FRAME_LENGTH_LINES_L }  (120 active rows now; raise if frame tears)

    // --- Output format + sub-sample + stream-on (keep this block LAST) ------
    // Boot default: direct 160x120 Sub4, no binning. Other modes are selected
    // in camera.c. H-binning stripes remain under investigation; even-column
    // removal is a workaround, not a verified native binning layout.
    {0x310F,0x40},   // OUTPUT_FMT: [6]=1 4-bit serial enable (D0-D3), [7]=0 1-bit off.
                     //   slew [5:3]/[2:0]=0 (kept from 1-bit). 1-bit used 0x80.
    {0x0380,0x02},   // H_SUB: Sub4 horizontal -> 640->160 columns
    {0x0381,0x02},   // V_SUB: Sub4 vertical -> 480->120 rows
    {0x0382,0x00},   // No binning (boot default)
    {0x0100,0x01},   // MODE_SELECT: 0x01 = start continuous streaming (MUST be last)
    {0xFFFF,0xFF},   // end-of-table marker
};
