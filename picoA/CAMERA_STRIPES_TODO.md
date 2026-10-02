# HM0360 horizontal-binning stripes — follow-up

## Current status

Camera firmware is in `bringup/camera.c`; initialization is in `drivers/hm0360_init.h`;
PIO capture is in `drivers/hm0360.pio`; the shared camera/ToF browser viewer is under `pc/bringup/`.
The current camera boot default is **160×120 Sub4, no binning**.
VL53L8CX integration is separate in `drivers/tof.c`.

On the custom PCB, **direct Sub4 + H/V binning (160×120) still shows vertical
stripes**. Leave investigation paused for now; use the legacy even-column
workaround if needed.

### What is established, and what is not

The earlier project's recorded tests found:
- With horizontal binning enabled, even-only columns gave a usable image;
  odd-only columns appeared black, apart from a stray stripe.
- The artifact also occurred with horizontal Sub2, not just Sub4.
- Capturing 320×120 and retaining even columns produced a usable 160×120 image.

These observations support the workaround. They **do not prove** that odd columns
are intentional dummy slots, that the sensor is defective, or that the retained
pixels implement the intended averaging kernel. The earlier log's claims that
H-binning was “SOLVED” and intentionally emitted alternate slots were too strong.
Likewise, visually lower noise alone does not establish the averaging kernel.

Reproduction on the custom PCB makes breadboard signal-integrity problems less
likely, but does not rule out interface timing or configuration errors shared by
both implementations.

## **Possible approaches to fixing the stripes**

### **1. Recheck Quad versus Channel readout — priority**

The original V04 datasheet, **§3.3, page 15**, distinguishes **Quad subsampling,
Channel subsampling, and Quad binning**, with Sub2/Sub4 and Bin2/Bin4 diagrams.
The local Markdown summary omits this distinction.

- Inspect Figures 3.3–3.5 visually, including their pixel grouping and ordering.
- Trace the documented controls governing those modes; do not invent a mode bit
  from the terminology alone.
- Check whether our monochrome setup and inherited initialization select a
  readout arrangement consistent with the intended Bin4 mode.
- Do not assume H-bin adds another independent divide-by-two or intentionally
  generates black slots: neither explanation was established from the spec.

### **2. Verify the complete monochrome configuration — priority**

Check actual register readbacks against **§10.6 (page 47)** and **§10.10 (page 49)**:

| Register | Documented function | Current initialization |
|---|---|---|
| `0x0370` | Mono-mode indicator | Writes `0x01` |
| `0x0371` | Mono mode for ISP | Relies on reset default `0x01`; verify |
| `0x0372` | Select mono-mode indicator from OTP | Writes `0x00` |
| `0x100A` | Mono control: bit 1 mono mode; bit 0 reserved, set to 1 | Writes `0x07`; audit reserved bits against the vendor setup |

A matching readback confirms the write, not that the whole configuration is
correct. Compare the complete vendor sequence with our trimmed table before
changing reserved or undocumented bits.

### **3. Audit output packing and capture timing — priority**

**§10.11, page 49:** `0x1014` includes two-pixel-mode options (bits 5 and 4),
clock gating, and H/V sync shifts. Our table writes `0x01`.

- Cross-check these options against **§6.6 / Figure 6.7 (4-bit interface)**.
- Check `0x310F`, `0x3112`, clock dividers, sync shifts and output porches together.
- Do not blindly enable the two-pixel bits; first determine applicability to the
  4-bit interface.
- With a logic analyzer, measure PCLK edges during HREF, byte/nibble alignment,
  line boundaries, and whether alternating output bytes really lack scene data.
- Verify capture completion stays within one frame; a completed DMA transfer alone
  does not prove correct geometry or alignment.

The PCB wiring matches the old project:
`D3→GP7, D2→GP8, D1→GP9, D0→GP10, PCLK→GP11, HREF→GP12, VSYNC→GP13`;
I2C1 is SDA GP14 / SCL GP15. PIO bit reversal plus the DMA FIFO top-byte read
compensates for descending data-pin order. Do not reverse the wiring casually.

### **4. Compare raw scene captures with test patterns**

For each mode below, capture a real scene and internal patterns using identical
capture settings. Start with color bars, then walking-1s/PN9 if useful.

- Direct Sub4 + H/V bin: `0380=02`, `0381=02`, `0382=03`.
- Direct Sub4 + V-bin only: `0380=02`, `0381=02`, `0382=01`.
- Direct Sub4 without binning: `0380=02`, `0381=02`, `0382=00`.
- Raw legacy 320×120: `0380=01`, `0381=02`, `0382=03`.

Inspect even and odd columns separately **before software column removal**.
Disable auto-stretch, leave gamma at 1, and disable inversion for initial
comparisons. Save raw pixels and register settings; evaluate each parity's
range, mean, and response to a moving scene or changing illumination. The current
viewer stretches using the whole frame, not each parity independently, so a
visually black odd-only image is not proof of zero signal.

Clean internal patterns would narrow the issue toward the sensor/readout path,
but would not prove binning itself works: the pattern generator may bypass it.

### **5. Revisit initialization and analog/BLC settings only after the above**

- Compare a known vendor initialization against the trimmed table under the same
  geometry and interface configuration.
- Audit BLC targets/reserved-bit requirements against the actual PDF, not old
  comments that may describe different register revisions or assumptions.
- Earlier pokes to `0x3147`, `0x3149`, `0x3164`, and `0x3165` changed stripe
  brightness/appearance. That did not establish either dead columns or a
  correctable gain/offset mismatch.
- Avoid another broad undocumented-register sweep. Make one controlled change,
  record raw results, and restore the baseline between tests.
- Use reset/reinitialization for clock/configuration experiments where live
  writes may not reproduce boot behavior.

## Working fallback and limits

Viewer mode **“160×120 — legacy even-column workaround”** captures 320×120 and
keeps even columns. Mode **“320×120 — raw legacy capture”** exposes the source
columns for diagnosis. Direct Sub4 modes capture 160×120 without that removal.

The fallback is a practical image-quality workaround, **not a verified native
4×4 average**. Keep that distinction in future code comments and conclusions.
Firmware readback messages appear in the PC host console. After a capture timeout,
select a working mode and press Stream; the host no longer restarts it automatically.

## References to reopen

- Original PDF: `../../hm0360/doc/HM0360-datasheet_v4.pdf`
  - §3.3 / Figures 3.3–3.5: readout grouping, page 15.
  - §6.6 / Figure 6.7: 4-bit output, pages 35–36.
  - §10.6: monochrome controls, page 47.
  - §10.7: subsampling/binning controls, page 48.
  - §10.10–10.11: mono and output-format controls, page 49.
- Summary (not a substitute for the PDF diagrams):
  `../../hm0360/doc/HM0360-datasheet.md`.
- Historical experiments: `../../arducam_b0319/SESSION_LOG.md`, especially
  June 20–21 entries. These contain mutually superseded hypotheses; treat observed
  results separately from the explanations attached to them.

**Next session: start with PDF diagrams + mono/output readbacks, then raw parity
comparisons. No new hardware-root-cause conclusion is established yet.**
