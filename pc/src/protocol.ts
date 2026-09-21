// Multiplexed USB CDC stream. Packets are written atomically by the Pico's main
// loop; binary payloads are never scanned for magic or interpreted as log text.
// HM03 | width:u16LE | height:u16LE | grayscale pixels
// TOF3 | length:u32LE | v1 metadata (16 bytes) | zones*(9 + targets*10 bytes)
// OPT3 | length:u32LE (28) | v1 metadata (12 bytes) | raw OPT4048 registers 0x00-0x07
import {decodeOpt, type OptFrame} from './opt4048.ts';
const CAMERA = Buffer.from("HM03");
const TOF = Buffer.from("TOF3");
const OPT = Buffer.from("OPT3");
const MAGICS = [CAMERA, TOF, OPT];
export type FrameHandler = (width: number, height: number, pixels: Buffer) => void;
export type TextHandler = (text: string) => void;

export function decodeTof(b: Buffer) {
  if (b.length < 16 || b[0] !== 1 || ![16, 64].includes(b[1]) || b[2] !== 4 ||
      b.length !== 16 + b[1] * (9 + b[2] * 10)) throw new Error("Invalid ToF packet");
  let p = 16;
  const zones = [];
  for (let z = 0; z < b[1]; z++) {
    const count = b[p++];
    const ambient = b.readUInt32LE(p); p += 4;
    const spads = b.readUInt32LE(p); p += 4;
    const targets = [];
    for (let t = 0; t < b[2]; t++) {
      targets.push({distance: b.readInt16LE(p), sigma: b.readUInt16LE(p + 2),
        signal: b.readUInt32LE(p + 4), reflectance: b[p + 8], status: b[p + 9]});
      p += 10;
    }
    zones.push({count, ambient, spads, targets});
  }
  return {type: "tof", side: b[1] === 64 ? 8 : 4, maxTargets: b[2],
    hz: b[3], temperature: b.readInt8(4), order: b[5], mode: b[6], sharpener: b[7],
    integration: b.readUInt16LE(8), sequence: b.readUInt32LE(12), zones};
}
export type TofFrame = ReturnType<typeof decodeTof>;

export class FrameParser {
  private buf = Buffer.alloc(0);
  private text = "";
  private readonly onFrame: FrameHandler;
  private readonly onText?: TextHandler;
  private readonly onTof?: (frame: TofFrame) => void;
  private readonly onOpt?: (frame: OptFrame) => void;
  constructor(onFrame: FrameHandler, onText?: TextHandler, onTof?: (frame: TofFrame) => void,
              onOpt?: (frame: OptFrame) => void) {
    this.onFrame = onFrame; this.onText = onText; this.onTof = onTof; this.onOpt = onOpt;
  }
  push(chunk: Buffer): void {
    this.buf = Buffer.concat([this.buf, chunk]);
    for (;;) {
      const positions = MAGICS.map(m => this.buf.indexOf(m)).filter(i => i >= 0);
      const i = positions.length ? Math.min(...positions) : -1;
      if (i < 0) {
        // Retain only a suffix that can actually begin a split magic. Keeping
        // an unconditional 3-byte tail delays final stop/error log lines forever.
        let keep = Math.min(3, this.buf.length);
        while (keep > 0 && !MAGICS.some(m =>
          this.buf.subarray(this.buf.length - keep).equals(m.subarray(0, keep)))) keep--;
        this.emitText(this.buf.subarray(0, this.buf.length - keep));
        this.buf = this.buf.subarray(this.buf.length - keep);
        return;
      }
      if (i > 0) { this.emitText(this.buf.subarray(0, i)); this.buf = this.buf.subarray(i); }
      if (this.buf.length < 8) return;
      const isTof = this.buf.subarray(0, 4).equals(TOF);
      const isOpt = this.buf.subarray(0, 4).equals(OPT);
      const w = this.buf.readUInt16LE(4), h = this.buf.readUInt16LE(6);
      const length = isTof || isOpt ? this.buf.readUInt32LE(4) : w * h;
      if (isOpt ? length !== 28 : isTof ? ![800, 3152].includes(length) : (w === 0 || h === 0 || w > 1024 || h > 1024)) {
        this.buf = this.buf.subarray(1); continue;
      }
      if (this.buf.length < 8 + length) return;
      const payload = Buffer.from(this.buf.subarray(8, 8 + length));
      this.buf = this.buf.subarray(8 + length);
      if (isTof) {
        try { const frame = decodeTof(payload); this.onTof?.(frame); }
        catch { this.onText?.("Invalid ToF telemetry packet discarded"); }
      } else if (isOpt) {
        try { this.onOpt?.(decodeOpt(payload)); }
        catch { this.onText?.("Invalid OPT4048 telemetry packet discarded"); }
      } else this.onFrame(w, h, payload);
    }
  }
  private emitText(bytes: Buffer): void {
    this.text += bytes.toString("latin1").replace(/\r/g, "");
    let end: number;
    while ((end = this.text.indexOf("\n")) >= 0) {
      const line = this.text.slice(0, end).trim();
      this.text = this.text.slice(end + 1);
      if (line) this.onText?.(line);
    }
    if (this.text.length > 4096) { this.onText?.(this.text.slice(0, 4096)); this.text = ""; }
  }
}
