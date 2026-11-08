// The ToF frame formats: the twin of picoA/app/tof_format.h (the layouts are there).
// A posed frame comes from the robot packed for a use; this unpacks it, and packs it
// again (the fake robot's replay: pack(unpack(bytes)) gives the same bytes). Pure
// functions, no I/O; both sides are checked against the same bytes
// (picoA/app/test/data/wire_golden.hex, its tof_compact line).
//
// compact: header 21 B (frame number u32, time u32 µs, period u16 0.1 ms, x, y i16 mm,
// yaw, pitch, turn i16 0.01°, flags u8: 1 pose known, 2 still), then 64 zones in row
// order (row 1 first, column 1 first; README conventions), 4 B each: distance u8 cm
// (0 nothing, 255 beyond 2.54 m), signal u8, sigma u8 mm, kind (bits 0-1) and targets (bits 2-4).
export const TOF_COMPACT_BYTES = 21 + 64 * 4; // 277
export const TOF_ROWS = 8, TOF_COLS = 8, TOF_ZONES = 64;
export const COMPACT_BEYOND_MM = 2550; // an unpacked distance this far means "beyond 2.54 m"

export interface TofFormat { name: string; messageType: number; size: number }
export const TOF_FORMATS: readonly TofFormat[] = [
  { name: "compact", messageType: 0x40, size: TOF_COMPACT_BYTES },
];

export const tofFormatOfType = (type: number): TofFormat | null => TOF_FORMATS.find(f => f.messageType === type) ?? null;

// tof_kind_t (picoA/app/tof_frame.h): how sure the sensor is of a zone's target.
export const KINDS = ["nothing", "unsure", "less_sure", "sure"] as const;
export type Kind = (typeof KINDS)[number];

// One zone: its target (the first valid one), how sure, how many targets the sensor saw.
export interface Zone { distanceMm: number; signal: number; sigmaMm: number; kind: Kind; targets: number }
// A frame and where the robot was while it was measured (the map frame: m, rad).
// robotClockUs: the middle of the measurement on PicoA's 32-bit clock, as packed; it
// wraps every 71.6 min and is for the robot's own use. The PC times a frame by
// TofMessage.tUs (wire.ts: µs since power-up).
export interface PosedFrame {
  number: number; robotClockUs: number; periodUs: number;
  x: number; y: number; yaw: number; pitch: number; turn: number;
  still: boolean; poseKnown: boolean;
  beyondMm: number;  // a distance this far or farther is only "at least this far"
  zones: Zone[];     // 64, row order (index = (row - 1) * 8 + col - 1)
}

const CENTI_DEG = 18000 / Math.PI; // 0.01° per rad
const FLAG_POSE_KNOWN = 1, FLAG_STILL = 2;

export function unpackTof(format: string, b: Buffer): PosedFrame {
  if (format !== "compact") throw new Error(`no ToF format ${format}`);
  if (b.length !== TOF_COMPACT_BYTES) throw new Error(`a compact ToF frame is ${TOF_COMPACT_BYTES} B, not ${b.length}`);
  const zones: Zone[] = [];
  for (let i = 0, at = 21; i < TOF_ZONES; i++, at += 4)
    zones.push({ distanceMm: b[at] * 10, signal: b[at + 1], sigmaMm: b[at + 2], kind: KINDS[b[at + 3] & 3], targets: (b[at + 3] >> 2) & 7 });
  return {
    number: b.readUInt32LE(0), robotClockUs: b.readUInt32LE(4), periodUs: b.readUInt16LE(8) * 100,
    x: b.readInt16LE(10) / 1000, y: b.readInt16LE(12) / 1000,
    yaw: b.readInt16LE(14) / CENTI_DEG, pitch: b.readInt16LE(16) / CENTI_DEG, turn: b.readInt16LE(18) / CENTI_DEG,
    poseKnown: (b[20] & FLAG_POSE_KNOWN) !== 0, still: (b[20] & FLAG_STILL) !== 0, beyondMm: COMPACT_BEYOND_MM, zones,
  };
}

const i16 = (v: number) => Math.max(-32767, Math.min(32767, Math.floor(v + 0.5)));
const cap8 = (v: number) => Math.min(255, Math.max(0, Math.round(v)));
// −π < a ≤ π.
function wrapped(a: number): number {
  a %= 2 * Math.PI;
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a <= -Math.PI) a += 2 * Math.PI;
  return a;
}
// mm to the distance byte: whole cm, at least 1 for a distance, 255 from 2.545 m on.
const distanceCode = (mm: number) => (mm === 0 ? 0 : Math.max(1, Math.min(255, Math.floor((mm + 5) / 10))));

export function packTof(format: string, f: PosedFrame): Buffer {
  if (format !== "compact") throw new Error(`no ToF format ${format}`);
  const b = Buffer.alloc(TOF_COMPACT_BYTES);
  b.writeUInt32LE(f.number >>> 0, 0);
  b.writeUInt32LE(f.robotClockUs >>> 0, 4);
  b.writeUInt16LE(Math.min(0xffff, Math.floor((f.periodUs + 50) / 100)), 8);
  b.writeInt16LE(i16(f.x * 1000), 10);
  b.writeInt16LE(i16(f.y * 1000), 12);
  const yaw = i16(wrapped(f.yaw) * CENTI_DEG);
  b.writeInt16LE(yaw === -18000 ? 18000 : yaw, 14);
  b.writeInt16LE(i16(f.pitch * CENTI_DEG), 16);
  b.writeInt16LE(i16(f.turn * CENTI_DEG), 18);
  b[20] = (f.poseKnown ? FLAG_POSE_KNOWN : 0) | (f.still ? FLAG_STILL : 0);
  f.zones.forEach((z, i) => {
    const at = 21 + 4 * i, mm = z.kind === "nothing" ? 0 : z.distanceMm;
    b[at] = mm >= f.beyondMm ? 255 : distanceCode(mm);
    b[at + 1] = cap8(z.signal);
    b[at + 2] = cap8(z.sigmaMm);
    b[at + 3] = KINDS.indexOf(z.kind) | (Math.min(7, z.targets) << 2);
  });
  return b;
}

// A zone's row and column (1-8, README conventions) from its index, and back.
export const rowCol = (zone: number): [number, number] => [Math.floor(zone / TOF_COLS) + 1, (zone % TOF_COLS) + 1];
export const zoneAt = (row: number, col: number) => (row - 1) * TOF_COLS + col - 1;
// A valid zone (sure or less sure) whose distance is only a lower bound.
export const isValid = (z: Zone) => z.kind === "sure" || z.kind === "less_sure";
export const isBeyond = (f: Pick<PosedFrame, "beyondMm">, z: Zone) => isValid(z) && z.distanceMm >= f.beyondMm;
