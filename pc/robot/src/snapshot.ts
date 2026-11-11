// Snapshots from the robot (doc/ARCHITECTURE.md, snapshots; picoA/app/snapshot.c): what it
// knows when a command asks (after every move), sent in parts on the bulk lane (link.ts puts them
// together). Each is stored as it arrives, its bytes too (store.ts); its camera
// picture is a PNG in recordings/camera/<boot's folder>/ (boot_folders.ts).
//
// A snapshot: "SNAP", version u8 (3), total length u32, sections [type u8][length u32]
// [payload], CRC-32 of everything before it. Little-endian. The ToF section is a
// compact frame (tof_format.ts), its pose in it.
import zlib from "node:zlib";
import { mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { decodeOpt } from "../../bringup/src/opt4048.ts";
import { END_WORDS } from "./wire.ts";
import { unpackTof, type PosedFrame } from "./tof_format.ts";

export const SNAPSHOT_MAGIC = "SNAP";
export const SNAPSHOT_VERSION = 3;
const SEC = { HEAD: 1, POSE: 2, TOF: 4, CAMERA: 5, COLOR: 6, FEATURES: 9 } as const; // 3 was geometry, 7 window, 8 tiles
// vision_feature_t.flags (picoA/app/vision.h)
export const FEATURE = { NEW: 1, MISSED: 2, LOST: 4, LEFT_IMAGE: 8 } as const;
export const WHY = ["action ended", "command", "server connected"]; // snapshot_why_t; 0 until 14 Oct, 1 was key S
export const MODES = ["idle", "manual"];
export const TILE_COLS = 20; // picoA/app/cell_map.h
// column_t (picoA/app/cell_map.h)
export const COLUMN = { UNKNOWN: 0, BLOCKED: 1, OVERHANG: 2, NO_FLOOR: 3, DRIVABLE: 4, OPEN: 5 } as const;

// Times in a snapshot (robotClockUs) are PicoA's 32-bit clock, as the robot wrote
// them: they wrap every 71.6 min. Order snapshots by sequence or message_number, not by these.
export interface SnapshotPose { robotClockUs: number; x: number; y: number; yaw: number; pitch: number; roll: number; v: number; w: number; stationary: boolean; motorsOn: boolean; stopReason: number }
export interface SnapshotCamera { number: number; lastRowUs: number; lineUs: number; exposureUs: number; gain: number; width: number; height: number; pixels: Buffer }
export interface SnapshotColor { robotClockUs: number; sequence: number; overload: boolean; raw: Buffer; lux: number | null; cie: { x: number; y: number } | null; xyz: { X: number; Y: number; Z: number } | null; channels: number[] }
export interface SnapshotFeature { id: number; age: number; flags: number; x: number; y: number; dx: number; dy: number; strength: number; match: number }
export interface SnapshotFeatures { sequence: number; cameraNumber: number; features: SnapshotFeature[] }
export interface Snapshot {
  version: number; length: number;
  bootId: string; sequence: number; robotClockUs: number; why: string; action: string; endReason: string | null; actionsEnded: number;
  running: string; mode: string; build: string;
  pose: SnapshotPose | null; tof: PosedFrame | null; camera: SnapshotCamera | null; color: SnapshotColor | null;
  features: SnapshotFeatures | null;
}

const text = (b: Buffer) => { const z = b.indexOf(0); return b.subarray(0, z < 0 ? b.length : z).toString("latin1"); };

// The colour reading as the bring-up viewer converts it (pc/bringup/src/opt4048.ts).
export function colorOf(raw: Buffer, overload: boolean): Pick<SnapshotColor, "lux" | "cie" | "xyz" | "channels"> {
  const packet = Buffer.alloc(28);
  packet[0] = 1;
  packet[1] = overload ? 1 : 0;
  packet.writeUInt16LE(0x0821, 2);
  packet.writeUInt16LE(100, 8);
  raw.copy(packet, 12);
  const d = decodeOpt(packet);
  return { lux: d.lux, cie: d.chromaticity, xyz: d.xyz, channels: d.channels.map(c => c.adc) };
}

// Throws if it isn't a whole, undamaged snapshot.
export function parseSnapshot(b: Buffer): Snapshot {
  if (b.length < 13 || b.subarray(0, 4).toString("latin1") !== SNAPSHOT_MAGIC) throw new Error("not a snapshot");
  const length = b.readUInt32LE(5);
  if (length !== b.length) throw new Error(`snapshot: ${b.length} bytes, it says ${length}`);
  if (zlib.crc32(b.subarray(0, length - 4)) !== b.readUInt32LE(length - 4)) throw new Error("snapshot: CRC error");
  if (b[4] !== SNAPSHOT_VERSION && b[4] !== 2) throw new Error(`snapshot version ${b[4]}, this server reads ${SNAPSHOT_VERSION}`);
  const s: Snapshot = {
    version: b[4], length, bootId: "", sequence: 0, robotClockUs: 0, why: "", action: "", endReason: null, actionsEnded: 0, running: "",
    mode: "", build: "", pose: null, tof: null, camera: null, color: null, features: null,
  };
  for (let at = 9; at < length - 4;) {
    const type = b[at], n = b.readUInt32LE(at + 1), p = b.subarray(at + 5, at + 5 + n);
    if (at + 5 + n > length - 4) throw new Error(`snapshot: section ${type} runs past the end`);
    at += 5 + n;
    switch (type) {
      case SEC.HEAD:
        s.bootId = p.readUInt32LE(0).toString(16).padStart(8, "0");
        s.sequence = p.readUInt32LE(4);
        s.robotClockUs = p.readUInt32LE(8);
        s.why = WHY[p[12]] ?? `why ${p[12]}`;
        s.action = text(p.subarray(13, 25));
        s.endReason = p[25] === 255 ? null : END_WORDS[p[25]] ?? `end ${p[25]}`;
        s.actionsEnded = p.readUInt32LE(26);
        s.running = text(p.subarray(30, 42));
        s.mode = MODES[p[42]] ?? `mode ${p[42]}`;
        s.build = text(p.subarray(p.length - 24));
        break;
      case SEC.POSE: {
        const f = (i: number) => p.readFloatLE(4 + 4 * i);
        s.pose = { robotClockUs: p.readUInt32LE(0), x: f(0), y: f(1), yaw: f(2), pitch: f(3), roll: f(4), v: f(5), w: f(6),
                   stationary: p[32] !== 0, motorsOn: p[33] !== 0, stopReason: p[34] };
        break;
      }
      case SEC.TOF:
        s.tof = unpackTof("compact", Buffer.from(p));
        break;
      case SEC.CAMERA: {
        const width = p.readUInt16LE(20), height = p.readUInt16LE(22);
        if (p.length !== 24 + width * height) throw new Error("snapshot: camera section size");
        s.camera = { number: p.readUInt32LE(0), lastRowUs: p.readUInt32LE(4), lineUs: p.readFloatLE(8), exposureUs: p.readFloatLE(12),
                     gain: p.readFloatLE(16), width, height, pixels: Buffer.from(p.subarray(24)) };
        break;
      }
      case SEC.COLOR: {
        const raw = Buffer.from(p.subarray(9, 25)), overload = p[8] !== 0;
        s.color = { robotClockUs: p.readUInt32LE(0), sequence: p.readUInt32LE(4), overload, raw, ...colorOf(raw, overload) };
        break;
      }
      case SEC.FEATURES: {
        const n = p[8], size = 15;
        if (p.length !== 9 + n * size) throw new Error("snapshot: features section size");
        const features: SnapshotFeature[] = [];
        for (let i = 0, q = 9; i < n; i++, q += size)
          features.push({ id: p.readUInt16LE(q), age: p.readUInt16LE(q + 2), flags: p[q + 4], x: p.readInt16LE(q + 5) / 16,
                          y: p.readInt16LE(q + 7) / 16, dx: p.readInt16LE(q + 9) / 64, dy: p.readInt16LE(q + 11) / 64,
                          strength: p[q + 13] / 255, match: p[q + 14] / 255 });
        s.features = { sequence: p.readUInt32LE(0), cameraNumber: p.readUInt32LE(4), features };
        break;
      }
      default: break; // a section from another version
    }
  }
  return s;
}

// Puts the snapshot bytes back together into whole snapshots. Bytes that don't
// start one are skipped (counted) until "SNAP" comes.
export class SnapshotAssembler {
  private buf = Buffer.alloc(0);
  skipped = 0;

  push(bytes: Buffer): Buffer[] {
    this.buf = this.buf.length ? Buffer.concat([this.buf, bytes]) : Buffer.from(bytes);
    const out: Buffer[] = [];
    for (;;) {
      const start = this.buf.indexOf(SNAPSHOT_MAGIC, 0, "latin1");
      const drop = start < 0 ? Math.max(0, this.buf.length - 3) : start;
      if (drop) { this.skipped += drop; this.buf = this.buf.subarray(drop); }
      if (start < 0 || this.buf.length < 9) break;
      const length = this.buf.readUInt32LE(5);
      if (length < 13 || length > 1 << 20) { this.skipped += 4; this.buf = this.buf.subarray(4); continue; }
      if (this.buf.length < length) break;
      out.push(Buffer.from(this.buf.subarray(0, length)));
      this.buf = this.buf.subarray(length);
    }
    return out;
  }
}

// A tile's columns, [ix * 20 + iy] (picoA/app/house_map.h).
export function tileColumns(t: { columns: Buffer }): number[] {
  const out: number[] = [];
  for (let k = 0; k < TILE_COLS * TILE_COLS; k++) out.push(k & 1 ? t.columns[k >> 1] >> 4 : t.columns[k >> 1] & 15);
  return out;
}

// An 8-bit PNG: grey (1 byte a pixel) or RGB (3), rows top first.
function png(width: number, height: number, pixels: Buffer, channels: 1 | 3): Buffer {
  const chunk = (type: string, data: Buffer) => {
    const head = Buffer.alloc(8);
    head.writeUInt32BE(data.length, 0);
    head.write(type, 4, "latin1");
    const crc = Buffer.alloc(4);
    crc.writeUInt32BE(zlib.crc32(Buffer.concat([head.subarray(4), data])) >>> 0, 0);
    return Buffer.concat([head, data, crc]);
  };
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8; // bits
  ihdr[9] = channels === 1 ? 0 : 2; // grey, RGB
  const row = width * channels, rows = Buffer.alloc(height * (row + 1));
  for (let y = 0; y < height; y++) pixels.copy(rows, y * (row + 1) + 1, y * row, (y + 1) * row);
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ihdr),
                        chunk("IDAT", zlib.deflateSync(rows)), chunk("IEND", Buffer.alloc(0))]);
}
export const grayPng = (width: number, height: number, pixels: Buffer) => png(width, height, pixels, 1);
export const rgbPng = (width: number, height: number, pixels: Buffer) => png(width, height, pixels, 3);

export const snapshotId = (bootId: string, sequence: number) => `${bootId}-${sequence}`;
// Under recordings/: the camera pictures as camera/<folder>/<sequence>.png, folder: the
// boot's folder name (boot_folders.ts).
export const cameraPicture = (bootFolder: string, sequence: number) => path.join("camera", bootFolder, `${sequence}.png`);

// The camera frame as camera/<folder>/<sequence>.png under recordings/ (directory), folder: the
// boot's folder name; its path relative to directory, null without a picture.
export function savePicture(directory: string, bootFolder: string, s: Snapshot): string | null {
  if (!s.camera) return null;
  const picture = cameraPicture(bootFolder, s.sequence), file = path.join(directory, picture);
  mkdirSync(path.dirname(file), { recursive: true });
  writeFileSync(file, grayPng(s.camera.width, s.camera.height, s.camera.pixels));
  return picture;
}
