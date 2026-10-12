// Snapshots from the robot (doc/ARCHITECTURE.md, snapshots; picoA/app/snapshot.c): what it
// knows after each action, sent between its console text (robot.ts takes the bytes
// out). Each boot's snapshots are saved as they arrive, never changed, in
// recordings/snapshots/<time>-boot-<id>.snap (the truth); the database rows and
// the PNG pictures are derived from them (store.ts, npm run import).
//
// A snapshot: "SNAP", version u8, total length u32, sections [type u8][length u32]
// [payload], CRC-32 of everything before it. Little-endian.
import zlib from "node:zlib";
import { createWriteStream, mkdirSync, writeFileSync, type WriteStream } from "node:fs";
import path from "node:path";
import { EventEmitter } from "node:events";
import { END_REASONS, parseRecord, REC, type Geometry, type TofRaw } from "./recording.ts";
import { decodeOpt } from "../../bringup/src/opt4048.ts";

export const SNAP_MAGIC = "SNAP";
export const SNAP_FILE_MAGIC = "RCSNAP01"; // a file of snapshots: then [arrival ms f64][snapshot] each
const SEC = { HEAD: 1, POSE: 2, GEOMETRY: 3, TOF: 4, CAMERA: 5, COLOR: 6, WINDOW: 7, TILES: 8, FEATURES: 9 } as const;
// vision_feature_t.flags (picoA/app/vision.h)
export const FEATURE = { NEW: 1, MISSED: 2, LOST: 4, LEFT_IMAGE: 8 } as const;
export const WHY = ["action ended", "key S", "server connected"];
export const MODES = ["idle", "manual"];
export const TILE_COLS = 20; // picoA/app/house_map.h
export const TILE_BYTES = TILE_COLS * TILE_COLS / 2;
// column_t (picoA/app/cell_map.h)
export const COLUMN = { UNKNOWN: 0, BLOCKED: 1, OVERHANG: 2, NO_FLOOR: 3, DRIVABLE: 4, OPEN: 5 } as const;

export interface SnapPose { tUs: number; x: number; y: number; yaw: number; pitch: number; roll: number; v: number; w: number; stationary: boolean; motorsOn: boolean; stopReason: number }
export interface SnapTof { poseOk: boolean; x: number; y: number; yaw: number; pitch: number; raw: TofRaw }
export interface SnapCamera { number: number; lastRowUs: number; lineUs: number; exposureUs: number; gain: number; width: number; height: number; pixels: Buffer }
export interface SnapColor { tUs: number; sequence: number; overload: boolean; raw: Buffer; lux: number | null; cie: { x: number; y: number } | null; xyz: { X: number; Y: number; Z: number } | null; channels: number[] }
export interface SnapWindow { orgX: number; orgY: number; cellsX: number; cellsY: number; layers: number; cells: Buffer }
export interface SnapTile { tx: number; ty: number; columns: Buffer }
export interface SnapFeature { id: number; age: number; flags: number; x: number; y: number; dx: number; dy: number; strength: number; match: number }
export interface SnapFeatures { seq: number; cameraNumber: number; features: SnapFeature[] }
export interface Snapshot {
  version: number; length: number;
  bootId: string; seq: number; tUs: number; why: string; action: string; endReason: string | null; actionsEnded: number;
  running: string; mode: string; mapClears: number; build: string;
  pose: SnapPose | null; geometry: Geometry | null; tof: SnapTof | null; camera: SnapCamera | null; color: SnapColor | null;
  window: SnapWindow | null; tiles: SnapTile[]; features: SnapFeatures | null;
}

const text = (b: Buffer) => { const z = b.indexOf(0); return b.subarray(0, z < 0 ? b.length : z).toString("latin1"); };

// The colour reading as the bring-up viewer converts it (pc/bringup/src/opt4048.ts).
export function colorOf(raw: Buffer, overload: boolean): Pick<SnapColor, "lux" | "cie" | "xyz" | "channels"> {
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
  if (b.length < 13 || b.subarray(0, 4).toString("latin1") !== SNAP_MAGIC) throw new Error("not a snapshot");
  const length = b.readUInt32LE(5);
  if (length !== b.length) throw new Error(`snapshot: ${b.length} bytes, it says ${length}`);
  if (zlib.crc32(b.subarray(0, length - 4)) !== b.readUInt32LE(length - 4)) throw new Error("snapshot: CRC error");
  const s: Snapshot = {
    version: b[4], length, bootId: "", seq: 0, tUs: 0, why: "", action: "", endReason: null, actionsEnded: 0, running: "",
    mode: "", mapClears: 0, build: "", pose: null, geometry: null, tof: null, camera: null, color: null, window: null, tiles: [],
    features: null,
  };
  for (let at = 9; at < length - 4;) {
    const type = b[at], n = b.readUInt32LE(at + 1), p = b.subarray(at + 5, at + 5 + n);
    if (at + 5 + n > length - 4) throw new Error(`snapshot: section ${type} runs past the end`);
    at += 5 + n;
    switch (type) {
      case SEC.HEAD:
        s.bootId = p.readUInt32LE(0).toString(16).padStart(8, "0");
        s.seq = p.readUInt32LE(4);
        s.tUs = p.readUInt32LE(8);
        s.why = WHY[p[12]] ?? `why ${p[12]}`;
        s.action = text(p.subarray(13, 25));
        s.endReason = p[25] === 255 ? null : END_REASONS[p[25]] ?? `reason ${p[25]}`;
        s.actionsEnded = p.readUInt32LE(26);
        s.running = text(p.subarray(30, 42));
        s.mode = MODES[p[42]] ?? `mode ${p[42]}`;
        s.mapClears = p.readUInt32LE(43);
        s.build = text(p.subarray(47, 71));
        break;
      case SEC.POSE: {
        const f = (i: number) => p.readFloatLE(4 + 4 * i);
        s.pose = { tUs: p.readUInt32LE(0), x: f(0), y: f(1), yaw: f(2), pitch: f(3), roll: f(4), v: f(5), w: f(6),
                   stationary: p[32] !== 0, motorsOn: p[33] !== 0, stopReason: p[34] };
        break;
      }
      case SEC.GEOMETRY:
        s.geometry = parseRecord({ type: REC.GEOMETRY, payload: p, crcOk: true }) as Geometry;
        break;
      case SEC.TOF:
        s.tof = { poseOk: p[0] !== 0, x: p.readFloatLE(1), y: p.readFloatLE(5), yaw: p.readFloatLE(9), pitch: p.readFloatLE(13),
                  raw: parseRecord({ type: REC.TOF_RAW, payload: p.subarray(17), crcOk: true }) as TofRaw };
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
        s.color = { tUs: p.readUInt32LE(0), sequence: p.readUInt32LE(4), overload, raw, ...colorOf(raw, overload) };
        break;
      }
      case SEC.WINDOW: {
        const w = { orgX: p.readInt16LE(0), orgY: p.readInt16LE(2), cellsX: p[4], cellsY: p[5], layers: p[6] };
        if (p.length !== 7 + w.cellsX * w.cellsY * w.layers) throw new Error("snapshot: window section size");
        s.window = { ...w, cells: Buffer.from(p.subarray(7)) };
        break;
      }
      case SEC.TILES: {
        const n = p.readUInt16LE(0);
        if (p.length !== 2 + n * (4 + TILE_BYTES)) throw new Error("snapshot: tiles section size");
        for (let i = 0, q = 2; i < n; i++, q += 4 + TILE_BYTES)
          s.tiles.push({ tx: p.readInt16LE(q), ty: p.readInt16LE(q + 2), columns: Buffer.from(p.subarray(q + 4, q + 4 + TILE_BYTES)) });
        break;
      }
      case SEC.FEATURES: {
        const n = p[8], size = 15;
        if (p.length !== 9 + n * size) throw new Error("snapshot: features section size");
        const features: SnapFeature[] = [];
        for (let i = 0, q = 9; i < n; i++, q += size)
          features.push({ id: p.readUInt16LE(q), age: p.readUInt16LE(q + 2), flags: p[q + 4], x: p.readInt16LE(q + 5) / 16,
                          y: p.readInt16LE(q + 7) / 16, dx: p.readInt16LE(q + 9) / 64, dy: p.readInt16LE(q + 11) / 64,
                          strength: p[q + 13] / 255, match: p[q + 14] / 255 });
        s.features = { seq: p.readUInt32LE(0), cameraNumber: p.readUInt32LE(4), features };
        break;
      }
      default: break; // a section from a newer robot
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
      const start = this.buf.indexOf(SNAP_MAGIC, 0, "latin1");
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

// A column's class from the window's three cells (cell_map.c column_of()): the
// cells' state is in their low 2 bits (0 unknown, 1 free, 2 occupied).
export function columnOf(g: number, l1: number, l2: number): number {
  const st = (c: number) => c & 3, OCC = 2, FREE = 1;
  if (st(l1) === OCC) return COLUMN.BLOCKED;
  if (st(l2) === OCC) return COLUMN.OVERHANG;
  if (st(g) === FREE) return COLUMN.NO_FLOOR;
  if (st(l1) !== FREE || st(l2) !== FREE) return COLUMN.UNKNOWN;
  return st(g) === OCC ? COLUMN.DRIVABLE : COLUMN.OPEN;
}

// The window's columns, [ix * cellsY + iy], and the confidence (0-15) of the cell
// that decides each.
export function windowColumns(w: SnapWindow): { columns: number[]; confidence: number[] } {
  const n = w.cellsX * w.cellsY, columns: number[] = [], confidence: number[] = [];
  for (let i = 0; i < n; i++) {
    const g = w.cells[i], l1 = w.cells[n + i], l2 = w.cells[2 * n + i], c = columnOf(g, l1, l2);
    columns.push(c);
    const deciding = c === COLUMN.BLOCKED ? l1 : c === COLUMN.OVERHANG ? l2 : c === COLUMN.NO_FLOOR ? g : Math.max(l1, l2);
    confidence.push(c === COLUMN.UNKNOWN ? 0 : deciding >> 4);
  }
  return { columns, confidence };
}

// A tile's columns, [ix * 20 + iy] (picoA/app/house_map.h).
export function tileColumns(t: SnapTile): number[] {
  const out: number[] = [];
  for (let k = 0; k < TILE_COLS * TILE_COLS; k++) out.push(k & 1 ? t.columns[k >> 1] >> 4 : t.columns[k >> 1] & 15);
  return out;
}

// An 8-bit grey PNG.
export function grayPng(width: number, height: number, pixels: Buffer): Buffer {
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
  ihdr[9] = 0; // grey
  const rows = Buffer.alloc(height * (width + 1));
  for (let y = 0; y < height; y++) pixels.copy(rows, y * (width + 1) + 1, y * width, (y + 1) * width);
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ihdr),
                        chunk("IDAT", zlib.deflateSync(rows)), chunk("IEND", Buffer.alloc(0))]);
}

export const snapId = (bootId: string, seq: number) => `${bootId}-${seq}`;
export const imagePath = (dir: string, bootId: string, seq: number) => path.join(dir, bootId, `${seq}.png`);

// The snapshots in a raw file, with their arrival times; damaged ones are skipped (counted).
export function* readSnapshotFile(buf: Buffer): Generator<{ arrivalMs: number; snapshot: Snapshot | null; error?: string }> {
  if (buf.subarray(0, 8).toString("latin1") !== SNAP_FILE_MAGIC) throw new Error("not a snapshot file");
  for (let at = 8; at + 17 <= buf.length;) {
    const ms = buf.readDoubleLE(at), length = buf.readUInt32LE(at + 8 + 5);
    const bytes = buf.subarray(at + 8, at + 8 + length);
    at += 8 + length;
    try { yield { arrivalMs: ms, snapshot: parseSnapshot(bytes) }; }
    catch (err) { yield { arrivalMs: ms, snapshot: null, error: (err as Error).message }; }
  }
}

// Saves each snapshot as it arrives (one raw file per boot) and its picture, and
// passes it on. Events: "snapshot" (Snapshot, arrivalMs, file), "note" (string).
export class SnapshotSaver extends EventEmitter {
  private dir: string;
  private bootId = "";
  private file: WriteStream | null = null;
  private fileName = "";

  constructor(dir: string) {
    super();
    this.dir = dir;
    mkdirSync(path.join(dir, "raw"), { recursive: true });
  }

  arrived(bytes: Buffer): void {
    let s: Snapshot;
    try { s = parseSnapshot(bytes); } catch (err) { this.emit("note", `a snapshot NOT kept: ${(err as Error).message}`); return; }
    if (s.bootId !== this.bootId) this.newFile(s.bootId);
    const ms = Date.now(), head = Buffer.alloc(8);
    head.writeDoubleLE(ms, 0);
    this.file!.write(Buffer.concat([head, bytes]));
    savePicture(this.dir, s);
    this.emit("snapshot", s, ms, this.fileName);
  }

  stop(): void { this.file?.end(); }

  private newFile(bootId: string): void {
    this.file?.end();
    this.bootId = bootId;
    const stamp = new Date().toLocaleString("sv-SE").replace(/[ :]/g, "-");
    this.fileName = path.join(this.dir, "raw", `${stamp}-boot-${bootId}.snap`);
    this.file = createWriteStream(this.fileName);
    this.file.write(SNAP_FILE_MAGIC);
    this.emit("note", `snapshots from boot ${bootId}: saving to snapshots/raw/${path.basename(this.fileName)}`);
  }
}

// The camera frame as recordings/snapshots/<boot>/<seq>.png.
export function savePicture(dir: string, s: Snapshot): string | null {
  if (!s.camera) return null;
  const file = imagePath(dir, s.bootId, s.seq);
  mkdirSync(path.dirname(file), { recursive: true });
  writeFileSync(file, grayPng(s.camera.width, s.camera.height, s.camera.pixels));
  return file;
}
