// Recordings from the robot (doc/TELEMETRY_PLAN.md): PicoA sends its records (§2.1)
// as UDP datagrams to port 4212. Each boot's datagrams are saved as they arrive,
// never changed, in recordings/raw/<time>-boot-<id>.rec (each with its arrival
// time), and decoded on the way for a note per take (frames, datagrams lost, the
// rate). Beside it, <same>.arrivals.csv says when the console's text arrived (for
// finding WiFi stalls). Files from before UDP (record versions 1-2, one TCP stream
// per connection) are still read. Storing takes and DuckDB come with T3.
import dgram from "node:dgram";
import zlib from "node:zlib";
import { createWriteStream, mkdirSync, type WriteStream } from "node:fs";
import path from "node:path";
import { EventEmitter } from "node:events";

export const DATA_PORT = 4212;
export const REC = { TAKE_START: 1, GEOMETRY: 2, TOF_RAW: 3, ODOM: 4, DRIVE: 5, MARK: 6, TAKE_END: 7, WIFI: 8 } as const;
// rec_end_t (picoA/app/recorder.h)
export const END_REASONS = ["done", "stopped", "replaced", "safety stop", "PicoB lost", "didn't start", "recording off"];
export const FILE_MAGIC = "RCDGRAM3"; // a file of datagrams: then [length u16][arrival ms f64][datagram] each
const HEAD = 3, CRC = 4, DGRAM_HEAD = 9;

export interface RawRecord { type: number; payload: Buffer; crcOk: boolean }

// [type u8][length u16][payload][CRC-32 of type, length and payload], little-endian.
export function encodeRecord(type: number, payload: Buffer): Buffer {
  const out = Buffer.alloc(HEAD + payload.length + CRC);
  out.writeUInt8(type, 0);
  out.writeUInt16LE(payload.length, 1);
  payload.copy(out, HEAD);
  out.writeUInt32LE(zlib.crc32(out.subarray(0, HEAD + payload.length)), HEAD + payload.length);
  return out;
}

// A datagram: [version u8][boot id u32][number u32], then whole records.
export function encodeDatagram(version: number, bootId: number, seq: number, records: Buffer[]): Buffer {
  const h = Buffer.alloc(DGRAM_HEAD);
  h.writeUInt8(version, 0);
  h.writeUInt32LE(bootId, 1);
  h.writeUInt32LE(seq, 5);
  return Buffer.concat([h, ...records]);
}

// Splits a stream of records into records, wherever the chunks end (TCP files;
// inside a datagram, the records are whole).
export class RecordReader {
  private buf = Buffer.alloc(0);

  push(chunk: Buffer): RawRecord[] {
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : chunk;
    const out: RawRecord[] = [];
    let at = 0;
    while (this.buf.length - at >= HEAD) {
      const len = this.buf.readUInt16LE(at + 1), end = at + HEAD + len + CRC;
      if (this.buf.length < end) break;
      const crcOk = zlib.crc32(this.buf.subarray(at, at + HEAD + len)) === this.buf.readUInt32LE(at + HEAD + len);
      out.push({ type: this.buf[at], payload: Buffer.from(this.buf.subarray(at + HEAD, at + HEAD + len)), crcOk });
      at = end;
    }
    this.buf = this.buf.subarray(at);
    return out;
  }

  get pending(): number { return this.buf.length; } // bytes of a record not complete yet
}

export interface Datagram { version: number; bootId: string; seq: number; records: RawRecord[]; broken: boolean }

export function parseDatagram(d: Buffer): Datagram | null {
  if (d.length < DGRAM_HEAD) return null;
  const reader = new RecordReader();
  const records = reader.push(d.subarray(DGRAM_HEAD));
  return {
    version: d[0], bootId: d.readUInt32LE(1).toString(16).padStart(8, "0"), seq: d.readUInt32LE(5),
    records, broken: reader.pending > 0,
  };
}

export interface TakeStart { kind: "TAKE_START"; version: number; bootId: string; takeNo: number; tUs: number; key: string; action: string; param: number; build: string }
export interface Geometry { kind: "GEOMETRY"; sensorM: number[]; zoneRad: number; rows: number; cols: number; zoneOfRay: number[] }
export interface Target { distanceMm: number; sigmaMm: number; signalPerSpad: number; reflectance: number; status: number }
export interface Zone { zone: number; ambientPerSpad: number; spads: number; targets: Target[] }
export interface TofRaw { kind: "TOF_RAW"; frameNo: number; tUs: number; skipped: number; tempC: number; firstZone: number; zones: Zone[] }
export interface Odom {
  kind: "ODOM"; tUs: number; picoBUs: number; x: number; y: number; yaw: number; v: number; w: number; pitch: number; roll: number;
  wheelLeft: number; wheelRight: number; stationary: boolean; motorsOn: boolean; stopReason: number; motorsRequest: number;
  imuError: boolean; gyroBias: number;
}
export interface Drive { kind: "DRIVE"; tUs: number; v: number; w: number }
export interface Mark { kind: "MARK"; tUs: number; text: string }
export interface TakeEnd {
  kind: "TAKE_END"; tUs: number; reason: string; records: number; recordsDropped: number; frames: number; framesDropped: number;
  bytes: number; datagrams?: number; datagramsFailed?: number;
}
export interface ConnStatus {
  waiting: number; unacked: number; segments: number; retries: number; rtoMs: number; cwnd: number;
  peerWindow: number; writeErrors: number; outputErrors: number;
}
export interface Wifi {
  kind: "WIFI"; tUs: number; rssiDbm: number; consoleSilentMs: number; console: ConnStatus;
  data?: ConnStatus; datagrams?: number; datagramsFailed?: number; // data: version 2 (TCP); datagrams: 3
}
export type Parsed = TakeStart | Geometry | TofRaw | Odom | Drive | Mark | TakeEnd | Wifi | { kind: "UNKNOWN"; type: number };

// rec_conn_t (picoA/app/recorder.c), 29 bytes from `at`.
function connStatus(p: Buffer, at: number): ConnStatus {
  return {
    waiting: p.readUInt32LE(at), unacked: p.readUInt32LE(at + 4), segments: p.readUInt16LE(at + 8), retries: p[at + 10],
    rtoMs: p.readUInt16LE(at + 11), cwnd: p.readUInt32LE(at + 13), peerWindow: p.readUInt32LE(at + 17),
    writeErrors: p.readUInt32LE(at + 21), outputErrors: p.readUInt32LE(at + 25),
  };
}

const text = (b: Buffer) => { const z = b.indexOf(0); return b.subarray(0, z < 0 ? b.length : z).toString("latin1"); };

// ToF zones: version 3 is compact (u16 ambient, SPADs, signal) and holds a range of
// zones; 1-2 hold all 64 with u32s.
function tofRaw(p: Buffer, version: number): TofRaw {
  const v3 = version >= 3;
  const head = v3 ? 13 : 11, firstZone = v3 ? p[11] : 0, n = v3 ? p[12] : 64;
  const zoneSize = v3 ? 5 : 9, targetSize = v3 ? 8 : 10;
  const zones: Zone[] = [];
  let at = head;
  for (let z = firstZone; z < firstZone + n; z++) {
    const targets = p[at];
    const zone: Zone = {
      zone: z, ambientPerSpad: v3 ? p.readUInt16LE(at + 1) : p.readUInt32LE(at + 1),
      spads: v3 ? p.readUInt16LE(at + 3) : p.readUInt32LE(at + 5), targets: [],
    };
    at += zoneSize;
    for (let t = 0; t < targets; t++, at += targetSize) {
      zone.targets.push({
        distanceMm: p.readInt16LE(at), sigmaMm: p.readUInt16LE(at + 2),
        signalPerSpad: v3 ? p.readUInt16LE(at + 4) : p.readUInt32LE(at + 4),
        reflectance: p[at + targetSize - 2], status: p[at + targetSize - 1],
      });
    }
    zones.push(zone);
  }
  if (at !== p.length) throw new Error(`TOF_RAW: ${p.length} bytes, the zones take ${at}`);
  return { kind: "TOF_RAW", frameNo: p.readUInt32LE(0), tUs: p.readUInt32LE(4), skipped: p.readUInt16LE(8), tempC: p.readInt8(10), firstZone, zones };
}

// The payloads, as picoA/app/recorder.c writes them in record version `version`
// (TAKE_START's). Throws on a short payload.
export function parseRecord(r: RawRecord, version = 3): Parsed {
  const p = r.payload;
  switch (r.type) {
    case REC.TAKE_START:
      return {
        kind: "TAKE_START", version: p.readUInt8(0), bootId: p.readUInt32LE(1).toString(16).padStart(8, "0"),
        takeNo: p.readUInt16LE(5), tUs: p.readUInt32LE(7), key: String.fromCharCode(p[11]),
        action: text(p.subarray(12, 24)), param: p.readFloatLE(24), build: text(p.subarray(28, 52)),
      };
    case REC.GEOMETRY:
      return {
        kind: "GEOMETRY", sensorM: [p.readFloatLE(0), p.readFloatLE(4), p.readFloatLE(8)], zoneRad: p.readFloatLE(12),
        rows: p[16], cols: p[17], zoneOfRay: [...p.subarray(18, 18 + p[16] * p[17])],
      };
    case REC.TOF_RAW:
      return tofRaw(p, version);
    case REC.ODOM: {
      const f = (i: number) => p.readFloatLE(8 + 4 * i); // odom_report_t (common/link_msgs.h) from byte 4
      return {
        kind: "ODOM", tUs: p.readUInt32LE(0), picoBUs: p.readUInt32LE(4), x: f(0), y: f(1), yaw: f(2), v: f(3), w: f(4),
        pitch: f(5), roll: f(6), wheelLeft: f(7), wheelRight: f(8), stationary: p[44] !== 0, motorsOn: p[45] !== 0,
        stopReason: p[46], motorsRequest: p[47], imuError: p[48] !== 0, gyroBias: p.readFloatLE(49),
      };
    }
    case REC.DRIVE:
      return { kind: "DRIVE", tUs: p.readUInt32LE(0), v: p.readFloatLE(4), w: p.readFloatLE(8) };
    case REC.MARK:
      return { kind: "MARK", tUs: p.readUInt32LE(0), text: p.subarray(4).toString("utf8") };
    case REC.TAKE_END:
      return {
        kind: "TAKE_END", tUs: p.readUInt32LE(0), reason: END_REASONS[p[4]] ?? `reason ${p[4]}`, records: p.readUInt32LE(5),
        recordsDropped: p.readUInt32LE(9), frames: p.readUInt32LE(13), framesDropped: p.readUInt32LE(17), bytes: p.readUInt32LE(21),
        ...(version >= 3 ? { datagrams: p.readUInt32LE(25), datagramsFailed: p.readUInt32LE(29) } : {}),
      };
    case REC.WIFI:
      return version >= 3
        ? { kind: "WIFI", tUs: p.readUInt32LE(0), rssiDbm: p.readInt8(4), consoleSilentMs: p.readUInt32LE(5), console: connStatus(p, 9), datagrams: p.readUInt32LE(38), datagramsFailed: p.readUInt32LE(42) }
        : { kind: "WIFI", tUs: p.readUInt32LE(0), rssiDbm: p.readInt8(4), consoleSilentMs: p.readUInt32LE(5), console: connStatus(p, 9), data: connStatus(p, 38) };
    default:
      return { kind: "UNKNOWN", type: r.type };
  }
}

const elapsedUs = (from: number, to: number) => (to - from) >>> 0; // PicoA's clock wraps at 2^32 µs

// What one take brought: counted as its records arrive.
export class Take {
  start: TakeStart;
  end: TakeEnd | null = null;
  counts: Record<string, number> = {};
  bytes = 0;
  crcErrors = 0;
  gaps = 0;        // version 1-2: places where ToF frames were left out
  widestGap = 0;   // the most frames left out in one place
  lastTUs: number;
  frameZones = new Map<number, number>(); // frame number -> zones arrived
  seqs = new Set<number>();               // datagrams of the take (version 3)
  firstSeq = -1;
  lastSeq = -1;

  constructor(start: TakeStart, bytes: number) { this.start = start; this.lastTUs = start.tUs; this.add(start, bytes); }

  add(r: Parsed, bytes: number, crcOk = true): void {
    this.counts[r.kind] = (this.counts[r.kind] ?? 0) + 1;
    this.bytes += bytes;
    if (!crcOk) this.crcErrors++;
    if (r.kind === "TOF_RAW") {
      if (r.skipped && this.start.version < 3) { this.gaps++; this.widestGap = Math.max(this.widestGap, r.skipped); }
      this.frameZones.set(r.frameNo, (this.frameZones.get(r.frameNo) ?? 0) + r.zones.length);
    }
    if ("tUs" in r && r.kind !== "TOF_RAW") this.lastTUs = r.tUs;
    if (r.kind === "TAKE_END") this.end = r;
  }

  datagram(seq: number): void {
    if (this.firstSeq < 0) this.firstSeq = seq;
    this.lastSeq = Math.max(this.lastSeq, seq);
    this.seqs.add(seq);
  }

  get name(): string { return `take ${this.start.takeNo} (${this.start.key === " " ? "space" : this.start.key} ${this.start.action}, boot ${this.start.bootId})`; }

  // One line: how it ended, the frames, what was lost, the size and rate.
  summary(): string {
    const s = elapsedUs(this.start.tUs, this.end?.tUs ?? this.lastTUs) / 1e6;
    const rate = (n: number) => (n / Math.max(s, 1e-3)).toFixed(1);
    const parts = [this.end ? this.end.reason : "CUT (no TAKE_END)", `${s.toFixed(1)} s`];
    if (this.start.version >= 3) {
      const nums = [...this.frameZones.keys()];
      const whole = [...this.frameZones.values()].filter(z => z >= 64).length, half = this.frameZones.size - whole;
      const range = nums.length ? Math.max(...nums) - Math.min(...nums) + 1 : 0;
      parts.push(`${whole} ToF frames (${rate(whole)}/s)` + (half ? `, ${half} half` : "") +
        `, ${range - this.frameZones.size} missing`);
      const expected = this.lastSeq - this.firstSeq + 1, lost = expected - this.seqs.size;
      parts.push(`${this.seqs.size} datagrams, ${lost} lost (${(100 * lost / Math.max(expected, 1)).toFixed(1)} %)` +
        (this.end?.datagramsFailed ? `, ${this.end.datagramsFailed} refused on the robot` : ""));
    } else {
      const frames = this.counts.TOF_RAW ?? 0;
      parts.push(`${frames} ToF frames (${rate(frames)}/s)` + (this.end ? `, ${this.end.framesDropped} left out` : "") +
        (this.gaps ? ` in ${this.gaps} gaps, widest ${this.widestGap}` : ""));
      if (this.end?.recordsDropped) parts.push(`${this.end.recordsDropped} other records left out`);
    }
    parts.push(`${this.counts.ODOM ?? 0} ODOM, ${this.counts.DRIVE ?? 0} DRIVE, ${this.counts.MARK ?? 0} MARK`);
    if (this.crcErrors) parts.push(`${this.crcErrors} CRC ERRORS`);
    parts.push(`${(this.bytes / 1024).toFixed(0)} KB, ${(this.bytes / 1024 / Math.max(s, 1e-3)).toFixed(1)} KB/s`);
    return `${this.name}: ${parts.join("; ")}`;
  }
}

// Follows the takes in a stream of records: calls back when one starts and when it
// ends (or is cut: a new TAKE_START without TAKE_END, or the stream ends). An ODOM
// already seen in the take (the robot repeats each once) is left out.
export class TakeTracker {
  current: Take | null = null;
  takes: Take[] = [];
  outside = 0; // records outside any take (shouldn't happen)
  onStart: (t: Take) => void = () => {};
  onEnd: (t: Take) => void = () => {};
  private odoms = new Set<number>();

  // Returns the record, or null for a repeat.
  add(raw: RawRecord, seq?: number): Parsed | null {
    const bytes = HEAD + raw.payload.length + CRC;
    let r: Parsed;
    try { r = parseRecord(raw, raw.type === REC.TAKE_START ? 3 : this.current?.start.version ?? 3); } catch { r = { kind: "UNKNOWN", type: raw.type }; }
    if (r.kind === "TAKE_START") {
      this.finish();
      this.current = new Take(r, bytes);
      this.odoms.clear();
      this.takes.push(this.current);
      if (seq !== undefined) this.current.datagram(seq);
      this.onStart(this.current);
      return r;
    }
    if (r.kind === "ODOM") { // also after TAKE_END: its datagram repeats the ones before
      if (this.odoms.has(r.picoBUs)) return null;
      this.odoms.add(r.picoBUs);
    }
    if (!this.current) { this.outside++; return r; }
    if (seq !== undefined) this.current.datagram(seq);
    this.current.add(r, bytes, raw.crcOk);
    if (r.kind === "TAKE_END") this.finish();
    return r;
  }

  finish(): void {
    if (this.current) this.onEnd(this.current);
    this.current = null;
  }
}

// Leaves out a datagram already seen (the robot sends a take's first and last ones
// more than once).
export class Dedup {
  private seen = new Set<string>();
  fresh(d: Datagram): boolean {
    const key = `${d.bootId}:${d.seq}`;
    if (this.seen.has(key)) return false;
    this.seen.add(key);
    if (this.seen.size > 100000) this.seen.clear();
    return true;
  }
}

// What a recording file holds, in order: datagrams (with their arrival time) or,
// for files from before UDP, one stream of records.
export function* readRecordingFile(buf: Buffer): Generator<{ arrivalMs: number | null; datagram: Datagram | null; records: RawRecord[] }> {
  if (buf.subarray(0, 8).toString("latin1") !== FILE_MAGIC) {
    const reader = new RecordReader();
    yield { arrivalMs: null, datagram: null, records: reader.push(buf) };
    if (reader.pending) throw new Error(`${reader.pending} bytes at the end: a record cut off`);
    return;
  }
  for (let at = 8; at + 10 <= buf.length;) {
    const len = buf.readUInt16LE(at), ms = buf.readDoubleLE(at + 2);
    const d = parseDatagram(buf.subarray(at + 10, at + 10 + len));
    at += 10 + len;
    yield { arrivalMs: ms, datagram: d, records: d?.records ?? [] };
  }
}

export interface RecordingServerOptions { port?: number; dir: string }

// Listens for PicoA's datagrams; a file per robot boot. Events: "note" (string).
export class RecordingServer extends EventEmitter {
  private socket = dgram.createSocket("udp4");
  private port: number;
  private dir: string;
  private bootId = "";
  private file: WriteStream | null = null;
  private arrivals: WriteStream | null = null;
  private tracker = new TakeTracker();
  private dedup = new Dedup();

  constructor(opts: RecordingServerOptions) {
    super();
    this.port = opts.port ?? DATA_PORT;
    this.dir = path.join(opts.dir, "raw");
    this.socket.on("message", msg => this.received(msg));
  }

  async start(): Promise<number> {
    mkdirSync(this.dir, { recursive: true });
    await new Promise<void>((resolve, reject) => {
      this.socket.once("error", reject);
      this.socket.bind(this.port, () => resolve());
    });
    return this.socket.address().port;
  }

  stop(): void {
    this.tracker.finish();
    this.file?.end();
    this.arrivals?.end();
    this.socket.close();
  }

  // The console's text arrived (n bytes): noted beside the recording, if one is open.
  consoleArrived(n: number): void { this.arrivals?.write(`${Date.now()},console,${n}\n`); }

  private received(msg: Buffer): void {
    const d = parseDatagram(msg);
    if (!d || d.version < 3) return;
    if (d.bootId !== this.bootId) this.newFile(d.bootId);
    const entry = Buffer.alloc(10);
    entry.writeUInt16LE(msg.length, 0);
    entry.writeDoubleLE(Date.now(), 2);
    this.file!.write(Buffer.concat([entry, msg]));
    if (!this.dedup.fresh(d)) return;
    for (const r of d.records) this.tracker.add(r, d.seq);
    if (d.broken) this.emit("note", `datagram ${d.seq}: a record cut off`);
  }

  private newFile(bootId: string): void {
    this.tracker.finish();
    this.file?.end();
    this.arrivals?.end();
    this.bootId = bootId;
    const stamp = new Date().toLocaleString("sv-SE").replace(/[ :]/g, "-");
    const file = path.join(this.dir, `${stamp}-boot-${bootId}.rec`);
    this.file = createWriteStream(file);
    this.file.write(FILE_MAGIC);
    this.arrivals = createWriteStream(file.replace(/\.rec$/, ".arrivals.csv"));
    this.arrivals.write("ms,connection,bytes\n");
    this.tracker = new TakeTracker();
    this.tracker.onStart = t => this.emit("note", `recording ${t.name}`);
    this.tracker.onEnd = t => this.emit("note", t.summary());
    this.emit("note", `recordings from boot ${bootId}: saving to raw/${path.basename(file)}`);
  }
}
