// The messages between the robot and this server on their one TCP connection: the
// twin of picoA/app/wire.h (the table, the layouts and the reasons are there). Pure
// functions, no I/O: only this module knows the bytes. Both sides are checked
// against the same bytes, picoA/app/test/data/wire_golden.hex (wire.test.ts here,
// test_wire.c on the robot's side).
//
// A message: [type u8][payload length u16][payload], little-endian, floats f32.
import { TOF_FORMATS, packTof, tofFormatOfType, unpackTof, type PosedFrame } from "./tof_format.ts";

export const WIRE_VERSION = 3; // as picoA/app/wire.h; the server refuses a robot with another
export const HEADER = 3;
export const PAYLOAD_MAX = 1400;

export const TYPE = {
  HELLO: 0x01, CONSOLE: 0x02, ANSWER: 0x03, STATUS: 0x04, MAP_CLEARED: 0x05, MOVE_START: 0x06, MOVE_END: 0x07,
  SNAPSHOT_PART: 0x10, IMU_BATCH: 0x11,
  COMMAND: 0x20, HEARTBEAT: 0x21,
} as const;

export const LANE = { CONTROL: 0, BULK: 1, TO_ROBOT: 2 } as const;
export type Lane = (typeof LANE)[keyof typeof LANE];
export interface TypeInfo { lane: Lane; min: number; max: number; droppable: boolean }

const HELLO_SIZE = 29, BUILD_LEN = 24, CONSOLE_MAX = 240, ANSWER_SIZE = 34, STATUS_SIZE = 33, MODE_LEN = 8, ACTION_LEN = 12;
const MOVE_START_SIZE = 22, MOVE_END_SIZE = 41, IMU_BATCH_HEADER = 11, IMU_RECORD = 12;
export const PART_MAX = 1400, COMMAND_TEXT_MAX = 46;
export const IMU_BATCH_MAX = 10;          // records in an imu batch
export const IMU_RECORD_PERIOD_US = 9615; // 4 samples at 416 Hz (common/link_msgs.h)
// An imu record's counts (common/link_msgs.h): accel 0.061 mg, gyro 8.75 mdps a count.
export const IMU_ACCEL_G_PER_COUNT = 0.000061, IMU_GYRO_DPS_PER_COUNT = 0.00875;

const TABLE = new Map<number, TypeInfo>([
  [TYPE.HELLO, { lane: LANE.CONTROL, min: 1, max: PAYLOAD_MAX, droppable: false }], // any length: another version's hello is read for its version
  [TYPE.CONSOLE, { lane: LANE.CONTROL, min: 0, max: CONSOLE_MAX, droppable: true }],
  [TYPE.ANSWER, { lane: LANE.CONTROL, min: ANSWER_SIZE, max: ANSWER_SIZE, droppable: false }],
  [TYPE.STATUS, { lane: LANE.CONTROL, min: STATUS_SIZE, max: STATUS_SIZE, droppable: true }],
  [TYPE.MAP_CLEARED, { lane: LANE.CONTROL, min: 0, max: 0, droppable: false }],
  [TYPE.MOVE_START, { lane: LANE.CONTROL, min: MOVE_START_SIZE, max: MOVE_START_SIZE, droppable: false }],
  [TYPE.MOVE_END, { lane: LANE.CONTROL, min: MOVE_END_SIZE, max: MOVE_END_SIZE, droppable: false }],
  [TYPE.SNAPSHOT_PART, { lane: LANE.BULK, min: 1, max: PART_MAX, droppable: false }],
  [TYPE.IMU_BATCH, { lane: LANE.BULK, min: IMU_BATCH_HEADER + IMU_RECORD, max: IMU_BATCH_HEADER + IMU_BATCH_MAX * IMU_RECORD, droppable: true }],
  [TYPE.COMMAND, { lane: LANE.TO_ROBOT, min: 5, max: 4 + COMMAND_TEXT_MAX, droppable: false }],
  [TYPE.HEARTBEAT, { lane: LANE.TO_ROBOT, min: 0, max: 0, droppable: false }],
]);

// What the table says of a type (the ToF frames' from tof_format.ts); null if it doesn't know it.
export function typeInfo(type: number): TypeInfo | null {
  const t = TABLE.get(type);
  if (t) return t;
  const f = tofFormatOfType(type);
  return f ? { lane: LANE.BULK, min: f.size, max: f.size, droppable: true } : null;
}

// The table's types are all different (the ToF's included).
export function wireCheck(): boolean {
  const types = [...TABLE.keys(), ...TOF_FORMATS.map(f => f.messageType)];
  return new Set(types).size === types.length && TOF_FORMATS.every(f => f.size <= PAYLOAD_MAX);
}

// How a command ended (wire_end_t): the robot's end words, as api/answers.ts names them.
export const END_WORDS = ["ok", "obstacle", "stopped", "replaced", "tilt", "wheels", "no_drive", "picob_lost", "no_start",
                          "refused", "safety", "ended"] as const;
export const HAS = { MOVE: 1, GOAL: 2, SNAPSHOT: 4 } as const;
// A move's kind (wire_move_kind_t; picoA/app/move.h).
export const MOVE_KINDS = ["drive", "turn", "goto", "watch"] as const;

export interface Hello { version: number; bootId: string; build: string } // bootId: 8 hex digits, as the snapshots'
export interface Answer {
  n: number; end: string; has: number; x: number; y: number; yaw: number; moved: number; rotated: number; goal: number; snapshot: number;
}
export interface Status { x: number; y: number; yaw: number; mode: string; action: string; rssi: number }
// A move's start (picoA/app/move.h): its number, its command's, kind, parameter
// (drive m, turn rad, goto m to the goal, watch s), PicoA's time, the still times;
// refused: it can't start, its end follows at once and a running move goes on.
export interface MoveStart { move: number; command: number; kind: string; param: number; tUs: number; preMs: number; postMs: number; refused: boolean }
// A move's end: how (an end word), PicoA's time, its frames sent and dropped, the first
// and last frame number sent (0: none), how far it went (m) and turned (rad), its imu
// batches sent and dropped.
export interface MoveEnd {
  move: number; end: string; tUs: number; framesSent: number; framesDropped: number; firstFrame: number; lastFrame: number;
  moved: number; rotated: number; imuSent: number; imuDropped: number;
}
// The IMU as it measured (common/link_msgs.h, imu_record_t): PicoA's time (the middle
// of its 4 samples), accel and gyro x, y, z as the sensor's raw counts in its own axes.
export type Vec3 = [number, number, number];
export interface ImuRecord { tUs: number; accel: Vec3; gyro: Vec3 }
// Up to 10 imu records of a move, measured one after the other, evenly spaced.
export interface ImuBatch { move: number; records: ImuRecord[] }
export interface Command { n: number; text: string }

export type Message =
  | ({ type: "hello" } & Hello)
  | { type: "console"; text: string }
  | ({ type: "answer" } & Answer)
  | ({ type: "status" } & Status)
  | { type: "map_cleared" }
  | ({ type: "move_start" } & MoveStart)
  | ({ type: "move_end" } & MoveEnd)
  | { type: "snapshot_part"; bytes: Buffer }
  | ({ type: "imu_batch" } & ImuBatch)
  | ({ type: "command" } & Command)
  | { type: "heartbeat" }
  | { type: "tof"; format: string; frame: PosedFrame };

// --- the envelope ---

export interface RawMessage { type: number; payload: Buffer }
export type Take = { length: number; message: RawMessage } | { length: 0 } | { error: string };

// The first message in bytes: whole (its length), not whole yet (length 0), or an error.
export function take(bytes: Buffer): Take {
  if (bytes.length < 1) return { length: 0 };
  const info = typeInfo(bytes[0]);
  if (!info) return { error: `unknown message type 0x${bytes[0].toString(16).padStart(2, "0")}` };
  if (bytes.length < HEADER) return { length: 0 };
  const n = bytes.readUInt16LE(1);
  if (n > info.max) return { error: `message 0x${bytes[0].toString(16)} too long for its type (${n} B)` };
  if (n < info.min) return { error: `message 0x${bytes[0].toString(16)} too short for its type (${n} B)` };
  if (bytes.length < HEADER + n) return { length: 0 };
  return { length: HEADER + n, message: { type: bytes[0], payload: bytes.subarray(HEADER, HEADER + n) } };
}

// Splits a TCP stream into messages, whatever its chunks; once broken, it stays broken.
export class WireReader {
  private buf = Buffer.alloc(0);
  error: string | null = null;

  push(chunk: Buffer): RawMessage[] {
    if (this.error) return [];
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : Buffer.from(chunk);
    const out: RawMessage[] = [];
    for (;;) {
      const t = take(this.buf);
      if ("error" in t) { this.error = t.error; this.buf = Buffer.alloc(0); break; }
      if (!("message" in t)) break;
      out.push({ type: t.message.type, payload: Buffer.from(t.message.payload) });
      this.buf = this.buf.subarray(t.length);
    }
    return out;
  }
}

function header(type: number, payload: Buffer): Buffer {
  const h = Buffer.alloc(HEADER);
  h[0] = type;
  h.writeUInt16LE(payload.length, 1);
  return Buffer.concat([h, payload]);
}

// A fixed text field: NUL-padded, cut to size; read up to the first NUL.
const textField = (text: string, size: number) => { const b = Buffer.alloc(size); b.write(text.slice(0, size), "latin1"); return b; };
const readText = (b: Buffer) => { const z = b.indexOf(0); return b.subarray(0, z < 0 ? b.length : z).toString("latin1"); };

// --- decoding: throws if the payload doesn't fit its type ---

export function decode(m: RawMessage): Message {
  const p = m.payload, need = (n: number) => { if (p.length !== n) throw new Error(`message 0x${m.type.toString(16)}: ${p.length} B, not ${n}`); };
  switch (m.type) {
    case TYPE.HELLO:
      // The version first: another version's hello may have another length; it is
      // read for its version only, so the robot can be refused for that.
      if (p[0] !== WIRE_VERSION) return { type: "hello", version: p[0], bootId: "", build: "" };
      need(HELLO_SIZE);
      return { type: "hello", version: p[0], bootId: p.readUInt32LE(1).toString(16).padStart(8, "0"), build: readText(p.subarray(5, 5 + BUILD_LEN)) };
    case TYPE.CONSOLE: return { type: "console", text: p.toString("utf8") }; // a character cut at 240 B reads as U+FFFD
    case TYPE.ANSWER:
      need(ANSWER_SIZE);
      return { type: "answer", n: p.readUInt32LE(0), end: END_WORDS[p[4]] ?? `end ${p[4]}`, has: p[5], x: p.readFloatLE(6), y: p.readFloatLE(10),
               yaw: p.readFloatLE(14), moved: p.readFloatLE(18), rotated: p.readFloatLE(22), goal: p.readFloatLE(26), snapshot: p.readUInt32LE(30) };
    case TYPE.STATUS:
      need(STATUS_SIZE);
      return { type: "status", x: p.readFloatLE(0), y: p.readFloatLE(4), yaw: p.readFloatLE(8), mode: readText(p.subarray(12, 12 + MODE_LEN)),
               action: readText(p.subarray(20, 20 + ACTION_LEN)), rssi: p.readInt8(32) };
    case TYPE.MAP_CLEARED: need(0); return { type: "map_cleared" };
    case TYPE.MOVE_START:
      need(MOVE_START_SIZE);
      return { type: "move_start", move: p.readUInt32LE(0), command: p.readUInt32LE(4), kind: MOVE_KINDS[p[8]] ?? `kind ${p[8]}`,
               param: p.readFloatLE(9), tUs: p.readUInt32LE(13), preMs: p.readUInt16LE(17), postMs: p.readUInt16LE(19),
               refused: (p[21] & 1) !== 0 };
    case TYPE.MOVE_END:
      need(MOVE_END_SIZE);
      return { type: "move_end", move: p.readUInt32LE(0), end: END_WORDS[p[4]] ?? `end ${p[4]}`, tUs: p.readUInt32LE(5),
               framesSent: p.readUInt32LE(9), framesDropped: p.readUInt32LE(13), firstFrame: p.readUInt32LE(17),
               lastFrame: p.readUInt32LE(21), moved: p.readFloatLE(25), rotated: p.readFloatLE(29), imuSent: p.readUInt32LE(33),
               imuDropped: p.readUInt32LE(37) };
    case TYPE.SNAPSHOT_PART: return { type: "snapshot_part", bytes: p };
    case TYPE.IMU_BATCH: {
      const n = p[10];
      if (n < 1 || n > IMU_BATCH_MAX) throw new Error(`imu batch: ${n} records`);
      need(IMU_BATCH_HEADER + n * IMU_RECORD);
      const first = p.readUInt32LE(4), period = p.readUInt16LE(8);
      const records = Array.from({ length: n }, (_, i): ImuRecord => {
        const at = IMU_BATCH_HEADER + i * IMU_RECORD, v = (k: number) => p.readInt16LE(at + 2 * k);
        return { tUs: (first + i * period) % 2 ** 32, accel: [v(0), v(1), v(2)], gyro: [v(3), v(4), v(5)] };
      });
      return { type: "imu_batch", move: p.readUInt32LE(0), records };
    }
    case TYPE.COMMAND: {
      const text = p.subarray(4).toString("latin1");
      if (!commandTextOk(text)) throw new Error(`command: not printable text: ${JSON.stringify(text)}`);
      return { type: "command", n: p.readUInt32LE(0), text };
    }
    case TYPE.HEARTBEAT: need(0); return { type: "heartbeat" };
  }
  const f = tofFormatOfType(m.type);
  if (f) { need(f.size); return { type: "tof", format: f.name, frame: unpackTof(f.name, p) }; }
  throw new Error(`unknown message type 0x${m.type.toString(16)}`);
}

// --- encoding: each returns the whole message ---

export function encodeHello(h: Hello): Buffer {
  const p = Buffer.alloc(HELLO_SIZE);
  p[0] = h.version;
  p.writeUInt32LE(parseInt(h.bootId, 16) >>> 0, 1);
  textField(h.build, BUILD_LEN).copy(p, 5);
  return header(TYPE.HELLO, p);
}

export const encodeConsole = (text: string): Buffer => header(TYPE.CONSOLE, Buffer.from(text, "utf8").subarray(0, CONSOLE_MAX));

export function encodeAnswer(a: Answer): Buffer {
  const p = Buffer.alloc(ANSWER_SIZE), end = END_WORDS.indexOf(a.end as (typeof END_WORDS)[number]);
  if (end < 0) throw new Error(`not an end word: ${a.end}`);
  p.writeUInt32LE(a.n, 0); p[4] = end; p[5] = a.has;
  [a.x, a.y, a.yaw, a.moved, a.rotated, a.goal].forEach((v, i) => p.writeFloatLE(v, 6 + 4 * i));
  p.writeUInt32LE(a.snapshot, 30);
  return header(TYPE.ANSWER, p);
}

export function encodeStatus(s: Status): Buffer {
  const p = Buffer.alloc(STATUS_SIZE);
  p.writeFloatLE(s.x, 0); p.writeFloatLE(s.y, 4); p.writeFloatLE(s.yaw, 8);
  textField(s.mode, MODE_LEN).copy(p, 12);
  textField(s.action, ACTION_LEN).copy(p, 20);
  p.writeInt8(s.rssi, 32);
  return header(TYPE.STATUS, p);
}

export const encodeMapCleared = (): Buffer => header(TYPE.MAP_CLEARED, Buffer.alloc(0));

export function encodeMoveStart(s: MoveStart): Buffer {
  const p = Buffer.alloc(MOVE_START_SIZE), kind = MOVE_KINDS.indexOf(s.kind as (typeof MOVE_KINDS)[number]);
  if (kind < 0) throw new Error(`not a move kind: ${s.kind}`);
  p.writeUInt32LE(s.move >>> 0, 0); p.writeUInt32LE(s.command >>> 0, 4); p[8] = kind; p.writeFloatLE(s.param, 9);
  p.writeUInt32LE(s.tUs >>> 0, 13); p.writeUInt16LE(s.preMs, 17); p.writeUInt16LE(s.postMs, 19); p[21] = s.refused ? 1 : 0;
  return header(TYPE.MOVE_START, p);
}

export function encodeMoveEnd(e: MoveEnd): Buffer {
  const p = Buffer.alloc(MOVE_END_SIZE), end = END_WORDS.indexOf(e.end as (typeof END_WORDS)[number]);
  if (end < 0) throw new Error(`not an end word: ${e.end}`);
  p.writeUInt32LE(e.move >>> 0, 0); p[4] = end; p.writeUInt32LE(e.tUs >>> 0, 5);
  [e.framesSent, e.framesDropped, e.firstFrame, e.lastFrame].forEach((v, i) => p.writeUInt32LE(v >>> 0, 9 + 4 * i));
  p.writeFloatLE(e.moved, 25); p.writeFloatLE(e.rotated, 29);
  p.writeUInt32LE(e.imuSent >>> 0, 33); p.writeUInt32LE(e.imuDropped >>> 0, 37);
  return header(TYPE.MOVE_END, p);
}

// The records' times go as the first and a period (from the first to the last, over
// n - 1, rounded as the robot does): even spacing (the fake robot's replay, tests).
export function encodeImuBatch(b: ImuBatch): Buffer {
  const n = b.records.length;
  if (n < 1 || n > IMU_BATCH_MAX) throw new Error(`an imu batch has 1-${IMU_BATCH_MAX} records, not ${n}`);
  const p = Buffer.alloc(IMU_BATCH_HEADER + n * IMU_RECORD), first = b.records[0].tUs >>> 0;
  const span = (b.records[n - 1].tUs - first) >>> 0;
  const period = n > 1 ? Math.floor((span + Math.floor((n - 1) / 2)) / (n - 1)) : IMU_RECORD_PERIOD_US;
  p.writeUInt32LE(b.move >>> 0, 0); p.writeUInt32LE(first, 4); p.writeUInt16LE(Math.min(0xffff, period), 8); p[10] = n;
  b.records.forEach((r, i) => [...r.accel, ...r.gyro].forEach((v, k) => p.writeInt16LE(v, IMU_BATCH_HEADER + i * IMU_RECORD + 2 * k)));
  return header(TYPE.IMU_BATCH, p);
}

// A posed frame in its format's message (the fake robot's replay).
export function encodeTof(format: string, frame: PosedFrame): Buffer {
  const f = TOF_FORMATS.find(x => x.name === format);
  if (!f) throw new Error(`no ToF format ${format}`);
  return header(f.messageType, packTof(format, frame));
}

export function encodeSnapshotPart(bytes: Buffer): Buffer {
  if (bytes.length < 1 || bytes.length > PART_MAX) throw new Error(`a snapshot part is 1-${PART_MAX} B, not ${bytes.length}`);
  return header(TYPE.SNAPSHOT_PART, bytes);
}

const commandTextOk = (text: string) => /^[\x20-\x7e]{1,46}$/.test(text);

export function encodeCommand(c: Command): Buffer {
  if (!commandTextOk(c.text)) throw new Error(`not a command: ${JSON.stringify(c.text)}`);
  const p = Buffer.alloc(4 + c.text.length);
  p.writeUInt32LE(c.n >>> 0, 0);
  p.write(c.text, 4, "latin1");
  return header(TYPE.COMMAND, p);
}

export const encodeHeartbeat = (): Buffer => header(TYPE.HEARTBEAT, Buffer.alloc(0));
