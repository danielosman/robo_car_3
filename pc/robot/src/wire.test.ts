// Tests for wire.ts: the golden bytes the robot's side checks too
// (picoA/app/test/data/wire_golden.hex, written by test_wire.c): each sample encoded
// here gives its line's bytes and each line decodes to its sample, so what the robot
// makes is what this server reads and the reverse; the table and the version are
// the robot's; the compact tof frame at its limits as the robot packs it, unpacked
// and packed again to the same bytes, after its move and time; the imu batches (10
// records, one record; times past 2^32 µs). Then every message encoded and decoded again, and the envelope: a
// stream cut anywhere, unknown types, too long, too short. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import {
  HAS, IMU_RECORD_PERIOD_US, LANE, TYPE, WIRE_VERSION, WireReader, decode, encodeAnswer, encodeCommand, encodeConsole, encodeHeartbeat,
  encodeHello, encodeImuBatch, encodeMapCleared, encodeMoveEnd, encodeMoveStart, encodeSnapshotPart, encodeStatus, encodeTof, take, typeInfo,
  wireCheck, type ImuBatch, type ImuRecord, type Message,
} from "./wire.ts";
import { KINDS, TOF_COMPACT_BYTES, packTof, unpackTof } from "./tof_format.ts";

const GOLDEN = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../picoA/app/test/data/wire_golden.hex");
const f = Math.fround;

// test_wire.c's sample_batch(n): records 9615 µs apart from 5000000123 µs after
// power-up (past 2^32), counts i × 1000 - 7k and -i × 300 + k; 1 g up on the first,
// the int16 limits on the last.
function sampleBatch(n: number): ImuBatch {
  const records = Array.from({ length: n }, (_, i): ImuRecord => ({
    tUs: 5000000123 + i * IMU_RECORD_PERIOD_US,
    accel: [0, 1, 2].map(k => i * 1000 - k * 7) as ImuRecord["accel"], gyro: [0, 1, 2].map(k => -i * 300 + k) as ImuRecord["gyro"],
  }));
  records[0].accel[2] = 16384;
  records[n - 1].accel[0] = 32767; records[n - 1].gyro[1] = -32768;
  return { move: 12, records };
}

// The samples, as test_wire.c's samples() encodes them; floats as f32 holds them.
const samples: Record<string, { bytes: Buffer; message: Message }> = {
  hello: { bytes: encodeHello({ version: WIRE_VERSION, bootId: "7a587226", build: "Nov  7 2026 18:00:00" }),
           message: { type: "hello", version: WIRE_VERSION, bootId: "7a587226", build: "Nov  7 2026 18:00:00" } },
  console: { bytes: encodeConsole("Turn: done, 90.0 deg"), message: { type: "console", text: "Turn: done, 90.0 deg" } },
  console_empty: { bytes: encodeConsole(""), message: { type: "console", text: "" } },
  answer_move: {
    bytes: encodeAnswer({ n: 17, end: "obstacle", has: HAS.MOVE | HAS.GOAL, x: 0.43, y: -0.04, yaw: 0.5236, moved: 0.31, rotated: -0.125, goal: 4.2, snapshot: 0 }),
    message: { type: "answer", n: 17, end: "obstacle", has: 3, x: f(0.43), y: f(-0.04), yaw: f(0.5236), moved: f(0.31), rotated: -0.125, goal: f(4.2), snapshot: 0 },
  },
  answer_snapshot: {
    bytes: encodeAnswer({ n: 4, end: "ok", has: HAS.SNAPSHOT, x: 1.25, y: -0.5, yaw: 0.75, moved: 0, rotated: 0, goal: 0, snapshot: 41 }),
    message: { type: "answer", n: 4, end: "ok", has: 4, x: 1.25, y: -0.5, yaw: 0.75, moved: 0, rotated: 0, goal: 0, snapshot: 41 },
  },
  answer_refused: {
    bytes: encodeAnswer({ n: 9, end: "refused", has: 0, x: 0, y: 0, yaw: 0, moved: 0, rotated: 0, goal: 0, snapshot: 0 }),
    message: { type: "answer", n: 9, end: "refused", has: 0, x: 0, y: 0, yaw: 0, moved: 0, rotated: 0, goal: 0, snapshot: 0 },
  },
  status: {
    bytes: encodeStatus({ x: 1.25, y: -0.5, yaw: 7, mode: "manual", action: "goto", rssi: -61 }),
    message: { type: "status", x: 1.25, y: -0.5, yaw: 7, mode: "manual", action: "goto", rssi: -61 },
  },
  map_cleared: { bytes: encodeMapCleared(), message: { type: "map_cleared" } },
  move_start: {
    bytes: encodeMoveStart({ move: 12, command: 17, kind: "goto", param: 4.2, tUs: 4300000000, preMs: 200, postMs: 300, refused: true }),
    message: { type: "move_start", move: 12, command: 17, kind: "goto", param: f(4.2), tUs: 4300000000, preMs: 200, postMs: 300, refused: true },
  },
  move_end: {
    bytes: encodeMoveEnd({ move: 12, end: "tilt", tUs: 0xffffffffffff, framesSent: 41, framesDropped: 2, moved: 0.31, rotated: -0.125,
                           imuSent: 44, imuDropped: 3 }),
    message: { type: "move_end", move: 12, end: "tilt", tUs: 0xffffffffffff, framesSent: 41, framesDropped: 2, moved: f(0.31), rotated: -0.125,
               imuSent: 44, imuDropped: 3 },
  },
  imu_batch: { bytes: encodeImuBatch(sampleBatch(10)), message: { type: "imu_batch", ...sampleBatch(10) } },
  imu_batch_one: { bytes: encodeImuBatch(sampleBatch(1)), message: { type: "imu_batch", ...sampleBatch(1) } },
  snapshot_part: { bytes: encodeSnapshotPart(Buffer.from([0x53, 0x4e, 0x41, 0x50, 1, 2, 0xfe, 0xff])),
                   message: { type: "snapshot_part", bytes: Buffer.from([0x53, 0x4e, 0x41, 0x50, 1, 2, 0xfe, 0xff]) } },
  command: { bytes: encodeCommand({ n: 17, text: "drive 50" }), message: { type: "command", n: 17, text: "drive 50" } },
  heartbeat: { bytes: encodeHeartbeat(), message: { type: "heartbeat" } },
};

function decodeWhole(bytes: Buffer): Message {
  const t = take(bytes);
  assert.ok("message" in t && t.length === bytes.length, `one whole message: ${bytes.toString("hex")}`);
  return decode(t.message);
}

// The table as test_wire.c writes it: each known type's number, lane, min, max.
function tableLine(): string {
  const out: string[] = [];
  for (let t = 0; t < 256; t++) {
    const i = typeInfo(t);
    if (i) out.push(`${t.toString(16).padStart(2, "0")}:${i.lane}:${i.min}:${i.max}`);
  }
  return out.join(" ");
}

test("the golden bytes: the robot's encoding is ours, and its table and version", () => {
  const lines = readFileSync(GOLDEN, "utf8").split("\n").filter(l => l && !l.startsWith("#"));
  const seen = new Set<string>();
  for (const line of lines) {
    const [name, rest] = [line.slice(0, line.indexOf(" ")), line.slice(line.indexOf(" ") + 1)];
    if (name === "version") { assert.equal(Number(rest), WIRE_VERSION, "the robot's wire version"); continue; }
    if (name === "table") { assert.equal(tableLine(), rest, "the message table"); continue; }
    if (name === "tof_compact") { checkTof(Buffer.from(rest, "hex")); seen.add(name); continue; }
    const sample = samples[name];
    assert.ok(sample, `a sample for ${name}`);
    assert.equal(sample.bytes.toString("hex"), rest, `${name}: our bytes`);
    assert.deepEqual(decodeWhole(Buffer.from(rest, "hex")), sample.message, `${name}: the robot's bytes read`);
    seen.add(name);
  }
  assert.deepEqual([...seen].sort(), [...Object.keys(samples), "tof_compact"].sort(), "every sample in the file");
});

// test_wire.c's tof frame at the format's limits: zone k distance 47k mm (zone 63 3 m:
// beyond), signal 9k, sigma 5k (capped at 255), kind k % 4, targets k % 5.
function checkTof(bytes: Buffer): void {
  const m = decodeWhole(bytes);
  assert.ok(m.type === "tof" && m.format === "compact" && m.move === 12 && m.tUs === 4300000000);
  const t = m.frame;
  assert.deepEqual([t.number, t.robotClockUs, t.periodUs, t.x, t.y, t.still, t.poseKnown], [4000000000, 0xfffffff0, 66700, -1.234, 32, true, true]);
  assert.ok(Math.abs(t.yaw - -3) < 1e-4 && Math.abs(t.pitch - -0.0157) < 1e-4 && Math.abs(t.turn - 0.1) < 1e-4);
  for (const [k, z] of t.zones.entries()) {
    const kind = KINDS[k % 4], mm = kind === "nothing" ? 0 : k === 63 || 47 * k >= 2545 ? 2550 : Math.max(10, Math.floor((47 * k + 5) / 10) * 10);
    assert.deepEqual(z, { distanceMm: k === 0 ? 0 : mm, signal: Math.min(255, 9 * k), sigmaMm: Math.min(255, 5 * k), kind, targets: k % 5 }, `zone ${k}`);
  }
  assert.deepEqual(packTof("compact", t), bytes.subarray(3 + 12), "packed again: the same bytes");
  assert.deepEqual(encodeTof("compact", 12, 4300000000, t), bytes);
  assert.throws(() => unpackTof("compact", bytes.subarray(3 + 12 + 1)), /277 B/);
}

test("the table: lanes, the ToF's type from tof_format, no collisions", () => {
  assert.ok(wireCheck());
  assert.equal(typeInfo(TYPE.ANSWER)!.lane, LANE.CONTROL);
  assert.equal(typeInfo(TYPE.SNAPSHOT_PART)!.lane, LANE.BULK);
  assert.deepEqual(typeInfo(TYPE.MOVE_START), { lane: LANE.CONTROL, min: 26, max: 26 });
  assert.deepEqual(typeInfo(TYPE.MOVE_END), { lane: LANE.CONTROL, min: 37, max: 37 });
  assert.deepEqual(typeInfo(TYPE.STATUS), { lane: LANE.CONTROL, min: 33, max: 33 });
  assert.deepEqual(typeInfo(TYPE.IMU_BATCH), { lane: LANE.BULK, min: 27, max: 135 });
  assert.deepEqual(typeInfo(0x40), { lane: LANE.BULK, min: 12 + TOF_COMPACT_BYTES, max: 12 + TOF_COMPACT_BYTES }); // move, time, frame
  assert.equal(typeInfo(0x99), null);
  assert.equal(typeInfo("W".charCodeAt(0)), null); // an old robot's text
});

test("every message encoded and decoded again", () => {
  const cases: Message[] = [
    { type: "hello", version: WIRE_VERSION, bootId: "00000001", build: "123456789012345678901234" },
    { type: "console", text: "Turn: 90°" },
    { type: "answer", n: 0xffffffff, end: "picob_lost", has: 7, x: -1.5, y: 2.25, yaw: -3, moved: 0.5, rotated: 6.25, goal: 3, snapshot: 123456 },
    { type: "status", x: 0, y: 0, yaw: -12.5, mode: "idle", action: "", rssi: 0 },
    { type: "map_cleared" },
    { type: "move_start", move: 1, command: 0, kind: "watch", param: 60, tUs: 0, preMs: 0, postMs: 2000, refused: false },
    { type: "move_end", move: 0xffffffff, end: "no_start", tUs: 0xffffffff, framesSent: 0, framesDropped: 9, moved: 0, rotated: -6.5,
      imuSent: 0, imuDropped: 0xffffffff },
    { type: "imu_batch", ...sampleBatch(7) },
    { type: "imu_batch", move: 1, records: [{ tUs: 0xffffffff, accel: [-32768, 0, 32767], gyro: [1, -1, 0] }] },
    { type: "snapshot_part", bytes: Buffer.alloc(1400, 7) },
    { type: "command", n: 3, text: "goto 120 -35" },
    { type: "heartbeat" },
  ];
  const encode = (m: Message): Buffer => {
    switch (m.type) {
      case "hello": return encodeHello(m);
      case "console": return encodeConsole(m.text);
      case "answer": return encodeAnswer(m);
      case "status": return encodeStatus(m);
      case "map_cleared": return encodeMapCleared();
      case "move_start": return encodeMoveStart(m);
      case "move_end": return encodeMoveEnd(m);
      case "imu_batch": return encodeImuBatch(m);
      case "snapshot_part": return encodeSnapshotPart(m.bytes);
      case "command": return encodeCommand(m);
      case "heartbeat": return encodeHeartbeat();
      default: throw new Error("no encoder");
    }
  };
  for (const m of cases) assert.deepEqual(decodeWhole(encode(m)), m);
  for (const end of ["ok", "obstacle", "stopped", "replaced", "tilt", "wheels", "no_drive", "picob_lost", "no_start", "refused", "safety", "ended"]) {
    const a = decodeWhole(encodeAnswer({ n: 1, end, has: 0, x: 0, y: 0, yaw: 0, moved: 0, rotated: 0, goal: 0, snapshot: 0 }));
    assert.equal(a.type === "answer" && a.end, end);
  }
  assert.equal(decodeWhole(encodeConsole("x".repeat(300))).type === "console" && (decodeWhole(encodeConsole("x".repeat(300))) as { text: string }).text.length, 240);
  assert.throws(() => encodeCommand({ n: 1, text: "" }));
  assert.throws(() => encodeCommand({ n: 1, text: "drive\n1" }));
  assert.throws(() => encodeCommand({ n: 1, text: "a".repeat(47) }));
  assert.throws(() => encodeSnapshotPart(Buffer.alloc(1401)));
  assert.throws(() => encodeImuBatch({ move: 1, records: [] }), /1-10 records/);
  assert.throws(() => encodeImuBatch({ move: 1, records: sampleBatch(10).records.concat(sampleBatch(1).records) }), /not 11/);
  // Times off the even spacing come back on it: the period from the first to the last.
  const jitter = sampleBatch(3);
  jitter.records[1].tUs += 40; jitter.records[2].tUs += 1;
  assert.throws(() => encodeImuBatch(jitter), /evenly spaced/); // the robot rounds them (move.c), not the encoder
  assert.throws(() => encodeMoveStart({ move: 1, command: 0, kind: "fly", param: 0, tUs: 0, preMs: 0, postMs: 0, refused: false }));
  assert.throws(() => encodeAnswer({ n: 1, end: "maybe", has: 0, x: 0, y: 0, yaw: 0, moved: 0, rotated: 0, goal: 0, snapshot: 0 }));
});

test("the envelope: a stream cut anywhere, merged chunks; errors stop it", () => {
  const stream = Buffer.concat([samples.hello.bytes, samples.console.bytes, samples.status.bytes, encodeSnapshotPart(Buffer.alloc(1400, 3)),
                                samples.answer_move.bytes, samples.map_cleared.bytes]);
  for (const size of [1, 2, 3, 5, 64, 1000, stream.length]) {
    const r = new WireReader(), types: number[] = [];
    for (let at = 0; at < stream.length; at += size) types.push(...r.push(stream.subarray(at, at + size)).map(m => m.type));
    assert.deepEqual(types, [TYPE.HELLO, TYPE.CONSOLE, TYPE.STATUS, TYPE.SNAPSHOT_PART, TYPE.ANSWER, TYPE.MAP_CLEARED]);
    assert.equal(r.error, null);
  }
  for (let len = 0; len < samples.answer_move.bytes.length; len++) assert.deepEqual(take(samples.answer_move.bytes.subarray(0, len)), { length: 0 });
  const errors: [Buffer, RegExp][] = [
    [Buffer.from("WiFi: connected\n"), /unknown message type 0x57/],         // an old robot's text
    [Buffer.from([0x02, 241, 0]), /too long/],                              // console over 240 B
    [Buffer.from([0x10, 0x79, 0x05]), /too long/],                          // a 1401 B snapshot part
    [Buffer.from([0x03, 33, 0]), /too short/],                              // an answer of 33 B
    [Buffer.from([0x21, 1, 0]), /too long/],                                // a heartbeat with a payload
    [Buffer.from([0x04, 49, 0]), /too long/],                               // the old status, with PicoB's peaks
    [Buffer.from([0x07, 41, 0]), /too long/],                               // version 3's move end (frame numbers, a u32 time)
    [Buffer.from([0x11, 26, 0]), /too short/],                              // an imu batch without a record
    [Buffer.from([0x11, 136, 0]), /too long/],                              // more than 10 records
  ];
  for (const [bytes, error] of errors) {
    const r = new WireReader();
    assert.deepEqual(r.push(Buffer.concat([samples.console.bytes, bytes])).map(m => m.type), [TYPE.CONSOLE]);
    assert.match(r.error!, error);
    assert.deepEqual(r.push(samples.console.bytes), []); // broken: stays broken
  }
  // A hello of another length passes the envelope (another version), but isn't decoded as ours.
  const other = Buffer.concat([Buffer.from([0x01, 40, 0, 9]), Buffer.alloc(39)]);
  const t = take(other);
  assert.ok("message" in t && t.message.payload[0] === 9);
  assert.deepEqual(decode(t.message), { type: "hello", version: 9, bootId: "", build: "" }); // read for its version only
  const ours = Buffer.concat([Buffer.from([0x01, 40, 0, WIRE_VERSION]), Buffer.alloc(39)]);
  const t2 = take(ours);
  assert.ok("message" in t2);
  assert.throws(() => decode(t2.message), /40 B, not 29/); // our version must have our length
});

test("an imu batch whose count doesn't match its length, or is 0, is refused", () => {
  const bytes = Buffer.from(encodeImuBatch(sampleBatch(3)));
  bytes[3 + 14] = 2;
  const t = take(bytes);
  assert.ok("message" in t);
  assert.throws(() => decode(t.message), /51 B, not 39/);
  bytes[3 + 14] = 0;
  assert.throws(() => decode((take(bytes) as { message: { type: number; payload: Buffer } }).message), /0 records/);
});

test("a console line cut at 240 B inside a UTF-8 character: the half reads as U+FFFD", () => {
  const m = decodeWhole(encodeConsole(`${"x".repeat(239)}°C`)); // ° is 2 bytes: its first one is the 240th
  assert.deepEqual(m, { type: "console", text: `${"x".repeat(239)}\uFFFD` });
});
