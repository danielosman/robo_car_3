// Tests for boot.ts: the messages numbered (going on from the store's count); every
// row's time in s since power-up; a frame and an imu batch filed under the move they
// name, also when they come after its end. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { Boot } from "./boot.ts";
import { loadMounting } from "./mounting.ts";
import type { PosedFrame } from "./tof_format.ts";
import type { ImuBatch } from "./wire.ts";

const close = (a: number, b: number, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);

const frame = (number: number): PosedFrame =>
  ({ number, robotClockUs: 0, periodUs: 100000, x: 0, y: 0, yaw: 0, pitch: 0, turn: 0, still: true, poseKnown: true, beyondMm: 2550, zones: [] });

test("messages numbered; times since power-up; frames under the move they name", () => {
  const b = new Boot("0badf00d", loadMounting());
  assert.equal(b.stamp().msg, 1);
  const start = b.moveStart({ move: 1, command: 9, kind: "turn", param: 0.5, tUs: 29e6, preMs: 200, postMs: 200, refused: false });
  assert.equal(start.msg, 2);
  close(start.tS, 29);
  b.moveEnd({ move: 1, end: "ok", tUs: 31e6, framesSent: 3, framesDropped: 0, moved: 0, rotated: 0.5, imuSent: 0, imuDropped: 0 });
  b.moveStart({ move: 2, command: 10, kind: "drive", param: 0.5, tUs: 31.5e6, preMs: 200, postMs: 200, refused: false });
  const late = b.frame({ format: "compact", move: 1, tUs: 30.9e6, frame: frame(7) }); // move 1's, after its end and move 2's start
  assert.equal(late.move, 1);
  close(late.tS, 30.9);
  // Past 2^32 µs (71.6 min): nothing wraps.
  close(b.frame({ format: "compact", move: 2, tUs: 5000e6, frame: frame(8) }).tS, 5000);
  assert.equal(b.messages, 6);
  // Seen again after a server restart: numbering goes on from the store's count.
  assert.equal(new Boot("0badf00d", loadMounting(), 6).stamp().msg, 7);
});

// n records of move from t0 (µs since power-up), 9615 µs apart.
const batch = (move: number, t0: number, n: number): ImuBatch =>
  ({ move, records: Array.from({ length: n }, (_, i) => ({ tUs: t0 + i * 9615, accel: [i, 0, 16384], gyro: [0, 0, -i] })) });

test("an imu batch: its records in s since power-up, under the move it names however late", () => {
  const b = new Boot("0badf00d", loadMounting());
  const row = b.imu(batch(1, 29_950_000, 10));
  assert.equal(row.move, 1);
  assert.equal(row.records.length, 10);
  row.records.forEach((r, i) => { close(r.tS, 29.95 + i * 0.009615); assert.deepEqual(r.accel, [i, 0, 16384]); });
  b.moveStart({ move: 2, command: 10, kind: "drive", param: 0.5, tUs: 31.5e6, preMs: 200, postMs: 200, refused: false });
  assert.equal(b.imu(batch(1, 30.9e6, 4)).move, 1); // move 1's last batch, after move 2 started
});
