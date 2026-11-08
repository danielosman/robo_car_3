// Tests for boot.ts: PicoA's 32-bit µs clock unwrapped into the boot's time since
// power-up (it starts 30 s before the wrap, as the robot's does: common/clock_start.h),
// across the wrap and the next ones, with times that come a little out of order (a
// frame's middle before the message before it), first seen after the wrap, and after
// the server was away for hours; the messages numbered; a frame's move (the open one,
// or the one that ended and sent it); an imu batch's records on the boot's time
// (across the wrap), filed under the move it names, also late. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { Boot, BootClock, CLOCK_START_US, WRAP_US } from "./boot.ts";
import { loadMounting } from "./mounting.ts";
import type { PosedFrame } from "./tof_format.ts";
import type { ImuBatch } from "./wire.ts";

const close = (a: number, b: number, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const raw = (sincePowerUpUs: number) => (CLOCK_START_US + sincePowerUpUs) % WRAP_US; // what PicoA's time_us_32() says then

test("the clock from power-up: across the wrap 30 s in, and every 71.6 min after", () => {
  const c = new BootClock(), t0 = Date.UTC(2026, 10, 7, 20);
  assert.equal(c.sincePowerUp(raw(0), t0), 0);
  assert.equal(c.sincePowerUp(raw(29_999_999), t0 + 30_000), 29_999_999);
  assert.equal(raw(30_000_001), 1); // the wrap
  assert.equal(c.sincePowerUp(raw(30_000_001), t0 + 30_000), 30_000_001);
  // A frame's middle a little before the message before it: a little earlier, not 71 min later.
  assert.equal(c.sincePowerUp(raw(29_950_000), t0 + 30_001), 29_950_000);
  assert.equal(c.sincePowerUp(raw(30_100_000), t0 + 30_100), 30_100_000);
  // Then every few seconds for 3 hours: two more wraps, the time only grows.
  let last = 30_100_000;
  for (let s = 31; s < 3 * 3600; s += 7) {
    const us = c.sincePowerUp(raw(s * 1e6), t0 + s * 1000);
    assert.equal(us, s * 1e6);
    assert.ok(us > last);
    last = us;
  }
  assert.ok(last > 2 * WRAP_US);
});

test("first seen after the wrap; the server away for hours", () => {
  const t0 = Date.UTC(2026, 10, 7, 20);
  const late = new BootClock(); // a server that starts 10 min after the robot
  assert.equal(late.sincePowerUp(raw(600e6), t0), 600e6);
  // The server stops; 3 h 20 min later the robot (still on) is heard again: PicoA's
  // clock wrapped 2 more times meanwhile; the wall clock says how many.
  const away = 12_000e6;
  const after = new BootClock(late.state);
  assert.equal(after.sincePowerUp(raw(600e6 + away), t0 + away / 1000), 600e6 + away);
  // A fast replay (no wall time between messages) still counts by the robot's own steps.
  const fast = new BootClock();
  let us = 0;
  for (let s = 0; s < 5 * 3600; s += 600) us = fast.sincePowerUp(raw(s * 1e6), t0);
  assert.equal(us, (5 * 3600 - 600) * 1e6);
});

const frame = (number: number, tUs: number): PosedFrame =>
  ({ number, tUs, periodUs: 100000, x: 0, y: 0, yaw: 0, pitch: 0, turn: 0, still: true, poseKnown: true, beyondMm: 2550, zones: [] });

test("messages numbered; each frame filed under its move", () => {
  const b = new Boot("0badf00d", loadMounting());
  assert.equal(b.stamp().msg, 1);
  const start = b.moveStart({ move: 1, command: 9, kind: "turn", param: 0.5, tUs: raw(29e6), preMs: 200, postMs: 200, refused: false });
  assert.equal(start.msg, 2);
  close(start.tS, 29);
  assert.equal(b.frame("compact", frame(5, raw(29.1e6))).move, 1);
  b.moveEnd({ move: 1, end: "ok", tUs: raw(31e6), framesSent: 3, framesDropped: 0, firstFrame: 5, lastFrame: 7, moved: 0, rotated: 0.5, imuSent: 0, imuDropped: 0 });
  b.moveStart({ move: 2, command: 10, kind: "drive", param: 0.5, tUs: raw(31.5e6), preMs: 200, postMs: 200, refused: false });
  const late = b.frame("compact", frame(7, raw(30.9e6))); // move 1's, after its end and move 2's start
  assert.equal(late.move, 1);
  close(late.tS, 30.9);
  assert.equal(b.frame("compact", frame(8, raw(31.6e6))).move, 2);
  b.moveEnd({ move: 2, end: "stopped", tUs: raw(32e6), framesSent: 0, framesDropped: 4, firstFrame: 0, lastFrame: 0, moved: 0, rotated: 0, imuSent: 0, imuDropped: 0 });
  assert.equal(b.frame("compact", frame(20, raw(40e6))).move, null); // no move: none (never sent)
  assert.deepEqual(b.state.ended, [{ move: 1, first: 5, last: 7 }]);
  assert.equal(b.state.messages, 9);
});

test("a refused move during a running one: the running one stays open, its frames before and after are its", () => {
  const b = new Boot("0badf00d", loadMounting());
  b.moveStart({ move: 5, command: 1, kind: "watch", param: 60, tUs: raw(40e6), preMs: 200, postMs: 200, refused: false });
  assert.equal(b.frame("compact", frame(100, raw(41e6))).move, 5);
  // A blocked drive while watching: numbered 6, its start and end at once, no frames.
  b.moveStart({ move: 6, command: 2, kind: "drive", param: 0.5, tUs: raw(42e6), preMs: 200, postMs: 200, refused: true });
  b.moveEnd({ move: 6, end: "obstacle", tUs: raw(42e6), framesSent: 0, framesDropped: 0, firstFrame: 0, lastFrame: 0, moved: 0, rotated: 0, imuSent: 0, imuDropped: 0 });
  assert.equal(b.frame("compact", frame(101, raw(43e6))).move, 5);
  assert.equal(b.state.openMove, 5);
  b.moveEnd({ move: 5, end: "ok", tUs: raw(100e6), framesSent: 3, framesDropped: 0, firstFrame: 100, lastFrame: 102, moved: 0, rotated: 0, imuSent: 0, imuDropped: 0 });
  assert.equal(b.frame("compact", frame(102, raw(99e6))).move, 5); // after its end: by number
  assert.equal(b.state.openMove, null);
});

test("a refused move as the boot's very first: no open move, before or after", () => {
  const b = new Boot("0badf00d", loadMounting());
  b.moveStart({ move: 1, command: 1, kind: "drive", param: 0.5, tUs: raw(31e6), preMs: 200, postMs: 200, refused: true });
  assert.equal(b.state.openMove, null);
  b.moveEnd({ move: 1, end: "obstacle", tUs: raw(31e6), framesSent: 0, framesDropped: 0, firstFrame: 0, lastFrame: 0, moved: 0, rotated: 0, imuSent: 0, imuDropped: 0 });
  assert.equal(b.state.openMove, null);
  assert.deepEqual(b.state.ended, []); // it sent no frames
  b.moveStart({ move: 2, command: 2, kind: "turn", param: 1, tUs: raw(32e6), preMs: 200, postMs: 200, refused: false });
  assert.equal(b.frame("compact", frame(3, raw(32.1e6))).move, 2);
});

// n records of move from PicoA's time t0 (raw), 9615 µs apart.
const batch = (move: number, t0: number, n: number): ImuBatch =>
  ({ move, records: Array.from({ length: n }, (_, i) => ({ tUs: (t0 + i * 9615) % WRAP_US, accel: [i, 0, 16384], gyro: [0, 0, -i] })) });

test("an imu batch: its records on the boot's time across the wrap, under the move it names however late", () => {
  const b = new Boot("0badf00d", loadMounting());
  b.moveStart({ move: 1, command: 9, kind: "turn", param: 0.5, tUs: raw(29.9e6), preMs: 200, postMs: 200, refused: false });
  const row = b.imu(batch(1, raw(29_950_000), 10)); // 29.95 s to 30.04 s: across the wrap at 30 s
  assert.equal(row.move, 1);
  assert.equal(row.records.length, 10);
  row.records.forEach((r, i) => { close(r.tS, 29.95 + i * 0.009615); assert.deepEqual(r.accel, [i, 0, 16384]); });
  assert.ok(row.records[9].tUs < row.records[0].tUs); // PicoA's clock wrapped in between
  b.moveEnd({ move: 1, end: "ok", tUs: raw(31e6), framesSent: 0, framesDropped: 0, firstFrame: 0, lastFrame: 0, moved: 0, rotated: 0.5,
              imuSent: 2, imuDropped: 0 });
  b.moveStart({ move: 2, command: 10, kind: "drive", param: 0.5, tUs: raw(31.5e6), preMs: 200, postMs: 200, refused: false });
  // Move 1's last batch after its end and move 2's start: still move 1's (it says so).
  const late = b.imu(batch(1, raw(30.9e6), 4));
  assert.equal(late.move, 1);
  close(late.records[3].tS, 30.9 + 3 * 0.009615);
  assert.equal(b.imu(batch(2, raw(31.6e6), 10)).move, 2);
  assert.equal(b.state.messages, 6);
});
