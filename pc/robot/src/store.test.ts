// Tests for store.ts: a zone's world position (hand-computed cases, the boot's
// mounting); a boot's moves, frames and zones written as they arrive and read back
// (robot frames' reading, the fake robot's replay in the order they came); a frame
// that comes after its move's end filed under that move; each boot keeps its own copy
// of the mounting, also when the server starts again (and the boot's clock and moves
// go on); the imu records: written per batch, read back (robot imu, the counts for
// robot frames, the replay as batches), a late batch under the move it names, times
// across the clock's wrap, also with the server restarted; the status rows (no peaks)
// and the path driven; and the old database: the store's
// file is robot2.duckdb, never robot.duckdb, and a store in a temp dir writes only
// its own file there. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, readdirSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { DB_FILE, DB_PATH, Store, tofGeometry, worldPoint } from "./store.ts";
import { loadMounting, type Mounting } from "./mounting.ts";
import { WIRE_VERSION, type ImuBatch } from "./wire.ts";
import type { PosedFrame, Zone } from "./tof_format.ts";
import type { LivePose } from "./link.ts";

const DEG = Math.PI / 180;
const close = (a: number, b: number, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const now = loadMounting();
// A mounting from before the PCB was lifted: the ToF at 7 cm, the camera at 8.8 cm.
const before: Mounting = { ...now, tof: { ...now.tof, z_m: 0.07 }, camera: { ...now.camera, z_m: 0.088 } };

test("world points: the boot's mounting, the robot turned and moved", () => {
  const g = tofGeometry(before), Z = 0.07, still = { x: 0, y: 0, yaw: 0, pitch: 0 };
  assert.deepEqual(g.sensorM, [before.tof.x_m, before.tof.y_m, Z]);
  // Row 4, column 4 (1-8): half a zone up and half a zone left of straight ahead.
  const h = 2.8125 * DEG;
  let [x, y, z] = worldPoint(g, still, 4, 4, 1);
  close(x, before.tof.x_m + Math.cos(h) * Math.cos(h));
  close(y, before.tof.y_m + Math.cos(h) * Math.sin(h));
  close(z, Z + Math.sin(h));
  // The robot at (1, 2), turned 90° left: forward is +y, left is -x.
  [x, y] = worldPoint(g, { x: 1, y: 2, yaw: 90 * DEG, pitch: 0 }, 4, 4, 1);
  close(x, 1 - before.tof.y_m - Math.cos(h) * Math.sin(h));
  close(y, 2 + before.tof.x_m + Math.cos(h) * Math.cos(h));
  // The bottom row (8) is 3.5 zones down; at the distance where that ray meets the floor, z = 0.
  [, , z] = worldPoint(g, still, 8, 4, Z / Math.sin(3.5 * 5.625 * DEG));
  close(z, 0);
  // Nose up by half a zone: row 5's ray (half a zone down) becomes level.
  [, , z] = worldPoint(g, { ...still, pitch: h }, 5, 4, 1);
  close(z, Z);
});

const zones = (mm: number): Zone[] => Array.from({ length: 64 }, (_, k) =>
  ({ distanceMm: k % 9 ? mm + k : 0, signal: k, sigmaMm: 3, kind: k % 9 ? "sure" : "nothing", targets: k % 9 ? 1 : 0 }));
const frame = (number: number, tUs: number, mm = 500): PosedFrame =>
  ({ number, tUs, periodUs: 100000, x: 0.25, y: -0.5, yaw: 0.125, pitch: -0.015, turn: 0.01, still: false, poseKnown: true, beyondMm: 2550, zones: zones(mm) });
const pose = (x: number): LivePose => ({ x, y: 0, yaw: 0, mode: "manual", action: "drive", rssi: -60, at: 0 });
// n imu records of move from PicoA's time t0, 9615 µs apart; counts from k (the int16 limits on the first).
const imu = (move: number, t0: number, n: number, k = 0): ImuBatch => ({ move, records: Array.from({ length: n }, (_, i) => ({
  tUs: (t0 + i * 9615) % 2 ** 32, accel: i === 0 ? [32767, -32768, 16384] : [k + i, -(k + i), 16384], gyro: [i, 0, 0 - k] })) }); // 0 - k: no -0

test("a boot's moves, frames, zones and status: written as they come, read back, replayed in order; late frames under their move", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-store-"));
  try {
    const store = new Store(path.join(dir, DB_FILE));
    const boot = await store.openBoot({ version: WIRE_VERSION, bootId: "0badf00d", build: "test" }, now);
    const t0 = 0xffffff00; // just before PicoA's clock wraps
    await store.status(boot, boot.status(pose(0)));
    void store.moveStart(boot, boot.moveStart({ move: 1, command: 4, kind: "drive", param: 0.5, tUs: t0, preMs: 200, postMs: 300, refused: false }));
    void store.frame(boot, boot.frame("compact", frame(11, (t0 + 50000) % 2 ** 32)));
    void store.imu(boot, boot.imu(imu(1, t0 - 30000, 10)));            // across the wrap (at t0 + 256 µs)
    void store.status(boot, boot.status(pose(0.1)));
    // The move's end overtakes its last frame (control before bulk); the next move starts meanwhile.
    void store.moveEnd(boot, boot.moveEnd({ move: 1, end: "obstacle", tUs: (t0 + 150000) % 2 ** 32, framesSent: 2, framesDropped: 1,
                                            firstFrame: 11, lastFrame: 12, moved: 0.3, rotated: 0.01, imuSent: 2, imuDropped: 0 }));
    void store.moveStart(boot, boot.moveStart({ move: 2, command: 5, kind: "watch", param: 60, tUs: (t0 + 200000) % 2 ** 32, preMs: 200, postMs: 300, refused: false }));
    void store.frame(boot, boot.frame("compact", frame(12, (t0 + 150000) % 2 ** 32)));
    void store.imu(boot, boot.imu(imu(1, (t0 + 66150) % 2 ** 32, 4, 100))); // move 1's last batch, late: after move 2's start
    void store.imu(boot, boot.imu(imu(2, (t0 + 210000) % 2 ** 32, 10, 200)));
    await store.frame(boot, boot.frame("compact", frame(13, (t0 + 250000) % 2 ** 32, 2600)));
    await store.flush();

    const one = (await store.moveFrames("0badf00d", 1))!;
    assert.deepEqual(one.move, { move: 1, command: 4, kind: "drive", param: 0.5, preMs: 200, postMs: 300, refused: false, startTS: one.move.startTS,
      endTS: one.move.endTS, end: "obstacle", framesSent: 2, framesDropped: 1, firstFrame: 11, lastFrame: 12, moved: 0.3, rotated: 0.01,
      imuSent: 2, imuDropped: 0 });
    assert.deepEqual(one.imu, { records: 14, batches: 2, missing: 0 }); // its late batch too, right after the first
    close(one.move.endTS! - one.move.startTS!, 0.15, 1e-6); // across the wrap
    assert.deepEqual(one.frames.map(f => f.frame.number), [11, 12]); // 12 came after the end: still move 1's
    const f11 = one.frames[0];
    assert.deepEqual(f11.frame, frame(11, (t0 + 50000) % 2 ** 32)); // every field back as it went in
    close(f11.tS - one.move.startTS!, 0.05, 1e-6);
    const two = (await store.moveFrames("0badf00d", 2))!;
    assert.deepEqual(two.frames.map(f => f.frame.number), [13]);
    assert.equal(two.move.end, null); // not ended yet
    assert.equal(await store.moveFrames("0badf00d", 3), null);
    // robot imu's reading: every record back, oldest first, on the boot's time across the wrap.
    const oneImu = (await store.moveImu("0badf00d", 1))!;
    assert.equal(oneImu.frames, 2);
    assert.equal(oneImu.records.length, 14);
    const sent = [...imu(1, t0 - 30000, 10).records, ...imu(1, (t0 + 66150) % 2 ** 32, 4, 100).records];
    assert.deepEqual(oneImu.records.map(r => ({ tUs: r.tUs, accel: r.accel, gyro: r.gyro })), sent);
    oneImu.records.forEach((r, i) => close(r.tS - one.move.startTS!, -0.03 + i * 0.009615, 2e-6)); // even 9.6 ms steps, through the wrap
    assert.ok(oneImu.records[9].tUs < oneImu.records[0].tUs); // the raw times wrapped
    assert.deepEqual((await store.moveImu("0badf00d", 2))!.imu, { records: 10, batches: 1, missing: 0 });
    assert.equal(await store.moveImu("0badf00d", 3), null);
    assert.deepEqual(await store.query("SELECT ax, ay, az, gx, gy, gz FROM imu WHERE move = 1 AND n = 0 ORDER BY msg LIMIT 1"),
      [{ ax: 32767, ay: -32768, az: 16384, gx: 0, gy: 0, gz: 0 }]);
    const z = await store.query("SELECT row, col, distance_mm, kind, signal, targets FROM zones WHERE number = 13 AND row = 8 AND col >= 7 ORDER BY col");
    assert.deepEqual(z, [{ row: 8, col: 7, distance_mm: 2662, kind: "sure", signal: 62, targets: 1 },
                         { row: 8, col: 8, distance_mm: 0, kind: "nothing", signal: 63, targets: 0 }]);
    assert.deepEqual(await store.query("SELECT count(*) AS n FROM zones"), [{ n: "192" }]);

    // The replay: every message in the order it came, numbered from 1.
    const replay = await store.replay("0badf00d");
    assert.deepEqual(replay.map(m => m.type),
      ["status", "move_start", "frame", "imu", "status", "move_end", "move_start", "frame", "imu", "imu", "frame"]);
    assert.deepEqual(replay.map(m => m.msg), [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    const end = replay[5];
    assert.ok(end.type === "move_end" && end.end.lastFrame === 12 && end.end.end === "obstacle" && end.end.imuSent === 2);
    const st = replay[4];
    assert.ok(st.type === "status" && st.pose.x === 0.1 && st.pose.mode === "manual");
    const late = replay[8];
    assert.ok(late.type === "imu");
    assert.deepEqual(late.batch, imu(1, (t0 + 66150) % 2 ** 32, 4, 100)); // a batch back as it came
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test("each boot its own mounting; a restarted server goes on with the boot's clock and moves", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-boots-"));
  try {
    const file = path.join(dir, DB_FILE);
    let store = new Store(file);
    const a = await store.openBoot({ version: WIRE_VERSION, bootId: "aaaaaaaa", build: "old" }, before);
    const b = await store.openBoot({ version: WIRE_VERSION, bootId: "bbbbbbbb", build: "new" }, now);
    assert.equal(a.mounting.tof.z_m, 0.07);
    assert.equal(b.mounting.tof.z_m, now.tof.z_m);
    await store.moveStart(a, a.moveStart({ move: 7, command: 1, kind: "turn", param: 1, tUs: 0xfffff000, preMs: 200, postMs: 200, refused: false }));
    await store.moveEnd(a, a.moveEnd({ move: 7, end: "ok", tUs: 0x1000, framesSent: 3, framesDropped: 0, firstFrame: 100, lastFrame: 102, moved: 0, rotated: 1, imuSent: 2, imuDropped: 0 }));
    await store.flush();
    // The server starts again; boot a connects again: its own mounting, not the file's now.
    store = new Store(file);
    const again = await store.openBoot({ version: WIRE_VERSION, bootId: "aaaaaaaa", build: "old" }, now);
    assert.equal(again.mounting.tof.z_m, 0.07);
    assert.deepEqual(await store.mountingOf("aaaaaaaa"), before);
    assert.equal(await store.mountingOf("cccccccc"), null);
    // Its clock goes on (0x2000 is 4 ms after the last time it saw), its messages are numbered on,
    // and a late frame of move 7 is still filed under it.
    const s = again.moveStart({ move: 8, command: 2, kind: "drive", param: 0.1, tUs: 0x2000, preMs: 200, postMs: 200, refused: false });
    assert.equal(s.msg, 3);
    close(s.tS, (0x2000 + 30e6) / 1e6, 1e-9);
    assert.equal(again.frame("compact", frame(101, 0x1800)).move, 7);
    assert.equal(again.frame("compact", frame(103, 0x2800)).move, 8);
    await store.setMounting("aaaaaaaa", now);
    assert.equal((await store.mountingOf("aaaaaaaa"))!.tof.z_m, now.tof.z_m);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test("status rows and the path driven up to a snapshot", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-path-"));
  try {
    const store = new Store(path.join(dir, DB_FILE));
    const boot = await store.openBoot({ version: WIRE_VERSION, bootId: "0000beef", build: "test" }, now);
    for (const x of [0, 0.1, 0.2]) void store.status(boot, boot.status(pose(x)));
    await store.flush();
    const columns = await store.query("SELECT column_name AS c FROM information_schema.columns WHERE table_name = 'status' ORDER BY ordinal_position");
    assert.deepEqual(columns.map(r => r.c), ["boot_id", "msg", "arrived_at", "x", "y", "yaw", "mode", "action", "rssi"]); // no peaks
    assert.deepEqual(await store.query("SELECT msg, x, rssi FROM status ORDER BY msg"),
      [{ msg: 1, x: 0, rssi: -60 }, { msg: 2, x: 0.1, rssi: -60 }, { msg: 3, x: 0.2, rssi: -60 }]);
    assert.deepEqual(await store.path("0000beef", 1), []); // no such snapshot: nothing
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test("the old database is never opened: the store's own file, only it in a temp dir", async () => {
  assert.equal(path.basename(DB_PATH), process.env.ROBOT_DB ? path.basename(process.env.ROBOT_DB) : "robot2.duckdb");
  assert.notEqual(path.basename(DB_PATH), "robot.duckdb");
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-new-"));
  try {
    const store = new Store(path.join(dir, DB_FILE));
    await store.query("SELECT 1");
    assert.deepEqual(readdirSync(dir).filter(f => !f.endsWith(".wal")), ["robot2.duckdb"]);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test("a boot's moves across the clock's wrap, the server restarted in the middle: t_s read back grows", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-wrap-"));
  try {
    const file = path.join(dir, DB_FILE), wall = Date.UTC(2026, 10, 7, 20);
    let store = new Store(file);
    const hello = { version: WIRE_VERSION, bootId: "0000aaaa", build: "test" };
    let boot = await store.openBoot(hello, now, wall);
    const raw = (s: number) => (2 ** 32 - 30e6 + s * 1e6) % 2 ** 32; // PicoA's clock s seconds after power-up
    await store.moveStart(boot, boot.moveStart({ move: 1, command: 1, kind: "turn", param: 1, tUs: raw(28), preMs: 200, postMs: 200, refused: false }, wall + 28000));
    await store.frame(boot, boot.frame("compact", frame(1, raw(29.5)), wall + 29500));
    await store.imu(boot, boot.imu(imu(1, raw(29.9), 10), wall + 29950));
    await store.flush();
    // The server stops just before the wrap and starts again after it: the boot is opened from the store.
    store = new Store(file);
    boot = await store.openBoot(hello, now, wall + 40000);
    await store.frame(boot, boot.frame("compact", frame(2, raw(30.5)), wall + 40000));
    await store.imu(boot, boot.imu(imu(1, raw(30.5), 3), wall + 40000));
    await store.moveEnd(boot, boot.moveEnd({ move: 1, end: "ok", tUs: raw(31), framesSent: 2, framesDropped: 0, firstFrame: 1, lastFrame: 2,
                                             moved: 0, rotated: 1, imuSent: 2, imuDropped: 0 }, wall + 41000));
    await store.flush();
    const m = (await store.moveFrames("0000aaaa", 1))!;
    assert.deepEqual(m.frames.map(f => Math.round(f.tS * 10) / 10), [29.5, 30.5]);
    close(m.move.startTS!, 28, 1e-6);
    close(m.move.endTS!, 31, 1e-6);
    assert.ok(raw(30.5) < raw(29.5)); // the raw times wrapped
    const ts = (await store.moveImu("0000aaaa", 1))!.records.map(r => r.tS);
    assert.equal(ts.length, 13);
    close(ts[0], 29.9, 1e-6);
    close(ts[12], 30.5 + 2 * 0.009615, 1e-6);
    assert.ok(ts.every((t, i) => i === 0 || t > ts[i - 1])); // grows through the wrap and the restart
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test("imu records missing: counted from the gaps in the stored times", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-gaps-"));
  try {
    const store = new Store(path.join(dir, DB_FILE));
    const boot = await store.openBoot({ version: WIRE_VERSION, bootId: "0000cafe", build: "test" }, now);
    const t0 = 2 ** 32 - 40_000; // across the wrap
    const at = (move: number, ks: number[], jitterUs = 0) => ({ move, records: ks.map((k, i) =>
      ({ tUs: (t0 + k * 9615 + (i % 2) * jitterUs) % 2 ** 32, accel: [k, 0, 16384] as [number, number, number], gyro: [0, 0, 0] as [number, number, number] })) });
    for (const move of [1, 2, 3]) {
      await store.moveStart(boot, boot.moveStart({ move, command: move, kind: "turn", param: 1, tUs: t0, preMs: 200, postMs: 200, refused: false }));
    }
    // Move 1: a batch of 10, then one record missing at the batch boundary.
    await store.imu(boot, boot.imu(at(1, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])));
    await store.imu(boot, boot.imu(at(1, [11, 12, 13])));
    // Move 2: two missing inside the data, one more between its batches.
    await store.imu(boot, boot.imu(at(2, [0, 1, 2, 5, 6])));
    await store.imu(boot, boot.imu(at(2, [8, 9])));
    // Move 3: steps of 1.4 and 0.6 periods (jitter) are not gaps.
    await store.imu(boot, boot.imu(at(3, [0, 1, 2, 3], 0.4 * 9615)));
    await store.flush();
    const count = async (move: number) => (await store.moveImu("0000cafe", move))!.imu;
    assert.deepEqual(await count(1), { records: 13, batches: 2, missing: 1 });
    assert.deepEqual(await count(2), { records: 7, batches: 2, missing: 3 });
    assert.deepEqual(await count(3), { records: 4, batches: 1, missing: 0 });
    assert.equal((await store.moveFrames("0000cafe", 2))!.imu.missing, 3);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});
