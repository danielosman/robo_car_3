// Tests for ingest.ts end to end: a robot played on localhost sends its messages to
// link.ts, ingest stores them (store.ts) as main.ts does: a refused move during a
// running one; the same boot saying hello again (a new connection, mid-move) goes on
// with its message numbers; frames and imu batches stored under the move they name,
// also ones that come after a refused move or their move's end on the new connection. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import net from "node:net";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { RobotLink } from "./link.ts";
import { Store } from "./store.ts";
import { ingest } from "./ingest.ts";
import { BootFolders } from "./boot_folders.ts";
import { WIRE_VERSION, encodeHello, encodeImuBatch, encodeMoveEnd, encodeMoveStart, encodeTof, type MoveStart } from "./wire.ts";
import type { PosedFrame } from "./tof_format.ts";

const zones = Array.from({ length: 64 }, () => ({ distanceMm: 500, signal: 9, sigmaMm: 3, kind: "sure" as const, targets: 1 }));
const frame = (number: number): PosedFrame => ({ number, robotClockUs: number * 100000, periodUs: 100000, x: 0, y: 0, yaw: 0, pitch: 0, turn: 0,
                                                 still: true, poseKnown: true, beyondMm: 2550, zones });
const start = (move: number, kind: string, refused: boolean): MoveStart =>
  ({ move, command: move, kind, param: 1, tUs: move * 1000, preMs: 200, postMs: 200, refused });
const end = (move: number, how: string, sent: number) =>
  encodeMoveEnd({ move, end: how, tUs: move * 1000 + 500, framesSent: sent, framesDropped: 0, moved: 0, rotated: 0, imuSent: 0, imuDropped: 0 });

// n imu records of move, PicoA's time from t0, 9615 µs apart.
const imu = (move: number, t0: number, n: number) => encodeImuBatch({ move, records: Array.from({ length: n }, (_, i) =>
  ({ tUs: t0 + i * 9615, accel: [i, 0, 16384] as [number, number, number], gyro: [0, 0, i] as [number, number, number] })) });

async function connect(port: number, bootId: string): Promise<net.Socket> {
  const s = net.connect(port, "127.0.0.1");
  await new Promise(r => s.once("connect", r));
  s.write(encodeHello({ version: WIRE_VERSION, bootId, build: "test" }));
  return s;
}

test("refused moves and a second hello of the same boot, stored as main.ts stores them", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "ingest-"));
  const link = new RobotLink({ robotPort: 0, announceTo: ["127.0.0.1"], announcePort: 9, heartbeatEveryMs: 100 });
  const store = new Store(path.join(dir, "robot2.duckdb")), notes: string[] = [];
  const stored = ingest(link, store, { recordingsDir: dir, bootFolders: new BootFolders(dir), note: n => notes.push(n) });
  try {
    const port = await link.start();
    let robot = await connect(port, "1234abcd");
    robot.write(Buffer.concat([encodeMoveStart(start(1, "watch", false)), encodeTof("compact", 1, 1_000_000, frame(10)), imu(1, 1_000_000, 10),
      encodeMoveStart(start(2, "drive", true)), end(2, "obstacle", 0), encodeTof("compact", 1, 1_100_000, frame(11))]));
    await new Promise(r => setTimeout(r, 200));
    // A new connection of the same boot, mid-move: it goes on.
    robot.destroy();
    robot = await connect(port, "1234abcd");
    robot.write(Buffer.concat([encodeTof("compact", 1, 1_200_000, frame(12)), end(1, "ok", 3), imu(1, 1_096_150, 4)])); // its last batch after its end
    await new Promise(r => setTimeout(r, 200));
    robot.destroy();
    await stored.idle();
    assert.deepEqual(notes.filter(n => n.includes("NOT")), []);
    const frames = await store.query("SELECT number, move, msg FROM frames ORDER BY number");
    assert.deepEqual(frames, [{ number: "10", move: 1, msg: 2 }, { number: "11", move: 1, msg: 6 }, { number: "12", move: 1, msg: 7 }]);
    assert.deepEqual(await store.query("SELECT move, msg, count(*) AS n FROM imu GROUP BY move, msg ORDER BY msg"),
      [{ move: 1, msg: 3, n: "10" }, { move: 1, msg: 9, n: "4" }]);
    const moves = await store.query(`SELECT move, kind, refused, "end" FROM moves ORDER BY move`);
    assert.deepEqual(moves, [{ move: 1, kind: "watch", refused: false, end: "ok" }, { move: 2, kind: "drive", refused: true, end: "obstacle" }]);
    assert.deepEqual(await store.query("SELECT messages FROM boots"), [{ messages: 9 }]);
  } finally { link.stop(); rmSync(dir, { recursive: true, force: true }); }
});
