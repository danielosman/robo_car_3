// Tests for routes.ts, the API over HTTP, end to end below the socket: steering
// streams one answer per command as each ends, a command the API can't take is
// refused with nothing queued, stop, --no-wait, --check on the newest snapshot's map;
// the reads (status, snap, map, tof, camera, log, queue) on the firmware test's
// snapshots in a fresh database; the help. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { SNAP_FILE_MAGIC, SnapshotAssembler } from "../snapshot.ts";
import { Store } from "../store.ts";
import { importSnapshots } from "../import.ts";
import { Log } from "../log.ts";
import { LiveRobot } from "../live.ts";
import { parsePose } from "../robot.ts";
import { CommandRunner, StoredSnapshots } from "./runner.ts";
import { CommandQueue } from "./queue.ts";
import { createApi } from "./routes.ts";

const FIRMWARE = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../../build/test_snapshot.snap");
const skip = !existsSync(FIRMWARE) && "run ./run_tests.sh first";

// A robot that answers every line at once: ok, at (43, -4) cm; a snapshot as number 5.
class EchoRobot extends EventEmitter {
  lines: string[] = [];
  end = "ok";
  sendLine(line: string): boolean {
    this.lines.push(line);
    const n = Number(/#(\d+)$/.exec(line)![1]), snap = line.startsWith("snapshot") ? 5 : null;
    setTimeout(() => this.emit("done", { n, end: line.startsWith("drive") ? this.end : "ok", pose: { x: 0.43, y: -0.04, yaw: 0 },
                                         movedM: 0.31, goalM: null, snapshot: snap }), 5);
    return true;
  }
}

async function setUp() {
  const dir = mkdtempSync(path.join(os.tmpdir(), "api-"));
  const file = path.join(dir, "snapshots", "test.snap");
  mkdirSync(path.dirname(file), { recursive: true });
  const parts: Buffer[] = [Buffer.from(SNAP_FILE_MAGIC)];
  new SnapshotAssembler().push(readFileSync(FIRMWARE)).forEach((b, i) => {
    const t = Buffer.alloc(8);
    t.writeDoubleLE(Date.UTC(2026, 9, 14, 10, 0, i), 0);
    parts.push(t, b);
  });
  writeFileSync(file, Buffer.concat(parts));
  const store = new Store(path.join(dir, "test.duckdb"));
  await importSnapshots(store, file, dir);
  const robot = new EchoRobot(), stored = new StoredSnapshots(), log = new Log(null, false);
  stored.add("x", 5);
  const live = new LiveRobot(store, err => { throw err; });
  live.setStatus({ state: "connected", ip: "10.0.0.7", since: 0, rssi: -58, quietSince: null });
  live.setPose(parsePose("pose 43.0 -4.0 30.0 manual - -58 0 0.20 0.10 1.05 4.0")!);
  const queue = new CommandQueue(new CommandRunner(robot, stored), a => log.answer(a));
  const app = createApi({ store, live, queue, log, recordingsDir: dir });
  const done = async () => { await live.stop(); rmSync(dir, { recursive: true, force: true }); };
  return { app, robot, log, dir, done };
}

test("steer: answers stream as commands end; errors refused; stop; no-wait", { skip }, async () => {
  const { app, robot, log, done } = await setUp();
  try {
    let r = await app.request("/api/robot/steer", { method: "POST", body: "r30d50" });
    assert.equal(r.status, 200);
    assert.equal(await r.text(), "queued: r30 s d50 s\ndone: r30 ok, at (43, -4) cm, yaw +0°\n" +
      "done: s snapshot 5, at (43, -4) cm, yaw +0°\ndone: d50 ok, at (43, -4) cm, yaw +0°\ndone: s snapshot 5, at (43, -4) cm, yaw +0°\n");
    assert.deepEqual(robot.lines, ["rotate 30 #1", "snapshot #2", "drive 50 #3", "snapshot #4"]);
    robot.end = "obstacle";
    r = await app.request("/api/robot/steer?from=page", { method: "POST", body: "d50 r30" });
    assert.equal(await r.text(), "queued: d50 s r30 s\ndone: d50 obstacle after 31 cm, at (43, -4) cm, yaw +0°\n" +
      "done: s snapshot 5, at (43, -4) cm, yaw +0°\ndropped: r30\ndropped: s\n");
    assert.ok(log.tail(5).includes("[page] done: d50 obstacle after 31 cm, at (43, -4) cm, yaw +0°"));
    r = await app.request("/api/robot/steer", { method: "POST", body: "d500" });
    assert.equal(r.status, 400);
    assert.equal(await r.text(), "error: d500 out of range: drive takes -300 to 300 cm\n");
    assert.equal(robot.lines.length, 6);
    r = await app.request("/api/robot/steer", { method: "POST", body: "x" });
    assert.equal(await r.text(), "done: x ok\n");
    r = await app.request("/api/robot/steer?nowait", { method: "POST", body: "c" });
    assert.equal(await r.text(), "queued: c\n");
  } finally { await done(); }
});

test("--check on the newest snapshot's map", { skip }, async () => {
  const { app, done } = await setUp();
  try {
    const r = await app.request("/api/robot/steer?check", { method: "POST", body: "r90 d20" });
    assert.match(await r.text(), /^check: r90 .*turns to yaw \+\d+°.*\ncheck: d20 (clear|blocked)/);
    const bad = await app.request("/api/robot/steer?check", { method: "POST", body: "s" });
    assert.equal(bad.status, 400);
  } finally { await done(); }
});

test("reads: status, snap, map, tof, camera, log, queue; json; errors", { skip }, async () => {
  const { app, dir, done } = await setUp();
  const get = async (url: string) => { const r = await app.request(url); return { status: r.status, text: await r.text() }; };
  try {
    const status = (await get("/api/robot/status")).text;
    assert.match(status, /^robot connected \(10\.0\.0\.7\), WiFi -58 dBm\nat \(43, -4\) cm, yaw \+30°; mode manual, recording off\n/);
    assert.match(status, /largest jolt x 0\.20 g, y 0\.10 g, z 1\.05 g; largest tilt 4°/);
    const snap = (await get("/api/robot/snap")).text;
    assert.match(snap, /^snapshot 5 \(boot [0-9a-f]{8}\)/);
    assert.match(snap, /\nat \(125, -50\) cm, yaw \+43°\n/);
    assert.match((await get("/api/robot/snap/2")).text, /last action: move, obstacle/);
    assert.equal((await get("/api/robot/snap/9")).status, 404);
    assert.equal((await get("/api/robot/snap/x")).status, 400);
    const map = (await get("/api/robot/map/1")).text;
    assert.match(map, /^snapshot 1\. The map, 10 cm a character/);
    assert.ok(map.includes("#"));
    const png = (await get("/api/robot/map/1?png")).text.trim();
    assert.equal(png, path.join(dir, "maps", png.split(path.sep).at(-2)!, "1.png"));
    assert.ok(existsSync(png));
    const tof = (await get("/api/robot/tof/1")).text.split("\n");
    assert.equal(tof.length, 1 + 8 + 1);
    assert.ok(existsSync((await get("/api/robot/camera/1")).text.trim()));
    const json = JSON.parse((await get("/api/robot/tof/1?json")).text);
    assert.equal(json.seq, 1);
    assert.equal(json.rows.length, 8);
    assert.deepEqual(await get("/api/robot/light/1"), { status: 404, text: "snapshot 1 has no colour reading\n" }); // the test's raw bytes aren't one
    assert.equal((await get("/api/robot/queue")).text, "running: nothing; waiting: nothing\n");
    assert.equal((await get("/api/robot/nope")).status, 404);
    assert.match((await get("/api/robot/help")).text, /robot steer <commands>/);
    assert.match((await get("/api/robot/help/steer")).text, /d {2}drive <-300\.\.300 cm>/);
    assert.match((await get("/api/robot/help/tof")).text, /^robot tof \[n\]/);
    assert.equal((await get("/api/robot/help/nope")).status, 404);
  } finally { await done(); }
});
