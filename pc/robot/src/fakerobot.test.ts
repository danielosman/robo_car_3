// The fake robot end to end with the server's side: fakerobot.ts speaking the wire
// to link.ts, commands run by the API's runner (api/runner.ts) as the page and the
// CLI run them: hello, the first snapshot when it connects, a move answered with how
// far it went, a snapshot answered once it is stored, clear_map with "map cleared",
// a refused command, the console lines and status; a fake robot of another wire
// version refused; a robot that connects again; and a stored boot replayed into a
// second server's own database (ingest.ts, as main.ts stores): the same moves,
// frames, zones, imu records, statuses and snapshots come out, the old database untouched, and
// the fake's own moves get a start and an end. The snapshots are the firmware's
// (build/test_snapshot.snap, from ./run_tests.sh). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { RobotLink, type DoneRecord, type LivePose } from "./link.ts";
import { FakeRobot, type FakeSnapshot } from "./fakerobot.ts";
import { SnapshotAssembler, parseSnapshot } from "./snapshot.ts";
import { CommandRunner, StoredSnapshots } from "./api/runner.ts";
import { parseAction, parseSteer, type Command } from "./api/commands.ts";
import { WIRE_VERSION } from "./wire.ts";

const FIRMWARE = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../build/test_snapshot.snap");
const skip = !existsSync(FIRMWARE) && "run ./run_tests.sh first";

const until = async (ok: () => boolean, ms = 5000) => {
  for (let i = 0; i < ms / 10 && !ok(); i++) await new Promise(r => setTimeout(r, 10));
  assert.ok(ok());
};
const steer = (text: string) => (parseSteer(text) as { commands: Command[] }).commands[0];
const action = (name: string, value = "") => (parseAction(name, value) as { commands: Command[] }).commands[0];

async function setUp() {
  const snapshots: FakeSnapshot[] = new SnapshotAssembler().push(readFileSync(FIRMWARE)).map(bytes => {
    const s = parseSnapshot(bytes);
    return { bytes, seq: s.seq, pose: s.pose };
  });
  const link = new RobotLink({ robotPort: 0, announceTo: ["127.0.0.1"], announcePort: 9, heartbeatEveryMs: 100 });
  const port = await link.start();
  const stored = new StoredSnapshots(), notes: string[] = [], text: string[] = [], poses: LivePose[] = [], seqs: number[] = [];
  let cleared = 0;
  link.on("snapshot", (b: Buffer) => { const s = parseSnapshot(b); seqs.push(s.seq); stored.add(s.bootId, s.seq); });
  link.on("note", n => notes.push(n));
  link.on("text", t => text.push(t));
  link.on("pose", p => poses.push(p));
  link.on("mapCleared", () => cleared++);
  const runner = new CommandRunner(link, stored);
  return { link, port, snapshots, runner, notes, text, poses, seqs, cleared: () => cleared };
}

test("the fake robot end to end: hello, commands, answers, snapshots, map cleared", { skip }, async () => {
  const t = await setUp();
  let robot = new FakeRobot({ port: t.port, snapshots: t.snapshots });
  try {
    await until(() => t.link.status.state === "connected");
    await until(() => t.seqs.length === 1); // a server connected: a snapshot
    assert.equal(t.seqs[0], t.snapshots[0].seq);
    await until(() => t.poses.length > 0);
    assert.equal(t.poses.at(-1)!.mode, "manual");
    assert.ok(t.text.join("").includes("Server: connected\n"));

    const moved = await t.runner.run(steer("d5"));
    assert.equal(moved.result, "ok");
    assert.equal(moved.moved, 5);
    const snap = await t.runner.run(action("snap"));
    assert.deepEqual([snap.result, snap.snapshot], ["ok", t.snapshots[1 % t.snapshots.length].seq]);
    assert.equal((await t.runner.run(action("clear_map"))).result, "ok");
    await until(() => t.cleared() === 1);
    const refused = new Promise<DoneRecord>(resolve => t.link.once("done", resolve));
    assert.ok(t.link.sendCommand(99, "fly 3"));
    assert.equal((await refused).end, "refused");
    assert.ok(t.text.join("").includes('Command refused: "fly 3"\n'));

    // It connects again: hello, a snapshot, and commands work.
    robot.stop();
    await until(() => t.link.status.state === "lost");
    robot = new FakeRobot({ port: t.port, snapshots: t.snapshots });
    await until(() => t.link.status.state === "connected" && t.seqs.length === 3);
    assert.equal((await t.runner.run(action("mode", "idle"))).result, "ok");
  } finally { robot.stop(); t.link.stop(); }
});

test("a fake robot of another wire version is refused", async () => {
  const link = new RobotLink({ robotPort: 0, announceTo: ["127.0.0.1"], announcePort: 9 });
  const port = await link.start(), notes: string[] = [];
  link.on("note", n => notes.push(n));
  const robot = new FakeRobot({ port, snapshots: [], version: WIRE_VERSION + 1 });
  try {
    await robot.closed;
    assert.ok(notes.some(n => n.includes(`it speaks wire version ${WIRE_VERSION + 1}, this server ${WIRE_VERSION}`)));
    assert.notEqual(link.status.state, "connected");
  } finally { robot.stop(); link.stop(); }
});

test("a stored boot replayed to a test server: the same moves, frames, zones, imu records, statuses and snapshots in its own database", { skip }, async () => {
  const { mkdtempSync, rmSync, statSync } = await import("node:fs");
  const os = await import("node:os");
  const { storeWithSnapshots } = await import("./test_store.ts");
  const { Store } = await import("./store.ts");
  const { ingest } = await import("./ingest.ts");
  const { BootFolders } = await import("./boot_folders.ts");
  const { snapshotsOf } = await import("./fakerobot.ts");
  const from = mkdtempSync(path.join(os.tmpdir(), "replay-from-")), to = mkdtempSync(path.join(os.tmpdir(), "replay-to-"));
  const link = new RobotLink({ robotPort: 0, announceTo: ["127.0.0.1"], announcePort: 9, heartbeatEveryMs: 100 });
  let robot: FakeRobot | null = null;
  try {
    // The stored boot: its snapshots, then a move with two frames, statuses around it.
    const { store: old, boot } = await storeWithSnapshots(from);
    const zones = Array.from({ length: 64 }, (_, k) => ({ distanceMm: 400 + 10 * k, signal: k, sigmaMm: 4, kind: "sure" as const, targets: 1 }));
    const pose = (x: number) => ({ x, y: 0, yaw: 0, mode: "manual", action: "drive", rssi: -50, at: 0 });
    await old.status(boot, boot.status(pose(1.25)));
    await old.moveStart(boot, boot.moveStart({ move: 1, command: 3, kind: "drive", param: 0.25, tUs: 0xfffff000, preMs: 200, postMs: 200, refused: false }));
    const { packTof, unpackTof } = await import("./tof_format.ts");
    for (const n of [7, 8]) { // as the robot sends them: packed (yaw in 0.01°)
      const f = unpackTof("compact", packTof("compact", { number: n, tUs: (0xfffff000 + n * 1000), periodUs: 100000, x: 1.25 + n / 100,
        y: -0.5, yaw: 0.75, pitch: 0, turn: 0, still: false, poseKnown: true, beyondMm: 2550, zones }));
      await old.frame(boot, boot.frame("compact", 1, f.tUs, f));
    }
    // Its imu records: a batch of 10 and the last 3 (evenly spaced, as the wire carries them).
    const imu = (from: number, n: number) => ({ move: 1, records: Array.from({ length: n }, (_, i) => ({
      tUs: (0xfffff000 + (from + i) * 9615), accel: [from + i, -7, 16384] as [number, number, number],
      gyro: [32767, -32768, from] as [number, number, number] })) });
    await old.imu(boot, boot.imu(imu(0, 10)));
    await old.imu(boot, boot.imu(imu(10, 3)));
    await old.moveEnd(boot, boot.moveEnd({ move: 1, end: "ok", tUs: 0x5000, framesSent: 2, framesDropped: 0, moved: 0.25, rotated: 0, imuSent: 2, imuDropped: 0 })); // f32-exact, as the wire carries them
    await old.status(boot, boot.status(pose(1.55)));
    await old.flush();
    const messages = await old.replay(boot.id);
    const oldFile = path.join(from, "test.duckdb"), oldTime = statSync(oldFile).mtimeMs;

    // The test server: its own database; the fake robot replays at once.
    const store = new Store(path.join(to, "robot2.duckdb")), notes: string[] = [];
    const stored = ingest(link, store, { recordingsDir: to, bootFolders: new BootFolders(to), note: n => notes.push(n) });
    const port = await link.start();
    robot = new FakeRobot({ port, bootId: boot.id, snapshots: snapshotsOf(messages), replay: messages, speed: Infinity });
    await robot.replayed;
    await until(() => link.status.state === "connected");
    await new Promise(r => setTimeout(r, 300)); // the last bytes through TCP
    await stored.idle();
    assert.deepEqual(notes.filter(n => n.includes("NOT")), []);
    const count = async (s: InstanceType<typeof Store>, table: string) => Number((await s.query(`SELECT count(*) AS n FROM ${table}`))[0].n);
    for (const table of ["moves", "frames", "zones", "imu", "snapshots", "snapshot_zones"]) assert.equal(await count(store, table), await count(old, table), table);
    assert.equal(await count(store, "imu"), 13);
    const ia = (await old.moveImu(boot.id, 1))!, ib = (await store.moveImu(boot.id, 1))!;
    const raw = (rs: typeof ia.records) => rs.map(r => ({ tUs: r.tUs, accel: r.accel, gyro: r.gyro }));
    assert.deepEqual(raw(ib.records), raw(ia.records));
    assert.deepEqual(ib.imu, { records: 13, batches: 2, missing: 0 });
    assert.ok(await count(store, "status") >= 2);
    const a = (await old.moveFrames(boot.id, 1))!, b = (await store.moveFrames(boot.id, 1))!;
    assert.deepEqual(b.frames.map(f => f.frame), a.frames.map(f => f.frame));
    assert.deepEqual({ ...b.move, startTS: 0, endTS: 0 }, { ...a.move, startTS: 0, endTS: 0 });
    assert.deepEqual(await store.mountingOf(boot.id), boot.mounting);
    // Then it takes commands: a move of its own, with its start and end (number 2).
    const runner = new CommandRunner(link, new StoredSnapshots());
    assert.equal((await runner.run(steer("r10"))).result, "ok");
    await new Promise(r => setTimeout(r, 100));
    await stored.idle();
    const own = (await store.moveFrames(boot.id, 2))!;
    assert.deepEqual([own.move.kind, own.move.end, own.move.preMs], ["turn", "ok", 200]);
    assert.equal(statSync(oldFile).mtimeMs, oldTime); // the old one only read
  } finally {
    robot?.stop();
    link.stop();
    rmSync(from, { recursive: true, force: true });
    rmSync(to, { recursive: true, force: true });
  }
});
