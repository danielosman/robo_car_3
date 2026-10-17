// Tests for routes.ts, the API over HTTP, end to end below the socket: steer and the
// actions stream one JSON line per command as each ends; a command the API can't take
// is refused with nothing queued; stop, no-wait, auto_snap; check on the newest
// snapshot's map; the reads on the firmware test's snapshots in a fresh database.
// Run with `npm test`.
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
import type { Chat } from "../objects/model.ts";

const FIRMWARE = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../../build/test_snapshot.snap");
const skip = !existsSync(FIRMWARE) && "run ./run_tests.sh first";

// A robot that answers every line at once: ok, at (43, -4) cm, having moved 31 cm; a
// snapshot as number 5.
class EchoRobot extends EventEmitter {
  lines: string[] = [];
  end = "ok";
  sendLine(line: string): boolean {
    this.lines.push(line);
    const n = Number(/#(\d+)$/.exec(line)![1]), snap = line.startsWith("snapshot") ? 5 : null;
    setTimeout(() => this.emit("done", { n, end: line.startsWith("drive") ? this.end : "ok", pose: { x: 0.43, y: -0.04, yaw: 0 },
                                         movedM: 0.31, rotatedDeg: 0, goalM: null, snapshot: snap }), 5);
    return true;
  }
}

async function setUp(chat?: Chat) {
  const dir = mkdtempSync(path.join(os.tmpdir(), "api-"));
  const file = path.join(dir, "snapshots", "raw", "test.snap");
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
  const robot = new EchoRobot(), snapshots = new StoredSnapshots(), log = new Log(null, false);
  snapshots.add("x", 5);
  const live = new LiveRobot(store, err => { throw err; });
  live.setStatus({ state: "connected", ip: "10.0.0.7", since: 0, rssi: -58, quietSince: null });
  live.setPose(parsePose("pose 43.0 -4.0 30.0 manual - -58 0 0.20 0.10 1.05 4.0")!);
  const queue = new CommandQueue(new CommandRunner(robot, snapshots), (a, s) => log.answer(a, s));
  const app = createApi({ store, live, queue, log, snapshots, config: { autoSnap: true }, recordingsDir: dir, chat, modelName: "fake" });
  const post = async (command: string, body = "", query = "") => {
    const r = await app.request(`/api/robot/${command}${query}`, { method: "POST", body });
    return { status: r.status, text: await r.text() };
  };
  const done = async () => { await live.stop(); rmSync(dir, { recursive: true, force: true }); };
  return { post, robot, log, dir, done };
}

const AT = '"pose":{"x":43,"y":-4,"yaw":0}';

test("steer and the actions: one JSON line per command as it ends; refused; stop; no-wait; auto_snap", { skip }, async () => {
  const { post, robot, log, done } = await setUp();
  try {
    let r = await post("steer", "r30d50");
    assert.equal(r.status, 200);
    assert.equal(r.text, [
      `{"command":"steer r30","result":"ok",${AT},"moved":31,"rotated":0}`,
      `{"command":"snap","result":"ok",${AT},"snapshot":5}`,
      `{"command":"steer d50","result":"ok",${AT},"moved":31,"rotated":0}`,
      `{"command":"snap","result":"ok",${AT},"snapshot":5}`, ""].join("\n"));
    assert.deepEqual(robot.lines, ["rotate 30 #1", "snapshot #2", "drive 50 #3", "snapshot #4"]);
    robot.end = "obstacle";
    r = await post("steer", "d50 r30", "?from=page");
    assert.equal(r.text, [
      `{"command":"steer d50","result":"obstacle",${AT},"moved":31,"rotated":0}`,
      `{"command":"snap","result":"ok",${AT},"snapshot":5}`,
      '{"command":"steer r30","result":"dropped","why":"steer d50 ended: obstacle"}',
      '{"command":"snap","result":"dropped","why":"steer d50 ended: obstacle"}', ""].join("\n"));
    assert.ok(log.tail(5).includes(`[page] {"command":"steer d50","result":"obstacle",${AT},"moved":31,"rotated":0}`));
    r = await post("steer", "d500");
    assert.deepEqual(r, { status: 400, text: '{"command":"steer d500","result":"failed","why":"d500: d takes -300 to 300"}\n' });
    assert.equal(robot.lines.length, 6);
    assert.equal((await post("snap")).text, `{"command":"snap","result":"ok",${AT},"snapshot":5}\n`);
    assert.equal((await post("recording", "on")).text, '{"command":"recording on","result":"ok"}\n');
    assert.equal((await post("mode", "fast")).status, 400);
    assert.equal((await post("stop")).text, `{"command":"stop","result":"ok",${AT}}\n`);
    assert.equal((await post("clear_map", "", "?nowait")).text, '{"queued":["clear_map"]}\n');
    assert.equal((await post("config", "auto_snap off")).text, '{\n  "auto_snap": false\n}\n');
    robot.end = "ok";
    assert.equal((await post("steer", "d10", "?nowait")).text, '{"queued":["steer d10"]}\n');
  } finally { await done(); }
});

test("check on the newest snapshot's map", { skip }, async () => {
  const { post, done } = await setUp();
  try {
    const lines = (await post("check", "r90 d20")).text.trim().split("\n").map(l => JSON.parse(l));
    assert.equal(lines.length, 2);
    assert.equal(lines[0].command, "steer r90");
    assert.equal(lines[0].rotated, 90);
    assert.equal((await post("check", "w5")).status, 400);
  } finally { await done(); }
});

test("reads: status, map, tof, camera, light, features, log; errors; help", { skip }, async () => {
  const { post, dir, done } = await setUp();
  const read = async (command: string, arg = "", query = "") => JSON.parse((await post(command, arg, query)).text);
  try {
    const status = await read("status");
    assert.deepEqual(status, { connected: true, pose: { x: 43, y: -4, yaw: 30 }, mode: "manual", recording: false, wifi: -58,
      running: null, waiting: [], newest_snapshot: 5, safety: { jolt: { x: 0.2, y: 0.1, z: 1.05 }, tilt: 4 } });
    const map = await read("map", "1");
    assert.equal(map.snapshot, 1);
    assert.equal(map.rows.length, 60);
    assert.ok(map.rows.join("").includes("#"));
    assert.equal(map.legend["#"], "blocked (3-13 cm high)");
    const png = (await read("map", "1", "?png")).file;
    assert.equal(png, path.join(dir, "map", png.split(path.sep).at(-2), "1.png"));
    assert.ok(existsSync(png));
    const tofText = (await post("tof", "1")).text;
    assert.match(tofText, /"distance": \[\n {4}\[/); // a matrix, one row a line
    const tof = JSON.parse(tofText);
    assert.equal(tof.distance.length, 8);
    const total = tof.targets.flat().reduce((a: number, b: number) => a + b, 0);
    assert.equal(total, Array.from({ length: 64 }, (_, z) => z % 3).reduce((a, b) => a + b, 0)); // the test frame: zone z has z % 3 targets
    assert.ok(existsSync((await read("camera", "1")).file));
    assert.deepEqual(await post("light", "1"), { status: 404, text: '{"error":"snapshot 1 has no colour reading"}\n' }); // the test's bytes aren't one
    assert.equal((await read("features", "1")).features.length, 2);
    assert.ok(Array.isArray((await read("log", "5")).lines));
    assert.equal((await post("map", "9")).status, 404);
    assert.equal((await post("map", "x")).status, 400);
    assert.equal((await post("nope")).status, 404);
  } finally { await done(); }
});

// A vision model that reports one chair the first time, a door the next.
function fakeModel(): Chat {
  const answers = [[{ label: "chair", bbox: [300, 400, 617, 600] }], [{ label: "door", bbox: [0, 300, 600, 700] }]];
  return async () => ({ reasoning: "looked", tokens: 1, message: { role: "assistant", content: "", tool_calls: [{ id: "c1", type: "function",
    function: { name: "report_objects", arguments: JSON.stringify({ objects: answers.shift() }) } }] } });
}

test("detect and objects: placed, saved, read back, replaced on a second run", { skip }, async () => {
  const { post, done } = await setUp(fakeModel());
  try {
    const r = await post("detect", "1");
    assert.equal(r.status, 200, r.text);
    const d = JSON.parse(r.text);
    assert.equal(d.snapshot, 1);
    assert.equal(d.result, "ok");
    assert.equal(d.objects.length, 1);
    assert.deepEqual(d.objects[0].bbox, [300, 400, 617, 600]);
    assert.equal(d.objects[0].floor, "floor");
    assert.equal(d.objects[0].shape.kind, "rect");
    assert.ok(d.objects[0].distance > 50 && d.objects[0].distance < 200, `${d.objects[0].distance}`); // the row 1 m ahead when level; the test pose has some pitch
    const o = JSON.parse((await post("objects", "1")).text);
    assert.equal(o.model, "fake");
    assert.equal(o.thinking, true);
    assert.deepEqual(o.objects, d.objects);
    const t = JSON.parse((await post("objects", "1", "?transcript")).text);
    assert.equal(t.transcript[0].role, "system");
    assert.equal(t.transcript.at(-2).reasoning, "looked");
    const again = JSON.parse((await post("detect", "1", "?nothinking")).text);
    assert.deepEqual(again.objects.map((x: { label: string }) => x.label), ["door"]);
    const after = JSON.parse((await post("objects", "1")).text);
    assert.equal(after.thinking, false);
    assert.equal(after.objects.length, 1);
    assert.equal((await post("objects", "2")).status, 404);
  } finally { await done(); }
});

test("detect without a model set up: 503", { skip }, async () => {
  const { post, done } = await setUp();
  try { assert.equal((await post("detect", "1")).status, 503); } finally { await done(); }
});
