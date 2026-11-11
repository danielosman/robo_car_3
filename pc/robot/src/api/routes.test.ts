// Tests for routes.ts, the API over HTTP, end to end below the socket: steer and the
// actions stream one JSON line per command as each ends; a command the API can't take
// is refused with nothing queued; stop, no-wait, auto_snap, config pre_ms / post_ms to
// the robot; check on the PC's map at the newest snapshot; the reads on the firmware test's
// snapshots in a fresh database (frames on a move stored there too).
// Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { existsSync, mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { noFirmware, snapshotAgain, storeWithSnapshots } from "../test_store.ts";
import { Log } from "../log.ts";
import { LiveRobot } from "../live.ts";
import { livePose } from "../link.ts";
import { CommandRunner, StoredSnapshots } from "./runner.ts";
import { CommandQueue } from "./queue.ts";
import { createApi } from "./routes.ts";
import type { Chat } from "../objects/model.ts";
import { BootFolders } from "../boot_folders.ts";
import { CellMaps } from "../maps/cell_map.ts";

const skip = noFirmware;

// A robot that answers every line at once: ok, at (43, -4) cm, having moved 31 cm; a
// snapshot as number 5.
class EchoRobot extends EventEmitter {
  lines: string[] = [];
  end = "ok";
  printed: (text: string) => void = () => {}; // the robot's console text, into the log
  sendCommand(n_: number, text: string): boolean {
    const line = `${text} #${n_}`;
    this.lines.push(line);
    if (text === "status") this.printed("Link to PicoB: ok\nToF: 10 Hz set, 10.0 frames a second\n");
    if (text === "map") {
      const cells = Buffer.alloc(3 * 60 * 60);
      cells[3600 + 35 * 60 + 30] = (10 << 4) | 2; // L1 occupied 50 cm ahead
      setTimeout(() => this.emit("mapWindow", { originX: -30, originY: -30, cellsX: 60, cellsY: 60, cells }), 3);
    }
    const n = Number(/#(\d+)$/.exec(line)![1]), snap = line.startsWith("snap") ? 5 : null;
    setTimeout(() => this.emit("done", { n, end: line.startsWith("drive") ? this.end : "ok", pose: { x: 0.43, y: -0.04, yaw: 0 },
                                         movedM: 0.31, rotatedDeg: 0, goalM: null, snapshot: snap }), 5);
    return true;
  }
}

async function setUp(chat?: Chat) {
  const directory = mkdtempSync(path.join(os.tmpdir(), "api-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const robot = new EchoRobot(), snapshots = new StoredSnapshots(), log = new Log(null, false);
  snapshots.add("x", 5);
  const live = new LiveRobot();
  live.setStatus({ state: "connected", ip: "10.0.0.7", since: 0, rssi: -58, quietSince: null });
  live.setBoot(boot.id);
  live.setPose(livePose({ type: "status", x: 0.43, y: -0.04, yaw: 30 * Math.PI / 180, mode: "manual", action: "", rssi: -58 }));
  const queue = new CommandQueue(new CommandRunner(robot, snapshots), (a, s) => log.answer(a, s));
  const maps = new CellMaps(store);
  const app = createApi({ store, maps, live, queue, log, snapshots, config: { autoSnap: true }, recordingsDirectory: directory, bootFolders: new BootFolders(directory),
                          chat, modelName: "gemma-fake" });
  const post = async (command: string, body = "", query = "") => {
    const r = await app.request(`/api/robot/${command}${query}`, { method: "POST", body });
    return { status: r.status, text: await r.text() };
  };
  const done = async () => { maps.close(); rmSync(directory, { recursive: true, force: true }); };
  return { post, robot, log, directory, done, store, boot, live };
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
    assert.deepEqual(robot.lines, ["rotate 30 #1", "snap #2", "drive 50 #3", "snap #4"]);
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
    assert.equal((await post("recording", "on")).status, 404); // gone with the recorder
    assert.equal((await post("mode", "fast")).status, 400);
    // The still times go to the robot, queued; config shows them (and auto_snap).
    assert.equal((await post("config", "pre_ms 300")).text, '{"command":"config pre_ms 300","result":"ok"}\n');
    assert.equal(robot.lines.at(-1), "config pre_ms 300 #8");
    assert.equal((await post("config", "post_ms 2001")).status, 400);
    assert.equal((await post("config", "")).text, '{\n  "auto_snap": true,\n  "pre_ms": 300,\n  "post_ms": 200\n}\n');
    assert.equal((await post("stop")).text, `{"command":"stop","result":"ok",${AT}}\n`);
    assert.equal((await post("clear_map", "", "?nowait")).text, '{"queued":["clear_map"]}\n');
    assert.equal((await post("config", "auto_snap off")).text, '{\n  "auto_snap": false,\n  "pre_ms": 300,\n  "post_ms": 200\n}\n');
    robot.end = "ok";
    assert.equal((await post("steer", "d10", "?nowait")).text, '{"queued":["steer d10"]}\n');
  } finally { await done(); }
});

test("report: the robot's status word; its printout in the answer and in the log", { skip }, async () => {
  const { post, robot, log, done } = await setUp();
  try {
    log.text("before\n");
    robot.printed = text => log.text(text);
    const r = await post("report");
    assert.equal(robot.lines.at(-1), "status #1");
    assert.equal(r.text, `{"command":"report","result":"ok","printout":["Link to PicoB: ok","ToF: 10 Hz set, 10.0 frames a second"]}\n`);
    assert.ok(log.tail(5).some(l => l.includes("ToF: 10 Hz set")));
    assert.equal((await post("report", "now")).status, 400);
  } finally { await done(); }
});

test("check on the PC's map at the newest snapshot", { skip }, async () => {
  const { post, done, store, boot, directory } = await setUp();
  try {
    // The newest snapshot (3, stored again) comes after frames of the floor and a wall
    // 50 cm ahead of where it says the robot is: a 1 m drive stops before the wall.
    const [p] = await store.query(`SELECT x, y, yaw FROM snapshots WHERE boot_id = '${boot.id}' AND sequence = 3`);
    const x = Number(p.x), y = Number(p.y), yaw = Number(p.yaw);
    for (let n = 1; n <= 3; n++) {
      const back = (3 - n) * 0.15;
      const zones = Array.from({ length: 64 }, (_, k) => ({
        distanceMm: k < 32 ? 500 + Math.round(back * 1000) : k < 56 ? 370 : 270, signal: 50, sigmaMm: 5,
        kind: k < 32 || k >= 48 ? "sure" as const : "unsure" as const, targets: 1,
      }));
      void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: { number: n, robotClockUs: n * 100000,
        periodUs: 100000, x: x - back * Math.cos(yaw), y: y - back * Math.sin(yaw), yaw, pitch: 0, turn: 0, still: true, poseKnown: true, beyondMm: 2550, zones } }));
    }
    await snapshotAgain(store, boot, directory, 3);
    const [blocked] = (await post("check", "d100")).text.trim().split("\n").map(l => JSON.parse(l));
    assert.equal(blocked.result, "obstacle");
    assert.ok(blocked.moved > 10 && blocked.moved < 35, String(blocked.moved));
    const lines = (await post("check", "r90 d20")).text.trim().split("\n").map(l => JSON.parse(l));
    assert.equal(lines.length, 2);
    assert.equal(lines[0].command, "steer r90");
    assert.equal(lines[0].rotated, 90);
    assert.equal((await post("check", "w5")).status, 400);
  } finally { await done(); }
});

test("reads: status, map, tof, camera, light, features, log; errors; help", { skip }, async () => {
  const { post, directory, done, boot } = await setUp();
  const read = async (command: string, arg = "", query = "") => JSON.parse((await post(command, arg, query)).text);
  try {
    const status = await read("status");
    assert.deepEqual(status, { connected: true, boot: boot.id, pose: { x: 43, y: -4, yaw: 30 }, mode: "manual", moving: null, wifi: -58,
      running: null, waiting: [], newest_snapshot: 5 }); // no safety peaks since step 4c
    // The PC's map: no frames stored here, so nothing known yet but the 35 cm floor seed.
    const map = await read("map", "1");
    assert.deepEqual([map.snapshot, map.map], [1, "pc"]);
    assert.deepEqual([map.window.cells_x, map.window.cells_y, map.window.origin_cell_x], [60, 60, -30]);
    for (const layer of ["G", "L1", "L2"]) assert.equal(map.window[layer].length, 3600);
    assert.ok(map.window.G.some((b: number) => (b & 3) === 2));
    assert.ok(!map.window.L1.some((b: number) => (b & 3) === 2));
    assert.ok(map.format.cell_byte.startsWith("bits 0-1 the state"));
    // The robot's own, asked from it right now: L1 occupied 50 cm ahead; takes no snapshot number.
    assert.equal((await post("map", "1", "?robot")).status, 400);
    const own = await read("map", "", "?robot");
    assert.equal(own.map, "robot");
    assert.ok(own.window.L1.some((b: number) => (b & 3) === 2));
    const png = (await read("map", "1", "?png")).file;
    assert.equal(png, path.join(directory, "map", png.split(path.sep).at(-2), "1.png"));
    assert.ok(existsSync(png));
    assert.ok((await read("map", "", "?png&robot")).file.endsWith("robot.png"));
    // The PC's line maps over the cells, or alone; only with --png, never the robot's.
    const lined = (await read("map", "1", "?png&lines")).file, linesOnly = (await read("map", "1", "?png&lines&nocells")).file;
    assert.ok(lined.endsWith(`${path.sep}1-lines.png`) && existsSync(lined));
    assert.ok(linesOnly.endsWith(`${path.sep}1-lines-only.png`) && existsSync(linesOnly));
    for (const query of ["?lines", "?png&nocells", "?nocells", "?png&lines&robot"]) assert.equal((await post("map", "", query)).status, 400, query);
    const tofText = (await post("tof", "1")).text;
    assert.match(tofText, /"distance": \[\n {4}\[/); // a matrix, one row a line
    const tof = JSON.parse(tofText);
    assert.deepEqual(Object.keys(tof), ["snapshot", "distance", "kind", "signal", "sigma", "targets"]); // one target: no distance_2
    assert.equal(tof.distance.length, 8);
    const total = tof.targets.flat().reduce((a: number, b: number) => a + b, 0);
    assert.equal(total, Array.from({ length: 64 }, (_, z) => z % 3).reduce((a, b) => a + b, 0)); // the test frame: zone z has z % 3 targets
    // Zones without targets: nothing (null); the others sure, 255 cm where beyond 2.54 m.
    tof.kind.flat().forEach((k: string, i: number) => assert.equal(k, tof.targets.flat()[i] ? "sure" : "nothing"));
    assert.ok(tof.distance.flat().includes(null) && tof.distance.flat().includes(255));
    assert.ok(existsSync((await read("camera", "1")).file));
    assert.deepEqual(await post("light", "1"), { status: 404, text: '{"error":"snapshot 1 has no colour reading"}\n' }); // the test's bytes aren't one
    assert.equal((await read("features", "1")).features.length, 2);
    assert.ok(Array.isArray((await read("log", "5")).lines));
    assert.equal((await post("map", "9")).status, 404);
    assert.equal((await post("map", "x")).status, 400);
    assert.equal((await post("nope")).status, 404);
  } finally { await done(); }
});

test("frames and imu: a move's frames, its imu records, as text and as JSON; a boot by its folder name; errors", { skip }, async () => {
  const { post, done, store, boot } = await setUp();
  try {
    const t0 = 5e9; // µs since power-up, past 2^32: nothing wraps
    await store.moveStart(boot, boot.moveStart({ move: 1, command: 7, kind: "rotate", parameter: Math.PI / 6, tUs: t0, preMs: 200, postMs: 200, refused: false }));
    const zones = Array.from({ length: 64 }, (_, k) => ({ distanceMm: k === 0 ? 0 : k === 1 ? 2550 : 300 + 10 * k, signal: k, sigmaMm: 5,
      kind: k === 0 ? "nothing" as const : k === 2 ? "unsure" as const : k === 3 ? "less_sure" as const : "sure" as const, targets: k ? 1 : 0 }));
    for (let n = 1; n <= 2; n++) {
      await store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: t0 + n * 100000, frame: { number: 40 + n, robotClockUs: (t0 + n * 100000) % 2 ** 32, periodUs: 100000, x: 0.5, y: -0.25,
        yaw: 0.1 * n, pitch: -0.01, turn: 0.02, still: n === 1, poseKnown: true, beyondMm: 2550, zones } }));
    }
    // Its imu records: a batch of 10 and its last 3.
    const records = (from: number, n: number) => Array.from({ length: n }, (_, i) =>
      ({ tUs: (t0 - 20000 + (from + i) * 9615), acceleration: [from + i, -1, 16384] as [number, number, number],
         gyro: [-32768, 32767, 0 - (from + i) % 10] as [number, number, number] }));
    await store.imuBatch(boot, boot.imuBatch({ move: 1, records: records(0, 10) }));
    await store.imuBatch(boot, boot.imuBatch({ move: 1, records: records(10, 3) }));
    await store.moveEnd(boot, boot.moveEnd({ move: 1, end: "ok", tUs: t0 + 300000, framesSent: 2, framesDropped: 0, moved: 0.01, rotated: Math.PI / 6, imuSent: 2, imuDropped: 0 }));
    await store.flush();
    const r = await post("frames", `${boot.id} 1`);
    assert.equal(r.status, 200);
    const lines = r.text.split("\n");
    assert.match(lines[0], new RegExp(`^boot ${boot.id} \\(\\d{12}\\), move 1: rotate 30°, command 7, still 200 ms before, 200 ms after$`));
    assert.match(lines[1], /^ended ok after 0.3 s; 2 stored of 2 sent; 0 dropped on the robot; 0 lost \(dropped connection\); moved 1 cm, turned 30°$/);
    assert.equal(lines[2], "imu: 13 records stored (0.1 s at 104 a second), 0 missing (gaps), 2 of 2 batches; 0 dropped on the robot; 0 lost (dropped connection)");
    assert.match(r.text, /frame 41 {2}t \d+\.\d{3} s {2}period 100 ms {2}x 50 cm {2}y -25 cm {2}yaw 5.73° {2}pitch -0.57° {2}turn 1.15° {2}still\n/);
    const row1 = lines.find(l => l.includes("- >254"))!; // row 1: nothing, beyond, unsure, less sure, then sure; signal; sigma
    assert.equal(row1, "     - >254  32?  33~   34   35   36   37      0    1    2    3    4    5    6    7      5    5    5    5    5    5    5    5");
    const j = JSON.parse((await post("frames", `${boot.id} 1`, "?json")).text);
    assert.equal(j.frames.length, 2);
    assert.deepEqual([j.imu_records_stored, j.imu_records_missing, j.imu_batches_stored, j.move.imuSent, j.move.imuDropped], [13, 0, 2, 2, 0]);
    assert.equal(j.move.end, "ok");
    assert.equal(j.frames[1].number, 42);
    assert.ok(Math.abs(j.frames[1].t_s - j.frames[0].t_s - 0.1) < 1e-9); // 0.1 s later
    assert.deepEqual(j.frames[0].kind[0].slice(0, 4), ["nothing", "sure", "unsure", "less_sure"]);
    assert.equal(j.frames[0].distance_mm[7][7], 300 + 630);
    // A move with frames dropped on the robot (the lane full) and lost with a dropped
    // connection (sent, never arrived): 1 stored of 3 sent, 2 dropped, 2 lost.
    await store.moveStart(boot, boot.moveStart({ move: 2, command: 8, kind: "drive", parameter: 0.5, tUs: t0 + 400000, preMs: 200, postMs: 200, refused: false }));
    await store.tof(boot, boot.tof({ format: "compact", move: 2, tUs: t0 + 450000, frame: { number: 50, robotClockUs: (t0 + 450000) % 2 ** 32, periodUs: 100000, x: 0.5, y: -0.25, yaw: 0.2,
      pitch: 0, turn: 0, still: false, poseKnown: true, beyondMm: 2550, zones } }));
    await store.moveEnd(boot, boot.moveEnd({ move: 2, end: "ok", tUs: t0 + 900000, framesSent: 3, framesDropped: 2, moved: 0.5, rotated: 0, imuSent: 3, imuDropped: 1 }));
    await store.imuBatch(boot, boot.imuBatch({ move: 2, records: records(40, 10) }));
    await store.imuBatch(boot, boot.imuBatch({ move: 2, records: records(53, 3) })); // 50-52 missing
    await store.flush();
    const lossy = (await post("frames", `${boot.id} 2`)).text.split("\n");
    assert.match(lossy[1], /^ended ok after 0.5 s; 1 stored of 3 sent; 2 dropped on the robot; 2 lost \(dropped connection\);/);
    assert.equal(lossy[2], "imu: 13 records stored (0.1 s at 104 a second), 3 missing (gaps), 2 of 3 batches; 1 dropped on the robot; 1 lost (dropped connection)");
    const lj = JSON.parse((await post("frames", `${boot.id} 2`, "?json")).text);
    assert.deepEqual([lj.frames_stored, lj.frames_lost, lj.move.framesDropped], [1, 2, 2]);
    assert.deepEqual([lj.imu_records_stored, lj.imu_records_missing, lj.imu_batches_stored], [13, 3, 2]);
    const folder = lines[0].match(/\((\d{12})\)/)![1];
    assert.equal((await post("frames", `${folder} 1`)).status, 200);
    assert.equal((await post("frames", `${boot.id} 9`)).status, 404);
    assert.equal((await post("frames", "deadbeef 1")).status, 404);
    assert.equal((await post("frames", boot.id)).status, 400);

    // robot imu: the move's lines, then a record a line, oldest first, the raw counts.
    const imu = (await post("imu", `${boot.id} 1`)).text.split("\n");
    assert.equal(imu[0], lines[0]);
    assert.equal(imu[2], lines[2]);
    assert.match(imu[3], /^ax, ay, az \(acceleration\) and gx, gy, gz \(gyro\): the IMU's raw counts in its own axes/);
    assert.equal(imu[5], "       t s      ax     ay     az     gx     gy     gz");
    const counts = (line: string) => line.trim().split(/ +/);
    assert.equal(imu[6].length, imu[5].length);
    assert.deepEqual(counts(imu[6]).slice(1), ["0", "-1", "16384", "-32768", "32767", "0"]);
    assert.deepEqual(counts(imu[18]).slice(1), ["12", "-1", "16384", "-32768", "32767", "-2"]);
    assert.equal(imu.filter(l => /^ +\d+\.\d{4} /.test(l)).length, 13);
    const ij = JSON.parse((await post("imu", `${boot.id} 1`, "?json")).text);
    assert.equal(ij.records.length, 13);
    assert.deepEqual(ij.records[12].acceleration, [12, -1, 16384]);
    assert.ok(Math.abs(ij.records[12].t_s - ij.records[0].t_s - 12 * 0.009615) < 1e-6);
    assert.deepEqual([ij.imu_records_stored, ij.imu_batches_stored], [13, 2]);
    assert.equal((await post("imu", `${folder} 2`)).status, 200);
    assert.equal((await post("imu", `${boot.id} 9`)).status, 404);
    assert.equal((await post("imu", boot.id)).status, 400);
  } finally { await done(); }
});

// A vision model that reports one chair the first time, a door the next; budgets: the
// reasoningBudget each request was sent with.
const budgets: (number | null)[] = [];
function fakeModel(): Chat {
  const answers = [[{ label: "chair", bbox: [300, 400, 617, 600], confidence: 85 }], [{ label: "doorway", bbox: [0, 300, 600, 700], confidence: 40 }],
                   [{ label: "box", bbox: [300, 400, 617, 600], confidence: 50 }]];
  return async (_m, _t, opts) => (budgets.push(opts.reasoningBudget), { reasoning: "looked", tokens: 1, message: { role: "assistant", content: "", tool_calls: [{ id: "c1", type: "function",
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
    assert.equal(d.objects[0].confidence, 85);
    assert.equal(d.objects[0].floor, "floor");
    assert.equal(d.objects[0].shape.kind, "rect");
    assert.ok(d.objects[0].distance > 50 && d.objects[0].distance < 200, `${d.objects[0].distance}`); // the row 1 m ahead when level; the test pose has some pitch
    const o = JSON.parse((await post("objects", "1")).text);
    assert.equal(o.model, "gemma-fake");
    assert.equal(o.thinking, true);
    assert.deepEqual(o.objects, d.objects);
    const t = JSON.parse((await post("objects", "1", "?transcript")).text);
    assert.equal(t.transcript[0].role, "system");
    assert.equal(t.transcript.at(-2).reasoning, "looked");
    const again = JSON.parse((await post("detect", "1", "?nothinking")).text);
    assert.deepEqual(again.objects.map((x: { label: string }) => x.label), ["doorway"]);
    const after = JSON.parse((await post("objects", "1")).text);
    assert.equal(after.thinking, false);
    assert.equal(after.reasoning_budget, null);
    assert.equal(after.objects.length, 1);
    assert.equal((await post("detect", "1", "?nothinking&reasoningbudget=500")).status, 400);
    assert.equal((await post("detect", "1", "?reasoningbudget=lots")).status, 400);
    assert.equal((await post("detect", "1", "?reasoningbudget=500")).status, 200);
    assert.equal(JSON.parse((await post("objects", "1")).text).reasoning_budget, 500);
    assert.deepEqual(budgets, [null, null, 500]);  // REASONING_BUDGET unset in tests: no limit
    assert.equal((await post("objects", "2")).status, 404);
  } finally { await done(); }
});

test("detect_save: boxes from outside checked, placed, saved with who found them; wrong ones refused", { skip }, async () => {
  const { post, done } = await setUp();
  try {
    const chair = { label: "chair", bbox: [300, 400, 617, 600], confidence: 85 };
    const r = await post("detect_save", `1 ${JSON.stringify({ objects: [chair] })}`, "?by=claude-code");
    assert.equal(r.status, 200, r.text);
    const d = JSON.parse(r.text);
    assert.equal(d.result, "ok");
    assert.equal(d.seconds, null);
    assert.deepEqual(d.objects[0].bbox, chair.bbox);
    assert.equal(d.objects[0].shape.kind, "rect");
    const o = JSON.parse((await post("objects", "1")).text);
    assert.equal(o.model, "claude-code");
    assert.equal(o.thinking, null);
    assert.equal(o.prompt_version, null);
    assert.deepEqual(o.objects, d.objects);
    const t = await post("objects", "1", "?transcript");
    assert.equal(t.status, 404);
    assert.match(t.text, /saved by claude-code without a conversation/);
    // A program can say how it worked, in the JSON.
    const full = JSON.parse((await post("detect_save", `1 ${JSON.stringify({ objects: [], by: "other", prompt_version: 5, thinking: false,
                                                                           seconds: 2.5, turns: 1, transcript: [{ role: "user", text: "hi" }] })}`)).text);
    assert.deepEqual(full.objects, []);
    const o2 = JSON.parse((await post("objects", "1")).text);
    assert.equal(o2.model, "other");
    assert.equal(o2.prompt_version, 5);
    assert.equal(JSON.parse((await post("objects", "1", "?transcript")).text).transcript[0].text, "hi");
    // Refused, nothing saved: what was there stays.
    const bad = await post("detect_save", `1 ${JSON.stringify({ objects: [{ label: "pouf", bbox: [0, 0, 10, 10], confidence: 50 }] })}`, "?by=x");
    assert.equal(bad.status, 400);
    assert.match(bad.text, /not saved; fix these:.*pouf/s);
    assert.equal(JSON.parse((await post("objects", "1")).text).model, "other");
    assert.equal((await post("detect_save", `1 {"objects":[]}`)).status, 400, "without --by");
    assert.equal((await post("detect_save", "1 {nope", "?by=x")).status, 400);
    assert.equal((await post("detect_save", `{"objects":[]}`, "?by=x")).status, 400, "without n");
    assert.equal((await post("detect_save", `9 {"objects":[]}`, "?by=x")).status, 404);
  } finally { await done(); }
});

test("detect without a model set up: 503", { skip }, async () => {
  const { post, done } = await setUp();
  try { assert.equal((await post("detect", "1")).status, 503); } finally { await done(); }
});
