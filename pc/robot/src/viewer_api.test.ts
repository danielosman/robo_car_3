// Tests for viewer_api.ts and replay.ts: scoring a map against truth boxes, truth
// boxes kept per boot, a take's view (frames, targets, path), and both maps replayed
// by the robot's own map code (built here with cc) from the firmware's sample take
// (build/test_recorder.rec, written by ./run_tests.sh). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Store } from "./store.ts";
import { takesIn } from "./import.ts";
import { addTruth, deleteTruth, listTruth, score, takeView, type Truth } from "./viewer_api.ts";
import { exportFrames, replayMaps, type Cell } from "./replay.ts";

const box = (name: string, x: number, y: number, s = 0.08): Truth =>
  ({ truth_id: name, boot_id: "7f3a5c01", name, x, y, z: 0, size_x: s, size_y: s, size_z: 0.1, note: "" });
const cell = (x: number, y: number, column: number): Cell => [x, y, column, 0, column === 1 ? 2 : 1, column === 2 ? 2 : 1, 80];

test("a map's score: truth boxes found, solid cells nobody explains", () => {
  const cells = [cell(0.45, -0.25, 1), cell(0.05, 0.35, 1), cell(0.05, 0.45, 2), cell(1.25, 0.05, 1), cell(2.05, 0.05, 1), cell(0.25, 0.05, 4)];
  const s = score(cells, [box("cup", 0.47, -0.22), box("box", 0.05, 0.4, 0.15), box("chair", -0.5, 0.5)]);
  assert.deepEqual(s.objects.map(o => [o.name, o.found, o.cells]), [["cup", true, 1], ["box", true, 2], ["chair", false, 0]]);
  assert.equal(s.solid, 4);       // within 1.5 m: 2.05 m is out, drivable doesn't count
  assert.equal(s.unexplained, 1); // the cell at 1.25 m
  assert.deepEqual(score(cells, []).objects, []);
});

const repo = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../..");
const sample = path.join(repo, "build/test_recorder.rec");

test("truth boxes per boot; the view and both replayed maps of a take", { skip: !existsSync(sample) && "run ./run_tests.sh first" }, async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-view-"));
  try {
    const store = new Store(path.join(dir, "robot.duckdb"));
    await store.importTake(takesIn(sample)[0]);
    const id = await addTruth(store, { boot_id: "7f3a5c01", name: "cup", x: 0.5, y: -0.2, size_x: 0.08, size_y: 0.08, size_z: 0.1 });
    await addTruth(store, { boot_id: "11111111", name: "other boot", x: 1, y: 1 });
    const kept = await listTruth(store, "7f3a5c01");
    assert.deepEqual(kept.map(t => [t.name, t.x, t.size_z]), [["cup", 0.5, 0.1]]);
    await assert.rejects(addTruth(store, { name: "no boot" }));
    await deleteTruth(store, id);
    assert.equal((await listTruth(store, "7f3a5c01")).length, 0);

    const view = (await takeView(store, "7f3a5c01-1"))!;
    assert.ok(view.take.action === "scan" && view.take.geometry.rows === 8 && Math.abs(view.take.geometry.sensorM[2] - 0.07) < 1e-6);
    assert.equal(view.frames.frame_no.length, 2);
    assert.equal(view.points.x.length, 126);
    assert.ok(view.points.row.every(r => r >= 1 && r <= 8) && view.odom.t_s.length === 2);
    assert.equal(await takeView(store, "7f3a5c01-9"), null);

    const frames = await exportFrames(store, "7f3a5c01-1");
    assert.equal(frames.split("\n").length - 1, 2);
    assert.equal(frames.split("\n")[0].split(" ").length, 6 + 2 * 64);
    const maps = await replayMaps(store, "7f3a5c01-1");
    assert.ok(maps.frames === 2 && maps.votes.length > 0 && maps.readings.length > 0);
    assert.ok(maps.readings.every(c => c.length === 7 && c[6] >= 0 && c[6] <= 100));
    await assert.rejects(exportFrames(store, "x'; DROP TABLE takes; --"));
  } finally { rmSync(dir, { recursive: true, force: true }); }
});
