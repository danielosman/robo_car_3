// Tests for store.ts: a target's world position (hand-computed cases), the pose
// between odometry reports, and a take imported into DuckDB and again (its rows
// replaced, its note kept), from the firmware's own datagrams (build/test_recorder.rec,
// written by ./run_tests.sh). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { Geometry, Odom } from "./recording.ts";
import { Store, poseAt, worldPoint } from "./store.ts";
import { takesIn } from "./import.ts";

const DEG = Math.PI / 180;
const geometry: Geometry = {
  kind: "GEOMETRY", sensorM: [0.025, -0.03, 0.07], zoneRad: 5.625 * DEG, rows: 8, cols: 8,
  zoneOfRay: Array.from({ length: 64 }, (_, ray) => (7 - ray % 8) * 8 + Math.floor(ray / 8)),
};
const close = (a: number, b: number, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);

test("world points: the robot's geometry, turned and moved", () => {
  const still = { x: 0, y: 0, yaw: 0, pitch: 0 };
  // Row 4, column 4 (1-8): half a zone up and half a zone left of straight ahead.
  const h = 2.8125 * DEG;
  let [x, y, z] = worldPoint(geometry, still, 4, 4, 1);
  close(x, 0.025 + Math.cos(h) * Math.cos(h));
  close(y, -0.03 + Math.cos(h) * Math.sin(h));
  close(z, 0.07 + Math.sin(h));
  // The robot at (1, 2), turned 90° left: forward is +y, left is -x.
  [x, y, z] = worldPoint(geometry, { x: 1, y: 2, yaw: 90 * DEG, pitch: 0 }, 4, 4, 1);
  close(x, 1 + 0.03 - Math.cos(h) * Math.sin(h));
  close(y, 2 + 0.025 + Math.cos(h) * Math.cos(h));
  // The bottom row (8) is 3.5 zones down; at the distance where that ray meets the floor, z = 0.
  const down = 3.5 * 5.625 * DEG;
  [, , z] = worldPoint(geometry, still, 8, 4, 0.07 / Math.sin(down));
  close(z, 0);
  // Nose up by half a zone: row 5's ray (half a zone down) becomes level.
  [, , z] = worldPoint(geometry, { ...still, pitch: h }, 5, 4, 1);
  close(z, 0.07);
});

test("the pose between odometry reports, none far from them", () => {
  const o = (t: number, x: number, yaw: number) => ({ t, o: { kind: "ODOM", x, y: 0, yaw, pitch: 0 } as Odom });
  const reports = [o(0, 0, 0), o(0.02, 0.01, 0.1), o(0.04, 0.02, 0.2), o(1.0, 0.5, 0.2)];
  const p = poseAt(reports, 0.03)!;
  close(p.x, 0.015);
  close(p.yaw, 0.15);
  assert.equal(poseAt(reports, 0.5), null);     // 0.96 s without a report
  assert.ok(poseAt(reports, -0.05));            // just before the first: that one
  assert.equal(poseAt(reports, -0.2), null);
  assert.equal(poseAt([], 0), null);
});

const repo = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../..");
const sample = path.join(repo, "build/test_recorder.rec");

test("a take into DuckDB and again: rows replaced, note kept", { skip: !existsSync(sample) && "run ./run_tests.sh first" }, async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-db-"));
  try {
    const store = new Store(path.join(dir, "robot.duckdb"));
    const [take] = takesIn(sample);
    const n = await store.importTake(take);
    assert.deepEqual(n, { frames: 2, zones: 128, targets: 126, odom: 2 });
    const [row] = await store.query("SELECT * FROM takes");
    assert.equal(row.take_id, "7f3a5c01-1");
    assert.ok(row.key === "s" && row.action === "scan" && row.end_reason === "done" && row.version === 3);
    assert.ok(row.frames_whole === 2 && row.frames_missing === 0 && row.datagrams_lost === 0);
    // Sensor zone 5 (row 6, column 8) has two targets at 500 and 510 mm, in every frame.
    const targets = await store.query("SELECT row, col, target, distance_mm, x, y, z FROM tof_targets WHERE zone = 5 ORDER BY frame_no, target");
    assert.deepEqual(targets.slice(0, 2).map(t => [t.row, t.col, t.target, t.distance_mm]), [[6, 8, 0, 500], [6, 8, 1, 510]]);
    assert.ok(targets.every(t => t.x !== null)); // the odometry was there
    const frames = await store.query("SELECT frame_no, zones, x FROM tof_frames ORDER BY frame_no");
    assert.ok(frames.length === 2 && frames.every(f => f.zones === 64 && f.x !== null));
    const counts = await store.query(`SELECT (SELECT count(*) FROM odom) o, (SELECT count(*) FROM marks) m,
      (SELECT count(*) FROM drive) d, (SELECT count(*) FROM wifi) w, (SELECT count(*) FROM tof_zones) z`);
    assert.deepEqual(counts[0], { o: "2", m: "3", d: "1", w: "1", z: "128" });
    await store.setNote("7f3a5c01-1", "cup at 45 cm");
    await store.importTake(takesIn(sample)[0]);
    const again = await store.query(`SELECT note, (SELECT count(*) FROM tof_targets) t FROM takes`);
    assert.deepEqual(again, [{ note: "cup at 45 cm", t: "126" }]);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});
