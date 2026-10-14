// Tests for map.ts and check.ts: the TypeScript obstacle check gives what the
// robot's own C check gave on the same map (build/test_snapshot_checks.txt and
// build/test_snapshot.snap, written by ./run_tests.sh), inside the window and out in
// the whole-house tiles; --check's answers; the map as text. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { SnapshotAssembler, parseSnapshot } from "../snapshot.ts";
import { mapViewOf } from "../snapshots_api.ts";
import { parseSteer, type Command } from "./commands.ts";
import { checkSteer } from "./check.ts";
import { RobotMap } from "./map.ts";

const BUILD = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../../build");
const have = existsSync(path.join(BUILD, "test_snapshot_checks.txt"));
const skip = !have && "run ./run_tests.sh first";
// The firmware test's first snapshot: one blocked column in the window at world cell
// (0, 1), i.e. x 0-10 cm, y 10-20 cm; tiles with a blocked column at (-45, 0).
const firstMap = () => {
  const view = mapViewOf(parseSnapshot(new SnapshotAssembler().push(readFileSync(path.join(BUILD, "test_snapshot.snap")))[0]));
  return new RobotMap({ ...view, robot: { x: -1, y: 0.15, yaw: 0 } });
};
const steer = (text: string) => (parseSteer(text) as { commands: Command[] }).commands;

test("the same obstacle check as the robot's, on the same map", { skip }, () => {
  const map = firstMap();
  const lines = readFileSync(path.join(BUILD, "test_snapshot_checks.txt"), "utf8").trim().split("\n");
  assert.ok(lines.length >= 8);
  for (const line of lines) {
    const [x, y, dx, dy, half, max, robot] = line.split(" ").map(Number);
    assert.ok(Math.abs(map.obstacleAlong(x, y, dx, dy, half, max) - robot) < 1e-4, line);
  }
});

test("--check: drive, rotate, goto, each from where the one before ends", { skip }, () => {
  const map = firstMap(), start = map.view.robot!;
  assert.deepEqual(checkSteer(steer("d50 r90"), map, start), [
    "check: d50 clear: would end at (-50, 15) cm, yaw +0°",
    "check: r90 clear: turns to yaw +90°",
  ]);
  // 1 m to the blocked column, less the front edge (9.5 cm) and 15 cm.
  assert.deepEqual(checkSteer(steer("d100 r30"), map, start), [
    "check: d100 blocked: obstacle after 76 cm, would stop at (-24, 15) cm, yaw +0°",
    "check: r30 would be dropped",
  ]);
  assert.deepEqual(checkSteer(steer("d75 d10"), map, start)[1], "check: d10 blocked: obstacle after 0 cm, would stop at (-25, 15) cm, yaw +0°");
  assert.deepEqual(checkSteer(steer("g-100,-185"), map, start), [
    "check: g-100,-185 turn -90° to yaw -90°, then drive 200 cm; clear: would end at (-100, -185) cm, yaw -90°",
  ]);
  assert.match(checkSteer(steer("g250,250"), map, { x: -2.5, y: -2.5, yaw: 0 })[0], /drive 300 cm \(3 m towards it; the goal is 7\.1 m away\)/);
  assert.match(checkSteer(steer("r90"), map, { x: 0.05, y: 0.0, yaw: 0 })[0], /careful: an obstacle within the 15 cm/);
});

test("the map as text: one character a column, the robot, the path, the edges", { skip }, () => {
  const view = firstMap().view;
  const text = new RobotMap({ ...view, robot: { x: 0.55, y: -0.05, yaw: 0 }, path: [{ x: 0, y: -0.05 }, { x: 0.55, y: -0.05 }] }).text()!;
  const lines = text.split("\n"), grid = lines.filter(l => /^ {0,5}[+-]?\d* *\|/.test(l));
  assert.equal(grid.length, 60);
  assert.ok(grid.every(l => l.length === 6 + 1 + 60 + 1));
  const at = (cx: number, cy: number) => grid[29 - cx][7 + 29 - cy]; // window -30…29
  assert.equal(at(0, 1), "#");
  assert.equal(at(5, -1), "R");
  assert.equal(at(8, -1), "*");
  assert.equal(at(2, -1), "o");
  assert.ok(lines.some(l => l.startsWith("y cm:") && l.includes("+100") && l.includes("-100")));
  assert.ok(/R the robot/.test(text));
});
