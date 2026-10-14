// Tests for map.ts and check.ts: the TypeScript obstacle check gives what the
// robot's own C check gave on the same map (build/test_snapshot_checks.txt and
// build/test_snapshot.snap, written by ./run_tests.sh), inside the window and out in
// the whole-house tiles; check's answers; the map as rows of characters. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { SnapshotAssembler, parseSnapshot } from "../snapshot.ts";
import { mapViewOf } from "../snapshots_api.ts";
import { parseSteer, type Command } from "./commands.ts";
import { checkMoves } from "./check.ts";
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

test("check: drive, rotate, goto, each from where the one before ends", { skip }, () => {
  const map = firstMap(), start = map.view.robot!;
  const short = (text: string, from = start) => checkMoves(steer(text), map, from).map(a =>
    `${a.command} ${a.result}${a.why ? ` (${a.why})` : ""}${a.pose ? ` at ${a.pose.x},${a.pose.y},${a.pose.yaw} moved ${a.moved} rotated ${a.rotated}` : ""}`);
  assert.deepEqual(short("d50 r90"), ["steer d50 ok at -50,15,0 moved 50 rotated 0", "steer r90 ok at -50,15,90 moved 0 rotated 90"]);
  // 1 m to the blocked column, less the front edge (9.5 cm) and 15 cm.
  assert.deepEqual(short("d100 r30"), ["steer d100 obstacle at -24,15,0 moved 76 rotated 0", "steer r30 dropped (steer d100 ended: obstacle)"]);
  assert.equal(short("d75 d10")[1], "steer d10 obstacle at -25,15,0 moved 0 rotated 0");
  assert.deepEqual(short("g-100,-185"), ["steer g-100,-185 ok at -100,-185,-90 moved 200 rotated -90"]);
  assert.match(short("g250,250", { x: -2.5, y: -2.5, yaw: 0 })[0], /^steer g250,250 limited .* moved 300 rotated 45$/);
  const turn = checkMoves(steer("r90"), map, { x: 0.05, y: 0.0, yaw: 0 })[0];
  assert.match(turn.warning ?? "", /an obstacle within 15 cm/);
});

test("the map as rows: one character a column, the robot, the path, the top-left corner", { skip }, () => {
  const view = firstMap().view;
  const g = new RobotMap({ ...view, robot: { x: 0.55, y: -0.05, yaw: 0 }, path: [{ x: 0, y: -0.05 }, { x: 0.55, y: -0.05 }] }).grid()!;
  assert.equal(g.rows.length, 60);
  assert.ok(g.rows.every(r => r.length === 60));
  assert.deepEqual([g.xTop, g.yLeft], [290, 290]); // the window is -300…299 cm
  const at = (cx: number, cy: number) => g.rows[29 - cx][29 - cy];
  assert.equal(at(0, 1), "#");
  assert.equal(at(5, -1), "R");
  assert.equal(at(8, -1), "*");
  assert.equal(at(2, -1), "o");
});
