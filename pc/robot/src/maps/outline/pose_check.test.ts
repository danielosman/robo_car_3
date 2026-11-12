// Tests for pose_check.ts on hand-made sights: one wall corrects the pose's turn and its
// distance across the wall, not along it; two walls at an angle correct the whole pose;
// a measurement too far off is left out; a wall made from an unsure pose doesn't make the
// pose surer. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { PoseCheck } from "./pose_check.ts";
import { Walls } from "./walls.ts";
import { BANDS, type Sight } from "./sights.ts";
import type { Edge } from "./shape.ts";
import type { Pose } from "./pose.ts";
import { DEG, row, sight } from "./test_sights.ts";

const AT: Pose = { x: 0, y: 0, yaw: 0 }, ABOVE = BANDS[2];
const close = (a: number, b: number, eps: number) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const along = (e: Edge): Sight[] => Array.from({ length: 21 }, (_, i) => {
  const x = e.ax + (e.bx - e.ax) * i / 20, y = e.ay + (e.by - e.ay) * i / 20;
  return sight(Math.atan2(y, x), Math.hypot(x, y));
});
const X_WALL: Edge = { ax: 1, ay: -0.5, bx: 1, by: 0.5, kind: "wall" }, Y_WALL: Edge = { ax: -0.5, ay: 1, bx: 0.5, by: 1, kind: "wall" };
// The robot truly at the origin; its frame placed at `placed` (odometry off by that much).
// A row facing along `facing` (in truth) reads a wall `at` metres ahead, square to it.
const frame = (placed: Pose, facing: number, at: number): Sight[] =>
  row(b => at / Math.cos(b - placed.yaw - facing), facing + placed.yaw, { x: placed.x, y: placed.y });
// Known walls, made from the true pose and seen there 10 times (sure); then, unless `still`,
// the pose's doubt grows by a full turn.
function known(edges: Edge[], still = false): { check: PoseCheck; walls: Walls } {
  const check = new PoseCheck(), walls = new Walls(ABOVE, check);
  walls.found(edges, 0, AT, check.doubt(), edges.flatMap(along));
  for (let i = 0; i < 10; i++) walls.apply(i, AT, check.doubt(), ...(() => { const ms = walls.measure(AT, check.doubt(), edges.flatMap(along)), c = check.check(ms); return [c.used, c.walls] as const; })());
  if (still) return { check, walls };
  check.moved(AT, { x: 0, y: 0, yaw: Math.PI });
  check.moved({ x: 0, y: 0, yaw: Math.PI }, { x: 0, y: 0, yaw: 2 * Math.PI });
  return { check, walls };
}

test("one wall corrects the turn and the distance across it, not along it", () => {
  const { check, walls } = known([X_WALL]), placed = { x: 0.05, y: 0.05, yaw: 3 * DEG };
  const ms = walls.measure(placed, check.doubt(), frame(placed, 0, 1));
  const { pose: [dx, dy, dYaw], used } = check.check(ms);
  assert.equal(used.length, 1);
  close(dYaw, -3 * DEG, 0.5 * DEG);
  close(dx, -0.05, 0.015);
  close(dy, 0, 0.01); // along the wall: nothing to go by
});

test("two walls at an angle correct the whole pose", () => {
  const { check, walls } = known([X_WALL, Y_WALL]), placed = { x: 0.05, y: -0.04, yaw: 3 * DEG };
  const ms = walls.measure(placed, check.doubt(), [...frame(placed, 0, 1), ...frame(placed, Math.PI / 2, 1)]);
  assert.equal(ms.length, 2);
  const { pose: [dx, dy, dYaw] } = check.check(ms);
  close(dYaw, -3 * DEG, 0.5 * DEG);
  close(dx, -0.05, 0.015);
  close(dy, 0.04, 0.015);
  assert.ok(check.doubt().xyM < 0.02 && check.doubt().yawRad < 2 * DEG, JSON.stringify(check.doubt())); // from 6 cm and 12.6°
});

test("a measurement too far off for the doubts is left out: a thing moved", () => {
  const { check, walls } = known([X_WALL], true);
  // Wall and pose are sure. A measurement says the hits lie 20 cm behind the wall (made by
  // hand: measure() itself would not take hits that far off as on the wall).
  const [m] = walls.measure(AT, check.doubt(), frame(AT, 0, 1));
  const far = { ...m, z: [m.z[0] - 0.2, ...m.z.slice(1)] };
  const { pose, used } = check.check([far]);
  assert.deepEqual([used.length, ...pose], [0, 0, 0, 0]);
  // The same measurement as seen is used.
  assert.equal(check.check([m]).used.length, 1);
});

test("a wall made from an unsure pose, seen again from there, doesn't make the pose surer", () => {
  const check = new PoseCheck(), walls = new Walls(ABOVE, check), turned = { x: 0, y: 0, yaw: Math.PI };
  check.moved(AT, turned);
  check.moved(turned, { x: 0, y: 0, yaw: 2 * Math.PI });
  const before = check.doubt();
  walls.found([X_WALL], 0, AT, before, along(X_WALL));
  for (let i = 0; i < 10; i++) check.check(walls.measure(AT, check.doubt(), frame(AT, 0, 1)));
  close(check.doubt().yawRad, before.yawRad, 0.1 * DEG);
});
