// Tests for mapper.ts's rule for removal: the pose is well fixed without a wall only if it
// was just checked against two other walls at an angle that are still known. (Whole
// drives through the mapper are in sim/sim.test.ts.) Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { fixedWithout, type Fix } from "./mapper.ts";
import { NO_DOUBT } from "./pose.ts";
import type { Wall } from "./walls.ts";

const DEG = Math.PI / 180;
const wall = (number: number, angle: number): Wall => ({ number, x: 0, y: 0, angle, halfM: 1, endSd: [0.1, 0.1], cov: [0, 0, 0], surest: NO_DOUBT,
                                                         sightings: 1, firstFrame: 0, lastFrame: 0, through: 0 });

test("well fixed without a wall: two other known walls at 45° or more, checked just now", () => {
  const a = wall(1, 0), b = wall(2, 90 * DEG), c = wall(3, 90 * DEG), other = wall(4, 30 * DEG);
  const fix: Fix = { walls: [a, b], added: NO_DOUBT }, known = [a, b, c, other];
  assert.equal(fixedWithout(fix, known, other), true);
  assert.equal(fixedWithout(null, known, other), false);
  // Not by the wall itself: without a, b alone is left.
  assert.equal(fixedWithout(fix, known, a), false);
  // Two walls in line don't fix it; 40° apart neither.
  assert.equal(fixedWithout({ walls: [b, c], added: NO_DOUBT }, known, other), false);
  assert.equal(fixedWithout({ walls: [a, wall(5, 40 * DEG)], added: NO_DOUBT }, [...known], other), false);
  // A wall merged away or removed since doesn't count.
  assert.equal(fixedWithout(fix, [a, c, other], other), false);
  // Odometry added more than 2 cm or 3° since: no longer fixed.
  assert.equal(fixedWithout({ ...fix, added: { xyM: 0.019, yawRad: 2.9 * DEG } }, known, other), true);
  assert.equal(fixedWithout({ ...fix, added: { xyM: 0.021, yawRad: 0 } }, known, other), false);
  assert.equal(fixedWithout({ ...fix, added: { xyM: 0, yawRad: 3.1 * DEG } }, known, other), false);
});
