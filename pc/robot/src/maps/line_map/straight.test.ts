// Tests for straight.ts: lines through hand-made points: a wall, a corner, two walls side
// by side, a wall with a doorway; clutter and too few points give none. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { straightLines, type Point, type Segment } from "./straight.ts";

const DEG = Math.PI / 180;
const within = (v: number, low: number, high: number) => assert.ok(v >= low && v <= high, `${v} not in ${low}..${high}`);
const lengthOf = (s: Segment) => Math.hypot(s.bx - s.ax, s.by - s.ay);
const angleOf = (s: Segment) => ((Math.atan2(s.by - s.ay, s.bx - s.ax) / DEG) % 180 + 180) % 180;
// Points every 5 cm from (x, y) along `deg` for `metres`, each `wobble(i)` off the line.
function wall(x: number, y: number, deg: number, metres: number, wobble: (i: number) => number = () => 0): Point[] {
  const ux = Math.cos(deg * DEG), uy = Math.sin(deg * DEG);
  return Array.from({ length: Math.round(metres / 0.05) + 1 }, (_, i) => ({ x: x + ux * i * 0.05 - uy * wobble(i), y: y + uy * i * 0.05 + ux * wobble(i) }));
}
const byLength = (lines: Segment[]) => lines.sort((a, b) => lengthOf(b) - lengthOf(a));

test("a wall is one line, end to end, also at an angle between the directions tried and with noise", () => {
  // 2 m at 10.7°, the points up to 1.5 cm to either side.
  const [line, ...rest] = straightLines(wall(1, 2, 10.7, 2, i => [0.015, -0.01, 0, 0.01, -0.015][i % 5]));
  assert.equal(rest.length, 0);
  within(lengthOf(line), 1.99, 2.01);
  within(angleOf(line), 10.2, 11.2);
  const [nearX, nearY] = line.ax < line.bx ? [line.ax, line.ay] : [line.bx, line.by];
  within(nearX, 0.98, 1.02);
  within(nearY, 1.98, 2.02);
});

test("a corner is two lines at right angles that meet at it", () => {
  const [long, short, ...rest] = byLength(straightLines([...wall(0, 0, 0, 1), ...wall(1, 0.05, 90, 0.6)]));
  assert.equal(rest.length, 0);
  within(Math.min(angleOf(long), 180 - angleOf(long)), 0, 1); // the corner's own point may pull it a little
  within(lengthOf(long), 0.95, 1.05);
  within(angleOf(short), 89, 91);
  within(lengthOf(short), 0.5, 0.65);
});

test("two walls 20 cm apart are two lines; a wall with a 30 cm doorway is two", () => {
  const side = byLength(straightLines([...wall(0, 0, 0, 1), ...wall(0, 0.2, 0, 1)]));
  assert.deepEqual(side.map(l => Math.round(lengthOf(l) * 100)), [100, 100]);
  assert.deepEqual(side.map(l => Math.round(l.ay * 100)).sort((a, b) => a - b), [0, 20]);
  const doorway = byLength(straightLines([...wall(0, 0, 0, 1), ...wall(1.3, 0, 0, 0.5)]));
  assert.deepEqual(doorway.map(l => Math.round(lengthOf(l) * 100)), [100, 50]);
});

test("clutter and too few points give no line", () => {
  assert.deepEqual(straightLines([]), []);
  assert.deepEqual(straightLines(wall(0, 0, 0, 0.1)), []); // 3 points
  // Points scattered 20 cm and more apart: no four in a row.
  assert.deepEqual(straightLines([{ x: 0, y: 0 }, { x: 0.2, y: 0.3 }, { x: 0.5, y: 0.1 }, { x: 0.7, y: 0.5 }, { x: 0.3, y: 0.7 }, { x: 0.9, y: 0.9 }]), []);
  // A leg next to a wall: the wall is a line, the leg's two points aren't.
  const lines = straightLines([...wall(0, 0, 0, 1), { x: 0.5, y: 0.5 }, { x: 0.52, y: 0.53 }]);
  assert.equal(lines.length, 1);
});
