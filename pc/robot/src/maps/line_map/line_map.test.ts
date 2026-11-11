// Tests for line_map.ts, on hand-made sights: a wall seen in three frames is a line
// where the wall is; a close look counts more than a far one; a thing that went away
// goes from the map, unless the sights passed over it or only brushed it. Run with
// `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { LineMap, type Segment } from "./line_map.ts";
import { row, sight, ZONE } from "./test_sights.ts";
import type { Sight } from "./sights.ts";

const lengthOf = (s: Segment) => Math.hypot(s.bx - s.ax, s.by - s.ay);
const within = (v: number, low: number, high: number) => assert.ok(v >= low && v <= high, `${v} not in ${low}..${high}`);
// A frame of one row from a sensor at (x, 0) looking along `facing` at the wall x = wallAt.
const wallFrame = (wallAt: number, x = 0, facing = 0, more: Partial<Sight> = {}): Sight[] =>
  row(b => (wallAt - x) / Math.cos(b), facing, { x, ...more });
const times = (n: number, map: LineMap, frame: () => Sight[]) => { for (let i = 0; i < n; i++) map.add(frame()); };

test("a wall seen in three frames is one line along it; seen twice it isn't drawn yet", () => {
  const map = new LineMap();
  times(2, map, () => wallFrame(1.02));
  assert.deepEqual(map.lines(), []);
  map.add(wallFrame(1.02));
  const [wall, ...rest] = map.lines();
  assert.equal(rest.length, 0);
  within(wall.ax, 1.019, 1.021);
  within(wall.bx, 1.019, 1.021);
  // 8 zones: the outer hits are 2 x 1.02 m x tan(3.5 zones) = 73 cm apart.
  within(lengthOf(wall), 0.72, 0.74);
});

test("a close look counts more than a far one", () => {
  const map = new LineMap();
  // Eight sights side by side, 5 cm apart, each straight ahead at the same wall from `rangeM` away.
  const frame = (wallAt: number, rangeM: number) =>
    Array.from({ length: 8 }, (_, i) => sight(0, rangeM, { x: wallAt - rangeM, y: (i - 3.5) * 0.05 }));
  // From 1.04 m the wall reads at x = 1.04, three times.
  times(3, map, () => frame(1.04, 1.04));
  for (const x of map.lines().flatMap(l => [l.ax, l.bx])) within(x, 1.039, 1.041);
  // From half as far it reads at 1.02, three times: each such hit counts four times as much.
  times(3, map, () => frame(1.02, 0.52));
  const [wall, ...rest] = map.lines();
  assert.equal(rest.length, 0);
  for (const x of [wall.ax, wall.bx]) within(x, 1.0235, 1.0245); // (1.04 + 4 x 1.02) / 5
});

test("a frame counts once, however many of its rows hit a spot", () => {
  const map = new LineMap();
  // Three rows of one frame on the same wall: still one frame. Two more frames: it stands.
  map.add([...wallFrame(1.02), ...wallFrame(1.02), ...wallFrame(1.02)]);
  assert.deepEqual(map.lines(), []);
  times(2, map, () => wallFrame(1.02));
  assert.equal(map.lines().length, 1);
});

test("a thing seen for long is still forgotten soon after it went: its score stops at 5", () => {
  const map = new LineMap();
  times(40, map, () => wallFrame(0.52));
  times(5, map, () => wallFrame(1.22));
  assert.deepEqual(map.lines().filter(l => l.ax < 1), []);
});

// A box 50 cm ahead, seen in five frames; then `after` frames; what is left of it.
function boxThen(after: () => Sight[], box: Partial<Sight> = {}): Segment[] {
  const map = new LineMap();
  times(5, map, () => wallFrame(0.52, 0, 0, box));
  assert.equal(map.lines().length, 1);
  times(8, map, after);
  return map.lines().filter(l => l.ax < 1);
}

test("a thing that went away goes from the map; the wall behind it shows", () => {
  const map = new LineMap();
  times(5, map, () => wallFrame(0.52));
  times(8, map, () => wallFrame(1.22));
  const lines = map.lines();
  assert.equal(lines.length, 1);
  within(lines[0].ax, 1.219, 1.221);
});

test("a sight that stops before a thing, or passes over it, leaves it", () => {
  // The same wedges reading something 55 cm away: they saw through 45 cm only.
  assert.equal(boxThen(() => wallFrame(0.57)).length, 1);
  // A low box (3-8 cm up at 50 cm); the wedges that read the wall behind pass 14-19 cm up there.
  assert.equal(boxThen(() => wallFrame(1.22), { lowPerM: -0.12, highPerM: -0.02 }).length, 1);
  // The same low wedges reading the floor behind it see through it: it goes.
  assert.equal(boxThen(() => wallFrame(1.22, 0, 0, { lowPerM: -0.12, highPerM: -0.02 }), { lowPerM: -0.12, highPerM: -0.02 }).length, 0);
});

test("only a wedge's middle sees through: a thing under the wedges' edges stays", () => {
  // The row turned by half a zone: every hit of the box lies on the border between two wedges.
  assert.equal(boxThen(() => wallFrame(1.22, 0, ZONE / 2)).length, 1);
});
