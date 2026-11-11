// Tests for outline_map.ts, on hand-made sights: the outline starts as a 35 cm circle,
// grows where sights saw through, lies on a wall seen in three frames, and moves out
// again when the thing went away. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { OutlineMap, type Edge, type Loop } from "./outline_map.ts";
import { BANDS, type Sight } from "./sights.ts";
import { row, sight } from "./test_sights.ts";

const L2 = BANDS[1];
const lengthOf = (e: Edge) => Math.hypot(e.bx - e.ax, e.by - e.ay);
const isClosed = (l: Loop) => l.edges.every((e, i) => { const n = l.edges[(i + 1) % l.edges.length]; return Math.hypot(n.ax - e.bx, n.ay - e.by) < 1e-9; });
// The area inside a loop (its edges run clockwise round the floor).
const areaOf = (l: Loop) => Math.abs(l.edges.reduce((sum, e) => sum + e.ax * e.by - e.bx * e.ay, 0)) / 2;
// A frame of one row looking along x from the origin at the wall x = wallAt (test_sights.ts:
// the wedges rise 10 to 20 cm a metre from 9 cm up, so 0.9 m away they cover 18-27 cm: L2).
const wallFrame = (wallAt: number, more: Partial<Sight> = {}): Sight[] => row(b => wallAt / Math.cos(b), 0, more);
const times = (n: number, map: OutlineMap, frame: () => Sight[]) => { for (let i = 0; i < n; i++) map.add(frame()); };

test("at the start the outline is a 35 cm circle, all guess", () => {
  const [circle, ...rest] = new OutlineMap(L2, { x: 1, y: -2 }).outline();
  assert.equal(rest.length, 0);
  assert.equal(circle.island, false);
  assert.ok(isClosed(circle) && circle.edges.every(e => e.kind === "guess"));
  assert.ok(Math.abs(areaOf(circle) - Math.PI * 0.35 ** 2) < 0.06, `${areaOf(circle)}`);
  for (const e of circle.edges) assert.ok(Math.abs(Math.hypot(e.ax - 1, e.ay + 2) - 0.35) < 0.06);
});

test("a wall seen in three frames: the outline lies on it, as a wall; seen twice it is still a guess before it", () => {
  const map = new OutlineMap(L2);
  times(2, map, () => wallFrame(0.9));
  const before = map.outline()[0];
  assert.ok(isClosed(before) && before.edges.every(e => e.kind === "guess"));
  assert.ok(Math.max(...before.edges.map(e => e.ax)) < 0.87); // the floor seen free ends a margin before the wall
  map.add(wallFrame(0.9));
  const [outline, ...rest] = map.outline();
  assert.equal(rest.length, 0);
  assert.ok(isClosed(outline));
  const [wall, ...more] = outline.edges.filter(e => e.kind === "wall");
  assert.equal(more.length, 0);
  // 8 zones: the outer hits are 2 x 0.9 m x tan(3.5 zones) = 64 cm apart.
  assert.ok(Math.abs(wall.ax - 0.9) < 1e-3 && Math.abs(wall.bx - 0.9) < 1e-3);
  assert.ok(Math.abs(lengthOf(wall) - 0.64) < 0.02, `${lengthOf(wall)}`);
  // The rest is a guess: the fan's two sides and the start circle.
  assert.ok(outline.edges.every(e => e.kind !== "rough"));
  assert.ok(areaOf(outline) > 0.5 && areaOf(outline) < 0.75, `${areaOf(outline)}`);
});

test("a thing that went away: the outline moves out to the wall behind it", () => {
  const map = new OutlineMap(L2);
  times(5, map, () => wallFrame(0.7));
  assert.ok(map.outline()[0].edges.some(e => e.kind === "wall" && Math.abs(e.ax - 0.7) < 1e-3));
  times(8, map, () => wallFrame(1.0));
  const walls = map.outline()[0].edges.filter(e => e.kind === "wall");
  assert.equal(walls.length, 1);
  assert.ok(Math.abs(walls[0].ax - 1.0) < 1e-3);
});

test("a thing hit in the start square: there is still an outline; a hole is an island", () => {
  const map = new OutlineMap(L2);
  times(4, map, () => [sight(0, 0.02)]);
  assert.equal(map.outline().length, 1);
  // A thing 60 cm ahead, 2 zones wide, seen past on both sides to a wall at 1 m: a hole in the floor.
  const island = new OutlineMap(L2);
  times(5, island, () => row((b, col) => (col === 4 || col === 5 ? 0.6 : 1.0) / Math.cos(b)));
  const loops = island.outline();
  assert.deepEqual(loops.map(l => l.island), [false]); // behind the thing nothing was seen: a dent, not a hole
  times(5, island, () => row(b => 1.0 / Math.cos(b), 0, { y: 0.3 }));  // from 30 cm to the left: behind it too
  times(5, island, () => row(b => 1.0 / Math.cos(b), 0, { y: -0.3 }));
  assert.ok(island.outline().some(l => l.island && l.edges.some(e => e.kind !== "guess")));
});

test("a band's floor is seen free only by wedges at its heights; close to the sensor by any", () => {
  // The same wedges are 18-27 cm up at 0.9 m: over L1 (3-13 cm). L1's floor grows only close to the sensor.
  const map = new OutlineMap(BANDS[0]);
  times(5, map, () => wallFrame(0.9));
  const [outline] = map.outline(), reach = Math.max(...outline.edges.map(e => e.ax));
  assert.ok(reach > 0.4 && reach < 0.5, `${reach}`);
  assert.ok(outline.edges.every(e => e.kind === "guess"));
});
