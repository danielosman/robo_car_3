// Tests for shape.ts: a border loop's points as edges: a long straight run of hits is a
// wall, a short or jagged one rough, the rest a guess; the loop stays closed and doesn't
// cut through itself. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { edgesOf, type BorderPoint, type Edge } from "./shape.ts";

const lengthOf = (e: Edge) => Math.hypot(e.bx - e.ax, e.by - e.ay);
const isClosed = (edges: Edge[]) => edges.every((e, i) => { const n = edges[(i + 1) % edges.length]; return Math.hypot(n.ax - e.bx, n.ay - e.by) < 1e-9; });
// Points every 5 cm from (ax, ay) up to, not including, (bx, by).
const run = (ax: number, ay: number, bx: number, by: number, hit: boolean, wobble = 0): BorderPoint[] => {
  const n = Math.round(Math.hypot(bx - ax, by - ay) / 0.05), nx = -(by - ay) / (n * 0.05), ny = (bx - ax) / (n * 0.05);
  return Array.from({ length: n }, (_, i) => { const off = i % 2 ? wobble : -wobble; return { x: ax + (bx - ax) * i / n + nx * off, y: ay + (by - ay) * i / n + ny * off, hit }; });
};

test("a square of hits: four walls, corner to corner", () => {
  const edges = edgesOf([...run(0, 0, 1, 0, true), ...run(1, 0, 1, 1, true), ...run(1, 1, 0, 1, true), ...run(0, 1, 0, 0, true)]);
  assert.deepEqual(edges.map(e => e.kind), ["wall", "wall", "wall", "wall"]);
  assert.ok(isClosed(edges));
  for (const e of edges) assert.ok(Math.abs(lengthOf(e) - 1) < 1e-9);
});

test("hits 1 cm off a line are still a wall; a jagged run and a short run are rough; no hits: a guess", () => {
  const edges = edgesOf([
    ...run(0, 0, 2, 0, true, 0.01),   // a wall with a little noise
    ...run(2, 0, 2, 1, true, 0.08),   // jagged
    ...run(2, 1, 1.7, 1, true),       // straight but short
    ...run(1.7, 1, 0, 1, false),      // nothing seen
    ...run(0, 1, 0, 0, false),
  ]);
  assert.ok(isClosed(edges));
  const walls = edges.filter(e => e.kind === "wall");
  assert.equal(walls.length, 1);
  assert.ok(lengthOf(walls[0]) > 1.9 && Math.abs(walls[0].ay) < 0.011 && Math.abs(walls[0].by) < 0.011);
  const rough = edges.filter(e => e.kind === "rough");
  assert.ok(rough.length >= 2 && rough.every(e => e.ax > 1.6 && e.bx > 1.6));
  // The guess goes from the last hit round to the first: simplified to its corner.
  const guess = edges.filter(e => e.kind === "guess");
  assert.ok(guess.length >= 2 && guess.length <= 4, `${guess.length}`);
  assert.ok(Math.abs(guess.reduce((sum, e) => sum + lengthOf(e), 0) - 2.75) < 0.1);
});

test("two hits far apart are not joined by a hit edge: an opening may be between them", () => {
  const edges = edgesOf([...run(0, 0, 1, 0, true), ...run(1.5, 0, 2.5, 0, true), ...run(2.5, 0, 2.5, 1, false), ...run(2.5, 1, 0, 1, false), ...run(0, 1, 0, 0, false)]);
  const across = edges.find(e => Math.abs(e.ay) < 1e-9 && Math.abs(e.by) < 1e-9 && e.ax < 1 && e.bx > 1.4)!;
  assert.equal(across.kind, "guess");
  assert.equal(edges.filter(e => e.kind === "wall").length, 2);
});

test("a circle nothing was seen round: a closed guess; one point: a dot", () => {
  const circle = Array.from({ length: 44 }, (_, i) => ({ x: 0.35 * Math.cos(i * Math.PI / 22), y: 0.35 * Math.sin(i * Math.PI / 22), hit: false }));
  const edges = edgesOf(circle);
  assert.ok(isClosed(edges) && edges.every(e => e.kind === "guess"));
  assert.ok(edges.length >= 6 && edges.length <= 16, `${edges.length}`);
  assert.deepEqual(edgesOf([{ x: 1, y: 2, hit: true }]), [{ ax: 1, ay: 2, bx: 1, by: 2, kind: "rough" }]);
  assert.deepEqual(edgesOf([]), []);
});

test("a knot of many small edges on a room of few long walls: the knot goes, the room stays", () => {
  // A 2 m square of hits; on its top side the border makes a jagged curl of guess points that crosses itself.
  const curl: BorderPoint[] = Array.from({ length: 16 }, (_, i) => ({ x: 1.3 - 0.04 * i + (i % 2 ? 0.25 : -0.25), y: 2 + (i % 4 < 2 ? 0.2 : 0.05), hit: false }));
  const edges = edgesOf([...run(0, 0, 2, 0, true), ...run(2, 0, 2, 2, true), ...run(2, 2, 1.4, 2, true), ...curl, ...run(0.6, 2, 0, 2, true), ...run(0, 2, 0, 0, true)]);
  assert.ok(isClosed(edges));
  assert.ok(edges.filter(e => e.kind === "wall").length >= 4);
  assert.ok(Math.abs(edges.reduce((sum, e) => sum + e.ax * e.by - e.bx * e.ay, 0)) / 2 > 3.5);
});

test("a loop that cuts through itself is cut at the crossing: closed, no crossing left", () => {
  // A square whose border makes a small bow before its second corner.
  const edges = edgesOf([...run(0, 0, 2, 0, false), { x: 2, y: 0.5, hit: false }, { x: 2.3, y: 0.8, hit: false }, { x: 1.7, y: 0.8, hit: false },
                         { x: 2, y: 1.1, hit: false }, ...run(2, 2, 0, 2, false), ...run(0, 2, 0, 0, false)]);
  assert.ok(isClosed(edges));
  const side = (e: Edge, x: number, y: number) => (e.bx - e.ax) * (y - e.ay) - (e.by - e.ay) * (x - e.ax);
  for (const a of edges) for (const b of edges)
    assert.ok(!(side(a, b.ax, b.ay) * side(a, b.bx, b.by) < -1e-9 && side(b, a.ax, a.ay) * side(b, a.bx, a.by) < -1e-9));
});
