// Tests for border.ts: the loops round a set of squares: one round a block, one more
// round each hole, the inside on the right; squares touching at a corner are apart.
// Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { bordersOf, connected, keyOf, type Crossing } from "./border.ts";

const set = (...squares: [number, number][]) => new Set(squares.map(([x, y]) => keyOf(x, y)));
const block = (x0: number, y0: number, x1: number, y1: number) => {
  const out: [number, number][] = [];
  for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) out.push([x, y]);
  return out;
};
// Twice the area a loop goes round: more than 0 anticlockwise (x right, y up).
const turn = (loop: Crossing[]) => loop.reduce((sum, c, i) => {
  const n = loop[(i + 1) % loop.length];
  return sum + (c.ix + c.dx / 2) * (n.iy + n.dy / 2) - (n.ix + n.dx / 2) * (c.iy + c.dy / 2);
}, 0);

test("one square: a loop of its four sides, clockwise; every crossing leaves the set", () => {
  const [loop, ...rest] = bordersOf(set([3, -2]));
  assert.equal(rest.length, 0);
  assert.equal(loop.length, 4);
  assert.ok(turn(loop) < 0);
  assert.deepEqual(new Set(loop.map(c => `${c.ix},${c.iy}>${c.dx},${c.dy}`)), new Set(["3,-2>1,0", "3,-2>0,1", "3,-2>-1,0", "3,-2>0,-1"]));
});

test("a block with a hole: the outer loop clockwise, the hole's anticlockwise; each step goes to a neighbour", () => {
  const squares = block(0, 0, 4, 3).filter(([x, y]) => !(x === 2 && y >= 1 && y <= 2));
  const loops = bordersOf(set(...squares)).sort((a, b) => b.length - a.length);
  assert.deepEqual(loops.map(l => l.length), [18, 6]);
  assert.ok(turn(loops[0]) < 0 && turn(loops[1]) > 0);
  for (const loop of loops)
    loop.forEach((c, i) => {
      const n = loop[(i + 1) % loop.length];
      assert.ok(Math.hypot(n.ix + n.dx / 2 - c.ix - c.dx / 2, n.iy + n.dy / 2 - c.iy - c.dy / 2) <= 1 + 1e-9);
    });
});

test("squares that touch only at a corner are not joined: two loops, two groups", () => {
  const squares = set([0, 0], [1, 1]);
  assert.deepEqual(bordersOf(squares).map(l => l.length), [4, 4]);
  assert.deepEqual([...connected(squares, keyOf(0, 0))], [keyOf(0, 0)]);
  assert.equal(connected(set(...block(0, 0, 2, 2), [5, 5]), keyOf(1, 1)).size, 9);
  assert.equal(connected(squares, keyOf(7, 7)).size, 0);
});
