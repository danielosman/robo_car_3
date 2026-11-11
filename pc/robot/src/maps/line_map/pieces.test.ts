// Tests for pieces.ts, on hand-made sights: a wall is one piece, a corner two, a leg in
// front of a wall three; a lone hit is a stub across its zone; zones without a hit break
// a run. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { piecesOf, type Piece } from "./pieces.ts";
import { row, ZONE } from "./test_sights.ts";

const close = (a: number, b: number, eps = 1e-3) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
// How far along its bearing a zone meets the wall x = d, or the wall y = d.
const wallX = (d: number) => (bearing: number) => d / Math.cos(bearing);
const wallY = (d: number) => (bearing: number) => d / Math.sin(bearing);
const lengthOf = (p: Piece) => Math.hypot(p.bx - p.ax, p.by - p.ay);

test("a flat wall is one piece, from the left zone's border to the right zone's", () => {
  const [wall, ...rest] = piecesOf(row(wallX(1)));
  assert.equal(rest.length, 0);
  assert.equal(wall.points.length, 8);
  close(wall.ax, 1);
  close(wall.bx, 1);
  // 8 zones at 1 m: the outer hits at ±tan(3.5 zones), and half a zone's width beyond each.
  const outer = Math.tan(3.5 * ZONE), half = Math.tan(ZONE / 2) / Math.cos(3.5 * ZONE);
  close(wall.ay, outer + half);
  close(wall.by, -outer - half);
  assert.deepEqual([wall.lowM, wall.highM, wall.weight], [0.2, 0.3, 1]);
  // A close hit pulls a fitted line more than a far one: 1 / its zone's width squared.
  close(wall.points[3].fitWeight, 1 / (2 * Math.tan(ZONE / 2) / Math.cos(ZONE / 2)) ** 2, 0.01);
  assert.ok(wall.points[3].fitWeight > wall.points[0].fitWeight);
});

test("a corner is two pieces that share the corner's hit", () => {
  // A room's corner 1 m away along column 4 (its farthest hit); the walls at 45° to that.
  const tip = 0.5 * ZONE, pieces = piecesOf(row(b => 1 / (Math.cos(b - tip) + Math.sin(Math.abs(b - tip)))));
  assert.deepEqual(pieces.map(p => p.points.length), [4, 5]);
  const [left, right] = pieces;
  assert.deepEqual(left.points.at(-1), right.points[0]);
  close(left.points.at(-1)!.x, Math.cos(tip));
  // The two meet at the corner's hit: neither goes on behind the other wall.
  assert.deepEqual([left.bx, left.by], [right.ax, right.ay]);
  close(left.bx, Math.cos(tip));
  // Each side runs at 45° to the corner's bearing: the two are at right angles.
  close((left.bx - left.ax) * (right.bx - right.ax) + (left.by - left.ay) * (right.by - right.ay), 0);
});

test("a wall seen at a shallow angle breaks into stubs: its neighbours differ too much in distance", () => {
  // A side wall 15 cm to the left, seen by columns 1 and 2 at 20° and 14°.
  const pieces = piecesOf(row((b, col) => (col <= 2 ? wallY(0.15)(b) : null)));
  assert.deepEqual(pieces.map(p => p.points.length), [1, 1]);
});

test("a leg in front of a wall: the wall, the leg as a stub across its zone, the wall again", () => {
  const pieces = piecesOf(row((b, col) => (col === 4 ? 0.5 : wallX(1.5)(b))));
  assert.deepEqual(pieces.map(p => p.points.length), [3, 1, 4]);
  const leg = pieces[1], bearing = 0.5 * ZONE;
  close(lengthOf(leg), 2 * 0.5 * Math.tan(ZONE / 2));
  // Across its wedge, the left end first.
  close((leg.bx - leg.ax) * Math.cos(bearing) + (leg.by - leg.ay) * Math.sin(bearing), 0);
  assert.ok(leg.ay > leg.by);
  close((leg.ax + leg.bx) / 2, 0.5 * Math.cos(bearing));
});

test("zones without a hit break a run; a noisy wall stays one piece", () => {
  const broken = piecesOf(row((b, col) => (col === 3 ? null : wallX(1)(b))));
  assert.deepEqual(broken.map(p => p.points.length), [2, 5]);
  assert.deepEqual(piecesOf([null, ...row(() => null)]), []);
  // 1 cm steps of the distance (the compact format) don't split a wall.
  const noisy = piecesOf(row((b, col) => wallX(1)(b) + (col % 2 ? 0.01 : -0.01)));
  assert.equal(noisy.length, 1);
  // A piece may be as far off as its loosest point: here a share of the outer zone's width.
  assert.ok(noisy[0].slackM > 0.03 && noisy[0].slackM < 0.05, String(noisy[0].slackM));
});
