// Tests for walls.ts and pose.ts, on hand-made sights: a wall is made from an outline's
// wall edge, moved by the hits seen on it (much while unsure, little once sure), turned by
// their slope, lengthened to them; hits far off it are a new thing; two walls that grow
// into each other are one; a border point close to a wall is put on it; the pose's doubt
// grows with turning and driving. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { sureOf, Walls, type Wall } from "./walls.ts";
import { grown, NO_DOUBT, type Pose } from "./pose.ts";
import type { Edge } from "./shape.ts";
import { BANDS, type Sight } from "./sights.ts";
import { DEG, row, sight, ZONE } from "./test_sights.ts";

const AT: Pose = { x: 0, y: 0, yaw: 0 }, ABOVE = BANDS[2]; // the test rows' hits are 19-29 cm up at 1 m
const close = (a: number, b: number, eps: number) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
// An outline's wall edge along the line x = at, from y = from to y = to.
const edge = (at: number, from: number, to: number): Edge => ({ ax: at, ay: from, bx: at, by: to, kind: "wall" });
// One frame of one row looking along x from the origin, its hits on the line x = at + slope·y.
const facing = (at: number, slope = 0): Sight[] => row(b => at / (Math.cos(b) - slope * Math.sin(b)));
// A frame's hits along edges, seen from the origin, every 5 cm (found() fits a new wall's line to them).
const along = (edges: Edge[]): Sight[] => edges.flatMap(e => {
  const n = Math.round(Math.hypot(e.bx - e.ax, e.by - e.ay) / 0.05);
  return Array.from({ length: n + 1 }, (_, i) => { const x = e.ax + (e.bx - e.ax) * i / n, y = e.ay + (e.by - e.ay) * i / n; return sight(Math.atan2(y, x), Math.hypot(x, y)); });
});
// The x where the wall crosses y = 0.
const xAtMiddle = (w: Wall) => w.x - w.y / Math.tan(w.angle);
const see = (walls: Walls, frames: number, sights: () => Sight[]) => { for (let i = 0; i < frames; i++) walls.see(i, AT, NO_DOUBT, sights()); };

test("a new wall from a wall edge; the hits seen on it move it, make it surer, lengthen it; its sightings", () => {
  const walls = new Walls(ABOVE);
  const [made] = walls.found([edge(1, -0.3, 0.3), { ...edge(1, 0.3, 1), kind: "rough" }], 7, AT, NO_DOUBT, along([edge(1, -0.3, 0.3), { ...edge(1, 0.3, 1), kind: "rough" }]));
  assert.equal(walls.all().length, 1);
  assert.deepEqual([made.number, made.firstFrame], [1, 7]);
  close(made.halfM, 0.3, 1e-9);
  const sureAtFirst = sureOf(made);
  // The wall truly stands 3 cm farther. A row at 1 m has hits up to ±37 cm.
  const [sighting] = walls.see(8, AT, NO_DOUBT, facing(1.03));
  assert.equal(sighting.wall, 1);
  assert.equal(sighting.hits.length, 8);
  close(sighting.offM!, -0.03, 0.002); // the hits lay 3 cm off, on its far side (x = 1 runs along +y: left is -x)
  assert.ok(xAtMiddle(walls.all()[0]) > 1.01, `${xAtMiddle(walls.all()[0])}`);
  see(walls, 20, () => facing(1.03));
  const w = walls.all()[0];
  close(xAtMiddle(w), 1.03, 0.005);
  assert.ok(sureOf(w) > sureAtFirst && sureOf(w) > 0.9, `${sureOf(w)}`);
  close(w.halfM, 1.03 * Math.tan(3.5 * ZONE), 0.01); // to the outer hits
  assert.equal(w.sightings, 21);
});

test("a sure wall moves little, an unsure one much; a slope turns it", () => {
  const sure = new Walls(ABOVE), unsure = new Walls(ABOVE);
  for (const walls of [sure, unsure]) walls.found([edge(1, -0.4, 0.4)], 0, AT, NO_DOUBT, along([edge(1, -0.4, 0.4)]));
  see(sure, 30, () => facing(1));
  sure.see(31, AT, NO_DOUBT, facing(1.05));
  unsure.see(31, AT, NO_DOUBT, facing(1.05));
  assert.ok(Math.abs(xAtMiddle(sure.all()[0]) - 1) < 0.005, `${xAtMiddle(sure.all()[0])}`);
  assert.ok(xAtMiddle(unsure.all()[0]) - 1 > 0.02, `${xAtMiddle(unsure.all()[0])}`);
  // The wall truly leans 3°: x = 1 + tan(3°)·y.
  const turned = new Walls(ABOVE);
  turned.found([edge(1, -0.4, 0.4)], 0, AT, NO_DOUBT, along([edge(1, -0.4, 0.4)]));
  see(turned, 20, () => facing(1, Math.tan(3 * DEG)));
  close(Math.abs(Math.PI / 2 - turned.all()[0].angle), 3 * DEG, 0.3 * DEG);
});

test("hits far off a wall are not on it (a thing in front of it); an unsure pose widens the gate only a little", () => {
  const walls = new Walls(ABOVE);
  walls.found([edge(1, -0.4, 0.4)], 0, AT, NO_DOUBT, along([edge(1, -0.4, 0.4)]));
  assert.deepEqual(walls.see(1, AT, NO_DOUBT, facing(0.8)), []);
  assert.deepEqual(walls.see(2, AT, { xyM: 1, yawRad: 1 }, facing(0.8)), []);
  close(xAtMiddle(walls.all()[0]), 1, 1e-9);
  // An edge on the known wall is not a new wall; one 30 cm in front of it is.
  assert.deepEqual(walls.found([edge(1.05, 0.3, 0.9)], 3, AT, NO_DOUBT, along([edge(1.05, 0.3, 0.9)])), []);
  assert.equal(walls.found([edge(0.7, -0.4, 0.4)], 3, AT, NO_DOUBT, along([edge(0.7, -0.4, 0.4)])).length, 1);
});

test("two walls that grow into each other are one, with the older number", () => {
  // 40 cm apart: a frame's hits (at ±5, 15, 25, 36 cm) lengthen both to ±5 cm.
  const two = [edge(1, -1, -0.2), edge(1, 0.2, 1)], walls = new Walls(ABOVE);
  walls.found(two, 0, AT, NO_DOUBT, along(two));
  assert.equal(walls.all().length, 2);
  see(walls, 3, () => facing(1));
  const [w, ...rest] = walls.all();
  assert.equal(rest.length, 0);
  assert.equal(w.number, 1);
  close(w.halfM, 1, 0.01);
  assert.equal(walls.numberNow(2), 1);
  // Three in a chain, 25 cm apart: one frame's hits (at ±5, 15, 25, 36 cm) lengthen all three,
  // so they reach each other at once. One wall, number 1; 2 and 3 are 1 now.
  const chain = new Walls(ABOVE);
  const three = [edge(1, -1, -0.45), edge(1, -0.2, 0.2), edge(1, 0.45, 1)];
  chain.found(three, 0, AT, NO_DOUBT, along(three));
  assert.equal(chain.all().length, 3);
  chain.see(1, AT, NO_DOUBT, facing(1));
  assert.deepEqual(chain.all().map(w => w.number), [1]);
  close(chain.all()[0].halfM, 1, 0.01);
  assert.deepEqual([chain.numberNow(2), chain.numberNow(3)], [1, 1]);
});

test("a border point close to a wall is put on it; one farther off, or beyond its end, is not", () => {
  const walls = new Walls(ABOVE);
  walls.found([edge(1, -0.4, 0.4)], 0, AT, NO_DOUBT, along([edge(1, -0.4, 0.4)]));
  const on = walls.on(1.08, 0.1)!;
  assert.equal(on.wall.number, 1);
  close(on.x, 1, 1e-9);
  close(on.y, 0.1, 1e-9);
  assert.equal(walls.on(1.2, 0), null);
  assert.equal(walls.on(1, 0.7), null);
});

test("the pose's doubt grows with every turn and metre driven", () => {
  let doubt = NO_DOUBT, pose = AT;
  for (let i = 1; i <= 12; i++) { const next = { ...pose, yaw: i * 30 * DEG }; doubt = grown(doubt, pose, next); pose = next; }
  close(doubt.yawRad, 0.035 * 2 * Math.PI, 1e-9); // a full turn: about 12°, as the measured drift (3 %) gives 11°
  const driven = grown(NO_DOUBT, AT, { x: 1, y: 0, yaw: 0 });
  close(driven.xyM, 0.01, 1e-9);
  // Driving with an unsure heading puts the position off sideways too.
  assert.ok(grown(doubt, AT, { x: 1, y: 0, yaw: 0 }).xyM > 0.2);
});

test("a wall's end, first too long, moves to where the wedges show it ends, surer the nearer it is seen", () => {
  // A wall x = 1 truly ends at y = 0.2; past that the wedges see through to 2 m. The outline's edge said it goes to y = 0.5.
  const from = (x: number) => row(b => ((1 - x) * Math.tan(b) <= 0.2 ? (1 - x) / Math.cos(b) : null), 0, { x });
  const at = (x: number): Pose => ({ x, y: 0, yaw: 0 });
  const walls = new Walls(ABOVE);
  walls.found([edge(1, -0.6, 0.5)], 0, AT, NO_DOUBT, along([edge(1, -0.6, 0.2)]));
  const end = () => { const w = walls.all()[0]; return w.y + w.halfM * Math.sign(Math.sin(w.angle)); };
  close(end(), 0.5, 1e-9);
  // From 1.5 m the wedges are 15 cm apart along the wall: the last hit at 4 cm, the first past it at 19 cm.
  for (let i = 0; i < 5; i++) walls.see(i, at(-0.5), NO_DOUBT, from(-0.5));
  close(end(), 0.2, 0.08);
  const far = walls.all()[0].endSd[1];
  // From 1 m they are 10 cm apart (hits up to 15 cm, past from 25 cm): the end is surer, and closer to the truth.
  for (let i = 0; i < 5; i++) walls.see(10 + i, AT, NO_DOUBT, from(0));
  close(end(), 0.2, 0.03);
  assert.ok(walls.all()[0].endSd[1] < far, `${walls.all()[0].endSd[1]} vs ${far}`);
});

test("two walls that meet at more than 45° end where their lines cross: a corner", () => {
  const walls = new Walls(ABOVE);
  const two = [edge(1, -0.6, 0.7), { ax: -0.4, ay: 0.5, bx: 1.2, by: 0.5, kind: "wall" as const }];
  walls.found(two, 0, AT, NO_DOUBT, along(two));
  const [a, b] = walls.all();
  close(a.y + a.halfM, 0.5, 1e-6);      // x = 1 runs along +y: its far end
  close(b.x + b.halfM, 1, 1e-6);        // y = 0.5 runs along +x
  assert.ok(a.endSd[1] < 0.06 && b.endSd[1] < 0.06); // about the other wall's spread across (5 cm)
});
