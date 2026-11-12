// Tests for the simulator (doc/OUTLINE_MAP_PLAN.md, step S): the room's rays, the
// robot's two poses and their drift, the ToF's readings, bands and noise, and whole
// scenarios through the real sights and map code. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { CellMaps } from "../maps/cell_map.ts";
import { Store } from "../store.ts";
import { WIRE_VERSION } from "../wire.ts";
import { noFirmware } from "../test_store.ts";
import { BANDS, inBand, sightsOf, type Sight } from "../maps/outline/sights.ts";
import { loadMounting } from "../mounting.ts";
import { zoneAt } from "../tof_format.ts";
import { box, cast, facesAt, wall, type Room } from "./room.ts";
import { drive, MEASURED_DRIFT, movesOf, NO_DRIFT, randomOf, type Moment, type Pose } from "./drive.ts";
import { ALL_NOISE, NO_NOISE, tofFrame, ZONE_RAD, type Noise } from "./tof.ts";
import { run, SCENARIOS, type Scenario } from "./scenarios.ts";
import { play } from "./play.ts";
import { closed, floorError, offFaces, poseError, wallErrors } from "./measure.ts";
import { sureOf, type Wall } from "../maps/outline/walls.ts";

const DEG = Math.PI / 180, tof = loadMounting().tof;
const close = (a: number, b: number, eps = 1e-6) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const scenario = (name: string, change: Partial<Scenario> = {}): Scenario => ({ ...SCENARIOS.find(s => s.name === name)!, ...change });
const EXACT = { noise: NO_NOISE, drift: NO_DRIFT };

test("the room: a ray meets the nearest face at its height; a shape stands only in its steps", () => {
  const room: Room = [wall([2, -1, 2, 1]), box(1, -0.2, 1.3, 0.2, { highM: 0.1, untilStep: 3 })];
  const faces = facesAt(room, 0);
  assert.equal(faces.length, 5);
  close(cast(faces, 0, 0, 0, 5)!.d, 1);
  close(cast(faces, 0, 0, 0, 5, (_, f) => f.highM > 0.2)!.d, 2);  // a ray 20 cm up passes over the box
  assert.equal(cast(faces, 0, 0, 0, 0.9), null);
  assert.equal(cast(faces, 0, 0, Math.PI / 2, 5), null);
  close(cast(facesAt(room, 3), 0, 0, 0, 5)!.d, 2);                 // the box was taken away
});

test("the robot: moves as frames, still before and after; the truth parts from odometry by the drift", () => {
  assert.deepEqual(movesOf("r30 d50 r-90"), [{ turnRad: 30 * DEG }, { driveM: 0.5 }, { turnRad: -90 * DEG }]);
  assert.throws(() => movesOf("x5"), /not a move/);
  const [turn, ahead] = drive(movesOf("r30 d50"), NO_DRIFT, randomOf(1));
  // 30° at 29°/s is 11 frames; 2 still before, 3 after (the move's end and the still time).
  assert.deepEqual(turn.map(m => m.still), [true, true, ...Array(10).fill(false), true, true, true]);
  assert.equal(ahead.length, 2 + 49 + 3);
  const end = ahead.at(-1)!;
  assert.deepEqual(end.truth, end.odometry);
  close(end.odometry.yaw, 30 * DEG);
  close(end.odometry.x, 0.5 * Math.cos(30 * DEG));
  // A turn comes out 3 % too large, a drive 0.15 % too long.
  const scaled = drive(movesOf("r30 d100"), { ...NO_DRIFT, turnScale: 1.03, driveScale: 1.0015 }, randomOf(1)).at(-1)!.at(-1)!;
  close(scaled.odometry.yaw, 30 * DEG);
  close(scaled.truth.yaw, 30.9 * DEG);
  close(Math.hypot(scaled.truth.x, scaled.truth.y), 1.0015);
  // The wander is random but the same for the same seed.
  const wander = (seed: number) => drive(movesOf("r90 d100"), MEASURED_DRIFT, randomOf(seed)).at(-1)!.at(-1)!.truth;
  assert.deepEqual(wander(7), wander(7));
  assert.notDeepEqual(wander(7), wander(8));
});

// The sights of one frame taken at `truth` (odometry says `odometry`), by row and column (1-8).
function look(room: Room, noise: Noise = NO_NOISE, truth: Pose = { x: 0, y: 0, yaw: 0 }, odometry = truth, still = true, seed = 1) {
  const moment: Moment = { truth, odometry, still }, { frame, rays } = tofFrame(facesAt(room, 0), moment, 1, noise, randomOf(seed));
  const sights = sightsOf(frame, rays, ZONE_RAD);
  return { frame, sights, at: (row: number, col: number) => sights[zoneAt(row, col)]! };
}

test("the ToF without noise: the true distance along each zone, the floor, beyond; placed by odometry", () => {
  const facing: Room = [wall([1.5, -3, 1.5, 3])];
  const view = look(facing);
  assert.equal(view.sights.length, 64);
  // Row 4, column 4 looks half a zone up and half a zone left; the wall is 1.5 m ahead of the robot's middle.
  const s = view.at(4, 4);
  close(s.bearing, ZONE_RAD / 2);
  close(s.hit!.x, 1.5, 1e-3);
  close(s.hit!.rangeM, (1.5 - tof.x_m) / Math.cos(ZONE_RAD / 2), 1e-3);
  close(s.hit!.y, tof.y_m + (1.5 - tof.x_m) * Math.tan(ZONE_RAD / 2), 1e-3);
  // Rows 6-8 meet the floor before the wall: no hit, seen through. Row 5 meets the wall low: unsure, no hit.
  for (const row of [5, 6, 7, 8]) assert.equal(view.at(row, 4).hit, null, `row ${row}`);
  close(view.at(8, 4).reachM + 0.05, tof.z_m / Math.tan(3.5 * ZONE_RAD), 1e-3);
  // A wall 3 m away is beyond: no hit, seen through to the sensor's reach less the margin.
  const far = look([wall([3, -3, 3, 3])]).at(4, 4);
  assert.equal(far.hit, null);
  assert.ok(far.reachM > 2.2 && far.reachM < 2.54);
  // The robot truly stands turned 10° and 20 cm aside; odometry says it is at the origin:
  // the readings are the true ones, placed as seen from the origin.
  const turned = look(facing, NO_NOISE, { x: 0.2, y: 0, yaw: 10 * DEG }, { x: 0, y: 0, yaw: 0 }).at(4, 4);
  close(turned.x, tof.x_m);
  close(turned.bearing, ZONE_RAD / 2);
  assert.ok(turned.hit!.rangeM < s.hit!.rangeM - 0.15);
});

test("the ToF's bands: a 10 cm box is in L1 only, a table top in above only", () => {
  const inBands = (sights: (Sight | null)[]) => BANDS.map(band => inBand(sights.filter(s => s !== null), band).filter(s => s.hit).length);
  const [l1, l2, above] = inBands(look([box(0.7, -0.5, 1, 0.5, { highM: 0.1 })]).sights);
  assert.ok(l1 >= 8, `${l1}`);
  assert.deepEqual([l2, above], [0, 0]);
  const top = inBands(look([box(0.5, -0.5, 1.1, 0.5, { lowM: 0.3, highM: 0.33 })]).sights);
  assert.deepEqual(top.slice(0, 2), [0, 0]);
  assert.ok(top[2] >= 4, `${top[2]}`);
});

test("the ToF's noise: 1 cm steps, a few cm close and still, a smear far or moving, between two depths, unsure", () => {
  const facing = (at: number): Room => [wall([at, -3, at, 3])];
  const ranges = (noise: Partial<Noise>, at: number, still = true) => {
    const out: number[] = [];
    for (let seed = 1; seed <= 40; seed++) out.push(look(facing(at), { ...NO_NOISE, ...noise }, undefined, undefined, still, seed).at(4, 4).hit!.rangeM);
    return out;
  };
  const spread = (v: number[]) => { const mean = v.reduce((a, b) => a + b) / v.length; return Math.sqrt(v.reduce((a, b) => a + (b - mean) ** 2, 0) / v.length); };
  assert.ok(look(facing(0.777), { ...NO_NOISE, steps: true }).frame.zones.every(z => z.distanceMm % 10 === 0));
  assert.ok(spread(ranges({}, 0.8)) < 1e-9);
  const near = spread(ranges({ near: true, far: true }, 0.8)), far = spread(ranges({ near: true, far: true }, 1.6)), moving = spread(ranges({ near: true, far: true }, 0.8, false));
  assert.ok(near > 0.005 && near < 0.02, `${near}`);
  assert.ok(far > 0.04 && far < 0.1, `${far}`);
  assert.ok(moving > 0.04 && moving < 0.1, `${moving}`);
  // A box's corner just left of a zone's middle, the wall 1 m behind it: without noise the zone's middle ray
  // reads the wall; with it, somewhere between the two.
  const edge: Room = [wall([1.6, -3, 1.6, 3]), box(0.6, tof.y_m + (0.6 - tof.x_m) * Math.tan(0.6 * ZONE_RAD), 0.62, 1)];
  const between = (noise: Noise, seed: number) => { const h = look(edge, noise, undefined, undefined, true, seed).at(4, 4).hit!; return h.rangeM > 0.65 && h.rangeM < 1.45; };
  assert.equal(between(NO_NOISE, 1), false);
  let mixed = 0;
  for (let seed = 1; seed <= 20; seed++) if (between({ ...NO_NOISE, edges: true }, seed)) mixed++;
  assert.ok(mixed >= 10, `${mixed}`);
  // A slanted wall is one depth: it reads exactly with the edge noise on.
  const slanted: Room = [wall([0.5, -1.5, 3.5, 1.5])];
  assert.deepEqual(look(slanted, { ...NO_NOISE, edges: true }).frame.zones, look(slanted).frame.zones);
  // About 3 % of the readings are unsure: they give no hit.
  const unsure = look(facing(1), { ...NO_NOISE, missing: true }).frame.zones.filter(z => z.kind === "unsure").length;
  assert.ok(unsure >= 1 && unsure <= 6, `${unsure}`);
});

test("a frame's rays are the real helper's: where each zone ends, its heights, its verdict", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-sim-"));
  const store = new Store(path.join(directory, "test.duckdb")), maps = new CellMaps(store);
  try {
    await store.openBoot({ version: WIRE_VERSION, bootId: "0000beef", build: "test" }, loadMounting());
    // A box close by, a wall behind, the robot turned and off the origin; 1 cm steps, as a stored frame has.
    const room: Room = [wall([1.4, -3, 1.4, 3]), box(0.5, -0.1, 0.8, 0.6, { highM: 0.1 })];
    const pose = { x: 0.1, y: -0.2, yaw: 20 * DEG };
    const { frame, rays } = tofFrame(facesAt(room, 0), { truth: pose, odometry: pose, still: true }, 1, { ...NO_NOISE, steps: true }, randomOf(1));
    const real = await maps.rays("0000beef", frame);
    assert.deepEqual(rays.map(r => r!.verdict), real.map(r => r!.verdict));
    assert.ok(new Set(rays.map(r => r!.verdict)).size === 3); // the frame has all three verdicts
    rays.forEach((r, zone) => {
      const h = real[zone]!;
      for (const [a, b] of [...r!.point.map((v, i) => [v, h.point[i]]), ...r!.origin.map((v, i) => [v, h.origin[i]]), [r!.lowM, h.lowM], [r!.highM, h.highM]])
        close(a, b, 2e-3);
    });
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});

// What the outline measures against the truth after a scenario's last step, per band (L1, L2, above).
function measured(s: Scenario) {
  const played = play(s), last = played.at(-1)!, poses = played.flatMap(p => p.step.frames.filter((_, i) => i % 5 === 0).map(f => f.truth));
  return { played, last, bands: BANDS.map(band => ({ loops: last.bands[band.name], ...floorError(last.bands[band.name], last.step.faces, band, poses) })) };
}
const lengthOf = (e: { ax: number; ay: number; bx: number; by: number }) => Math.hypot(e.bx - e.ax, e.by - e.ay);

test("the square room without noise or drift: one closed outline a band, its walls on the true walls, its floor the room's", () => {
  const { last, bands } = measured(scenario("square", EXACT));
  assert.deepEqual(poseError(last.step.odometry, last.step.truth), { m: 0, rad: 0 });
  bands.forEach((band, i) => {
    assert.equal(band.loops.length, 1);
    assert.equal(band.loops[0].island, false);
    const walls = band.loops[0].edges.filter(e => e.kind === "wall");
    for (const e of walls) for (const [x, y] of [[e.ax, e.ay], [e.bx, e.by]]) assert.ok(offFaces(last.step.faces, x, y) < 0.01, `${x}, ${y}`);
    // L1's floor is seen to about 1.65 m only (by row 5): its walls stop short of the corners.
    assert.ok(walls.reduce((sum, e) => sum + lengthOf(e), 0) > (i === 0 ? 8.5 : 10), BANDS[i].name);
    assert.equal(band.wrongM2, 0);
    assert.ok(band.seen > (i === 0 ? 0.9 : 0.98), `${BANDS[i].name} ${band.seen}`);
  });
});

test("the box and the table without noise: the legs are islands in every band; the box is in L1 only", () => {
  const { bands } = measured(scenario("things", EXACT));
  const nearBox = (e: { ax: number; ay: number }) => e.ax > 0.6 && e.ax < 1.1 && e.ay > 0.2 && e.ay < 0.7;
  bands.forEach((band, i) => {
    assert.ok(band.loops.filter(l => l.island).length >= 4, BANDS[i].name);
    assert.equal(band.loops.flatMap(l => l.edges).some(e => e.kind !== "guess" && nearBox(e)), i === 0, BANDS[i].name);
  });
});

test("every scenario, with all noise and drift: at every step each band's loops are closed and never cross", () => {
  for (const s of SCENARIOS)
    for (const { bands } of play(s))
      for (const band of BANDS) {
        const loops = bands[band.name];
        assert.deepEqual(closed(loops), { open: 0, crossings: 0 }, `${s.name} ${band.name}`);
        assert.equal(loops.filter(l => !l.island).length, 1, `${s.name} ${band.name}`);
      }
});

test("the outline with noise: it covers most of the floor the robot saw, and little else", () => {
  for (const name of ["square", "loop"]) {
    const { bands } = measured(scenario(name, { drift: NO_DRIFT }));
    bands.forEach((band, i) => {
      assert.ok(band.seen > 0.8, `${name} ${BANDS[i].name} seen ${band.seen}`);
      assert.ok(band.wrongM2 < 0.06 * band.areaM2, `${name} ${BANDS[i].name} wrong ${band.wrongM2} of ${band.areaM2}`);
    });
  }
});

test("through the door the outline grows; in the big room it ends in a guess at the sensor's reach", () => {
  const { played, bands } = measured(scenario("door", EXACT));
  const areaAt = (step: number) => floorError(played[step].bands.above, played[step].step.faces, BANDS[2], [played[0].step.truth]).areaM2;
  assert.ok(areaAt(11) > 8 && areaAt(11) < 10.5, `${areaAt(11)}`);   // the first room, and a little through the door
  assert.ok(bands[2].areaM2 > 17, `${bands[2].areaM2}`);                // both rooms: 18 m2
  const big = measured(scenario("big")).bands[2];
  assert.ok(big.loops[0].edges.every(e => e.kind === "guess"));
  for (const e of big.loops[0].edges) assert.ok(Math.hypot(e.ax, e.ay) > 1.9 && Math.hypot(e.ax, e.ay) < 2.6, `${Math.hypot(e.ax, e.ay)}`);
});

// Each band's walls after the scenario's last step: every face of the room found once, every wall on a face within offM and rad.
// Step B's claims: the pose is not checked against the walls.
function wallsFound(s: Scenario, bands: typeof BANDS, offM: number, rad: number) {
  const last = play(s, { correct: false }).at(-1)!;
  for (const band of bands) {
    const { errors, perFace } = wallErrors(last.walls[band.name], last.step.faces, band), what = `${s.name} ${band.name}`;
    assert.deepEqual(perFace, [1, 1, 1, 1], what);
    for (const e of errors) {
      assert.ok(e.face, `${what}: wall ${e.wall.number} is on no face`);
      assert.ok(e.offM < offM && e.rad < rad, `${what}: wall ${e.wall.number} ${e.offM} m, ${e.rad / DEG}°`);
    }
  }
}

test("the walls without noise or drift: each of the room's four walls found once, within 1 cm and 1°", () => {
  for (const name of ["square", "loop", "box-stays"]) wallsFound(scenario(name, EXACT), BANDS, 0.01, 1 * DEG);
});

test("the walls with noise, no drift: in the band above each wall found once, within 5 cm and 2°", () => {
  for (const name of ["square", "loop", "box-stays"]) wallsFound(scenario(name, { drift: NO_DRIFT }), [BANDS[2]], 0.05, 2 * DEG);
  // things finds 3 of its 4 since step C's rules for hits near a wall's end and in corners (doc/OUTLINE_MAP_PLAN.md).
  const things = play(scenario("things", { drift: NO_DRIFT }), { correct: false }).at(-1)!;
  assert.ok(wallErrors(things.walls.above, things.step.faces, BANDS[2]).perFace.length >= 3);
});

test("without the pose check, turning makes the pose less sure: the walls seen later are less sure; the edges carry their wall's number", () => {
  const last = play(scenario("square", EXACT), { correct: false }).at(-1)!, sure = last.walls.above.map(sureOf);
  assert.equal(sure.length, 4);
  for (let i = 1; i < sure.length; i++) assert.ok(sure[i] < sure[i - 1], JSON.stringify(sure));
  assert.ok(sure[0] > 0.9 && sure[3] < 0.5, JSON.stringify(sure));
  const edges = last.bands.above.flatMap(l => l.edges).filter(e => e.wall);
  assert.deepEqual([...new Set(edges.map(e => e.wall!.number))].sort(), last.walls.above.map(w => w.number).sort());
});

test("the pose check, without noise or drift: the pose stays exact all the way, in every scenario", () => {
  for (const sc of SCENARIOS)
    for (const { step, corrected } of play(scenario(sc.name, EXACT))) {
      const off = poseError(corrected, step.truth);
      assert.ok(off.m < 0.01 && off.rad < 0.1 * DEG, `${sc.name}: ${off.m} m, ${off.rad / DEG}°`);
    }
});

test("the pose check, with the measured drift and all noise: the pose ends far closer to the truth than odometry", () => {
  // Turning on the spot: the heading to a third of odometry's (11° and 22° off).
  for (const name of ["square", "things", "box-stays"]) {
    const last = play(scenario(name)).at(-1)!, odometry = poseError(last.step.odometry, last.step.truth), corrected = poseError(last.corrected, last.step.truth);
    assert.ok(corrected.rad < odometry.rad / 3 && corrected.m < 0.05, `${name}: ${corrected.m} m, ${corrected.rad / DEG}° (odometry ${odometry.rad / DEG}°)`);
  }
  // Driving: the heading to half of odometry's, the position to 0.6 (loop 27 cm and 12°, door 58 cm and 22°).
  for (const name of ["loop", "door"]) {
    const last = play(scenario(name)).at(-1)!, odometry = poseError(last.step.odometry, last.step.truth), corrected = poseError(last.corrected, last.step.truth);
    assert.ok(corrected.m < 0.6 * odometry.m && corrected.rad < odometry.rad / 2, `${name}: ${corrected.m} m, ${corrected.rad / DEG}° (odometry ${odometry.m} m, ${odometry.rad / DEG}°)`);
  }
});

test("the pose check, with noise but no drift: it may push the pose off a little, never far", () => {
  // A known weak spot (doc/OUTLINE_MAP_PLAN.md): a still robot's frames count as new each time.
  for (const sc of SCENARIOS)
    for (const { step, corrected } of play(scenario(sc.name, { drift: NO_DRIFT }))) {
      const off = poseError(corrected, step.truth);
      assert.ok(off.m < 0.25 && off.rad < 7 * DEG, `${sc.name}: ${off.m} m, ${off.rad / DEG}°`);
    }
});

test("the pose check, without noise or drift: the square room's walls end at its corners", () => {
  const last = play(scenario("square", EXACT)).at(-1)!, corners = [[1.5, 1.5], [1.5, -1.5], [-1.5, 1.5], [-1.5, -1.5]];
  assert.equal(last.walls.above.length, 4);
  for (const w of last.walls.above)
    for (const k of [-1, 1]) {
      const x = w.x + k * w.halfM * Math.cos(w.angle), y = w.y + k * w.halfM * Math.sin(w.angle);
      assert.ok(Math.min(...corners.map(([cx, cy]) => Math.hypot(x - cx, y - cy))) < 0.01, `wall ${w.number}: ${x}, ${y}`);
    }
});

// The board of "board-removed" and "board-stays": a wall at x = 1.1 (sim/scenarios.ts).
const onBoard = (w: Wall) => Math.abs(w.x - 1.1) < 0.08 && Math.abs(Math.cos(w.angle)) < 0.3;

test("removal, without noise or drift: the board's walls go the first time the robot looks there with the pose fixed; staying, they stay", () => {
  const removed = play(scenario("board-removed", EXACT)), stays = play(scenario("board-stays", EXACT)).at(-1)!;
  assert.deepEqual(BANDS.map(b => removed[11].walls[b.name].filter(onBoard).length), [1, 1, 1]);
  // It goes at step 12, while the robot turns past the empty place (steps 12 and 13).
  assert.deepEqual(BANDS.map(b => removed[13].walls[b.name].filter(onBoard).length), [0, 0, 0]);
  assert.deepEqual(removed.at(-1)!.removed.map(r => r.band), ["L1", "L2", "above"]);
  assert.ok(removed.at(-1)!.removed.every(r => onBoard(r.wall)));
  assert.deepEqual(BANDS.map(b => stays.walls[b.name].filter(onBoard).length), [1, 1, 1]);
  assert.equal(stays.removed.length, 0);
});

test("removal, with all noise and the measured drift: no wall that still stands is removed, in any scenario; the board goes", () => {
  for (const sc of SCENARIOS) {
    const played = play(scenario(sc.name));
    for (const r of played.at(-1)!.removed) {
      const step = played.findIndex(p => p.removed.includes(r));
      const band = BANDS.find(b => b.name === r.band)!;
      assert.equal(wallErrors([r.wall], played[step].step.faces, band).errors[0].face, null, `${sc.name}: ${r.band} wall ${r.wall.number} removed at step ${step}, but it stands`);
    }
    if (sc.name === "board-removed") assert.deepEqual(BANDS.map(b => played.at(-1)!.walls[b.name].filter(onBoard).length), [0, 0, 0]);
  }
});

test("with the measured drift a full turn ends about 11° off; odometry doesn't see it", () => {
  const last = run(scenario("square", { noise: NO_NOISE })).at(-1)!;
  close(last.odometry.yaw, 2 * Math.PI, 1e-9);
  const off = poseError(last.odometry, last.truth);
  assert.ok(off.rad > 9 * DEG && off.rad < 13 * DEG, `${off.rad / DEG}`);
  assert.ok(off.m < 0.05);
});

test("every scenario runs with all noise and drift, the same each time", () => {
  for (const s of SCENARIOS) {
    assert.deepEqual(s.noise, ALL_NOISE);
    const steps = run(s);
    assert.equal(steps.length, movesOf(s.moves).length, s.name);
    assert.ok(steps.every(step => step.frames.length >= 5 && step.frames.every(f => f.sights.length > 32)), s.name);
    assert.deepEqual(run(s).at(-1)!.frames.at(-1)!.frame, steps.at(-1)!.frames.at(-1)!.frame, s.name);
    // The true robot keeps clear of everything: its half width (11 cm) and a margin.
    for (const step of steps) for (const f of step.frames) assert.ok(offFaces(step.faces, f.truth.x, f.truth.y) > 0.2, `${s.name}: the robot meets a thing`);
  }
  // The same seed gives the same drive whatever the noise.
  assert.deepEqual(run(scenario("loop", { noise: NO_NOISE })).at(-1)!.truth, run(scenario("loop")).at(-1)!.truth);
  // The floor read while moving gives no phantom things: the big room has no hit closer than 50 cm.
  const near = run(scenario("big")).flatMap(step => step.frames.flatMap(f => f.sights)).filter(sight => sight.hit && sight.hit.rangeM < 0.5);
  assert.equal(near.length, 0);
  // The box of "box-removed" stands in the first look round only.
  const removed = run(scenario("box-removed"));
  assert.equal(removed[11].faces.length, removed[12].faces.length + 4);
});
