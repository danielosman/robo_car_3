// Tests for the PC's outline maps at a snapshot (index.ts), through the real store and
// helper: a wall in the frames stored before a snapshot is on each band's outline;
// before those frames the outline is the start circle; the picture (api/map.ts) draws
// the outlines over the cells, or alone; the kept mapper is fed only the frames since
// its last snapshot; the robot, path and snapshots come placed by the correction; the
// log line; with drift (the simulator's frames, stored), the correction used everywhere,
// also by the PC's cell map. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import zlib from "node:zlib";
import { edgeRgb, OutlineMaps } from "./index.ts";
import { corrected } from "./mapper.ts";
import { run, SCENARIOS } from "../../sim/scenarios.ts";
import { play } from "../../sim/play.ts";
import { CellMaps } from "../cell_map.ts";
import { cellMapAt } from "../../snapshots_api.ts";
import { mapPng } from "../../api/map.ts";
import { loadMounting } from "../../mounting.ts";
import { noFirmware, snapshotAgain, storeWithSnapshots } from "../../test_store.ts";
import { rowCol, type PosedFrame, type Zone } from "../../tof_format.ts";
import { COLUMN } from "../../snapshot.ts";

// A frame at the origin, still: rows 1-4 read mm (sure), rows 5-8 unsure.
const frame = (number: number, mm: number): PosedFrame => ({
  number, robotClockUs: number * 100000, periodUs: 100000, x: 0, y: 0, yaw: 0, pitch: 0, turn: 0, still: true, poseKnown: true, beyondMm: 2550,
  zones: Array.from({ length: 64 }, (_, k): Zone => ({ distanceMm: mm, signal: 50, sigmaMm: 5, kind: k < 32 ? "sure" : "unsure", targets: 1 })),
});

// The picture's pixels (RGB, rows top first) and its width. As snapshot.ts writes a PNG:
// one IDAT chunk right after IHDR, no row filter.
function pixelsOf(png: Buffer): { width: number; rgb: (px: number, py: number) => number[] } {
  const width = png.readUInt32BE(16), rows = zlib.inflateSync(png.subarray(41, 41 + png.readUInt32BE(33)));
  return { width, rgb: (px, py) => [...rows.subarray(py * (width * 3 + 1) + 1 + px * 3, py * (width * 3 + 1) + 4 + px * 3)] };
}

test("a boot's outlines at a snapshot: a wall 50 cm ahead, on each band's outline; the start circle before its frames; the picture", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-outline-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store), outlines = new OutlineMaps(store, maps);
  const outlinesAt = async (n: number) => (await outlines.at(boot.id, n))?.bands ?? null;
  try {
    // Snapshots 1-3 come first; then five frames facing a wall 50 cm away; then snapshot 3 again.
    for (let n = 1; n <= 5; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: frame(n, 500) }));
    await snapshotAgain(store, boot, directory, 3);

    const bands = (await outlinesAt(3))!;
    assert.deepEqual(bands.map(b => b.band), ["L1", "L2", "above"]);
    // Rows 1-4 at 50 cm reach from about 10 cm to 26 cm up: every band's outline has
    // edges backed by hits there, about 35 cm wide in all (too short for a wall: rough).
    // Every zone reads 50 cm along its own ray, so the hits lie on a bowl 42 to 50 cm
    // ahead of the sensor (x_m ahead of the robot's middle).
    const sensorX = loadMounting().tof.x_m;
    const hitEdges = (b: (typeof bands)[number]) => b.loops[0].edges.filter(e => e.kind !== "guess");
    for (const b of bands) {
      assert.equal(b.loops.length, 1, b.band);
      const edges = hitEdges(b);
      assert.ok(edges.length >= 1, b.band);
      for (const x of edges.flatMap(e => [e.ax, e.bx])) assert.ok(x > sensorX + 0.42 && x < sensorX + 0.5, `${b.band} ${x}`);
      const ys = edges.flatMap(e => [e.ay, e.by]);
      assert.ok(Math.abs(Math.max(...ys) - Math.min(...ys) - 0.35) < 0.06, b.band);
    }
    // Before the frames: the start circle, all guess. No such snapshot: null.
    assert.ok((await outlinesAt(1))!.every(b => b.loops.length === 1 && b.loops[0].edges.every(e => e.kind === "guess")));
    assert.equal(await outlinesAt(99), null);

    // The picture: 6 pixels a 10 cm column, x up, y to the left; the window covers ±3 m.
    const view = (await cellMapAt(store, maps, boot.id, 3))!;
    const at = (x: number, y: number) => [Math.round((3 - y) * 60), Math.round((3 - x) * 60)] as const;
    const first = hitEdges(bands[0])[0], WHITE = [...edgeRgb("L1", first)], UNKNOWN = [0x15, 0x15, 0x15];
    const cells = pixelsOf(mapPng(view)), both = pixelsOf(mapPng(view, { outlines: bands })), alone = pixelsOf(mapPng(view, { cells: false, outlines: bands }));
    assert.equal(both.width, cells.width);
    assert.equal(alone.width, cells.width);
    // On the wall, in the middle of an L1 edge: the L1 outline (white, as bright as its kind) over the cells.
    const wall = at((first.ax + first.bx) / 2, (first.ay + first.by) / 2);
    assert.notDeepEqual(cells.rgb(...wall), WHITE);
    assert.deepEqual(both.rgb(...wall), WHITE);
    assert.deepEqual(alone.rgb(...wall), WHITE);
    // Beside the robot, inside the start circle: its cells with them, nothing without.
    const floor = at(0, 0.2);
    assert.notDeepEqual(both.rgb(...floor), UNKNOWN);
    assert.deepEqual(both.rgb(...floor), cells.rgb(...floor));
    assert.deepEqual(alone.rgb(...floor), UNKNOWN);
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});

// A frame at the origin, still, facing a flat wall `flatM` ahead of the sensor: rows 1-4
// read it (sure), rows 5-8 unsure. A zone reads along its own ray, up and to the side.
const facingWall = (number: number, flatM: number): PosedFrame => {
  const deg = loadMounting().tof.zone_deg * Math.PI / 180;
  return { ...frame(number, 0), zones: Array.from({ length: 64 }, (_, k): Zone => {
    const [row, col] = rowCol(k), up = (4.5 - row) * deg, left = (4.5 - col) * deg;
    return { distanceMm: Math.round(flatM * 1000 / (Math.cos(up) * Math.cos(left))), signal: 50, sigmaMm: 5, kind: k < 32 ? "sure" : "unsure", targets: 1 };
  }) };
};

test("a boot's walls at a snapshot: a flat wall ahead, in L2 and above; the tables written, then replaced", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-walls-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store), outlines = new OutlineMaps(store, maps);
  try {
    // 30 frames facing a wall 1 m ahead of the sensor (new walls are looked for every 10 frames).
    for (let n = 1; n <= 30; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: facingWall(n, 1.0) }));
    await snapshotAgain(store, boot, directory, 3);
    const bands = (await outlines.walls(boot.id, 3))!;
    assert.deepEqual(bands.map(b => b.band), ["L1", "L2", "above"]);
    const wallX = loadMounting().tof.x_m + 1.0;
    // Rows 1-4 at 1 m reach from about 10 cm up to 44 cm: the wall is in every band. But
    // L1's floor is seen only by the lower rows (unsure here), so its outline never reaches the wall.
    assert.equal(bands[0].walls.length, 0);
    for (const { band, walls } of bands.slice(1)) {
      assert.equal(walls.length, 1, band);
      const w = walls[0];
      assert.ok(Math.abs(w.x - wallX) < 0.02 && Math.abs(Math.cos(w.angle)) < 0.05 && w.sightings >= 10, `${band} ${JSON.stringify(w)}`);
    }
    const count = async () => (await store.query(`SELECT (SELECT count(*) FROM walls) AS walls, (SELECT count(*) FROM wall_sightings) AS sightings,
      (SELECT count(*) FROM frame_poses) AS poses, (SELECT count(*) FROM wall_sightings s JOIN walls w USING (boot_id, band, wall)) AS joined`))[0];
    const first = await count();
    assert.deepEqual([Number(first.walls), Number(first.poses)], [2, 30]);
    assert.ok(Number(first.sightings) >= 20 && Number(first.joined) === Number(first.sightings), JSON.stringify(first));
    await outlines.walls(boot.id, 3);
    assert.deepEqual(await count(), first);
    assert.equal(await outlines.walls(boot.id, 99), null);
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});

test("the kept mapper: fed only the frames since its last snapshot; the robot, path and snapshots placed by the correction; the log; started again when a snapshot comes again, or after a failed feed", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-kept-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store), notes: string[] = [], outlines = new OutlineMaps(store, maps, t => notes.push(t));
  // How many frames the mapper is given, counted where it reads them.
  let read = 0;
  const framesBetween = store.framesBetween.bind(store);
  store.framesBetween = async (...a) => { const f = await framesBetween(...a); read += f.length; return f; };
  const tof = (from: number, to: number) => {
    for (let n = from; n <= to; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: facingWall(n, 1.0) }));
  };
  try {
    // Snapshot 2 after frames 1-15, snapshot 3 after frames 16-30.
    tof(1, 15);
    await snapshotAgain(store, boot, directory, 2);
    tof(16, 30);
    await snapshotAgain(store, boot, directory, 3);
    await outlines.added(boot.id, 2);
    assert.equal(read, 15);
    await outlines.added(boot.id, 3);
    assert.equal(read, 30);
    // Snapshot 2 again, after 3: kept, nothing read.
    const two = (await outlines.at(boot.id, 2))!, three = (await outlines.at(boot.id, 3))!;
    assert.equal(read, 30);
    assert.equal(two.path.length, 15);
    assert.equal(three.path.length, 30);
    // As a mapper built for snapshot 2 alone draws it.
    const alone = (await new OutlineMaps(store, maps).at(boot.id, 2))!;
    assert.deepEqual(two.bands, alone.bands);

    // The robot and the snapshots where the correction puts them; the path is the corrected poses.
    const snapshots = await store.snapshotsUpTo(boot.id, 3);
    for (const s of snapshots) assert.ok(s.pose);
    assert.deepEqual(three.robot, corrected(three.correction, snapshots[2].pose!));
    assert.deepEqual(three.snapshots.map(s => s.sequence), [1, 2, 3]);
    assert.deepEqual(three.snapshots[1], { sequence: 2, x: corrected(two.correction, snapshots[1].pose!).x, y: corrected(two.correction, snapshots[1].pose!).y });
    // Facing one wall, standing still, no drift: nothing to correct.
    for (const v of [two, three]) {
      assert.ok(Math.hypot(v.correction.x, v.correction.y) < 0.005 && Math.abs(v.correction.yaw) < 0.005, JSON.stringify(v.correction));
      for (const p of v.path) assert.ok(Math.hypot(p.x, p.y) < 0.005);
    }
    // One log line a snapshot that came.
    assert.equal(notes.length, 2);
    assert.match(notes[1], /^outline map, snapshot 3: pose corrected by \d+\.\d cm, -?\d+\.\d° in all; this move \d+\.\d cm, -?\d+\.\d°$/);

    // Snapshot 2 stored again after 10 more frames: the kept mapper starts again; as a fresh one draws it.
    tof(31, 40);
    await snapshotAgain(store, boot, directory, 2);
    const again = (await outlines.at(boot.id, 2))!;
    assert.equal(again.path.length, 40);
    assert.deepEqual(again, (await new OutlineMaps(store, maps).at(boot.id, 2))!);

    // A feed that fails part way (the helper gone): the next read builds the map again, no frame given twice.
    tof(41, 50);
    await snapshotAgain(store, boot, directory, 3);
    const rays = maps.rays.bind(maps);
    let calls = 0;
    maps.rays = async (...a) => { if (++calls === 5) throw new Error("the cell map helper stopped"); return rays(...a); };
    await assert.rejects(outlines.at(boot.id, 3), /helper stopped/);
    const after = (await outlines.at(boot.id, 3))!;
    assert.equal(after.path.length, 50);
    assert.deepEqual(after, (await new OutlineMaps(store, maps).at(boot.id, 3))!);
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});

test("a drive with drift (the simulator's square room, stored): the correction is used for the robot, the path, the log and the PC's cell map", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-drift-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store), plain = new CellMaps(store), notes: string[] = [], outlines = new OutlineMaps(store, maps, t => notes.push(t));
  maps.placeBy(outlines);
  try {
    // A full turn on the spot in 30° steps; odometry ends about 10° off. Half before snapshot 2, half before 3.
    const steps = run(SCENARIOS.find(s => s.name === "square")!), half = steps.length / 2;
    const storeSteps = (from: number, to: number) => {
      for (const { frame } of steps.slice(from, to).flatMap(s => s.frames))
        void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: frame.robotClockUs, frame }));
    };
    storeSteps(0, half);
    await snapshotAgain(store, boot, directory, 2);
    storeSteps(half, steps.length);
    await snapshotAgain(store, boot, directory, 3);
    await outlines.added(boot.id, 2);
    await outlines.added(boot.id, 3);
    const view = (await outlines.at(boot.id, 3))!, end = steps.at(-1)!, deg = (r: number) => Math.abs(r) * 180 / Math.PI;

    // The pose at the end: corrected near the truth, odometry not; as the simulator's own run corrects it.
    const last = view.path.length - 1, played = play(SCENARIOS.find(s => s.name === "square")!).at(-1)!.corrected;
    assert.equal(view.path.length, steps.flatMap(s => s.frames).length);
    assert.ok(deg(end.odometry.yaw - end.truth.yaw) > 5, "odometry drifted");
    assert.ok(deg(view.correction.yaw) > 3, JSON.stringify(view.correction));
    assert.ok(Math.hypot(view.path[last].x - played.x, view.path[last].y - played.y) < 0.01);
    const yawEnd = corrected(view.correction, end.odometry).yaw;
    assert.ok(deg(yawEnd - played.yaw) < 0.5 && deg(yawEnd - end.truth.yaw) < 4, `${deg(yawEnd - end.truth.yaw)}°`);

    // The robot and the snapshots placed by it (not odometry's place).
    const snapshots = await store.snapshotsUpTo(boot.id, 3), robot = corrected(view.correction, snapshots[2].pose!);
    assert.deepEqual(view.robot, robot);
    assert.ok(Math.hypot(robot.x - snapshots[2].pose!.x, robot.y - snapshots[2].pose!.y) > 0.02);
    // The log: the turn's correction, in degrees.
    const [, inAll] = /pose corrected by [\d.]+ cm, (-?[\d.]+)° in all/.exec(notes[1])!;
    assert.ok(Math.abs(Math.abs(Number(inAll)) - deg(view.correction.yaw)) < 0.1, notes[1]);

    // The PC's cell map: the snapshot's pose corrected, its cells placed by the corrected poses (not as without).
    const cells = (await maps.viewAtSnapshot(boot.id, 3))!, odometryCells = (await plain.viewAtSnapshot(boot.id, 3))!;
    assert.deepEqual(cells.pose, robot);
    assert.deepEqual(odometryCells.pose, snapshots[2].pose);
    assert.notDeepEqual(cells.view.columns, odometryCells.view.columns);
    // ... nearer the room's true walls (3 x 3 m round the start) than odometry's.
    const onWalls = (v: typeof cells.view) => {
      const blocked = v.columns.flatMap((c, i) => c === COLUMN.BLOCKED ? [[(v.originX + Math.floor(i / v.cellsY) + 0.5) * 0.1, (v.originY + i % v.cellsY + 0.5) * 0.1]] : []);
      return blocked.filter(([x, y]) => Math.min(Math.abs(Math.abs(x) - 1.5), Math.abs(Math.abs(y) - 1.5)) < 0.1).length / blocked.length;
    };
    assert.ok(onWalls(cells.view) > onWalls(odometryCells.view), `${onWalls(cells.view)} ${onWalls(odometryCells.view)}`);

    // robot check asks in the robot's own frame: the snapshot's odometry pose, a place
    // turned into the map's frame (a blocked column, found there).
    const i = cells.view.columns.findIndex(c => c === COLUMN.BLOCKED), v = cells.view;
    const [bx, by] = [(v.originX + Math.floor(i / v.cellsY) + 0.5) * 0.1, (v.originY + i % v.cellsY + 0.5) * 0.1];
    const c = view.correction, cos = Math.cos(c.yaw), sin = Math.sin(c.yaw);
    const own = { x: cos * (bx - c.x) + sin * (by - c.y), y: -sin * (bx - c.x) + cos * (by - c.y) };  // (bx, by) in the robot's frame
    assert.deepEqual(await maps.atSnapshot(boot.id, 3, async (m, pose) => [pose, await m.obstacleWithin(own.x, own.y, 0.06)]), [snapshots[2].pose, true]);
  } finally { maps.close(); plain.close(); rmSync(directory, { recursive: true, force: true }); }
});
