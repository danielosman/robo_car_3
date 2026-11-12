// Tests for the PC's outline maps at a snapshot (index.ts), through the real store and
// helper: a wall in the frames stored before a snapshot is on each band's outline;
// before those frames the outline is the start circle; the picture (api/map.ts) draws
// the outlines over the cells, or alone. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import zlib from "node:zlib";
import { edgeRgb, outlinesAt, wallsAt } from "./index.ts";
import { CellMaps } from "../cell_map.ts";
import { cellMapAt } from "../../snapshots_api.ts";
import { mapPng } from "../../api/map.ts";
import { loadMounting } from "../../mounting.ts";
import { noFirmware, snapshotAgain, storeWithSnapshots } from "../../test_store.ts";
import { rowCol, type PosedFrame, type Zone } from "../../tof_format.ts";

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
  const maps = new CellMaps(store);
  try {
    // Snapshots 1-3 come first; then five frames facing a wall 50 cm away; then snapshot 3 again.
    for (let n = 1; n <= 5; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: frame(n, 500) }));
    await snapshotAgain(store, boot, directory, 3);

    const bands = (await outlinesAt(store, maps, boot.id, 3))!;
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
    assert.ok((await outlinesAt(store, maps, boot.id, 1))!.every(b => b.loops.length === 1 && b.loops[0].edges.every(e => e.kind === "guess")));
    assert.equal(await outlinesAt(store, maps, boot.id, 99), null);

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
  const maps = new CellMaps(store);
  try {
    // 30 frames facing a wall 1 m ahead of the sensor (new walls are looked for every 10 frames).
    for (let n = 1; n <= 30; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: facingWall(n, 1.0) }));
    await snapshotAgain(store, boot, directory, 3);
    const bands = (await wallsAt(store, maps, boot.id, 3))!;
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
    await wallsAt(store, maps, boot.id, 3);
    assert.deepEqual(await count(), first);
    assert.equal(await wallsAt(store, maps, boot.id, 99), null);
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});
