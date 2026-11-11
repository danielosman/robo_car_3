// Tests for the PC's line maps at a snapshot (index.ts), through the real store and
// helper: a wall in the frames stored before a snapshot is a line in the bands its rows
// reach; before those frames there is none; the picture (api/map.ts) draws the lines
// over the cells, or alone. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import zlib from "node:zlib";
import { lineMapsAt } from "./index.ts";
import { CellMaps } from "../cell_map.ts";
import { cellMapAt } from "../../snapshots_api.ts";
import { mapPng } from "../../api/map.ts";
import { loadMounting } from "../../mounting.ts";
import { noFirmware, snapshotAgain, storeWithSnapshots } from "../../test_store.ts";
import type { PosedFrame, Zone } from "../../tof_format.ts";

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

test("a boot's lines at a snapshot: a wall 50 cm ahead, in each band; none before its frames; the picture", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-linemap-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store);
  try {
    // Snapshots 1-3 come first; then five frames facing a wall 50 cm away; then snapshot 3 again.
    for (let n = 1; n <= 5; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: frame(n, 500) }));
    await snapshotAgain(store, boot, directory, 3);

    const bands = (await lineMapsAt(store, maps, boot.id, 3))!;
    assert.deepEqual(bands.map(b => b.band), ["L1", "L2", "above"]);
    // Rows 1-4 at 50 cm reach from about 10 cm to 26 cm up: every band has a line there,
    // about 35 cm wide. Every zone reads 50 cm along its own ray, so the hits lie on a
    // bowl 42 to 50 cm ahead of the sensor (x_m ahead of the robot's middle).
    const sensorX = loadMounting().tof.x_m;
    for (const b of bands) {
      assert.equal(b.lines.length, 1, b.band);
      const l = b.lines[0];
      for (const x of [l.ax, l.bx]) assert.ok(x > sensorX + 0.42 && x < sensorX + 0.5, `${b.band} ${x}`);
      assert.ok(Math.abs(Math.abs(l.ay - l.by) - 0.35) < 0.06, `${b.band} ${l.ay} ${l.by}`);
    }
    // Before the frames: nothing. No such snapshot: null.
    assert.ok((await lineMapsAt(store, maps, boot.id, 1))!.every(b => b.lines.length === 0));
    assert.equal(await lineMapsAt(store, maps, boot.id, 99), null);

    // The picture: 6 pixels a 10 cm column, x up, y to the left; the window covers ±3 m.
    const view = (await cellMapAt(store, maps, boot.id, 3))!;
    const at = (x: number, y: number) => [Math.round((3 - y) * 60), Math.round((3 - x) * 60)] as const;
    const WHITE = [0xff, 0xff, 0xff], UNKNOWN = [0x15, 0x15, 0x15];
    const cells = pixelsOf(mapPng(view)), both = pixelsOf(mapPng(view, { lines: bands })), alone = pixelsOf(mapPng(view, { cells: false, lines: bands }));
    assert.equal(both.width, cells.width);
    assert.equal(alone.width, cells.width);
    // On the wall, a little to the left of the robot's heading line: the L1 line (white) over the cells.
    const wall = at(bands[0].lines[0].ax, 0.1);
    assert.notDeepEqual(cells.rgb(...wall), WHITE);
    assert.deepEqual(both.rgb(...wall), WHITE);
    assert.deepEqual(alone.rgb(...wall), WHITE);
    // Beside the robot: its cells with them, nothing without.
    const floor = at(0, 0.2);
    assert.notDeepEqual(both.rgb(...floor), UNKNOWN);
    assert.deepEqual(both.rgb(...floor), cells.rgb(...floor));
    assert.deepEqual(alone.rgb(...floor), UNKNOWN);
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});
