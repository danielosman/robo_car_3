// Tests for cell_map.ts, the PC's cell map (the robot's map code in the helper): a
// zone's reading placed in the world with the boot's mounting (hand-computed cases); a
// boot's map at a snapshot built from the frames stored before it: a wall ahead, a drive
// stopped by the robot's own rule, the whole-house tiles once the robot has gone far,
// back in time and forward again, the same after the server starts again. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { CellMaps, type CellMapView } from "./cell_map.ts";
import { Store } from "../store.ts";
import { loadMounting, type Mounting } from "../mounting.ts";
import { WIRE_VERSION } from "../wire.ts";
import { windowColumns, tileColumns, COLUMN } from "../snapshot.ts";
import { noFirmware, snapshotAgain, storeWithSnapshots } from "../test_store.ts";
import type { PosedFrame, Zone } from "../tof_format.ts";

const DEG = Math.PI / 180;
const close = (a: number, b: number, eps = 2e-3) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const now = loadMounting();
// A mounting from before the PCB was lifted: the ToF at 7 cm.
const before: Mounting = { ...now, tof: { ...now.tof, z_m: 0.07 } };

// A frame at pose (x, y, yaw, pitch): rows 1-4 read mm (sure; 0: unsure, they tell
// nothing), rows 5-8 unsure.
const frame = (number: number, mm: number, x = 0, y = 0, yaw = 0, pitch = 0): PosedFrame => ({
  number, robotClockUs: number * 100000, periodUs: 100000, x, y, yaw, pitch, turn: 0, still: true, poseKnown: true, beyondMm: 2550,
  zones: Array.from({ length: 64 }, (_, k): Zone => ({ distanceMm: mm, signal: 50, sigmaMm: 5, kind: k < 32 && mm ? "sure" : "unsure", targets: 1 })),
});

async function withStore(fn: (store: Store, maps: CellMaps) => Promise<void>): Promise<void> {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-cellmap-"));
  const store = new Store(path.join(directory, "test.duckdb"));
  const maps = new CellMaps(store);
  try { await fn(store, maps); } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
}

test("rays: each zone's reading in the world, with the boot's own mounting", () => withStore(async (store, maps) => {
  await store.openBoot({ version: WIRE_VERSION, bootId: "0000beef", build: "test" }, before);
  const Z = 0.07, h = 2.8125 * DEG, zone = (row: number, col: number) => (row - 1) * 8 + col - 1;
  // Row 4, column 4 (1-8): half a zone up and half a zone left of straight ahead.
  let r = (await maps.rays("0000beef", frame(1, 1000)))[zone(4, 4)]!;
  assert.equal(r.verdict, "obstacle");
  close(r.point[0], before.tof.x_m + Math.cos(h) * Math.cos(h));
  close(r.point[1], before.tof.y_m + Math.cos(h) * Math.sin(h));
  close(r.point[2], Z + Math.sin(h));
  // The robot at (1, 2), turned 90° left: forward is +y, left is -x.
  r = (await maps.rays("0000beef", frame(1, 1000, 1, 2, 90 * DEG)))[zone(4, 4)]!;
  close(r.point[0], 1 - before.tof.y_m - Math.cos(h) * Math.sin(h));
  close(r.point[1], 2 + before.tof.x_m + Math.cos(h) * Math.cos(h));
  // Nose up by half a zone: row 4's ray rises by a whole zone.
  r = (await maps.rays("0000beef", frame(1, 1000, 0, 0, 0, h)))[zone(4, 4)]!;
  close(r.point[2], Z + Math.sin(2 * h));
  // Unsure zones and frames without a pose: nothing.
  assert.equal((await maps.rays("0000beef", frame(1, 1000)))[zone(5, 4)], null);
  assert.ok((await maps.rays("0000beef", { ...frame(1, 1000), poseKnown: false })).every(z => z === null));
}));

// A world column of a map: the window's inside it, its tiles' outside.
function column(v: CellMapView, cx: number, cy: number): number {
  if (cx >= v.originX && cx < v.originX + v.cellsX && cy >= v.originY && cy < v.originY + v.cellsY)
    return windowColumns({ ...v, layers: 3 }).columns[(cx - v.originX) * v.cellsY + (cy - v.originY)];
  const t = v.tiles.find(t => t.tx === Math.floor((cx + 10) / 20) && t.ty === Math.floor((cy + 10) / 20));
  return t ? tileColumns(t)[(cx - (t.tx * 20 - 10)) * 20 + (cy - (t.ty * 20 - 10))] : COLUMN.UNKNOWN;
}

test("a boot's map at a snapshot: a wall ahead, a drive stopped; tiles; back in time; after a restart", { skip: noFirmware }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-cellmap-"));
  const { store, boot } = await storeWithSnapshots(directory);
  const maps = new CellMaps(store);
  try {
    // Snapshots 1-3 come first; then frames at the origin facing a wall 50 cm ahead, then
    // snapshot 5; then frames 3.6 m away (the window follows), then snapshot 4.
    for (let n = 1; n <= 5; n++) void store.tof(boot, boot.tof({ format: "compact", move: 1, tUs: n * 100000, frame: frame(n, 500) }));
    await snapshotAgain(store, boot, directory, 5);
    for (let n = 6; n <= 8; n++) void store.tof(boot, boot.tof({ format: "compact", move: 2, tUs: n * 100000, frame: frame(n, 0, 3.6) }));
    await snapshotAgain(store, boot, directory, 4);
    const drive = (sequence: number, metres: number) => maps.atSnapshot(boot.id, sequence, m => m.drive(0, 0, 0, metres));

    // At 5: blocked columns from x 50 cm; a 1 m drive stops 15 cm before them, measured from
    // its front edge (9.5 cm from the centre); one backwards doesn't.
    const ahead = (await drive(5, 1))!;
    assert.ok(ahead.obstacle && ahead.movedM > 0.15 && ahead.movedM < 0.3, JSON.stringify(ahead));
    assert.deepEqual(await drive(5, -1), { movedM: 1, obstacle: false });
    assert.deepEqual(await drive(5, 0.1), { movedM: 0.1, obstacle: false });
    assert.equal(await maps.atSnapshot(boot.id, 5, m => m.obstacleWithin(0, 0, 0.3)), false);
    assert.equal(await maps.atSnapshot(boot.id, 5, m => m.obstacleWithin(0.4, 0, 0.3)), true);
    const at5 = (await maps.viewAtSnapshot(boot.id, 5))!;
    assert.deepEqual([at5.view.originX, at5.view.originY, at5.view.cellsX, at5.view.cellsY, at5.view.cells.length, at5.view.tiles.length],
                     [-30, -30, 60, 60, 3 * 3600, 0]);
    assert.equal(column(at5.view, 5, 0), COLUMN.BLOCKED);   // x 50-60 cm
    assert.equal(column(at5.view, 0, 0), COLUMN.DRIVABLE);  // under the robot
    // At 4: the window moved by two tiles; the wall is in the whole-house tiles now.
    const at4 = (await maps.viewAtSnapshot(boot.id, 4))!.view;
    assert.equal(at4.originX, 10);
    assert.ok(at4.tiles.length > 0);
    assert.equal(column(at4, 5, 0), COLUMN.BLOCKED);
    // Back in time: before any frame nothing ahead; then forward again, the same.
    assert.deepEqual(await drive(1, 1), { movedM: 1, obstacle: false });
    assert.deepEqual(await drive(5, 1), ahead);
    assert.equal(await maps.atSnapshot(boot.id, 99, async () => 1), null);
    // A new server: the same map from the store.
    const again = new CellMaps(store);
    try { assert.deepEqual(await again.atSnapshot(boot.id, 5, m => m.drive(0, 0, 0, 1)), ahead); } finally { again.close(); }
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
});

test("an unknown boot: an error", () => withStore(async (_store, maps) => {
  await assert.rejects(maps.rays("00000000", frame(1, 500)), /no boot 00000000/);
}));
