// Tests for snapshot.ts and snapshots_api.ts: the firmware's own snapshots
// (build/test_snapshot.snap, written by ./run_tests.sh) put back together from
// pieces of any size and read (the ToF section a compact frame); a damaged one or one
// of another version refused; the map's columns from the window's cells and the
// tiles' 4-bit columns; the PNG; and the snapshots stored in DuckDB and served as the
// page asks: the boots, the timeline, one snapshot with its zones (for the page each in
// the world, placed by the PC's cell map helper) and 8 x 8 grid, the robot's map then
// (the newest of each tile up to it, since the last clear). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import zlib from "node:zlib";
import { existsSync, mkdtempSync, readFileSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { COLUMN, SnapshotAssembler, columnOf, grayPng, parseSnapshot, tileColumns, windowColumns } from "./snapshot.ts";
import { listBoots, listSnapshots, snapshotDetail, snapshotForPage, snapshotMap } from "./snapshots_api.ts";
import { CellMaps } from "./maps/cell_map.ts";
import { loadMounting } from "./mounting.ts";
import { BootFolders, folderNameAt } from "./boot_folders.ts";
import { FIRMWARE_SNAPSHOTS, storeWithSnapshots } from "./test_store.ts";
import { rowCol, zoneAt } from "./tof_format.ts";

const have = existsSync(FIRMWARE_SNAPSHOTS);
const firmware = () => readFileSync(FIRMWARE_SNAPSHOTS);
// The test frame's zones (test_snapshot.c): sensor zone z has z % 3 targets, the first
// 100 * z mm away; the robot's zone at row, col reads the sensor's zone (7 - col) * 8
// + row (the grid turned 90° on the PCB), all 1-8 here less one.
const sensorZone = (zone: number) => { const [r, c] = rowCol(zone); return (8 - c) * 8 + r - 1; };

test("the firmware's snapshots, put together from pieces of any size, read as written", { skip: !have && "run ./run_tests.sh first" }, () => {
  const bytes = firmware(), a = new SnapshotAssembler(), whole: Buffer[] = [];
  whole.push(...a.push(Buffer.from("xx garbage SNA"))); // bytes before a snapshot are skipped
  for (let at = 0, n = 1; at < bytes.length; at += n, n = (n * 7) % 3001 + 1) whole.push(...a.push(bytes.subarray(at, at + n)));
  assert.equal(whole.length, 5);
  assert.ok(a.skipped >= 14);
  const s = whole.map(parseSnapshot);
  assert.deepEqual(s.map(x => x.sequence), [1, 2, 3, 4, 5]);
  assert.deepEqual(s.map(x => x.why), ["server connected", "command", "command", "server connected", "command"]);
  const first = s[0];
  assert.equal(first.mode, "manual");
  assert.equal(first.endReason, null);
  assert.equal(first.pose!.x, 1.25);
  assert.equal(first.pose!.robotClockUs, 4242);
  const tof = first.tof!;
  assert.ok(Math.abs(tof.x - 1.2) < 1e-6 && Math.abs(tof.yaw - 0.7) < 1e-4 && tof.poseKnown && tof.number === 1);
  assert.equal(tof.zones.length, 64);
  for (let k = 0; k < 64; k++) {
    const z = sensorZone(k), zone = tof.zones[k];
    assert.equal(zone.targets, z % 3);
    assert.equal(zone.kind, z % 3 ? "sure" : "nothing");
    if (z % 3) assert.equal(zone.distanceMm, Math.min(100 * z, 2550)); // whole cm; 2550: beyond 2.54 m
  }
  assert.equal(first.camera!.width * first.camera!.height, first.camera!.pixels.length);
  assert.equal(first.camera!.pixels[10], (10 * 13) & 255);
  assert.equal(first.color!.raw[15], 0xaf);
  assert.equal(first.window!.originX, -30);
  assert.equal(first.tiles.length, 2);
  assert.equal(first.features!.cameraNumber, 77);
  assert.deepEqual(first.features!.features[0], { id: 9, age: 3, flags: 0, x: 12.5, y: 100.25, dx: -0.5, dy: 0.75, strength: 128 / 255, match: 230 / 255 });
  assert.equal(first.features!.features[1].flags, 1);
  assert.equal(s[1].action, "drive");
  assert.equal(s[1].endReason, "obstacle");
  assert.equal(s[2].running, "goto");
  assert.equal(s[2].mode, "idle");
  assert.equal(s[3].tiles.length + s[4].tiles.length, 42);
  // Damaged: refused.
  const bad = Buffer.from(whole[0]);
  bad[100] ^= 1;
  assert.throws(() => parseSnapshot(bad), /CRC/);
  const old = Buffer.from(whole[0]); // an old robot's (version 1): refused, the layout changed
  old[4] = 1;
  old.writeUInt32LE(zlib.crc32(old.subarray(0, old.length - 4)) >>> 0, old.length - 4);
  assert.throws(() => parseSnapshot(old), /snapshot version 1, this server reads 2/);
});

test("columns: from the window's three cells, from a tile's 4 bits", { skip: !have && "run ./run_tests.sh first" }, () => {
  const OCC = 2 | 12 << 4, FREE = 1 | 9 << 4;
  assert.equal(columnOf(0, OCC, 0), COLUMN.BLOCKED);
  assert.equal(columnOf(0, FREE, OCC), COLUMN.OVERHANG);
  assert.equal(columnOf(FREE, 0, 0), COLUMN.NO_FLOOR);
  assert.equal(columnOf(OCC, FREE, FREE), COLUMN.DRIVABLE);
  assert.equal(columnOf(0, FREE, FREE), COLUMN.OPEN);
  assert.equal(columnOf(0, FREE, 0), COLUMN.UNKNOWN);
  const s = parseSnapshot(new SnapshotAssembler().push(firmware())[0]);
  const w = windowColumns(s.window!);
  assert.equal(w.columns[30 * 60 + 31], COLUMN.BLOCKED); // the test's blocked cell, 12 of 15 confident
  assert.equal(w.confidence[30 * 60 + 31], 12);
  assert.equal(w.columns.filter(c => c !== COLUMN.UNKNOWN).length, 1);
  const tile = s.tiles.find(t => t.tx === -2)!;
  assert.equal(tileColumns(tile)[5 * 20 + 10], COLUMN.BLOCKED); // column (-45, 0)
});

test("a grey PNG", () => {
  const pixels = Buffer.from(Array.from({ length: 12 }, (_, i) => i * 20));
  const png = grayPng(4, 3, pixels);
  assert.deepEqual([...png.subarray(0, 8)], [137, 80, 78, 71, 13, 10, 26, 10]);
  assert.equal(png.readUInt32BE(16), 4);
  assert.equal(png.readUInt32BE(20), 3);
  const idatAt = png.indexOf("IDAT"), len = png.readUInt32BE(idatAt - 4);
  const rows = zlib.inflateSync(png.subarray(idatAt + 4, idatAt + 4 + len));
  assert.deepEqual([...rows], [0, 0, 20, 40, 60, 0, 80, 100, 120, 140, 0, 160, 180, 200, 220]);
  for (let at = 8; at < png.length;) { // every chunk's CRC
    const n = png.readUInt32BE(at);
    assert.equal(png.readUInt32BE(at + 8 + n), zlib.crc32(png.subarray(at + 4, at + 8 + n)) >>> 0);
    at += 12 + n;
  }
});

test("stored in DuckDB, served as the page asks", { skip: !have && "run ./run_tests.sh first" }, async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "snaps-"));
  try {
    const { store, boot: b } = await storeWithSnapshots(directory);
    const boot = b.id;
    const folder = folderNameAt(Date.UTC(2026, 9, 14, 10, 0, 0)); // the boot's folder: when its first snapshot arrived
    assert.deepEqual(JSON.parse(readFileSync(path.join(directory, "boots.json"), "utf8")), { [boot]: folder });
    assert.ok(existsSync(path.join(directory, "camera", folder, "1.png")));
    // Stored again: replaced, not added.
    const again = parseSnapshot(new SnapshotAssembler().push(firmware())[0]);
    await store.importSnapshot(b, b.stamp().messageNumber, again, new SnapshotAssembler().push(firmware())[0], Date.UTC(2026, 9, 14, 10), null);
    assert.deepEqual(await store.query("SELECT count(*) AS n FROM snapshots"), [{ n: "5" }]);

    const boots = await listBoots(store, new BootFolders(directory));
    assert.equal(boots.length, 1);
    assert.equal(boots[0].folder, folder);
    assert.equal(Number(boots[0].snapshots), 5);
    const list = await listSnapshots(store, boot);
    assert.deepEqual(list.map(r => Number(r.sequence)), [1, 2, 3, 4, 5]);
    assert.equal(list[1].action, "drive");
    assert.equal(list[1].end_reason, "obstacle");
    assert.equal(list[2].has_image, true);

    const one = (await snapshotDetail(store, boot, 2))!;
    assert.equal(one.snapshot.why, "command");
    assert.equal(one.snapshot.image, path.join("camera", folder, "2.png"));
    assert.equal(one.zones.length, 64);
    // The grid: row 1, column 1 reads sensor zone 56: 56 % 3 = 2 targets, 5.6 m: beyond.
    assert.deepEqual(one.grid[0][0], { mm: 2550, kind: "sure", beyond: true });
    assert.equal(sensorZone(zoneAt(1, 8)), 0);
    assert.equal(one.grid[0][7], null); // row 1, column 8: sensor zone 0, no target
    assert.equal(one.grid.flat().filter(c => c).length, Array.from({ length: 64 }, (_, z) => (z % 3 ? 1 : 0)).reduce((a: number, c) => a + c, 0));
    assert.equal(await snapshotDetail(store, boot, 99), null);
    // For the page: each valid zone in the world (the helper's tof_rays, the frame's pose),
    // the others null; row 1 looks higher than row 8, column 1 further left than column 8.
    const maps = new CellMaps(store);
    try {
      const page = (await snapshotForPage(store, maps, boot, 2))!, m = loadMounting(), pose = one.snapshot;
      const seen = (z: Record<string, unknown>) => {
        const dx = Number(z.x) - Number(pose.tof_x), dy = Number(z.y) - Number(pose.tof_y), along = Math.hypot(dx, dy);
        return { up: Math.atan2(Number(z.z) - m.tof.z_m, along), left: Math.atan2(dy, dx) - Number(pose.tof_yaw) };
      };
      for (const z of page.zones) assert.equal(z.x === null, z.kind !== "sure");
      const valid = page.zones.filter(z => z.x !== null), mean = (zs: typeof valid, k: "up" | "left") => zs.reduce((a, z) => a + seen(z)[k], 0) / zs.length;
      assert.ok(mean(valid.filter(z => Number(z.row) === 1), "up") > mean(valid.filter(z => Number(z.row) === 8), "up"));
      assert.ok(mean(valid.filter(z => Number(z.col) === 1), "left") > mean(valid.filter(z => Number(z.col) === 8), "left"));
      assert.equal(await snapshotForPage(store, maps, boot, 99), null);
    } finally { maps.close(); }
    assert.equal(one.features.length, 2);
    assert.equal(one.features[0].x, 12.5);

    // The map then: snapshot 1 has the two first tiles; snapshot 3 changed one (newest wins);
    // 5 has every tile the robot has (42).
    const m1 = (await snapshotMap(store, boot, 1))!;
    assert.equal(m1.window!.originX, -30);
    assert.equal(m1.window!.columns[30 * 60 + 31], COLUMN.BLOCKED);
    assert.equal(m1.tiles.length, 2);
    const t1 = m1.tiles.find(t => t.tx === -2)!;
    assert.equal(t1.columns[5 * 20 + 10], COLUMN.BLOCKED);
    const m3 = (await snapshotMap(store, boot, 3))!;
    const t3 = m3.tiles.find(t => t.tx === -2)!;
    assert.equal(t3.sequence, 3);
    assert.equal(t3.columns[5 * 20 + 10], COLUMN.OPEN);
    assert.equal((await snapshotMap(store, boot, 5))!.tiles.length, 42);
  } finally { rmSync(directory, { recursive: true, force: true }); }
});
