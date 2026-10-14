// Tests for snapshot.ts and snapshots_api.ts: the firmware's own snapshots
// (build/test_snapshot.snap, written by ./run_tests.sh) put back together from
// pieces of any size and read; a damaged one refused; the map's columns from the
// window's cells and the tiles' 4-bit columns; the PNG; and the snapshots stored in
// DuckDB and served as the page asks: the boots, the timeline, one snapshot with
// its 8 x 8 grid, the map then (the newest of each tile up to it, since the last
// clear). Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import zlib from "node:zlib";
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import {
  COLUMN, SNAP_FILE_MAGIC, SnapshotAssembler, columnOf, grayPng, parseSnapshot, readSnapshotFile, tileColumns, windowColumns,
} from "./snapshot.ts";
import { Store } from "./store.ts";
import { importSnapshots } from "./import.ts";
import { listBoots, listSnapshots, snapshotDetail, snapshotMap } from "./snapshots_api.ts";

const FIRMWARE = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../build/test_snapshot.snap");
const have = existsSync(FIRMWARE);
const firmware = () => readFileSync(FIRMWARE);

test("the firmware's snapshots, put together from pieces of any size, read as written", { skip: !have && "run ./run_tests.sh first" }, () => {
  const bytes = firmware(), a = new SnapshotAssembler(), whole: Buffer[] = [];
  whole.push(...a.push(Buffer.from("xx garbage SNA"))); // bytes before a snapshot are skipped
  for (let at = 0, n = 1; at < bytes.length; at += n, n = (n * 7) % 3001 + 1) whole.push(...a.push(bytes.subarray(at, at + n)));
  assert.equal(whole.length, 5);
  assert.ok(a.skipped >= 14);
  const s = whole.map(parseSnapshot);
  assert.deepEqual(s.map(x => x.seq), [1, 2, 3, 4, 5]);
  assert.deepEqual(s.map(x => x.why), ["server connected", "command", "command", "server connected", "command"]);
  const first = s[0];
  assert.equal(first.mode, "manual");
  assert.equal(first.endReason, null);
  assert.equal(first.pose!.x, 1.25);
  assert.equal(first.pose!.tUs, 4242);
  assert.ok(Math.abs(first.tof!.x - 1.2) < 1e-6 && first.tof!.poseOk);
  assert.equal(first.tof!.raw.zones.length, 64);
  assert.equal(first.tof!.raw.zones[5].targets.length, 2);
  assert.equal(first.tof!.raw.zones[5].targets[1].distanceMm, 501);
  assert.equal(first.geometry!.rows, 8);
  assert.equal(first.camera!.width * first.camera!.height, first.camera!.pixels.length);
  assert.equal(first.camera!.pixels[10], (10 * 13) & 255);
  assert.equal(first.color!.raw[15], 0xaf);
  assert.equal(first.window!.orgX, -30);
  assert.equal(first.tiles.length, 2);
  assert.equal(first.features!.cameraNumber, 77);
  assert.deepEqual(first.features!.features[0], { id: 9, age: 3, flags: 0, x: 12.5, y: 100.25, dx: -0.5, dy: 0.75, strength: 128 / 255, match: 230 / 255 });
  assert.equal(first.features!.features[1].flags, 1);
  assert.equal(s[1].action, "move");
  assert.equal(s[1].endReason, "obstacle");
  assert.equal(s[2].running, "goto");
  assert.equal(s[2].mode, "idle");
  assert.equal(s[3].tiles.length + s[4].tiles.length, 42);
  // Damaged: refused.
  const bad = Buffer.from(whole[0]);
  bad[100] ^= 1;
  assert.throws(() => parseSnapshot(bad), /CRC/);
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
  const dir = mkdtempSync(path.join(os.tmpdir(), "snaps-"));
  try {
    const snaps = new SnapshotAssembler().push(firmware());
    const file = path.join(dir, "snapshots", "test.snap");
    const parts: Buffer[] = [Buffer.from(SNAP_FILE_MAGIC)];
    snaps.forEach((b, i) => { const t = Buffer.alloc(8); t.writeDoubleLE(Date.UTC(2026, 9, 12, 10, 0, i), 0); parts.push(t, b); });
    const damaged = Buffer.from(snaps[1]);
    damaged[50] ^= 1;
    parts.push(Buffer.alloc(8), damaged);
    const { mkdirSync } = await import("node:fs");
    mkdirSync(path.dirname(file), { recursive: true });
    writeFileSync(file, Buffer.concat(parts));
    assert.equal([...readSnapshotFile(readFileSync(file))].length, 6);
    const store = new Store(path.join(dir, "test.duckdb"));
    assert.deepEqual(await importSnapshots(store, file, dir), { stored: 5, damaged: 1 });
    assert.deepEqual(await importSnapshots(store, file, dir), { stored: 5, damaged: 1 }); // again: replaced, not added
    const boot = parseSnapshot(snaps[0]).bootId;
    assert.ok(existsSync(path.join(dir, "camera", boot, "1.png")));

    const boots = await listBoots(store);
    assert.equal(boots.length, 1);
    assert.equal(Number(boots[0].snapshots), 5);
    const list = await listSnapshots(store, boot);
    assert.deepEqual(list.map(r => Number(r.seq)), [1, 2, 3, 4, 5]);
    assert.equal(list[1].action, "move");
    assert.equal(list[0].has_image, true);

    const one = (await snapshotDetail(store, boot, 1))!;
    assert.equal(one.snapshot.why, "server connected");
    assert.equal(one.snapshot.image, path.join("camera", boot, "1.png"));
    assert.equal(one.targets.length, Array.from({ length: 64 }, (_, z) => z % 3).reduce((a, b) => a + b, 0));
    assert.ok(one.targets.every(t => t.x !== null));
    // Zone 5 (the sensor's own number) has two targets; the grid shows each zone's first sure one.
    const filled = one.grid.flat().filter(c => c).length;
    assert.equal(filled, Array.from({ length: 64 }, (_, z) => (z % 3 ? 1 : 0)).reduce((a: number, b) => a + b, 0));
    assert.equal(await snapshotDetail(store, boot, 99), null);
    assert.equal(one.features.length, 2);
    assert.equal(one.features[0].x, 12.5);

    // The map then: snapshot 1 has the two first tiles; snapshot 3 changed one (newest wins);
    // 5 has every tile the robot has (42).
    const m1 = (await snapshotMap(store, boot, 1))!;
    assert.equal(m1.window!.orgX, -30);
    assert.equal(m1.window!.columns[30 * 60 + 31], COLUMN.BLOCKED);
    assert.equal(m1.tiles.length, 2);
    const t1 = m1.tiles.find(t => t.tx === -2)!;
    assert.equal(t1.columns[5 * 20 + 10], COLUMN.BLOCKED);
    const m3 = (await snapshotMap(store, boot, 3))!;
    const t3 = m3.tiles.find(t => t.tx === -2)!;
    assert.equal(t3.seq, 3);
    assert.equal(t3.columns[5 * 20 + 10], COLUMN.OPEN);
    assert.equal((await snapshotMap(store, boot, 5))!.tiles.length, 42);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});
