// Tests for sights.ts: a frame's zones as flat wedges, through the real helper (the
// robot's tof_rays): where a wedge points, how far it saw through, its heights, what
// counts as a hit (an obstacle; not floor, not "beyond"), and a hit's bands. Run with
// `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { BANDS, heightsAt, inBand, sightsOf, type Sight } from "./sights.ts";
import { CellMaps } from "../cell_map.ts";
import { Store } from "../../store.ts";
import { loadMounting } from "../../mounting.ts";
import { WIRE_VERSION } from "../../wire.ts";
import { zoneAt, type PosedFrame, type Zone } from "../../tof_format.ts";

const DEG = Math.PI / 180, BOOT = "0000beef";
const mounting = loadMounting(), ZONE = mounting.tof.zone_deg * DEG, Z = mounting.tof.z_m;
const close = (a: number, b: number, eps = 2e-3) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);

// A frame at (x, y, yaw): every zone reads mm (sure); `change` alters single zones.
function frame(mm: number, pose: { x?: number; y?: number; yaw?: number; still?: boolean } = {}, change: (z: Zone, row: number, col: number) => void = () => {}): PosedFrame {
  const zones = Array.from({ length: 64 }, (_, k): Zone => {
    const z: Zone = { distanceMm: mm, signal: 50, sigmaMm: 5, kind: "sure", targets: 1 };
    change(z, Math.floor(k / 8) + 1, (k % 8) + 1);
    return z;
  });
  return { number: 1, robotClockUs: 100000, periodUs: 100000, x: pose.x ?? 0, y: pose.y ?? 0, yaw: pose.yaw ?? 0, pitch: 0, turn: 0,
           still: pose.still ?? true, poseKnown: true, beyondMm: 2550, zones };
}

// The frame's sights, read by row and column (1-8).
async function sights(f: PosedFrame): Promise<{ all: (Sight | null)[]; at: (row: number, col: number) => Sight | null }> {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-sights-"));
  const store = new Store(path.join(directory, "test.duckdb")), maps = new CellMaps(store);
  try {
    await store.openBoot({ version: WIRE_VERSION, bootId: BOOT, build: "test" }, mounting);
    const all = sightsOf(f, await maps.rays(BOOT, f), ZONE);
    return { all, at: (row, col) => all[zoneAt(row, col)] };
  } finally { maps.close(); rmSync(directory, { recursive: true, force: true }); }
}

test("a wedge: from the sensor, one zone wide, along its zone; it saw through to a margin before the reading", async () => {
  const rows = await sights(frame(1000));
  assert.equal(rows.all.length, 64);
  const s = rows.at(4, 4)!; // row 4, column 4: half a zone up, half a zone left of straight ahead
  close(s.x, mounting.tof.x_m);
  close(s.y, mounting.tof.y_m);
  close(s.z, Z);
  close(s.bearing, ZONE / 2);
  close(s.halfWidth, ZONE / 2);
  // On the floor plan the reading is 1 m * cos(half a zone up) away; the margin there is a zone width.
  const range = Math.cos(ZONE / 2);
  close(s.hit!.rangeM, range);
  close(s.reachM, range - range * ZONE);
  close(Math.hypot(s.hit!.x - s.x, s.hit!.y - s.y), range);
  // Its heights: row 4 spans level to one zone up; the helper's rays are at 1/6 and 5/6 of it.
  close(s.hit!.lowM, Z + Math.sin(ZONE / 6));
  close(s.hit!.highM, Z + Math.sin(5 * ZONE / 6));
  // Half way there the wedge is half as far above the sensor.
  const [low, high] = heightsAt(s, range / 2);
  close(low, Z + Math.sin(ZONE / 6) / 2);
  close(high, Z + Math.sin(5 * ZONE / 6) / 2);
  assert.equal(s.weight, 1);
  // Columns go left to right: column 1 looks furthest left.
  assert.ok(rows.at(4, 1)!.bearing > rows.at(4, 8)!.bearing);
});

test("the robot elsewhere, turned: the wedge follows; moving or less sure weighs less", async () => {
  const s = (await sights(frame(1000, { x: 1, y: 2, yaw: 90 * DEG, still: false }, z => { z.kind = "less_sure"; }))).at(4, 4)!;
  close(s.x, 1 - mounting.tof.y_m);
  close(s.y, 2 + mounting.tof.x_m);
  close(s.bearing, 90 * DEG + ZONE / 2);
  assert.equal(s.weight, 0.25);
});

test("what counts as a hit: an obstacle; floor, beyond and unusable zones don't", async () => {
  const rows = await sights(frame(400, {}, (z, row, col) => {
    if (row === 4 && col === 1) z.distanceMm = 2550;           // beyond: at least that far
    if (row === 4 && col === 2) z.kind = "unsure";             // its distance tells nothing
    if (row === 4 && col === 3) { z.kind = "nothing"; z.distanceMm = 0; }
  }));
  assert.ok(rows.at(4, 4)!.hit);
  // Close by the margin is 5 cm, not a zone width (4 cm at 40 cm).
  close(rows.at(4, 4)!.reachM, 0.4 * Math.cos(ZONE / 2) - 0.05);
  // Beyond: no hit, but it saw through 2.55 m less the margin.
  const beyond = rows.at(4, 1)!;
  assert.equal(beyond.hit, null);
  assert.ok(beyond.reachM > 2.2 && beyond.reachM < 2.4, String(beyond.reachM));
  assert.equal(rows.at(4, 2), null);
  assert.equal(rows.at(4, 3), null);
  // Row 8 at 40 cm reads the floor: no hit, and it saw through up to a margin before it.
  const floor = rows.at(8, 4)!;
  assert.equal(floor.hit, null);
  assert.ok(floor.reachM > 0.2 && floor.reachM < 0.4, String(floor.reachM));
  // No pose: nothing.
  assert.ok((await sights({ ...frame(400), poseKnown: false })).all.every(s => s === null));
});

test("a hit goes to each band its zone reaches, with its heights cut to the band", async () => {
  const rows = await sights(frame(1000)), row4 = rows.at(4, 4)!, row1 = rows.at(1, 4)!;
  const [l1, l2, above] = BANDS.map(b => inBand([row4, row1], b));
  // Row 4 at 1 m covers about 10.6 to 17.1 cm: L1 and L2, not above.
  const low = row4.hit!.lowM, high = row4.hit!.highM;
  assert.ok(low > 0.10 && low < 0.13 && high > 0.13 && high < 0.23, `${low} ${high}`);
  assert.deepEqual([l1[0].hit!.lowM, l1[0].hit!.highM], [low, 0.13]);
  assert.deepEqual([l2[0].hit!.lowM, l2[0].hit!.highM], [0.13, high]);
  assert.equal(above[0].hit, null);
  // Row 1 at 1 m is about 40 cm up: only above. What it saw through is the same in every band.
  assert.equal(l1[1].hit, null);
  assert.equal(l2[1].hit, null);
  assert.deepEqual(above[1].hit, row1.hit);
  assert.equal(l1[1].reachM, row1.reachM);
});
