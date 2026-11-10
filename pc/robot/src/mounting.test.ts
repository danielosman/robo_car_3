// Tests for mounting.ts: common/mounting.json reads and passes its checks; a missing or
// wrong field is refused, naming it; the page gets it. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import type http from "node:http";
import { MOUNTING_FILE, loadMounting, parseMounting } from "./mounting.ts";
import { handleSnapshotApi } from "./snapshots_api.ts";
import type { Store } from "./store.ts";
import type { BootFolders } from "./boot_folders.ts";

const fileJson = () => JSON.parse(readFileSync(MOUNTING_FILE, "utf8"));

test("common/mounting.json reads: the ToF, the camera, the IMU", () => {
  const m = loadMounting();
  assert.ok(m.tof.z_m > 0 && m.camera.z_m > 0 && m.camera.f_px > 0);
  assert.equal(m.imu.robot_from_sensor.length, 3);
  assert.deepEqual(m, parseMounting(fileJson()));
});

test("a missing or wrong field is refused, naming it", () => {
  const without = (section: string, key: string) => { const j = fileJson(); delete j[section][key]; return j; };
  const withValue = (section: string, key: string, v: unknown) => { const j = fileJson(); j[section][key] = v; return j; };
  assert.throws(() => parseMounting(without("tof", "z_m"), "m.json"), /^Error: m\.json: tof\.z_m is missing$/);
  assert.throws(() => parseMounting(without("camera", "f_px")), /camera\.f_px is missing/);
  assert.throws(() => parseMounting({ ...fileJson(), camera: undefined }), /camera is missing/);
  assert.throws(() => parseMounting(withValue("tof", "x_m", "3.2 cm")), /tof\.x_m must be a number/);
  assert.throws(() => parseMounting(withValue("camera", "z_m", 0)), /camera\.z_m must be more than 0/);
  assert.throws(() => parseMounting(withValue("camera", "width_px", 160.5)), /camera\.width_px must be a whole number/);
  assert.throws(() => parseMounting(withValue("tof", "zones_turn_deg", 45)), /tof\.zones_turn_deg must be 0, 90, 180 or 270/);
  assert.throws(() => parseMounting(withValue("imu", "robot_from_sensor", [[1, 0, 0], [0, 1, 0]])), /imu\.robot_from_sensor must be 3 rows/);
  assert.throws(() => parseMounting({ ...fileJson(), imu: {} }), /imu\.robot_from_sensor is missing/);
  assert.throws(() => parseMounting([]), /not a JSON object/);
  assert.throws(() => parseMounting({ ...fileJson(), tof: undefined }), /tof is missing/);
  assert.throws(() => parseMounting(withValue("tof", "z_m", NaN)), /tof\.z_m must be a number/);
  assert.throws(() => parseMounting(withValue("camera", "f_px", Infinity)), /camera\.f_px must be a number/);
  assert.throws(() => parseMounting(withValue("camera", "width_px", 0)), /camera\.width_px must be more than 0/);
});

test("a file that doesn't read or parse says which file", () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), "robocar-mounting-"));
  try {
    const bad = path.join(directory, "mounting.json");
    writeFileSync(bad, "{");
    assert.throws(() => loadMounting(bad), new RegExp(`${bad.replace(/[.\\/]/g, "\\$&")} doesn't read`));
    assert.throws(() => loadMounting(path.join(directory, "none.json")), /none\.json doesn't read/);
  } finally { rmSync(directory, { recursive: true, force: true }); }
});

test("the page gets it: GET /api/mounting", async () => {
  let code = 0, body = "";
  const req = { method: "GET", url: "/api/mounting" } as http.IncomingMessage;
  const res = { writeHead: (c: number) => { code = c; }, end: (b: string) => { body = b; } } as unknown as http.ServerResponse;
  assert.ok(await handleSnapshotApi(req, res, {} as Store, "", {} as BootFolders));
  assert.equal(code, 200);
  assert.deepEqual(JSON.parse(body), loadMounting());
});
