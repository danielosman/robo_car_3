// The robot's mounting: where the ToF, the camera and the IMU sit (common/mounting.json,
// the same file the firmware's build reads). Robot frame: x forward from the centre,
// y left, z up from the floor; metres, degrees.
//   loadMounting() reads and checks the file; a missing or wrong field throws, naming it.
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

export const MOUNTING_FILE = fileURLToPath(new URL("../../../common/mounting.json", import.meta.url));

export interface Mounting {
  tof: { x_m: number; y_m: number; z_m: number; zone_deg: number; zones_turn_deg: number };
  camera: { x_m: number; y_m: number; z_m: number; f_px: number; tilt_deg: number; width_px: number; height_px: number };
  imu: { robot_from_sensor: number[][] };  // robot = robot_from_sensor × sensor, accelerometer and gyro
}

// Each section's fields: "number", "int" (whole), "positive" (more than 0), "count" (whole, more than 0).
const FIELDS = {
  tof: { x_m: "number", y_m: "number", z_m: "positive", zone_deg: "positive", zones_turn_deg: "int" },
  camera: { x_m: "number", y_m: "number", z_m: "positive", f_px: "positive", tilt_deg: "number", width_px: "count", height_px: "count" },
} as const;

// Checks a parsed mounting.json; `source` names it in the errors.
export function parseMounting(json: unknown, source = "mounting.json"): Mounting {
  const fail = (what: string): never => { throw new Error(`${source}: ${what}`); };
  const isObject = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
  if (!isObject(json)) fail("not a JSON object");
  const root = json as Record<string, unknown>;
  for (const [section, fields] of Object.entries(FIELDS)) {
    const s = root[section];
    if (!isObject(s)) fail(`${section} is missing`);
    for (const [key, kind] of Object.entries(fields)) {
      const v = (s as Record<string, unknown>)[key], name = `${section}.${key}`;
      if (v === undefined) fail(`${name} is missing`);
      if (typeof v !== "number" || !Number.isFinite(v)) fail(`${name} must be a number`);
      if ((kind === "int" || kind === "count") && !Number.isInteger(v)) fail(`${name} must be a whole number`);
      if ((kind === "positive" || kind === "count") && (v as number) <= 0) fail(`${name} must be more than 0`);
    }
  }
  const turn = (root.tof as Record<string, number>).zones_turn_deg;
  if (![0, 90, 180, 270].includes(turn)) fail("tof.zones_turn_deg must be 0, 90, 180 or 270");
  const m = isObject(root.imu) ? root.imu.robot_from_sensor : undefined;
  if (m === undefined) fail("imu.robot_from_sensor is missing");
  const ok = Array.isArray(m) && m.length === 3 && m.every(r => Array.isArray(r) && r.length === 3 && r.every(v => v === -1 || v === 0 || v === 1));
  if (!ok) fail("imu.robot_from_sensor must be 3 rows of 3 numbers, each -1, 0 or 1");
  return json as unknown as Mounting;
}

export function loadMounting(file = MOUNTING_FILE): Mounting {
  let json: unknown;
  try { json = JSON.parse(readFileSync(file, "utf8")); }
  catch (err) { throw new Error(`${file} doesn't read: ${(err as Error).message}`, { cause: err }); }
  return parseMounting(json, file);
}
