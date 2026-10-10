// The recordings in DuckDB (doc/TELEMETRY_PLAN.md §3): one row per take, ToF
// frame, zone and target (each target also in world coordinates), odometry report,
// drive command, mark and WiFi status. Derived from the raw files: a take can be
// imported again at any time and replaces its rows; its note and the truth objects
// (what Daniel enters) stay. The database is opened for each operation and closed
// after it, so the duckdb command line can use it in between.
import { DuckDBInstance, type DuckDBConnection } from "@duckdb/node-api";
import type { Geometry, Odom, Parsed, Take } from "./recording.ts";

const SCHEMA = `
CREATE TABLE IF NOT EXISTS takes (
  take_id VARCHAR PRIMARY KEY,  -- boot id - take number
  boot_id VARCHAR, take_no INTEGER, started_at TIMESTAMP, day DATE,  -- local time; before UDP: when the file was opened
  key VARCHAR, action VARCHAR, param DOUBLE, version INTEGER, build VARCHAR,
  duration_s DOUBLE, end_reason VARCHAR,  -- 'cut' without TAKE_END
  frames_whole INTEGER, frames_half INTEGER, frames_missing INTEGER,
  datagrams INTEGER, datagrams_lost INTEGER, odom_reports INTEGER,
  file VARCHAR, note VARCHAR);
CREATE TABLE IF NOT EXISTS tof_frames (
  take_id VARCHAR, frame_no INTEGER, t_us UINTEGER, t_s DOUBLE,  -- t_s: since the take started
  skipped INTEGER, temp_c INTEGER, zones INTEGER,                 -- zones: 64, or 32 when half was lost
  x DOUBLE, y DOUBLE, yaw DOUBLE, pitch DOUBLE);                  -- the robot then (interpolated); NULL if unknown
CREATE TABLE IF NOT EXISTS tof_zones (  -- row 1-8 top to bottom, col 1-8 left to right (README); zone: the sensor's own 0-63
  take_id VARCHAR, frame_no INTEGER, zone INTEGER, row INTEGER, col INTEGER,
  targets INTEGER, ambient INTEGER, spads INTEGER);
CREATE TABLE IF NOT EXISTS tof_targets (
  take_id VARCHAR, frame_no INTEGER, zone INTEGER, row INTEGER, col INTEGER, target INTEGER,
  distance_mm INTEGER, sigma_mm INTEGER, signal INTEGER, reflectance INTEGER, status INTEGER,
  x DOUBLE, y DOUBLE, z DOUBLE);  -- world (odometry's frame from power-up), m; NULL without a pose
CREATE TABLE IF NOT EXISTS odom (
  take_id VARCHAR, t_us UINTEGER, t_s DOUBLE, picob_us UINTEGER,
  x DOUBLE, y DOUBLE, yaw DOUBLE, pitch DOUBLE, roll DOUBLE, v DOUBLE, w DOUBLE,
  wheel_left DOUBLE, wheel_right DOUBLE, stationary BOOLEAN, motors_on BOOLEAN,
  stop_reason INTEGER, imu_error BOOLEAN, gyro_bias DOUBLE);
CREATE TABLE IF NOT EXISTS drive (take_id VARCHAR, t_us UINTEGER, t_s DOUBLE, v DOUBLE, w DOUBLE);
CREATE TABLE IF NOT EXISTS marks (take_id VARCHAR, t_us UINTEGER, t_s DOUBLE, text VARCHAR);
CREATE TABLE IF NOT EXISTS wifi (
  take_id VARCHAR, t_us UINTEGER, t_s DOUBLE, rssi_dbm INTEGER, console_silent_ms INTEGER,
  console_unacked INTEGER, console_retries INTEGER, datagrams INTEGER, datagrams_refused INTEGER);
CREATE TABLE IF NOT EXISTS truth (  -- what really stood there (entered by hand, T4)
  boot_id VARCHAR, take_id VARCHAR, name VARCHAR,
  x DOUBLE, y DOUBLE, z DOUBLE, size_x DOUBLE, size_y DOUBLE, size_z DOUBLE, note VARCHAR);
`;
const TAKE_TABLES = ["tof_frames", "tof_zones", "tof_targets", "odom", "drive", "marks", "wifi"];
const POSE_GAP_S = 0.1; // a frame further than this from any odometry report has no pose

export interface Pose { x: number; y: number; yaw: number; pitch: number }

// The robot at time t (s since the take started), between the reports around it;
// null if none is within POSE_GAP_S.
export function poseAt(odom: { t: number; o: Odom }[], t: number): Pose | null {
  if (!odom.length) return null;
  let lo = 0, hi = odom.length - 1;
  if (t <= odom[0].t) return odom[0].t - t <= POSE_GAP_S ? pick(odom[0].o) : null;
  if (t >= odom[hi].t) return t - odom[hi].t <= POSE_GAP_S ? pick(odom[hi].o) : null;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (odom[mid].t <= t) lo = mid; else hi = mid; }
  const a = odom[lo], b = odom[hi];
  if (b.t - a.t > 2 * POSE_GAP_S) return null;
  const f = (t - a.t) / (b.t - a.t || 1);
  const mix = (p: number, q: number) => p + f * (q - p);
  return { x: mix(a.o.x, b.o.x), y: mix(a.o.y, b.o.y), yaw: mix(a.o.yaw, b.o.yaw), pitch: mix(a.o.pitch, b.o.pitch) };
}
const pick = (o: Odom): Pose => ({ x: o.x, y: o.y, yaw: o.yaw, pitch: o.pitch });

// Where a target is in the world, as the robot's code places it (rangefinder.c,
// cell_map.c): the ray at row, col (1-8) points (row - 4.5) zone angles below
// horizontal (less the robot's nose-up pitch) and (4.5 - col) to the left; the sensor
// sits at sensorM in the robot frame; distance along the ray.
export function worldPoint(g: Geometry, pose: Pose, row: number, col: number, distanceM: number): [number, number, number] {
  const down = (row - (g.rows + 1) / 2) * g.zoneRad - pose.pitch, left = ((g.cols + 1) / 2 - col) * g.zoneRad;
  const dx = Math.cos(down) * Math.cos(left), dy = Math.cos(down) * Math.sin(left), dz = -Math.sin(down);
  const rx = g.sensorM[0] + distanceM * dx, ry = g.sensorM[1] + distanceM * dy;
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  return [pose.x + c * rx - s * ry, pose.y + s * rx + c * ry, g.sensorM[2] + distanceM * dz];
}

export const takeId = (t: Take) => `${t.start.bootId}-${t.start.takeNo}`;

// Runs fn with a connection to the database at dbPath, one operation at a time.
export class Store {
  private queue: Promise<unknown> = Promise.resolve();
  readonly dbPath: string;
  constructor(dbPath: string) { this.dbPath = dbPath; }

  private use<T>(fn: (c: DuckDBConnection) => Promise<T>): Promise<T> {
    const run = async () => {
      const db = await DuckDBInstance.create(this.dbPath);
      const c = await db.connect();
      try {
        await c.run(SCHEMA);
        return await fn(c);
      } finally {
        c.closeSync();
        db.closeSync();
      }
    };
    const next = this.queue.then(run, run);
    this.queue = next.catch(() => {});
    return next;
  }

  async query(sql: string): Promise<Record<string, unknown>[]> {
    return this.use(async c => (await c.runAndReadAll(sql)).getRowObjectsJson() as Record<string, unknown>[]);
  }

  async setNote(id: string, note: string): Promise<void> {
    await this.use(c => c.run("UPDATE takes SET note = $1 WHERE take_id = $2", [note, id]));
  }

  // Imports the take, replacing what was there for it. Returns the rows written.
  async importTake(take: Take): Promise<{ frames: number; zones: number; targets: number; odom: number }> {
    const id = takeId(take), start = take.start;
    const rel = (tUs: number) => ((tUs - start.tUs) | 0) / 1e6; // signed: reports before the start too
    const geometry = take.records.find(r => r.kind === "GEOMETRY") as Geometry | undefined;
    const odoms = take.records.filter((r): r is Odom => r.kind === "ODOM").map(o => ({ t: rel(o.tUs), o })).sort((a, b) => a.t - b.t);
    const rayOf = new Map<number, number>();
    geometry?.zoneOfRay.forEach((zone, ray) => rayOf.set(zone, ray));
    const end = take.end;
    const { whole, half, missing } = take.frameCounts, { received, lost } = take.datagramCounts;
    const counts = { frames: 0, zones: 0, targets: 0, odom: odoms.length };

    return this.use(async c => {
      const old = (await c.runAndReadAll("SELECT note FROM takes WHERE take_id = $1", [id])).getRowObjects();
      await c.run("BEGIN TRANSACTION");
      await c.run("DELETE FROM takes WHERE take_id = $1", [id]);
      for (const t of TAKE_TABLES) await c.run(`DELETE FROM ${t} WHERE take_id = $1`, [id]);
      // Local time (the robot's room), from when the take's first datagram arrived.
      const startedMs = take.startedAtMs === null ? null : take.startedAtMs - new Date(take.startedAtMs).getTimezoneOffset() * 60000;
      await c.run(
        `INSERT INTO takes VALUES ($1, $2, $3, ${startedMs === null ? "NULL" : `make_timestamp(${Math.round(startedMs * 1000)}::BIGINT)`},
           ${startedMs === null ? "NULL" : `CAST(make_timestamp(${Math.round(startedMs * 1000)}::BIGINT) AS DATE)`},
           $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16, $17, $18)`,
        [id, start.bootId, start.takeNo, start.key, start.action, start.param, start.version, start.build,
         elapsed(start.tUs, end?.tUs ?? lastTime(take.records, start.tUs)), end ? end.reason : "cut",
         start.version >= 3 ? whole : take.counts.TOF_RAW ?? 0, half, start.version >= 3 ? missing : end?.framesDropped ?? 0,
         received, lost, odoms.length, take.file, (old[0]?.note as string | undefined) ?? ""]);

      const frames = await c.createAppender("tof_frames"), zones = await c.createAppender("tof_zones");
      const targets = await c.createAppender("tof_targets");
      const seen = new Map<number, { zones: number }>();
      for (const r of take.records) {
        if (r.kind !== "TOF_RAW") continue;
        const t = rel(r.tUs), pose = poseAt(odoms, t);
        const f = seen.get(r.frameNo);
        if (f) f.zones += r.zones.length;
        else seen.set(r.frameNo, { zones: r.zones.length });
        for (const z of r.zones) {
          const ray = rayOf.get(z.zone) ?? z.zone, cols = geometry?.cols ?? 8;
          const row = Math.floor(ray / cols) + 1, col = (ray % cols) + 1; // 1-8, as people count them
          zones.appendVarchar(id); zones.appendInteger(r.frameNo); zones.appendInteger(z.zone); zones.appendInteger(row);
          zones.appendInteger(col); zones.appendInteger(z.targets.length); zones.appendInteger(z.ambientPerSpad);
          zones.appendInteger(z.spads); zones.endRow();
          counts.zones++;
          z.targets.forEach((tg, k) => {
            targets.appendVarchar(id); targets.appendInteger(r.frameNo); targets.appendInteger(z.zone);
            targets.appendInteger(row); targets.appendInteger(col); targets.appendInteger(k);
            targets.appendInteger(tg.distanceMm); targets.appendInteger(tg.sigmaMm); targets.appendInteger(tg.signalPerSpad);
            targets.appendInteger(tg.reflectance); targets.appendInteger(tg.status);
            if (pose && geometry) {
              const [x, y, zz] = worldPoint(geometry, pose, row, col, tg.distanceMm / 1000);
              targets.appendDouble(x); targets.appendDouble(y); targets.appendDouble(zz);
            } else { targets.appendNull(); targets.appendNull(); targets.appendNull(); }
            targets.endRow();
            counts.targets++;
          });
        }
      }
      for (const r of take.records) { // one row per frame, with the zones that arrived
        if (r.kind !== "TOF_RAW" || !seen.has(r.frameNo)) continue;
        const t = rel(r.tUs), pose = poseAt(odoms, t);
        frames.appendVarchar(id); frames.appendInteger(r.frameNo); frames.appendUInteger(r.tUs); frames.appendDouble(t);
        frames.appendInteger(r.skipped); frames.appendInteger(r.tempC); frames.appendInteger(seen.get(r.frameNo)!.zones);
        if (pose) { frames.appendDouble(pose.x); frames.appendDouble(pose.y); frames.appendDouble(pose.yaw); frames.appendDouble(pose.pitch); }
        else for (let k = 0; k < 4; k++) frames.appendNull();
        frames.endRow();
        seen.delete(r.frameNo);
        counts.frames++;
      }
      frames.closeSync(); zones.closeSync(); targets.closeSync();

      const odom = await c.createAppender("odom");
      for (const { t, o } of odoms) {
        odom.appendVarchar(id); odom.appendUInteger(o.tUs); odom.appendDouble(t); odom.appendUInteger(o.picoBUs);
        for (const v of [o.x, o.y, o.yaw, o.pitch, o.roll, o.v, o.w, o.wheelLeft, o.wheelRight]) odom.appendDouble(v);
        odom.appendBoolean(o.stationary); odom.appendBoolean(o.motorsOn); odom.appendInteger(o.stopReason);
        odom.appendBoolean(o.imuError); odom.appendDouble(o.gyroBias);
        odom.endRow();
      }
      odom.closeSync();
      // t_us is UINTEGER: appended as an integer that may exceed INT32, so through appendUInteger.
      const withTime = async (table: string, rows: (r: Parsed) => [number, ...(number | string)[]] | null) => {
        const a = await c.createAppender(table);
        for (const r of take.records) {
          const values = rows(r);
          if (!values) continue;
          const [tUs, ...rest] = values;
          a.appendVarchar(id); a.appendUInteger(tUs); a.appendDouble(rel(tUs));
          for (const v of rest) typeof v === "string" ? a.appendVarchar(v) : table === "drive" ? a.appendDouble(v) : a.appendInteger(v);
          a.endRow();
        }
        a.closeSync();
      };
      await withTime("drive", r => r.kind === "DRIVE" ? [r.tUs, r.v, r.w] : null);
      await withTime("marks", r => r.kind === "MARK" ? [r.tUs, r.text] : null);
      await withTime("wifi", r => r.kind === "WIFI"
        ? [r.tUs, r.rssiDbm, r.consoleSilentMs, r.console.unacked, r.console.retries, r.datagrams ?? 0, r.datagramsFailed ?? 0] : null);
      await c.run("COMMIT");
      return counts;
    });
  }
}

const elapsed = (from: number, to: number) => ((to - from) >>> 0) / 1e6;
function lastTime(records: Parsed[], fallback: number): number {
  for (let i = records.length - 1; i >= 0; i--) { const r = records[i]; if ("tUs" in r && r.kind !== "TOF_RAW") return r.tUs; }
  return fallback;
}
