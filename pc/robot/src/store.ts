// The recordings in DuckDB (doc/ARCHITECTURE.md, the PC): one row per take, ToF
// frame, zone and target (each target also in world coordinates), odometry report,
// drive command, mark and WiFi status. The pose lines (live.ts): the path driven and
// PicoB's jolt and tilt. And the snapshots (snapshot.ts): one row each
// with its pose, action, camera and colour reading; its ToF targets; its camera
// features; its map window
// and whole-house map tiles as BLOBs; the picture is a PNG beside the database. Derived from the raw files: a take can be
// imported again at any time and replaces its rows; its note and the truth objects
// (what Daniel enters) stay. The database is opened for each operation and closed
// after it, so the duckdb command line can use it in between.
import { DuckDBInstance, timestampValue, type DuckDBConnection } from "@duckdb/node-api";
import type { Geometry, Odom, Parsed, Take } from "./recording.ts";
import { snapId, type Snapshot } from "./snapshot.ts";
import type { LivePose } from "./robot.ts";

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
CREATE TABLE IF NOT EXISTS truth (  -- what really stood there (placed in the viewer): a box per object
  boot_id VARCHAR, take_id VARCHAR, name VARCHAR,
  x DOUBLE, y DOUBLE, z DOUBLE, size_x DOUBLE, size_y DOUBLE, size_z DOUBLE, note VARCHAR);
CREATE TABLE IF NOT EXISTS snapshots (  -- picoA/app/snapshot.c; times: PicoA's clock (t_us), the PC's (arrived_at, local)
  snap_id VARCHAR PRIMARY KEY, boot_id VARCHAR, seq INTEGER, t_us UINTEGER, arrived_at TIMESTAMP,
  why VARCHAR, action VARCHAR, end_reason VARCHAR, actions_ended INTEGER, running VARCHAR, mode VARCHAR,
  map_clears INTEGER, build VARCHAR, bytes INTEGER,
  x DOUBLE, y DOUBLE, yaw DOUBLE, pitch DOUBLE, roll DOUBLE, v DOUBLE, w DOUBLE, stationary BOOLEAN, motors_on BOOLEAN,
  stop_reason INTEGER,                                      -- NULL: PicoB not connected
  tof_frame_no INTEGER, tof_x DOUBLE, tof_y DOUBLE, tof_yaw DOUBLE, tof_pitch DOUBLE, -- where the robot was for the ToF frame
  camera_frame_no INTEGER, exposure_ms DOUBLE, gain DOUBLE, image VARCHAR, -- image: the PNG, relative to recordings/
  lux DOUBLE, cie_x DOUBLE, cie_y DOUBLE, color_x DOUBLE, color_y DOUBLE, color_z DOUBLE, color_raw BLOB,
  map_org_cx INTEGER, map_org_cy INTEGER, map_cells_x INTEGER, map_cells_y INTEGER, map_cells BLOB, -- [layer][x][y], one byte a cell
  tiles INTEGER, file VARCHAR);
CREATE TABLE IF NOT EXISTS snapshot_targets (  -- the snapshot's ToF frame; row, col 1-8 (README)
  snap_id VARCHAR, zone INTEGER, row INTEGER, col INTEGER, target INTEGER,
  distance_mm INTEGER, sigma_mm INTEGER, signal INTEGER, reflectance INTEGER, status INTEGER, x DOUBLE, y DOUBLE, z DOUBLE);
CREATE TABLE IF NOT EXISTS snapshot_features (  -- the camera features of the snapshot's picture (picoA/app/vision.h);
  snap_id VARCHAR, id INTEGER, age INTEGER, flags INTEGER,  -- flags: 1 new, 2 missed, 4 lost, 8 left the image
  x DOUBLE, y DOUBLE, dx DOUBLE, dy DOUBLE, strength DOUBLE, match DOUBLE); -- px, as the robot sees the picture (x right, y down)
CREATE TABLE IF NOT EXISTS snapshot_tiles (  -- whole-house map tiles as they came: 20 x 20 columns, 4 bits each (house_map.h)
  snap_id VARCHAR, boot_id VARCHAR, seq INTEGER, map_clears INTEGER, tx INTEGER, ty INTEGER, columns BLOB);
CREATE TABLE IF NOT EXISTS poses (  -- the pose lines (4 a second while the robot is connected): the path driven, and
  boot_id VARCHAR, arrived_at TIMESTAMP, -- PicoB's safety numbers; the PC's local time, as snapshots.arrived_at
  x DOUBLE, y DOUBLE, yaw DOUBLE, mode VARCHAR, action VARCHAR, rssi INTEGER, recording BOOLEAN,
  jolt_x DOUBLE, jolt_y DOUBLE, jolt_z DOUBLE, tilt DOUBLE);  -- g and rad, the largest since the line before; NULL from older firmware
-- Objects on the map (doc/OBJECTS_PLAN.md, objects/): robot detect on a snapshot's picture. Not derived from
-- the raw files: npm run import leaves them. Running detect again on a snapshot replaces its rows.
CREATE TABLE IF NOT EXISTS detect_runs (  -- one per snapshot detected: how the model was asked and how it went
  snap_id VARCHAR PRIMARY KEY, boot_id VARCHAR, seq INTEGER, ran_at TIMESTAMP, model VARCHAR, prompt_version INTEGER,
  thinking BOOLEAN, seconds DOUBLE, turns INTEGER, result VARCHAR, why VARCHAR, -- result: ok or failed (why)
  transcript VARCHAR);  -- the conversation as JSON, without the picture
CREATE TABLE IF NOT EXISTS detections (  -- the boxes y first on 0-1000, whatever the model gave (objects/labels.ts)
  snap_id VARCHAR, n INTEGER, label VARCHAR, ymin INTEGER, xmin INTEGER, ymax INTEGER, xmax INTEGER);
CREATE TABLE IF NOT EXISTS map_objects (  -- the shapes on the map, m and rad in the boot's map frame
  object_id VARCHAR PRIMARY KEY, boot_id VARCHAR, seq INTEGER, label VARCHAR,
  source VARCHAR,     -- camera (a first guess from one detection)
  floor VARCHAR,      -- floor, cut_off (at most this far), above_horizon, too_far (the last two: not placed)
  kind VARCHAR,       -- rect, line, or NULL (not placed)
  distance DOUBLE,    -- camera to the front edge's middle, along the floor
  cx DOUBLE, cy DOUBLE, width DOUBLE, depth DOUBLE, angle DOUBLE);
CREATE TABLE IF NOT EXISTS object_detections (object_id VARCHAR, snap_id VARCHAR, n INTEGER);  -- which boxes made an object
-- Added later: kept by "IF NOT EXISTS", filled by importing again (npm run import).
ALTER TABLE takes ADD COLUMN IF NOT EXISTS geometry VARCHAR;   -- GEOMETRY as JSON (the viewer's rays)
ALTER TABLE truth ADD COLUMN IF NOT EXISTS truth_id VARCHAR;
ALTER TABLE detections ADD COLUMN IF NOT EXISTS confidence INTEGER;  -- 0-100 %, the model's own (prompt 2 on)
ALTER TABLE detect_runs ADD COLUMN IF NOT EXISTS reasoning_budget INTEGER;  -- the thinking limit in tokens; NULL: none (or no thinking)
`;
const TAKE_TABLES = ["tof_frames", "tof_zones", "tof_targets", "odom", "drive", "marks", "wifi"];
const SNAP_TABLES = ["snapshots", "snapshot_targets", "snapshot_tiles", "snapshot_features"];
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

// A timestamp in the PC's local time (as the database keeps them), in µs.
const localUs = (ms: number) => BigInt(Math.round((ms - new Date(ms).getTimezoneOffset() * 60000) * 1000));

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

  // Runs fn with a connection (read-only use: the API's queries with BLOBs).
  async read<T>(fn: (c: DuckDBConnection) => Promise<T>): Promise<T> { return this.use(fn); }
  // Runs fn with a connection that writes (objects/db.ts).
  async write<T>(fn: (c: DuckDBConnection) => Promise<T>): Promise<T> { return this.use(fn); }

  // Imports a snapshot, replacing what was there for it. image: its PNG, relative to recordings/.
  async importSnapshot(s: Snapshot, arrivalMs: number, file: string, image: string | null): Promise<{ targets: number; tiles: number }> {
    const id = snapId(s.bootId, s.seq);
    const p = s.pose, t = s.tof, cam = s.camera, col = s.color, w = s.window;
    let targets = 0;
    return this.use(async c => {
      await c.run("BEGIN TRANSACTION");
      for (const table of SNAP_TABLES) await c.run(`DELETE FROM ${table} WHERE snap_id = $1`, [id]);
      const a = await c.createAppender("snapshots");
      const d = (v: number | null | undefined) => (v === null || v === undefined ? a.appendNull() : a.appendDouble(v));
      const i = (v: number | null | undefined) => (v === null || v === undefined ? a.appendNull() : a.appendInteger(v));
      const t8 = (v: string | null) => (v === null ? a.appendNull() : a.appendVarchar(v));
      a.appendVarchar(id); a.appendVarchar(s.bootId); a.appendInteger(s.seq); a.appendUInteger(s.tUs);
      a.appendTimestamp(timestampValue(localUs(arrivalMs)));
      t8(s.why); t8(s.action); t8(s.endReason); a.appendInteger(s.actionsEnded); t8(s.running); t8(s.mode);
      a.appendInteger(s.mapClears); t8(s.build); a.appendInteger(s.length);
      for (const v of [p?.x, p?.y, p?.yaw, p?.pitch, p?.roll, p?.v, p?.w]) d(v);
      if (p) { a.appendBoolean(p.stationary); a.appendBoolean(p.motorsOn); a.appendInteger(p.stopReason); }
      else { a.appendNull(); a.appendNull(); a.appendNull(); }
      i(t?.raw.frameNo);
      for (const v of t?.poseOk ? [t.x, t.y, t.yaw, t.pitch] : [null, null, null, null]) d(v);
      i(cam?.number); d(cam ? cam.exposureUs / 1000 : null); d(cam?.gain); t8(image);
      d(col?.lux); d(col?.cie?.x); d(col?.cie?.y); d(col?.xyz?.X); d(col?.xyz?.Y); d(col?.xyz?.Z);
      if (col) a.appendBlob(new Uint8Array(col.raw)); else a.appendNull();
      i(w?.orgX); i(w?.orgY); i(w?.cellsX); i(w?.cellsY);
      if (w) a.appendBlob(new Uint8Array(w.cells)); else a.appendNull();
      a.appendInteger(s.tiles.length); a.appendVarchar(file);
      a.endRow();
      a.closeSync();
      const tg = await c.createAppender("snapshot_targets");
      if (t && s.geometry) {
        const g = s.geometry, rayOf = new Map<number, number>();
        g.zoneOfRay.forEach((zone, ray) => rayOf.set(zone, ray));
        for (const z of t.raw.zones) {
          const ray = rayOf.get(z.zone) ?? z.zone, row = Math.floor(ray / g.cols) + 1, colN = (ray % g.cols) + 1;
          z.targets.forEach((target, k) => {
            tg.appendVarchar(id); tg.appendInteger(z.zone); tg.appendInteger(row); tg.appendInteger(colN); tg.appendInteger(k);
            tg.appendInteger(target.distanceMm); tg.appendInteger(target.sigmaMm); tg.appendInteger(target.signalPerSpad);
            tg.appendInteger(target.reflectance); tg.appendInteger(target.status);
            if (t.poseOk) {
              const [x, y, zz] = worldPoint(g, { x: t.x, y: t.y, yaw: t.yaw, pitch: t.pitch }, row, colN, target.distanceMm / 1000);
              tg.appendDouble(x); tg.appendDouble(y); tg.appendDouble(zz);
            } else { tg.appendNull(); tg.appendNull(); tg.appendNull(); }
            tg.endRow();
            targets++;
          });
        }
      }
      tg.closeSync();
      const tl = await c.createAppender("snapshot_tiles");
      for (const tile of s.tiles) {
        tl.appendVarchar(id); tl.appendVarchar(s.bootId); tl.appendInteger(s.seq); tl.appendInteger(s.mapClears);
        tl.appendInteger(tile.tx); tl.appendInteger(tile.ty); tl.appendBlob(new Uint8Array(tile.columns));
        tl.endRow();
      }
      tl.closeSync();
      const ft = await c.createAppender("snapshot_features");
      for (const f of s.features?.features ?? []) {
        ft.appendVarchar(id); ft.appendInteger(f.id); ft.appendInteger(f.age); ft.appendInteger(f.flags);
        for (const v of [f.x, f.y, f.dx, f.dy, f.strength, f.match]) ft.appendDouble(v);
        ft.endRow();
      }
      ft.closeSync();
      await c.run("COMMIT");
      return { targets, tiles: s.tiles.length };
    });
  }

  // Appends pose lines, each with the boot it belongs to.
  async appendPoses(rows: { boot: string; pose: LivePose }[]): Promise<void> {
    if (!rows.length) return;
    await this.use(async c => {
      const a = await c.createAppender("poses");
      for (const { boot, pose: p } of rows) {
        a.appendVarchar(boot); a.appendTimestamp(timestampValue(localUs(p.at)));
        a.appendDouble(p.x); a.appendDouble(p.y); a.appendDouble(p.yaw); a.appendVarchar(p.mode); a.appendVarchar(p.action);
        if (p.rssi === null) a.appendNull(); else a.appendInteger(p.rssi);
        if (p.recording === null) a.appendNull(); else a.appendBoolean(p.recording);
        for (const v of [...(p.jolt ?? [null, null, null]), p.tilt]) { if (v === null) a.appendNull(); else a.appendDouble(v); }
        a.endRow();
      }
      a.closeSync();
    });
  }

  // The path driven up to snapshot seq of a boot, since the map was last cleared before
  // it: the pose lines' x, y (m), oldest first.
  async path(boot: string, seq: number): Promise<{ x: number; y: number }[]> {
    return this.use(async c => {
      const rows = (await c.runAndReadAll(`
        WITH s AS (SELECT arrived_at, map_clears FROM snapshots WHERE boot_id = $1 AND seq = $2),
             cleared AS (SELECT max(arrived_at) AS t FROM snapshots WHERE boot_id = $1 AND map_clears < (SELECT map_clears FROM s))
        SELECT x, y FROM poses WHERE boot_id = $1 AND arrived_at <= (SELECT arrived_at FROM s)
          AND arrived_at >= coalesce((SELECT t FROM cleared), '-infinity'::TIMESTAMP) ORDER BY arrived_at`, [boot, seq])).getRowObjects();
      return rows.map(r => ({ x: Number(r.x), y: Number(r.y) }));
    });
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
           $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16, $17, $18, $19)`,
        [id, start.bootId, start.takeNo, start.key, start.action, start.param, start.version, start.build,
         elapsed(start.tUs, end?.tUs ?? lastTime(take.records, start.tUs)), end ? end.reason : "cut",
         start.version >= 3 ? whole : take.counts.TOF_RAW ?? 0, half, start.version >= 3 ? missing : end?.framesDropped ?? 0,
         received, lost, odoms.length, take.file, (old[0]?.note as string | undefined) ?? "",
         geometry ? JSON.stringify({ sensorM: geometry.sensorM, zoneRad: geometry.zoneRad, rows: geometry.rows, cols: geometry.cols }) : null]);

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
          for (const v of rest) {
            if (typeof v === "string") a.appendVarchar(v);
            else if (table === "drive") a.appendDouble(v);
            else a.appendInteger(v);
          }
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
