// The one store (doc/ARCHITECTURE.md, the PC): DuckDB, a file of its own
// (recordings/robot2.duckdb, or ROBOT_DB; the old recordings/robot.duckdb is never
// opened). What the robot sends is written as it arrives, one row per message:
//   boots      one per power-up: its build, its copy of the mounting (JSON), how many
//              messages it sent (boot.ts), so a restarted server numbers on
//   moves      one per move: its start (command, kind, parameter, still times) and its
//              end (how, frames sent and dropped, moved, imu batches sent and dropped)
//   frames     one per tof frame: its number, move, time, pose, period, turn, flags
//   zones      one per zone of each frame: row, col 1-8 (README), distance, kind,
//              signal, sigma, targets
//   imu        one per imu record (104 a second during moves): its move (its batch names
//              it), time, and the IMU's raw counts in the sensor's axes (training data)
//   status     the robot's status messages (4 a second): the path driven
//   snapshots  one per snapshot, its bytes as they came, its pose, action, camera and
//              colour reading; snapshot_zones (its ToF frame), snapshot_features; the
//              picture a PNG beside the database
//   detect_runs, detections, map_objects, object_detections: objects (objects/db.ts)
// Times: t_us and t_s are since power-up (µs as sent, s); robot_clock_us is PicoA's
// 32-bit clock (it wraps every 71.6 min), kept only where the robot's own bytes hold
// it (a packed frame, a snapshot); arrived_at the PC's local time. message_number numbers a
// boot's messages in the order they came (the fake robot replays them in that
// order). The database is opened for each operation and closed after it, so the
// duckdb command line can use it in between; writes that come while one runs go
// together in the next.
import { DuckDBInstance, timestampValue, type DuckDBAppender, type DuckDBConnection } from "@duckdb/node-api";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { snapshotId, type Snapshot } from "./snapshot.ts";
import type { Pose } from "./link.ts";
import { Boot, type TofRow, type ImuBatchRow, type MoveEndRow, type MoveStartRow, type StatusRow } from "./boot.ts";
import type { Mounting } from "./mounting.ts";
import { KINDS, rowCol, type Kind, type PosedFrame } from "./tof_format.ts";
import { IMU_RECORD_PERIOD_US, type Hello, type ImuBatch, type ImuRecord, type TofMessage, type Vec3 } from "./wire.ts";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
// Where the recordings live (camera pictures, map pictures, boots.json) and the database.
export const RECORDINGS_DIRECTORY = process.env.RECORDINGS ? path.resolve(process.env.RECORDINGS) : path.join(root, "recordings");
export const DB_FILE = "robot2.duckdb";
export const DB_PATH = process.env.ROBOT_DB ? path.resolve(process.env.ROBOT_DB) : path.join(RECORDINGS_DIRECTORY, DB_FILE);

const SCHEMA = `
CREATE TABLE IF NOT EXISTS boots (
  boot_id VARCHAR PRIMARY KEY, first_seen TIMESTAMP, last_seen TIMESTAMP, build VARCHAR, wire_version INTEGER,
  mounting VARCHAR,                                    -- the boot's copy of common/mounting.json, when it was first seen
  messages INTEGER);                                   -- how many it sent that are stored (boot.ts numbers them)
CREATE TABLE IF NOT EXISTS moves (
  boot_id VARCHAR, move INTEGER, command INTEGER, kind VARCHAR, parameter DOUBLE,  -- drive m, rotate rad, goto m to the goal, watch s
  pre_ms INTEGER, post_ms INTEGER, refused BOOLEAN,   -- refused: it couldn't start (blocked, already there): no frames
  start_message_number INTEGER, start_at TIMESTAMP, start_t_us UBIGINT, start_t_s DOUBLE,  -- t_us, t_s: since power-up
  end_message_number INTEGER, end_at TIMESTAMP, end_t_us UBIGINT, end_t_s DOUBLE,
  "end" VARCHAR,                                       -- as the answer's end word (ok, obstacle, stopped, ...)
  frames_sent INTEGER, frames_dropped INTEGER, moved DOUBLE, rotated DOUBLE,
  imu_sent INTEGER, imu_dropped INTEGER,               -- imu batches (10 records each, the last fewer)
  PRIMARY KEY (boot_id, move));
CREATE TABLE IF NOT EXISTS frames (
  boot_id VARCHAR, number UBIGINT, move INTEGER, message_number INTEGER, arrived_at TIMESTAMP, format VARCHAR,
  t_s DOUBLE, period_ms DOUBLE,                        -- the middle of the measurement (since power-up), how long it took
  robot_clock_us UBIGINT,                              -- the middle on PicoA's 32-bit clock, as packed (wraps; the replay repacks it)
  x DOUBLE, y DOUBLE, yaw DOUBLE, pitch DOUBLE, turn DOUBLE,  -- m, rad (yaw wrapped); turn: during the frame
  still BOOLEAN, pose_known BOOLEAN, beyond_mm INTEGER);      -- beyond_mm: a distance this far is "at least"
CREATE TABLE IF NOT EXISTS zones (                    -- row 1-8 top to bottom, col 1-8 left to right (README)
  boot_id VARCHAR, number UBIGINT, row TINYINT, col TINYINT,
  distance_mm SMALLINT, kind VARCHAR, signal SMALLINT, sigma_mm SMALLINT, targets TINYINT); -- kind: sure, less_sure, unsure, nothing
CREATE TABLE IF NOT EXISTS imu (                      -- one per imu record (common/link_messages.h): 4 IMU samples averaged
  boot_id VARCHAR, move INTEGER, message_number INTEGER, n TINYINT,  -- message_number: its batch's; n: its place in the batch (0-9)
  arrived_at TIMESTAMP, t_us UBIGINT, t_s DOUBLE,      -- when it was measured (since power-up): the middle of its samples
  ax SMALLINT, ay SMALLINT, az SMALLINT,               -- raw counts in the sensor's axes, nothing taken out: acceleration 0.061 mg a count,
  gx SMALLINT, gy SMALLINT, gz SMALLINT);              -- gyro 8.75 mdps a count; the boot's mounting (imu.robot_from_sensor) turns them
CREATE TABLE IF NOT EXISTS status (
  boot_id VARCHAR, message_number INTEGER, arrived_at TIMESTAMP,
  x DOUBLE, y DOUBLE, yaw DOUBLE, mode VARCHAR, action VARCHAR, rssi INTEGER);
CREATE TABLE IF NOT EXISTS snapshots (                -- picoA/app/snapshot.c
  snapshot_id VARCHAR PRIMARY KEY, boot_id VARCHAR, sequence INTEGER, message_number INTEGER, robot_clock_us UBIGINT, arrived_at TIMESTAMP,
  why VARCHAR, action VARCHAR, end_reason VARCHAR, actions_ended INTEGER, running VARCHAR, mode VARCHAR,
  build VARCHAR, length INTEGER, bytes BLOB,                -- bytes: the snapshot as it came
  x DOUBLE, y DOUBLE, yaw DOUBLE, pitch DOUBLE, roll DOUBLE, v DOUBLE, w DOUBLE, stationary BOOLEAN, motors_on BOOLEAN,
  stop_reason INTEGER,                                      -- NULL: PicoB not connected
  tof_frame_no UBIGINT, tof_x DOUBLE, tof_y DOUBLE, tof_yaw DOUBLE, tof_pitch DOUBLE, -- where the robot was for the ToF frame
  camera_frame_no INTEGER, exposure_ms DOUBLE, gain DOUBLE, image VARCHAR, -- image: the PNG, relative to recordings/
  lux DOUBLE, cie_x DOUBLE, cie_y DOUBLE, color_x DOUBLE, color_y DOUBLE, color_z DOUBLE, color_raw BLOB);
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_clears;     -- until step 6: the map is built on the PC from the ToF frames
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_origin_cx;
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_origin_cy;
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_cells_x;
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_cells_y;
ALTER TABLE snapshots DROP COLUMN IF EXISTS map_cells;
ALTER TABLE snapshots DROP COLUMN IF EXISTS tiles;
CREATE TABLE IF NOT EXISTS snapshot_zones (           -- the snapshot's ToF frame (in the world: maps/cell_map.ts, rays)
  snapshot_id VARCHAR, row TINYINT, col TINYINT, distance_mm SMALLINT, kind VARCHAR, signal SMALLINT, sigma_mm SMALLINT,
  targets TINYINT);
ALTER TABLE snapshot_zones DROP COLUMN IF EXISTS x;  -- until 10 Nov: each zone in the world, now placed when read
ALTER TABLE snapshot_zones DROP COLUMN IF EXISTS y;
ALTER TABLE snapshot_zones DROP COLUMN IF EXISTS z;
CREATE TABLE IF NOT EXISTS snapshot_features (        -- the camera features of the snapshot's picture (picoA/app/vision.h);
  snapshot_id VARCHAR, id INTEGER, age INTEGER, flags INTEGER,  -- flags: 1 new, 2 missed, 4 lost, 8 left the image
  x DOUBLE, y DOUBLE, dx DOUBLE, dy DOUBLE, strength DOUBLE, match DOUBLE); -- px, as the robot sees the picture (x right, y down)
DROP TABLE IF EXISTS snapshot_tiles;
-- Objects on the map (doc/OBJECTS_PLAN.md, objects/): robot detect on a snapshot's picture. Running detect
-- again on a snapshot replaces its rows.
CREATE TABLE IF NOT EXISTS detect_runs (  -- one per snapshot detected: how the model was asked and how it went
  snapshot_id VARCHAR PRIMARY KEY, boot_id VARCHAR, sequence INTEGER, ran_at TIMESTAMP, model VARCHAR, prompt_version INTEGER,
  thinking BOOLEAN, seconds DOUBLE, turns INTEGER, result VARCHAR, why VARCHAR, -- result: ok or failed (why)
  transcript VARCHAR,       -- the conversation as JSON, without the picture
  reasoning_budget INTEGER);  -- the thinking limit in tokens; NULL: none (or no thinking)
CREATE TABLE IF NOT EXISTS detections (  -- the boxes y first on 0-1000, whatever the model gave (objects/labels.ts)
  snapshot_id VARCHAR, n INTEGER, label VARCHAR, ymin INTEGER, xmin INTEGER, ymax INTEGER, xmax INTEGER,
  confidence INTEGER);  -- 0-100 %, the model's own
CREATE TABLE IF NOT EXISTS map_objects (  -- the shapes on the map, m and rad in the boot's map frame
  object_id VARCHAR PRIMARY KEY, boot_id VARCHAR, sequence INTEGER, label VARCHAR,
  source VARCHAR,     -- camera (a first guess from one detection)
  floor VARCHAR,      -- floor, cut_off (at most this far), above_horizon, too_far (the last two: not placed)
  kind VARCHAR,       -- rect, line, or NULL (not placed)
  distance DOUBLE,    -- camera to the front edge's middle, along the floor
  cx DOUBLE, cy DOUBLE, width DOUBLE, depth DOUBLE, angle DOUBLE);
CREATE TABLE IF NOT EXISTS object_detections (object_id VARCHAR, snapshot_id VARCHAR, n INTEGER);  -- which boxes made an object
`;

// A timestamp in the PC's local time (as the database keeps them), in µs.
const localUs = (ms: number) => BigInt(Math.round((ms - new Date(ms).getTimezoneOffset() * 60000) * 1000));
const at = (ms: number) => timestampValue(localUs(ms));

// A move as stored: its start and end; null where that part hasn't come.
export interface MoveRow {
  move: number; command: number | null; kind: string | null; parameter: number | null; preMs: number | null; postMs: number | null; refused: boolean;
  startTS: number | null; endTS: number | null; end: string | null; framesSent: number | null; framesDropped: number | null;
  moved: number | null; rotated: number | null;
  imuSent: number | null; imuDropped: number | null;
}
// A frame as stored, with its move and time since power-up.
export interface StoredFrame { frame: PosedFrame; format: string; move: number; tS: number }
// An imu record as stored, with its time since power-up.
export type StoredImu = ImuRecord & { tS: number }

// A boot's messages as they came, for the fake robot's replay.
export type Replayed =
  | { messageNumber: number; atMs: number; type: "move_start"; start: MoveStartRow }
  | { messageNumber: number; atMs: number; type: "move_end"; end: MoveEndRow }
  | { messageNumber: number; atMs: number; type: "tof"; tof: TofMessage }
  | { messageNumber: number; atMs: number; type: "imu_batch"; batch: ImuBatch }
  | { messageNumber: number; atMs: number; type: "status"; pose: StatusRow["pose"] }
  | { messageNumber: number; atMs: number; type: "snapshot"; bytes: Buffer };

const num = (v: unknown) => (v === null || v === undefined ? null : Number(v));
const msOf = (v: unknown) => { // a TIMESTAMP read back (local time, as written) to ms since 1970
  if (v === null || v === undefined) return 0;
  const us = (v as { micros: bigint }).micros;
  const local = Number(us) / 1000;
  return local + new Date(local).getTimezoneOffset() * 60000;
};

// Runs fn with a connection to the database at dbPath, one operation at a time.
export class Store {
  private queue: Promise<unknown> = Promise.resolve();
  readonly dbPath: string;
  private pending: { starts: MoveStartRow[]; ends: MoveEndRow[]; frames: TofRow[]; imu: ImuBatchRow[]; status: StatusRow[]; boots: Map<string, Boot> } =
    { starts: [], ends: [], frames: [], imu: [], status: [], boots: new Map() };
  private flushing: Promise<void> | null = null;
  onError: (err: Error) => void = () => {};

  constructor(dbPath = DB_PATH) { this.dbPath = dbPath; }

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

  // --- boots ---

  // The boot a hello names: as the store left it (its own mounting), or new, with
  // mountingNow as its copy.
  async openBoot(hello: Hello, mountingNow: Mounting, atMs = Date.now()): Promise<Boot> {
    return this.use(async c => {
      const [row] = (await c.runAndReadAll("SELECT * FROM boots WHERE boot_id = $1", [hello.bootId])).getRowObjects();
      if (!row) {
        await c.run(`INSERT INTO boots (boot_id, first_seen, last_seen, build, wire_version, mounting, messages)
          VALUES ($1, $2, $2, $3, $4, $5, 0)`, [hello.bootId, at(atMs), hello.build, hello.version, JSON.stringify(mountingNow)]);
        return new Boot(hello.bootId, mountingNow);
      }
      await c.run("UPDATE boots SET last_seen = $2, build = $3 WHERE boot_id = $1", [hello.bootId, at(atMs), hello.build]);
      return new Boot(hello.bootId, JSON.parse(String(row.mounting)) as Mounting, Number(row.messages ?? 0));
    });
  }

  // A boot's copy of the mounting; null for a boot the store doesn't know.
  async mountingOf(boot: string): Promise<Mounting | null> {
    const [row] = await this.read(async c => (await c.runAndReadAll("SELECT mounting FROM boots WHERE boot_id = $1", [boot])).getRowObjects());
    return row ? (JSON.parse(String(row.mounting)) as Mounting) : null;
  }

  // Replaces a boot's copy of the mounting (npm run place-objects -- --from-file).
  async setMounting(boot: string, m: Mounting): Promise<void> {
    await this.write(c => c.run("UPDATE boots SET mounting = $2 WHERE boot_id = $1", [boot, JSON.stringify(m)]));
  }

  // --- the messages, as they arrive ---

  moveStart(boot: Boot, row: MoveStartRow): Promise<void> { this.pending.starts.push(row); return this.queued(boot); }
  moveEnd(boot: Boot, row: MoveEndRow): Promise<void> { this.pending.ends.push(row); return this.queued(boot); }
  tof(boot: Boot, row: TofRow): Promise<void> { this.pending.frames.push(row); return this.queued(boot); }
  imuBatch(boot: Boot, row: ImuBatchRow): Promise<void> { this.pending.imu.push(row); return this.queued(boot); }
  status(boot: Boot, row: StatusRow): Promise<void> { this.pending.status.push(row); return this.queued(boot); }

  // Resolves once everything given so far is written.
  flush(): Promise<void> {
    const p = this.pending;
    if (!p.starts.length && !p.ends.length && !p.frames.length && !p.imu.length && !p.status.length && !p.boots.size) {
      return this.flushing ?? Promise.resolve();
    }
    return this.queued(null);
  }

  // Writes what waits, together, as soon as the last write has finished.
  private queued(boot: Boot | null): Promise<void> {
    if (boot) this.pending.boots.set(boot.id, boot);
    if (this.flushing) return this.flushing.then(() => this.flush());
    this.flushing = (async () => {
      await Promise.resolve(); // what arrives in the same tick goes along
      const p = this.pending;
      this.pending = { starts: [], ends: [], frames: [], imu: [], status: [], boots: new Map() };
      try { await this.writeRows(p); } catch (err) { this.onError(err as Error); }
      finally { this.flushing = null; }
    })();
    return this.flushing;
  }

  private async writeRows(p: Store["pending"]): Promise<void> {
    await this.use(async c => {
      await c.run("BEGIN TRANSACTION");
      for (const s of p.starts) {
        await c.run(`INSERT INTO moves (boot_id, move, command, kind, parameter, pre_ms, post_ms, start_message_number, start_at, start_t_us, start_t_s, refused)
          VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12) ON CONFLICT (boot_id, move) DO UPDATE SET command = excluded.command,
          kind = excluded.kind, parameter = excluded.parameter, pre_ms = excluded.pre_ms, post_ms = excluded.post_ms, start_message_number = excluded.start_message_number,
          refused = excluded.refused,
          start_at = excluded.start_at, start_t_us = excluded.start_t_us, start_t_s = excluded.start_t_s`,
          [s.boot, s.move, s.command, s.kind, s.parameter, s.preMs, s.postMs, s.messageNumber, at(s.atMs), s.tUs, s.tS, s.refused]);
      }
      for (const e of p.ends) {
        await c.run(`INSERT INTO moves (boot_id, move, end_message_number, end_at, end_t_us, end_t_s, "end", frames_sent, frames_dropped,
          moved, rotated, imu_sent, imu_dropped) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13)
          ON CONFLICT (boot_id, move) DO UPDATE SET end_message_number = excluded.end_message_number, end_at = excluded.end_at, end_t_us = excluded.end_t_us,
          end_t_s = excluded.end_t_s, "end" = excluded."end", frames_sent = excluded.frames_sent, frames_dropped = excluded.frames_dropped,
          moved = excluded.moved, rotated = excluded.rotated, imu_sent = excluded.imu_sent, imu_dropped = excluded.imu_dropped`,
          [e.boot, e.move, e.messageNumber, at(e.atMs), e.tUs, e.tS, e.end, e.framesSent, e.framesDropped, e.moved, e.rotated,
           e.imuSent, e.imuDropped]);
      }
      if (p.frames.length) {
        const fa = await c.createAppender("frames"), za = await c.createAppender("zones");
        for (const r of p.frames) appendFrame(fa, za, r);
        fa.closeSync();
        za.closeSync();
      }
      if (p.imu.length) {
        const a = await c.createAppender("imu");
        for (const r of p.imu) appendImu(a, r);
        a.closeSync();
      }
      if (p.status.length) {
        const a = await c.createAppender("status");
        for (const r of p.status) appendStatus(a, r);
        a.closeSync();
      }
      for (const b of p.boots.values()) await saveBootState(c, b);
      await c.run("COMMIT");
    });
  }

  // A snapshot as it came (bytes) and as parsed, its boot's message number; image: its
  // PNG, relative to recordings/. Replaces what was there for it.
  async importSnapshot(boot: Boot, messageNumber: number, s: Snapshot, bytes: Buffer, arrivalMs: number, image: string | null): Promise<void> {
    const id = snapshotId(s.bootId, s.sequence);
    const p = s.pose, t = s.tof, cam = s.camera, col = s.color;
    await this.flush(); // the path and the frames up to it go in first
    await this.use(async c => {
      await c.run("BEGIN TRANSACTION");
      for (const table of ["snapshots", "snapshot_zones", "snapshot_features"]) await c.run(`DELETE FROM ${table} WHERE snapshot_id = $1`, [id]);
      const a = await c.createAppender("snapshots");
      const d = (v: number | null | undefined) => (v === null || v === undefined ? a.appendNull() : a.appendDouble(v));
      const i = (v: number | null | undefined) => (v === null || v === undefined ? a.appendNull() : a.appendInteger(v));
      const t8 = (v: string | null) => (v === null ? a.appendNull() : a.appendVarchar(v));
      a.appendVarchar(id); a.appendVarchar(s.bootId); a.appendInteger(s.sequence); a.appendInteger(messageNumber); a.appendUBigInt(BigInt(s.robotClockUs));
      a.appendTimestamp(at(arrivalMs));
      t8(s.why); t8(s.action); t8(s.endReason); a.appendInteger(s.actionsEnded); t8(s.running); t8(s.mode);
      t8(s.build); a.appendInteger(s.length); a.appendBlob(new Uint8Array(bytes));
      for (const v of [p?.x, p?.y, p?.yaw, p?.pitch, p?.roll, p?.v, p?.w]) d(v);
      if (p) { a.appendBoolean(p.stationary); a.appendBoolean(p.motorsOn); a.appendInteger(p.stopReason); }
      else { a.appendNull(); a.appendNull(); a.appendNull(); }
      if (t) a.appendUBigInt(BigInt(t.number)); else a.appendNull();
      for (const v of t?.poseKnown ? [t.x, t.y, t.yaw, t.pitch] : [null, null, null, null]) d(v);
      i(cam?.number); d(cam ? cam.exposureUs / 1000 : null); d(cam?.gain); t8(image);
      d(col?.lux); d(col?.cie?.x); d(col?.cie?.y); d(col?.xyz?.X); d(col?.xyz?.Y); d(col?.xyz?.Z);
      if (col) a.appendBlob(new Uint8Array(col.raw)); else a.appendNull();
      a.endRow();
      a.closeSync();
      const zs = await c.createAppender("snapshot_zones");
      t?.zones.forEach((z, k) => {
        const [row, colN] = rowCol(k);
        zs.appendVarchar(id); zs.appendTinyInt(row); zs.appendTinyInt(colN); zs.appendSmallInt(z.distanceMm); zs.appendVarchar(z.kind);
        zs.appendSmallInt(z.signal); zs.appendSmallInt(z.sigmaMm); zs.appendTinyInt(z.targets);
        zs.endRow();
      });
      zs.closeSync();
      const ft = await c.createAppender("snapshot_features");
      for (const f of s.features?.features ?? []) {
        ft.appendVarchar(id); ft.appendInteger(f.id); ft.appendInteger(f.age); ft.appendInteger(f.flags);
        for (const v of [f.x, f.y, f.dx, f.dy, f.strength, f.match]) ft.appendDouble(v);
        ft.endRow();
      }
      ft.closeSync();
      await saveBootState(c, boot);
      await c.run("COMMIT");
    });
  }

  // --- reads ---

  // The path driven up to snapshot sequence of a boot: the statuses' x, y (m), oldest
  // first, since power-up.
  async path(boot: string, sequence: number): Promise<{ x: number; y: number }[]> {
    return this.use(async c => {
      const rows = (await c.runAndReadAll(`
        SELECT x, y FROM status WHERE boot_id = $1
          AND message_number <= (SELECT message_number FROM snapshots WHERE boot_id = $1 AND sequence = $2)
        ORDER BY message_number`, [boot, sequence])).getRowObjects();
      return rows.map(r => ({ x: Number(r.x), y: Number(r.y) }));
    });
  }

  // A boot named by its id (8 hex digits); null if the store doesn't have it.
  async hasBoot(boot: string): Promise<boolean> {
    const rows = await this.read(async c => (await c.runAndReadAll("SELECT 1 FROM boots WHERE boot_id = $1", [boot])).getRowObjects());
    return rows.length > 0;
  }

  // A move of a boot and its frames (with their zones), oldest first, and how many imu
  // records it has; null if there is no such move.
  async moveFrames(boot: string, move: number): Promise<{ move: MoveRow; frames: StoredFrame[]; imu: ImuCount } | null> {
    return this.use(async c => {
      const [m] = (await c.runAndReadAll("SELECT * FROM moves WHERE boot_id = $1 AND move = $2", [boot, move])).getRowObjects();
      if (!m) return null;
      const frames = (await c.runAndReadAll("SELECT * FROM frames WHERE boot_id = $1 AND move = $2 ORDER BY number", [boot, move])).getRowObjects();
      const zones = (await c.runAndReadAll(`SELECT z.* FROM zones z JOIN frames f ON f.boot_id = z.boot_id AND f.number = z.number
        WHERE f.boot_id = $1 AND f.move = $2 ORDER BY z.number, z.row, z.col`, [boot, move])).getRowObjects();
      return { move: moveRow(m), frames: framesOf(frames, zones), imu: await imuCount(c, boot, move) };
    });
  }

  // A boot's frames with message numbers after `after`, up to `upTo`, in the order they
  // came (the PC's cell map: maps/cell_map.ts).
  async framesBetween(boot: string, after: number, upTo: number): Promise<StoredFrame[]> {
    return this.use(async c => {
      const range = [boot, after, upTo];
      const frames = (await c.runAndReadAll(`SELECT * FROM frames WHERE boot_id = $1 AND message_number > $2 AND message_number <= $3
        ORDER BY message_number`, range)).getRowObjects();
      const zones = (await c.runAndReadAll(`SELECT z.* FROM zones z JOIN frames f ON f.boot_id = z.boot_id AND f.number = z.number
        WHERE f.boot_id = $1 AND f.message_number > $2 AND f.message_number <= $3 ORDER BY z.number, z.row, z.col`, range)).getRowObjects();
      return framesOf(frames, zones);
    });
  }

  // A snapshot as the PC's cell map needs it: the message number of the boot's last frame
  // before it (0: none), and the robot's pose in it (null: PicoB not connected); null if
  // there is no such snapshot.
  async snapshotFrames(boot: string, sequence: number): Promise<{ lastFrame: number; pose: Pose | null } | null> {
    return this.use(async c => {
      const [r] = (await c.runAndReadAll(`SELECT x, y, yaw, (SELECT coalesce(max(f.message_number), 0) FROM frames f
        WHERE f.boot_id = s.boot_id AND f.message_number < s.message_number) AS last_frame
        FROM snapshots s WHERE boot_id = $1 AND sequence = $2`, [boot, sequence])).getRowObjects();
      if (!r) return null;
      return { lastFrame: Number(r.last_frame), pose: r.x === null ? null : { x: Number(r.x), y: Number(r.y), yaw: Number(r.yaw) } };
    });
  }

  // A move of a boot and its imu records, oldest first, and how many frames it has;
  // null if there is no such move.
  async moveImu(boot: string, move: number): Promise<{ move: MoveRow; frames: number; imu: ImuCount; records: StoredImu[] } | null> {
    return this.use(async c => {
      const [m] = (await c.runAndReadAll("SELECT * FROM moves WHERE boot_id = $1 AND move = $2", [boot, move])).getRowObjects();
      if (!m) return null;
      const rows = (await c.runAndReadAll("SELECT * FROM imu WHERE boot_id = $1 AND move = $2 ORDER BY t_s, message_number, n", [boot, move])).getRowObjects();
      const [f] = (await c.runAndReadAll("SELECT count(*) AS n FROM frames WHERE boot_id = $1 AND move = $2", [boot, move])).getRowObjects();
      return { move: moveRow(m), frames: Number(f.n), imu: await imuCount(c, boot, move), records: rows.map(imuOf) };
    });
  }

  // Every message a boot sent that the store keeps, in the order they came.
  async replay(boot: string): Promise<Replayed[]> {
    return this.use(async c => {
      const out: Replayed[] = [];
      const rows = (sql: string) => c.runAndReadAll(sql, [boot]).then(r => r.getRowObjects());
      for (const m of await rows("SELECT * FROM moves WHERE boot_id = $1")) {
        const base = { boot, move: Number(m.move) };
        if (m.start_message_number !== null) {
          out.push({ messageNumber: Number(m.start_message_number), atMs: msOf(m.start_at), type: "move_start", start: {
            ...base, messageNumber: Number(m.start_message_number), atMs: msOf(m.start_at), command: Number(m.command), kind: String(m.kind), parameter: Number(m.parameter),
            tUs: Number(m.start_t_us), preMs: Number(m.pre_ms), postMs: Number(m.post_ms), refused: Boolean(m.refused), tS: Number(m.start_t_s) } });
        }
        if (m.end_message_number !== null) {
          out.push({ messageNumber: Number(m.end_message_number), atMs: msOf(m.end_at), type: "move_end", end: {
            ...base, messageNumber: Number(m.end_message_number), atMs: msOf(m.end_at), end: String(m.end), tUs: Number(m.end_t_us),
            framesSent: Number(m.frames_sent), framesDropped: Number(m.frames_dropped),
            moved: Number(m.moved), rotated: Number(m.rotated), imuSent: Number(m.imu_sent ?? 0),
            imuDropped: Number(m.imu_dropped ?? 0), tS: Number(m.end_t_s) } });
        }
      }
      const frames = await rows("SELECT * FROM frames WHERE boot_id = $1 ORDER BY number");
      const zones = await rows("SELECT * FROM zones WHERE boot_id = $1 ORDER BY number, row, col");
      framesOf(frames, zones).forEach((f, k) => out.push({ messageNumber: Number(frames[k].message_number), atMs: msOf(frames[k].arrived_at), type: "tof",
        tof: { format: f.format, move: f.move, tUs: Math.round(f.tS * 1e6), frame: f.frame } }));
      // An imu batch: its rows (one message), in their order.
      const batches = new Map<number, { atMs: number; batch: ImuBatch }>();
      for (const r of await rows("SELECT * FROM imu WHERE boot_id = $1 ORDER BY message_number, n")) {
        const messageNumber = Number(r.message_number);
        const b = batches.get(messageNumber) ?? batches.set(messageNumber, { atMs: msOf(r.arrived_at), batch: { move: Number(r.move), records: [] } }).get(messageNumber)!;
        const { tUs, acceleration, gyro } = imuOf(r);
        b.batch.records.push({ tUs, acceleration, gyro });
      }
      for (const [messageNumber, b] of batches) out.push({ messageNumber, atMs: b.atMs, type: "imu_batch", batch: b.batch });
      for (const s of await rows("SELECT * FROM status WHERE boot_id = $1")) {
        out.push({ messageNumber: Number(s.message_number), atMs: msOf(s.arrived_at), type: "status", pose: {
          x: Number(s.x), y: Number(s.y), yaw: Number(s.yaw), mode: String(s.mode), action: String(s.action), rssi: num(s.rssi), at: msOf(s.arrived_at) } });
      }
      for (const s of await rows("SELECT message_number, arrived_at, bytes FROM snapshots WHERE boot_id = $1")) {
        out.push({ messageNumber: Number(s.message_number), atMs: msOf(s.arrived_at), type: "snapshot", bytes: Buffer.from((s.bytes as { bytes: Uint8Array }).bytes) });
      }
      return out.sort((a, b) => a.messageNumber - b.messageNumber);
    });
  }
}

function appendFrame(fa: DuckDBAppender, za: DuckDBAppender, r: TofRow): void {
  const f = r.frame;
  fa.appendVarchar(r.boot); fa.appendUBigInt(BigInt(f.number));
  fa.appendInteger(r.move);
  fa.appendInteger(r.messageNumber); fa.appendTimestamp(at(r.atMs)); fa.appendVarchar(r.format);
  fa.appendDouble(r.tS); fa.appendDouble(f.periodUs / 1000); fa.appendUBigInt(BigInt(f.robotClockUs));
  for (const v of [f.x, f.y, f.yaw, f.pitch, f.turn]) fa.appendDouble(v);
  fa.appendBoolean(f.still); fa.appendBoolean(f.poseKnown); fa.appendInteger(f.beyondMm);
  fa.endRow();
  f.zones.forEach((z, k) => {
    const [row, col] = rowCol(k);
    za.appendVarchar(r.boot); za.appendUBigInt(BigInt(f.number)); za.appendTinyInt(row); za.appendTinyInt(col);
    za.appendSmallInt(z.distanceMm); za.appendVarchar(z.kind); za.appendSmallInt(z.signal); za.appendSmallInt(z.sigmaMm);
    za.appendTinyInt(z.targets);
    za.endRow();
  });
}

function appendImu(a: DuckDBAppender, r: ImuBatchRow): void {
  r.records.forEach((rec, n) => {
    a.appendVarchar(r.boot); a.appendInteger(r.move); a.appendInteger(r.messageNumber); a.appendTinyInt(n); a.appendTimestamp(at(r.atMs));
    a.appendUBigInt(BigInt(rec.tUs)); a.appendDouble(rec.tS);
    for (const v of [...rec.acceleration, ...rec.gyro]) a.appendSmallInt(v);
    a.endRow();
  });
}

function appendStatus(a: DuckDBAppender, r: StatusRow): void {
  const p = r.pose;
  a.appendVarchar(r.boot); a.appendInteger(r.messageNumber); a.appendTimestamp(at(r.atMs));
  a.appendDouble(p.x); a.appendDouble(p.y); a.appendDouble(p.yaw); a.appendVarchar(p.mode); a.appendVarchar(p.action);
  if (p.rssi === null) a.appendNull(); else a.appendInteger(p.rssi);
  a.endRow();
}

// A move's imu records stored, its batches stored (of the move end's sent), and the
// records missing between them: two stored records more than 1.5 periods apart have
// round(gap / period) - 1 missing between them (a batch dropped or lost, a record lost
// on the UART, a sample skipped on PicoB). Records lost before the first stored one or
// after the last are not seen.
export interface ImuCount { records: number; batches: number; missing: number }
const IMU_PERIOD_S = IMU_RECORD_PERIOD_US / 1e6;
async function imuCount(c: DuckDBConnection, boot: string, move: number): Promise<ImuCount> {
  const [r] = (await c.runAndReadAll(`SELECT count(*) AS records, count(DISTINCT message_number) AS batches,
      coalesce(sum(CASE WHEN gap > 1.5 * $3 THEN round(gap / $3) - 1 ELSE 0 END), 0) AS missing
    FROM (SELECT message_number, t_s - lag(t_s) OVER (ORDER BY t_s, message_number, n) AS gap FROM imu WHERE boot_id = $1 AND move = $2)`,
    [boot, move, IMU_PERIOD_S])).getRowObjects();
  return { records: Number(r.records), batches: Number(r.batches), missing: Number(r.missing) };
}

function imuOf(r: Record<string, unknown>): StoredImu {
  return { tUs: Number(r.t_us), tS: Number(r.t_s), acceleration: [Number(r.ax), Number(r.ay), Number(r.az)] as Vec3,
           gyro: [Number(r.gx), Number(r.gy), Number(r.gz)] as Vec3 };
}

async function saveBootState(c: DuckDBConnection, b: Boot): Promise<void> {
  await c.run("UPDATE boots SET messages = $2 WHERE boot_id = $1", [b.id, b.messages]);
}

function moveRow(m: Record<string, unknown>): MoveRow {
  return {
    move: Number(m.move), command: num(m.command), kind: m.kind === null ? null : String(m.kind), parameter: num(m.parameter),
    preMs: num(m.pre_ms), postMs: num(m.post_ms), refused: Boolean(m.refused), startTS: num(m.start_t_s), endTS: num(m.end_t_s),
    end: m.end === null ? null : String(m.end), framesSent: num(m.frames_sent), framesDropped: num(m.frames_dropped),
    moved: num(m.moved), rotated: num(m.rotated),
    imuSent: num(m.imu_sent), imuDropped: num(m.imu_dropped),
  };
}

// Frame rows and their zone rows (both by number, the zones by row and column) as frames.
function framesOf(frames: Record<string, unknown>[], zones: Record<string, unknown>[]): StoredFrame[] {
  const byNumber = new Map<number, Record<string, unknown>[]>();
  for (const z of zones) {
    const n = Number(z.number);
    (byNumber.get(n) ?? byNumber.set(n, []).get(n)!).push(z);
  }
  return frames.map(f => {
    const n = Number(f.number);
    return {
      format: String(f.format), move: Number(f.move), tS: Number(f.t_s),
      frame: {
        number: n, robotClockUs: Number(f.robot_clock_us), periodUs: Math.round(Number(f.period_ms) * 1000),
        x: Number(f.x), y: Number(f.y), yaw: Number(f.yaw), pitch: Number(f.pitch), turn: Number(f.turn),
        still: Boolean(f.still), poseKnown: Boolean(f.pose_known), beyondMm: Number(f.beyond_mm),
        zones: (byNumber.get(n) ?? []).map(z => ({ distanceMm: Number(z.distance_mm), signal: Number(z.signal), sigmaMm: Number(z.sigma_mm),
                                                  kind: (KINDS.includes(z.kind as Kind) ? z.kind : "nothing") as Kind, targets: Number(z.targets) })),
      },
    };
  });
}
