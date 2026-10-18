// The objects' rows in DuckDB (the tables are in store.ts' SCHEMA): a detection run on a
// snapshot with its boxes and their shapes on the map, saved replacing that snapshot's
// earlier run; read back for one snapshot, and the boot's objects up to a snapshot.
import { timestampValue } from "@duckdb/node-api";
import type { Store } from "../store.ts";
import type { Detection } from "./labels.ts";
import type { Placed } from "./place.ts";

// model: who found the boxes (the vision model, or detect_save's --by). From detect_save
// the rest may be null: an outside agent doesn't say how it worked.
export interface DetectRun {
  boot: string; seq: number; atMs: number; model: string; promptVersion: number | null; thinking: boolean | null;
  reasoningBudget: number | null; seconds: number | null; turns: number | null; result: "ok" | "failed"; why: string | null; transcript: string | null;
}

export interface StoredObject {
  object_id: string; seq: number; n: number; label: string; floor: string; kind: string | null; distance: number | null;
  cx: number | null; cy: number | null; width: number | null; depth: number | null; angle: number | null;
  ymin: number; xmin: number; ymax: number; xmax: number; confidence: number | null;
}

const snapId = (boot: string, seq: number) => `${boot}-${seq}`;
const localUs = (ms: number) => BigInt(Math.round((ms - new Date(ms).getTimezoneOffset() * 60000) * 1000));

// Saves a run, replacing the snapshot's earlier one (to tweak later: keep every run).
export async function saveRun(store: Store, run: DetectRun, detections: Detection[], placed: Placed[]): Promise<void> {
  const id = snapId(run.boot, run.seq);
  await store.write(async c => {
    await c.run("BEGIN TRANSACTION");
    const objects = (await c.runAndReadAll("SELECT object_id FROM object_detections WHERE snap_id = $1", [id])).getRowObjects();
    for (const o of objects) await c.run("DELETE FROM map_objects WHERE object_id = $1", [String(o.object_id)]);
    for (const table of ["detect_runs", "detections", "object_detections"]) await c.run(`DELETE FROM ${table} WHERE snap_id = $1`, [id]);
    await c.run(`INSERT INTO detect_runs VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13)`,
      [id, run.boot, run.seq, timestampValue(localUs(run.atMs)), run.model, run.promptVersion, run.thinking, run.seconds,
       run.turns, run.result, run.why, run.transcript, run.reasoningBudget]);
    for (const [n, d] of detections.entries()) {
      const p = placed[n], objectId = `${id}-${n}`;
      await c.run("INSERT INTO detections VALUES ($1, $2, $3, $4, $5, $6, $7, $8)", [id, n, d.label, ...d.bbox, d.confidence]);
      await c.run("INSERT INTO map_objects VALUES ($1, $2, $3, $4, 'camera', $5, $6, $7, $8, $9, $10, $11, $12)",
        [objectId, run.boot, run.seq, d.label, p.floor, p.kind, p.distanceM, p.cx, p.cy, p.width, p.depth, p.angle]);
      await c.run("INSERT INTO object_detections VALUES ($1, $2, $3)", [objectId, id, n]);
    }
    await c.run("COMMIT");
  });
}

const OBJECTS_SQL = `SELECT o.object_id, o.seq, od.n, o.label, o.floor, o.kind, o.distance, o.cx, o.cy, o.width, o.depth, o.angle,
    d.ymin, d.xmin, d.ymax, d.xmax, d.confidence
  FROM map_objects o JOIN object_detections od USING (object_id) JOIN detections d ON d.snap_id = od.snap_id AND d.n = od.n`;

const numberOrNull = (v: unknown) => (v === null || v === undefined ? null : Number(v));
function objectRow(r: Record<string, unknown>): StoredObject {
  return {
    object_id: String(r.object_id), seq: Number(r.seq), n: Number(r.n), label: String(r.label), floor: String(r.floor),
    kind: r.kind === null ? null : String(r.kind), distance: numberOrNull(r.distance),
    cx: numberOrNull(r.cx), cy: numberOrNull(r.cy), width: numberOrNull(r.width), depth: numberOrNull(r.depth), angle: numberOrNull(r.angle),
    ymin: Number(r.ymin), xmin: Number(r.xmin), ymax: Number(r.ymax), xmax: Number(r.xmax), confidence: numberOrNull(r.confidence),
  };
}

// The snapshot's run and its objects; null if detect never ran on it.
export async function runOf(store: Store, boot: string, seq: number) {
  const id = snapId(boot, seq);
  return store.read(async c => {
    const run = (await c.runAndReadAll(`SELECT model, prompt_version, thinking, reasoning_budget, seconds, turns, result, why,
      strftime(ran_at, '%Y-%m-%d %H:%M:%S') AS ran_at FROM detect_runs WHERE snap_id = $1`, [id])).getRowObjectsJson()[0];
    if (!run) return null;
    const objects = (await c.runAndReadAll(`${OBJECTS_SQL} WHERE od.snap_id = $1 ORDER BY od.n`, [id])).getRowObjectsJson();
    return { run: run as Record<string, unknown>, objects: objects.map(o => objectRow(o as Record<string, unknown>)) };
  });
}

// The boot's objects from snapshots up to seq (the map as the page shows it then).
export async function objectsUpTo(store: Store, boot: string, seq: number): Promise<StoredObject[]> {
  return store.read(async c => (await c.runAndReadAll(`${OBJECTS_SQL} WHERE o.boot_id = $1 AND o.seq <= $2 ORDER BY o.seq, od.n`,
    [boot, seq])).getRowObjectsJson().map(o => objectRow(o as Record<string, unknown>)));
}

// For the page: the snapshot's run and boxes, and the boot's objects up to it.
export async function pageObjects(store: Store, boot: string, seq: number) {
  const here = await runOf(store, boot, seq), all = await objectsUpTo(store, boot, seq);
  return { run: here?.run ?? null, here: here?.objects ?? [], map: all };
}

// The run's conversation (JSON), for looking at what the model said: null if nothing
// ran; transcript null if detect_save saved it without one.
export async function transcriptOf(store: Store, boot: string, seq: number): Promise<{ by: string; transcript: string | null } | null> {
  const rows = await store.read(async c => (await c.runAndReadAll("SELECT model, transcript FROM detect_runs WHERE snap_id = $1",
    [snapId(boot, seq)])).getRowObjects());
  if (!rows.length) return null;
  return { by: String(rows[0].model), transcript: rows[0].transcript === null ? null : String(rows[0].transcript) };
}

// Places every saved detection again with the camera's numbers now (place.ts), from
// the boxes as the model gave them and the snapshots' poses: after a calibration. The
// model isn't asked again. Returns how many objects were placed.
export async function placeAgain(store: Store, placeOne: (d: Pick<Detection, "label" | "bbox">, pose: { x: number; y: number; yaw: number; pitch: number }) => Placed): Promise<number> {
  return store.write(async c => {
    const rows = (await c.runAndReadAll(`SELECT od.object_id, d.label, d.ymin, d.xmin, d.ymax, d.xmax, s.x, s.y, s.yaw, s.pitch
      FROM object_detections od JOIN detections d ON d.snap_id = od.snap_id AND d.n = od.n JOIN snapshots s ON s.snap_id = od.snap_id
      WHERE s.x IS NOT NULL`)).getRowObjects();
    await c.run("BEGIN TRANSACTION");
    for (const r of rows) {
      const p = placeOne({ label: String(r.label) as Detection["label"], bbox: [Number(r.ymin), Number(r.xmin), Number(r.ymax), Number(r.xmax)] },
                         { x: Number(r.x), y: Number(r.y), yaw: Number(r.yaw), pitch: Number(r.pitch ?? 0) });
      await c.run(`UPDATE map_objects SET floor = $2, kind = $3, distance = $4, cx = $5, cy = $6, width = $7, depth = $8, angle = $9
        WHERE object_id = $1`, [String(r.object_id), p.floor, p.kind, p.distanceM, p.cx, p.cy, p.width, p.depth, p.angle]);
    }
    await c.run("COMMIT");
    return rows.length;
  });
}
