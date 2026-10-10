// The viewer's data (doc/TELEMETRY_PLAN.md §4): a take's frames, targets and path,
// both replayed maps, the truth boxes Daniel places, and how well each map finds them.
//   GET    /api/takes/<id>/view          the take, its frames, targets (world x, y, z), odometry
//   GET    /api/takes/<id>/maps?margin=  both variants' cells (replay.ts)
//   GET    /api/takes/<id>/score?margin= per variant: each truth box found or not, cells not explained
//   GET    /api/truth?boot=<boot id>     the truth boxes of a boot (one odometry frame per boot)
//   POST   /api/truth                    {boot_id, name, x, y, z, size_x, size_y, size_z, note} -> {truth_id}
//   DELETE /api/truth/<truth id>
import type http from "node:http";
import type { Store } from "./store.ts";
import { COLUMN, isTakeId, replayMaps, type Cell } from "./replay.ts";

export interface Truth { truth_id: string; boot_id: string; name: string; x: number; y: number; z: number; size_x: number; size_y: number; size_z: number; note: string }
export interface Score { objects: { truth_id: string; name: string; found: boolean; cells: number }[]; unexplained: number; solid: number }

const CELL_M = 0.1;
const SCORE_RANGE_M = 1.5; // cells farther from where the take started don't count as unexplained

// Which truth boxes a map finds: a box counts as found when a blocked or overhang
// column overlaps its footprint. Solid columns within 1.5 m that overlap no box
// (grown by one cell) are "unexplained": false cells, or objects not placed yet.
export function score(cells: Cell[], truth: Truth[]): Score {
  const solid = cells.filter(c => c[2] === COLUMN.BLOCKED || c[2] === COLUMN.OVERHANG);
  const overlaps = (c: Cell, t: Truth, grow: number) =>
    Math.abs(c[0] - t.x) < (t.size_x + CELL_M) / 2 + grow && Math.abs(c[1] - t.y) < (t.size_y + CELL_M) / 2 + grow;
  return {
    objects: truth.map(t => {
      const n = solid.filter(c => overlaps(c, t, 0)).length;
      return { truth_id: t.truth_id, name: t.name, found: n > 0, cells: n };
    }),
    unexplained: solid.filter(c => Math.hypot(c[0], c[1]) <= SCORE_RANGE_M && !truth.some(t => overlaps(c, t, CELL_M))).length,
    solid: solid.filter(c => Math.hypot(c[0], c[1]) <= SCORE_RANGE_M).length,
  };
}

// Columns of numbers (smaller than objects), floats rounded.
function columns(rows: Record<string, unknown>[], names: string[], decimals: Record<string, number> = {}): Record<string, number[]> {
  const out: Record<string, number[]> = {};
  for (const n of names) {
    const k = 10 ** (decimals[n] ?? -1);
    out[n] = rows.map(r => {
      const v = Number(r[n]);
      return decimals[n] === undefined ? v : Math.round(v * k) / k;
    });
  }
  return out;
}

export async function takeView(store: Store, id: string) {
  const [take] = await store.query(`SELECT take_id, boot_id, take_no, key, action, round(duration_s, 1) AS duration_s, end_reason,
    strftime(started_at, '%Y-%m-%d %H:%M:%S') AS started, geometry, note FROM takes WHERE take_id = '${id}'`);
  if (!take) return null;
  const frames = await store.query(`SELECT frame_no, t_s, x, y, yaw, pitch, zones FROM tof_frames
    WHERE take_id = '${id}' AND x IS NOT NULL ORDER BY frame_no`);
  const points = await store.query(`SELECT frame_no, row, col, target, distance_mm, status, signal, x, y, z FROM tof_targets
    WHERE take_id = '${id}' AND x IS NOT NULL ORDER BY frame_no, row, col, target`);
  const odom = await store.query(`SELECT t_s, x, y, yaw FROM odom WHERE take_id = '${id}' ORDER BY t_s`);
  return {
    take: { ...take, geometry: take.geometry ? JSON.parse(take.geometry as string) : null },
    frames: columns(frames, ["frame_no", "t_s", "x", "y", "yaw", "pitch", "zones"], { t_s: 3, x: 4, y: 4, yaw: 4, pitch: 4 }),
    points: columns(points, ["frame_no", "row", "col", "target", "distance_mm", "status", "signal", "x", "y", "z"], { x: 3, y: 3, z: 3 }),
    odom: columns(odom, ["t_s", "x", "y", "yaw"], { t_s: 3, x: 4, y: 4, yaw: 4 }),
  };
}

const num = (v: unknown, fallback: number) => (typeof v === "number" && Number.isFinite(v) ? v : fallback);
const str = (v: unknown) => (typeof v === "string" ? v.slice(0, 200) : "");
const quote = (s: string) => `'${s.replace(/'/g, "''")}'`;

export async function listTruth(store: Store, boot: string): Promise<Truth[]> {
  if (!/^[0-9a-f]{8}$/.test(boot)) throw new Error(`not a boot id: ${boot}`);
  return (await store.query(`SELECT truth_id, boot_id, name, x, y, z, size_x, size_y, size_z, note FROM truth
    WHERE boot_id = '${boot}' AND truth_id IS NOT NULL ORDER BY name`)) as unknown as Truth[];
}

export async function addTruth(store: Store, b: Record<string, unknown>): Promise<string> {
  const boot = str(b.boot_id);
  if (!/^[0-9a-f]{8}$/.test(boot)) throw new Error("boot_id missing");
  const id = `t${Date.now().toString(36)}${Math.floor(Math.random() * 1e4).toString(36)}`;
  const v = [num(b.x, 0), num(b.y, 0), num(b.z, 0), Math.max(0.01, num(b.size_x, 0.1)), Math.max(0.01, num(b.size_y, 0.1)),
    Math.max(0.01, num(b.size_z, 0.1))];
  await store.query(`INSERT INTO truth (truth_id, boot_id, take_id, name, x, y, z, size_x, size_y, size_z, note)
    VALUES (${quote(id)}, ${quote(boot)}, NULL, ${quote(str(b.name) || "object")}, ${v.join(", ")}, ${quote(str(b.note))})`);
  return id;
}

export async function deleteTruth(store: Store, id: string): Promise<void> {
  if (!/^t[0-9a-z]+$/.test(id)) throw new Error("not a truth id");
  await store.query(`DELETE FROM truth WHERE truth_id = ${quote(id)}`);
}

async function bodyOf(req: http.IncomingMessage): Promise<Record<string, unknown>> {
  let body = "";
  for await (const chunk of req) { body += chunk; if (body.length > 10000) throw new Error("too long"); }
  return JSON.parse(body || "{}");
}

// Answers the viewer's requests; false if the URL isn't one of them.
export async function handleViewerApi(req: http.IncomingMessage, res: http.ServerResponse, store: Store): Promise<boolean> {
  const url = new URL(req.url ?? "/", "http://localhost");
  const send = (code: number, body: unknown) => {
    res.writeHead(code, { "content-type": "application/json", "cache-control": "no-store" });
    res.end(JSON.stringify(body));
  };
  const take = /^\/api\/takes\/([^/]+)\/(view|maps|score)$/.exec(url.pathname);
  const margin = Math.min(1.5, Math.max(0.3, Number(url.searchParams.get("margin") ?? 0.9) || 0.9));
  try {
    if (take && req.method === "GET") {
      const id = take[1];
      if (!isTakeId(id)) { send(400, { error: "not a take id" }); return true; }
      if (take[2] === "view") {
        const view = await takeView(store, id);
        if (view) send(200, view); else send(404, { error: "no such take" });
      } else if (take[2] === "maps") {
        send(200, await replayMaps(store, id, margin));
      } else {
        const [row] = await store.query(`SELECT boot_id FROM takes WHERE take_id = '${id}'`);
        if (!row) { send(404, { error: "no such take" }); return true; }
        const [maps, truth] = await Promise.all([replayMaps(store, id, margin), listTruth(store, row.boot_id as string)]);
        send(200, { votes: score(maps.votes, truth), readings: score(maps.readings, truth) });
      }
      return true;
    }
    if (url.pathname === "/api/truth" && req.method === "GET") { send(200, await listTruth(store, url.searchParams.get("boot") ?? "")); return true; }
    if (url.pathname === "/api/truth" && req.method === "POST") { send(200, { truth_id: await addTruth(store, await bodyOf(req)) }); return true; }
    const del = /^\/api\/truth\/([^/]+)$/.exec(url.pathname);
    if (del && req.method === "DELETE") { await deleteTruth(store, del[1]); send(200, { ok: true }); return true; }
  } catch (err) {
    send(500, { error: (err as Error).message });
    return true;
  }
  return false;
}
