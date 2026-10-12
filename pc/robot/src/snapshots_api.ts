// The viewer's snapshot data (doc/ARCHITECTURE.md, the PC; snapshot.ts):
//   GET /api/boots                         each boot with snapshots: how many, first and last
//   GET /api/snapshots?boot=<id>           a boot's snapshots for the timeline: when, why, the action, the pose
//   GET /api/snapshots/<boot>/<seq>        one snapshot: its row, the ToF frame (8 x 8 grid and targets in the world),
//                                          the camera features
//   GET /api/snapshots/<boot>/<seq>/map    the robot's map then: the window's columns over the whole-house
//                                          map's tiles as they were (the newest of each tile up to this
//                                          snapshot since the map was last cleared)
//   GET /snapshots/<boot>/<seq>.png        the camera picture
import type http from "node:http";
import { readFile } from "node:fs/promises";
import path from "node:path";
import type { Store } from "./store.ts";
import { tileColumns, windowColumns } from "./snapshot.ts";

const BOOT = /^[0-9a-f]{8}$/;

export interface MapView {
  window: { orgX: number; orgY: number; cellsX: number; cellsY: number; columns: number[]; confidence: number[] } | null;
  tiles: { tx: number; ty: number; seq: number; columns: number[] }[];
}

const bytes = (v: unknown): Buffer => Buffer.from((v as { bytes: Uint8Array }).bytes);

export async function listBoots(store: Store) {
  return store.query(`SELECT boot_id, count(*) AS snapshots, strftime(min(arrived_at), '%Y-%m-%d %H:%M:%S') AS first,
    strftime(max(arrived_at), '%Y-%m-%d %H:%M:%S') AS last FROM snapshots GROUP BY boot_id ORDER BY max(arrived_at) DESC`);
}

export async function listSnapshots(store: Store, boot: string) {
  if (!BOOT.test(boot)) throw new Error(`not a boot id: ${boot}`);
  return store.query(`SELECT seq, strftime(arrived_at, '%H:%M:%S') AS arrived, t_us, why, action, end_reason, running, mode,
    round(x, 4) AS x, round(y, 4) AS y, round(yaw, 4) AS yaw, image IS NOT NULL AS has_image, round(lux, 1) AS lux, tiles
    FROM snapshots WHERE boot_id = '${boot}' ORDER BY seq`);
}

export async function snapshotDetail(store: Store, boot: string, seq: number) {
  if (!BOOT.test(boot) || !Number.isInteger(seq)) throw new Error("not a snapshot");
  const id = `${boot}-${seq}`;
  const [row] = await store.query(`SELECT * EXCLUDE (map_cells, color_raw), strftime(arrived_at, '%Y-%m-%d %H:%M:%S') AS arrived
    FROM snapshots WHERE snap_id = '${id}'`);
  if (!row) return null;
  const targets = await store.query(`SELECT row, col, target, distance_mm, sigma_mm, signal, status,
    round(x, 3) AS x, round(y, 3) AS y, round(z, 3) AS z FROM snapshot_targets WHERE snap_id = '${id}' ORDER BY row, col, target`);
  // The grid as the robot sees it (row 1 at the top, column 1 on the left): each zone's
  // nearest sure target (status 5, 6, 9), else its first target, else nothing.
  const grid: ({ mm: number; status: number; sure: boolean } | null)[][] = Array.from({ length: 8 }, () => Array(8).fill(null));
  for (const t of targets) {
    const r = Number(t.row) - 1, c = Number(t.col) - 1, status = Number(t.status), sure = [5, 6, 9].includes(status);
    const now = grid[r]?.[c];
    if (r < 0 || r > 7 || c < 0 || c > 7) continue;
    if (!now || (sure && !now.sure)) grid[r][c] = { mm: Number(t.distance_mm), status, sure };
  }
  const features = await store.query(`SELECT id, age, flags, x, y, dx, dy, round(strength, 2) AS strength, round(match, 3) AS match
    FROM snapshot_features WHERE snap_id = '${id}' ORDER BY id`);
  return { snapshot: row, targets, grid, features };
}

export async function snapshotMap(store: Store, boot: string, seq: number): Promise<MapView | null> {
  if (!BOOT.test(boot) || !Number.isInteger(seq)) throw new Error("not a snapshot");
  return store.read(async c => {
    const head = (await c.runAndReadAll(`SELECT map_clears, map_org_cx, map_org_cy, map_cells_x, map_cells_y, map_cells
      FROM snapshots WHERE boot_id = $1 AND seq = $2`, [boot, seq])).getRowObjects();
    if (!head.length) return null;
    const h = head[0];
    const tiles = (await c.runAndReadAll(`SELECT tx, ty, arg_max(seq, seq) AS seq, arg_max(columns, seq) AS columns FROM snapshot_tiles
      WHERE boot_id = $1 AND seq <= $2 AND map_clears = $3 GROUP BY tx, ty ORDER BY tx, ty`, [boot, seq, Number(h.map_clears)])).getRowObjects();
    let window: MapView["window"] = null;
    if (h.map_cells !== null) {
      const w = { orgX: Number(h.map_org_cx), orgY: Number(h.map_org_cy), cellsX: Number(h.map_cells_x), cellsY: Number(h.map_cells_y) };
      window = { ...w, ...windowColumns({ ...w, layers: 3, cells: bytes(h.map_cells) }) };
    }
    return {
      window,
      tiles: tiles.map(t => ({ tx: Number(t.tx), ty: Number(t.ty), seq: Number(t.seq),
                               columns: tileColumns({ tx: 0, ty: 0, columns: bytes(t.columns) }) })),
    };
  });
}

// Answers the snapshot requests; false if the URL isn't one of them. dir: recordings/.
export async function handleSnapshotApi(req: http.IncomingMessage, res: http.ServerResponse, store: Store, dir: string): Promise<boolean> {
  if (req.method !== "GET") return false;
  const url = new URL(req.url ?? "/", "http://localhost");
  const send = (code: number, body: unknown) => {
    res.writeHead(code, { "content-type": "application/json", "cache-control": "no-store" });
    res.end(JSON.stringify(body));
  };
  try {
    if (url.pathname === "/api/boots") { send(200, await listBoots(store)); return true; }
    if (url.pathname === "/api/snapshots") { send(200, await listSnapshots(store, url.searchParams.get("boot") ?? "")); return true; }
    const one = /^\/api\/snapshots\/([0-9a-f]{8})\/(\d+)(\/map)?$/.exec(url.pathname);
    if (one) {
      const body = one[3] ? await snapshotMap(store, one[1], Number(one[2])) : await snapshotDetail(store, one[1], Number(one[2]));
      if (body) send(200, body); else send(404, { error: "no such snapshot" });
      return true;
    }
    const png = /^\/snapshots\/([0-9a-f]{8})\/(\d+)\.png$/.exec(url.pathname);
    if (png) {
      try {
        const data = await readFile(path.join(dir, "snapshots", png[1], `${png[2]}.png`));
        res.writeHead(200, { "content-type": "image/png", "cache-control": "max-age=86400" });
        res.end(data);
      } catch { res.writeHead(404); res.end("no picture"); }
      return true;
    }
  } catch (err) {
    send(500, { error: (err as Error).message });
    return true;
  }
  return false;
}
