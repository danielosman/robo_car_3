// The viewer's snapshot data (doc/ARCHITECTURE.md, the PC; snapshot.ts):
//   GET /api/boots                         each boot with snapshots: its folder name, how many, first and last
//   GET /api/snapshots?boot=<id>           a boot's snapshots for the timeline: when, why, the action, the pose
//   GET /api/snapshots/<boot>/<sequence>        one snapshot: its row, the ToF frame (its 64 zones, each also in the world,
//                                          placed by the PC's cell map helper; the 8 x 8 grid), the camera features
//   GET /api/snapshots/<boot>/<sequence>/map    the PC's cell map then (maps/cell_map.ts: the boot's frames that came
//                                          before the snapshot): its window's columns over its whole-house
//                                          tiles, where the robot was, and the path it drove since power-up
//   GET /api/snapshots/<boot>/<sequence>/objects  what robot detect found in its picture, and the boot's
//                                          objects on the map up to it (objects/api.ts)
//   GET /camera/<boot>/<sequence>.png           the camera picture (its file: camera/<boot's folder>/)
//   GET /api/mounting?boot=<id>            where the sensors sat for that boot: its own copy (boot.ts); without a
//                                          boot (or one the store doesn't know), the file now (common/mounting.json)
import type http from "node:http";
import { readFile } from "node:fs/promises";
import path from "node:path";
import type { Store } from "./store.ts";
import type { Pose } from "./link.ts";
import { parseSnapshot, tileColumns, windowColumns, type Snapshot } from "./snapshot.ts";
import type { CellMaps, CellMapView } from "./maps/cell_map.ts";
import { pageObjects } from "./objects/db.ts";
import type { BootFolders } from "./boot_folders.ts";
import { loadMounting } from "./mounting.ts";
import { COMPACT_BEYOND_MM, zoneAt } from "./tof_format.ts";

const BOOT = /^[0-9a-f]{8}$/;

export interface MapView {
  // cells: the robot's bytes as it holds them (cell_map.h): layers G, L1, L2 one after the other, each
  // [ix * cellsY + iy]; columns and confidence are computed from them.
  window: { originX: number; originY: number; cellsX: number; cellsY: number; cells: Buffer; columns: number[]; confidence: number[] } | null;
  tiles: { tx: number; ty: number; sequence?: number; columns: number[] }[];  // sequence: the snapshot that brought it (the robot's map)
  robot: Pose | null;          // the snapshot's pose
  path: { x: number; y: number }[];
}

// The map of one snapshot on its own (its window, the tiles it brought), without a path.
export function mapViewOf(s: Snapshot): MapView {
  const w = s.window;
  return {
    window: w && { originX: w.originX, originY: w.originY, cellsX: w.cellsX, cellsY: w.cellsY, cells: w.cells, ...windowColumns(w) },
    tiles: s.tiles.map(t => ({ tx: t.tx, ty: t.ty, sequence: s.sequence, columns: tileColumns(t) })),
    robot: s.pose && { x: s.pose.x, y: s.pose.y, yaw: s.pose.yaw },
    path: [],
  };
}

const bytes = (v: unknown): Buffer => Buffer.from((v as { bytes: Uint8Array }).bytes);

// The boots with snapshots, newest first; folder: the boot's folder name (YYYYMMDDHHMM,
// boot_folders.ts), what the page shows; the boot id if it has none.
export async function listBoots(store: Store, bootFolders: BootFolders) {
  const boots = await store.query(`SELECT boot_id, count(*) AS snapshots, strftime(min(arrived_at), '%Y-%m-%d %H:%M:%S') AS first,
    strftime(max(arrived_at), '%Y-%m-%d %H:%M:%S') AS last FROM snapshots GROUP BY boot_id ORDER BY max(arrived_at) DESC`);
  return boots.map((b): Record<string, unknown> => ({ ...b, folder: bootFolders.knownFolder(String(b.boot_id)) ?? String(b.boot_id) }));
}

export async function listSnapshots(store: Store, boot: string) {
  if (!BOOT.test(boot)) throw new Error(`not a boot id: ${boot}`);
  return store.query(`SELECT sequence, strftime(arrived_at, '%H:%M:%S') AS arrived, robot_clock_us, why, action, end_reason, running, mode,
    round(x, 4) AS x, round(y, 4) AS y, round(yaw, 4) AS yaw, image IS NOT NULL AS has_image, round(lux, 1) AS lux, tiles
    FROM snapshots WHERE boot_id = '${boot}' ORDER BY sequence`);
}

export async function snapshotDetail(store: Store, boot: string, sequence: number) {
  if (!BOOT.test(boot) || !Number.isInteger(sequence)) throw new Error("not a snapshot");
  const id = `${boot}-${sequence}`;
  const [row] = await store.query(`SELECT * EXCLUDE (map_cells, color_raw, bytes), strftime(arrived_at, '%Y-%m-%d %H:%M:%S') AS arrived
    FROM snapshots WHERE snapshot_id = '${id}'`);
  if (!row) return null;
  const zones = await store.query(`SELECT row, col, distance_mm, kind, signal, sigma_mm, targets,
    FROM snapshot_zones WHERE snapshot_id = '${id}' ORDER BY row, col`);
  // The grid as the robot sees it (row 1 at the top, column 1 on the left): each zone's
  // reading and how sure (sure, less_sure, unsure); null: nothing in range.
  const grid: ({ mm: number; kind: string; beyond: boolean } | null)[][] = Array.from({ length: 8 }, () => Array(8).fill(null));
  for (const z of zones) {
    const r = Number(z.row) - 1, c = Number(z.col) - 1, kind = String(z.kind), mm = Number(z.distance_mm);
    if (r < 0 || r > 7 || c < 0 || c > 7 || kind === "nothing") continue;
    grid[r][c] = { mm, kind, beyond: kind !== "unsure" && mm >= COMPACT_BEYOND_MM };
  }
  const features = await store.query(`SELECT id, age, flags, x, y, dx, dy, round(strength, 2) AS strength, round(match, 3) AS match
    FROM snapshot_features WHERE snapshot_id = '${id}' ORDER BY id`);
  return { snapshot: row, zones, grid, features };
}

// The robot's own map at a snapshot (robot map --robot): the window it sent in it, over the
// whole-house tiles as they were (the newest of each up to it since the robot's map was
// last cleared), and the path since then.
export async function snapshotMap(store: Store, boot: string, sequence: number): Promise<MapView | null> {
  if (!BOOT.test(boot) || !Number.isInteger(sequence)) throw new Error("not a snapshot");
  const path = await store.path(boot, sequence);
  return store.read(async c => {
    const head = (await c.runAndReadAll(`SELECT map_clears, map_origin_cx, map_origin_cy, map_cells_x, map_cells_y, map_cells, x, y, yaw
      FROM snapshots WHERE boot_id = $1 AND sequence = $2`, [boot, sequence])).getRowObjects();
    if (!head.length) return null;
    const h = head[0];
    const tiles = (await c.runAndReadAll(`SELECT tx, ty, arg_max(sequence, sequence) AS sequence, arg_max(columns, sequence) AS columns FROM snapshot_tiles
      WHERE boot_id = $1 AND sequence <= $2 AND map_clears = $3 GROUP BY tx, ty ORDER BY tx, ty`, [boot, sequence, Number(h.map_clears)])).getRowObjects();
    let window: MapView["window"] = null;
    if (h.map_cells !== null) {
      const w = { originX: Number(h.map_origin_cx), originY: Number(h.map_origin_cy), cellsX: Number(h.map_cells_x), cellsY: Number(h.map_cells_y) };
      const cells = bytes(h.map_cells);
      window = { ...w, cells, ...windowColumns({ ...w, layers: 3, cells }) };
    }
    return {
      window,
      tiles: tiles.map(t => ({ tx: Number(t.tx), ty: Number(t.ty), sequence: Number(t.sequence),
                               columns: tileColumns({ tx: 0, ty: 0, columns: bytes(t.columns) }) })),
      robot: h.x === null ? null : { x: Number(h.x), y: Number(h.y), yaw: Number(h.yaw) },
      path,
    };
  });
}

// A map's cells as the page and robot map show them.
export function mapViewOfCells(v: CellMapView, robot: Pose | null, path: MapView["path"]): MapView {
  return {
    window: { originX: v.originX, originY: v.originY, cellsX: v.cellsX, cellsY: v.cellsY, cells: v.cells, ...windowColumns({ ...v, layers: 3 }) },
    tiles: v.tiles.map(t => ({ tx: t.tx, ty: t.ty, columns: tileColumns(t) })),
    robot, path,
  };
}

// The PC's cell map at a snapshot: the boot's frames that came before it, the snapshot's
// pose, the path since power-up.
export async function cellMapAt(store: Store, maps: CellMaps, boot: string, sequence: number): Promise<MapView | null> {
  if (!BOOT.test(boot) || !Number.isInteger(sequence)) throw new Error("not a snapshot");
  const at = await maps.viewAtSnapshot(boot, sequence);
  return at && mapViewOfCells(at.view, at.pose, await store.path(boot, sequence, false));
}

// A snapshot for the page: snapshotDetail, each zone with its reading in the world (x, y,
// z, m; null without a pose or a valid reading), placed by the PC's cell map helper from
// the snapshot's own ToF frame.
export async function snapshotForPage(store: Store, maps: CellMaps, boot: string, sequence: number) {
  const d = await snapshotDetail(store, boot, sequence);
  if (!d) return null;
  const [row] = await store.read(async c => (await c.runAndReadAll("SELECT bytes FROM snapshots WHERE boot_id = $1 AND sequence = $2",
    [boot, sequence])).getRowObjects());
  const frame = parseSnapshot(bytes(row.bytes)).tof;
  const rays = frame ? await maps.rays(boot, frame) : [];
  const round = (v: number) => Number(v.toFixed(3));
  const zones = d.zones.map((z): Record<string, unknown> => {
    const r = rays[zoneAt(Number(z.row), Number(z.col))];
    return { ...z, x: r ? round(r.point[0]) : null, y: r ? round(r.point[1]) : null, z: r ? round(r.point[2]) : null };
  });
  return { ...d, zones };
}

// The map for the page: the columns it draws, without the robot's cell bytes.
function pageMap(view: MapView | null) {
  if (!view?.window) return view;
  const w = view.window;
  return { ...view, window: { originX: w.originX, originY: w.originY, cellsX: w.cellsX, cellsY: w.cellsY, columns: w.columns, confidence: w.confidence } };
}

// Answers the snapshot requests; false if the URL isn't one of them. directory: recordings/.
export async function handleSnapshotApi(req: http.IncomingMessage, res: http.ServerResponse, store: Store, directory: string,
                                        bootFolders: BootFolders, maps: CellMaps): Promise<boolean> {
  if (req.method !== "GET") return false;
  const url = new URL(req.url ?? "/", "http://localhost");
  const send = (code: number, body: unknown) => {
    res.writeHead(code, { "content-type": "application/json", "cache-control": "no-store" });
    res.end(JSON.stringify(body));
  };
  try {
    if (url.pathname === "/api/mounting") {
      const boot = url.searchParams.get("boot") ?? "";
      send(200, (BOOT.test(boot) && await store.mountingOf(boot)) || loadMounting());
      return true;
    }
    if (url.pathname === "/api/boots") { send(200, await listBoots(store, bootFolders)); return true; }
    if (url.pathname === "/api/snapshots") { send(200, await listSnapshots(store, url.searchParams.get("boot") ?? "")); return true; }
    const one = /^\/api\/snapshots\/([0-9a-f]{8})\/(\d+)(\/map|\/objects)?$/.exec(url.pathname);
    if (one) {
      const boot = one[1], sequence = Number(one[2]);
      const body = one[3] === "/map" ? pageMap(await cellMapAt(store, maps, boot, sequence)) : one[3] === "/objects" ? await pageObjects(store, boot, sequence)
        : await snapshotForPage(store, maps, boot, sequence);
      if (body) send(200, body); else send(404, { error: "no such snapshot" });
      return true;
    }
    const png = /^\/camera\/([0-9a-f]{8})\/(\d+)\.png$/.exec(url.pathname);
    if (png) {
      try {
        // The URL has the boot id; the picture's file (in the boot's folder) is in its row.
        const [row] = await store.query(`SELECT image FROM snapshots WHERE snapshot_id = '${png[1]}-${Number(png[2])}'`);
        if (!row?.image) throw new Error("no picture");
        const data = await readFile(path.join(directory, String(row.image)));
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
