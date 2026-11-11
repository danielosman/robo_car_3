// A map (snapshots_api.ts, MapView: the PC's cell map or the robot's own) as a PNG,
// drawn here on the PC: the whole building (every tile seen and the window), the path,
// the robot. Map frame: x forward at power-up, y left; drawn with x up and y to the
// left, as the page does.
import type { MapView } from "../snapshots_api.ts";
import { COLUMN, TILE_COLS, rgbPng } from "../snapshot.ts";

const CELL_M = 0.1;

// The page's colours (public/app.html).
const COLORS: Record<number, [number, number, number]> = {
  [COLUMN.UNKNOWN]: [0x15, 0x15, 0x15], [COLUMN.BLOCKED]: [0xd0, 0x45, 0x3b], [COLUMN.OVERHANG]: [0xe3, 0x9b, 0x3a],
  [COLUMN.NO_FLOOR]: [0x9b, 0x59, 0xd0], [COLUMN.DRIVABLE]: [0x3d, 0x7f, 0x4f], [COLUMN.OPEN]: [0x2f, 0x4a, 0x3a],
};

// The whole map (its window and every tile) as an RGB PNG, scale pixels a column.
export function mapPng(view: MapView, scale = 6): Buffer {
  const w = view.window;
  let minCx = w.originX, maxCx = w.originX + w.cellsX, minCy = w.originY, maxCy = w.originY + w.cellsY;
  for (const t of view.tiles) {
    const tx0 = t.tx * TILE_COLS - TILE_COLS / 2, ty0 = t.ty * TILE_COLS - TILE_COLS / 2;
    minCx = Math.min(minCx, tx0); maxCx = Math.max(maxCx, tx0 + TILE_COLS);
    minCy = Math.min(minCy, ty0); maxCy = Math.max(maxCy, ty0 + TILE_COLS);
  }
  const cellsX = maxCx - minCx, cellsY = maxCy - minCy;
  const width = cellsY * scale, height = cellsX * scale, pixels = Buffer.alloc(width * height * 3);
  const put = (px: number, py: number, rgb: readonly number[]) => {
    if (px < 0 || py < 0 || px >= width || py >= height) return;
    pixels.set(rgb, (py * width + px) * 3);
  };
  const putCell = (cx: number, cy: number, col: number) => {
    const rgb = COLORS[col] ?? COLORS[COLUMN.UNKNOWN];
    const left = (maxCy - 1 - cy) * scale, top = (maxCx - 1 - cx) * scale;
    for (let a = 0; a < scale; a++) for (let b = 0; b < scale; b++) put(left + b, top + a, rgb);
  };
  for (let cx = minCx; cx < maxCx; cx++) for (let cy = minCy; cy < maxCy; cy++) putCell(cx, cy, COLUMN.UNKNOWN);
  for (const t of view.tiles) {
    const tx0 = t.tx * TILE_COLS - TILE_COLS / 2, ty0 = t.ty * TILE_COLS - TILE_COLS / 2;
    for (let ix = 0; ix < TILE_COLS; ix++)
      for (let iy = 0; iy < TILE_COLS; iy++) {
        const c = t.columns[ix * TILE_COLS + iy];
        if (c) putCell(tx0 + ix, ty0 + iy, c);
      }
  }
  for (let ix = 0; ix < w.cellsX; ix++)
    for (let iy = 0; iy < w.cellsY; iy++)
      putCell(w.originX + ix, w.originY + iy, w.columns[ix * w.cellsY + iy]);
  const toPx = (x: number, y: number) => [(maxCy - y / CELL_M) * scale, (maxCx - x / CELL_M) * scale];
  const line = (x0: number, y0: number, x1: number, y1: number, rgb: readonly number[]) => {
    const [ax, ay] = toPx(x0, y0), [bx, by] = toPx(x1, y1), n = Math.ceil(Math.hypot(bx - ax, by - ay)) + 1;
    for (let i = 0; i <= n; i++) put(Math.round(ax + (bx - ax) * i / n), Math.round(ay + (by - ay) * i / n), rgb);
  };
  const path = view.path;
  for (let i = 1; i < path.length; i++) line(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, [0x7a, 0xa7, 0xd9]);
  const r = view.robot;
  if (r) {
    const [px, py] = toPx(r.x, r.y);
    for (let a = -scale; a <= scale; a++) for (let b = -scale; b <= scale; b++) put(Math.round(px) + a, Math.round(py) + b, [0x5f, 0xe0, 0x8a]);
    line(r.x, r.y, r.x + 0.3 * Math.cos(r.yaw), r.y + 0.3 * Math.sin(r.yaw), [0xff, 0xff, 0xff]);
  }
  return rgbPng(width, height, pixels);
}
