// The robot's map at one snapshot (snapshots_api.ts, MapView) for the API: each world
// column (10 cm) as the robot has it (picoA/app/cell_map.c, column_at: the window's
// inside it, the whole-house map's tiles outside), the robot's own obstacle check
// along a path (cell_map_obstacle_along), and the map as a PNG (drawn here, on the PC). Map frame: x forward at power-up, y left; drawn with x up and
// y to the left, as the page does.
import type { MapView } from "../snapshots_api.ts";
import { COLUMN, TILE_COLS, rgbPng } from "../snapshot.ts";

const CELL_M = 0.1;
const cellOf = (m: number) => Math.floor(m / CELL_M);
const tileOf = (c: number) => Math.floor((c + TILE_COLS / 2) / TILE_COLS); // picoA/app/house_map.c

// The page's colours (public/app.html).
const COLORS: Record<number, [number, number, number]> = {
  [COLUMN.UNKNOWN]: [0x15, 0x15, 0x15], [COLUMN.BLOCKED]: [0xd0, 0x45, 0x3b], [COLUMN.OVERHANG]: [0xe3, 0x9b, 0x3a],
  [COLUMN.NO_FLOOR]: [0x9b, 0x59, 0xd0], [COLUMN.DRIVABLE]: [0x3d, 0x7f, 0x4f], [COLUMN.OPEN]: [0x2f, 0x4a, 0x3a],
};

export class RobotMap {
  readonly view: MapView;
  private tiles = new Map<string, number[]>();

  constructor(view: MapView) {
    this.view = view;
    for (const t of view.tiles) this.tiles.set(`${t.tx},${t.ty}`, t.columns);
  }

  // The column at world cell (cx, cy): COLUMN.*.
  column(cx: number, cy: number): number {
    const w = this.view.window;
    if (w && cx >= w.originX && cx < w.originX + w.cellsX && cy >= w.originY && cy < w.originY + w.cellsY)
      return w.columns[(cx - w.originX) * w.cellsY + (cy - w.originY)];
    const tx = tileOf(cx), ty = tileOf(cy), t = this.tiles.get(`${tx},${ty}`);
    return t ? t[(cx - (tx * TILE_COLS - TILE_COLS / 2)) * TILE_COLS + (cy - (ty * TILE_COLS - TILE_COLS / 2))] : COLUMN.UNKNOWN;
  }

  // From (x, y) along the unit direction (dx, dy): how far to the nearest blocked or
  // overhang column in a strip halfWidth to each side, at most max (m). As the robot's
  // cell_map_obstacle_along().
  obstacleAlong(x: number, y: number, dx: number, dy: number, halfWidth: number, max: number): number {
    let nearest = max;
    const ex = x + max * dx, ey = y + max * dy, wx = Math.abs(dy) * halfWidth, wy = Math.abs(dx) * halfWidth;
    for (let ix = cellOf(Math.min(x, ex) - wx); ix <= cellOf(Math.max(x, ex) + wx); ix++)
      for (let iy = cellOf(Math.min(y, ey) - wy); iy <= cellOf(Math.max(y, ey) + wy); iy++) {
        const c = this.column(ix, iy);
        if (c !== COLUMN.BLOCKED && c !== COLUMN.OVERHANG) continue;
        let alongLo = Infinity, alongHi = -Infinity, sideLo = Infinity, sideHi = -Infinity;
        for (let k = 0; k < 4; k++) { // the cell's corners in the path's frame
          const cx = (ix + (k & 1)) * CELL_M - x, cy = (iy + (k >> 1)) * CELL_M - y;
          const along = cx * dx + cy * dy, side = -cx * dy + cy * dx;
          alongLo = Math.min(alongLo, along); alongHi = Math.max(alongHi, along);
          sideLo = Math.min(sideLo, side); sideHi = Math.max(sideHi, side);
        }
        if (sideHi <= -halfWidth || sideLo >= halfWidth || alongHi <= 0) continue;
        nearest = Math.min(nearest, Math.max(alongLo, 0));
      }
    return nearest;
  }

  // A blocked or overhang column within r of (x, y).
  obstacleWithin(x: number, y: number, r: number): boolean {
    for (let ix = cellOf(x - r); ix <= cellOf(x + r); ix++)
      for (let iy = cellOf(y - r); iy <= cellOf(y + r); iy++) {
        const c = this.column(ix, iy);
        if (c !== COLUMN.BLOCKED && c !== COLUMN.OVERHANG) continue;
        const nx = Math.max(ix * CELL_M, Math.min(x, (ix + 1) * CELL_M)), ny = Math.max(iy * CELL_M, Math.min(y, (iy + 1) * CELL_M));
        if (Math.hypot(nx - x, ny - y) < r) return true;
      }
    return false;
  }

  // The window as an RGB PNG, scale pixels a column; null without a window.
  png(scale = 6): Buffer | null {
    const w = this.view.window;
    if (!w) return null;
    const width = w.cellsY * scale, height = w.cellsX * scale, pixels = Buffer.alloc(width * height * 3);
    const put = (px: number, py: number, rgb: readonly number[]) => {
      if (px < 0 || py < 0 || px >= width || py >= height) return;
      pixels.set(rgb, (py * width + px) * 3);
    };
    const toPx = (x: number, y: number) => [(w.originY + w.cellsY - y / CELL_M) * scale, (w.originX + w.cellsX - x / CELL_M) * scale];
    for (let ix = 0; ix < w.cellsX; ix++)
      for (let iy = 0; iy < w.cellsY; iy++) {
        const rgb = COLORS[w.columns[ix * w.cellsY + iy]] ?? COLORS[COLUMN.UNKNOWN];
        const left = (w.cellsY - 1 - iy) * scale, top = (w.cellsX - 1 - ix) * scale;
        for (let a = 0; a < scale; a++) for (let b = 0; b < scale; b++) put(left + b, top + a, rgb);
      }
    const line = (x0: number, y0: number, x1: number, y1: number, rgb: readonly number[]) => {
      const [ax, ay] = toPx(x0, y0), [bx, by] = toPx(x1, y1), n = Math.ceil(Math.hypot(bx - ax, by - ay)) + 1;
      for (let i = 0; i <= n; i++) put(Math.round(ax + (bx - ax) * i / n), Math.round(ay + (by - ay) * i / n), rgb);
    };
    const path = this.view.path;
    for (let i = 1; i < path.length; i++) line(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y, [0x7a, 0xa7, 0xd9]);
    const r = this.view.robot;
    if (r) {
      const [px, py] = toPx(r.x, r.y);
      for (let a = -scale; a <= scale; a++) for (let b = -scale; b <= scale; b++) put(Math.round(px) + a, Math.round(py) + b, [0x5f, 0xe0, 0x8a]);
      line(r.x, r.y, r.x + 0.3 * Math.cos(r.yaw), r.y + 0.3 * Math.sin(r.yaw), [0xff, 0xff, 0xff]);
    }
    return rgbPng(width, height, pixels);
  }
}
