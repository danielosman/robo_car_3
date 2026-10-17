// Places a detection's box on the floor (doc/OBJECTS_PLAN.md, "Placing one box"): the
// box's bottom edge is where the object meets the floor, so the rays through its two
// bottom corners, from the camera down to the floor, give the object's front edge in
// the map. Camera only: no ToF yet. Behind the front edge the object is as deep as its
// label's default (labels.ts); a door or wall is the front edge alone.
import { DEPTH_M, type Detection, type Label } from "./labels.ts";

// The camera in the robot frame: x forward, y left, z up, m (measured by Daniel, 17 Oct
// 2026: 8.8 cm up, 2.5 cm ahead of the centre, 0.5 cm right).
// focalPx 188 (~46° across): the table in boot 7a587226 snapshot 1, 80.5 cm across its
// front legs (measured by Daniel), 83 px wide at the ToF's ~182 cm. The pouf
// (snapshot 31) gives 182 px. The firmware's FOCAL_PX 160 (the data sheet's 64°
// diagonal) is wrong (doc/OBJECTS_PLAN.md, "Settled by the table").
// tiltRad -1.86°: the camera looks this much further down than the robot's pitch says
// (+ = up). From two objects at known distances, their floor contact read from the
// pixels, with f 188: the pouf (snapshot 31, ToF ~124 cm) -1.84°, the table (snapshot
// 1, ~185 cm from the camera) -1.88° (doc/OBJECTS_PLAN.md, the calibration).
export const CAMERA_GEOMETRY = {
  x: 0.025, y: -0.005, z: 0.088,
  width: 160, height: 120,
  focalPx: 188,
  tiltRad: -1.86 * Math.PI / 180,
};
export type CameraGeometry = typeof CAMERA_GEOMETRY;

const MAX_M = 5;          // further than this along the floor: too far to place
const CUT_OFF = 995;      // a bottom edge this low (0-1000) touches the picture's bottom

// floor: the bottom edge meets the floor where it says; cut_off: the box reaches the
// bottom of the picture, so the object is at most this far; above_horizon: the bottom
// edge doesn't look down at the floor; too_far: further than MAX_M.
export type Floor = "floor" | "cut_off" | "above_horizon" | "too_far";

export interface Placed {
  label: Label;
  floor: Floor;
  kind: "rect" | "line" | null;  // null: not placed
  distanceM: number | null;      // from the camera to the front edge's middle, along the floor
  // The shape in the map frame (m, rad): its centre, width along the front edge, depth
  // away from the robot, and the front edge's direction (left end to right end, as the
  // camera sees it, + = left of x).
  cx: number | null; cy: number | null; width: number | null; depth: number | null; angle: number | null;
}

export interface SnapPoseLike { x: number; y: number; yaw: number; pitch: number }

// Where the ray through picture point (px, py) (pixels, continuous: 0 the top left
// corner, 160 the right edge) meets the floor, in the robot frame; null if it doesn't
// look down.
export function floorPoint(px: number, py: number, pitch: number, cam: CameraGeometry = CAMERA_GEOMETRY): [number, number] | null {
  const f = cam.focalPx, left = cam.width / 2 - px, up = cam.height / 2 - py;
  const t = pitch + cam.tiltRad; // nose up turns the ray up
  const fwd = f * Math.cos(t) - up * Math.sin(t), upW = f * Math.sin(t) + up * Math.cos(t);
  if (upW >= -1e-6) return null;
  const k = cam.z / -upW;
  return [cam.x + k * fwd, cam.y + k * left];
}

export function place(d: Pick<Detection, "label" | "bbox">, pose: SnapPoseLike, cam: CameraGeometry = CAMERA_GEOMETRY): Placed {
  const [, x1, y2, x2] = d.bbox;
  const none = (floor: Floor, distanceM: number | null = null): Placed =>
    ({ label: d.label, floor, kind: null, distanceM, cx: null, cy: null, width: null, depth: null, angle: null });
  const px = (v: number) => v / 1000 * cam.width, py = (v: number) => v / 1000 * cam.height;
  const bottom = py(Math.min(y2, 1000));
  const a = floorPoint(px(x1), bottom, pose.pitch, cam), b = floorPoint(px(x2), bottom, pose.pitch, cam);
  const mid = floorPoint(px((x1 + x2) / 2), bottom, pose.pitch, cam);
  if (!a || !b || !mid) return none("above_horizon");
  const distanceM = Math.hypot(mid[0] - cam.x, mid[1] - cam.y);
  if (distanceM > MAX_M) return none("too_far", distanceM);

  // Robot frame to map frame.
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const map = ([rx, ry]: [number, number]): [number, number] => [pose.x + c * rx - s * ry, pose.y + s * rx + c * ry];
  const [ax, ay] = map(a), [bx, by] = map(b), [mx, my] = map(mid);
  const width = Math.hypot(bx - ax, by - ay), angle = Math.atan2(by - ay, bx - ax);
  const depth = DEPTH_M[d.label];
  // The centre is half the depth behind the front edge's middle, away from the camera:
  // along the edge's normal (left of its direction: from the left end to the right end
  // the camera looks across it, so that normal points away).
  const [camX, camY] = map([cam.x, cam.y]), half = (depth ?? 0) / 2;
  const nx = -Math.sin(angle), ny = Math.cos(angle), away = nx * (mx - camX) + ny * (my - camY) >= 0 ? 1 : -1;
  return {
    label: d.label, floor: y2 >= CUT_OFF ? "cut_off" : "floor", kind: depth === null ? "line" : "rect", distanceM,
    cx: mx + away * nx * half, cy: my + away * ny * half, width, depth: depth ?? 0, angle,
  };
}

// The shape's corners in the map frame (m): a line's two ends, a rectangle's four.
export function corners(p: Pick<Placed, "kind" | "cx" | "cy" | "width" | "depth" | "angle">): [number, number][] {
  if (p.kind === null || p.cx === null || p.cy === null || p.width === null || p.angle === null) return [];
  const ex = Math.cos(p.angle) * p.width / 2, ey = Math.sin(p.angle) * p.width / 2;
  if (p.kind === "line") return [[p.cx - ex, p.cy - ey], [p.cx + ex, p.cy + ey]];
  const d = (p.depth ?? 0) / 2, nx = Math.sin(p.angle) * d, ny = -Math.cos(p.angle) * d;
  return [[p.cx - ex - nx, p.cy - ey - ny], [p.cx + ex - nx, p.cy + ey - ny], [p.cx + ex + nx, p.cy + ey + ny], [p.cx - ex + nx, p.cy - ey + ny]];
}
