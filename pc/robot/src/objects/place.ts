// Places a detection's box on the floor (doc/OBJECTS_PLAN.md, "Placing one box"): the
// box's bottom edge is where the object meets the floor, so the rays through its two
// bottom corners, from the camera down to the floor, give the object's front edge in
// the map. Camera only: no ToF yet. Behind the front edge the object is as deep as its
// label's default (labels.ts); a door or wall is the front edge alone.
import { DEPTH_M, type Detection, type Label } from "./labels.ts";
import type { Mounting } from "../mounting.ts";

// The camera in the robot frame (x forward, y left, z up, m), from a mounting: the
// boot's own copy (boot.ts, store.ts mountingOf), so a snapshot is placed with the
// numbers of when it was taken (common/mounting.json's numbers and how they were
// found: doc/CAMERA_PARAMS.md). tiltRad: + = up, on top of the robot's pitch.
export interface CameraGeometry { x: number; y: number; z: number; width: number; height: number; focalPx: number; tiltRad: number }

export function cameraGeometry(m: Mounting): CameraGeometry {
  const c = m.camera;
  return { x: c.x_m, y: c.y_m, z: c.z_m, width: c.width_px, height: c.height_px, focalPx: c.f_px, tiltRad: c.tilt_deg * Math.PI / 180 };
}

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

export interface SnapshotPoseLike { x: number; y: number; yaw: number; pitch: number }

// Where the ray through picture point (px, py) (pixels, continuous: 0 the top left
// corner, 160 the right edge) meets the floor, in the robot frame; null if it doesn't
// look down.
export function floorPoint(px: number, py: number, pitch: number, cam: CameraGeometry): [number, number] | null {
  const f = cam.focalPx, left = cam.width / 2 - px, up = cam.height / 2 - py;
  const t = pitch + cam.tiltRad; // nose up turns the ray up
  const fwd = f * Math.cos(t) - up * Math.sin(t), upW = f * Math.sin(t) + up * Math.cos(t);
  if (upW >= -1e-6) return null;
  const k = cam.z / -upW;
  return [cam.x + k * fwd, cam.y + k * left];
}

export function place(d: Pick<Detection, "label" | "bbox">, pose: SnapshotPoseLike, cam: CameraGeometry): Placed {
  // The box is y first on 0-1000 (labels.ts): [ymin, xmin, ymax, xmax]. ymin isn't used.
  const [, x1, y2, x2] = d.bbox;
  const none = (floor: Floor, distanceM: number | null = null): Placed =>
    ({ label: d.label, floor, kind: null, distanceM, cx: null, cy: null, width: null, depth: null, angle: null });
  // 0-1000 to the picture's pixels, x across and y down; from here on everything is x first.
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
