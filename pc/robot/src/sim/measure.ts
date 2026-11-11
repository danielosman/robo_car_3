// The simulator's numbers (doc/OUTLINE_MAP_PLAN.md, "What is measured"): what the map
// under test drew against the truth the simulator knows.
//   poseError(pose, truth)   how far off a pose is, and by what angle
//   offFaces(faces, x, y)    how far a point is from the nearest true face
import type { Face } from "./room.ts";
import type { Pose } from "./drive.ts";

export function poseError(pose: Pose, truth: Pose): { m: number; rad: number } {
  const turn = (pose.yaw - truth.yaw) % (2 * Math.PI);
  return { m: Math.hypot(pose.x - truth.x, pose.y - truth.y), rad: Math.abs(turn > Math.PI ? turn - 2 * Math.PI : turn < -Math.PI ? turn + 2 * Math.PI : turn) };
}

export function offFaces(faces: Face[], x: number, y: number): number {
  let best = Infinity;
  for (const f of faces) {
    const ex = f.bx - f.ax, ey = f.by - f.ay;
    const t = Math.max(0, Math.min(1, ((x - f.ax) * ex + (y - f.ay) * ey) / (ex * ex + ey * ey || 1)));
    best = Math.min(best, Math.hypot(x - f.ax - t * ex, y - f.ay - t * ey));
  }
  return best;
}
