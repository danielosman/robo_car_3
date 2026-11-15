// Corrections of the robot's pose (doc/OUTLINE_MAP_PLAN.md, step F). Pure.
// A correction turns a pose by `yaw` about the origin, then moves it by (x, y). Two are
// used:
//   c_pc     the PC's (maps/outline/mapper.ts correction()): raw odometry to the best pose
//   c_robot  the one the robot applies to every pose it sends (picoA/app/correction.h, the
//            twin of corrected() here); 0 at power-up, set by a correct command
// The robot says which one it applies from which message on (wire.ts, "corrected"); the
// store keeps those as a boot's corrections, and turns every pose it reads back to raw
// odometry with them (rawOf), so the maps take raw poses, as before step F; the page gets
// raw poses too and corrects them itself (public/app.js corrected(), a copy of the one here).
//   corrected(c, p)            p moved by c
//   uncorrected(c, p)          back: corrected(c, uncorrected(c, p)) is p
//   rawOf(corrections, m, p)   a pose the robot sent in message m, back to raw odometry
//   NO_CORRECTION
export interface Planar { x: number; y: number; yaw: number }
// From message `messageNumber` on (the robot's "corrected"), the robot applies `c`.
export interface CorrectionRow { messageNumber: number; c: Planar }

export const NO_CORRECTION: Planar = { x: 0, y: 0, yaw: 0 };

export function corrected<P extends Planar>(c: Planar, p: P): P {
  const cos = Math.cos(c.yaw), sin = Math.sin(c.yaw);
  return { ...p, x: cos * p.x - sin * p.y + c.x, y: sin * p.x + cos * p.y + c.y, yaw: p.yaw + c.yaw };
}

export function uncorrected<P extends Planar>(c: Planar, p: P): P {
  const cos = Math.cos(c.yaw), sin = Math.sin(c.yaw), dx = p.x - c.x, dy = p.y - c.y;
  return { ...p, x: cos * dx + sin * dy, y: -sin * dx + cos * dy, yaw: p.yaw - c.yaw };
}

// The correction the robot applied to message m: the last one it set before m (none: 0).
// corrections oldest first.
export function correctionAt(corrections: readonly CorrectionRow[], messageNumber: number): Planar {
  let c = NO_CORRECTION;
  for (const r of corrections) { if (r.messageNumber >= messageNumber) break; c = r.c; }
  return c;
}

export function rawOf<P extends Planar>(corrections: readonly CorrectionRow[], messageNumber: number, p: P): P {
  return corrections.length ? uncorrected(correctionAt(corrections, messageNumber), p) : p;
}
