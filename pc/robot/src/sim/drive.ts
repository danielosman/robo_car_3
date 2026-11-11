// The simulator's robot (doc/OUTLINE_MAP_PLAN.md, "The simulator"): a list of moves like
// the real ones (`r30 d50`), each with a still time before and after, a ToF frame every
// 100 ms. It keeps two poses: the true one (where it is in the room) and the odometry
// one (what the frames carry, and all the map code sees). They part by the drift.
//   movesOf("r30 d50")   the moves
//   drive(moves, drift, random, start)   every frame's two poses, move by move
export interface Pose { x: number; y: number; yaw: number }
export type Move = { turnRad: number } | { driveM: number };

// How the true pose parts from the odometry's.
export interface Drift {
  turnScale: number;   // a turn comes out this much larger than odometry says (1.03: 30° is truly 31°)
  driveScale: number;  // the same for a distance driven
  yawPerM: number;     // the heading wanders: a random step with this spread (rad) per metre driven ...
  yawPerRad: number;   // ... and this spread per radian turned
  slipM: number;       // turning in place slides the body: a random step with this spread (m) per radian turned
}
export const NO_DRIFT: Drift = { turnScale: 1, driveScale: 1, yawPerM: 0, yawPerRad: 0, slipM: 0 };
// As measured on the robot (README, "Measured on the robot"; the stored drives: a 30°
// turn is about 31°; the body slides about 1.5 cm in a full turn).
export const MEASURED_DRIFT: Drift = { turnScale: 1.03, driveScale: 1.0015, yawPerM: 0.01, yawPerRad: 0.005, slipM: 0.0025 };

const DEG = Math.PI / 180;
export const FRAME_S = 0.1, STILL_S = 0.2;        // a frame every 100 ms; still 200 ms before and after a move
const SPEED_MPS = 0.1, TURN_RADPS = 29 * DEG;     // the robot's speeds

// "r30 d50 r-90": rotate 30° left, drive 50 cm, rotate 90° right.
export function movesOf(text: string): Move[] {
  return text.trim().split(/\s+/).filter(Boolean).map(word => {
    const m = /^([rd])(-?\d+(?:\.\d+)?)$/.exec(word);
    if (!m) throw new Error(`${word} is not a move (r<degrees> or d<cm>)`);
    return m[1] === "r" ? { turnRad: Number(m[2]) * DEG } : { driveM: Number(m[2]) / 100 };
  });
}

// A random number source with a seed, so a scenario runs the same every time: uniform()
// in 0..1, normal() with spread 1.
export interface Random { uniform(): number; normal(): number }
export function randomOf(seed: number): Random {
  let a = seed >>> 0;
  const uniform = () => { // mulberry32
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return { uniform, normal: () => Math.sqrt(-2 * Math.log(1 - uniform())) * Math.cos(2 * Math.PI * uniform()) };
}

// One frame's moment: where the robot truly is, where odometry says it is, whether it stands still.
export interface Moment { truth: Pose; odometry: Pose; still: boolean }

// The frames' moments, one list per move (a step). Both poses start at `start`.
export function drive(moves: Move[], drift: Drift, random: Random, start: Pose = { x: 0, y: 0, yaw: 0 }): Moment[][] {
  const truth = { ...start }, odometry = { ...start };
  const moment = (still: boolean): Moment => ({ truth: { ...truth }, odometry: { ...odometry }, still });
  const stillFrames = Math.round(STILL_S / FRAME_S);
  return moves.map(move => {
    const moments: Moment[] = [];
    for (let i = 0; i < stillFrames; i++) moments.push(moment(true));
    const total = "turnRad" in move ? move.turnRad : move.driveM, speed = "turnRad" in move ? TURN_RADPS : SPEED_MPS;
    const frames = Math.max(1, Math.ceil(Math.abs(total) / (speed * FRAME_S))), part = total / frames;
    for (let i = 0; i < frames; i++) {
      if ("turnRad" in move) {
        odometry.yaw += part;
        truth.yaw += part * drift.turnScale + random.normal() * drift.yawPerRad * Math.sqrt(Math.abs(part));
        truth.x += random.normal() * drift.slipM * Math.sqrt(Math.abs(part));
        truth.y += random.normal() * drift.slipM * Math.sqrt(Math.abs(part));
      } else {
        odometry.x += part * Math.cos(odometry.yaw);
        odometry.y += part * Math.sin(odometry.yaw);
        truth.x += part * drift.driveScale * Math.cos(truth.yaw);
        truth.y += part * drift.driveScale * Math.sin(truth.yaw);
        truth.yaw += random.normal() * drift.yawPerM * Math.sqrt(Math.abs(part));
      }
      moments.push(moment(false));
    }
    moments.pop(); // the move's last frame is taken standing: the first of the still time after
    for (let i = 0; i <= stillFrames; i++) moments.push(moment(true));
    return moments;
  });
}
