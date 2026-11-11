// The simulator's scenarios (doc/OUTLINE_MAP_PLAN.md, "Scenarios"): a made-up room, a
// drive through it, the drift and the noise. `npm run sim -- <name>` draws one; the
// tests run them with numbers checked. The robot starts at the origin, facing x.
//   SCENARIOS        all of them, by name
//   run(scenario)    the drive: each move's frames (as the real reader gets them) and the truth
import type { PosedFrame } from "../tof_format.ts";
import type { ZoneRays } from "../maps/cell_map.ts";
import { sightsOf, type Sight } from "../maps/line_map/sights.ts";
import { box, facesAt, round, wall, type Face, type Room } from "./room.ts";
import { drive, MEASURED_DRIFT, movesOf, randomOf, type Drift, type Pose } from "./drive.ts";
import { ALL_NOISE, tofFrame, ZONE_RAD, type Noise } from "./tof.ts";

export interface Scenario { name: string; about: string; room: Room; moves: string; drift: Drift; noise: Noise; seed: number }

const scenario = (name: string, about: string, room: Room, moves: string): Scenario =>
  ({ name, about, room, moves, drift: MEASURED_DRIFT, noise: ALL_NOISE, seed: 1 });

// A room with its walls along x and y, from (x0, y0) to (x1, y1).
const walls = (x0: number, y0: number, x1: number, y1: number) => wall([x0, y0, x1, y0, x1, y1, x0, y1, x0, y0]);
const LOOK_ROUND = "r30 ".repeat(12);
const SQUARE = walls(-1.5, -1.5, 1.5, 1.5);
const BOX = { lowM: 0, highM: 0.1 }; // a box 10 cm high: in L1 only
// A table 30 cm high: its top is in "above" only, its four legs in every band.
const TABLE: Room = [
  box(-1.1, -0.4, -0.5, 0.4, { lowM: 0.3, highM: 0.33 }),
  ...[[-1.05, -0.35], [-1.05, 0.35], [-0.55, -0.35], [-0.55, 0.35]].map(([x, y]) => round(x, y, 0.02, 8, { highM: 0.3 })),
];

export const SCENARIOS: Scenario[] = [
  scenario("square", "An empty 3 x 3 m room; the robot turns 360° in 30° steps in the middle.", [SQUARE], LOOK_ROUND),
  scenario("things", "The same room with a 10 cm box and a table: islands, bands.",
           [SQUARE, box(0.7, 0.3, 1.0, 0.6, BOX), ...TABLE], LOOK_ROUND),
  scenario("door", "Two rooms and a door: the robot looks round, drives through, looks round again.",
           [walls(-1.5, -1.5, 4.5, 1.5), wall([1.5, -1.5, 1.5, -0.6]), wall([1.5, 0.6, 1.5, 1.5])],
           `${LOOK_ROUND} d100 d100 d100 ${LOOK_ROUND}`),
  scenario("loop", "A loop: the robot drives a 2 m square round a pouf and comes back to its start.",
           [walls(-1, -1, 3, 3), round(1, 1, 0.3, 12, { highM: 0.4 })], "d100 d100 r30 r30 r30 ".repeat(4)),
  scenario("box-removed", "A box is taken away half way through two looks round.",
           [SQUARE, box(0.7, 0.3, 1.0, 0.6, { ...BOX, untilStep: 12 })], LOOK_ROUND + LOOK_ROUND),
  scenario("box-stays", "The same drive, the box stays: only the drift moves things.",
           [SQUARE, box(0.7, 0.3, 1.0, 0.6, BOX)], LOOK_ROUND + LOOK_ROUND),
  scenario("big", "A big room (walls 4 m away, beyond the sensor's reach): guess edges.", [walls(-4, -4, 4, 4)], LOOK_ROUND),
];

// One frame of the drive: the frame and its rays as the real reader gets them from the
// store and the helper, its sights, and where the robot truly was.
export interface SimFrame { frame: PosedFrame; rays: ZoneRays[]; sights: Sight[]; truth: Pose }
// One move: its frames, the room's faces during it, and the two poses at its end.
export interface SimStep { frames: SimFrame[]; faces: Face[]; truth: Pose; odometry: Pose }

export function run(s: Scenario): SimStep[] {
  // Two sources, so the same seed gives the same drive whatever the noise.
  const random = randomOf(s.seed + 1);
  let number = 0;
  return drive(movesOf(s.moves), s.drift, randomOf(s.seed)).map((moments, step) => {
    const faces = facesAt(s.room, step);
    const frames = moments.map(moment => {
      const { frame, rays } = tofFrame(faces, moment, ++number, s.noise, random);
      return { frame, rays, sights: sightsOf(frame, rays, ZONE_RAD).filter(sight => sight !== null), truth: moment.truth };
    });
    const last = moments[moments.length - 1];
    return { frames, faces, truth: last.truth, odometry: last.odometry };
  });
}
