// The robot's pose and how sure it is (doc/OUTLINE_MAP_PLAN.md, idea 3). Odometry drifts,
// so the pose gets less sure with every turn and every centimetre driven. The doubt is a
// spread that adds up (the drift is mostly a steady 3 % of each turn, not random), as one
// spread for the position and one for the heading. Step C makes it surer again at each
// check against the walls.
//   NO_DOUBT                      at the start: the map's frame is the start pose
//   grown(doubt, from, to)        the doubt after odometry moved from one pose to the next
//   lineDoubt(doubt, ...)         what the pose's doubt does to a line seen from it
export interface Pose { x: number; y: number; yaw: number }
// One spread each (m, rad).
export interface PoseDoubt { xyM: number; yawRad: number }

export const NO_DOUBT: PoseDoubt = { xyM: 0, yawRad: 0 };

// How fast the doubt grows: a 30° turn is about 31° (3 %, a little more to be safe); the
// heading wanders a little while driving; driving 1 m is within about 1 cm; the body
// slides about 1.5 cm in a full turn (README, "Measured on the robot").
const YAW_PER_RAD = 0.035, YAW_PER_M = 0.02, XY_PER_M = 0.01, XY_PER_RAD = 0.01;

export function grown(doubt: PoseDoubt, from: Pose, to: Pose): PoseDoubt {
  const driven = Math.hypot(to.x - from.x, to.y - from.y);
  const turned = Math.abs(Math.atan2(Math.sin(to.yaw - from.yaw), Math.cos(to.yaw - from.yaw)));
  const yawRad = doubt.yawRad + YAW_PER_RAD * turned + YAW_PER_M * driven;
  // Driving with an unsure heading puts the position off sideways.
  return { xyM: doubt.xyM + (XY_PER_M + doubt.yawRad) * driven + XY_PER_RAD * turned, yawRad };
}

// A line seen from a pose: a shift of the pose moves it across, a turn of the pose turns
// it and moves it across by the turn times how far along the line the point is from the
// robot. Here as the spreads of the line's across offset at `along` and its direction,
// with their covariance [across², across·direction, direction²]; `along` is measured
// along the line from the robot's foot point on it.
export function lineDoubt(doubt: PoseDoubt, along: number): [number, number, number] {
  const yaw2 = doubt.yawRad ** 2;
  return [doubt.xyM ** 2 + along * along * yaw2, along * yaw2, yaw2];
}
