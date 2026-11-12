// The outline maps' one object on top (doc/OUTLINE_MAP_PLAN.md, "How it fits the code"):
// per band an outline map (outline_map.ts) and its walls (walls.ts), and the pose with how
// sure it is (pose.ts). It takes a frame's odometry pose and sights, checks the pose
// against the walls (pose_check.ts), places the sights by the corrected pose and feeds the
// maps and the walls (which then see the frame as in step B, from the corrected pose). The simulator (sim/play.ts) and the store's reader (index.ts) call it.
//   new Mapper({ correct })     correct: check each frame against the walls (default); false: odometry as it is
//   mapper.add(frame, sights)   one frame: its number, odometry pose and whether it stood still; its sights (placed by that pose)
//   mapper.outlines()           each band's outline, its wall edges on the known walls
//   mapper.walls()              each band's walls
//   mapper.sightings()          every sighting of a wall, by band (wall numbers as they are now)
//   mapper.poses()              every frame's pose: odometry's, corrected, how sure
// The pose is checked against the walls of one band, "above": a wall stands in every band,
// and counting it three times would make the pose far too sure. "above" finds the most
// walls (the simulator's numbers in the plan); things on the floor are mostly below it.
// A correction holds from then on: earlier frames stay where they were put.
import { OutlineMap, type Loop } from "./outline_map.ts";
import { BANDS, inBand, type BandName, type Sight } from "./sights.ts";
import { Walls, type Sighting, type Wall } from "./walls.ts";
import { grown, NO_DOUBT, type Pose, type PoseDoubt } from "./pose.ts";
import { PoseCheck } from "./pose_check.ts";

// One band's outline; the bands come in sights.ts's order (BANDS): L1, L2, above.
export interface BandOutline { band: BandName; loops: Loop[] }
export interface BandWalls { band: BandName; walls: readonly Wall[] }
export interface BandSighting extends Sighting { band: BandName }
// A frame's pose: as odometry gave it, as the map placed it (corrected), how sure after the check.
export interface FramePose { frame: number; odometry: Pose; pose: Pose; doubt: PoseDoubt }

// New walls are looked for in the outline every this many frames: outline() is too slow for every frame.
const FIND_EVERY = 10;
const POSE_BAND: BandName = "above";

export class Mapper {
  private readonly check: PoseCheck | null;
  private readonly bands: { band: (typeof BANDS)[number]; map: OutlineMap; walls: Walls }[];
  private doubt = NO_DOUBT;  // without the pose check
  // The corrected pose is the odometry's turned by `yaw` about the origin, then moved by (x, y).
  private shift: Pose = { x: 0, y: 0, yaw: 0 };
  private readonly placed: FramePose[] = [];
  private readonly seen: BandSighting[] = [];

  constructor(options: { correct?: boolean } = {}) {
    this.check = options.correct ?? true ? new PoseCheck() : null;
    this.bands = BANDS.map(band => ({ band, map: new OutlineMap(band), walls: new Walls(band, band.name === POSE_BAND ? this.check : null) }));
  }

  private get poseDoubt(): PoseDoubt { return this.check ? this.check.doubt() : this.doubt; }

  add(frame: { number: number; still?: boolean } & Pose, sights: Sight[]): void {
    const odometry = { x: frame.x, y: frame.y, yaw: frame.yaw }, last = this.placed.at(-1);
    let pose = this.place(odometry), own = sights.map(s => this.placeSight(s));
    if (last && this.check) this.check.moved(last.pose, pose);
    else if (last) this.doubt = grown(this.doubt, last.odometry, odometry);
    const poseBand = this.bands.find(b => b.band.name === POSE_BAND)!;
    if (this.check) {
      // Check the pose against the pose band's walls; the walls take their share.
      const before = this.check.doubt(), ms = poseBand.walls.measure(pose, before, inBand(own, poseBand.band));
      if (ms.length) {
        const check = this.check.check(ms), [dx, dy, dYaw] = check.pose;
        // The hits and ends seen move with the pose: turned about it, then shifted.
        const c = Math.cos(dYaw), sn = Math.sin(dYaw);
        const moved = (x: number, y: number): [number, number] =>
          [pose.x + dx + c * (x - pose.x) - sn * (y - pose.y), pose.y + dy + sn * (x - pose.x) + c * (y - pose.y)];
        const used = check.used.map(m => ({ ...m, hits: m.hits.map(([x, y]) => moved(x, y)),
                                            ends: m.ends.map(e => { const [x, y] = moved(e.x, e.y); return { ...e, x, y }; }) }));
        const placed = { x: pose.x + dx, y: pose.y + dy, yaw: pose.yaw + dYaw };
        for (const s of poseBand.walls.apply(frame.number, placed, before, used, check.walls)) this.seen.push({ ...s, band: POSE_BAND });
        pose = placed;
        this.shift.yaw += dYaw;
        const [x, y] = this.turned(odometry);
        this.shift.x = pose.x - x; this.shift.y = pose.y - y;
        own = sights.map(s => this.placeSight(s));
      }
    }
    this.placed.push({ frame: frame.number, odometry, pose, doubt: this.poseDoubt });
    // The other bands' walls see the frame as placed, the pose's doubt as noise.
    for (const b of this.bands) {
      if (!this.check || b !== poseBand)
        for (const s of b.walls.see(frame.number, pose, this.poseDoubt, inBand(own, b.band))) this.seen.push({ ...s, band: b.band.name });
      b.map.add(own);
    }
    // New walls: every few frames; with the pose check also whenever the robot stands still
    // (its hits are sharp then, and walls to check against are wanted early).
    const look = this.placed.length % FIND_EVERY === 0 || (this.check !== null && (frame.still ?? true));
    if (look)
      for (const { band, map, walls } of this.bands)
        walls.found(map.outline(walls).flatMap(l => l.edges), frame.number, pose, this.poseDoubt, inBand(own, band));
  }

  private turned(p: { x: number; y: number }): [number, number] {
    const c = Math.cos(this.shift.yaw), s = Math.sin(this.shift.yaw);
    return [c * p.x - s * p.y, s * p.x + c * p.y];
  }
  private place(p: Pose): Pose { const [x, y] = this.turned(p); return { x: x + this.shift.x, y: y + this.shift.y, yaw: p.yaw + this.shift.yaw }; }
  private placeSight(s: Sight): Sight {
    const [x, y] = this.turned(s), at = { ...s, x: x + this.shift.x, y: y + this.shift.y, bearing: s.bearing + this.shift.yaw };
    if (s.hit) { const [hx, hy] = this.turned(s.hit); at.hit = { ...s.hit, x: hx + this.shift.x, y: hy + this.shift.y }; }
    return at;
  }

  outlines(): BandOutline[] { return this.bands.map(({ band, map, walls }) => ({ band: band.name, loops: map.outline(walls) })); }
  walls(): BandWalls[] { return this.bands.map(({ band, walls }) => ({ band: band.name, walls: walls.all() })); }
  poses(): readonly FramePose[] { return this.placed; }

  sightings(): BandSighting[] {
    const walls = new Map(this.bands.map(b => [b.band.name, b.walls]));
    return this.seen.map(s => ({ ...s, wall: walls.get(s.band)!.numberNow(s.wall) }));
  }
}
