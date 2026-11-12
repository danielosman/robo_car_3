// The outline maps' one object on top (doc/OUTLINE_MAP_PLAN.md, "Step B and C"): per
// band an outline map (outline_map.ts) and its walls (walls.ts), and the pose with how
// sure it is (pose.ts). It takes a frame's odometry pose and sights and feeds the maps
// and the walls. The simulator (sim/play.ts) and the store's reader (index.ts) call it.
//   mapper.add(frame, sights)   one frame: its number and odometry pose, its sights (placed by that pose)
//   mapper.outlines()           each band's outline, its wall edges on the known walls
//   mapper.walls()              each band's walls
//   mapper.sightings()          every sighting of a wall, by band (wall numbers as they are now)
//   mapper.poses()              every frame's pose and how sure it was
// Step B: the pose is the odometry's; it only gets less sure. Step C will check each
// frame against the walls and correct it.
import { OutlineMap, type Loop } from "./outline_map.ts";
import { BANDS, inBand, type BandName, type Sight } from "./sights.ts";
import { Walls, type Sighting, type Wall } from "./walls.ts";
import { grown, NO_DOUBT, type Pose, type PoseDoubt } from "./pose.ts";

// One band's outline; the bands come in sights.ts's order (BANDS): L1, L2, above.
export interface BandOutline { band: BandName; loops: Loop[] }
export interface BandWalls { band: BandName; walls: readonly Wall[] }
export interface BandSighting extends Sighting { band: BandName }
// A frame's pose: as odometry gave it, as the map placed it (the same until step C), how sure.
export interface FramePose { frame: number; odometry: Pose; pose: Pose; doubt: PoseDoubt }

// New walls are looked for in the outline every this many frames: outline() is too slow for every frame.
const FIND_EVERY = 10;

export class Mapper {
  private readonly bands = BANDS.map(band => ({ band, map: new OutlineMap(band), walls: new Walls() }));
  private doubt = NO_DOUBT;
  private readonly placed: FramePose[] = [];
  private readonly seen: BandSighting[] = [];

  add(frame: { number: number } & Pose, sights: Sight[]): void {
    const pose = { x: frame.x, y: frame.y, yaw: frame.yaw }, last = this.placed.at(-1);
    if (last) this.doubt = grown(this.doubt, last.odometry, pose);
    this.placed.push({ frame: frame.number, odometry: pose, pose, doubt: this.doubt });
    for (const { band, map, walls } of this.bands) {
      for (const s of walls.see(frame.number, pose, this.doubt, inBand(sights, band))) this.seen.push({ ...s, band: band.name });
      map.add(sights);
    }
    if (this.placed.length % FIND_EVERY === 0)
      for (const { map, walls } of this.bands) walls.found(map.outline(walls).flatMap(l => l.edges), frame.number, pose, this.doubt);
  }

  outlines(): BandOutline[] { return this.bands.map(({ band, map, walls }) => ({ band: band.name, loops: map.outline(walls) })); }
  walls(): BandWalls[] { return this.bands.map(({ band, walls }) => ({ band: band.name, walls: walls.all() })); }
  poses(): readonly FramePose[] { return this.placed; }

  sightings(): BandSighting[] {
    const walls = new Map(this.bands.map(b => [b.band.name, b.walls]));
    return this.seen.map(s => ({ ...s, wall: walls.get(s.band)!.numberNow(s.wall) }));
  }
}
