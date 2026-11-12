// A scenario played through the map under test (doc/OUTLINE_MAP_PLAN.md, "The
// simulator"): the one place that says which map code the simulator runs. main.ts
// draws what this gives; the tests measure it.
//   play(scenario, { correct })   per step: the simulator's step, what the map drew for each band,
//                                 its walls, and the pose it placed the step's last frame at
import { Mapper } from "../maps/outline/mapper.ts";
import type { Loop } from "../maps/outline/outline_map.ts";
import type { Pose } from "../maps/outline/pose.ts";
import type { Wall } from "../maps/outline/walls.ts";
import type { BandName } from "../maps/outline/sights.ts";
import { run, type Scenario, type SimStep } from "./scenarios.ts";

export interface Played { step: SimStep; bands: Record<BandName, Loop[]>; walls: Record<BandName, Wall[]>; corrected: Pose }

export function play(s: Scenario, options: { correct?: boolean } = {}): Played[] {
  const mapper = new Mapper(options);
  return run(s).map(step => {
    for (const f of step.frames) mapper.add(f.frame, f.sights);
    return {
      step,
      bands: Object.fromEntries(mapper.outlines().map(o => [o.band, o.loops])) as Record<BandName, Loop[]>,
      corrected: mapper.poses().at(-1)!.pose,
      walls: Object.fromEntries(mapper.walls().map(w => [w.band, w.walls.map(wall => ({ ...wall }))])) as Record<BandName, Wall[]>,
    };
  });
}
