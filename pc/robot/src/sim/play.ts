// A scenario played through the map under test (doc/OUTLINE_MAP_PLAN.md, "The
// simulator"): the one place that says which map code the simulator runs. main.ts
// draws what this gives; the tests measure it.
//   play(scenario)   per step: the simulator's step and what the map drew for each band
import { OutlineMap, type Loop } from "../maps/outline/outline_map.ts";
import { BANDS, type BandName } from "../maps/outline/sights.ts";
import { run, type Scenario, type SimStep } from "./scenarios.ts";

export interface Played { step: SimStep; bands: Record<BandName, Loop[]> }

export function play(s: Scenario): Played[] {
  const maps = BANDS.map(band => new OutlineMap(band));
  return run(s).map(step => {
    for (const f of step.frames) for (const map of maps) map.add(f.sights);
    return { step, bands: Object.fromEntries(BANDS.map((band, i) => [band.name, maps[i].outline()])) as Record<BandName, Loop[]> };
  });
}
