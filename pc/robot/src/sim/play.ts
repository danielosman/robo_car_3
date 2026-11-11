// A scenario played through the map under test (doc/OUTLINE_MAP_PLAN.md, "The
// simulator"): the one place that says which map code the simulator runs. main.ts
// draws what this gives; the tests measure it.
//   play(scenario)   per step: the simulator's step and what the map drew for each band
import { LineMap, type Segment } from "../maps/line_map/line_map.ts";
import { BANDS, inBand, type BandName } from "../maps/line_map/sights.ts";
import { run, type Scenario, type SimStep } from "./scenarios.ts";

export interface Played { step: SimStep; bands: Record<BandName, Segment[]> }

export function play(s: Scenario): Played[] {
  const maps = BANDS.map(() => new LineMap());
  return run(s).map(step => {
    for (const f of step.frames) BANDS.forEach((band, i) => maps[i].add(inBand(f.sights, band)));
    return { step, bands: Object.fromEntries(BANDS.map((band, i) => [band.name, maps[i].lines()])) as Record<BandName, Segment[]> };
  });
}
