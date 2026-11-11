// Hand-made sights for the line map's tests (pieces, line_map): no helper, no ToF frame.
import type { Sight, SightRow } from "./sights.ts";

export const DEG = Math.PI / 180, ZONE = 5.625 * DEG;

// A sight from the sensor (the origin unless `more` says otherwise) along `bearing` that
// hit something `rangeM` away, 20-30 cm up; null: no hit, it saw through 2 m.
export function sight(bearing: number, rangeM: number | null, more: Partial<Sight> = {}): Sight {
  const x = more.x ?? 0, y = more.y ?? 0;
  const hit = rangeM === null ? null
    : { x: x + rangeM * Math.cos(bearing), y: y + rangeM * Math.sin(bearing), rangeM, sigmaM: 0.005, lowM: 0.2, highM: 0.3 };
  return { x, y, bearing, halfWidth: ZONE / 2, reachM: rangeM === null ? 2 : rangeM * 0.9, z: 0.09, lowPerM: 0.1, highPerM: 0.2, weight: 1, hit, ...more };
}

// A row of 8 looking along `facing` from the sensor, column 1 on the left: what each zone
// hits, by its bearing and column (1-8); null: nothing.
export function row(rangeAt: (bearing: number, col: number) => number | null, facing = 0, more: Partial<Sight> = {}): SightRow {
  return Array.from({ length: 8 }, (_, c) => { const bearing = facing + (3.5 - c) * ZONE; return sight(bearing, rangeAt(bearing, c + 1), more); });
}
