// Hand-made sights for the line map's tests: no helper, no ToF frame.
import { heightsAt, type Sight } from "./sights.ts";

export const DEG = Math.PI / 180, ZONE = 5.625 * DEG;

// A sight from the sensor (the origin, 9 cm up, unless `more` says otherwise) along
// `bearing` that hit something `rangeM` away; null: no hit, it saw through 2 m. The
// wedge rises 10 to 20 cm a metre unless `more` gives other slopes; a hit is as high as
// the wedge is there. It saw through to 10 cm before its hit.
export function sight(bearing: number, rangeM: number | null, more: Partial<Sight> = {}): Sight {
  const s: Sight = { x: 0, y: 0, bearing, halfWidth: ZONE / 2, reachM: rangeM === null ? 2 : Math.max(0, rangeM - 0.1),
                     z: 0.09, lowPerM: 0.1, highPerM: 0.2, weight: 1, hit: null, ...more };
  if (rangeM !== null) {
    const [lowM, highM] = heightsAt(s, rangeM);
    s.hit = { x: s.x + rangeM * Math.cos(bearing), y: s.y + rangeM * Math.sin(bearing), rangeM, lowM, highM };
  }
  return s;
}

// A row of 8 looking along `facing` from the sensor, column 1 on the left: what each zone
// hits, by its bearing and column (1-8); null: nothing.
export function row(rangeAt: (bearing: number, col: number) => number | null, facing = 0, more: Partial<Sight> = {}): Sight[] {
  return Array.from({ length: 8 }, (_, c) => { const bearing = facing + (3.5 - c) * ZONE; return sight(bearing, rangeAt(bearing, c + 1), more); });
}
