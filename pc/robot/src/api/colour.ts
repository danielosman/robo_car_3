// A name for the colour sensor's reading (CIE 1931 x, y): a white light by its colour
// temperature when it lies near the black-body curve (warm, neutral, cool), else a
// hue around the D65 white point. Rough on purpose: the sensor's colours aren't
// calibrated for the board (README, OPT4048).

// The black-body curve at temperature t (K), 1667-25000 K (Kim et al., 2002).
function planck(t: number): [number, number] {
  const x = t <= 4000 ? -0.2661239e9 / t ** 3 - 0.2343589e6 / t ** 2 + 0.8776956e3 / t + 0.17991
                      : -3.0258469e9 / t ** 3 + 2.1070379e6 / t ** 2 + 0.2226347e3 / t + 0.24039;
  const y = t <= 2222 ? -1.1063814 * x ** 3 - 1.3481102 * x ** 2 + 2.18555832 * x - 0.20219683
          : t <= 4000 ? -0.9549476 * x ** 3 - 1.37418593 * x ** 2 + 2.09137015 * x - 0.16748867
          : 3.081758 * x ** 3 - 5.8733867 * x ** 2 + 3.75112997 * x - 0.37001483;
  return [x, y];
}

const HUES: [number, string][] = [[10, "red"], [35, "orange"], [75, "yellow"], [150, "green"], [210, "cyan"],
                                  [250, "blue"], [285, "purple"], [345, "pink"], [360, "red"]];

export function colourName(x: number, y: number, lux: number): string {
  if (lux < 1) return "dark";
  const n = (x - 0.332) / (0.1858 - y), cct = 449 * n ** 3 + 3525 * n ** 2 + 6823.3 * n + 5520.33; // McCamy
  if (cct >= 1667 && cct <= 25000) {
    const [px, py] = planck(cct);
    if (Math.hypot(x - px, y - py) < 0.02) {
      const k = Math.round(cct / 100) * 100;
      return `${cct < 3500 ? "warm" : cct < 5000 ? "neutral" : "cool"} white (${k} K)`;
    }
  }
  const hue = (Math.atan2(y - 0.329, x - 0.3127) * 180 / Math.PI + 360) % 360;
  return HUES.find(([upTo]) => hue < upTo)![1];
}
