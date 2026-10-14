// The colour sensor's colour on a screen: its XYZ (any scale) as sRGB at a fixed
// brightness, as the page's swatch shows it (public/app.js, swatch()).
export function displayRgb(X: number, Y: number, Z: number): [number, number, number] | null {
  if (!(Y > 0)) return null;
  const k = 0.5 / Y;
  const lin = [3.2406 * X - 1.5372 * Y - 0.4986 * Z, -0.9689 * X + 1.8758 * Y + 0.0415 * Z, 0.0557 * X - 0.204 * Y + 1.057 * Z].map(v => v * k);
  const max = Math.max(1, ...lin);
  const g = lin.map(v => Math.max(0, v / max)).map(v => (v <= 0.0031308 ? 12.92 * v : 1.055 * v ** (1 / 2.4) - 0.055));
  return g.map(v => Math.round(v * 255)) as [number, number, number];
}
