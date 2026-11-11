// The simulator's pictures (doc/OUTLINE_MAP_PLAN.md, "What is drawn"): one PNG per step,
// three panels side by side (L1, L2, above). Each shows the room as it truly is at that
// band's heights (grey), the true robot (green), the odometry robot (blue), and what
// the map under test drew for the band. Map frame as `robot map --png`: x up, y left.
//   stepPicture(room faces, poses, each band's strokes)  the PNG (the outline's colours: maps/outline/index.ts)
//   sliderPage(title, captions)                        index.html: a slider over step-<n>.png
import { rgbPng } from "../snapshot.ts";
import { BANDS, type BandName } from "../maps/outline/sights.ts";
import type { Rgb, Stroke } from "../maps/outline/index.ts";
import type { Face } from "./room.ts";
import type { Pose } from "./drive.ts";


const PX_PER_M = 100, MARGIN_M = 0.3, GAP_PX = 12;
const BACK: Rgb = [0x15, 0x15, 0x15], ROOM: Rgb = [0x70, 0x70, 0x70], TRUE: Rgb = [0x5f, 0xe0, 0x8a], ODOMETRY: Rgb = [0x5a, 0x8d, 0xff];
const ROBOT_M = 0.1; // the robot is drawn as a circle this wide across, with a line along its heading

export interface StepDrawing { faces: Face[]; truth: Pose; odometry: Pose; corrected?: Pose; bands: Record<BandName, Stroke[]> }

export function stepPicture(all: Face[], d: StepDrawing): Buffer {
  // The picture covers every face of the scenario, so all steps have the same size.
  const xs = all.flatMap(f => [f.ax, f.bx]), ys = all.flatMap(f => [f.ay, f.by]);
  const maxX = Math.max(...xs) + MARGIN_M, maxY = Math.max(...ys) + MARGIN_M;
  const panelW = Math.ceil((maxY - Math.min(...ys) + MARGIN_M) * PX_PER_M), height = Math.ceil((maxX - Math.min(...xs) + MARGIN_M) * PX_PER_M);
  const width = panelW * BANDS.length + GAP_PX * (BANDS.length - 1), pixels = Buffer.alloc(width * height * 3);
  for (let i = 0; i < width * height; i++) pixels.set((i % width) % (panelW + GAP_PX) < panelW ? BACK : [0, 0, 0], i * 3);
  BANDS.forEach((band, panel) => {
    const left = panel * (panelW + GAP_PX);
    const put = (px: number, py: number, rgb: Rgb) => {
      if (px >= 0 && py >= 0 && px < panelW && py < height) pixels.set(rgb, (py * width + left + px) * 3);
    };
    const line = (s: Stroke) => {
      const ax = (maxY - s.ay) * PX_PER_M, ay = (maxX - s.ax) * PX_PER_M, bx = (maxY - s.by) * PX_PER_M, by = (maxX - s.bx) * PX_PER_M;
      const n = Math.ceil(Math.hypot(bx - ax, by - ay)) + 1;
      for (let i = 0; i <= n; i++) put(Math.round(ax + (bx - ax) * i / n), Math.round(ay + (by - ay) * i / n), s.rgb);
    };
    const robot = (p: Pose, rgb: Rgb) => {
      for (let i = 0; i < 24; i++) {
        const a = i * Math.PI / 12, b = a + Math.PI / 12;
        line({ ax: p.x + ROBOT_M * Math.cos(a), ay: p.y + ROBOT_M * Math.sin(a), bx: p.x + ROBOT_M * Math.cos(b), by: p.y + ROBOT_M * Math.sin(b), rgb });
      }
      line({ ax: p.x, ay: p.y, bx: p.x + 2 * ROBOT_M * Math.cos(p.yaw), by: p.y + 2 * ROBOT_M * Math.sin(p.yaw), rgb });
    };
    for (const f of d.faces) if (f.lowM < band.highM && f.highM > band.lowM) line({ ...f, rgb: ROOM });
    for (const s of d.bands[band.name]) line(s);
    robot(d.odometry, ODOMETRY);
    if (d.corrected) robot(d.corrected, [0xff, 0xff, 0xff]);
    robot(d.truth, TRUE);
  });
  return rgbPng(width, height, pixels);
}

export const stepFile = (step: number) => `step-${String(step).padStart(3, "0")}.png`;

// A page with a slider over the steps' pictures (step-000.png ...), each with its caption.
export function sliderPage(title: string, about: string, captions: string[]): string {
  const escaped = (s: string) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  return `<!doctype html>
<meta charset="utf-8">
<title>${escaped(title)}</title>
<style>
  body { background: #111; color: #ddd; font: 14px system-ui, sans-serif; margin: 16px; }
  img { display: block; max-width: 100%; image-rendering: pixelated; margin-top: 8px; }
  input { width: min(100%, 900px); }
  .key span { margin-right: 14px; }
</style>
<h3>${escaped(title)}</h3>
<p>${escaped(about)}</p>
<p class="key">Panels: L1 (3-13 cm), L2 (13-23 cm), above.
  <span style="color:#707070">■ the room</span><span style="color:#5fe08a">■ the true robot</span>
  <span style="color:#5a8dff">■ odometry</span><span style="color:#fff">■ corrected</span></p>
<input id="slider" type="range" min="0" max="${captions.length - 1}" value="${captions.length - 1}">
<div id="caption"></div>
<img id="picture" alt="the step's picture">
<script>
  const captions = ${JSON.stringify(captions)};
  const slider = document.getElementById("slider");
  const show = () => {
    const n = Number(slider.value);
    document.getElementById("picture").src = "step-" + String(n).padStart(3, "0") + ".png";
    document.getElementById("caption").textContent = captions[n];
  };
  slider.oninput = show;
  show();
</script>
`;
}
