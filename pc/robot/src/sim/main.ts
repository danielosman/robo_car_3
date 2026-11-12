// `npm run sim -- <scenario> [--no-noise] [--no-drift] [--no-correction]` (doc/OUTLINE_MAP_PLAN.md,
// "The simulator"): runs a scenario through the map code (--no-correction: the pose isn't checked against the walls) and writes a folder with one
// picture per step and an index.html with a slider over them
// (recordings/sim/<scenario>/). Without a name it lists the scenarios.
import { mkdirSync, rmSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { BANDS, type BandName } from "../maps/outline/sights.ts";
import { NO_DRIFT } from "./drive.ts";
import { NO_NOISE } from "./tof.ts";
import { allFaces } from "./room.ts";
import { SCENARIOS } from "./scenarios.ts";
import { play } from "./play.ts";
import { poseError } from "./measure.ts";
import { outlineStrokes, type Stroke } from "../maps/outline/index.ts";
import { sliderPage, stepFile, stepPicture } from "./picture.ts";

const args = process.argv.slice(2), flags = new Set(args.filter(a => a.startsWith("--"))), name = args.find(a => !a.startsWith("--"));
const found = SCENARIOS.find(s => s.name === name);
if (!found || [...flags].some(f => f !== "--no-noise" && f !== "--no-drift" && f !== "--no-correction")) {
  console.log("npm run sim -- <scenario> [--no-noise] [--no-drift] [--no-correction]\n\nScenarios:");
  for (const s of SCENARIOS) console.log(`  ${s.name.padEnd(12)} ${s.about}`);
  process.exit(name ? 1 : 0);
}
const scenario = { ...found, ...(flags.has("--no-noise") ? { noise: NO_NOISE } : {}), ...(flags.has("--no-drift") ? { drift: NO_DRIFT } : {}) };

const directory = path.join(fileURLToPath(new URL("../../recordings/sim", import.meta.url)), scenario.name);
rmSync(directory, { recursive: true, force: true });
mkdirSync(directory, { recursive: true });

const DEG = Math.PI / 180, cm = (m: number) => (m * 100).toFixed(1), deg = (rad: number) => (rad / DEG).toFixed(1);
const everyFace = allFaces(scenario.room), captions: string[] = [];
const correct = !flags.has("--no-correction");
play(scenario, { correct }).forEach(({ step, bands: drawn, walls, corrected }, n) => {
  const bands = Object.fromEntries(BANDS.map(band => [band.name, outlineStrokes({ band: band.name, loops: drawn[band.name] })])) as Record<BandName, Stroke[]>;
  writeFileSync(path.join(directory, stepFile(n)), stepPicture(everyFace, { faces: step.faces, truth: step.truth, odometry: step.odometry, ...(correct ? { corrected } : {}), bands }));
  const off = poseError(step.odometry, step.truth), fixed = poseError(corrected, step.truth);
  captions.push(`Step ${n} of ${scenario.moves.trim().split(/\s+/).length - 1} (${scenario.moves.trim().split(/\s+/)[n]}): odometry is ${cm(off.m)} cm and ${deg(off.rad)}° off the truth` +
                (correct ? `, the corrected pose ${cm(fixed.m)} cm and ${deg(fixed.rad)}°; ` : "; ") +
                `edges: ${BANDS.map(band => `${band.name} ${bands[band.name].length}`).join(", ")}; ` +
                `walls: ${BANDS.map(band => `${band.name} ${walls[band.name].length}`).join(", ")}`);
});
writeFileSync(path.join(directory, "index.html"), sliderPage(`Simulator: ${scenario.name}`, scenario.about, captions));
console.log(captions[captions.length - 1]);
console.log(`${captions.length} pictures and index.html in ${directory}`);
