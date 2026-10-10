// A take's ToF frames as the robot's map receives them, for replay (tools/replay.sh;
// the format is replay.ts's exportFrames).
//   npm run frames -- <take_id> <out file>
import { writeFileSync } from "node:fs";
import { Store } from "./store.ts";
import { DB_PATH } from "./import.ts";
import { exportFrames } from "./replay.ts";

const [take, out] = process.argv.slice(2);
if (!take || !out) { console.error("usage: npm run frames -- <take_id> <out file>"); process.exit(2); }
const lines = await exportFrames(new Store(DB_PATH), take);
writeFileSync(out, lines);
console.log(`${take}: ${lines.split("\n").length - 1} frames -> ${out}`);
