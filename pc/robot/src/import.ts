// Imports recordings into DuckDB (recordings/robot.duckdb), every take again:
// what the database holds is derived from the raw files (notes and truth stay).
//   npm run import                 every file in recordings/raw/
//   npm run import -- <file.rec>   only these
import { readFileSync, readdirSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Dedup, TakeTracker, readRecordingFile, type Take } from "./recording.ts";
import { Store } from "./store.ts";

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
export const DB_PATH = path.join(root, "recordings", "robot.duckdb");

// The takes in a raw file. Files from before UDP have no arrival times: the
// file's name (when its connection opened) stands in.
export function takesIn(file: string): Take[] {
  const tracker = new TakeTracker(), dedup = new Dedup();
  tracker.file = file;
  const named = /(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)/.exec(path.basename(file));
  const fileMs = named ? new Date(+named[1], +named[2] - 1, +named[3], +named[4], +named[5], +named[6]).getTime() : null;
  for (const { arrivalMs, datagram, records } of readRecordingFile(readFileSync(file))) {
    if (datagram && !dedup.fresh(datagram)) continue;
    for (const r of records) tracker.add(r, datagram?.seq, arrivalMs ?? fileMs);
  }
  tracker.finish();
  return tracker.takes;
}

async function main(): Promise<void> {
  const args = process.argv.slice(2);
  const rawDir = path.join(root, "recordings", "raw");
  const files = args.length ? args : readdirSync(rawDir).filter(f => f.endsWith(".rec")).sort().map(f => path.join(rawDir, f));
  const store = new Store(DB_PATH);
  for (const file of files) {
    for (const take of takesIn(file)) {
      const n = await store.importTake(take);
      console.log(`${path.basename(file)}: ${take.name}: ${n.frames} frames, ${n.zones} zones, ${n.targets} targets, ${n.odom} odometry reports`);
    }
  }
  console.log(`database: ${path.relative(process.cwd(), DB_PATH)}`);
}

if (process.argv[1] === fileURLToPath(import.meta.url)) main().catch(err => { console.error(err); process.exit(1); });
