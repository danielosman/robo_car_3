// Imports recordings into DuckDB (recordings/robot.duckdb), every take and snapshot
// again: what the database holds is derived from the raw files (notes and truth
// stay), and so are the snapshots' pictures (recordings/snapshots/<boot>/<seq>.png).
//   npm run import                         every file in recordings/raw/ and recordings/snapshots/raw/
//   npm run import -- <file.rec|file.snap>  only these
import { readFileSync, readdirSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Dedup, TakeTracker, readRecordingFile, type Take } from "./recording.ts";
import { Store } from "./store.ts";
import { readSnapshotFile, savePicture } from "./snapshot.ts";

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

// Every snapshot in a raw snapshot file into the database, its picture written again.
export async function importSnapshots(store: Store, file: string, recordingsDir: string): Promise<{ stored: number; damaged: number }> {
  let stored = 0, damaged = 0;
  for (const { arrivalMs, snapshot } of readSnapshotFile(readFileSync(file))) {
    if (!snapshot) { damaged++; continue; }
    const png = savePicture(path.join(recordingsDir, "snapshots"), snapshot);
    await store.importSnapshot(snapshot, arrivalMs, file, png ? path.relative(recordingsDir, png) : null);
    stored++;
  }
  return { stored, damaged };
}

const listed = (dir: string, ext: string) => {
  try { return readdirSync(dir).filter(f => f.endsWith(ext)).sort().map(f => path.join(dir, f)); } catch { return []; }
};

async function main(): Promise<void> {
  const args = process.argv.slice(2);
  const recordingsDir = path.join(root, "recordings");
  const files = args.length ? args : [...listed(path.join(recordingsDir, "raw"), ".rec"),
                                      ...listed(path.join(recordingsDir, "snapshots", "raw"), ".snap")];
  const store = new Store(DB_PATH);
  for (const file of files) {
    if (file.endsWith(".snap")) {
      const n = await importSnapshots(store, file, recordingsDir);
      console.log(`${path.basename(file)}: ${n.stored} snapshots${n.damaged ? `, ${n.damaged} damaged` : ""}`);
      continue;
    }
    for (const take of takesIn(file)) {
      const n = await store.importTake(take);
      console.log(`${path.basename(file)}: ${take.name}: ${n.frames} frames, ${n.zones} zones, ${n.targets} targets, ${n.odom} odometry reports`);
    }
  }
  console.log(`database: ${path.relative(process.cwd(), DB_PATH)}`);
}

if (process.argv[1] === fileURLToPath(import.meta.url)) main().catch(err => { console.error(err); process.exit(1); });
