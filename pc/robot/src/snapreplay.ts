// Snapshots from a recorded drive, for trying the viewer without the robot
// (tools/replay_snapshots.c: the robot's own map and snapshot code on the drive's
// ToF frames, one snapshot after each take). Writes the raw snapshot file and
// imports it; the viewer then shows it as boot <boot id>.
//   npm run snapreplay -- <boot id> [out file, default recordings/snapshots/raw/replay-boot-<id>.snap]
import { execFile } from "node:child_process";
import { mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import { Store } from "./store.ts";
import { DB_PATH, importSnapshots } from "./import.ts";
import { exportFrames } from "./replay.ts";

const run = promisify(execFile);
const root = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");
const repo = path.join(root, "../..");
const BUILD = path.join(repo, "build", "replay");

const [boot, outArg] = process.argv.slice(2);
if (!boot || !/^[0-9a-f]{8}$/.test(boot)) { console.error("usage: npm run snapreplay -- <boot id> [out file]"); process.exit(2); }
const recordingsDir = path.join(root, "recordings");
const out = outArg ?? path.join(recordingsDir, "snapshots", "raw", `replay-boot-${boot}.snap`);
mkdirSync(BUILD, { recursive: true });
mkdirSync(path.dirname(out), { recursive: true });
const binary = path.join(BUILD, "replay_snapshots");
await run("cc", ["-std=c11", "-O2", `-I${repo}/picoA/app/test/stubs`, `-I${repo}/picoA/drivers`, `-I${repo}/common/test/fakes`,
  `-I${repo}/common`, `-I${repo}/picoA/app`, "-o", binary, path.join(repo, "tools/replay_snapshots.c"), "-lm"]);
const store = new Store(DB_PATH);
const takes = await store.query(`SELECT take_id, action, epoch_ms(started_at) AS ms FROM takes WHERE boot_id = '${boot}' ORDER BY take_no`);
if (!takes.length) { console.error(`no takes of boot ${boot} (npm run import?)`); process.exit(1); }
// started_at is local time: back to the PC's clock, the snapshots "arriving" as the drive did.
const startMs = Number(takes[0].ms) + new Date().getTimezoneOffset() * 60000;
const args = [out, boot, String(startMs)];
for (const t of takes) {
  const file = path.join(BUILD, `${t.take_id}.frames`);
  writeFileSync(file, await exportFrames(store, t.take_id as string));
  args.push(`${t.action}=${file}`);
}
const { stdout } = await run(binary, args, { maxBuffer: 16 << 20 });
process.stdout.write(stdout);
const n = await importSnapshots(store, out, recordingsDir);
console.log(`${out}: ${n.stored} snapshots imported`);
