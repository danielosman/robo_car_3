// npm run place-objects [-- [--from-file] [<boot>]]: places every saved detection again
// (db.ts placeAgain), without asking the model, each with its boot's own copy of the
// mounting (store.ts boots). --from-file first gives the boot (every boot without
// one named) today's common/mounting.json as its copy: after a calibration that also
// holds for those drives.
import { DB_PATH, Store } from "../store.ts";
import { loadMounting } from "../mounting.ts";
import { placeAgain } from "./db.ts";
import { cameraGeometry, place } from "./place.ts";

const args = process.argv.slice(2), fromFile = args.includes("--from-file");
const boot = args.find(a => !a.startsWith("--")) ?? null;
if (boot !== null && !/^[0-9a-f]{8}$/.test(boot)) { console.error("usage: npm run place-objects -- [--from-file] [<boot id>]"); process.exit(2); }
const store = new Store(DB_PATH);
if (fromFile) {
  const m = loadMounting();
  const boots = boot ? [boot] : (await store.query("SELECT boot_id FROM boots")).map(r => String(r.boot_id));
  for (const b of boots) await store.setMounting(b, m);
  console.log(`gave ${boots.length} boot(s) today's mounting.json`);
}
const n = await placeAgain(store, (d, pose, mounting) => place(d, pose, cameraGeometry(mounting)), boot);
console.log(`placed ${n} objects again`);
