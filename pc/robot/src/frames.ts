// A take's ToF frames as the robot's map receives them, for replay (tools/replay.sh):
// one line per whole frame with a pose: frame number, t (s), x, y, yaw, pitch, then per
// ray (row index × 8 + column index) the closest sure distance in mm (as
// rangefinder.c's closest_sure_mm: 0 nothing in range, 65535 not sure) and the first
// target's status.
//   npm run frames -- <take_id> <out file>
import { writeFileSync } from "node:fs";
import { Store } from "./store.ts";
import { DB_PATH } from "./import.ts";

const [take, out] = process.argv.slice(2);
if (!take || !out) { console.error("usage: npm run frames -- <take_id> <out file>"); process.exit(2); }
if (!/^[0-9a-f]{8}-\d+$/.test(take)) { console.error(`not a take id: ${take}`); process.exit(2); }
const store = new Store(DB_PATH);
const frames = await store.query(`SELECT frame_no, t_s, x, y, yaw, pitch FROM tof_frames
  WHERE take_id = '${take}' AND zones = 64 AND x IS NOT NULL ORDER BY frame_no`);
const zones = await store.query(`SELECT z.frame_no, (z.row - 1) * 8 + (z.col - 1) AS ray, z.targets,
    (SELECT min_by(t.distance_mm, t.target) FROM tof_targets t WHERE t.take_id = z.take_id AND t.frame_no = z.frame_no
       AND t.zone = z.zone AND t.status IN (5, 6, 9) AND t.distance_mm > 0) AS sure_mm,
    (SELECT min_by(t.status, t.target) FROM tof_targets t WHERE t.take_id = z.take_id AND t.frame_no = z.frame_no
       AND t.zone = z.zone) AS first_status
  FROM tof_zones z WHERE z.take_id = '${take}'`);
const rays = new Map<number, string[]>();
for (const z of zones) {
  const f = z.frame_no as number;
  if (!rays.has(f)) rays.set(f, Array(64).fill("65535 0"));
  rays.get(f)![z.ray as number] = z.targets === 0 ? "0 0" : z.sure_mm === null ? `65535 ${z.first_status}` : `${z.sure_mm} ${z.first_status}`;
}
const lines = frames.filter(f => rays.has(f.frame_no as number))
  .map(f => `${f.frame_no} ${f.t_s} ${f.x} ${f.y} ${f.yaw} ${f.pitch} ${rays.get(f.frame_no as number)!.join(" ")}`);
writeFileSync(out, lines.join("\n") + "\n");
console.log(`${take}: ${lines.length} frames -> ${out}`);
