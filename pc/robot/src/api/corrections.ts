// The PC's correction sent to the robot (doc/OUTLINE_MAP_PLAN.md, step F): a Runner round
// the CommandRunner that keeps the robot's correction (c_robot) the same as the PC's
// (c_pc, the outline map's at the newest snapshot), and turns goto targets from the map's
// frame into the robot's. In the one queue, so a correction is never sent during a move:
//   - after a snap is answered (its snapshot stored, the outline map brought up to it),
//     and before every move: if c_pc differs from c_robot, "correct x y yaw" (mm, 0.01°)
//     goes to the robot, and the log says how far the robot's pose moved;
//   - a goto's point (the map's frame: the page's click, robot steer g) goes to the robot
//     in its own frame: corrected(c_robot, uncorrected(c_pc, point)). With c_robot = c_pc
//     that is the point itself; with an older c_robot (a correct not answered) the robot
//     still drives to the place meant.
// c_robot is as the robot last said it ("corrected", wire.ts; robot_corrections in the
// store), never as sent: a correct without an answer is sent again before the next move.
// A new boot starts at 0. A correction that can't be had (no outline map, an error) is
// skipped with a note: the move runs as it would without step F. c_robot is also in the
// store (robot_corrections, for its readers); both come from the same "corrected" message.
// A goto before the server's first snapshot of a run (a server restarted mid-boot) goes as given.
//   new Corrections(runner, { live, snapshots, outlines, store, note })
//   corrections.heard(boot, c)    the robot's "corrected" (main.ts)
//   corrections.robotHas(boot)    c_robot as the robot last said it (from the store after a server restart)
//   corrections.raw(boot, pose)   a pose the robot sends now (its status), back to raw odometry
import type { DoneRecord, MapWindow, Pose } from "../link.ts";
import { corrected, NO_CORRECTION, uncorrected, type Planar } from "../correction.ts";
import { isMove, type Command } from "./commands.ts";
import type { Outcome } from "./answers.ts";
import type { Runner } from "./runner.ts";

const CORRECT_MS = 5000;   // how long the robot may take to answer a correct

export interface CorrectionDeps {
  live: { boot: string | null; pose: Planar | null };
  snapshots: { newestOf(boot: string): number | null };
  outlines: { correction(boot: string, sequence: number): Promise<Planar | null> };
  store: { corrections(boot: string): Promise<{ c: Planar }[]> };
  note: (text: string) => void;
}
interface Inner extends Runner { send(text: string, ms: number): Promise<DoneRecord | string> }

// A correction as the robot takes it: mm and 0.01°, yaw within ±180°.
function words(c: Planar): [number, number, number] {
  const yaw = Math.atan2(Math.sin(c.yaw), Math.cos(c.yaw));
  return [Math.round(c.x * 1000), Math.round(c.y * 1000), Math.round(yaw * 18000 / Math.PI)];
}
const same = (a: Planar, b: Planar) => words(a).every((v, i) => v === words(b)[i]);

export class Corrections implements Runner {
  private readonly inner: Inner;
  private readonly deps: CorrectionDeps;
  private readonly robot = new Map<string, Planar>(); // c_robot by boot

  constructor(inner: Inner, deps: CorrectionDeps) {
    this.inner = inner;
    this.deps = deps;
  }

  askMap(): Promise<{ window: MapWindow; pose: Pose | null } | { error: string }> {
    return this.inner.askMap ? this.inner.askMap() : Promise.resolve({ error: "robot not connected" });
  }

  heard(boot: string | null, c: Planar): void { if (boot) this.robot.set(boot, c); }

  async robotHas(boot: string): Promise<Planar> {
    if (!this.robot.has(boot)) {
      const last = (await this.deps.store.corrections(boot)).at(-1)?.c ?? NO_CORRECTION;
      if (!this.robot.has(boot)) this.robot.set(boot, last);  // a "corrected" heard meanwhile is newer
    }
    return this.robot.get(boot)!;
  }

  // Before robotHas() has read the store (a server restarted mid-boot), as if 0.
  raw<P extends Planar>(boot: string | null, p: P): P {
    const c = boot && this.robot.get(boot);
    return c ? uncorrected(c, p) : p;
  }

  async run(c: Command): Promise<Outcome> {
    let sent = c;
    if (isMove(c)) {
      const k = await this.sync();
      if (c.name === "goto" && k) {
        const t = corrected(k.robot, uncorrected(k.pc, { x: c.values[0] / 100, y: c.values[1] / 100, yaw: 0 }));
        sent = { ...c, values: [Math.round(t.x * 100), Math.round(t.y * 100)] };
      }
    }
    const out = await this.inner.run(sent);
    if (c.name === "snap" && out.result === "ok") await this.sync(out.snapshot);
    return out;
  }

  // c_robot made c_pc if it differs (the newest snapshot's, or `sequence`'s); both as they
  // are then (c_robot as the robot answered); null without a boot or an outline map.
  private async sync(sequence?: number): Promise<{ pc: Planar; robot: Planar } | null> {
    const boot = this.deps.live.boot, n = sequence ?? (boot ? this.deps.snapshots.newestOf(boot) : null);
    if (!boot || n === null) return null;
    let pc: Planar | null, robot: Planar;
    try {
      [pc, robot] = await Promise.all([this.deps.outlines.correction(boot, n), this.robotHas(boot)]);
    } catch (err) {
      this.deps.note(`correction not sent: the outline map failed (${(err as Error).message})`);
      return null;
    }
    if (!pc) return null;
    if (same(pc, robot)) return { pc, robot };
    const [x, y, yaw] = words(pc);
    const d = await this.inner.send(`correct ${x} ${y} ${yaw}`, CORRECT_MS);
    const now = await this.robotHas(boot);
    // Refused: out of the robot's range, or something still waits on its bulk lane (commands.h).
    if (typeof d === "string" || d.end !== "ok") {
      this.deps.note(`correction not taken by the robot (${typeof d === "string" ? d : d.end}): sent again before the next move`);
      return { pc, robot: now };
    }
    // How far the robot's own pose moved: where it is, by the old correction and the new.
    const at = this.deps.live.pose ?? NO_CORRECTION, raw = uncorrected(robot, at), moved = corrected(now, raw);
    const turned = Math.atan2(Math.sin(moved.yaw - at.yaw), Math.cos(moved.yaw - at.yaw));
    this.deps.note(`correction sent: the robot's pose moved ${(Math.hypot(moved.x - at.x, moved.y - at.y) * 100).toFixed(1)} cm, ` +
                   `${(Math.round(turned * 1800 / Math.PI) / 10 || 0).toFixed(1)}°`);
    return { pc, robot: now };
  }
}
