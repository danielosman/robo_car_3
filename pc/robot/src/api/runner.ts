// Runs one command on the robot: numbers it, sends it, waits for its answer
// (and, for a snapshot, for the snapshot to be stored), within its time limit. Never
// throws: whatever happens is an outcome.
import type { DoneRecord, MapWindow, Pose } from "../link.ts";
import { robotText, timeLimitMs, type Command } from "./commands.ts";
import { failed, outcome, type Outcome } from "./answers.ts";

// What the runner needs of the robot (RobotLink is one).
export interface RobotConnection {
  sendCommand(n: number, text: string): boolean; // false: not connected
  on(event: "done", listener: (d: DoneRecord) => void): unknown;
  on(event: "mapWindow", listener: (w: MapWindow) => void): unknown;
  off?(event: "mapWindow", listener: (w: MapWindow) => void): unknown;
}

export interface Runner {
  run(c: Command): Promise<Outcome>;
  askMap?(): Promise<{ window: MapWindow; pose: Pose | null } | { error: string }>;
}

// The snapshots stored in DuckDB, by number, for the robot's newest boot: a snapshot
// command is answered once its snapshot is stored, so it can be read right away.
export class StoredSnapshots {
  private boot = "";
  private sequences = new Set<number>();
  private last: number | null = null;
  private waiters = new Map<number, (() => void)[]>();

  add(boot: string, sequence: number): void {
    if (boot !== this.boot) { this.boot = boot; this.sequences.clear(); this.last = null; }
    this.sequences.add(sequence);
    this.last = Math.max(this.last ?? 0, sequence);
    if (this.sequences.size > 100) this.sequences.delete(this.sequences.values().next().value!);
    for (const wake of this.waiters.get(sequence) ?? []) wake();
    this.waiters.delete(sequence);
  }

  // The newest snapshot stored, of the newest boot; null before the first.
  newest(): number | null { return this.last; }

  // True once snapshot sequence is stored; false if it isn't within ms.
  stored(sequence: number, ms: number): Promise<boolean> {
    if (this.sequences.has(sequence)) return Promise.resolve(true);
    return new Promise(resolve => {
      const wake = () => { clearTimeout(timer); resolve(true); };
      const timer = setTimeout(() => {
        this.waiters.set(sequence, (this.waiters.get(sequence) ?? []).filter(w => w !== wake));
        resolve(false);
      }, Math.max(0, ms));
      this.waiters.set(sequence, [...(this.waiters.get(sequence) ?? []), wake]);
    });
  }
}

export class CommandRunner implements Runner {
  private next = 1;
  private waiting = new Map<number, (d: DoneRecord) => void>();
  private mapChain: Promise<unknown> = Promise.resolve();
  private robot: RobotConnection;
  private snapshots: StoredSnapshots;
  private limitMs: (c: Command) => number;

  constructor(robot: RobotConnection, snapshots: StoredSnapshots, limitMs = timeLimitMs) {
    this.robot = robot;
    this.snapshots = snapshots;
    this.limitMs = limitMs;
    robot.on("done", d => this.waiting.get(d.n)?.(d)); // a number nothing waits for: ignored
  }

  async run(c: Command): Promise<Outcome> {
    const n = this.next++, deadline = Date.now() + this.limitMs(c);
    let timer: NodeJS.Timeout | undefined;
    const done = new Promise<DoneRecord | null>(resolve => {
      timer = setTimeout(() => { this.waiting.delete(n); resolve(null); }, deadline - Date.now());
      this.waiting.set(n, d => { clearTimeout(timer); this.waiting.delete(n); resolve(d); });
    });
    if (!this.robot.sendCommand(n, robotText(c))) {
      clearTimeout(timer);
      this.waiting.delete(n);
      return failed("robot not connected");
    }
    const d = await done;
    if (!d) return failed("no answer");
    if (d.snapshot !== null && !(await this.snapshots.stored(d.snapshot, deadline - Date.now()))) return failed("no answer (the snapshot wasn't stored)");
    return outcome(c, d);
  }

  // Asks the robot for its 6 x 6 m sliding window ("map" command) and waits for the 9
  // map_part messages and its answer, one request at a time.
  askMap(): Promise<{ window: MapWindow; pose: Pose | null } | { error: string }> {
    const run = this.mapChain.then(() => {
      const n = this.next++, ms = this.limitMs({ name: "snap", values: [] });
      return new Promise<{ window: MapWindow; pose: Pose | null } | { error: string }>(resolve => {
        let d: DoneRecord | null = null, w: MapWindow | null = null;
        const finish = (res: { window: MapWindow; pose: Pose | null } | { error: string }) => {
          clearTimeout(timer);
          this.waiting.delete(n);
          this.robot.off?.("mapWindow", onWindow);
          resolve(res);
        };
        const check = () => {
          if (d && d.end !== "ok") finish({ error: `the robot's map was ${d.end}` });
          else if (d && w) finish({ window: w, pose: d.pose });
        };
        const onWindow = (win: MapWindow) => { w = win; check(); };
        const timer = setTimeout(() => finish({ error: "no answer from the robot" }), ms);
        this.waiting.set(n, done => { d = done; check(); });
        this.robot.on("mapWindow", onWindow);
        if (!this.robot.sendCommand(n, "map")) finish({ error: "robot not connected" });
      });
    });
    this.mapChain = run.catch(() => {});
    return run;
  }
}
