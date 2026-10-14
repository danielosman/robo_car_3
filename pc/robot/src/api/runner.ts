// Runs one command on the robot: numbers it, sends it, waits for its done record
// (and, for a snapshot, for the snapshot to be stored), within its time limit. Never
// throws: whatever happens is an outcome.
import type { DoneRecord } from "../robot.ts";
import { robotLine, timeLimitMs, type Command } from "./commands.ts";
import { describe, failed, type Outcome } from "./answers.ts";

// What the runner needs of the robot (RobotLink is one).
export interface RobotConnection {
  sendLine(line: string): boolean; // false: not connected
  on(event: "done", listener: (d: DoneRecord) => void): unknown;
}

export interface Runner { run(c: Command): Promise<Outcome> }

// The snapshots stored in DuckDB, by number, for the robot's newest boot: a snapshot
// command is answered once its snapshot is stored, so it can be read right away.
export class StoredSnapshots {
  private boot = "";
  private seqs = new Set<number>();
  private waiters = new Map<number, (() => void)[]>();

  add(boot: string, seq: number): void {
    if (boot !== this.boot) { this.boot = boot; this.seqs.clear(); }
    this.seqs.add(seq);
    if (this.seqs.size > 100) this.seqs.delete(this.seqs.values().next().value!);
    for (const wake of this.waiters.get(seq) ?? []) wake();
    this.waiters.delete(seq);
  }

  // True once snapshot seq is stored; false if it isn't within ms.
  stored(seq: number, ms: number): Promise<boolean> {
    if (this.seqs.has(seq)) return Promise.resolve(true);
    return new Promise(resolve => {
      const wake = () => { clearTimeout(timer); resolve(true); };
      const timer = setTimeout(() => {
        this.waiters.set(seq, (this.waiters.get(seq) ?? []).filter(w => w !== wake));
        resolve(false);
      }, Math.max(0, ms));
      this.waiters.set(seq, [...(this.waiters.get(seq) ?? []), wake]);
    });
  }
}

export class CommandRunner implements Runner {
  private next = 1;
  private waiting = new Map<number, (d: DoneRecord) => void>();
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
    if (!this.robot.sendLine(robotLine(c, n))) {
      clearTimeout(timer);
      this.waiting.delete(n);
      return failed("robot not connected");
    }
    const d = await done;
    if (!d) return failed("no answer");
    if (d.snapshot !== null && !(await this.snapshots.stored(d.snapshot, deadline - Date.now()))) return failed("no answer (the snapshot wasn't stored)");
    return describe(c, d);
  }
}
