// The robot now, from its live feed (robot.ts): the connection, the newest pose line,
// the boot (that of the newest snapshot on this connection: a robot that started
// again sends one when it connects), and the largest jolt and tilt since that boot
// started. Each pose line is stored with its boot (store.ts, poses), a few seconds'
// worth at a time; lines before the connection's first snapshot have no boot and
// are not stored.
import type { LivePose, RobotStatus } from "./robot.ts";
import type { Store } from "./store.ts";

const FLUSH_MS = 5000;

export interface Peaks { jolt: [number, number, number]; tilt: number }

export class LiveRobot {
  status: RobotStatus = { state: "waiting" };
  pose: LivePose | null = null;
  boot: string | null = null;
  peaks: Peaks = { jolt: [0, 0, 0], tilt: 0 };
  private peaksBoot: string | null = null;
  private rows: { boot: string; pose: LivePose }[] = [];
  private timer: NodeJS.Timeout;
  private store: Store;
  private onError: (err: Error) => void;

  constructor(store: Store, onError: (err: Error) => void) {
    this.store = store;
    this.onError = onError;
    this.timer = setInterval(() => this.flush(), FLUSH_MS);
  }

  get connected(): boolean { return this.status.state === "connected"; }

  setStatus(s: RobotStatus): void {
    if (s.state === "connected" && this.status.state !== "connected") this.boot = null; // perhaps another boot
    this.status = s;
  }

  setBoot(boot: string): void {
    this.boot = boot;
    if (boot !== this.peaksBoot) { this.peaksBoot = boot; this.peaks = { jolt: [0, 0, 0], tilt: 0 }; }
  }

  setPose(p: LivePose): void {
    this.pose = p;
    if (p.jolt) this.peaks.jolt = this.peaks.jolt.map((v, i) => Math.max(v, p.jolt![i])) as Peaks["jolt"];
    if (p.tilt !== null) this.peaks.tilt = Math.max(this.peaks.tilt, p.tilt);
    if (this.boot) this.rows.push({ boot: this.boot, pose: p });
  }

  // Stores the pose lines not stored yet.
  flush(): Promise<void> {
    const rows = this.rows.splice(0);
    return this.store.appendPoses(rows).catch(err => this.onError(err as Error));
  }

  stop(): Promise<void> {
    clearInterval(this.timer);
    return this.flush();
  }
}
