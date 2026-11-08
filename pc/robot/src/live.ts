// The robot now, from its live feed (link.ts): the connection, the newest status, the
// boot (from hello), and the still times its moves use (from the newest move start, or
// a config the PC sent). The statuses are stored by main.ts (store.ts, status).
import type { LivePose, RobotStatus } from "./link.ts";

export class LiveRobot {
  status: RobotStatus = { state: "waiting" };
  pose: LivePose | null = null;
  boot: string | null = null;
  still: { preMs: number; postMs: number } | null = null; // null until a move or a config says

  get connected(): boolean { return this.status.state === "connected"; }

  setStatus(s: RobotStatus): void { this.status = s; }

  setBoot(boot: string): void {
    if (boot !== this.boot) this.still = null;
    this.boot = boot;
  }

  setStill(preMs: number, postMs: number): void { this.still = { preMs, postMs }; }

  setPose(p: LivePose): void { this.pose = p; }
}
