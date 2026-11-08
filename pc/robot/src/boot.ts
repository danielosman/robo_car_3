// One power-up of the robot (a boot), as the PC sees it (doc/ARCHITECTURE.md, the PC):
// its id (8 hex digits, from hello), its own copy of the mounting (taken from
// common/mounting.json when the boot is first seen, kept with it in the store, so a
// later calibration doesn't move what this boot saw), and its messages in the order
// they came, numbered. Each message the store keeps becomes a row stamped here: the
// message number, and its time in s since power-up (the robot sends µs since
// power-up, which never wrap: wire.ts). Frames and imu batches name their move
// themselves.
import type { Mounting } from "./mounting.ts";
import type { ImuBatch, ImuRecord, MoveEnd, MoveFrame, MoveStart } from "./wire.ts";
import type { PosedFrame } from "./tof_format.ts";
import type { LivePose } from "./link.ts";

// The rows the store writes (store.ts), one per message.
export interface Stamp { boot: string; msg: number; atMs: number }
export type MoveStartRow = Stamp & MoveStart & { tS: number };
export type MoveEndRow = Stamp & MoveEnd & { tS: number };
export type FrameRow = Stamp & { format: string; frame: PosedFrame; move: number; tS: number };
export type StatusRow = Stamp & { pose: LivePose };
// An imu batch: its move and its records, each with its time since power-up.
export type ImuRow = Stamp & { move: number; records: (ImuRecord & { tS: number })[] };

const seconds = (us: number) => us / 1e6;

export class Boot {
  readonly id: string;
  readonly mounting: Mounting;
  private count: number;

  // messages: how many the store already has (a boot seen again after a server restart).
  constructor(id: string, mounting: Mounting, messages = 0) {
    this.id = id;
    this.mounting = mounting;
    this.count = messages;
  }

  get messages(): number { return this.count; }

  // The next message's stamp.
  stamp(atMs = Date.now()): Stamp { return { boot: this.id, msg: ++this.count, atMs }; }

  moveStart(m: MoveStart, atMs = Date.now()): MoveStartRow { return { ...this.stamp(atMs), ...m, tS: seconds(m.tUs) }; }
  moveEnd(m: MoveEnd, atMs = Date.now()): MoveEndRow { return { ...this.stamp(atMs), ...m, tS: seconds(m.tUs) }; }

  frame(f: MoveFrame, atMs = Date.now()): FrameRow {
    return { ...this.stamp(atMs), format: f.format, frame: f.frame, move: f.move, tS: seconds(f.tUs) };
  }

  status(pose: LivePose, atMs = Date.now()): StatusRow { return { ...this.stamp(atMs), pose }; }

  imu(batch: ImuBatch, atMs = Date.now()): ImuRow {
    return { ...this.stamp(atMs), move: batch.move, records: batch.records.map(r => ({ ...r, tS: seconds(r.tUs) })) };
  }
}
