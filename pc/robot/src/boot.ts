// One power-up of the robot (a boot), as the PC sees it (doc/ARCHITECTURE.md, the PC):
// its id (8 hex digits, from hello), its own copy of the mounting (taken from
// common/mounting.json when the boot is first seen, kept with it in the store, so a
// later calibration doesn't move what this boot saw), its clock, and its messages in
// the order they came, numbered. Each message the store keeps becomes a row stamped
// here: the message number, the time since power-up, for a tof frame its move (by its
// number), for an imu batch each record's time (the batch names its move itself).
//
// The clock: PicoA's times are time_us_32(), which wraps every 71.6 min; it starts
// 30 s before the wrap (common/clock_start.h). BootClock turns each into µs since
// power-up, which only grows. The first time seen at or after 2^32 - 30 s is in the
// first 30 s; one before it is taken as after the first wrap (a server that first sees
// a boot more than 72 min after power-up is off by 71.6 min). After that each time
// follows the one before (a step of at most ±35 min; how long the PC waited says how
// many wraps fell in between, so a server restarted hours later still counts right).
import type { Mounting } from "./mounting.ts";
import type { ImuBatch, ImuRecord, MoveEnd, MoveStart } from "./wire.ts";
import type { PosedFrame } from "./tof_format.ts";
import type { LivePose } from "./link.ts";

export const WRAP_US = 2 ** 32;
export const CLOCK_START_US = WRAP_US - 30_000_000; // common/clock_start.h

export interface ClockState { lastRaw: number; lastUs: number; lastWallMs: number }

export class BootClock {
  private s: ClockState | null;
  constructor(state: ClockState | null = null) { this.s = state; }

  get state(): ClockState | null { return this.s && { ...this.s }; }

  // µs since power-up of PicoA's time raw (0 .. 2^32 - 1), seen at wallMs.
  sincePowerUp(raw: number, wallMs = Date.now()): number {
    let us: number;
    if (!this.s) us = raw >= CLOCK_START_US ? raw - CLOCK_START_US : raw + WRAP_US - CLOCK_START_US;
    else {
      const step = ((raw - this.s.lastRaw) | 0); // signed, across the wrap
      const waitedUs = Math.max(0, wallMs - this.s.lastWallMs) * 1000;
      us = this.s.lastUs + step + Math.round((waitedUs - step) / WRAP_US) * WRAP_US;
    }
    if (!this.s || us >= this.s.lastUs) this.s = { lastRaw: raw >>> 0, lastUs: us, lastWallMs: wallMs };
    return us;
  }
}

// What a boot was doing when the server last saw it (the store keeps it): its clock,
// how many messages, its open move and the frame numbers of the moves that ended.
export interface BootState {
  clock: ClockState | null; messages: number; openMove: number | null;
  ended: { move: number; first: number; last: number }[];
}

// The rows the store writes (store.ts), one per message.
export interface Stamp { boot: string; msg: number; atMs: number }
export type MoveStartRow = Stamp & MoveStart & { tS: number };
export type MoveEndRow = Stamp & MoveEnd & { tS: number };
export type FrameRow = Stamp & { format: string; frame: PosedFrame; move: number | null; tS: number };
export type StatusRow = Stamp & { pose: LivePose };
// An imu batch: its move (as the robot numbered it: right however late the batch
// comes) and its records, each with its time since power-up.
export type ImuRow = Stamp & { move: number; records: (ImuRecord & { tS: number })[] };

const ENDED_KEPT = 16; // late frames are filed under one of the last this many moves

export class Boot {
  readonly id: string;
  readonly mounting: Mounting;
  readonly clock: BootClock;
  private messages: number;
  private openMove: number | null;
  private ended: BootState["ended"];

  constructor(id: string, mounting: Mounting, state: BootState = { clock: null, messages: 0, openMove: null, ended: [] }) {
    this.id = id;
    this.mounting = mounting;
    this.clock = new BootClock(state.clock);
    this.messages = state.messages;
    this.openMove = state.openMove;
    this.ended = [...state.ended];
  }

  get state(): BootState {
    return { clock: this.clock.state, messages: this.messages, openMove: this.openMove, ended: [...this.ended] };
  }

  // The next message's stamp.
  stamp(atMs = Date.now()): Stamp { return { boot: this.id, msg: ++this.messages, atMs }; }

  private seconds(tUs: number, atMs: number): number { return this.clock.sincePowerUp(tUs, atMs) / 1e6; }

  // A refused move (it ends at once) leaves the running one open: its later frames are its.
  moveStart(m: MoveStart, atMs = Date.now()): MoveStartRow {
    if (!m.refused) this.openMove = m.move;
    return { ...this.stamp(atMs), ...m, tS: this.seconds(m.tUs, atMs) };
  }

  moveEnd(m: MoveEnd, atMs = Date.now()): MoveEndRow {
    if (this.openMove === m.move) this.openMove = null;
    if (m.firstFrame) this.ended = [...this.ended, { move: m.move, first: m.firstFrame, last: m.lastFrame }].slice(-ENDED_KEPT);
    return { ...this.stamp(atMs), ...m, tS: this.seconds(m.tUs, atMs) };
  }

  // A frame's move: one that ended and sent it (by its frame numbers: frames can come
  // after their move's end, which travels on the faster lane), else the open one.
  frame(format: string, frame: PosedFrame, atMs = Date.now()): FrameRow {
    const done = this.ended.find(e => frame.number >= e.first && frame.number <= e.last);
    return { ...this.stamp(atMs), format, frame, move: done ? done.move : this.openMove, tS: this.seconds(frame.tUs, atMs) };
  }

  status(pose: LivePose, atMs = Date.now()): StatusRow { return { ...this.stamp(atMs), pose }; }

  imu(batch: ImuBatch, atMs = Date.now()): ImuRow {
    return { ...this.stamp(atMs), move: batch.move, records: batch.records.map(r => ({ ...r, tS: this.seconds(r.tUs, atMs) })) };
  }
}
