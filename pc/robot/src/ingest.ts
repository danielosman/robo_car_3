// What the robot sends (link.ts events), stored in the order it came (doc/ARCHITECTURE.md,
// the PC): hello opens its boot in the store (boot.ts: its own mounting and message
// count), then each move start and end, tof frame, imu batch, status and snapshot is
// stamped by the boot and written (store.ts); a snapshot's picture goes to recordings/camera/.
// Everything waits for its boot, which the store opens in a moment. main.ts and the
// fake robot's replay test use it.
import type { RobotLink, LivePose } from "./link.ts";
import type { Store } from "./store.ts";
import type { Boot } from "./boot.ts";
import type { BootFolders } from "./boot_folders.ts";
import type { Hello, ImuBatch, MoveEnd, TofMessage, MoveStart } from "./wire.ts";
import { loadMounting, type Mounting } from "./mounting.ts";
import { parseSnapshot, savePicture, type Snapshot } from "./snapshot.ts";

export interface IngestOptions {
  recordingsDir: string;
  bootFolders: BootFolders;
  note: (text: string) => void;
  mounting?: () => Mounting;                  // the copy a new boot gets (default: common/mounting.json now)
  stored?: (s: Snapshot) => void;             // a snapshot is in the store
}

// Stores what robot sends; returns a promise for "everything so far is stored".
export function ingest(robot: RobotLink, store: Store, o: IngestOptions): { idle: () => Promise<void> } {
  let boot: Boot | null = null, chain: Promise<unknown> = Promise.resolve();
  const inOrder = (what: string, fn: (b: Boot) => Promise<unknown> | void) => {
    chain = chain.then(() => (boot ? fn(boot) : undefined)).catch(err => o.note(`${what} NOT stored: ${(err as Error).message}`));
  };
  // The same boot saying hello again (a new connection): it goes on as it was.
  robot.on("hello", (h: Hello) => {
    chain = chain.then(async () => { if (boot?.id !== h.bootId) boot = await store.openBoot(h, (o.mounting ?? loadMounting)()); })
      .catch(err => { boot = null; o.note(`boot ${h.bootId} NOT opened in the store: ${(err as Error).message}`); });
  });
  robot.on("moveStart", (m: MoveStart) => inOrder(`move ${m.move}`, b => { void store.moveStart(b, b.moveStart(m)); }));
  robot.on("moveEnd", (m: MoveEnd) => inOrder(`move ${m.move}`, b => { void store.moveEnd(b, b.moveEnd(m)); }));
  robot.on("tof", (f: TofMessage) => inOrder(`frame ${f.frame.number}`, b => { void store.tof(b, b.tof(f)); }));
  robot.on("imuBatch", (batch: ImuBatch) => inOrder(`an imu batch of move ${batch.move}`, b => { void store.imuBatch(b, b.imuBatch(batch)); }));
  robot.on("pose", (p: LivePose) => inOrder("a status", b => { void store.status(b, b.status(p)); }));
  robot.on("snapshot", (bytes: Buffer) => {
    const ms = Date.now();
    let s: Snapshot;
    try { s = parseSnapshot(bytes); } catch (err) { o.note(`a snapshot NOT kept: ${(err as Error).message}`); return; }
    inOrder(`snapshot ${s.seq}`, async b => {
      const msg = b.stamp(ms).msg;
      let picture: string | null = null;
      try { picture = savePicture(o.recordingsDir, o.bootFolders.folderForBoot(s.bootId, ms), s); }
      catch (err) { o.note(`snapshot ${s.seq}: picture NOT saved: ${(err as Error).message}`); }
      await store.importSnapshot(b, msg, s, bytes, ms, picture);
      o.stored?.(s);
    });
  });
  return { idle: async () => { await chain; await store.flush(); } };
}
