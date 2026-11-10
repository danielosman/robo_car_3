// For the tests: the firmware's own snapshots (build/test_snapshot.snap, written by
// ./run_tests.sh) stored in a fresh database, as the server stores them when they
// arrive (store.ts: the boot opened from a hello, each snapshot with its bytes).
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { SnapshotAssembler, parseSnapshot, savePicture } from "./snapshot.ts";
import { Store } from "./store.ts";
import { loadMounting, type Mounting } from "./mounting.ts";
import { BootFolders } from "./boot_folders.ts";
import { WIRE_VERSION } from "./wire.ts";
import type { Boot } from "./boot.ts";

export const FIRMWARE_SNAPSHOTS = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../../build/test_snapshot.snap");
export const noFirmware = !existsSync(FIRMWARE_SNAPSHOTS) && "run ./run_tests.sh first";

// The firmware's snapshots, whole, as they come off the wire.
export const firmwareSnapshots = (): Buffer[] => new SnapshotAssembler().push(readFileSync(FIRMWARE_SNAPSHOTS));

// A store in directory (its own database file) with the firmware's snapshots, one a second
// from 14 Oct 2026 10:00 UTC, their pictures under directory; the boot opened with mounting.
export async function storeWithSnapshots(directory: string, mounting: Mounting = loadMounting()): Promise<{ store: Store; boot: Boot }> {
  const store = new Store(path.join(directory, "test.duckdb")), snapshots = firmwareSnapshots(), folders = new BootFolders(directory);
  const bootId = parseSnapshot(snapshots[0]).bootId;
  const boot = await store.openBoot({ version: WIRE_VERSION, bootId, build: "test" }, mounting, Date.UTC(2026, 9, 14, 10));
  for (const [i, bytes] of snapshots.entries()) {
    const s = parseSnapshot(bytes), ms = Date.UTC(2026, 9, 14, 10, 0, i);
    await store.importSnapshot(boot, boot.stamp(ms).messageNumber, s, bytes, ms, savePicture(directory, folders.folderForBoot(s.bootId, ms), s));
  }
  return { store, boot };
}
