// The robot names each power-up (a boot) with a random 8-hex boot id (picoA/app/
// recorder.c); it has no clock. Our folders under recordings/ (camera/<folder>,
// map/<folder>) are named instead by when the PC first saw the boot: YYYYMMDDHHMM,
// local time. recordings/boots.json maps each boot id from the robot to its folder
// name: {"7a587226": "202610142035"}. The database keeps the boot id.
//
// Read once at start. Everything that arrives belongs to the boot seen last, so its
// folder is remembered; only a different boot id looks the map up, and only a boot
// id never seen gets a new folder name (and boots.json is written).
import { existsSync, readFileSync, writeFileSync } from "node:fs";
import path from "node:path";

export const BOOTS_FILE = "boots.json";
const FOLDER_KINDS = ["camera", "map"];  // the folders under recordings/ named per boot

// YYYYMMDDHHMM in local time: the folder name of a boot first seen at ms.
export function folderNameAt(ms: number): string {
  const d = new Date(ms), two = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}${two(d.getMonth() + 1)}${two(d.getDate())}${two(d.getHours())}${two(d.getMinutes())}`;
}

export function loadBootFolders(file: string): Record<string, string> {
  try { return JSON.parse(readFileSync(file, "utf8")) as Record<string, string>; }
  catch (err) {
    if ((err as NodeJS.ErrnoException).code === "ENOENT") return {};
    throw new Error(`${file} doesn't read: ${(err as Error).message}`, { cause: err });
  }
}

export function saveBootFolders(file: string, folders: Record<string, string>): void {
  const sorted = Object.fromEntries(Object.entries(folders).sort(([, a], [, b]) => a.localeCompare(b)));
  writeFileSync(file, `${JSON.stringify(sorted, null, 2)}\n`);
}

export class BootFolders {
  private readonly recordingsDir: string;
  private readonly file: string;
  private readonly folders: Record<string, string>;
  private current = { bootId: "", folder: "" };

  constructor(recordingsDir: string) {
    this.recordingsDir = recordingsDir;
    this.file = path.join(recordingsDir, BOOTS_FILE);
    this.folders = loadBootFolders(this.file);
  }

  // The folder name for the robot's boot id; a boot never seen before is named by
  // firstSeenMs (when its first snapshot or recording arrived).
  folderForBoot(bootId: string, firstSeenMs = Date.now()): string {
    if (bootId !== this.current.bootId) {
      this.current = { bootId, folder: this.folders[bootId] ?? this.nameNewBoot(bootId, firstSeenMs) };
    }
    return this.current.folder;
  }

  // The folder name of a boot already in boots.json, null if it has none (for reads:
  // never names a boot).
  knownFolder(bootId: string): string | null { return this.folders[bootId] ?? null; }

  // Gives a new boot its folder name and saves it in boots.json. Two boots in the same
  // minute would share a name: an error (never expected).
  private nameNewBoot(bootId: string, firstSeenMs: number): string {
    const folder = folderNameAt(firstSeenMs);
    const owner = Object.keys(this.folders).find(id => this.folders[id] === folder);
    if (owner) throw new Error(`boot ${bootId} would get folder ${folder}, already boot ${owner}'s (${BOOTS_FILE})`);
    for (const kind of FOLDER_KINDS) {
      if (existsSync(path.join(this.recordingsDir, kind, folder))) throw new Error(`boot ${bootId}: folder ${kind}/${folder} already exists`);
    }
    this.folders[bootId] = folder;
    saveBootFolders(this.file, this.folders);
    return folder;
  }
}
