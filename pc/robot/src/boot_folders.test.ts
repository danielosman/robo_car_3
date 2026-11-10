// Tests for boot_folders.ts: the robot's boot ids to our folder names. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { BootFolders, folderNameAt } from "./boot_folders.ts";

const at = (y: number, mo: number, d: number, h: number, mi: number) => new Date(y, mo - 1, d, h, mi, 30).getTime();

function inTemporaryDirectory(body: (directory: string) => void) {
  const directory = mkdtempSync(path.join(os.tmpdir(), "boots-"));
  try { body(directory); } finally { rmSync(directory, { recursive: true, force: true }); }
}

test("a folder name is the local minute: YYYYMMDDHHMM", () => {
  assert.equal(folderNameAt(at(2026, 10, 14, 20, 35)), "202610142035");
  assert.equal(folderNameAt(at(2026, 1, 2, 3, 4)), "202601020304");
});

test("a known boot keeps its folder; a new boot is named when first seen and saved", () => {
  inTemporaryDirectory(directory => {
    writeFileSync(path.join(directory, "boots.json"), JSON.stringify({ "7a587226": "202610142035" }));
    const folders = new BootFolders(directory);
    assert.equal(folders.folderForBoot("7a587226", at(2026, 10, 18, 9, 0)), "202610142035");
    assert.equal(folders.folderForBoot("0badf00d", at(2026, 10, 18, 9, 0)), "202610180900");
    assert.equal(folders.folderForBoot("0badf00d", at(2026, 10, 18, 11, 0)), "202610180900", "named once");
    assert.equal(folders.folderForBoot("7a587226"), "202610142035", "back to an older boot");
    assert.deepEqual(JSON.parse(readFileSync(path.join(directory, "boots.json"), "utf8")), { "7a587226": "202610142035", "0badf00d": "202610180900" });
    assert.equal(new BootFolders(directory).folderForBoot("0badf00d"), "202610180900", "read again at the next start");
  });
});

test("a new boot whose folder name is taken: an error", () => {
  inTemporaryDirectory(directory => {
    const folders = new BootFolders(directory);
    folders.folderForBoot("11111111", at(2026, 10, 18, 9, 0));
    assert.throws(() => folders.folderForBoot("22222222", at(2026, 10, 18, 9, 0)), /already boot 11111111's/);
    mkdirSync(path.join(directory, "camera", "202610181000"), { recursive: true });
    assert.throws(() => folders.folderForBoot("33333333", at(2026, 10, 18, 10, 0)), /camera\/202610181000 already exists/);
  });
});
