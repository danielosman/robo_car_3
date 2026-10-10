// Tests for recording.ts: records split wherever the chunks end and CRC errors found
// (files from before UDP); takes followed and cut; the firmware's own datagrams
// decoded (build/test_recorder.rec, written by ./run_tests.sh); the UDP server
// saving every datagram with its arrival time, leaving out repeats, counting lost
// ones and noting each take. Run with `npm test`.
import { test } from "node:test";
import assert from "node:assert/strict";
import dgram from "node:dgram";
import { existsSync, mkdtempSync, readFileSync, readdirSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import {
  FILE_MAGIC, REC, Dedup, RecordReader, RecordingServer, TakeTracker, encodeDatagram, encodeRecord, parseRecord,
  readRecordingFile, type Take,
} from "./recording.ts";

function takeStart(takeNo: number, version: number, key = "s", action = "scan"): Buffer {
  const p = Buffer.alloc(52);
  p.writeUInt8(version, 0);
  p.writeUInt32LE(0x7f3a5c01, 1);
  p.writeUInt16LE(takeNo, 5);
  p.writeUInt32LE(4294967000, 7); // just before the clock wraps
  p.write(key, 11, "latin1");
  p.write(action, 12, "latin1");
  p.writeFloatLE(6.8, 24);
  p.write("Oct 10 2026 12:00:00", 28, "latin1");
  return encodeRecord(REC.TAKE_START, p);
}

// Version 1-2: all 64 zones, u32 fields; zone 0 has one target at 450 mm.
function tofRawV1(frameNo: number, skipped: number): Buffer {
  const p = Buffer.alloc(11 + 64 * 9 + 10);
  p.writeUInt32LE(frameNo, 0);
  p.writeUInt16LE(skipped, 8);
  p.writeInt8(-3, 10);
  p.writeUInt8(1, 11);
  p.writeInt16LE(450, 20);
  return encodeRecord(REC.TOF_RAW, p);
}

// Version 3: half a frame (32 zones), compact; its first zone has one target.
function tofHalf(frameNo: number, firstZone: number): Buffer {
  const p = Buffer.alloc(13 + 32 * 5 + 8);
  p.writeUInt32LE(frameNo, 0);
  p.writeUInt8(firstZone, 11);
  p.writeUInt8(32, 12);
  p.writeUInt8(1, 13);
  p.writeUInt16LE(7, 14);  // ambient
  p.writeInt16LE(450, 18); // the target's distance
  p.writeUInt16LE(1234, 22); // signal
  p.writeUInt8(5, 25);     // status
  return encodeRecord(REC.TOF_RAW, p);
}

function odom(picoBUs: number): Buffer {
  const p = Buffer.alloc(53);
  p.writeUInt32LE(picoBUs + 1000, 0);
  p.writeUInt32LE(picoBUs, 4);
  return encodeRecord(REC.ODOM, p);
}

function takeEnd(version: number, frames: number, framesDropped: number, records: number): Buffer {
  const p = Buffer.alloc(version >= 3 ? 33 : 25);
  p.writeUInt32LE(14_000_000 - 296, 0); // 14 s after the start, across the wrap
  p.writeUInt32LE(records, 5);
  p.writeUInt32LE(frames, 13);
  p.writeUInt32LE(framesDropped, 17);
  return encodeRecord(REC.TAKE_END, p);
}

test("before UDP: records split wherever the chunks end; a CRC error is found", () => {
  const stream = Buffer.concat([takeStart(1, 1), tofRawV1(10, 0), tofRawV1(13, 2), takeEnd(1, 2, 2, 3)]);
  const reader = new RecordReader();
  const got = [];
  for (let i = 0; i < stream.length; i += 7) got.push(...reader.push(stream.subarray(i, i + 7)));
  assert.deepEqual(got.map(r => r.type), [1, 3, 3, 7]);
  assert.ok(got.every(r => r.crcOk) && reader.pending === 0);
  const start = parseRecord(got[0]);
  assert.ok(start.kind === "TAKE_START" && start.version === 1 && start.bootId === "7f3a5c01" && start.action === "scan");
  const frame = parseRecord(got[2], 1);
  assert.ok(frame.kind === "TOF_RAW" && frame.skipped === 2 && frame.tempC === -3 && frame.zones.length === 64);
  assert.equal(frame.zones[0].targets[0].distanceMm, 450);
  const bad = Buffer.from(stream);
  bad[59 + 3 + 20] ^= 1; // in the second record's payload
  assert.deepEqual(new RecordReader().push(bad).map(x => x.crcOk), [true, false, true, true]);

  const tracker = new TakeTracker(), ended: Take[] = [];
  tracker.onEnd = t => ended.push(t);
  for (const r of new RecordReader().push(Buffer.concat([stream, takeStart(2, 1, "a", "watch"), tofRawV1(20, 0)]))) tracker.add(r);
  tracker.finish();
  assert.match(ended[0].summary(), /^take 1 \(s scan, boot 7f3a5c01\): done; 14\.0 s; 2 ToF frames \(0\.1\/s\), 2 left out in 1 gaps, widest 2/);
  assert.match(ended[1].summary(), /^take 2 \(a watch, .*CUT \(no TAKE_END\)/);
});

test("datagrams: halves make a frame, repeats are left out, lost datagrams counted", () => {
  const d = (seq: number, ...records: Buffer[]) => encodeDatagram(3, 0x7f3a5c01, seq, records);
  const datagrams = [
    d(10, takeStart(1, 3), odom(100)), d(10, takeStart(1, 3), odom(100)), // sent twice
    d(11, tofHalf(5, 0), odom(120)), d(12, tofHalf(5, 32), odom(140), odom(120)), // 120 repeated
    // 13 lost: frame 6's first half
    d(14, tofHalf(6, 32), odom(180)),
    d(15, tofHalf(8, 0), tofHalf(8, 32)), // frame 7 never arrived
    d(16, takeEnd(3, 4, 0, 9), odom(180)),
  ];
  const file = Buffer.concat([Buffer.from(FILE_MAGIC), ...datagrams.map((g, i) => {
    const e = Buffer.alloc(10);
    e.writeUInt16LE(g.length, 0);
    e.writeDoubleLE(1000 + i, 2);
    return Buffer.concat([e, g]);
  })]);
  const tracker = new TakeTracker(), dedup = new Dedup(), ended: Take[] = [];
  tracker.onEnd = t => ended.push(t);
  let arrivals = 0;
  for (const { arrivalMs, datagram, records } of readRecordingFile(file)) {
    assert.ok(datagram && arrivalMs !== null && arrivalMs >= 1000);
    arrivals++;
    if (!dedup.fresh(datagram)) continue;
    for (const r of records) tracker.add(r, datagram.seq);
  }
  assert.equal(arrivals, 7);
  const take = ended[0];
  assert.equal(take.counts.ODOM, 4); // 100, 120, 140, 180: repeats left out
  assert.equal(tracker.outside, 0);
  const frame = parseRecord(new RecordReader().push(tofHalf(5, 32))[0]);
  assert.ok(frame.kind === "TOF_RAW" && frame.firstZone === 32 && frame.zones[0].zone === 32);
  assert.ok(frame.zones[0].ambientPerSpad === 7 && frame.zones[0].targets[0].signalPerSpad === 1234 && frame.zones[0].targets[0].status === 5);
  assert.match(take.summary(), /: done; 14\.0 s; 2 ToF frames \(0\.1\/s\), 1 half, 1 missing; 6 datagrams, 1 lost \(14\.3 %\); 4 ODOM/);
});

const repo = path.join(path.dirname(fileURLToPath(import.meta.url)), "../../..");
const sample = path.join(repo, "build/test_recorder.rec");

test("the firmware's own datagrams decode (build/test_recorder.rec)", { skip: !existsSync(sample) && "run ./run_tests.sh first" }, () => {
  const tracker = new TakeTracker(), dedup = new Dedup(), parsed = [];
  let repeats = 0;
  for (const { datagram, records } of readRecordingFile(readFileSync(sample))) {
    assert.ok(datagram && !datagram.broken && datagram.version === 3);
    if (!dedup.fresh(datagram)) { repeats++; continue; }
    for (const r of records) { assert.ok(r.crcOk); const p = tracker.add(r, datagram.seq); if (p) parsed.push(p); }
  }
  assert.equal(repeats, 3); // the first datagram twice, the last three times
  assert.deepEqual(parsed.slice(0, 4).map(r => r.kind), ["TAKE_START", "GEOMETRY", "ODOM", "WIFI"]);
  const [start, geometry, odomRec, wifi] = parsed;
  assert.ok(start.kind === "TAKE_START" && start.version === 3 && start.takeNo === 1 && start.key === "s" && start.action === "scan");
  assert.ok(geometry.kind === "GEOMETRY" && geometry.zoneOfRay[0] === 56 && Math.abs(geometry.sensorM[2] - 0.07) < 1e-6);
  assert.ok(odomRec.kind === "ODOM" && Math.abs(odomRec.gyroBias - 0.001) < 1e-9 && odomRec.tUs === odomRec.picoBUs + 1000);
  assert.ok(wifi.kind === "WIFI" && wifi.rssiDbm === -61 && wifi.console.retries === 2 && wifi.datagrams !== undefined);
  const halves = parsed.filter(r => r.kind === "TOF_RAW");
  assert.equal(halves.length, 4);
  const zones = halves.slice(2).flatMap(h => h.kind === "TOF_RAW" ? h.zones : []);
  assert.equal(zones.length, 64);
  assert.equal(zones.reduce((n, z) => n + z.targets.length, 0), 63);
  assert.deepEqual(zones[5].targets.map(t => t.distanceMm), [500, 510]);
  assert.ok(zones[1].ambientPerSpad === 65535 && zones[1].targets[0].signalPerSpad === 65535); // capped
  const marks = parsed.flatMap(r => r.kind === "MARK" ? [r.text] : []);
  assert.deepEqual(marks, ["Scan: turning 390 deg", "key p", "x +1.0 cm  y +2.0 cm"]);
  const end = parsed.at(-1)!;
  assert.ok(end.kind === "TAKE_END" && end.reason === "done" && end.frames === 2 && end.records === parsed.length - 1);
  assert.ok(end.datagramsFailed === 0 && tracker.outside === 0);
  assert.match(tracker.takes[0].summary(), /2 ToF frames .*, 0 missing; \d+ datagrams, 0 lost/);
});

test("the server saves every datagram with its arrival time, a file per boot, and notes each take", async () => {
  const dir = mkdtempSync(path.join(os.tmpdir(), "robocar-rec-"));
  const server = new RecordingServer({ port: 0, dir });
  const notes: string[] = [];
  server.on("note", n => notes.push(n));
  const robot = dgram.createSocket("udp4");
  const until = async (ok: () => boolean) => { for (let i = 0; i < 200 && !ok(); i++) await new Promise(r => setTimeout(r, 10)); assert.ok(ok()); };
  try {
    const port = await server.start();
    const send = (g: Buffer) => new Promise<void>(r => robot.send(g, port, "127.0.0.1", () => r()));
    const sent = [
      encodeDatagram(3, 0x7f3a5c01, 0, [takeStart(1, 3), odom(100)]),
      encodeDatagram(3, 0x7f3a5c01, 0, [takeStart(1, 3), odom(100)]),
      encodeDatagram(3, 0x7f3a5c01, 1, [tofHalf(5, 0)]),
      encodeDatagram(3, 0x7f3a5c01, 3, [tofHalf(5, 32), takeEnd(3, 1, 0, 3)]), // 2 lost
    ];
    for (const g of sent) await send(g);
    await until(() => notes.some(n => /^take 1 /.test(n)));
    server.consoleArrived(12);
    assert.ok(notes.some(n => /^recordings from boot 7f3a5c01: saving to raw\/.*-boot-7f3a5c01\.rec$/.test(n)));
    assert.ok(notes.includes("recording take 1 (s scan, boot 7f3a5c01)"));
    assert.ok(notes.some(n => /^take 1 .*: done; .* 1 ToF frames .*; 3 datagrams, 1 lost/.test(n)));
    // Another boot: another file.
    await send(encodeDatagram(3, 0x11111111, 0, [takeStart(1, 3)]));
    await until(() => notes.some(n => n.startsWith("recordings from boot 11111111")));
    server.stop();
    await new Promise(r => setTimeout(r, 50));
    const files = readdirSync(path.join(dir, "raw")).sort();
    assert.equal(files.filter(f => f.endsWith(".rec")).length, 2);
    const recFile = files.find(f => f.endsWith("-boot-7f3a5c01.rec"))!;
    const saved = [...readRecordingFile(readFileSync(path.join(dir, "raw", recFile)))];
    assert.equal(saved.length, 4); // the repeat too: the file is what arrived
    assert.ok(saved.every((s, i) => s.datagram && s.arrivalMs! > 0 && s.datagram.seq === [0, 0, 1, 3][i]));
    const arrivals = readFileSync(path.join(dir, "raw", recFile.replace(/\.rec$/, ".arrivals.csv")), "utf8");
    assert.match(arrivals, /^ms,connection,bytes\n\d+,console,12\n$/);
  } finally {
    robot.close();
    try { server.stop(); } catch { /* stopped already */ }
    rmSync(dir, { recursive: true, force: true });
  }
});
