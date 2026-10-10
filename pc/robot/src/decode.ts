// Reads recordings (.rec, as the robot sent them: datagrams, or one TCP stream for
// files from before UDP) and prints each take: how it ended, frames, what was lost,
// the rate; --records prints every record too.
//   npm run decode -- recordings/raw/<file>.rec [--records]
import { readFileSync } from "node:fs";
import { Dedup, TakeTracker, readRecordingFile, type Parsed } from "./recording.ts";

function line(r: Parsed): string {
  switch (r.kind) {
    case "TAKE_START": return `TAKE_START take ${r.takeNo} key '${r.key}' ${r.action} ${r.param.toFixed(3)} boot ${r.bootId} build ${r.build} v${r.version}`;
    case "GEOMETRY": return `GEOMETRY sensor ${r.sensorM.map(v => v.toFixed(3)).join(", ")} m, zone ${(r.zoneRad * 180 / Math.PI).toFixed(3)} deg, ${r.rows}x${r.cols}`;
    case "TOF_RAW": {
      const targets = r.zones.reduce((n, z) => n + z.targets.length, 0);
      return `TOF_RAW frame ${r.frameNo} zones ${r.firstZone}-${r.firstZone + r.zones.length - 1}${r.skipped ? ` (${r.skipped} not sent before)` : ""}, ${targets} targets, ${r.tempC} C`;
    }
    case "ODOM": return `ODOM x ${(r.x * 100).toFixed(1)} cm, y ${(r.y * 100).toFixed(1)} cm, yaw ${(r.yaw * 180 / Math.PI).toFixed(1)} deg${r.stationary ? ", still" : ""}${r.motorsOn ? ", motors on" : ""}`;
    case "DRIVE": return `DRIVE v ${r.v.toFixed(3)} m/s, w ${(r.w * 180 / Math.PI).toFixed(1)} deg/s`;
    case "MARK": return `MARK ${r.text}`;
    case "WIFI": {
      const c = (s: NonNullable<typeof r.data>) => `waiting ${s.waiting}, unacked ${s.unacked}, retries ${s.retries}, rto ${s.rtoMs} ms, cwnd ${s.cwnd}, window ${s.peerWindow}, errors ${s.writeErrors}/${s.outputErrors}`;
      return `WIFI ${r.rssiDbm} dBm; console: ${c(r.console)}, silent ${r.consoleSilentMs} ms; ` +
        (r.data ? `rec: ${c(r.data)}` : `datagrams ${r.datagrams}, refused ${r.datagramsFailed}`);
    }
    case "TAKE_END": return `TAKE_END ${r.reason}: ${r.records} records, ${r.frames} frames, ${r.framesDropped} frames and ${r.recordsDropped} records not sent` +
      (r.datagrams !== undefined ? `; ${r.datagrams} datagrams, ${r.datagramsFailed} refused` : "");
    default: return `UNKNOWN type ${r.type}`;
  }
}

const files = process.argv.slice(2).filter(a => !a.startsWith("--"));
const all = process.argv.includes("--records");
if (!files.length) { console.error("usage: npm run decode -- <file.rec>... [--records]"); process.exit(2); }
for (const file of files) {
  const data = readFileSync(file);
  const tracker = new TakeTracker(), dedup = new Dedup();
  let crcErrors = 0, datagrams = 0, repeats = 0, broken = 0;
  console.log(`${file}: ${(data.length / 1024).toFixed(0)} KB`);
  const ended: string[] = [];
  tracker.onEnd = t => ended.push("  " + t.summary());
  try {
    for (const { datagram, records } of readRecordingFile(data)) {
      if (datagram) {
        datagrams++;
        if (datagram.broken) broken++;
        if (!dedup.fresh(datagram)) { repeats++; continue; }
      }
      for (const raw of records) {
        if (!raw.crcOk) crcErrors++;
        const r = tracker.add(raw, datagram?.seq);
        if (all && r) {
          const t = "tUs" in r ? String(r.tUs).padStart(10) : "".padStart(10);
          console.log(`    ${t}  ${raw.crcOk ? "" : "CRC ERROR "}${line(r)}`);
        }
        for (const s of ended.splice(0)) console.log(s);
      }
    }
  } catch (e) { console.log(`  ${(e as Error).message}`); }
  tracker.finish();
  for (const s of ended.splice(0)) console.log(s);
  if (datagrams) console.log(`  ${datagrams} datagrams received (${repeats} repeats)` + (broken ? `, ${broken} with a record cut off` : ""));
  if (tracker.outside) console.log(`  ${tracker.outside} records outside any take`);
  if (crcErrors) console.log(`  ${crcErrors} CRC errors`);
}
