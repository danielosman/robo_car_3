// `robot frames <boot> <move>` and `robot imu <boot> <move>` (doc/DATA_FLOW_PLAN.md,
// debugging without the robot): a move's posed frames, or its imu records, as the store
// has them, as text for a person, or as JSON for a tool (--json). Pure formatting; the
// reading is in reads.ts.
import type { ImuCount, MoveRow, StoredFrame, StoredImu } from "../store.ts";
import { IMU_ACCEL_G_PER_COUNT, IMU_GYRO_DPS_PER_COUNT, IMU_RECORD_PERIOD_US } from "../wire.ts";
import { TOF_COLS, TOF_ROWS, isBeyond, type Kind } from "../tof_format.ts";

const DEG = 180 / Math.PI;
const r1 = (v: number) => Math.round(v * 10) / 10;
const r2 = (v: number) => Math.round(v * 100) / 100;

// One character after a distance says how sure the sensor was.
const MARK: Record<Kind, string> = { sure: "", less_sure: "~", unsure: "?", nothing: "" };
export const FRAMES_LEGEND = "distance cm (~ less sure, ? unsure: tells nothing, - nothing in range, >254 beyond 2.54 m); " +
  "signal kcps per SPAD (capped at 255); sigma mm (capped at 255). Row 1 is the top row, column 1 the left (as the robot sees).";

// The move's imu records: how many are stored (and the time they cover at 104 a
// second), how many are missing between them (gaps in their times), its batches stored
// of those the robot sent, and dropped there.
function imuLine(m: MoveRow, imu: ImuCount): string {
  const s = r1(imu.records * IMU_RECORD_PERIOD_US / 1e6);
  return `imu: ${imu.records} records stored (${s} s at 104 a second), ${imu.missing} missing (gaps)` + (m.end === null ? "" :
    `, ${imu.batches} of ${m.imuSent ?? 0} batches; ${m.imuDropped ?? 0} dropped on the robot; ` +
    `${Math.max(0, (m.imuSent ?? 0) - imu.batches)} lost (dropped connection)`);
}

function moveLine(boot: string, folder: string | null, m: MoveRow, stored: number, imu: ImuCount): string {
  const param = m.kind === "rotate" ? `${r1((m.param ?? 0) * DEG)}°` : m.kind === "watch" ? `${m.param} s` : `${Math.round((m.param ?? 0) * 100)} cm`;
  const head = `boot ${boot}${folder ? ` (${folder})` : ""}, move ${m.move}: ${m.kind ?? "?"} ${param}, command ${m.command ?? "?"}, ` +
    `still ${m.preMs ?? "?"} ms before, ${m.postMs ?? "?"} ms after`;
  const end = m.end === null ? `not ended (yet); ${stored} frames stored`
    : `${m.refused ? "refused (didn't start), " : ""}ended ${m.end} after ${r1((m.endTS ?? 0) - (m.startTS ?? 0))} s; ` +
      `${stored} stored of ${m.framesSent} sent; ${m.framesDropped} dropped on the robot; ` +
      `${Math.max(0, (m.framesSent ?? 0) - stored)} lost (dropped connection)` +
      `; moved ${Math.round((m.moved ?? 0) * 100)} cm, turned ${r1((m.rotated ?? 0) * DEG)}°`;
  return `${head}\n${end}\n${imuLine(m, imu)}`;
}

// The frame's 64 zones as 8 rows of `cell(zone)`.
function matrix<T>(f: StoredFrame, cell: (k: number) => T): T[][] {
  return Array.from({ length: TOF_ROWS }, (_, r) => Array.from({ length: TOF_COLS }, (__, c) => cell(r * TOF_COLS + c)));
}

export function framesText(boot: string, folder: string | null, move: MoveRow, frames: StoredFrame[], imu: ImuCount): string {
  const out = [moveLine(boot, folder, move, frames.length, imu), FRAMES_LEGEND];
  for (const s of frames) {
    const f = s.frame;
    out.push("", `frame ${f.number}  t ${s.tS.toFixed(3)} s  period ${r1(f.periodUs / 1000)} ms  ` +
      (f.poseKnown ? `x ${r1(f.x * 100)} cm  y ${r1(f.y * 100)} cm  yaw ${r2(f.yaw * DEG)}°  pitch ${r2(f.pitch * DEG)}°  turn ${r2(f.turn * DEG)}°`
                   : "no pose (PicoB not connected)") + (f.still ? "  still" : ""));
    const distance = matrix(s, k => {
      const z = f.zones[k];
      if (!z || z.kind === "nothing") return "-";
      return isBeyond(f, z) ? ">254" : `${Math.round(z.distanceMm / 10)}${MARK[z.kind]}`;
    });
    const signal = matrix(s, k => String(f.zones[k]?.signal ?? ""));
    const sigma = matrix(s, k => String(f.zones[k]?.sigmaMm ?? ""));
    const width = Math.max(...[...distance, ...signal, ...sigma].flat().map(c => c.length));
    out.push(`  ${"distance".padEnd(TOF_COLS * (width + 1))}  ${"signal".padEnd(TOF_COLS * (width + 1))}  sigma`);
    for (let r = 0; r < TOF_ROWS; r++) {
      const row = (m: string[][]) => m[r].map(c => c.padStart(width)).join(" ");
      out.push(`  ${row(distance)}   ${row(signal)}   ${row(sigma)}`);
    }
  }
  return `${out.join("\n")}\n`;
}

export function framesJson(boot: string, folder: string | null, move: MoveRow, frames: StoredFrame[], imu: ImuCount) {
  return {
    boot, folder, frames_stored: frames.length, frames_lost: move.framesSent === null ? null : Math.max(0, move.framesSent - frames.length),
    imu_records_stored: imu.records, imu_records_missing: imu.missing, imu_batches_stored: imu.batches,
    move: { ...move, units: "param: drive and goto m, rotate rad, watch s; moved m, rotated rad; times s since power-up" },
    legend: "per frame 8 x 8 matrices, row 1 first, column 1 first; distance_mm (kind nothing: 0; beyond_mm or more: at least that far)",
    frames: frames.map(s => {
      const f = s.frame;
      return {
        number: f.number, t_s: s.tS, period_ms: f.periodUs / 1000, pose_known: f.poseKnown, still: f.still,
        x_m: f.x, y_m: f.y, yaw_rad: f.yaw, pitch_rad: f.pitch, turn_rad: f.turn, beyond_mm: f.beyondMm,
        distance_mm: matrix(s, k => f.zones[k].distanceMm), kind: matrix(s, k => f.zones[k].kind),
        signal: matrix(s, k => f.zones[k].signal), sigma_mm: matrix(s, k => f.zones[k].sigmaMm), targets: matrix(s, k => f.zones[k].targets),
      };
    }),
  };
}

export const IMU_LEGEND = `ax, ay, az (accel) and gx, gy, gz (gyro): the IMU's raw counts in its own axes, 4 samples averaged, nothing taken out ` +
  `(accel ${IMU_ACCEL_G_PER_COUNT * 1000} mg a count, 16384 is 1 g; gyro ${IMU_GYRO_DPS_PER_COUNT * 1000} mdps a count; ` +
  "the boot's mounting, imu.robot_from_sensor, turns them into the robot's axes).";

export function imuText(boot: string, folder: string | null, move: MoveRow, frames: number, imu: ImuCount, records: StoredImu[]): string {
  const row = (t: string, values: (string | number)[]) => `${t.padStart(10)} ${values.map(v => String(v).padStart(7)).join("")}`;
  const out = [moveLine(boot, folder, move, frames, imu), IMU_LEGEND, "", row("t s", ["ax", "ay", "az", "gx", "gy", "gz"])];
  for (const r of records) out.push(row(r.tS.toFixed(4), [...r.accel, ...r.gyro]));
  return `${out.join("\n")}\n`;
}

export function imuJson(boot: string, folder: string | null, move: MoveRow, imu: ImuCount, records: StoredImu[]) {
  return {
    boot, folder, imu_records_stored: imu.records, imu_records_missing: imu.missing, imu_batches_stored: imu.batches,
    move: { ...move, units: "param: drive and goto m, rotate rad, watch s; moved m, rotated rad; times s since power-up" },
    legend: IMU_LEGEND, records: records.map(r => ({ t_s: r.tS, t_us: r.tUs, accel: r.accel, gyro: r.gyro })),
  };
}
