import {test} from 'node:test';
import assert from 'node:assert/strict';
import {FrameParser, decodeTof} from './protocol.ts';
import {optCrc, type OptFrame} from './opt4048.ts';
import {selectTarget, zoneValue, sourceZone} from '../public/tof-view.js';

function tofPacket(zones: number) {
  const data = Buffer.alloc(16 + zones * 49);
  data.set([1, zones, 4, 10, 0xfb, 1, 1, 4]); // temperature -5
  data.writeUInt16LE(5, 8); data.writeUInt32LE(123, 12);
  for (let z = 0; z < zones; z++) {
    let p = 16 + z * 49;
    data[p++] = 2; data.writeUInt32LE(100, p); p += 4; data.writeUInt32LE(200, p); p += 4;
    for (let t = 0; t < 4; t++, p += 10) {
      data.writeInt16LE(t === 0 ? -10 : 1000 * t, p);
      data.writeUInt16LE(12, p + 2); data.writeUInt32LE(50 + t, p + 4);
      data[p + 8] = 30; data[p + 9] = t < 2 ? 5 : 255;
    }
  }
  const header = Buffer.alloc(8); header.write('TOF3'); header.writeUInt32LE(data.length, 4);
  return Buffer.concat([header, data]);
}
function cameraPacket() {
  const header = Buffer.from([72, 77, 48, 51, 160, 0, 120, 0]);
  const pixels = Buffer.alloc(160 * 120, 42);
  // Binary payload must never be mistaken for telemetry or log delimiters.
  pixels.write('TOF3\nHM03', 100);
  return Buffer.concat([header, pixels]);
}
for (const step of [1, 2, 3, 7, 64, 100000]) {
  test(`mixed camera / ToF / logs, chunks of ${step}`, () => {
    const frames: Buffer[] = [], tofs: ReturnType<typeof decodeTof>[] = [], logs: string[] = [];
    const parser = new FrameParser((w, h, b) => {
      assert.equal(w, 160); assert.equal(h, 120); frames.push(b);
    }, s => logs.push(s), t => tofs.push(t));
    const camera = cameraPacket();
    const stream = Buffer.concat([Buffer.from('Ready\n'), camera, tofPacket(64),
      Buffer.from('TOF running\n'), tofPacket(16), camera, Buffer.from('TOF stopped\n')]);
    for (let i = 0; i < stream.length; i += step) parser.push(stream.subarray(i, i + step));
    assert.equal(frames.length, 2); assert.deepEqual(frames[0], camera.subarray(8));
    assert.equal(tofs.length, 2); assert.equal(tofs[0].side, 8); assert.equal(tofs[1].side, 4);
    assert.equal(tofs[0].temperature, -5); assert.equal(tofs[0].sequence, 123);
    assert.equal(tofs[0].zones[0].targets[0].distance, -10);
    assert.equal(tofs[0].zones[63].targets[1].distance, 1000);
    assert.deepEqual(logs, ['Ready', 'TOF running', 'TOF stopped']);
  });
}
test('invalid lengths and invalid version recover to next packet', () => {
  let count = 0;
  const parser = new FrameParser(() => count++, () => {}, () => count++);
  const badHeader = Buffer.from('TOF3ffffffff', 'ascii');
  const badVersion = tofPacket(16); badVersion[8] = 99;
  parser.push(Buffer.concat([badHeader, badVersion, cameraPacket(), tofPacket(64)]));
  assert.equal(count, 2);
  assert.throws(() => decodeTof(Buffer.alloc(16)));
});
function optPacket(counts: number[], counter = 3, badCrc = false) {
  const p = Buffer.alloc(36); p.write('OPT3'); p.writeUInt32LE(28, 4);
  p[8] = 1; p.writeUInt16LE(0x0821, 10); p.writeUInt32LE(7, 12); p.writeUInt16LE(100, 16);
  counts.forEach((mantissa, i) => {
    const exponent = 2, o = 20 + 4*i;
    const crc = optCrc(mantissa, exponent, counter) ^ (badCrc && i === 0 ? 1 : 0);
    p.set([exponent << 4 | mantissa >> 16, (mantissa >> 8) & 255, mantissa & 255, counter << 4 | crc], o);
  });
  return p;
}
test('OPT4048 packets decode between camera / ToF packets and reject bad CRC', () => {
  const opts: OptFrame[] = [], logs: string[] = [];
  let frames = 0;
  const parser = new FrameParser(() => frames++, s => logs.push(s), () => frames++, o => opts.push(o));
  const stream = Buffer.concat([cameraPacket(), optPacket([1000, 20000, 500, 30000]), Buffer.from('log\n'),
    tofPacket(16), optPacket([1000, 20000, 500, 30000], 3, true)]);
  for (let i = 0; i < stream.length; i += 5) parser.push(stream.subarray(i, i + 5));
  assert.equal(frames, 2); assert.deepEqual(logs, ['log']); assert.equal(opts.length, 2);
  const [a, b] = opts;
  assert.equal(a.sequence, 7); assert.equal(a.conversionMs, 100); assert.ok(a.valid);
  assert.equal(a.channels[1].adc, 80000); assert.ok(Math.abs(a.lux! - 80000 * 0.00215) < 1e-9);
  assert.ok(a.chromaticity && a.chromaticity.x > 0 && a.chromaticity.y > 0);
  assert.equal(b.valid, false); assert.equal(b.lux, null); assert.equal(b.channels[0].crcOk, false);
});
test('target selection honors count, validity, distance and signal independently', () => {
  const a = {distance: 100, signal: 20, status: 5};
  const b = {distance: 300, signal: 80, status: 9};
  const c = {distance: 200, signal: 100, status: 5};
  const bad = {distance: 4000, signal: 1000, status: 255};
  const zone = {count: 4, ambient: 12, spads: 100, targets: [a, b, c, bad]};
  assert.equal(selectTarget(zone, 'closest', 'normal'), a);
  assert.equal(selectTarget(zone, 'farthest', 'normal'), b);
  assert.equal(selectTarget(zone, 'strongest', 'normal'), c);
  assert.equal(selectTarget(zone, 'farthest', 'strict'), c);
  assert.equal(selectTarget(zone, 'farthest', 'all'), bad);
  assert.equal(selectTarget({...zone, count: 0}, 'farthest', 'all'), null);
  assert.equal(selectTarget({...zone, count: 1}, '1', 'all'), null);
  assert.equal(zoneValue(zone, 'distance', '3', 'normal').value, null);
  assert.equal(zoneValue(zone, 'status', '3', 'normal').value, 255);
  assert.equal(zoneValue({...zone, count: 0}, 'ambient', 'closest', 'strict').value, 12);
});
test('view rotation preserves all zones', () => {
  for (const side of [4, 8]) for (const rotation of [0, 90, 180, 270]) {
    const indices = [];
    for (let r = 0; r < side; r++) for (let c = 0; c < side; c++) indices.push(sourceZone(r, c, side, rotation));
    assert.equal(new Set(indices).size, side * side);
    assert.equal(Math.min(...indices), 0); assert.equal(Math.max(...indices), side * side - 1);
  }
});
