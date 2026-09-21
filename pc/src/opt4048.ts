// TI OPT4048 SBOSA84 §§8.3.4.5 / 9.2.4. Channel ADC counts are NOT yet CIE XYZ:
// apply the example matrix before chromaticity. This is not board calibration.
// CRC parity uses sample COUNTER bits (as in Adafruit's OPT4048 implementation);
// the original TI PDF labels these inputs CRC, ambiguously/self-referentially.
function parity(n: number): number {
  n ^= n >>> 16; n ^= n >>> 8; n ^= n >>> 4; n ^= n >>> 2; n ^= n >>> 1;
  return n & 1;
}
export function optCrc(mantissa: number, exponent: number, counter: number) {
  const x0 = parity(mantissa) ^ parity(exponent) ^ parity(counter);
  const x1 = parity(mantissa & 0xaaaaa) ^ parity(exponent & 0xa) ^ parity(counter & 0xa);
  const x2 = parity(mantissa & 0x88888) ^ ((exponent >> 3) & 1) ^ ((counter >> 3) & 1);
  const x3 = parity(mantissa & 0x80808);
  return x0 | (x1 << 1) | (x2 << 2) | (x3 << 3);
}
export function decodeOpt(b: Buffer) {
  if (b.length !== 28 || b[0] !== 1 || b.readUInt16LE(2) !== 0x0821) throw new Error('Invalid OPT4048 packet');
  const channels = [];
  for (let i = 0; i < 4; ++i) {
    const p = 12 + 4*i, exponent = b[p] >> 4;
    const mantissa = ((b[p] & 15) << 16) | (b[p+1] << 8) | b[p+2];
    const counter = b[p+3] >> 4, crc = b[p+3] & 15;
    channels.push({adc: mantissa * 2 ** exponent, exponent, counter,
      crcOk: optCrc(mantissa, exponent, counter) === crc, rangeOk: exponent <= 8});
  }
  const overload = !!(b[1] & 1);
  const valid = !overload && channels.every(c => c.crcOk && c.rangeOk && c.counter === channels[0].counter);
  const [c0, c1, c2] = channels.map(c => c.adc);
  const X = c0 * 2.34892992e-4 + c1 * 4.07467441e-5 + c2 * 9.28619404e-5;
  const Y = c0 * -1.89652390e-5 + c1 * 1.98958202e-4 + c2 * -1.69739553e-5;
  const Z = c0 * 1.20811684e-5 + c1 * -1.58848115e-5 + c2 * 6.74021520e-4;
  const sum = X + Y + Z;
  const chromaticity = valid && X >= 0 && Y > 0 && Z >= 0 && sum > 0 ? {x: X / sum, y: Y / sum} : null;
  return {type: 'opt', sequence: b.readUInt32LE(4), conversionMs: b.readUInt16LE(8),
    channels, overload, valid, lux: valid ? c1 * 0.00215 : null,
    xyz: valid ? {X, Y, Z} : null, chromaticity};
}
export type OptFrame = ReturnType<typeof decodeOpt>;
