// Pure display/selection helpers, also exercised by Node tests.
export const statusNames = {
  0: 'Not updated', 1: 'SPAD signal too low', 2: 'Target phase', 3: 'Sigma too high',
  4: 'Target consistency failed', 5: 'Valid', 6: 'Wraparound not checked',
  7: 'Rate consistency failed', 8: 'Target signal too low', 9: 'Valid, large pulse / possible merged target',
  10: 'Valid, absent in previous range', 11: 'Measurement consistency failed',
  12: 'Blurred by sharpener', 13: 'Inconsistent secondary target', 255: 'No target',
};
export function eligible(target, validity) {
  return validity === 'all' || (validity === 'strict' ? target.status === 5 : [5, 6, 9].includes(target.status));
}
export function selectTarget(zone, selection, validity) {
  const targets = zone.targets.slice(0, Math.min(zone.count, zone.targets.length));
  if (/^[0-3]$/.test(selection)) return targets[Number(selection)] ?? null;
  const valid = targets.filter(t => eligible(t, validity));
  if (!valid.length) return null;
  return valid.reduce((a, b) => {
    if (selection === 'farthest') return b.distance > a.distance ? b : a;
    if (selection === 'strongest') return b.signal > a.signal ? b : a;
    return b.distance < a.distance ? b : a;
  });
}
export function zoneValue(zone, metric, selection, validity) {
  if (['ambient', 'count', 'spads'].includes(metric)) return {value: zone[metric], target: null};
  const target = selectTarget(zone, selection, validity);
  // Raw status stays inspectable for explicitly selected slots, even when invalid.
  if (!target || (metric !== 'status' && !eligible(target, validity))) return {value: null, target};
  return {value: target[metric], target};
}
export function sourceZone(row, col, side, rotation) {
  if (rotation === 90) return (side - 1 - col) * side + row;
  if (rotation === 180) return (side - 1 - row) * side + side - 1 - col;
  if (rotation === 270) return col * side + side - 1 - row;
  return row * side + col;
}
