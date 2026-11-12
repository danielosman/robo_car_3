// One Kalman step on small matrices (doc/OUTLINE_MAP_PLAN.md, idea 3): the one place the
// filter's arithmetic lives. pose_check.ts says what the state and the measurements are.
//   kalmanStep(P, G, z, R)   a state with covariance P (its prior change is 0) seen as
//                            z = G·change + noise R: the change, and the covariance after
export type Matrix = number[][];

const mul = (a: Matrix, b: Matrix): Matrix => a.map(row => b[0].map((_, c) => row.reduce((m, v, k) => m + v * b[k][c], 0)));
const transposed = (a: Matrix): Matrix => a[0].map((_, c) => a.map(row => row[c]));

// Gauss-Jordan with partial pivoting; the matrices here are a few rows, symmetric and positive.
function inverse(a: Matrix): Matrix {
  const n = a.length, m = a.map((row, i) => [...row, ...row.map((_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
    [m[c], m[p]] = [m[p], m[c]];
    const d = m[c][c];
    for (let k = 0; k < 2 * n; k++) m[c][k] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = m[r][c];
      for (let k = 0; k < 2 * n; k++) m[r][k] -= f * m[c][k];
    }
  }
  return m.map(row => row.slice(n));
}

export function kalmanStep(P: Matrix, G: Matrix, z: number[], R: Matrix): { change: number[]; cov: Matrix } {
  const PGt = mul(P, transposed(G)), S = mul(G, PGt).map((row, i) => row.map((v, j) => v + R[i][j]));
  const K = mul(PGt, inverse(S));
  const change = K.map(row => row.reduce((m, k, i) => m + k * z[i], 0));
  const KGP = mul(K, mul(G, P));
  const cov = P.map((row, i) => row.map((v, j) => v - KGP[i][j]));
  // Keep it symmetric against rounding.
  return { change, cov: cov.map((row, i) => row.map((v, j) => (v + cov[j][i]) / 2)) };
}
