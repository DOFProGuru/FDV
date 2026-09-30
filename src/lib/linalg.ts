/** 3x3 linear algebra (row-major) plus a weighted orthogonal Procrustes fit. */

export type Mat3 = [number, number, number, number, number, number, number, number, number];

export const ident = (): Mat3 => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const zeros = (): Mat3 => [0, 0, 0, 0, 0, 0, 0, 0, 0];

export function matMul(A: Mat3, B: Mat3): Mat3 {
  const C = zeros() as unknown as number[];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  }
  return C as unknown as Mat3;
}

export function matVec(A: Mat3, v: [number, number, number]): [number, number, number] {
  return [
    A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
    A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
    A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
  ];
}

export const transpose = (A: Mat3): Mat3 => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];

export function det(A: Mat3): number {
  return (
    A[0] * (A[4] * A[8] - A[5] * A[7]) -
    A[1] * (A[3] * A[8] - A[5] * A[6]) +
    A[2] * (A[3] * A[7] - A[4] * A[6])
  );
}

/** Symmetric 3x3 eigendecomposition by cyclic Jacobi. Returns ascending-order eigenvalues and V. */
export function jacobiEigen(Ain: Mat3): { values: [number, number, number]; vectors: Mat3 } {
  let A = [...Ain] as number[];
  const V = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let sweep = 0; sweep < 64; sweep++) {
    let off = 0;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) if (i !== j) off += A[i * 3 + j] * A[i * 3 + j];
    if (off < 1e-24) break;
    for (let p = 0; p < 2; p++) {
      for (let q = p + 1; q < 3; q++) {
        const apq = A[p * 3 + q];
        if (Math.abs(apq) < 1e-18) continue;
        const app = A[p * 3 + p];
        const aqq = A[q * 3 + q];
        const theta = ((aqq - app) / (2 * apq)) || 1e18;
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = A[k * 3 + p], akq = A[k * 3 + q];
          A[k * 3 + p] = c * akp - s * akq;
          A[k * 3 + q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = A[p * 3 + k], aqk = A[q * 3 + k];
          A[p * 3 + k] = c * apk - s * aqk;
          A[q * 3 + k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = V[k * 3 + p], vkq = V[k * 3 + q];
          V[k * 3 + p] = c * vkp - s * vkq;
          V[k * 3 + q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const values: [number, number, number] = [A[0], A[4], A[8]];
  const order = [0, 1, 2].sort((a, b) => values[a] - values[b]);
  const vectors = zeros() as unknown as number[];
  order.forEach((src, dst) => {
    for (let k = 0; k < 3; k++) vectors[k * 3 + dst] = V[k * 3 + src];
  });
  return { values: order.map((i) => values[i]) as [number, number, number], vectors: vectors as unknown as Mat3 };
}

/**
 * Solve min_R,t sum_i w_i |R x_i + t - y_i|^2 over rotations R (weighted orthogonal Procrustes).
 * Closed form via the cross-covariance SVD; the reflection case is removed by flipping the
 * smallest singular vector, exactly as Kabsch/Wahba prescribe.
 */
export function procrustesWeighted(
  xs: [number, number, number][],
  ys: [number, number, number][],
  ws: number[],
): { R: Mat3; t: [number, number, number]; rms: number } {
  let n = 0;
  const cb: [number, number, number] = [0, 0, 0];
  const cg: [number, number, number] = [0, 0, 0];
  for (let i = 0; i < xs.length; i++) {
    const w = ws[i];
    n += w;
    for (let k = 0; k < 3; k++) { cb[k] += w * xs[i][k]; cg[k] += w * ys[i][k]; }
  }
  if (!(n > 0)) return { R: ident(), t: [0, 0, 0], rms: NaN };
  for (let k = 0; k < 3; k++) { cb[k] /= n; cg[k] /= n; }

  const H = zeros() as unknown as number[];
  for (let i = 0; i < xs.length; i++) {
    const w = ws[i];
    const x0 = xs[i][0] - cb[0], x1 = xs[i][1] - cb[1], x2 = xs[i][2] - cb[2];
    const y0 = ys[i][0] - cg[0], y1 = ys[i][1] - cg[1], y2 = ys[i][2] - cg[2];
    const x = [x0, x1, x2], y = [y0, y1, y2];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) H[r * 3 + c] += w * x[r] * y[c];
  }
  // Eigen-decompose H'H to get the right singular vectors, then recover U from HV.
  const Ht = transpose(H as unknown as Mat3);
  const HtH = matMul(Ht, H as unknown as Mat3);
  const { values, vectors: V } = jacobiEigen(HtH);
  const sigma = values.map((v) => Math.sqrt(Math.max(0, v))) as [number, number, number];
  const Rv = zeros() as unknown as number[];
  for (let j = 0; j < 3; j++) {
    const vj: [number, number, number] = [V[j], V[3 + j], V[6 + j]];
    const Hv = matVec(H as unknown as Mat3, vj);
    const s = sigma[j];
    const u: [number, number, number] = s > 1e-9 ? [Hv[0] / s, Hv[1] / s, Hv[2] / s] : [0, 0, 0];
    for (let i = 0; i < 3; i++) Rv[i * 3 + j] = u[i];
  }
  // R = V * U^T
  let R = matMul(V, transpose(Rv as unknown as Mat3));
  if (det(R) < 0) {
    // Flip the singular direction belonging to the smallest singular value.
    let jMin = 0;
    for (let j = 1; j < 3; j++) if (sigma[j] < sigma[jMin]) jMin = j;
    const Vfix = [...V] as number[];
    for (let i = 0; i < 3; i++) Vfix[i * 3 + jMin] = -Vfix[i * 3 + jMin];
    R = matMul(Vfix as unknown as Mat3, transpose(Rv as unknown as Mat3));
  }
  const t: [number, number, number] = [...cg] as [number, number, number];
  const Rt = matVec(R, cb);
  for (let k = 0; k < 3; k++) t[k] -= Rt[k];

  let se = 0, sw = 0;
  for (let i = 0; i < xs.length; i++) {
    const p = matVec(R, xs[i]);
    const dx = p[0] + t[0] - ys[i][0], dy = p[1] + t[1] - ys[i][1], dz = p[2] + t[2] - ys[i][2];
    se += ws[i] * (dx * dx + dy * dy + dz * dz);
    sw += ws[i];
  }
  return { R, t, rms: sw > 0 ? Math.sqrt(se / sw) : NaN };
}

/** 3x3 covariance utilities (symmetric storage kept explicit for readability). */
export function covSymAddOuter(P: number[], v: number[], scale: number): number[] {
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) P[i * 3 + j] += scale * v[i] * v[j];
  return P;
}

export function symMat3Mul(P: Mat3, F: Mat3): Mat3 {
  // F * P * F^T
  const FP = matMul(F, P as unknown as Mat3);
  return matMul(FP, transpose(F));
}

/**
 * Weighted yaw-only registration about the Up axis.
 *
 * Both sensors are "up"-referenced: the Blue Raven builds down-range/cross-range from the rail
 * heading it observed at launch, while GPS gives true East/North. So the unknown between the two
 * frames is a single rotation about Up plus a translation - not a full 3x3. Fitting three angles
 * instead of one is not merely wasteful: it lets the fit tilt the vertical axis, which converts a
 * plain altitude offset into a bogus rotation of the whole trajectory.
 *
 * Least squares over a rotation-only 2D fit has the closed form theta = atan2(b, a); the smallest
 * singular direction is reported so an ill-conditioned (near-vertical, windless) flight can be
 * flagged instead of silently given a made-up rail heading.
 */
export function yawFitWeighted(
  xs: [number, number, number][],
  ys: [number, number, number][],
  ws: number[],
  sigmaFt = 0,
): { R: Mat3; t: [number, number, number]; rms: number; rmsHoriz: number; rmsVert: number; yawDeg: number; yawSigmaDeg: number; horizSpreadFt: number } {
  let n = 0;
  const cx: [number, number, number] = [0, 0, 0];
  const cy: [number, number, number] = [0, 0, 0];
  for (let i = 0; i < xs.length; i++) {
    n += ws[i];
    for (let k = 0; k < 3; k++) { cx[k] += ws[i] * xs[i][k]; cy[k] += ws[i] * ys[i][k]; }
  }
  if (!(n > 0)) {
    return { R: ident(), t: [0, 0, 0], rms: NaN, rmsHoriz: NaN, rmsVert: NaN, yawDeg: 0, yawSigmaDeg: Infinity, horizSpreadFt: 0 };
  }
  for (let k = 0; k < 3; k++) { cx[k] /= n; cy[k] /= n; }

  let a = 0, b = 0, vv = 0;
  for (let i = 0; i < xs.length; i++) {
    const w = ws[i];
    const xe = xs[i][0] - cx[0], xn = xs[i][1] - cx[1];
    const ye = ys[i][0] - cy[0], yn = ys[i][1] - cy[1];
    a += w * (xe * ye + xn * yn);
    b += w * (xe * yn - xn * ye);
    vv += w * (xe * xe + xn * xn);
  }
  const yaw = Math.atan2(b, a);
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const R: Mat3 = [c, -s, 0, s, c, 0, 0, 0, 1];
  const t: [number, number, number] = [cy[0] - (c * cx[0] - s * cx[1]), cy[1] - (s * cx[0] + c * cx[1]), cy[2] - cx[2]];

  let se = 0, seH = 0, seV = 0, sw = 0, spread = 0;
  for (let i = 0; i < xs.length; i++) {
    const w = ws[i];
    const pe = c * xs[i][0] - s * xs[i][1] + t[0];
    const pn = s * xs[i][0] + c * xs[i][1] + t[1];
    const pu = xs[i][2] + t[2];
    const de = pe - ys[i][0], dn = pn - ys[i][1], du = pu - ys[i][2];
    se += w * (de * de + dn * dn + du * du);
    seH += w * (de * de + dn * dn);
    seV += w * du * du;
    sw += w;
    const xe = xs[i][0] - cx[0], xn = xs[i][1] - cx[1];
    spread += w * (xe * xe + xn * xn);
  }
  const rms = sw > 0 ? Math.sqrt(se / sw) : NaN;
  const rmsHoriz = sw > 0 ? Math.sqrt(seH / sw) : NaN;
  const rmsVert = sw > 0 ? Math.sqrt(seV / sw) : NaN;
  const horizSpreadFt = sw > 0 ? Math.sqrt(spread / sw) : 0;
  // A rotation is only as well-determined as the lever arm is long: sigma_theta ~ sigma_perp /
  // (spread * sqrt(n_eff)). Report it so the UI can say "rail heading is not observable here".
  const nEff = (sw * sw) / (ws.reduce((acc, w) => acc + w * w, 0) || 1);
  // Use the sensor's stated error, not the fit residual: a perfectly-fitting but degenerate cloud
  // (a windless vertical flight) would otherwise report zero uncertainty on an angle that the data
  // cannot actually see.
  const sig = Math.max(rmsHoriz, sigmaFt, 1e-6);
  const yawSigmaDeg = horizSpreadFt > 1e-6 && nEff > 0 ? ((sig / (horizSpreadFt * Math.sqrt(nEff))) * 180) / Math.PI : Infinity;
  return { R, t, rms, rmsHoriz, rmsVert, yawDeg: (yaw * 180) / Math.PI, yawSigmaDeg, horizSpreadFt };
}
