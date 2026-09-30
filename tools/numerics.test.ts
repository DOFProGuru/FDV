// Unit checks for the numerics that the reconstruction depends on.
import { procrustesWeighted, yawFitWeighted, matVec, matMul, transpose, jacobiEigen, ident } from '../src/lib/linalg.ts';
import { alignByAnchors, alignByCrossCorrelation } from '../src/lib/signal.ts';

let fails = 0;
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  if (!ok) fails++;
}

// --- eigendecomposition ------------------------------------------------------
{
  const A = [4, 1, 0, 1, 3, 0, 0, 0, 2];
  const { values, vectors } = jacobiEigen(A as never);
  // [[4,1],[1,3]] has eigenvalues (7 +- sqrt(5))/2 = 2.382, 4.618; plus 2 from the z axis.
  const okVals = values[0] < values[1] && values[1] < values[2] &&
    Math.abs(values[0] - 2) < 1e-6 && Math.abs(values[1] - 2.381966) < 1e-5 && Math.abs(values[2] - 4.618034) < 1e-5;
  // V must be orthonormal and diagonalise A.
  const VtAV = matMul(matMul(transpose(vectors), A as never), vectors);
  const offDiag = Math.hypot(VtAV[1], VtAV[2], VtAV[3], VtAV[5], VtAV[6], VtAV[7]);
  check('jacobiEigen diagonalises a symmetric matrix', okVals && offDiag < 1e-7, `vals=${values.map((v) => v.toFixed(4))} offdiag=${offDiag.toExponential(1)}`);
}

// --- Procrustes: the Blue Raven's rail-heading rotation is a permutation ------
{
  // down-range == North, cross-range == West (a proper rotation of ENU by -90 deg about Up).
  const c = Math.cos(-Math.PI / 2), s = Math.sin(-Math.PI / 2);
  const Rz = [c, -s, 0, s, c, 0, 0, 0, 1];
  const truth = [120, -40, 15];
  const pts: [number, number, number][] = [];
  for (let i = 0; i < 400; i++) {
    const e = 100 * Math.sin(i * 0.07), n = 900 * (1 - Math.cos(i * 0.05)), u = 500 * i * 0.02;
    pts.push([e, n, u]);
  }
  const ys = pts.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0] + truth[0], r[1] + truth[1], r[2] + truth[2]] as [number, number, number];
  });
  const fit = procrustesWeighted(pts, ys, pts.map(() => 1));
  const err = Math.hypot(fit.t[0] - truth[0], fit.t[1] - truth[1], fit.t[2] - truth[2]);
  const rerr = Math.max(...[0, 1, 2].flatMap((i) => [0, 1, 2].map((j) => Math.abs(fit.R[i * 3 + j] - (Rz as never)[i * 3 + j]))));
  check('procrustes recovers an exact 90-degree rotation + translation', err < 1e-6 && rerr < 1e-6, `t err=${err.toExponential(1)} R err=${rerr.toExponential(1)} rms=${fit.rms.toExponential(1)}`);
}

// --- Procrustes: outliers from a tumble must not rotate the whole fit ---------
{
  const th = (30 * Math.PI) / 180;
  const Rz = [Math.cos(th), -Math.sin(th), 0, Math.sin(th), Math.cos(th), 0, 0, 0, 1];
  const pts: [number, number, number][] = [];
  // A real flight wanders in all three axes (wind, weathercock, drift); a cloud confined to a
  // plane would leave the twist about its normal genuinely unidentifiable.
  for (let i = 0; i < 300; i++) pts.push([900 * Math.sin(i * 0.02) + 30 * i * 0.05, 600 * i * 0.01 + 120 * Math.cos(i * 0.05), 300 * i * 0.03]);
  const ys = pts.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0], r[1], r[2]] as [number, number, number];
  });
  for (let i = 0; i < 40; i++) ys[i] = [ys[i][0] + 900, ys[i][1] - 1400, ys[i][2] + 700]; // divergence window
  const clean = pts.slice(40);
  const fit = yawFitWeighted(clean, ys.slice(40), clean.map(() => 1));
  check('yaw fit is unbiased by a diverged segment', Math.abs(fit.yawDeg - 30) < 0.5, `yaw=${fit.yawDeg.toFixed(2)} deg (expect 30), rms=${fit.rms.toFixed(1)}`);

  // Near-vertical, windless flight: the horizontal lever arm is a few feet, so the rail heading is
  // NOT knowable. The estimator must say so instead of inventing an angle.
  const vert: [number, number, number][] = [];
  for (let i = 0; i < 300; i++) vert.push([2 * Math.sin(i * 0.3), 1.5 * Math.cos(i * 0.3), 40 * i]);
  const vy = vert.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0] + 3, r[1] - 2, r[2] + 1] as [number, number, number];
  });
  const straight = yawFitWeighted(vert, vy, vert.map(() => 1), 15);
  check('yaw fit reports an unobservable rail heading for a vertical flight', straight.yawSigmaDeg > 5,
    `yaw sigma=${straight.yawSigmaDeg.toFixed(1)} deg, spread=${straight.horizSpreadFt.toFixed(1)} ft`);
}

// --- clock alignment ---------------------------------------------------------
{
  // Convention check first, with an unambiguous feature: the returned offset is ADDED to the
  // moving series' timestamps. A bump at 25 s matching a bump at 20 s must therefore be -5 s.
  const t: number[] = [], ref: number[] = [], mov: number[] = [];
  for (let i = 0; i < 4000; i++) {
    const s = i / 50;
    t.push(s);
    ref.push(Math.exp(-((s - 20) ** 2) / 2));
    mov.push(Math.exp(-((s - 25) ** 2) / 2));
  }
  const spike = alignByCrossCorrelation({ t, v: ref }, { t, v: mov }, { minShift: -10, maxShift: 10, step: 0.02 });
  check('alignByCrossCorrelation returns the offset to ADD to the moving series', Math.abs(spike.offset + 5) < 0.03,
    `got ${spike.offset.toFixed(3)} s (expect -5), r=${spike.score.toFixed(3)}`);
  const spikeA = alignByAnchors({ t, v: ref }, { t, v: mov }, { windowS: 1.5, step: 0.01, gridHz: 50 });
  check('anchor alignment agrees on the sign convention', Math.abs(spikeA.offset + 5) < 0.05, `got ${spikeA.offset.toFixed(3)} s`);
}
{
  // A realistic vertical-velocity profile: sharp boost, linear coast, parachute descent.
  const shape = (s: number) => s < 0 ? 0 : s < 2 ? 800 * s : s < 16 ? 1600 - 98 * (s - 2) : Math.max(-90, -98 * (s - 16) * Math.exp(-(s - 16) / 3) - 20 * (1 - Math.exp(-(s - 16) / 2)));
  // The moving series is built 2.14 s *ahead* of the reference, so its stamps sit 2.14 s early and
  // +2.14 must be added to them - which is exactly the documented convention.
  const offsetTrue = 2.14;
  const t: number[] = [], v1: number[] = [], v2: number[] = [];
  for (let i = 0; i < 12000; i++) { t.push(i / 50); v1.push(shape(i / 50)); }
  for (let i = 0; i < t.length; i++) v2.push(shape(t[i] + offsetTrue) + (Math.random() - 0.5) * 4);
  const a = alignByAnchors({ t, v: v1 }, { t, v: v2 }, { windowS: 1.5, step: 0.01, gridHz: 50 });
  check('anchor alignment recovers a flight-profile clock offset', Math.abs(a.offset - offsetTrue) < 0.03,
    `got ${a.offset.toFixed(3)} s of ${offsetTrue}, fit=${a.score.toFixed(4)}, anchors agree to ${(a.anchorAgreementS ?? NaN).toFixed(2)} s`);
}

// --- small-angle sanity: the identity is a valid rotation fit -----------------
{
  const pts: [number, number, number][] = [[0, 0, 0], [10, 0, 0], [0, 20, 0], [0, 0, 30]];
  const fit = procrustesWeighted(pts, pts.map((p) => [...p] as [number, number, number]), pts.map(() => 1));
  check('procrustes on identical data returns identity and no offset',
    fit.R.every((v, i) => Math.abs(v - (ident() as never)[i]) < 1e-9) && fit.t.every((v) => Math.abs(v) < 1e-9),
    `rms=${fit.rms.toExponential(1)}`);
}

console.log(fails ? `\n${fails} check(s) failed` : '\nall checks passed');
process.exit(fails ? 1 : 0);

// --- error-state filter + RTS smoother ---------------------------------------
{
  const dt = 0.02, T = 180, n = Math.round(T / dt);
  const t: number[] = [], accel: number[] = [];
  for (let i = 0; i < n; i++) {
    const s = i * dt;
    t.push(s);
    // boost, coast, parachute descent: a shape with real curvature in every phase
    accel.push(s < 2 ? 900 : s < 16 ? -32.2 : -32.2 + 300 * Math.exp(-(s - 16) / 1.5) - 28 * (1 - Math.exp(-(s - 16) / 2)));
  }
  // true vertical state
  const pTrue = new Array(n).fill(0), vTrue = new Array(n).fill(0);
  for (let i = 1; i < n; i++) {
    vTrue[i] = vTrue[i - 1] + 0.5 * dt * (accel[i] + accel[i - 1]);
    pTrue[i] = pTrue[i - 1] + 0.5 * dt * (vTrue[i] + vTrue[i - 1]);
  }
  // inertial nominal: the same shape plus a divergence that grows once the airframe tumbles
  const pNom = pTrue.map((p, i) => p - 0.0006 * t[i] ** 2 * (1 + 4 * Math.max(0, t[i] - 20) / 20));
  const vNom = pNom.map((_, i) => (i + 1 < n ? (pNom[i + 1] - pNom[i]) / dt : (pNom[i] - pNom[i - 1]) / dt));
  let seed = 11;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff - 0.5) * 2;
  const sigmaP = 56, sigmaV = 3, R = sigmaP ** 2, Rv = sigmaV ** 2;

  const f = new AxisFilter(3 * sigmaP, 2 * sigmaV, 1.5);
  let k = 0;
  for (let i = 0; i < n; i++) {
    if (i > 0) f.predict(dt, 0.35 + Math.max(0, (t[i] - 20) / 160) * 900, 0.004);
    f.xPred.push([...f.x]); f.PPred.push([...f.P]);
    while (k < t.length && k * dt * 5 <= i) {
      if (k % 5 === 0) {
        f.update(0, pTrue[k] + rnd() * sigmaP - pNom[k], R);
        f.update(1, vTrue[k] + rnd() * sigmaV - vNom[k], Rv);
      }
      k++;
    }
    f.xUpd.push([...f.x]); f.PUpd.push([...f.P]);
  }
  const sm = f.smooth();
  const fused = pNom.map((p, i) => p + sm[i][0]);
  const rms = (arr: number[]) => Math.sqrt(arr.reduce((a, b) => a + b * b, 0) / arr.length);
  const errNom = pNom.map((p, i) => p - pTrue[i]);
  const errFused = fused.map((p, i) => p - pTrue[i]);
  check('filter stays bounded over 9000 steps of divergent inertial data',
    fused.every(Number.isFinite) && Math.abs(Math.max(...fused)) < Math.max(...pTrue.map(Math.abs)) * 3,
    `max fused=${Math.max(...fused).toExponential(2)} ft vs truth ${Math.max(...pTrue).toFixed(0)} ft`);
  check('fusion beats the inertial nominal', rms(errFused) < rms(errNom) * 0.5,
    `inertial rms=${rms(errNom).toFixed(1)} ft, fused rms=${rms(errFused).toFixed(1)} ft`);
  check('fusion is close to the measurement accuracy it is given', rms(errFused) < sigmaP * 1.5,
    `fused rms=${rms(errFused).toFixed(1)} ft with ${sigmaP} ft fixes`);
  const velFused = vNom.map((v, i) => v + sm[i][1]);
  check('smoothed velocity is bounded too', Math.abs(Math.max(...velFused)) < Math.max(...vTrue.map(Math.abs)) * 3,
    `max fused vel=${Math.max(...velFused).toExponential(2)} ft/s`);
}
