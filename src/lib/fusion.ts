import { enuToGeodetic, geodeticToEnu, headingToEnu, type Pad } from './geo.ts';
import { ident, matMul, matVec, transpose, symMat3Mul, yawFitWeighted, type Mat3 } from './linalg.ts';
import { alignByAnchors, boxcar, clean, interp, madScale, median, type Series } from './signal.ts';
import { ferHas } from './parsers/blueRaven.ts';
import type { BrHighRow, BrLowRow, FlightData, FlightEvents, FlightStats, FusedSample, GpsRow } from './types.ts';

const G0 = 32.174; // ft/s^2
const GYRO_SAT_DPPS = 1900; // manual: the IMU wraps beyond roughly +/-2000 deg/s

export interface Vec3 {
  e: number;
  n: number;
  u: number;
}

export interface Track {
  t: number[];
  p: Vec3[];
  v: Vec3[];
}

export interface FusionTuning {
  /** inertial nav acceleration-noise PSD while the IMU is trustworthy (ft^2/s^3) */
  qBase: number;
  /** extra acceleration-noise PSD when the IMU is distrusted (tumble / gyro saturation) */
  qDistrust: number;
  /** bias random-walk PSD ((ft/s^2)^2 s) */
  qBias: number;
  /** floor on the GPS position sigma (ft) */
  sigmaPosMin: number;
  /** floor on the GPS velocity sigma (ft/s) */
  sigmaVelMin: number;
  /** dynamics inflation: sigma grows with speed, modelling antenna phase-centre and lever-arm error */
  posDynamic: number;
  velDynamic: number;
  /** Huber threshold for the registration IRLS, in robust sigmas */
  huberK: number;
}

export const DEFAULT_TUNING: FusionTuning = {
  qBase: 0.35,
  qDistrust: 900,
  qBias: 0.004,
  sigmaPosMin: 2.5,
  sigmaVelMin: 0.45,
  posDynamic: 0.012,
  velDynamic: 0.006,
  huberK: 1.8,
};

export interface Reconstruction {
  fused: Track;
  brOnly: Track;
  gpsOnly: Track;
  samples: FusedSample[];
  events: FlightEvents[];
  stats: FlightStats;
  registration: Registration;
  clock: { gpsOffsetS: number; score: number; runnerUpScore: number; anchorAgreementS: number | null };
  noise: { sigmaPosFt: number; sigmaVelFps: number; estimated: boolean };
  durationS: number;
  warnings: string[];
}

// --- helpers -----------------------------------------------------------------
const v3 = (e: number, n: number, u: number): Vec3 => ({ e, n, u });

/** Replace non-finite entries by linear interpolation across the neighbouring good samples. */
function spliceHoles(y: number[], ok: boolean[]): void {
  const n = y.length;
  let i = 0;
  while (i < n) {
    if (ok[i] && Number.isFinite(y[i])) { i++; continue; }
    let j = i;
    while (j < n && !(ok[j] && Number.isFinite(y[j]))) j++;
    const a = i - 1, b = j;
    if (a < 0) {
      const fill = b < n ? y[b] : 0;
      for (let k = i; k < Math.min(j, n); k++) y[k] = fill;
    } else if (b >= n) {
      for (let k = i; k < n; k++) y[k] = y[a];
    } else {
      const w = (b - a) || 1;
      for (let k = i; k < b; k++) y[k] = y[a] + ((y[b] - y[a]) * (k - a)) / w;
    }
    i = j;
  }
}

function finiteAll(...xs: number[]): boolean {
  return xs.every((x) => Number.isFinite(x));
}

/** Trapezoidal integration of a velocity series, starting at zero. */
function integrate(t: number[], v: number[]): number[] {
  const out = [0];
  for (let i = 1; i < t.length; i++) out.push(out[i - 1] + ((v[i] + v[i - 1]) / 2) * (t[i] - t[i - 1]));
  return out;
}

/** Central-difference velocity from a position series. */
function differentiate(t: number[], p: number[]): number[] {
  const n = p.length;
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    const d = t[b] - t[a];
    out[i] = d > 0 ? (p[b] - p[a]) / d : 0;
  }
  return out;
}

/**
 * The Blue Raven's inertial-navigation output in its own frame.
 *
 * The manual gives velocity and position as (up, down-range, cross-range) ground-relative
 * components. Down-range/cross-range are compass-oriented by the rail heading at launch, which
 * the log does not record, so the frame is built as (down-range, cross-range, up) and then
 * registered onto GPS by the Procrustes fit — which recovers exactly that unknown rotation.
 */
export function inertialTrack(br: BrLowRow[]): { track: Track; source: string } {
  const t = br.map((r) => r.t);
  const hasVel = br.some((r) => finiteAll(r.velDown, r.velCross, r.velUp));
  const hasPos = br.some((r) => finiteAll(r.posDown, r.posCross));
  let src: string;
  let e: number[], n: number[], u: number[];
  let ve: number[], vn: number[], vu: number[];

  if (hasPos && hasVel) {
    // Use the altimeter's own navigation output: it is the filter's best estimate and it carries
    // the barometric aiding in altitude, which re-integrating the velocity columns would throw away.
    e = br.map((r) => r.posDown);
    n = br.map((r) => r.posCross);
    u = br.map((r) => r.altNav);
    ve = br.map((r) => r.velDown);
    vn = br.map((r) => r.velCross);
    vu = br.map((r) => r.velUp);
    src = 'inertial navigation solution';
  } else if (hasVel) {
    ve = br.map((r) => r.velDown);
    vn = br.map((r) => r.velCross);
    vu = br.map((r) => r.velUp);
    e = integrate(t, ve);
    n = integrate(t, vn);
    u = integrate(t, vu);
    src = 'inertial velocity (integrated; no position columns in this log)';
  } else if (br.some((r) => Number.isFinite(r.altNav))) {
    // Vertical-only fallback: no horizontal reference exists in the file at all.
    e = br.map(() => 0);
    n = br.map(() => 0);
    u = br.map((r) => (Number.isFinite(r.altNav) ? r.altNav : 0));
    ve = br.map(() => 0);
    vn = br.map(() => 0);
    vu = differentiate(t, u);
    src = 'barometric/ins altitude only (no horizontal reference in this log)';
  } else {
    // Nothing navigational in the file: keep the timeline and let the caller's warnings explain.
    e = br.map(() => 0); n = br.map(() => 0); u = br.map(() => 0);
    ve = br.map(() => 0); vn = br.map(() => 0); vu = br.map(() => 0);
    src = 'unusable inertial data';
  }
  // Missing rows inside an otherwise good log read as 0, which would be a 5000 ft step; splice the
  // gaps from the neighbouring samples instead.
  for (const arr of [e, n, u, ve, vn, vu]) spliceHoles(arr, br.map((r) => finiteAll(r.posDown, r.posCross, r.altNav)));
  return {
    track: { t, p: e.map((_, i) => v3(e[i], n[i], u[i])), v: ve.map((_, i) => v3(ve[i], vn[i], vu[i])) },
    source: src,
  };
}

export interface GpsTrack extends Track {
  /** fix quality per retained sample, parallel to t/p/v */
  fix: number[];
}

/** GPS fixes -> local ENU track in feet. */
export function gpsTrack(gps: GpsRow[], pad: Pad): GpsTrack {
  const t: number[] = [];
  const p: Vec3[] = [];
  const v: Vec3[] = [];
  const fix: number[] = [];
  for (const r of gps) {
    if (!finiteAll(r.lat, r.lon)) continue;
    if (r.fixType === 0) continue; // no fix at all
    const [e, n, u] = geodeticToEnu(r.lat, r.lon, Number.isFinite(r.altFt) ? r.altFt : pad.altFt, pad);
    let ve: number, vn: number;
    if (Number.isFinite(r.hvel) && Number.isFinite(r.heading)) {
      [ve, vn] = headingToEnu(r.hvel, r.heading);
    } else if (Number.isFinite(r.hvel)) {
      // Horizontal speed without a heading still constrains the speed magnitude, which is most
      // of the information during boost; heading is recovered from the position track instead.
      const h = headingFromPosition(t, p);
      [ve, vn] = headingToEnu(r.hvel, h);
    } else {
      ve = NaN; vn = NaN;
    }
    t.push(r.t);
    p.push(v3(e, n, u));
    v.push(v3(ve, vn, Number.isFinite(r.upvel) ? r.upvel : NaN));
    fix.push(r.fixType);
  }
  return { t, p, v, fix };
}

function headingFromPosition(ts: number[], p: Vec3[]): number {
  const n = ts.length;
  if (n < 3) return 0;
  const i = n - 1;
  const d = Math.max(1, Math.min(3, n - 1));
  const de = p[i].e - p[i - d].e;
  const dn = p[i].n - p[i - d].n;
  return (Math.atan2(de, dn) * 180) / Math.PI;
}

/** Per-sample distrust in 0..1 from high-rate IMU health and attitude. */
export function distrustSeries(tBr: number[], br: BrLowRow[], brHigh?: BrHighRow[]): number[] {
  const satT: number[] = [];
  if (brHigh) {
    for (const h of brHigh) {
      const g = Math.max(Math.abs(h.gyro[0]), Math.abs(h.gyro[1]), Math.abs(h.gyro[2]));
      if (g >= GYRO_SAT_DPPS || !h.gyro.every(Number.isFinite)) satT.push(h.t);
    }
  }
  const raw = br.map((r, i) => {
    let d = 0;
    const tilt = Math.abs(Number.isFinite(r.tilt) ? r.tilt : 0);
    // Past 90 degrees of tilt the airframe is tumbling and its own idea of up is worthless - but
    // only while it is still climbing counts. Every airframe turns nose-down at apogee and then
    // flies upside down under the drogue on purpose, so tilt alone says nothing once the altimeter
    // is falling.
    const climbing = !Number.isFinite(r.velUp) ? true : r.velUp > 0;
    if (tilt > 90 && climbing) d = Math.max(d, Math.min(1, (tilt - 90) / 40));
    // The vertical-rate and negative-acceleration register bits latch on first downward motion and
    // stay latched, which makes them useless as a health signal: only their edges mean anything, and
    // an apogee is an edge too. They are therefore deliberately not consulted here.
    // Gyro saturation windows: widen by a second either side, the divergence bracketing it.
    const near = satT.some((s) => Math.abs(s - tBr[i]) < 1.2);
    if (near) d = Math.max(d, 1);
    return d;
  });
  return boxcar(raw, Math.round(1.0 / Math.max(1e-3, (tBr[1] ?? 1) - (tBr[0] ?? 0))));
}

/** Invert a symmetric-ish 3x3 covariance; null when it is numerically singular. */
function inv3(m: number[]): number[] | null {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (!Number.isFinite(det) || Math.abs(det) < 1e-30) return null;
  const id = 1 / det;
  return [
    A * id, (c * h - b * i) * id, (b * f - c * e) * id,
    B * id, (a * i - c * g) * id, (c * d - a * f) * id,
    C * id, (b * g - a * h) * id, (a * e - b * d) * id,
  ];
}

// --- error-state Kalman filter + RTS smoother --------------------------------
/**
 * One axis of an error-state filter. The nominal trajectory is the (registered) inertial
 * path, and the state is the deviation from it:
 *
 *   x = [dp, dv, db]      db = residual acceleration bias (scale factor, misalignment, wind)
 *   x' = [[0,1,0],[0,0,1],[0,0,0]] x + noise
 *
 * Keeping the bias state is what lets the GPS anchor the trajectory without throwing away the
 * high-rate shape: the filter attributes a slow position/velocity disagreement to a bias rather
 * than to a real manoeuvre.
 */
export class AxisFilter {
  /**
   * Error-state filter for ONE Cartesian axis: [position error, velocity error, acceleration bias].
   *
   * Cartesian dynamics are separable, so three 3-state filters give the same answer as one 9-state
   * filter with a tenth of the arithmetic, and each axis can carry its own noise.
   *
   * Every number here is held in units of the initial 1-sigma (x~ = D^-1 x, P~ = D^-1 P D^-1). That
   * is not decoration: by the end of a parachute descent the position covariance reaches ~1e7 ft^2
   * while the bias covariance has decayed below 1e-6, and inverting a matrix whose entries span 13
   * orders of magnitude returns garbage - which is how an earlier build of this smoother produced
   * "velocities" of 9e6 ft/s. In scaled coordinates P~ stays O(1) and its inverse is trustworthy.
   */
  private readonly d: [number, number, number];
  x: [number, number, number] = [0, 0, 0];
  P: number[] = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  xPred: [number, number, number][] = [];
  PPred: number[][] = [];
  xUpd: [number, number, number][] = [];
  PUpd: number[][] = [];
  Fs: (Mat3 | null)[] = [];

  constructor(posSigma: number, velSigma: number, biasSigma: number) {
    this.d = [Math.max(1e-6, posSigma), Math.max(1e-6, velSigma), Math.max(1e-6, biasSigma)];
  }

  predict(dt: number, qa: number, qb: number) {
    const [d0, d1, d2] = this.d;
    const a1 = (d1 / d0) * dt;
    const a2 = (d2 / d0) * 0.5 * dt * dt;
    const a3 = (d2 / d1) * dt;
    const F: Mat3 = [1, a1, a2, 0, 1, a3, 0, 0, 1];
    const g1: [number, number, number] = [(0.5 * dt * dt) / d0, dt / d1, 1 / d2];
    const g2: [number, number, number] = [dt ** 3 / (6 * d0), (dt * dt) / (2 * d1), dt / d2];
    this.x = [
      this.x[0] + a1 * this.x[1] + a2 * this.x[2],
      this.x[1] + a3 * this.x[2],
      this.x[2],
    ];
    let P = symMat3Mul(this.P as unknown as Mat3, F) as unknown as number[];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) P[i * 3 + j] += qa * g1[i] * g1[j] + qb * g2[i] * g2[j];
    P = [P[0], P[1], P[2], P[1], P[4], P[5], P[2], P[5], P[8]]; // enforce symmetry
    this.P = P;
    this.Fs.push(F);
  }

  /**
   * Scalar measurement of one error component: h=0 for the position error, h=1 for the velocity
   * error. `meas` is the GPS-minus-nominal difference, i.e. a noisy reading of dp or dv themselves
   * (the nominal cancels out of the error-state measurement), so the predicted measurement is the
   * current state and the innovation is meas - x[h].
   *
   * Subtracting x[h] is not a detail. Applying `meas` as though it were already the innovation
   * makes every fix push the state a further K*meas in the *same* direction with nothing ever
   * pulling it back, and a 20 ms loop then integrates that into nonsense - the runaway that shows
   * up as "velocities" of 1e6 ft/s. The filter must close its own loop.
   */
  update(h: 0 | 1, meas: number, r: number) {
    if (!Number.isFinite(meas) || !(r > 0)) return;
    const Ph = [this.P[h], this.P[3 + h], this.P[6 + h]]; // P e_h, in scaled units
    const dh = this.d[h];
    const y = meas - dh * this.x[h]; // innovation about the current estimate
    const s = dh * dh * Ph[h] + r;
    if (!(s > 0)) return;
    const K = [dh * Ph[0] / s, dh * Ph[1] / s, dh * Ph[2] / s];
    this.x = [this.x[0] + K[0] * y, this.x[1] + K[1] * y, this.x[2] + K[2] * y];
    // A = I - K H' with H = dh * e_h, i.e. only column h is modified.
    const A = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    A[0 * 3 + h] -= K[0] * dh;
    A[1 * 3 + h] -= K[1] * dh;
    A[2 * 3 + h] -= K[2] * dh;
    const APA = matMul(matMul(A as unknown as Mat3, this.P as unknown as Mat3), transpose(A as unknown as Mat3)) as unknown as number[];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) APA[i * 3 + j] += K[i] * r * K[j];
    // Joseph form leaves an asymmetric residue from rounding; the smoother inverts P, so keep it clean.
    this.P = [APA[0], APA[1], APA[2], APA[1], APA[4], APA[5], APA[2], APA[5], APA[8]];
  }

  /**
   * Rauch-Tung-Striebel backward pass over the stored forward epochs, returned in real units.
   *
   * The smoothed mean needs only the forward quantities: C_i = P+ F' (P-)'^-1, then
   * x^_i = x+_i + C_i (x^_{i+1} - x-_{i+1}). That is what makes the replay zero-lag: a causal
   * filter would trail the GPS by a second or more during coast, which is very visible in a 3D
   * replay. The smoothed covariance is not needed for the displayed trajectory.
   */
  smooth(): [number, number, number][] {
    const n = this.xUpd.length;
    const out: [number, number, number][] = new Array(n);
    if (!n) return out;
    let xs = [...this.xUpd[n - 1]] as [number, number, number];
    out[n - 1] = xs;
    for (let i = n - 2; i >= 0; i--) {
      const pPrevInv = inv3(this.PPred[i + 1]);
      if (!pPrevInv) {
        xs = [...this.xUpd[i]] as [number, number, number];
        out[i] = xs;
        continue;
      }
      const C = matMul(
        matMul(this.PUpd[i] as unknown as Mat3, transpose(this.Fs[i] ?? ident())),
        pPrevInv as unknown as Mat3,
      ) as unknown as number[];
      const dx: [number, number, number] = [
        xs[0] - this.xPred[i + 1][0],
        xs[1] - this.xPred[i + 1][1],
        xs[2] - this.xPred[i + 1][2],
      ];
      const corr = matVec(C as Mat3, dx);
      xs = [this.xUpd[i][0] + corr[0], this.xUpd[i][1] + corr[1], this.xUpd[i][2] + corr[2]];
      out[i] = xs;
    }
    return out.map((s) => [s[0] * this.d[0], s[1] * this.d[1], s[2] * this.d[2]] as [number, number, number]);
  }
}

// --- pad origin --------------------------------------------------------------
/**
 * The launch pad, from the fixes recorded before the airframe moved.
 *
 * Only the *leading* run of low-speed fixes is used: the descent under a main parachute is also
 * low-speed, and averaging those in would drag the pad miles downrange. Medians, so a degraded
 * early fix cannot pull the origin. If the tracker was not running on the pad there is no honest
 * pad to recover, and the caller is told rather than a silently wrong origin being used.
 */
export function derivePad(gps: GpsRow[], brLow: BrLowRow[]): { pad: Pad; notes: string[] } {
  const notes: string[] = [];
  const valid = gps.filter((r) => finiteAll(r.lat, r.lon) && r.fixType >= 1);
  if (valid.length) {
    const firstMoving = valid.findIndex((r) => Math.abs(r.hvel) > 5);
    const onPad = firstMoving < 0 ? valid : valid.slice(0, firstMoving);
    let pool = onPad;
    if (pool.length < 3) {
      pool = valid.filter((r) => Math.abs(r.hvel) < 3);
      if (pool.length < 3) {
        pool = valid.slice(0, Math.min(5, valid.length));
        notes.push(
          `The tracker has only ${pool.length} low-speed fix(es), so the launch pad is taken from the start of the ` +
          `log and may be downrange of the true pad.`,
        );
      }
    }
    const lat = median(pool.map((r) => r.lat));
    const lon = median(pool.map((r) => r.lon));
    const alts = pool.map((r) => r.altFt).filter(Number.isFinite);
    const altFt = alts.length ? median(alts) : 0;
    if (Number.isFinite(lat) && Number.isFinite(lon)) {
      if (!alts.length) notes.push('No GPS altitude on the pad; using the barometric estimate for height above sea level.');
      return { pad: { lat, lon, altFt }, notes };
    }
  }
  // No usable GPS: fall back to a standard-atmosphere altitude guess from the barometric pressure
  // and leave the latitude/longitude at the north-east corner of the map. This is the altimeter
  // formula from the manual, h = 145366 (1 - (p/p_sea)^0.190284) with the pressure in hPa.
  const p0 = brLow.find((r) => Number.isFinite(r.baroPressureAtm))?.baroPressureAtm;
  const altFt = Number.isFinite(p0) ? 145366.01 * (1 - Math.pow(p0 as number, 0.190284)) : 0;
  notes.push('No usable GPS fix: the map origin is approximate and altitude is referenced to standard atmosphere.');
  return { pad: { lat: 0, lon: 0, altFt }, notes };
}

// --- frame registration ------------------------------------------------------
interface Registration {
  R: Mat3;
  t: [number, number, number];
  rmsFt: number;
  inlierFraction: number;
  yawDeg: number;
  sigmaPosFt: number;
  sigmaVelFps: number;
  pairs: number;
  yawSigmaDeg: number;
  rmsHorizFt: number;
  rmsVertFt: number;
  horizSpreadFt: number;
}

/**
 * Rotate/translate the inertial frame onto GPS East-North-Up.
 *
 * The Blue Raven builds down-range/cross-range from the rail heading it saw at launch, so its
 * horizontal axes are a yaw of ENU that the log never records. Only that one angle is fitted: the
 * vertical axis is common to both sensors, and allowing tilt angles would spend them absorbing an
 * altitude offset (a 2000 ft pad-altitude error became a tilted trajectory in early tests).
 * Huber IRLS is essential: during a tumble the inertial path is hundreds of feet wrong, and an
 * unweighted fit would rotate the whole flight to chase it.
 */
/**
 * @param opts.uptoS ignore fixes after this time. After apogee the altimeter's own navigation is
 *   knowingly diverged (a parachute descent has almost no acceleration to hold attitude with), and
 *   fitting the frame to that segment rotates the whole flight to chase a bad inertial track and
 *   inflates the estimated GPS noise by two orders of magnitude. The ascent is where the inertial
 *   solution earns trust, so that is where the frame is taken from.
 */
export function register(
  br: Track,
  gps: Track,
  fixType: number[],
  tuning: FusionTuning,
  opts: { uptoS?: number } = {},
): Registration {
  const uptoS = Number.isFinite(opts.uptoS ?? NaN) ? (opts.uptoS as number) : Infinity;
  const xs: [number, number, number][] = [];
  const ys: [number, number, number][] = [];
  const vxs: [number, number, number][] = [];
  const vys: [number, number, number][] = [];
  const w0: number[] = [];
  for (let k = 0; k < gps.t.length; k++) {
    const t = gps.t[k];
    if (t < br.t[0] || t > br.t[br.t.length - 1] || t > uptoS) continue;
    if (!finiteAll(gps.p[k].e, gps.p[k].n, gps.p[k].u)) continue;
    const fix = fixType[k];
    const x: [number, number, number] = [
      interp({ t: br.t, v: br.p.map((p) => p.e) }, t),
      interp({ t: br.t, v: br.p.map((p) => p.n) }, t),
      interp({ t: br.t, v: br.p.map((p) => p.u) }, t),
    ];
    xs.push(x);
    ys.push([gps.p[k].e, gps.p[k].n, gps.p[k].u]);
    const hasVel = finiteAll(gps.v[k].e, gps.v[k].n, gps.v[k].u);
    vxs.push([
      interp({ t: br.t, v: br.v.map((p) => p.e) }, t),
      interp({ t: br.t, v: br.v.map((p) => p.n) }, t),
      interp({ t: br.t, v: br.v.map((p) => p.u) }, t),
    ]);
    vys.push(hasVel ? [gps.v[k].e, gps.v[k].n, gps.v[k].u] : [NaN, NaN, NaN]);
    // Weight by reported fix quality: a 2-D or satellite-starved fix carries far more error.
    w0.push(fix >= 3 ? 1 : fix === 2 ? 0.25 : 0.05);
  }

  const empty: Registration = {
    R: ident(), t: [0, 0, 0], rmsFt: NaN, inlierFraction: 0, yawDeg: 0, sigmaPosFt: NaN, sigmaVelFps: NaN,
    pairs: xs.length, yawSigmaDeg: Infinity, rmsHorizFt: NaN, rmsVertFt: NaN, horizSpreadFt: 0,
  };
  if (xs.length < 10) return empty;

  let ws = [...w0];
  let fit = yawFitWeighted(xs, ys, ws, tuning.sigmaPosMin);
  for (let iter = 0; iter < 12; iter++) {
    const res = xs.map((x, i) => {
      const p = matVec(fit.R, x);
      return Math.hypot(p[0] + fit.t[0] - ys[i][0], p[1] + fit.t[1] - ys[i][1], p[2] + fit.t[2] - ys[i][2]);
    });
    const scale = madScale(res) || Math.max(1e-3, fit.rms * 0.5) || 1;
    ws = w0.map((base, i) => {
      const u = res[i] / scale;
      return base * (u > tuning.huberK ? tuning.huberK / Math.max(u, 1e-6) : 1);
    });
    const next = yawFitWeighted(xs, ys, ws, tuning.sigmaPosMin);
    const delta = Math.abs(next.rms - fit.rms);
    fit = next;
    if (delta < 1e-4) break;
  }

  const resPos: number[] = [];
  const resVel: number[] = [];
  let inliers = 0;
  for (let i = 0; i < xs.length; i++) {
    const p = matVec(fit.R, xs[i]);
    for (let a = 0; a < 3; a++) {
      const r = [p[0] + fit.t[0], p[1] + fit.t[1], p[2] + fit.t[2]][a] - ys[i][a];
      if (ws[i] > 1e-3 && Number.isFinite(r)) resPos.push(r);
    }
    if (finiteAll(vys[i][0], vys[i][1], vys[i][2])) {
      const pv = matVec(fit.R, vxs[i]);
      for (let a = 0; a < 3; a++) resVel.push(pv[a] - vys[i][a]);
    }
    if (ws[i] > 1e-3) inliers++;
  }
  // Sensor noise is measured from the data, not assumed: the robust scale of the registration
  // residuals is the true combined GPS + inertial scatter, which is what the filter needs.
  const perAxis = [0, 1, 2].map((a) => madScale(resPos.filter((_, i) => i % 3 === a)) || 0);
  const sigmaPos = Math.sqrt((perAxis[0] ** 2 + perAxis[1] ** 2 + perAxis[2] ** 2) / 3);
  const velPerAxis = [0, 1, 2].map((a) => madScale(resVel.filter((_, i) => i % 3 === a)) || 0);
  return {
    R: fit.R,
    t: fit.t,
    rmsFt: fit.rms,
    inlierFraction: inliers / xs.length,
    yawDeg: (Math.atan2(fit.R[1], fit.R[0]) * 180) / Math.PI,
    sigmaPosFt: sigmaPos,
    sigmaVelFps: Math.sqrt((velPerAxis[0] ** 2 + velPerAxis[1] ** 2 + velPerAxis[2] ** 2) / 3),
    pairs: xs.length,
    yawSigmaDeg: fit.yawSigmaDeg,
    rmsHorizFt: fit.rmsHoriz,
    rmsVertFt: fit.rmsVert,
    horizSpreadFt: fit.horizSpreadFt,
  };
}

// --- reconstruction ----------------------------------------------------------
function speedOfSoundFps(altFt: number): number {
  const hM = Math.max(0, altFt * 0.3048);
  const T = hM < 11000 ? 288.15 - 0.0065 * hM : 216.65; // ISA troposphere / tropopause
  return Math.sqrt(1.4 * 287.05 * T) * 3.28084;
}

/**
 * Reconstruct a flight trajectory from a Blue Raven log and a GPS log.
 *
 * Division of labour: the inertial navigation supplies the shape at 50 Hz but drifts and can
 * diverge when the airframe tumbles; GPS supplies absolute position/velocity at ~10 Hz with
 * dropouts and a clock of its own. The pipeline therefore has to (1) find the clock offset,
 * (2) find the unknown rotation between the two horizontal frames, then (3) blend them with a
 * filter whose trust in the inertial source is explicitly time-varying.
 */
export function reconstruct(data: FlightData, tuning: FusionTuning = DEFAULT_TUNING): Reconstruction {
  const warnings: string[] = [...data.meta.warnings];
  const brLow = data.brLow;
  if (!brLow.length) throw new Error('Cannot reconstruct: the Blue Raven low-rate log is empty.');

  const inertial = inertialTrack(brLow);
  if (inertial.source.startsWith('unusable')) warnings.push('Blue Raven log has no usable navigation output; showing GPS only.');
  const nominal = inertial.track;
  const tBr = nominal.t;
  const pad = data.pad;

  const gpsRaw = gpsTrack(data.gps, pad);

  // --- 1. clock alignment on the vertical-velocity profile --------------------
  const refUp = clean(tBr, nominal.v.map((v) => v.u));
  let mvUp = clean(gpsRaw.t, gpsRaw.v.map((v) => v.u));
  if (mvUp.t.length < 20) {
    // Some loggers give altitude but no Doppler up-velocity; differentiate instead.
    const d = gpsRaw.p.map((p, i) => {
      if (i === 0) return NaN;
      const dt = gpsRaw.t[i] - gpsRaw.t[i - 1];
      return dt > 1e-3 ? (p.u - gpsRaw.p[i - 1].u) / dt : NaN;
    });
    mvUp = clean(gpsRaw.t, d);
    if (mvUp.t.length >= 20) warnings.push('GPS log has no upward-velocity column; derived it from altitude for clock alignment.');
  }

  let clock = { offset: 0, score: 0, runnerUpScore: -2, anchorAgreementS: null as number | null, coarse: 0 };
  if (mvUp.t.length >= 20 && refUp.t.length >= 40) {
    // Peak speed and apogee are the only two moments in a flight that both sensors see *uniquely*;
    // correlating the whole record instead is unreliable because the descent under a parachute is
    // quasi-periodic and many offsets score almost as well.
    const a = alignByAnchors(refUp, mvUp, { windowS: 1.5, step: 0.01, gridHz: 50 });
    clock = { offset: a.offset, score: a.score, runnerUpScore: a.runnerUpScore, anchorAgreementS: a.anchorAgreementS, coarse: a.coarse };
    if (clock.score < 0.6) warnings.push(`GPS clock alignment is weak (correlation ${clock.score.toFixed(2)}); the two logs may not be from the same flight.`);
    else if (clock.runnerUpScore > clock.score - 0.05) warnings.push(`GPS clock alignment is ambiguous (runner-up ${clock.runnerUpScore.toFixed(2)}); the coast phase gives little to lock onto.`);
    else if (a.anchorAgreementS !== null && a.anchorAgreementS > 0.25) {
      warnings.push(`Burnout and apogee disagree about the GPS clock by ${a.anchorAgreementS.toFixed(2)} s; the estimate relies on the boost phase.`);
    }
  } else if (data.gps.length) {
    warnings.push('Not enough GPS data to align its clock to the altimeter; assuming zero offset.');
  }

  const gps: GpsTrack = { ...gpsRaw, t: gpsRaw.t.map((t) => t + clock.offset) };

  // --- 2. frame registration (rotation + origin) ------------------------------
  // Apogee from the inertial track itself: the first crossing of zero vertical velocity after its
  // peak speed. Cheap, and it is the natural boundary between "the altimeter knows where it is"
  // and not, so the frame is fitted only to fixes on the near side of it.
  let iPeakGuess = 0;
  for (let i = 0; i < nominal.v.length; i++) if (Math.abs(nominal.v[i].u) > Math.abs(nominal.v[iPeakGuess].u)) iPeakGuess = i;
  let tApogeeGuess = NaN;
  for (let i = iPeakGuess + 1; i < nominal.v.length; i++) {
    const a = nominal.v[i - 1].u, b = nominal.v[i].u;
    if (a > 0 && b <= 0) { tApogeeGuess = tBr[i - 1] + (tBr[i] - tBr[i - 1]) * (a / Math.max(1e-9, a - b)); break; }
  }

  const reg = register(nominal, gps, gps.fix, tuning, { uptoS: tApogeeGuess });
  const R = reg.R;
  const registered: Track = {
    t: tBr,
    p: nominal.p.map((p) => {
      const q = matVec(R, [p.e, p.n, p.u]);
      return v3(q[0] + reg.t[0], q[1] + reg.t[1], q[2] + reg.t[2]);
    }),
    v: nominal.v.map((p) => {
      const q = matVec(R, [p.e, p.n, p.u]);
      return v3(q[0], q[1], q[2]);
    }),
  };
  if (!Number.isFinite(reg.rmsFt)) {
    warnings.push('Could not register the inertial frame to GPS (too few usable fixes); fusing with an identity frame.');
  } else if (reg.inlierFraction < 0.35) {
    warnings.push(`Only ${(reg.inlierFraction * 100).toFixed(0)}% of GPS fixes agree with the inertial path; the inertial solution drifted and is being over-ridden wherever GPS is available.`);
  }

  const sigmaPos = Math.max(tuning.sigmaPosMin, Number.isFinite(reg.sigmaPosFt) ? reg.sigmaPosFt * 1.3 : 12);
  const sigmaVel = Math.max(tuning.sigmaVelMin, Number.isFinite(reg.sigmaVelFps) ? reg.sigmaVelFps * 1.3 : 2);

  // --- 3. time-varying trust in the inertial solution -------------------------
  const useHigh = !!data.brHigh?.length;
  if (!useHigh && data.gps.length) warnings.push('No high-rate log supplied: gyro-saturation detection is limited to tilt and event flags.');
  const distrust = distrustSeries(tBr, brLow, useHigh ? data.brHigh : undefined);

  // --- 4. error-state filter + smoother, one axis at a time -------------------
  const n = tBr.length;
  const filters = [0, 1, 2].map(() => new AxisFilter(
    Math.max(30, sigmaPos * 3),
    Math.max(3, sigmaVel * 2),
    1.5,
  ));
  let j = 0;
  let updates = 0;
  for (let i = 0; i < n; i++) {
    if (i > 0) {
      const dt = tBr[i] - tBr[i - 1];
      const qa = tuning.qBase + distrust[i] * tuning.qDistrust;
      for (const f of filters) f.predict(dt, qa, tuning.qBias);
    }
    for (const f of filters) {
      f.xPred.push([...f.x] as [number, number, number]);
      f.PPred.push([...f.P]);
    }
    // GPS epochs that arrived during this inertial step are applied at its end: at 50 Hz that is
    // at most a 20 ms propagation delay, which the filter models directly.
    while (j < gps.t.length && gps.t[j] <= tBr[i] + 1e-9) {
      if (gps.t[j] >= tBr[0] - 0.5) {
        const speed = Math.hypot(gps.v[j].e, gps.v[j].n, gps.v[j].u);
        const fixInfl = gps.fix[j] >= 3 ? 1 : gps.fix[j] === 2 ? 2.2 : 6;
        const rp = (sigmaPos * fixInfl) ** 2 + (tuning.posDynamic * (Number.isFinite(speed) ? speed : 0)) ** 2;
        const rv = (sigmaVel * fixInfl) ** 2 + (tuning.velDynamic * (Number.isFinite(speed) ? speed : 0)) ** 2;
        for (let a = 0; a < 3; a++) {
          const pG = [gps.p[j].e, gps.p[j].n, gps.p[j].u][a];
          const p0 = [registered.p[i].e, registered.p[i].n, registered.p[i].u][a];
          if (Number.isFinite(pG)) filters[a].update(0, pG - p0, rp);
          const vG = [gps.v[j].e, gps.v[j].n, gps.v[j].u][a];
          const v0 = [registered.v[i].e, registered.v[i].n, registered.v[i].u][a];
          if (Number.isFinite(vG)) filters[a].update(1, vG - v0, rv);
        }
        updates++;
      }
      j++;
    }
    for (const f of filters) {
      f.xUpd.push([...f.x] as [number, number, number]);
      f.PUpd.push([...f.P]);
    }
  }
  const sm = filters.map((f) => f.smooth());

  const fused: Track = {
    t: tBr,
    p: tBr.map((_, i) => v3(registered.p[i].e + sm[0][i][0], registered.p[i].n + sm[1][i][0], registered.p[i].u + sm[2][i][0])),
    v: tBr.map((_, i) => v3(registered.v[i].e + sm[0][i][1], registered.v[i].n + sm[1][i][1], registered.v[i].u + sm[2][i][1])),
  };
  if (!updates && data.gps.length) warnings.push('No GPS epoch fell inside the altimeter recording window; the trajectory is inertial only.');

  // --- 4b. GPS-only comparison track (velocity filled from position when absent)
  const gpsVelFilled: Vec3[] = gps.v.map((v, i) => {
    if (finiteAll(v.e, v.n, v.u)) return v;
    const a = Math.max(0, i - 2), b = Math.min(gps.t.length - 1, i + 2);
    const dt = Math.max(1e-3, gps.t[b] - gps.t[a]);
    return v3((gps.p[b].e - gps.p[a].e) / dt, (gps.p[b].n - gps.p[a].n) / dt, (gps.p[b].u - gps.p[a].u) / dt);
  });
  const gpsOnly: Track = { t: gps.t, p: gps.p, v: gpsVelFilled };

  // --- 5. samples -------------------------------------------------------------
  const hr = data.brHigh;
  let hi = 0;
  const samples: FusedSample[] = fused.t.map((t, i) => {
    if (hr) {
      while (hi < hr.length - 1 && hr[hi + 1].t <= t) hi++;
    }
    const row = brLow[i];
    return {
      t,
      e: fused.p[i].e,
      n: fused.p[i].n,
      u: fused.p[i].u,
      ve: fused.v[i].e,
      vn: fused.v[i].n,
      vu: fused.v[i].u,
      q: hr && hr[hi] ? hr[hi].quat : undefined,
      tilt: row.tilt,
      roll: row.roll,
      brConfidence: 1 - distrust[i],
      baroAgl: row.altBaroAgl,
      altNav: row.altNav,
    };
  });

  const events = decodeEvents(data, fused, distrust, gps, warnings);
  const stats = computeStats(fused, events, pad.altFt, data.brHigh);

  return {
    fused,
    brOnly: registered,
    gpsOnly,
    samples,
    events,
    stats,
    registration: {
      R: reg.R, t: reg.t, rmsFt: reg.rmsFt, inlierFraction: reg.inlierFraction, yawDeg: reg.yawDeg,
      yawSigmaDeg: reg.yawSigmaDeg, rmsHorizFt: reg.rmsHorizFt, rmsVertFt: reg.rmsVertFt, horizSpreadFt: reg.horizSpreadFt,
      sigmaPosFt: reg.sigmaPosFt, sigmaVelFps: reg.sigmaVelFps, pairs: reg.pairs,
    },
    clock: { gpsOffsetS: clock.offset, score: clock.score, runnerUpScore: clock.runnerUpScore, anchorAgreementS: clock.anchorAgreementS },
    noise: { sigmaPosFt: sigmaPos, sigmaVelFps: sigmaVel, estimated: Number.isFinite(reg.sigmaPosFt) },
    durationS: tBr[n - 1] - tBr[0],
    warnings,
  };
}

// --- events ------------------------------------------------------------------
function firstWhere(ts: number[], pred: (i: number) => boolean, from = 0): number {
  for (let i = from; i < ts.length; i++) if (pred(i)) return i;
  return -1;
}

/**
 * Events, taken from the flight-event registers where they exist and from the physics where they
 * do not.
 *
 * Register bits (field reference): 0 liftoff, 1 apogee, 2 pressure increase, 3 APO charge,
 * 4 main charge, 5 3rd channel, 6 4th channel, 7 ECI velocity not positive, 8 acceleration not
 * positive, 9 tilt over 90 degrees.
 *
 * Burnout has no register bit, so it comes from the accelerometer: an accelerometer in free fall
 * reads zero, so a powered phase stands out by an order of magnitude and its end is the cutoff.
 * The threshold is a fraction of the boost peak rather than a fixed number of g, which keeps a
 * 40 G sustainer and a 6 G booster on the same code.
 *
 * Every time reported here is on the Blue Raven's own clock, which is what a printed log shows.
 */
function decodeEvents(
  data: FlightData,
  fused: Track,
  distrust: number[],
  gps: Track,
  warnings: string[],
): FlightEvents[] {
  const { brLow } = data;
  const t = fused.t;
  const n = t.length;
  const up: Series = { t, v: fused.p.map((p) => p.u) };
  const netG = t.map((_, i) => {
    const a = Math.max(0, i - 2);
    const b = Math.min(n - 1, i + 2);
    const dt = Math.max(1e-3, t[b] - t[a]);
    const dvec = [fused.v[b].e - fused.v[a].e, fused.v[b].n - fused.v[a].n, fused.v[b].u - fused.v[a].u];
    return Math.hypot(dvec[0], dvec[1], dvec[2]) / dt / G0;
  });

  const ev: FlightEvents[] = [];  const push = (time: number, code: FlightEvents['code'], label: string) => {
    if (Number.isFinite(time)) ev.push({ t: time, code, label, altFt: Math.max(0, interp(up, time)) });
  };

  const bitTime = (bit: number) => {
    const r = brLow.find((x) => ferHas(x.fer, bit));
    return r ? r.t : NaN;
  };
  // Charge firings. Their times are also what the clock cross-check below compares against, so they
  // are read here rather than where the events are pushed.
  const tDrogue = bitTime(3);
  const tMain = bitTime(4);

  // Apogee is the top of the fused track; the register's own apogee (bit 1) only latches once the
  // altimeter sees the pressure rise, so it is used as a cross-check rather than as the answer.
  let iApogee = 0;
  for (let i = 0; i < n; i++) if (fused.p[i].u > fused.p[iApogee].u) iApogee = i;
  const tApogee = t[iApogee];
  const speedArr = fused.v.map((v) => Math.hypot(v.e, v.n, v.u));
  const maxV = Math.max(...speedArr);
  const tApogeeReg = bitTime(1);
  if (Number.isFinite(tApogeeReg) && Math.abs(tApogeeReg - tApogee) > 2)
    warnings.push(`Apogee from the barometric register (${tApogeeReg.toFixed(1)} s) and from the fused track (${tApogee.toFixed(1)} s) disagree by more than 2 s.`);

  const tLiftoff = bitTime(0);
  const iLiftoff = Number.isFinite(tLiftoff)
    ? nearestIndex(t, tLiftoff)
    : Math.max(0, firstWhere(t, (i) => fused.v[i].u > 10));
  push(t[iLiftoff], 'liftoff', 'Liftoff (thrust-to-weight exceeded 1)');

  // Burnout from the speed trace. While a motor runs the airframe gains speed; once it is out, drag
  // can only take speed away, so the top of the speed trace below apogee is burnout - which is also
  // the number the manual prints as `BO:`. Reading it off the trajectory rather than off the
  // accelerometers is what keeps a separation kick or a drogue pop - both of which are some 20 G for
  // 100 ms on the accelerometer and a few ft/s on the trajectory - from looking like a motor. A
  // staged motor that finishes weaker than the one before it shows as a single burnout, which is what
  // an analyst reading the log would write down.
  let iBurn = iLiftoff;
  for (let i = iLiftoff; i <= iApogee; i++) if (speedArr[i] > speedArr[iBurn]) iBurn = i;
  if (speedArr[iBurn] > 0.1 * maxV)
    push(t[iBurn], 'burnout', `Burnout ${speedArr[iBurn].toFixed(0)} ft/s after ${(t[iBurn] - t[iLiftoff]).toFixed(1)} s of powered flight`);

  push(tApogee, 'apogee', `Apogee ${Math.round(fused.p[iApogee].u).toLocaleString()} ft AGL`);

  // Charge-channel firings. The APO channel (bit 3) is the drogue and the main channel (bit 4) the
  // mains; the 3rd and 4th channels are reported as themselves because on this firmware the 3rd
  // channel latches together with apogee, and reading it as a deployment would invent an event.
  const spikeAfter = (from: number, minG: number) => {
    const i = firstWhere(t, (j) => j > from && netG[j] > minG && fused.v[j].u < 0, Math.max(0, nearestIndex(t, from)));
    return i >= 0 ? t[i] : NaN;
  };
  const tDrogueUse = Number.isFinite(tDrogue) ? tDrogue : spikeAfter(tApogee, 3);
  const tMainUse = Number.isFinite(tMain) ? tMain : spikeAfter(tDrogueUse, 5);
  push(tDrogueUse, 'drogue', 'Drogue deployment');
  push(tMainUse, 'main', 'Main deployment');

  for (const [bit, name] of [[5, '3rd'], [6, '4th']] as [number, string][]) {
    const tb = bitTime(bit);
    if (!Number.isFinite(tb)) continue;
    const mirrored = Math.abs(tb - (Number.isFinite(tApogeeReg) ? tApogeeReg : tApogee)) < 0.2;
    push(tb, 'channel', `${name} charge channel fired${mirrored ? ' (latched with apogee on this firmware, not a deployment)' : ''}`);
  }

  // Landing. The GPS velocity is far too noisy to decide this on its own - a motionless airframe
  // still reads 20-30 ft/s of Doppler noise - so the barometric altitude does the deciding, with a
  // sustained standstill in the fused track as the fallback when the altimeter is unusable.
  const stillWindow = 3;
  const baroAgl = brLow.map((r) => r.altBaroAgl);
  const baroUsable = baroAgl.some((a) => Number.isFinite(a)) && Math.max(...baroAgl.filter(Number.isFinite)) < 200000;
  let tLand = NaN;
  if (baroUsable) {
    const iBaro = brLow.findIndex(
      (r, i) =>
        r.t > (Number.isFinite(tDrogueUse) ? tDrogueUse : tApogee) &&
        Number.isFinite(r.altBaroAgl) &&
        r.altBaroAgl < 10 &&
        baroAgl.slice(i, i + 100).filter(Number.isFinite).length > 0 &&
        Math.max(...baroAgl.slice(i, i + 100).filter(Number.isFinite)) < 20,
    );
    if (iBaro >= 0) tLand = brLow[iBaro].t;
  }
  if (!Number.isFinite(tLand)) {
    // No usable altimeter: require the track to hold still for the whole window, not merely to
    // have a low instantaneous speed.
    const i0 = Math.max(0, nearestIndex(t, Number.isFinite(tDrogueUse) ? tDrogueUse : tApogee));
    const span = Math.max(1, Math.round(stillWindow / (t[1] - t[0] || 0.02)));
    for (let i = i0; i + span < n; i++) {
      const move = Math.hypot(fused.p[i + span].e - fused.p[i].e, fused.p[i + span].n - fused.p[i].n, fused.p[i + span].u - fused.p[i].u);
      if (move < 40 && fused.p[i].u < 200) {
        tLand = t[i];
        break;
      }
    }
  }
  if (Number.isFinite(tLand)) {
    const iL = nearestIndex(t, tLand);
    const horiz = Math.hypot(fused.v[iL].e, fused.v[iL].n);
    push(tLand, 'landing', `Landing ${Math.abs(fused.v[iL].u).toFixed(0)} ft/s descent, ${horiz.toFixed(0)} ft/s horizontal`);
  } else warnings.push('Landing was not detected; the log ends before the airframe came to rest.');

  // Anomalies the registers do not report.
  const iSat = data.brHigh ? firstWhere(t, (i) => distrust[i] > 0.6) : -1;
  if (iSat >= 0) {
    const endI = firstWhere(t, (i) => i > iSat && distrust[i] < 0.2, iSat);
    const dur = (endI >= 0 ? t[endI] : t[n - 1]) - t[iSat];
    push(t[iSat], 'gyro-saturation', `Gyro saturation: the airframe's own attitude was unusable for ~${dur.toFixed(1)} s, so the solution is GPS-carried there`);
  }
  // Bit 8 latches the first time the airframe points more than 90 degrees away from up. That happens
  // to every rocket at apogee, so what the event means depends on when it happened: past vertical
  // while still climbing is a tumbling airframe and a termination condition, and past vertical after
  // the drogue has left it descending nose-first is what a recovery looks like.
  // Tilt past 90 degrees is read from the reported tilt angle rather than from bit 8, because bit 8
  // is latched by pad noise (a level airframe whose accelerometers are noisy enough to put the
  // tilt estimate anywhere on a cone) and a latch never clears. Two seconds of it is required: the
  // pad wander lasts milliseconds, a real turnover or tumble lasts the rest of the flight.
  let tTilt = NaN;
  {
    const need = Math.max(1, Math.round(2 / (t[1] - t[0] || 0.02)));
    for (let i = 0; i + need < brLow.length; i++) {
      if (!Number.isFinite(brLow[i].tilt) || brLow[i].tilt <= 90) continue;
      let held = true;
      for (let k = 1; k < need; k++)
        if (!Number.isFinite(brLow[i + k].tilt) || brLow[i + k].tilt <= 90) {
          held = false;
          i += k;
          break;
        }
      if (held) {
        tTilt = brLow[i].t;
        break;
      }
    }
  }
  if (Number.isFinite(tTilt)) {
    const what =
      tTilt < tApogee
        ? 'Airframe tilted past 90 degrees while still climbing (tumble, loss of stability)'
        : tTilt < tApogee + 15
          ? 'Airframe turned over at apogee (expected: it now flies nose-down under the drogue)'
          : 'Airframe tilted past 90 degrees during descent';
    push(tTilt, 'tilt-over-90', what);
  }

  const sorted = [...gps.t].filter(Number.isFinite).sort((a, b) => a - b);
  for (let i = 1; i < sorted.length; i++) {
    const gap = sorted[i] - sorted[i - 1];
    if (gap > 2.5) {
      ev.push({ t: sorted[i - 1], code: 'gps-loss', label: `GPS gap ${gap.toFixed(1)} s`, altFt: Math.max(0, interp(up, sorted[i - 1])) });
    }
  }
  if (data.gps.length && sorted.length === 0) ev.push({ t: 0, code: 'gps-loss', label: 'No usable GPS fixes', altFt: 0 });

  return ev.filter((e) => Number.isFinite(e.t)).sort((a, b) => a.t - b.t);
}

function nearestIndex(ts: number[], t: number): number {
  if (!ts.length) return -1;
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid;
  }
  return Math.abs(ts[lo] - t) <= Math.abs(ts[hi] - t) ? lo : hi;
}

function computeStats(fused: Track, events: FlightEvents[], padAltFt: number, high: BrHighRow[] = []): FlightStats {
  const t = fused.t;
  const speed = fused.v.map((v) => Math.hypot(v.e, v.n, v.u));
  let iAlt = 0, iVel = 0, iMach = 0;
  for (let i = 0; i < t.length; i++) {
    if (fused.p[i].u > fused.p[iAlt].u) iAlt = i;
    if (speed[i] > speed[iVel]) iVel = i;
    const mach = speed[i] / speedOfSoundFps(fused.p[i].u);
    if (mach > speed[iMach] / speedOfSoundFps(fused.p[iMach].u)) iMach = i;
  }
  // Peak acceleration is a property of the accelerometers, not of an integrated velocity: during
  // coast the two differ by the 1 G of gravity the filter carries and the accelerometer does not.
  let maxAccel = 0;
  if (high.length) {
    for (const r of high) maxAccel = Math.max(maxAccel, Math.hypot(r.accel[0], r.accel[1], r.accel[2]) * G0);
  } else {
    for (let i = 2; i < t.length - 2; i++) {
      const dt = Math.max(1e-3, t[i + 2] - t[i - 2]);
      const d = [fused.v[i + 2].e - fused.v[i - 2].e, fused.v[i + 2].n - fused.v[i - 2].n, fused.v[i + 2].u - fused.v[i - 2].u];
      maxAccel = Math.max(maxAccel, Math.hypot(d[0], d[1], d[2]) / dt);
    }
  }
  const at = (code: FlightEvents['code']) => events.find((e) => e.code === code);
  const iLand = at('landing') ? nearestIndex(t, at('landing')!.t) : t.length - 1;
  const iDrogue = at('drogue') ? nearestIndex(t, at('drogue')!.t) : -1;
  const iLiftoff = at('liftoff') ? nearestIndex(t, at('liftoff')!.t) : 0;
  return {
    maxAltFt: fused.p[iAlt].u,
    maxAltT: t[iAlt],
    maxVelFps: speed[iVel],
    maxMach: speed[iMach] / speedOfSoundFps(fused.p[iMach].u),
    maxAccelG: maxAccel / G0,
    flightTimeS: t[iLand] - t[iLiftoff],
    drogueVelFps: iDrogue >= 0 ? speed[iDrogue] : NaN,
    landingVelFps: speed[iLand],
    driftFt: Math.hypot(fused.p[iLand].e, fused.p[iLand].n),
    padAltFt,
  };
}

/** Convenience: geodetic landing position for recovery reporting. */
export function landingPosition(fused: Track, events: FlightEvents[], pad: Pad) {
  const land = events.find((e) => e.code === 'landing');
  const i = land ? nearestIndex(fused.t, land.t) : fused.t.length - 1;
  return enuToGeodetic(fused.p[i].e, fused.p[i].n, fused.p[i].u, pad);
}
