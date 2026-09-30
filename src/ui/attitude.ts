/**
 * Which of the two documented readings of the high-rate quaternion a particular log is using, and
 * the resulting airframe pointing direction.
 *
 * FORMATS.md records the encoding as `[axis * sin(theta/2), cos(theta/2)]` but not what the rotated
 * vector means, and the two plausible readings differ:
 *
 *   `axis-is-pointing`  the quaternion is a roll about the airframe's own axis, so the rotation axis
 *                       (the vector part) *is* the pointing direction, up to nose/tail sign.
 *   `body-to-world`     an ordinary attitude quaternion: rotating body +z by it gives the pointing
 *                       direction.
 *
 * The two cannot be told apart from the quaternion alone, but the low-rate log reports the tilt angle
 * independently at 50 Hz, which is a measurement of the same pointing direction. So the reading is
 * chosen by which one reproduces the reported tilt, per file, and the answer is surfaced in the
 * diagnostics panel rather than assumed. `tilt-track` is what remains when neither fits or no
 * high-rate log was supplied: tilt magnitude from the altimeter, azimuth from the velocity vector.
 */
import type { BrLowRow } from '../lib/types.ts';

/** Anything carrying a time and a quaternion, already on the log being described. */
export interface QuatEpoch {
  t: number;
  quat: [number, number, number, number];
}

export type AttitudeSource = 'axis-is-pointing' | 'body-to-world' | 'tilt-track';

export interface AttitudeAt {
  /** unit pointing direction in ENU (east, north, up) */
  axis: [number, number, number];
  /** radians about that axis, from the low-rate roll angle */
  roll: number;
}

export interface AttitudeResolver {
  source: AttitudeSource;
  /** median |predicted - reported| tilt over the flight, degrees; the evidence behind `source` */
  agreementDeg: number;
  /** fraction of epochs that agreed within 5 degrees */
  withinTol: number;
  /** @param hint horizontal velocity direction, used for the azimuth only in `tilt-track` mode */
  at(t: number, hint?: { e: number; n: number }): AttitudeAt | null;
}

/** Rotate body +z by a unit quaternion (xyz first, w last). */
function bodyUp(q: [number, number, number, number]): [number, number, number] {
  const [x, y, z, w] = q;
  return [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)];
}

function norm3(v: [number, number, number]): [number, number, number] | null {
  const n = Math.hypot(v[0], v[1], v[2]);
  return n > 1e-6 ? [v[0] / n, v[1] / n, v[2] / n] : null;
}

/** Tilt is unchanged by flipping nose and tail, so compare the two without committing to a sign. */
function tiltError(axisUp: number, tiltDeg: number): number {
  const a = (Math.acos(Math.max(-1, Math.min(1, Math.abs(axisUp)))) * 180) / Math.PI;
  const b = Math.min(tiltDeg, 180 - tiltDeg);
  return Math.abs(a - b);
}

function median(xs: number[]): number {
  if (!xs.length) return NaN;
  const s = [...xs].sort((a, b) => a - b);
  return s[s.length >> 1];
}

/**
 * @param high high-rate rows, already on the low-rate time axis
 * @param low  low-rate rows, for the independent tilt and the roll angle
 */
export function resolveAttitude(high: QuatEpoch[] | undefined, low: BrLowRow[]): AttitudeResolver {
  const lt = low.map((r) => r.t);
  const tiltAt = (t: number): number => {
    const n = lt.length;
    if (!n) return NaN;
    if (t <= lt[0]) return low[0].tilt;
    if (t >= lt[n - 1]) return low[n - 1].tilt;
    let a = 0, b = n - 1;
    while (b - a > 1) {
      const m = (a + b) >> 1;
      if (lt[m] <= t) a = m; else b = m;
    }
    const d = lt[b] - lt[a];
    const f = d > 0 ? (t - lt[a]) / d : 0;
    return low[a].tilt + (low[b].tilt - low[a].tilt) * f;
  };

  const mk = (source: AttitudeSource, agreementDeg: number, withinTol: number): AttitudeResolver => {
    const ht = (high ?? []).map((r) => r.t);
    const rollAt = (t: number): number => {
      const n = lt.length;
      if (!n) return 0;
      let a = 0, b = n - 1;
      while (b - a > 1) {
        const m = (a + b) >> 1;
        if (lt[m] <= t) a = m; else b = m;
      }
      return ((low[t - lt[a] < lt[b] - t ? a : b].roll ?? 0) * Math.PI) / 180;
    };
    return {
      source,
      agreementDeg,
      withinTol,
      at(t: number, hint?: { e: number; n: number }): AttitudeAt | null {
        let axis: [number, number, number] | null = null;
        if (high?.length && ht.length) {
          let a = 0, b = ht.length - 1;
          if (t < ht[0]) t = ht[0];
          if (t > ht[b]) t = ht[b];
          while (b - a > 1) {
            const m = (a + b) >> 1;
            if (ht[m] <= t) a = m; else b = m;
          }
          const r = high[t - ht[a] < ht[b] - t ? a : b];
          const q = r.quat;
          if (source === 'axis-is-pointing') {
            // The vector part points along the airframe, but the half-angle loses which end is the
            // nose. The independently reported tilt decides it: past 90 degrees the nose is below
            // the horizon, so the chosen sign must put the axis' vertical component on that side.
            const v = norm3([q[0], q[1], q[2]]);
            if (v) {
              const tl = tiltAt(t);
              const wantUp = Math.cos((tl * Math.PI) / 180) < 0 ? -1 : 1;
              let sgn = Math.sign(v[2]) === 0 ? 0 : Math.sign(v[2]) === wantUp ? 1 : -1;
              // Horizontal flight: the tilt says nothing, so fall back to the direction of travel.
              if (sgn === 0) sgn = hint && v[0] * hint.e + v[1] * hint.n < 0 ? -1 : 1;
              axis = [v[0] * sgn, v[1] * sgn, v[2] * sgn];
            }
          } else if (source === 'body-to-world') {
            axis = norm3(bodyUp(q));
          }
        }
        if (!axis) {
          // No usable quaternion: the tilt *magnitude* is measured, its azimuth is not, so tip the
          // airframe over in the direction of travel. A rocket flying a weathercock has the two
          // nearly coincident; a tumbling one does not, and the marker is labelled nominal.
          const tl = (tiltAt(t) * Math.PI) / 180;
          const hs = hint ? Math.hypot(hint.e, hint.n) : 0;
          const he = hs > 1e-6 ? [hint!.e / hs, hint!.n / hs] : [0, 1];
          axis = [Math.sin(tl) * he[0], Math.sin(tl) * he[1], Math.cos(tl)];
          const n = norm3(axis);
          if (!n) return null;
          axis = n;
        }
        return { axis, roll: rollAt(t) };
      },
    };
  };

  if (!high?.length || !low.length) return mk('tilt-track', NaN, 0);
  const stride = Math.max(1, Math.floor(high.length / 4000));
  const errA: number[] = [];
  const errB: number[] = [];
  for (let i = 0; i < high.length; i += stride) {
    const r = high[i];
    const tl = tiltAt(r.t);
    if (!Number.isFinite(tl)) continue;
    const b = bodyUp(r.quat);
    if (Number.isFinite(b[0])) errA.push(tiltError(b[2], tl));
    const v = norm3([r.quat[0], r.quat[1], r.quat[2]]);
    if (v) errB.push(tiltError(v[2], tl));
  }
  const score = (errs: number[]) => ({
    med: median(errs),
    frac: errs.length ? errs.filter((e) => e < 5).length / errs.length : 0,
  });
  const sa = score(errA);
  const sb = score(errB);
  // An airframe flying straight up makes both readings agree, so require a real margin, not just a
  // pass: the reading wins only if it explains the tilt much better than the other one does.
  const okA = sa.frac > 0.9 && sa.med < 3;
  const okB = sb.frac > 0.9 && sb.med < 3;
  // An airframe flying straight up makes both readings agree, so passing is not enough: the winner
  // must explain the tilt better by a real margin, or the log genuinely cannot tell the two apart
  // and the marker is drawn from the tilt and the track instead, and labelled as such.
  const MARGIN = 2;
  if (okB && sa.med - sb.med >= MARGIN) return mk('axis-is-pointing', sb.med, sb.frac);
  if (okA && sb.med - sa.med >= MARGIN) return mk('body-to-world', sa.med, sa.frac);
  return mk('tilt-track', Math.min(sa.med, sb.med), Math.max(sa.frac, sb.frac));
}
