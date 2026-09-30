/** Signal helpers: interpolation, resampling, robust scales, cross-correlation alignment. */

export interface Series {
  t: number[];
  v: number[];
}

/** Linear interpolation with clamping outside the sample window; NaN-aware. */
export function interp(s: Series, t: number): number {
  const { t: ts, v } = s;
  if (!ts.length) return NaN;
  if (t <= ts[0]) return v[0];
  if (t >= ts[ts.length - 1]) return v[v.length - 1];
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid;
  }
  const a = ts[lo], b = ts[hi];
  const f = b > a ? (t - a) / (b - a) : 0;
  return v[lo] + (v[hi] - v[lo]) * f;
}

/** Nearest index in a sorted time array. */
export function nearest(ts: number[], t: number): number {
  if (!ts.length) return -1;
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid;
  }
  return Math.abs(ts[lo] - t) <= Math.abs(ts[hi] - t) ? lo : hi;
}

/** Drop NaN samples, keeping t/v paired and time-sorted. */
export function clean(t: number[], v: number[]): Series {
  const ts: number[] = [];
  const vs: number[] = [];
  for (let i = 0; i < t.length; i++) {
    if (Number.isFinite(t[i]) && Number.isFinite(v[i])) { ts.push(t[i]); vs.push(v[i]); }
  }
  return { t: ts, v: vs };
}

export function median(xs: number[]): number {
  if (!xs.length) return NaN;
  const a = [...xs].sort((p, q) => p - q);
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : (a[m - 1] + a[m]) / 2;
}

/** Robust scale from the median absolute deviation, scaled to a Gaussian sigma. */
export function madScale(xs: number[]): number {
  const med = median(xs);
  if (!Number.isFinite(med)) return NaN;
  return 1.4826 * median(xs.map((x) => Math.abs(x - med)));
}

/** Centred moving average; used to smooth per-sample confidence into a stable process-noise schedule. */
export function boxcar(xs: number[], halfWidth: number): number[] {
  const n = xs.length;
  const out = new Array<number>(n);
  let acc = 0;
  for (let i = 0; i < n; i++) {
    acc += xs[i];
    if (i - halfWidth - 1 >= 0) acc -= xs[i - halfWidth - 1];
    const lo = Math.max(0, i - halfWidth);
    const hi = i;
    out[i] = acc / (hi - lo + 1);
  }
  return out;
}

/**
 * Align two clocks by maximising the normalised cross-correlation of a shared feature.
 * Returns the offset to ADD to `moving`'s timestamps so it lines up with `reference`.
 *
 * The vertical velocity profile is the right feature: it is large, unambiguous, and both
 * sensors measure it (inertial navigation at 50 Hz, GPS Doppler at 10 Hz).
 */
export function alignByCrossCorrelation(
  reference: Series,
  moving: Series,
  opts: { minShift?: number; maxShift?: number; step?: number; gridHz?: number } = {},
): { offset: number; score: number; runnerUpScore: number } {
  const minShift = opts.minShift ?? -6;
  const maxShift = opts.maxShift ?? 6;
  const step = opts.step ?? 0.01;
  const gridHz = opts.gridHz ?? 20;

  const t0 = Math.max(reference.t[0], moving.t[0] + minShift);
  const t1 = Math.min(reference.t[reference.t.length - 1], moving.t[moving.t.length - 1] + maxShift);
  if (!(t1 > t0)) return { offset: 0, score: 0, runnerUpScore: -2 };

  const n = Math.max(4, Math.floor((t1 - t0) * gridHz));
  const grid = new Array<number>(n);
  const refVals = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    grid[i] = t0 + ((t1 - t0) * i) / (n - 1);
    refVals[i] = interp(reference, grid[i]);
  }
  const refMean = refVals.reduce((a, b) => a + b, 0) / n;
  const refDc = refVals.map((v) => v - refMean);
  const refVar = refDc.reduce((a, b) => a + b * b, 0) / n || 1;

  // Scale: 1 means the shifted series explains the reference perfectly, 0 means it is no better
  // than a constant. Deliberately *not* Pearson: correlation is invariant to amplitude, and a
  // coast-phase velocity profile is very nearly a straight line, so a shifted-and-rescaled copy of
  // it scores ~0.999 and the peak wanders by half a second. A plain squared-error cost has no such
  // degeneracy - shifting a ramp by 0.7 s costs 0.7 * its slope in residual, which is what pins
  // the offset to milliseconds.
  let best = { offset: 0, score: -2 };
  const scores: { offset: number; score: number }[] = [];
  for (let s = minShift; s <= maxShift + 1e-9; s += step) {
    let se = 0;
    for (let i = 0; i < n; i++) {
      const d = interp(moving, grid[i] - s) - refDc[i] - refMean;
      se += d * d;
    }
    const score = 1 - se / n / refVar;
    scores.push({ offset: s, score });
    if (score > best.score) best = { offset: s, score };
  }
  // A coast-dominated flight can produce a near-tied second peak; report it so the UI can say
  // "this alignment is not confident" rather than presenting one number as fact.
  let runnerUp = -2;
  for (let i = 1; i < scores.length - 1; i++) {
    const isPeak = scores[i].score >= scores[i - 1].score && scores[i].score >= scores[i + 1].score;
    if (isPeak && Math.abs(scores[i].offset - best.offset) > 0.25 && scores[i].score > runnerUp) runnerUp = scores[i].score;
  }
  return { offset: best.offset, score: best.score, runnerUpScore: runnerUp };
}

/**
 * Unique, well-separated features that both sensors see: peak speed (motor burnout) and the
 * apogee zero-crossing. Global cross-correlation over a whole flight is fragile - the long descent
 * is quasi-periodic, so many offsets score about equally well - but these two anchors are not.
 */
export function anchors(s: Series): { tPeak: number; tApogee: number } | null {
  if (s.t.length < 10) return null;
  let iPeak = 0;
  for (let i = 0; i < s.v.length; i++) if (Math.abs(s.v[i]) > Math.abs(s.v[iPeak])) iPeak = i;
  // Require a real ascent: a log that never exceeds a few ft/s carries no usable anchor.
  if (Math.abs(s.v[iPeak]) < 50) return null;
  let tApogee = NaN;
  for (let i = iPeak + 1; i < s.v.length; i++) {
    if (s.v[i] <= 0 && s.v[i - 1] > 0) {
      const f = s.v[i - 1] / (s.v[i - 1] - s.v[i]);
      tApogee = s.t[i - 1] + f * (s.t[i] - s.t[i - 1]);
      break;
    }
  }
  return { tPeak: s.t[iPeak], tApogee };
}

/**
 * Coarse offset from the peak-speed anchor (and apogee as a cross-check), then a local
 * cross-correlation refinement. The refinement window is deliberately narrow: the anchors already
 * pin the offset to well under a second, and a narrow window is what stops the quasi-periodic
 * descent from winning.
 */
export function alignByAnchors(
  reference: Series,
  moving: Series,
  opts: { windowS?: number; step?: number; gridHz?: number } = {},
): { offset: number; score: number; runnerUpScore: number; coarse: number; anchorAgreementS: number | null } {
  const ref = anchors(reference);
  const mov = anchors(moving);
  const coarseCandidates: number[] = [];
  let anchorAgreementS: number | null = null;
  if (ref && mov) {
    const dPeak = ref.tPeak - mov.tPeak;
    coarseCandidates.push(dPeak);
    if (Number.isFinite(ref.tApogee) && Number.isFinite(mov.tApogee)) {
      const dApogee = ref.tApogee - mov.tApogee;
      anchorAgreementS = Math.abs(dApogee - dPeak);
      coarseCandidates.push((dPeak + dApogee) / 2);
    }
  }
  const window = opts.windowS ?? 1.5;
  const step = opts.step ?? 0.01;
  const gridHz = opts.gridHz ?? 50;

  const tryAround = (c: number) =>
    alignByCrossCorrelation(reference, moving, { minShift: c - window, maxShift: c + window, step, gridHz });

  let best = { offset: coarseCandidates[0] ?? 0, score: -2, runnerUpScore: -2, coarse: coarseCandidates[0] ?? 0 };
  for (const c of coarseCandidates.length ? coarseCandidates : [0]) {
    const r = tryAround(c);
    if (r.score > best.score) best = { offset: r.offset, score: r.score, runnerUpScore: r.runnerUpScore, coarse: c };
  }
  // A weak anchor match (or no anchors at all) means the whole record must be searched, and the
  // result deserves to be treated as uncertain.
  if (!coarseCandidates.length) {
    const wide = alignByCrossCorrelation(reference, moving, { minShift: -12, maxShift: 12, step: 0.05, gridHz: 20 });
    const r = tryAround(wide.offset);
    best = { offset: r.offset, score: r.score, runnerUpScore: r.runnerUpScore, coarse: wide.offset };
  }
  return { ...best, anchorAgreementS };
}
