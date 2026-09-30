import type { BrHighRow, BrLowRow } from './types.ts';

/**
 * Joining the two Blue Raven logs onto one time axis.
 *
 * Each log numbers its samples from its own first record, so the 50 Hz and 500 Hz files can start
 * anywhere relative to each other. What ties them together is the `sync_code` both devices stamp
 * every sample: a millisecond counter shared between the two files, which rolls over every 250 ms.
 * Matching that counter recovers the offset to about a millisecond.
 *
 * The counter's roll period is also the limit of what it can say: an offset of 137 ms and one of
 * 387 ms look identical to it. So the search is confined to half a roll period around the difference
 * the two logs' own time axes already claim, which is exact for a joint export carrying a common
 * elapsed-time column and is where any real offset will sit otherwise. A materially non-zero result
 * is therefore "the remainder modulo 250 ms", and is flagged as such: pinning the multiple down
 * needs the record datestamps, which CSV exports do not carry.
 */

const ROLL_MS = 250;

/** Distance between two readings of a counter that wraps every ROLL_MS, in ms. */
function rollDist(a: number, b: number): number {
  const d = Math.abs(a - b) % ROLL_MS;
  return Math.min(d, ROLL_MS - d);
}

function nearestIndex(t: number[], x: number): number {
  if (!t.length) return -1;
  let lo = 0, hi = t.length - 1;
  if (x <= t[0]) return 0;
  if (x >= t[hi]) return hi;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (t[m] <= x) lo = m; else hi = m;
  }
  return x - t[lo] <= t[hi] - x ? lo : hi;
}

export interface SyncAlignment {
  /** seconds to add to the high-rate time base to place it on the low-rate time base */
  offsetS: number;
  /** mean |sync-code| discrepancy at the chosen offset, ms; ~0 means the two logs really do agree */
  residualMs: number;
  /** low-rate samples that had a high-rate neighbour, i.e. the evidence behind the fit */
  used: number;
  /** the offset is not ~0, so only its remainder modulo the 250 ms roll period is established */
  aliased: boolean;
}

/**
 * Estimate the high-rate log's offset from the low-rate log's time axis. Null when there is too
 * little data, or when no offset in the searched window makes the counters agree - which happens if
 * the two files are from different flights, or a parser has mis-read the sync code.
 */
export function syncAlignment(low: BrLowRow[], high: BrHighRow[]): SyncAlignment | null {
  if (low.length < 50 || high.length < 50) return null;
  const tLow = low.map((r) => r.t);
  const tHigh = high.map((r) => r.t);
  // Probe evenly across the flight rather than use all a quarter-million samples: the offset is a
  // single number, and a few hundred well-spread epochs constrain it as well as the whole log does.
  const probes: number[] = [];
  const stride = Math.max(1, Math.floor(low.length / 240));
  for (let i = 0; i < low.length; i += stride) probes.push(i);

  // What the axes themselves say, plus or minus half a roll: outside that window the counter cannot
  // tell one candidate from another anyway, so there is nothing to gain from looking.
  const epochMs = (tLow[0] - tHigh[0]) * 1000;
  const from = Math.ceil(epochMs - ROLL_MS / 2), to = Math.floor(epochMs + ROLL_MS / 2);

  let best: { cost: number; offsetS: number; residualMs: number; used: number } | null = null;
  for (let off = from; off <= to; off += 1) {
    let cost = 0, sum = 0, used = 0;
    for (const li of probes) {
      // Looking for the high-rate sample of the same instant: t_high = t_low - offset.
      const hi = nearestIndex(tHigh, tLow[li] - off / 1000);
      if (hi < 0) continue;
      const d = rollDist(low[li].sync, high[hi].sync);
      if (d > 60) continue; // a neighbour this far off is a dropout, not the same instant
      cost += d * d;
      sum += d;
      used++;
    }
    if (used < probes.length * 0.4) continue;
    if (!best || cost < best.cost) best = { cost, offsetS: off / 1000, residualMs: sum / used, used };
  }
  if (!best) return null;
  // A genuine joint export lines up to well under a millisecond. If it does not, the counter has
  // been mis-read or these logs are not from the same flight, and any offset taken from it is
  // meaningless, so say so by declining to align.
  if (best.residualMs > 8) return null;
  return {
    offsetS: best.offsetS,
    residualMs: best.residualMs,
    used: best.used,
    aliased: Math.abs(best.offsetS) > 0.02,
  };
}

/** Re-stamp the high-rate rows onto the low-rate time axis. */
export function shiftHigh(high: BrHighRow[], offsetS: number): BrHighRow[] {
  if (!offsetS) return high;
  return high.map((r) => ({ ...r, t: r.t + offsetS }));
}
