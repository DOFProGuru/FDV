/** Display formatting. Every number in the app goes through here so NaN reads as a gap, not a zero. */

/** A real minus sign: the hyphen on a figure like -8 ft reads as a dash in a dark UI. */
const minus = (s: string): string => s.replace(/-/g, '\u2212');

function group(v: number, digits: number): string {
  return minus(v.toLocaleString('en-US', { maximumFractionDigits: digits }));
}

export function ft(v: number, digits = 0): string {
  return Number.isFinite(v) ? group(v, digits) : '\u2014';
}

export function fps(v: number, digits = 0): string {
  return Number.isFinite(v) ? group(v, digits) : '\u2014';
}
/** Feet, but in miles once it is more than a few thousand - drift reads better in miles. */
export function distance(ftValue: number): string {
  if (!Number.isFinite(ftValue)) return '—';
  return ftValue >= 3000 ? `${(ftValue / 5280).toFixed(2)} mi` : `${group(Math.round(ftValue), 0)} ft`;
}

/** Seconds of flight time as `1:45.3`, with a leading minus before the clock started. */
export function tCode(t: number): string {
  if (!Number.isFinite(t)) return '—';
  const s = Math.abs(t);
  const m = Math.floor(s / 60);
  const r = s - m * 60;
  const body = `${m}:${r.toFixed(1).padStart(4, '0')}`;
  return (t < 0 ? '−' : '') + body;
}

/** A mission clock label: `T+1:45.3`, or `T−0:04.5` while the clock counts up to zero. */
export function tLabel(t: number): string {
  return t < 0 ? `T−${tCode(-t)}` : `T+${tCode(t)}`;
}

/** A signed offset in seconds, with the unit attached: `+0.412 s`. */
export function offsetS(t: number, digits = 3): string {
  if (!Number.isFinite(t)) return '—';
  return `${t >= 0 ? '+' : '−'}${Math.abs(t).toFixed(digits)} s`;
}

export function deg(v: number, digits = 1): string {
  return Number.isFinite(v) ? `${minus(v.toFixed(digits))}°` : '—';
}

export function pct(v: number, digits = 0): string {
  return Number.isFinite(v) ? `${(100 * v).toFixed(digits)}%` : '—';
}

export function fixLabel(f: number): string {
  return f >= 3 ? '3-D' : f === 2 ? '2-D' : f === 1 ? 'dead-reckoned' : 'no fix';
}

/** A count or a plain number with thousands separators; NaN reads as a gap rather than a zero. */
export function num(v: number, digits = 0): string {
  return Number.isFinite(v) ? group(v, digits) : '—';
}

/** Descent speed under the drogue, or the reason there is not one. */
export function descentSpeed(v: number | undefined): string {
  return v === undefined ? 'not seen' : `${fps(v)} ft/s`;
}
