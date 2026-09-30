/**
 * Colours used by <canvas>, which cannot read the CSS custom properties. These are the same literals
 * as the `:root` block in style.css, which the HTML legend now refers to by `var(...)`; change a
 * colour here and there, and nothing else needs to follow.
 */
export const C = {
  bg: '#0a090d',
  panel: '#131118',
  line: '#26222f',
  grid: '#1c1926',
  text: '#e9e5ef',
  muted: '#8c8598',
  dim: '#5b5566',
  fused: '#4fd6be',
  br: '#f0a94b',
  gps: '#7aa2f7',
  baro: '#8c8598',
  /** the accelerometer trace, a colour of its own so it is not confused with the amber track */
  imu: '#a299b4',
  warn: '#ffb454',
  bad: '#ff6b6b',
  good: '#7ee081',
} as const;

export const MONO = 'ui-monospace, "SF Mono", "JetBrains Mono", Menlo, Consolas, monospace';
export const SANS = '-apple-system, BlinkMacSystemFont, "Segoe UI", Inter, Roboto, sans-serif';

/** Which solution carried each fused epoch; drives the trajectory colouring. */
export const KIND = { fused: 0, br: 1, gps: 2 } as const;
export const KIND_COLOR: Record<number, string> = {
  [KIND.fused]: C.fused,
  [KIND.br]: C.br,
  [KIND.gps]: C.gps,
};
