/** Small, tolerant CSV reader: delimiter sniffing, comment/blank skipping, quoted fields. */

export interface CsvTable {
  header: string[];
  /** index of header -> raw string value */
  rows: string[][];
  delimiter: string;
}

const QUOTE = '"';

function splitLine(line: string, delim: string): string[] {
  if (!line.includes(QUOTE)) return line.split(delim);
  const out: string[] = [];
  let cur = '';
  let inQ = false;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (inQ) {
      if (c === QUOTE) {
        if (line[i + 1] === QUOTE) { cur += QUOTE; i++; }
        else inQ = false;
      } else cur += c;
    } else if (c === QUOTE) inQ = true;
    else if (c === delim) { out.push(cur); cur = ''; }
    else cur += c;
  }
  out.push(cur);
  return out.map((s) => s.trim());
}

/** Sniff , ; \t | by counting occurrences on the first few data lines. */
function sniff(lines: string[]): string {
  const cands = [',', ';', '\t', '|'];
  const sample = lines.slice(0, 12);
  let best = ',';
  let bestScore = -1;
  for (const d of cands) {
    const counts = sample.map((l) => splitLine(l, d).length);
    if (!counts.length) continue;
    const mode = counts[0];
    if (mode < 2) continue;
    // Prefer the delimiter giving a consistent, high column count.
    const consistent = counts.filter((c) => c === mode).length / counts.length;
    const score = consistent * 100 + mode;
    if (score > bestScore) { bestScore = score; best = d; }
  }
  return best;
}

export function parseCsv(text: string): CsvTable {
  const lines = text.replace(/\r\n?/g, '\n').split('\n').filter((l) => l.trim() !== '' && !l.startsWith('#') && !l.startsWith('//'));
  if (!lines.length) return { header: [], rows: [], delimiter: ',' };
  const delimiter = sniff(lines);
  const header = splitLine(lines[0], delimiter).map((h) => h.replace(/^\uFEFF/, '').trim());
  const rows: string[][] = [];
  for (let i = 1; i < lines.length; i++) {
    const cells = splitLine(lines[i], delimiter);
    if (cells.length < Math.max(2, Math.floor(header.length * 0.5))) continue;
    rows.push(cells);
  }
  return { header, rows, delimiter };
}

/** Build header -> column index, lower-cased and whitespace-normalised. */
export function columnIndex(header: string[]): Map<string, number> {
  const m = new Map<string, number>();
  header.forEach((h, i) => {
    const k = h.toLowerCase().replace(/[\s\-\u2011-\u2015]/g, '_').replace(/_+/g, '_').replace(/^_|_$/g, '');
    if (!m.has(k)) m.set(k, i);
  });
  return m;
}

/** Resolve the first present column among aliases; -1 when none found. */
export function findColumn(idx: Map<string, number>, aliases: string[]): number {
  for (const a of aliases) {
    const k = a.toLowerCase().replace(/[\s\-\u2011-\u2015]/g, '_').replace(/_+/g, '_').replace(/^_|_$/g, '');
    const i = idx.get(k);
    if (i !== undefined) return i;
  }
  return -1;
}

export function num(v: string | undefined): number {
  if (v === undefined) return NaN;
  const s = v.trim();
  if (s === '' || s === '-' || /^(na|n\/a|nan|null|missing)$/i.test(s)) return NaN;
  const n = Number(s.replace(/[^0-9eE+\-.]/g, ''));
  return n;
}

/**
 * Decode a bitfield token.
 *
 * The flight-event registers are documented as hex bitmasks and the vendor writes them zero-padded
 * with no radix prefix (`180`, `3A7`), so a token made only of digits is still hex: `180` is bits
 * 7+8 (both velocity-not-positive flags), not one hundred and eighty. Reading it as decimal sets
 * bits 4-7 instead and makes every log look like the main and 3rd channels fired on the pad.
 * Decimal is honoured only when the file says so (`180d`), plus the usual `0x`/`h` decorations.
 */
export function parseBitmask(v: string | undefined): number {
  if (v === undefined) return NaN;
  const s = v.trim().replace(/^0x/i, '').replace(/h$/i, '');
  if (s === '') return NaN;
  if (/^[0-9]+d$/i.test(s)) return Number(s.slice(0, -1));
  if (!/^[0-9a-f]+$/i.test(s)) return NaN;
  return parseInt(s, 16);
}
