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

/**
 * Header cell -> lookup key.
 *
 * Every run of characters that is not a letter or a digit is a separator, so the units a vendor
 * puts in brackets (`Baro_Press_(atm)`, `Flight_Time_(s)`), the sigils of a counted column
 * (`#TOT`, `>40`) and the spaces of a human-readable one (`TRACKER Lat`) all arrive at the key the
 * alias lists use. The alternative - spelling every punctuation variant in every alias list - is a
 * list that has to grow one export at a time.
 */
export function columnKey(s: string): string {
  return s.toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_|_$/g, '');
}

/** Build header -> column index, normalised with `columnKey`. First occurrence of a name wins. */
export function columnIndex(header: string[]): Map<string, number> {
  const m = new Map<string, number>();
  header.forEach((h, i) => {
    const k = columnKey(h);
    if (k && !m.has(k)) m.set(k, i);
  });
  return m;
}

/** Resolve the first present column among aliases; -1 when none found. */
export function findColumn(idx: Map<string, number>, aliases: string[]): number {
  for (const a of aliases) {
    const i = idx.get(columnKey(a));
    if (i !== undefined) return i;
  }
  return -1;
}

/**
 * The header line of a file, normalised, without reading the body.
 *
 * Enough to decide *what* a file is (`identify`) before committing to parsing a quarter of a
 * million rows of the wrong kind. `limit` bounds the prefix scanned: a header line is a few hundred
 * bytes, and this must not copy a 20 MB log to look at its first line.
 */
export function headerKeys(text: string, limit = 32_000): string[] {
  const head = text.length > limit ? text.slice(0, limit) : text;
  const lines = head.replace(/\r\n?/g, '\n').split('\n');
  const first = lines.find((l) => l.trim() !== '' && !l.startsWith('#') && !l.startsWith('//'));
  if (!first || !first.includes(',')) return [];
  return splitLine(first, sniff(lines)).map(columnKey).filter((k) => k !== '');
}

/** True when the file's header carries any of these columns, under any of their punctuated forms. */
export function hasColumn(keys: Set<string>, names: string[]): boolean {
  return names.some((n) => keys.has(columnKey(n)));
}

export function num(v: string | undefined): number {
  if (v === undefined) return NaN;
  const s = v.trim();
  if (s === '' || s === '-' || /^(na|n\/a|nan|null|missing)$/i.test(s)) return NaN;
  // A clock time is not a number, and it is the one token the strip below cannot be trusted with:
  // `07:33:19.293` would come back as 73319.293, which looks exactly like a plausible elapsed
  // seconds and is neither. Files that carry a time of day are read by the clock helpers below.
  if (s.includes(':')) return NaN;
  return Number(s.replace(/[^0-9eE+\-.]/g, ''));
}

/**
 * The first of these columns that actually holds numbers, in the order given; -1 when none does.
 *
 * Column *names* are the vendor's and are not evidence of contents: two exports both called `Time`
 * can hold elapsed seconds and `07:33:19.293` respectively. Deciding from the first values that are
 * there, rather than from the header, is what keeps a clock out of the time axis.
 */
export function firstNumericColumn(rows: string[][], cols: number[]): number {
  for (const c of cols) {
    if (c < 0) continue;
    const n = Math.min(rows.length, 2000);
    for (let i = 0; i < n; i++) {
      if (Number.isFinite(num(rows[i][c]))) return c;
    }
    if (rows.length > n) {
      for (let i = n; i < rows.length; i += 97) {
        if (Number.isFinite(num(rows[i][c]))) return c;
      }
    }
  }
  return -1;
}

// --- wall clock ---------------------------------------------------------------

/** Where a file writes its wall clock. Any subset may be present; the first that yields wins. */
export interface ClockColumns {
  /** a single ISO-ish stamp column: `2026-08-08T06:43:42.313` */
  stamp?: number;
  /** a date column: `2026-08-08`, `8/8/2026` */
  date?: number;
  /** a time-of-day column: `06:43:42.313` */
  clock?: number;
  /** three separate columns, the way the Blue Raven export writes it */
  year?: number;
  month?: number;
  day?: number;
}

/** `2026-08-08`, `2026/08/08`, `8/8/2026` -> parts. Two-digit years are this century's. */
export function parseDateToken(s: string): { y: number; mo: number; d: number } | undefined {
  const t = s.trim();
  let m = /^(\d{4})[-/.](\d{1,2})[-/.](\d{1,2})$/.exec(t);
  if (m) return { y: +m[1], mo: +m[2], d: +m[3] };
  m = /^(\d{1,2})[-/.](\d{1,2})[-/.](\d{2,4})$/.exec(t);
  if (m) {
    const y = +m[3];
    // Month first: the vendor ships from the US, and 8/8/2026 is the same either way, which is the
    // only case a day-month ambiguity can actually be caught in.
    return { y: y < 100 ? 2000 + y : y, mo: +m[1], d: +m[2] };
  }
  return undefined;
}

/** `07:33:19.293`, `7:33:19`, `073319.293` -> seconds past midnight. */
export function parseClockToken(s: string): number {
  const t = s.trim();
  let m = /^(\d{1,2}):(\d{2})(?::(\d{2})(?:[.,](\d{1,6}))?)?$/.exec(t);
  if (m) return +m[1] * 3600 + +m[2] * 60 + +(m[3] ?? 0) + (m[4] ? Number(`0.${m[4]}`) : 0);
  m = /^(\d{1,2})(\d{2})(\d{2})(?:[.,](\d{1,6}))?$/.exec(t);
  if (m) return +m[1] * 3600 + +m[2] * 60 + +m[3] + (m[4] ? Number(`0.${m[4]}`) : 0);
  return NaN;
}

const pad2 = (n: number) => String(n).padStart(2, '0');

/**
 * Date and time-of-day -> `YYYY-MM-DDTHH:MM:SS.mmmZ`, read as UTC.
 *
 * Undefined unless both tokens read. The result is a *stamp*, not a clock reference: see
 * `readClock` for why the two logs are never joined by it.
 */
export function composeIso(dateTok: string, clockTok: string): string | undefined {
  const d = parseDateToken(dateTok);
  const secs = clockTok.trim() === '' ? 0 : parseClockToken(clockTok);
  if (!d || !Number.isFinite(secs) || d.mo < 1 || d.mo > 12 || d.d < 1 || d.d > 31) return undefined;
  const total = Math.round(secs * 1000);
  const h = Math.floor(total / 3_600_000) % 24;
  const mi = Math.floor(total / 60_000) % 60;
  const s = Math.floor(total / 1000) % 60;
  const ms = total % 1000;
  return `${String(d.y).padStart(4, '0')}-${pad2(d.mo)}-${pad2(d.d)}T${pad2(h)}:${pad2(mi)}:${pad2(s)}.${String(ms).padStart(3, '0')}Z`;
}

/**
 * Row -> UTC ISO-8601 stamp, from whichever of the shapes above the file uses.
 *
 * The stamp buys two things and only two: the flight's calendar date, and - for a log with no
 * elapsed column of its own - an internal time axis. It is deliberately *not* used to line the
 * altimeter's clock up with the tracker's, because the two devices are not synchronised and a real
 * pair of logs from one session disagree by as much as an hour.
 */
export function readClock(c: ClockColumns): (r: string[]) => string | undefined {
  return (r) => {
    if (c.stamp !== undefined && c.stamp >= 0) {
      const raw = (r[c.stamp] ?? '').trim();
      if (/[0-9]/.test(raw)) {
        const norm = raw.includes('T') ? raw : raw.replace(' ', 'T');
        const iso = /Z|[+-]\d{2}:?\d{2}$/.test(norm) ? norm : `${norm}Z`;
        if (Number.isFinite(Date.parse(iso))) return iso;
      }
    }
    let dateTok = c.date !== undefined && c.date >= 0 ? (r[c.date] ?? '').trim() : '';
    if (!dateTok && c.year !== undefined && c.year >= 0 && c.month !== undefined && c.month >= 0 && c.day !== undefined && c.day >= 0) {
      const y = (r[c.year] ?? '').trim(), mo = (r[c.month] ?? '').trim(), d = (r[c.day] ?? '').trim();
      if (y && mo && d) dateTok = `${y}-${mo}-${d}`;
    }
    if (!dateTok) return undefined;
    return composeIso(dateTok, c.clock !== undefined && c.clock >= 0 ? (r[c.clock] ?? '') : '');
  };
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
