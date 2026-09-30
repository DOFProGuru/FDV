/**
 * Turning files into something `reconstruct` can read.
 *
 * A drop of three files arrives with no guarantee of which is which, so identity is decided from
 * the contents (`looksLike*`), not the filename: a log can be called `blue_raven_low.csv` and be
 * the GPS tracker's output, and the app should still say so rather than plot nonsense.
 */
import { derivePad } from '../lib/fusion.ts';
import { looksLikeBlueRaven, parseBlueRaven } from '../lib/parsers/blueRaven.ts';
import { looksLikeGps, parseGps } from '../lib/parsers/gps.ts';
import type { BrHighRow, BrLowRow, Dialect, FlightData, GpsRow } from '../lib/types.ts';

export interface RawFile {
  name: string;
  text: string;
}

export interface Bundle {
  data: FlightData;
  lowName?: string;
  highName?: string;
  gpsName?: string;
  highHz: number;
}

export type FileKind = 'br-low' | 'br-high' | 'gps' | 'unknown';

export function identify(text: string): FileKind {
  if (looksLikeGps(text)) return 'gps';
  if (looksLikeBlueRaven(text)) {
    const head = text.slice(0, 40_000).toLowerCase();
    if (/@\s*log_hir/.test(head)) return 'br-high';
    if (/@\s*log_low/.test(head)) return 'br-low';
    const line = head.split('\n').find((l) => l.includes(',')) ?? '';
    const cols = new Set(line.split(',').map((c) => c.trim()));
    if ([...cols].some((c) => c.startsWith('quat') || c.startsWith('gyro_'))) return 'br-high';
    return 'br-low';
  }
  return 'unknown';
}

export function bundleFiles(files: RawFile[]): Bundle {
  const lows: { rows: BrLowRow[]; name: string; dialect: Dialect; flightDate?: string }[] = [];
  const highs: { rows: BrHighRow[]; name: string }[] = [];
  const gps: { rows: GpsRow[]; name: string; dialect: Dialect; flightDate?: string; warnings: string[] }[] = [];
  const warnings: string[] = [];

  for (const f of files) {
    const text = f.text.replace(/^\uFEFF/, '');
    if (!text.trim()) { warnings.push(`${f.name}: empty file.`); continue; }
    const kind = identify(text);
    if (kind === 'gps') {
      const p = parseGps(text);
      if (p.rows.length) gps.push({ rows: p.rows, name: f.name, dialect: p.dialect, flightDate: p.flightDate, warnings: p.warnings });
      else warnings.push(`${f.name}: recognised as a GPS log but no usable fixes in it.`);
      continue;
    }
    if (kind === 'br-low' || kind === 'br-high') {
      const p = parseBlueRaven(text, f.name);
      if (p.kind === 'low') {
        if (p.low.length) lows.push({ rows: p.low, name: f.name, dialect: p.dialect, flightDate: p.flightDate });
        else warnings.push(`${f.name}: recognised as a Blue Raven low-rate log but no rows parsed from it.`);
      } else if (p.high?.length) highs.push({ rows: p.high, name: f.name });
      else warnings.push(`${f.name}: recognised as a Blue Raven high-rate log but no rows parsed from it.`);
      for (const w of p.warnings) warnings.push(`${f.name}: ${w}`);
      continue;
    }
    warnings.push(`${f.name}: not a Blue Raven or GPS log; skipped.`);
  }

  if (!lows.length)
    throw new Error('No Blue Raven low-rate log among these files. The reconstruction needs the altimeter log and the GPS log together.');
  if (!gps.length)
    throw new Error('No GPS log among these files. Both are needed: the altimeter gives the shape of the flight, the tracker fixes where it was.');
  if (lows.length > 1) {
    warnings.push(`${lows.length} low-rate logs supplied; using ${lows[0].name} and ignoring the rest.`);
    lows.splice(1);
  }
  if (gps.length > 1) {
    warnings.push(`${gps.length} GPS logs supplied; using ${gps[0].name} and ignoring the rest.`);
    gps.splice(1);
  }

  const low = lows[0];
  const high = highs[0];
  const g = gps[0];
  for (const w of g.warnings) warnings.push(`${g.name}: ${w}`);
  if (highs.length > 1) warnings.push(`${highs.length} high-rate logs supplied; using ${high!.name} and ignoring the rest.`);

  const { pad, notes } = derivePad(g.rows, low.rows);
  for (const n of notes) warnings.push(n);

  // The high-rate log's own clock is joined to the low-rate one inside `reconstruct`, from the
  // shared sync counter; the nominal rate is only ever used to label the log.
  const highHz = high && high.rows.length > 1 ? Math.round((high.rows.length - 1) / (high.rows[high.rows.length - 1].t - high.rows[0].t)) : 0;

  return {
    data: {
      brLow: low.rows,
      brHigh: high?.rows,
      gps: g.rows,
      pad,
      meta: {
        brDialect: low.dialect,
        gpsDialect: g.dialect,
        brFileName: low.name,
        gpsFileName: g.name,
        flightDate: low.flightDate ?? g.flightDate,
        warnings,
      },
    },
    lowName: low.name,
    highName: high?.name,
    gpsName: g.name,
    highHz,
  };
}

/** The bundled sample flights, from the manifest the generator writes. */
export interface FlightEntry {
  id: string;
  label: string;
  low: string;
  high?: string;
  gps: string;
  maxAltFt: number;
  dur: number;
  mach: number;
  maxG: number;
  driftMi: number;
  gyroSatS: number;
  gpsBadFix: number;
  gpsMaxGapS: number;
}

export async function fetchManifest(): Promise<FlightEntry[]> {
  const r = await fetch('data/index.json', { cache: 'no-cache' });
  if (!r.ok) throw new Error(`Could not read data/index.json (${r.status}).`);
  return (await r.json()) as FlightEntry[];
}

export async function fetchFlight(e: FlightEntry): Promise<RawFile[]> {
  const want = [e.low, e.high, e.gps].filter(Boolean) as string[];
  const texts = await Promise.all(
    want.map(async (p) => {
      const r = await fetch(`data/${p}`, { cache: 'no-cache' });
      if (!r.ok) throw new Error(`Could not read data/${p} (${r.status}).`);
      return r.text();
    }),
  );
  return want.map((p, i) => ({ name: p, text: texts[i] }));
}

export async function readAll(list: FileList | File[]): Promise<RawFile[]> {
  const out: RawFile[] = [];
  for (const f of Array.from(list)) {
    if (f.size > 64 * 1024 * 1024) { out.push({ name: f.name, text: '' }); continue; }
    out.push({ name: f.name, text: await f.text() });
  }
  return out;
}
