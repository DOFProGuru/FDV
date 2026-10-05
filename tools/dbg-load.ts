/**
 * The "Open logs…" path, in a terminal.
 *
 * The drop zone does three things with a set of files: decides what each one is, bundles them into
 * the pair `reconstruct` reads, and reconstructs. The checks that judge numbers (`sample/verify.mjs`)
 * go straight to the parsers and to the simulator's truth, and the browser smoke test only ever
 * loads the bundled flights - so a user's own three files, arriving with headers nobody has seen and
 * clocks that disagree, are the one path with no headless way in. This is it.
 *
 *   node --experimental-strip-types tools/dbg-load.ts path/to/logs/*.csv
 *
 * It asserts nothing: it is a diagnostic, not a check. What it prints is what the app concluded -
 * the kind each file was judged to be, what came out of it, which files the bundler wanted and
 * which it refused, and what the fusion made of what survived.
 */
import { readFileSync } from 'node:fs';
import { reconstruct, type Reconstruction } from '../src/lib/fusion.ts';
import { parseBlueRaven } from '../src/lib/parsers/blueRaven.ts';
import { parseGps } from '../src/lib/parsers/gps.ts';
import { bundleFiles, identify, type Bundle, type FileKind, type RawFile } from '../src/ui/load.ts';

interface Read {
  file: RawFile;
  kind: FileKind;
  rows: { t: number }[];
  warnings: string[];
  dialect: string;
  flightDate?: string;
  detail: string;
}

const span = (t: number[]): string => (t.length > 1 ? `${t[0].toFixed(2)} → ${t[t.length - 1].toFixed(2)} s` : 'one row');
const tRel = (s: number, digits = 1): string => `T${s >= 0 ? '+' : '-'}${Math.abs(s).toFixed(digits)}`;
const rate = (n: number, t: number[]): string =>
  t.length > 1 && t[t.length - 1] > t[0] ? ` @ ${(n / (t[t.length - 1] - t[0])).toFixed(0)} Hz` : '';
const section = (title: string): void => { console.log(`\n${title}`); };

const reads: Read[] = [];
const unreadable: string[] = [];
for (const p of process.argv.slice(2)) {
  const name = p.split('/').pop() ?? p;
  let text: string;
  try {
    text = readFileSync(p, 'utf8').replace(/^\uFEFF/, '');
  } catch (e) {
    unreadable.push(`${name} - ${e instanceof Error ? e.message.split('\n')[0].replace(/^ENOENT: no such file or directory, /, '') : e}`);
    continue;
  }
  const file = { name, text };
  const kind = identify(text);
  const base = { file, kind, warnings: [] as string[], dialect: '', flightDate: undefined as string | undefined, detail: '' };
  if (kind === 'gps') {
    const g = parseGps(text);
    const nofix = g.rows.filter((r) => !(r.fixType > 0)).length;
    reads.push({
      ...base, warnings: g.warnings, dialect: g.dialect, flightDate: g.flightDate, rows: g.rows,
      detail: g.rows.length ? `first ${g.rows[0].lat.toFixed(5)}, ${g.rows[0].lon.toFixed(5)} ${g.rows[0].altFt.toFixed(0)} ft at ${g.rows[0].iso}${nofix ? ` (${nofix} with no fix)` : ''}` : 'no usable fixes',
    });
  } else if (kind === 'br-low' || kind === 'br-high') {
    const b = parseBlueRaven(text, name);
    const rows = b.kind === 'low' ? b.low : b.high ?? [];
    reads.push({
      ...base, warnings: b.warnings, dialect: b.dialect, flightDate: b.flightDate, rows,
      detail: b.kind === 'low' ? `first t=${rows[0]?.t.toFixed(2)} s, fer 0x${((rows[0] as { fer?: number })?.fer ?? 0).toString(16)}` : `first q=[${(rows[0] as { quat?: number[] })?.quat?.map((x) => x.toFixed(3)).join(', ') ?? '-'}]`,
    });
  } else {
    reads.push({ ...base, rows: [], detail: 'nothing in the header was recognised' });
  }
}

if (!process.argv.slice(2).length) {
  console.log('usage: node --experimental-strip-types tools/dbg-load.ts path/to/logs/*.csv');
  process.exit(2);
}

section('files — what each was judged to be, and what came out of it');
for (const u of unreadable) console.log(`  unreadable ${u}`);
for (const r of reads) {
  console.log(`  ${r.kind.padEnd(8)} ${r.file.name}`);
  console.log(`           ${r.rows.length} rows${rate(r.rows.length, r.rows.map((x) => x.t))} · t ${span(r.rows.map((x) => x.t))}${r.dialect ? ` · ${r.dialect}` : ''}${r.flightDate ? ` · date ${r.flightDate}` : ''}`);
  if (r.detail) console.log(`           ${r.detail}`);
  for (const w of r.warnings) console.log(`           ! ${w}`);
}

let bundle: Bundle;
try {
  bundle = bundleFiles(reads.filter((r) => r.kind !== 'unknown').map((r) => r.file));
} catch (e) {
  console.log(`\nnot a flight: ${e instanceof Error ? e.message : e}`);
  process.exit(1);
}

section('bundle — what reconstruct was handed');
const d = bundle.data;
console.log(`  low rate     ${bundle.lowName} · ${d.brLow.length} rows${rate(d.brLow.length, d.brLow.map((r) => r.t))}, ${d.meta.brDialect}`);
console.log(`  high rate    ${bundle.highName ? `${bundle.highName} · ${d.brHigh?.length ?? 0} rows @ ${bundle.highHz} Hz` : 'none: no attitude marker, no 500 Hz acceleration trace'}`);
console.log(`  gps          ${bundle.gpsName} · ${d.gps.length} fixes${rate(d.gps.length, d.gps.map((r) => r.t))}, ${d.meta.gpsDialect}`);
console.log(`  pad          ${d.pad.lat === 0 && d.pad.lon === 0 ? `${d.pad.altFt.toFixed(0)} ft MSL, position unknown (no on-pad GPS fix to derive it from)` : `${d.pad.lat.toFixed(5)}, ${d.pad.lon.toFixed(5)} · ${d.pad.altFt.toFixed(0)} ft MSL`}`);
console.log(`  flight date  ${d.meta.flightDate ?? 'unknown: no file carried a wall clock'}`);
for (const w of d.meta.warnings) console.log(`  ! ${w}`);

let recon: Reconstruction;
try {
  recon = reconstruct(d);
} catch (e) {
  console.log(`\nnot reconstructed: ${e instanceof Error ? e.message : e}`);
  process.exit(1);
}

section('reconstruct');
console.log(`  samples      ${recon.samples.length} over ${recon.durationS.toFixed(2)} s`);
console.log(`  events       ${recon.events.map((e) => `${e.code}@${tRel(e.t, 2)}`).join('  ') || 'none'}`);
console.log(`  apogee       ${recon.stats.maxAltFt.toFixed(0)} ft AGL at ${tRel(recon.stats.maxAltT)} · ${recon.stats.maxVelFps.toFixed(0)} ft/s · ${recon.stats.maxAccelG.toFixed(1)} g · drift ${(recon.stats.driftFt / 5280).toFixed(2)} mi`);
console.log(`  gps clock    ${recon.clock.gpsOffsetS.toFixed(3)} s (score ${recon.clock.score.toFixed(2)}, runner-up ${recon.clock.runnerUpScore.toFixed(2)}${recon.clock.anchorAgreementS !== null ? `, anchors agree to ${recon.clock.anchorAgreementS.toFixed(2)} s` : ''})`);
console.log(`  br sync      ${recon.brSync ? `${(-recon.brSync.offsetS).toFixed(3)} s apart, ${recon.brSync.residualMs.toFixed(2)} ms residual${recon.brSync.aliased ? ', aliased to 250 ms' : ''}` : 'none: too little of both logs, or no offset in the searched window makes the counters agree'}`);
console.log(`  pad rest     ${recon.padRest ? `${recon.padRest.seconds.toFixed(1)} s pinned to zero velocity, ${recon.padRest.fixes} fixes, to ${tRel(recon.padRest.untilT)}` : 'none: the GPS track holds no still stretch at the head of the flight'}`);
console.log(`  registration ${Number.isFinite(recon.registration.rmsFt) ? `${recon.registration.yawDeg.toFixed(1)}° ± ${recon.registration.yawSigmaDeg.toFixed(1)} · ${recon.registration.rmsFt.toFixed(0)} ft over ${recon.registration.pairs} pairs, ${(recon.registration.inlierFraction * 100).toFixed(0)} % inliers` : 'not fitted: too few GPS epochs to say how the body is turned, so the frame is taken as given'}`);
for (const w of recon.warnings) console.log(`  ! ${w}`);
