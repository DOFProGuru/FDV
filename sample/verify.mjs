// Verification harness: reconstructs each synthetic flight from its CSV logs and scores the
// result against the simulator's truth. Truth is used ONLY here, never by the app.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { parseBlueRaven } from '../src/lib/parsers/blueRaven.ts';
import { parseGps } from '../src/lib/parsers/gps.ts';
import { derivePad, reconstruct } from '../src/lib/fusion.ts';

const here = dirname(fileURLToPath(import.meta.url));
const ids = process.argv.slice(2);

/** Linear interpolation over truth samples. */
function truthAt(samples, t, field, axis) {
  const ts = samples.map((s) => s.t);
  if (t <= ts[0]) return samples[0][field][axis];
  const last = ts.length - 1;
  if (t >= ts[last]) return samples[last][field][axis];
  let lo = 0, hi = last;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ts[m] <= t) lo = m; else hi = m; }
  const f = (t - ts[lo]) / (ts[hi] - ts[lo]);
  return samples[lo][field][axis] + (samples[hi][field][axis] - samples[lo][field][axis]) * f;
}

function rms(xs) { return Math.sqrt(xs.reduce((a, b) => a + b * b, 0) / Math.max(1, xs.length)); }

for (const id of ids) {
  const dataDir = join(here, '..', 'public', 'data');
  const low = parseBlueRaven(readFileSync(join(dataDir, `${id}_blue_raven_low.csv`), 'utf8'));
  const high = parseBlueRaven(readFileSync(join(dataDir, `${id}_blue_raven_high.csv`), 'utf8'));
  const gpsP = parseGps(readFileSync(join(dataDir, `${id}_gps.csv`), 'utf8'));
  const truth = JSON.parse(readFileSync(join(here, 'truth', `${id}_truth.json`), 'utf8'));

  const gps = gpsP.rows;
  const derived = derivePad(gps, low.low);
  const pad = derived.pad;
  const rec = reconstruct({
    brLow: low.low,
    brHigh: high.high,
    gps,
    pad,
    meta: { brDialect: low.dialect, gpsDialect: gpsP.dialect, warnings: [...low.warnings, ...gpsP.warnings] },
  });

  // Map the altimeter's clock onto truth's clock using the liftoff event, which both know.
  const liftoff = rec.events.find((e) => e.code === 'liftoff');
  const tLiftoffTruth = truth.events.find((e) => e.type === 'liftoff')?.t ?? 0;
  const base = tLiftoffTruth - (liftoff?.t ?? 0);

  const errFused = [], errBr = [], errGps = [];
  const errFusedV = [], errBrV = [];
  for (let i = 0; i < rec.fused.t.length; i++) {
    const tt = rec.fused.t[i] + base;
    if (tt < 0 || tt > truth.samples.at(-1).t) continue;
    const pT = [0, 1, 2].map((a) => truthAt(truth.samples, tt, 'p', a));
    const vT = [0, 1, 2].map((a) => truthAt(truth.samples, tt, 'v', a));
    const dF = Math.hypot(rec.fused.p[i].e - pT[0], rec.fused.p[i].n - pT[1], rec.fused.p[i].u - pT[2]);
    const dB = Math.hypot(rec.brOnly.p[i].e - pT[0], rec.brOnly.p[i].n - pT[1], rec.brOnly.p[i].u - pT[2]);
    errFused.push(dF); errBr.push(dB);
    errFusedV.push(Math.hypot(rec.fused.v[i].e - vT[0], rec.fused.v[i].n - vT[1], rec.fused.v[i].u - vT[2]));
    errBrV.push(Math.hypot(rec.brOnly.v[i].e - vT[0], rec.brOnly.v[i].n - vT[1], rec.brOnly.v[i].u - vT[2]));
  }
  for (let i = 0; i < rec.gpsOnly.t.length; i++) {
    const tt = rec.gpsOnly.t[i] + base;
    if (tt < 0 || tt > truth.samples.at(-1).t) continue;
    const pT = [0, 1, 2].map((a) => truthAt(truth.samples, tt, 'p', a));
    errGps.push(Math.hypot(rec.gpsOnly.p[i].e - pT[0], rec.gpsOnly.p[i].n - pT[1], rec.gpsOnly.p[i].u - pT[2]));
  }

  // The altimeter log carries absolute t_s values, so the app's inertial base already IS truth
  // time; the tracker's stamps are rebased to its own first sample, which is gpsStartS late.
  const expectClock = truth.gpsStartS;
  const s = rec.stats;
  const apogeeTruth = truth.apogeeFt;
  console.log(`\n=== ${id} ===`);
  console.log(`  pad                ${pad.lat.toFixed(5)}, ${pad.lon.toFixed(5)}  ${pad.altFt.toFixed(0)} ft   (truth ${truth.pad.lat.toFixed(5)}, ${truth.pad.lon.toFixed(5)} ${truth.pad.altFt} ft)`);
  console.log(`  clock offset       ${rec.clock.gpsOffsetS.toFixed(3)} s of ${expectClock.toFixed(3)} s (err ${(rec.clock.gpsOffsetS - expectClock).toFixed(3)})  r=${rec.clock.score.toFixed(3)} anchors agree to ${(rec.clock.anchorAgreementS ?? NaN).toFixed(2)} s`);
  console.log(`  registration       rms ${rec.registration.rmsFt.toFixed(1)} ft  inliers ${(rec.registration.inlierFraction * 100).toFixed(0)}%  yaw ${rec.registration.yawDeg.toFixed(1)} deg`);
  for (const n of derived.notes) console.log(`  pad note           ${n}`);
  console.log(`  noise estimated    pos ${rec.noise.sigmaPosFt.toFixed(1)} ft  vel ${rec.noise.sigmaVelFps.toFixed(2)} ft/s`);
  console.log(`  apogee             ${s.maxAltFt.toFixed(0)} ft AGL   (truth ${apogeeTruth.toFixed(0)} ft, err ${Math.abs(s.maxAltFt - apogeeTruth).toFixed(0)} ft)`);
  console.log(`  max vel            ${s.maxVelFps.toFixed(0)} ft/s   max ${s.maxAccelG.toFixed(1)} G   mach ${s.maxMach.toFixed(2)}   drift ${(s.driftFt / 5280).toFixed(2)} mi`);
  console.log(`  pos err vs truth   fused rms ${rms(errFused).toFixed(1)} ft (max ${Math.max(...errFused).toFixed(0)})   BR-only rms ${rms(errBr).toFixed(1)} ft (max ${Math.max(...errBr).toFixed(0)})   GPS-only rms ${rms(errGps).toFixed(1)} ft`);
  console.log(`  vel err vs truth   fused rms ${rms(errFusedV).toFixed(2)} ft/s   BR-only rms ${rms(errBrV).toFixed(2)} ft/s`);
  console.log(`  events             ${rec.events.map((e) => `${e.code}@${e.t.toFixed(2)}`).join('  ')}`);
  console.log(`  truth events       ${truth.events.map((e) => `${e.type}@${e.t.toFixed(2)}`).join('  ')}`);
  for (const w of rec.warnings) console.log(`  ! ${w}`);
}
