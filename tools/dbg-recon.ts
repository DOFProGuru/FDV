import { readFileSync } from 'node:fs';
import { parseBlueRaven } from '../src/lib/parsers/blueRaven.ts';
import { parseGps } from '../src/lib/parsers/gps.ts';
import { derivePad, reconstruct, distrustSeries, inertialTrack } from '../src/lib/fusion.ts';

const name = process.argv[2] ?? 'f17-nominal';
const rd = (s: string) => readFileSync(`public/data/${name}${s}`, 'utf8');
const low = parseBlueRaven(rd('_blue_raven_low.csv'), `${name}_br`);
const high = parseBlueRaven(rd('_blue_raven_high.csv'), `${name}_brh`);
const gps = parseGps(rd('_gps.csv'), `${name}_gps`);
const { pad } = derivePad(gps.rows, low.low);
const rec = reconstruct({ brLow: low.low, brHigh: high.high, gps: gps.rows, pad, meta: { name, warnings: [] } } as never);
const conf = rec.samples.map((s) => s.brConfidence);
console.log('brConfidence min/max', Math.min(...conf).toFixed(3), Math.max(...conf).toFixed(3));
const dt = rec.samples.map((s, i) => (i ? s.t - rec.samples[i - 1].t : 0)).slice(1);
console.log('dt min/max', Math.min(...dt).toExponential(2), Math.max(...dt).toExponential(2));
const inr = inertialTrack(JSON.parse(readFileSync(`sample/truth/${name}_truth.json`,'utf8')) && [] );
const t = rec.fused.t;
const mx = (arr: number[]) => arr.reduce((a, b) => Math.max(a, Math.abs(b)), 0);
console.log('fused max |p|', mx(rec.fused.p.map((p) => p.e)), mx(rec.fused.p.map((p) => p.n)), mx(rec.fused.p.map((p) => p.u)));
console.log('brOnly max |p|', mx(rec.brOnly.p.map((p) => p.e)), mx(rec.brOnly.p.map((p) => p.n)), mx(rec.brOnly.p.map((p) => p.u)));
console.log('fused max |v|', mx(rec.fused.v.map((p) => p.e)), mx(rec.fused.v.map((p) => p.n)), mx(rec.fused.v.map((p) => p.u)));
console.log('brOnly max |v|', mx(rec.brOnly.v.map((p) => p.e)), mx(rec.brOnly.v.map((p) => p.n)), mx(rec.brOnly.v.map((p) => p.u)));
// where does it first exceed 2x the brOnly?
for (let i = 0; i < t.length; i++) {
  if (Math.abs(rec.fused.p[i].u) > 2 * (Math.abs(rec.brOnly.p[i].u) + 200)) { console.log('vertical diverges first at t=', t[i].toFixed(2), 'fused', rec.fused.p[i].u.toFixed(1), 'br', rec.brOnly.p[i].u.toFixed(1)); break; }
}
for (let i = 1; i < t.length; i++) {
  const dJump = Math.abs(rec.fused.v[i].u - rec.fused.v[i - 1].u);
  if (dJump > 5000) { console.log('velocity jump of', dJump.toFixed(0), 'ft/s at t=', t[i].toFixed(3), 'brVel there', rec.brOnly.v[i].u.toFixed(1)); break; }
}
