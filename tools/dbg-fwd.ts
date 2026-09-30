// Where does the FORWARD pass stop looking like a small correction to the inertial track?
import { readFileSync } from 'node:fs';
import { parseBlueRaven } from '../src/lib/parsers/blueRaven.ts';
import { parseGps } from '../src/lib/parsers/gps.ts';
import { derivePad, reconstruct, DEFAULT_TUNING, AxisFilter, gpsTrack } from '../src/lib/fusion.ts';

const name = process.argv[2] ?? 'f17-nominal';
const rd = (s: string) => readFileSync(`public/data/${name}${s}`, 'utf8');
const low = parseBlueRaven(rd('_blue_raven_low.csv'), 'br');
const high = parseBlueRaven(rd('_blue_raven_high.csv'), 'brh');
const gpsP = parseGps(rd('_gps.csv'), 'gps');
const { pad } = derivePad(gpsP.rows, low.low);
const rec = reconstruct({ brLow: low.low, brHigh: high.high, gps: gpsP.rows, pad, meta: { name, warnings: [] } } as never);
const reg = rec.brOnly;
const off = rec.clock.gpsOffsetS;
const raw = gpsTrack(gpsP.rows, pad);
const gps = { ...raw, t: raw.t.map((t) => t + off) };
const sigmaPos = rec.noise.sigmaPosFt, sigmaVel = rec.noise.sigmaVelFps;
const d0 = Math.max(30, sigmaPos * 3), d1 = Math.max(3, sigmaVel * 2), d2 = 1.5;
console.log('scales d =', d0.toFixed(1), d1.toFixed(2), d2);

const tBr = reg.t;
const f = [0, 1, 2].map(() => new AxisFilter(d0, d1, d2));
let j = 0;
const first = 4000;
for (let i = 0; i < tBr.length; i++) {
  if (i > 0) for (const g of f) g.predict(tBr[i] - tBr[i - 1], DEFAULT_TUNING.qBase, DEFAULT_TUNING.qBias);
  for (const g of f) { g.xPred.push([...g.x]); g.PPred.push([...g.P]); }
  while (j < gps.t.length && gps.t[j] <= tBr[i] + 1e-9) {
    if (gps.t[j] >= tBr[0] - 0.5) {
      const speed = Math.hypot(gps.v[j].e, gps.v[j].n, gps.v[j].u);
      const rp = (sigmaPos * 1) ** 2 + (DEFAULT_TUNING.posDynamic * speed) ** 2;
      const rv = (sigmaVel * 1) ** 2 + (DEFAULT_TUNING.velDynamic * speed) ** 2;
      for (let a = 0; a < 3; a++) {
        const pG = [gps.p[j].e, gps.p[j].n, gps.p[j].u][a], p0 = [reg.p[i].e, reg.p[i].n, reg.p[i].u][a];
        const vG = [gps.v[j].e, gps.v[j].n, gps.v[j].u][a], v0 = [reg.v[i].e, reg.v[i].n, reg.v[i].u][a];
        if (Number.isFinite(pG)) f[a].update(0, pG - p0, rp);
        if (Number.isFinite(vG)) f[a].update(1, vG - v0, rv);
      }
    }
    j++;
  }
  for (const g of f) { g.xUpd.push([...g.x]); g.PUpd.push([...g.P]); }
}

// Report the real-unit state every ~2 s after index `first`.
const ev = Math.max(1, Math.round(2 / (tBr[1] - tBr[0])));
console.log('  t       dp_e       dp_n       dp_u     |x[2] bias| (e,n,u)   P00(e)');
for (let i = first; i < tBr.length; i += ev) {
  const x = f.map((g) => g.xUpd[i]);
  const P0 = f.map((g) => g.PUpd[i][0]);
  console.log(
    tBr[i].toFixed(1).padStart(7),
    (x[0][0] * d0).toExponential(2).padStart(10),
    (x[1][0] * d0).toExponential(2).padStart(10),
    (x[2][0] * d0).toExponential(2).padStart(10),
    '  ' + x.map((z) => (z[2] * d2).toFixed(1)).join(' '),
    '  ' + P0.map((p) => p.toExponential(1)).join(' '),
  );
}
// First index where any axis's real dp exceeds 3x the total flight altitude.
for (let i = first; i < tBr.length; i++) {
  const m = Math.max(...f.map((g) => Math.abs(g.xUpd[i][0] * d0)));
  if (m > 30000) {
    console.log(`first |dp| > 30000 ft at i=${i} t=${tBr[i].toFixed(2)} m=${m.toExponential(2)}`);
    for (let k = Math.max(0, i - 6); k <= i; k++) {
      console.log(`  i=${k} t=${tBr[k].toFixed(3)} x0=[${f[0].xUpd[k].map((v) => v.toExponential(2)).join(' ')}] P00=${f[0].PUpd[k][0].toExponential(2)} reg.p=${reg.p[k].e.toFixed(0)}`);
    }
    break;
  }
}
