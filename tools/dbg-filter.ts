import { readFileSync } from 'node:fs';
import { parseBlueRaven } from '../src/lib/parsers/blueRaven.ts';
import { parseGps } from '../src/lib/parsers/gps.ts';
import { AxisFilter, derivePad, gpsTrack, reconstruct, DEFAULT_TUNING } from '../src/lib/fusion.ts';

const name = process.argv[2] ?? 'f17-nominal';
const rd = (s: string) => readFileSync(`public/data/${name}${s}`, 'utf8');
const low = parseBlueRaven(rd('_blue_raven_low.csv'), 'br');
const gpsP = parseGps(rd('_gps.csv'), 'gps');
const { pad } = derivePad(gpsP.rows, low.low);
const rec = reconstruct({ brLow: low.low, gps: gpsP.rows, pad, meta: { name, warnings: [] } } as never);
const reg = rec.brOnly;
const off = rec.clock.gpsOffsetS;
const raw = gpsTrack(gpsP.rows, pad);
const gps = { ...raw, t: raw.t.map((t) => t + off) };
const sigmaPos = rec.noise.sigmaPosFt / 1.3, sigmaVel = rec.noise.sigmaVelFps / 1.3;
console.log('sigmaPos', sigmaPos.toFixed(1), 'sigmaVel', sigmaVel.toFixed(2), 'clock', off.toFixed(3), 'gps n', gps.t.length);

const tBr = reg.t;
const f = [0, 1, 2].map(() => new AxisFilter(Math.max(30, sigmaPos * 3), Math.max(3, sigmaVel * 2), 1.5));
let j = 0;
for (let i = 0; i < tBr.length; i++) {
  if (i > 0) for (const g of f) g.predict(tBr[i] - tBr[i - 1], DEFAULT_TUNING.qBase, DEFAULT_TUNING.qBias); // distrust = 0 for this probe
  for (const g of f) { g.xPred.push([...g.x]); g.PPred.push([...g.P]); }
  let firstBig = false;
  while (j < gps.t.length && gps.t[j] <= tBr[i] + 1e-9) {
    if (gps.t[j] >= tBr[0] - 0.5) {
      const sp = Math.hypot(gps.v[j].e, gps.v[j].n, gps.v[j].u);
      const rp = (sigmaPos * 1) ** 2 + (DEFAULT_TUNING.posDynamic * sp) ** 2;
      const rv = (sigmaVel * 1) ** 2 + (DEFAULT_TUNING.velDynamic * sp) ** 2;
      for (let a = 0; a < 3; a++) {
        const pG = [gps.p[j].e, gps.p[j].n, gps.p[j].u][a], p0 = [reg.p[i].e, reg.p[i].n, reg.p[i].u][a];
        const vG = [gps.v[j].e, gps.v[j].n, gps.v[j].u][a], v0 = [reg.v[i].e, reg.v[i].n, reg.v[i].u][a];
        if (Number.isFinite(pG)) f[a].update(0, pG - p0, rp);
        if (Number.isFinite(vG)) f[a].update(1, vG - v0, rv);
        const big = f.some((g) => Math.abs(g.x[0]) > 20 || Math.abs(g.x[1]) > 5);
        if (big && !firstBig) {
          firstBig = true;
          console.log(`first big state at i=${i} t=${tBr[i].toFixed(2)} axis=${a}`);
          for (const [idx, g] of f.entries()) {
            console.log(`  axis${idx} x=[${g.x.map((v) => v.toExponential(2)).join(' ')}] P=[${g.P.map((v) => v.toExponential(1)).join(' ')}]`);
          }
          console.log(`  z_pos=${(gps.p[j].e - reg.p[i].e).toExponential(3)} z_vel=${(gps.v[j].e - reg.v[i].e).toExponential(3)} gps t=${gps.t[j].toFixed(3)} br t=${tBr[i].toFixed(3)}`);
          console.log(`  gpsPos=[${gps.p[j].e.toFixed(1)} ${gps.p[j].n.toFixed(1)} ${gps.p[j].u.toFixed(1)}] brPos=[${reg.p[i].e.toFixed(1)} ${reg.p[i].n.toFixed(1)} ${reg.p[i].u.toFixed(1)}]`);
        }
      }
    }
    j++;
  }
  for (const g of f) { g.xUpd.push([...g.x]); g.PUpd.push([...g.P]); }
}
const sm = f.map((g) => g.smooth());
const mx = (xs: number[]) => xs.reduce((a, b) => Math.max(a, Math.abs(b)), 0);
for (let a = 0; a < 3; a++) {
  console.log(`axis ${a}: max |xUpd| =`, mx(f[a].xUpd.map((x) => x[0])).toExponential(2), mx(f[a].xUpd.map((x) => x[1])).toExponential(2), mx(f[a].xUpd.map((x) => x[2])).toExponential(2));
  console.log(`       max |smoothed| =`, mx(sm[a].map((x) => x[0])).toExponential(2), mx(sm[a].map((x) => x[1])).toExponential(2), mx(sm[a].map((x) => x[2])).toExponential(2));
  const P0 = f[a].PPred[500], P1 = f[a].PPred[Math.floor(tBr.length / 2)], P2 = f[a].PPred[tBr.length - 10];
  console.log(`       P@500 ${P0.map((v) => v.toExponential(1)).join(' ')}`);
  console.log(`       P@mid ${P1.map((v) => v.toExponential(1)).join(' ')}`);
  console.log(`       P@end ${P2.map((v) => v.toExponential(1)).join(' ')}`);
}

// where does the backward pass blow up?
{
  const a = 0;
  const g = f[a];
  const n = g.xUpd.length;
  let worst = { i: -1, corr: 0, dx: 0, cNorm: 0 };
  let xs = [...g.xUpd[n - 1]] as [number, number, number];
  for (let i = n - 2; i >= 0; i--) {
    const Pi = (g as any).PPred[i + 1];
    const inv = (globalThis as any).__inv;
    // recompute C the same way smooth() does
    const Pu = g.PUpd[i] as number[];
    const F = ((g as any).Fs[i]) as number[];
    const det = Pi[0] * (Pi[4] * Pi[8] - Pi[5] * Pi[7]) - Pi[1] * (Pi[3] * Pi[8] - Pi[5] * Pi[6]) + Pi[2] * (Pi[3] * Pi[7] - Pi[4] * Pi[6]);
    const adj = [
      Pi[4] * Pi[8] - Pi[5] * Pi[7], Pi[2] * Pi[7] - Pi[1] * Pi[8], Pi[1] * Pi[5] - Pi[2] * Pi[4],
      Pi[5] * Pi[6] - Pi[3] * Pi[8], Pi[0] * Pi[8] - Pi[2] * Pi[6], Pi[2] * Pi[3] - Pi[0] * Pi[5],
      Pi[3] * Pi[7] - Pi[4] * Pi[6], Pi[1] * Pi[6] - Pi[0] * Pi[7], Pi[0] * Pi[4] - Pi[1] * Pi[3],
    ];
    const PF = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) PF[r * 3 + c] = Pu[r * 3 + 0] * F[c * 3 + 0] + Pu[r * 3 + 1] * F[c * 3 + 1] + Pu[r * 3 + 2] * F[c * 3 + 2];
    const C = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) C[r * 3 + c] = (PF[r * 3 + 0] * adj[0 * 3 + c] + PF[r * 3 + 1] * adj[1 * 3 + c] + PF[r * 3 + 2] * adj[2 * 3 + c]) / det;
    const dx = [xs[0] - g.xPred[i + 1][0], xs[1] - g.xPred[i + 1][1], xs[2] - g.xPred[i + 1][2]];
    const cn = Math.hypot(C[0], C[1], C[2]) / (Math.hypot(F[0], F[1], F[2]) || 1);
    const corr = Math.abs(C[0] * dx[0] + C[1] * dx[1] + C[2] * dx[2]);
    if (corr > worst.corr) worst = { i, corr, dx: dx[0], cNorm: cn };
    xs = [g.xUpd[i][0] + (C[0] * dx[0] + C[1] * dx[1] + C[2] * dx[2]), 0, 0];
  }
  console.log(`worst correction at i=${worst.i} t=${tBr[worst.i].toFixed(2)} |C dx|=${worst.corr.toExponential(2)} (scaled) dx=${worst.dx.toExponential(2)} |C_row|=${worst.cNorm.toFixed(2)}`);
}
