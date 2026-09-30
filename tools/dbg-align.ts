import { alignByCrossCorrelation, alignByAnchors } from '../src/lib/signal.ts';
const shape = (s: number) => s < 0 ? 0 : s < 2 ? 800 * s : s < 16 ? 1600 - 98 * (s - 2) : Math.max(-90, -98 * (s - 16) * Math.exp(-(s - 16) / 3) - 20 * (1 - Math.exp(-(s - 16) / 2)));
const t: number[] = [], v1: number[] = [], v2: number[] = [];
let seed = 7; const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
for (let i = 0; i < 12000; i++) { t.push(i / 50); v1.push(shape(i / 50)); }
for (let i = 0; i < t.length; i++) v2.push(shape(t[i] + 2.14) + (rnd() - 0.5) * 4);
const ref = { t, v: v1 }, mov = { t, v: v2 };
for (let s = 1.6; s <= 2.8; s += 0.1) {
  const r = alignByCrossCorrelation(ref, mov, { minShift: s, maxShift: s, step: 0.001, gridHz: 50 });
  console.log('shift', s.toFixed(2), 'score', r.score.toFixed(4));
}
console.log('wide', alignByCrossCorrelation(ref, mov, { minShift: -8, maxShift: 8, step: 0.02, gridHz: 20 }));
console.log('anchored', alignByAnchors(ref, mov, { windowS: 1.5, step: 0.01, gridHz: 50 }));
