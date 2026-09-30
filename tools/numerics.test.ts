// Unit checks for the numerics that the reconstruction depends on.
import { procrustesWeighted, yawFitWeighted, matVec, matMul, transpose, jacobiEigen, ident } from '../src/lib/linalg.ts';
import { alignByAnchors, alignByCrossCorrelation } from '../src/lib/signal.ts';
import { AxisFilter } from '../src/lib/fusion.ts';
import type { QuatEpoch } from '../src/ui/attitude.ts';

let fails = 0;
function check(name: string, ok: boolean, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  if (!ok) fails++;
}

// --- eigendecomposition ------------------------------------------------------
{
  const A = [4, 1, 0, 1, 3, 0, 0, 0, 2];
  const { values, vectors } = jacobiEigen(A as never);
  // [[4,1],[1,3]] has eigenvalues (7 +- sqrt(5))/2 = 2.382, 4.618; plus 2 from the z axis.
  const okVals = values[0] < values[1] && values[1] < values[2] &&
    Math.abs(values[0] - 2) < 1e-6 && Math.abs(values[1] - 2.381966) < 1e-5 && Math.abs(values[2] - 4.618034) < 1e-5;
  // V must be orthonormal and diagonalise A.
  const VtAV = matMul(matMul(transpose(vectors), A as never), vectors);
  const offDiag = Math.hypot(VtAV[1], VtAV[2], VtAV[3], VtAV[5], VtAV[6], VtAV[7]);
  check('jacobiEigen diagonalises a symmetric matrix', okVals && offDiag < 1e-7, `vals=${values.map((v) => v.toFixed(4))} offdiag=${offDiag.toExponential(1)}`);
}

// --- Procrustes: the Blue Raven's rail-heading rotation is a permutation ------
{
  // down-range == North, cross-range == West (a proper rotation of ENU by -90 deg about Up).
  const c = Math.cos(-Math.PI / 2), s = Math.sin(-Math.PI / 2);
  const Rz = [c, -s, 0, s, c, 0, 0, 0, 1];
  const truth = [120, -40, 15];
  const pts: [number, number, number][] = [];
  for (let i = 0; i < 400; i++) {
    const e = 100 * Math.sin(i * 0.07), n = 900 * (1 - Math.cos(i * 0.05)), u = 500 * i * 0.02;
    pts.push([e, n, u]);
  }
  const ys = pts.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0] + truth[0], r[1] + truth[1], r[2] + truth[2]] as [number, number, number];
  });
  const fit = procrustesWeighted(pts, ys, pts.map(() => 1));
  const err = Math.hypot(fit.t[0] - truth[0], fit.t[1] - truth[1], fit.t[2] - truth[2]);
  const rerr = Math.max(...[0, 1, 2].flatMap((i) => [0, 1, 2].map((j) => Math.abs(fit.R[i * 3 + j] - (Rz as never)[i * 3 + j]))));
  check('procrustes recovers an exact 90-degree rotation + translation', err < 1e-6 && rerr < 1e-6, `t err=${err.toExponential(1)} R err=${rerr.toExponential(1)} rms=${fit.rms.toExponential(1)}`);
}

// --- Procrustes: outliers from a tumble must not rotate the whole fit ---------
{
  const th = (30 * Math.PI) / 180;
  const Rz = [Math.cos(th), -Math.sin(th), 0, Math.sin(th), Math.cos(th), 0, 0, 0, 1];
  const pts: [number, number, number][] = [];
  // A real flight wanders in all three axes (wind, weathercock, drift); a cloud confined to a
  // plane would leave the twist about its normal genuinely unidentifiable.
  for (let i = 0; i < 300; i++) pts.push([900 * Math.sin(i * 0.02) + 30 * i * 0.05, 600 * i * 0.01 + 120 * Math.cos(i * 0.05), 300 * i * 0.03]);
  const ys = pts.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0], r[1], r[2]] as [number, number, number];
  });
  for (let i = 0; i < 40; i++) ys[i] = [ys[i][0] + 900, ys[i][1] - 1400, ys[i][2] + 700]; // divergence window
  const clean = pts.slice(40);
  const fit = yawFitWeighted(clean, ys.slice(40), clean.map(() => 1));
  check('yaw fit is unbiased by a diverged segment', Math.abs(fit.yawDeg - 30) < 0.5, `yaw=${fit.yawDeg.toFixed(2)} deg (expect 30), rms=${fit.rms.toFixed(1)}`);

  // Near-vertical, windless flight: the horizontal lever arm is a few feet, so the rail heading is
  // NOT knowable. The estimator must say so instead of inventing an angle.
  const vert: [number, number, number][] = [];
  for (let i = 0; i < 300; i++) vert.push([2 * Math.sin(i * 0.3), 1.5 * Math.cos(i * 0.3), 40 * i]);
  const vy = vert.map((p) => {
    const r = matVec(Rz as never, p);
    return [r[0] + 3, r[1] - 2, r[2] + 1] as [number, number, number];
  });
  const straight = yawFitWeighted(vert, vy, vert.map(() => 1), 15);
  check('yaw fit reports an unobservable rail heading for a vertical flight', straight.yawSigmaDeg > 5,
    `yaw sigma=${straight.yawSigmaDeg.toFixed(1)} deg, spread=${straight.horizSpreadFt.toFixed(1)} ft`);
}

// --- clock alignment ---------------------------------------------------------
{
  // Convention check first, with an unambiguous feature: the returned offset is ADDED to the
  // moving series' timestamps. A bump at 25 s matching a bump at 20 s must therefore be -5 s.
  const t: number[] = [], ref: number[] = [], mov: number[] = [];
  for (let i = 0; i < 4000; i++) {
    const s = i / 50;
    t.push(s);
    ref.push(Math.exp(-((s - 20) ** 2) / 2));
    mov.push(Math.exp(-((s - 25) ** 2) / 2));
  }
  const spike = alignByCrossCorrelation({ t, v: ref }, { t, v: mov }, { minShift: -10, maxShift: 10, step: 0.02 });
  check('alignByCrossCorrelation returns the offset to ADD to the moving series', Math.abs(spike.offset + 5) < 0.03,
    `got ${spike.offset.toFixed(3)} s (expect -5), r=${spike.score.toFixed(3)}`);
  const spikeA = alignByAnchors({ t, v: ref }, { t, v: mov }, { windowS: 1.5, step: 0.01, gridHz: 50 });
  check('anchor alignment agrees on the sign convention', Math.abs(spikeA.offset + 5) < 0.05, `got ${spikeA.offset.toFixed(3)} s`);
}
{
  // A realistic vertical-velocity profile: sharp boost, linear coast, parachute descent.
  const shape = (s: number) => s < 0 ? 0 : s < 2 ? 800 * s : s < 16 ? 1600 - 98 * (s - 2) : Math.max(-90, -98 * (s - 16) * Math.exp(-(s - 16) / 3) - 20 * (1 - Math.exp(-(s - 16) / 2)));
  // The moving series is built 2.14 s *ahead* of the reference, so its stamps sit 2.14 s early and
  // +2.14 must be added to them - which is exactly the documented convention.
  const offsetTrue = 2.14;
  const t: number[] = [], v1: number[] = [], v2: number[] = [];
  for (let i = 0; i < 12000; i++) { t.push(i / 50); v1.push(shape(i / 50)); }
  for (let i = 0; i < t.length; i++) v2.push(shape(t[i] + offsetTrue) + (Math.random() - 0.5) * 4);
  const a = alignByAnchors({ t, v: v1 }, { t, v: v2 }, { windowS: 1.5, step: 0.01, gridHz: 50 });
  check('anchor alignment recovers a flight-profile clock offset', Math.abs(a.offset - offsetTrue) < 0.03,
    `got ${a.offset.toFixed(3)} s of ${offsetTrue}, fit=${a.score.toFixed(4)}, anchors agree to ${(a.anchorAgreementS ?? NaN).toFixed(2)} s`);
}

// --- small-angle sanity: the identity is a valid rotation fit -----------------
{
  const pts: [number, number, number][] = [[0, 0, 0], [10, 0, 0], [0, 20, 0], [0, 0, 30]];
  const fit = procrustesWeighted(pts, pts.map((p) => [...p] as [number, number, number]), pts.map(() => 1));
  check('procrustes on identical data returns identity and no offset',
    fit.R.every((v, i) => Math.abs(v - (ident() as never)[i]) < 1e-9) && fit.t.every((v) => Math.abs(v) < 1e-9),
    `rms=${fit.rms.toExponential(1)}`);
}


// --- error-state filter + RTS smoother ---------------------------------------
{
  const dt = 0.02, T = 180, n = Math.round(T / dt);
  const t: number[] = [], accel: number[] = [];
  for (let i = 0; i < n; i++) {
    const s = i * dt;
    t.push(s);
    // boost, coast, parachute descent: a shape with real curvature in every phase
    accel.push(s < 2 ? 900 : s < 16 ? -32.2 : -32.2 + 300 * Math.exp(-(s - 16) / 1.5) - 28 * (1 - Math.exp(-(s - 16) / 2)));
  }
  // true vertical state
  const pTrue = new Array(n).fill(0), vTrue = new Array(n).fill(0);
  for (let i = 1; i < n; i++) {
    vTrue[i] = vTrue[i - 1] + 0.5 * dt * (accel[i] + accel[i - 1]);
    pTrue[i] = pTrue[i - 1] + 0.5 * dt * (vTrue[i] + vTrue[i - 1]);
  }
  // inertial nominal: the same shape plus a divergence that grows once the airframe tumbles
  const pNom = pTrue.map((p, i) => p - 0.0006 * t[i] ** 2 * (1 + 4 * Math.max(0, t[i] - 20) / 20));
  const vNom = pNom.map((_, i) => (i + 1 < n ? (pNom[i + 1] - pNom[i]) / dt : (pNom[i] - pNom[i - 1]) / dt));
  let seed = 11;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff - 0.5) * 2;
  const sigmaP = 56, sigmaV = 3, R = sigmaP ** 2, Rv = sigmaV ** 2;

  const f = new AxisFilter(3 * sigmaP, 2 * sigmaV, 1.5);
  for (let i = 0; i < n; i++) {
    if (i > 0) f.predict(dt, 0.35 + Math.max(0, (t[i] - 20) / 160) * 900, 0.004);
    f.xPred.push([...f.x]); f.PPred.push([...f.P]);
    // Fixes arrive at 10 Hz against a 50 Hz inertial nominal, and are stated relative to that
    // nominal because the filter estimates the error in it.
    if (i % 5 === 0) {
      f.update(0, pTrue[i] + rnd() * sigmaP - pNom[i], R);
      f.update(1, vTrue[i] + rnd() * sigmaV - vNom[i], Rv);
    }
    f.xUpd.push([...f.x]); f.PUpd.push([...f.P]);
  }
  const sm = f.smooth();
  const fused = pNom.map((p, i) => p + sm[i][0]);
  const rms = (arr: number[]) => Math.sqrt(arr.reduce((a, b) => a + b * b, 0) / arr.length);
  const errNom = pNom.map((p, i) => p - pTrue[i]);
  const errFused = fused.map((p, i) => p - pTrue[i]);
  check('filter stays bounded over 9000 steps of divergent inertial data',
    fused.every(Number.isFinite) && Math.abs(Math.max(...fused)) < Math.max(...pTrue.map(Math.abs)) * 3,
    `max fused=${Math.max(...fused).toExponential(2)} ft vs truth ${Math.max(...pTrue).toFixed(0)} ft`);
  check('fusion beats the inertial nominal', rms(errFused) < rms(errNom) * 0.5,
    `inertial rms=${rms(errNom).toFixed(1)} ft, fused rms=${rms(errFused).toFixed(1)} ft`);
  check('fusion is close to the measurement accuracy it is given', rms(errFused) < sigmaP * 1.5,
    `fused rms=${rms(errFused).toFixed(1)} ft with ${sigmaP} ft fixes`);
  const velFused = vNom.map((v, i) => v + sm[i][1]);
  check('smoothed velocity is bounded too', Math.abs(Math.max(...velFused)) < Math.max(...vTrue.map(Math.abs)) * 3,
    `max fused vel=${Math.max(...velFused).toExponential(2)} ft/s`);
}

// --- parser encodings ---------------------------------------------------------
// The rules that decide how a byte stream becomes numbers: hex registers, engineering units versus
// the vendor's scaled integers, quaternion form. Each is a coin-flip in a real file, so each needs
// a fixture that says which way it landed.
{
  const { parseBlueRaven, ferHas } = await import('../src/lib/parsers/blueRaven.ts');
  const { parseGps } = await import('../src/lib/parsers/gps.ts');

  // Hex registers, CSV form. 180 is 0x180 = the pair of bits the pad noise sets, and 3A7 is a
  // mid-flight register that a numeric scan would truncate to 3.
  const csvLow = [
    't_s,sync_code,vel_up_fps,vel_downrange_fps,vel_crossrange_fps,alt_nav_ft,pos_downrange_ft,pos_crossrange_ft,alt_baro_agl_ft,tilt_deg,roll_deg,fer',
    '-1.000,120,0.0,0.0,0.0,5212,0,0,0.4,1.2,0.0,180',
    '0.020,140,12.0,0.4,0.1,5212,0,0,0.6,1.4,0.5,0x181',
    '0.040,160,25.0,0.5,0.2,5213,0,0,0.9,1.5,1.1,3A7',
    '0.060,180,38.0,0.6,0.3,5214,1,0,1.3,1.6,1.7,180d',
  ].join('\n');
  const low = parseBlueRaven(csvLow, 'csv').low;
  check('CSV FER columns are hexadecimal bitmasks', low.length === 4 && ferHas(low[0].fer, 7) && ferHas(low[0].fer, 8) && !ferHas(low[0].fer, 0),
    `row0 fer=0x${(low[0].fer >>> 0).toString(16)} (180 hex)`);
  check('CSV FER with a prefix is hex too', ferHas(low[1].fer, 0) && ferHas(low[1].fer, 8), '0x181');
  check('CSV FER with letters survives the numeric scan', low[2].fer === 0x3a7, `fer=0x${(low[2].fer >>> 0).toString(16)}`);
  check('an explicit decimal suffix is honoured', low[3].fer === 180, `fer=${low[3].fer}`);

  // Native telemetry: the header lives between the frame marker and the first label, and the sync
  // code in it is the only clock, so a mis-parsed header loses both the time axis and the date.
  const nativeLow = [
    '@ LOG_LOW: 22 24 3 23 21 23 16 120 Bo: 68.4 50000 V: 8412 0 0 0 0 12 POS: 5212 0 0 VEL: 0 0 0 AGl: 0 Ang: 12 0 0 FER: 180 CRC: 6A1D',
    '@ LOG_LOW: 22 24 3 23 21 23 16 140 Bo: 68.5 49990 V: 8410 300 0 0 0 40 POS: 5213 1 0 VEL: 12 0 0 AGl: 1 Ang: 14 0 0 FER: 181 CRC: 6A1E',
    '@ LOG_LOW: 22 24 3 23 21 23 16 160 Bo: 68.6 49980 V: 8409 0 0 0 0 6 POS: 5214 2 0 VEL: 25 1 0 AGl: 2 Ang: 16 1 0 FER: 3A7 CRC: 6A1F',
  ].join('\n');
  const tl = parseBlueRaven(nativeLow, 'telemetry');
  check('native low-rate keeps the record datestamp', tl.flightDate === '2024-03-23', `date=${tl.flightDate}`);
  check('native low-rate time comes from the sync code', tl.low.length === 3 && Math.abs(tl.low[1].t - 0.020) < 1e-9 && Math.abs(tl.low[2].t - 0.040) < 1e-9,
    `t=${tl.low.map((r) => r.t.toFixed(3)).join(',')}`);
  check('native Bo: pressure is 50000 per atmosphere', Math.abs(tl.low[0].baroPressureAtm - 1) < 1e-9,
    `${tl.low[0].baroPressureAtm}`);
  check('native Ang: tilt is in tenths of a degree', Math.abs(tl.low[0].tilt - 1.2) < 1e-9, `${tl.low[0].tilt} deg`);
  check('native FER is hex decoded from the raw token', tl.low[2].fer === 0x3a7, `fer=0x${(tl.low[2].fer >>> 0).toString(16)}`);

  // Native high rate: sync then nine sensor terms then four quaternion terms, all scaled integers.
  const nativeHigh = [
    '@ LOG_HIR: 18 24 3 23 21 23 16 120 250 -140 30 20 -10 9950 0 0 30000 0 CRC: 1111',
    '@ LOG_HIR: 18 24 3 23 21 23 16 122 260 -130 35 25 -12 9940 150 0 29996 400 CRC: 1112',
  ].join('\n');
  const th = parseBlueRaven(nativeHigh, 'telemetry').high!;
  check('native high-rate gyro/accel are hundredths', Math.abs(th[0].gyro[0] - 2.5) < 1e-9 && Math.abs(th[0].accel[2] - 99.5) < 1e-9,
    `gyroX=${th[0].gyro[0]} accelZ=${th[0].accel[2]}`);
  check('native high-rate quaternion is a unit quaternion', Math.abs(Math.hypot(...th[1].quat) - 1) < 1e-6,
    `|q|=${Math.hypot(...th[1].quat).toFixed(6)} w=${th[1].quat[3].toFixed(4)}`);
  check('native high-rate quaternion keeps the axis-first order', th[1].quat[3] > th[1].quat[0] && Math.abs(th[1].quat[0]) < 0.01,
    `q=${th[1].quat.map((x) => x.toFixed(3))}`);
  check('native high-rate time comes from the sync code', Math.abs(th[1].t - 0.002) < 1e-9, `t=${th[1].t}`);

  // High-rate CSV in engineering units must pass through untouched...
  const engHigh = [
    't_s,sync_code,gyro_x_dpps,gyro_y_dpps,gyro_z_dpps,accel_x_g,accel_y_g,accel_z_g,quat_x,quat_y,quat_z,quat_w',
    '-0.002,100,0.40,0.10,-0.20,0.020,0.001,1.019,0.00001,0.00000,0.00000,1.00000',
    '0.000,102,41.20,-8.30,3.10,27.400,0.900,3.100,0.01000,0.00200,0.00040,0.99995',
  ].join('\n');
  const eng = parseBlueRaven(engHigh, 'csv').high!;
  check('engineering-unit high-rate CSV is passed through', Math.abs(eng[1].accel[0] - 27.4) < 1e-9 && Math.abs(eng[1].gyro[0] - 41.2) < 1e-9,
    `accelX=${eng[1].accel[0]} gyroX=${eng[1].gyro[0]}`);
  check('unit-form quaternion columns survive', Math.abs(Math.hypot(...eng[1].quat) - 1) < 1e-6, `|q|=${Math.hypot(...eng[1].quat).toFixed(6)}`);

  // ...and the vendor's centi-unit export must land in the same units.
  const centiHigh = engHigh
    .replace('41.20,-8.30,3.10,27.400,0.900,3.100', '4120,-830,310,2740,90,310')
    .replace('0.40,0.10,-0.20,0.020,0.001,1.019', '40,10,-20,2,0,102')
    .replace('0.01000,0.00200,0.00040,0.99995', '300,60,12,29998');
  const cen = parseBlueRaven(centiHigh, 'csv').high!;
  check('centi-unit high-rate CSV is rescaled to engineering units', Math.abs(cen[1].accel[0] - 27.4) < 1e-6,
    `accelX=${cen[1].accel[0]}`);
  check('both high-rate dialects agree on the boost acceleration',
    Math.abs(cen[1].accel[0] - eng[1].accel[0]) < 0.05 * Math.abs(eng[1].accel[0]),
    `centi=${cen[1].accel[0].toFixed(2)} eng=${eng[1].accel[0].toFixed(2)} g`);
  check('30000-scaled quaternion in CSV is divided back down', Math.abs(Math.hypot(...cen[1].quat) - 1) < 1e-3,
    `|q|=${Math.hypot(...cen[1].quat).toFixed(4)}`);

  // A GPS file's Doppler velocities are the only low-noise rate the app ever gets, so they must
  // survive parsing; a receiver with no fix must not be read as a fix at (0,0).
  const gpsCsv = [
    't_iso,gps_unit,lat_deg,lon_deg,alt_ft,hvel_fps,heading_deg,upvel_fps,fix_type,sats_total',
    '2026-05-16T15:41:55.500,TRK,34.259071,-106.363103,5182.2,0.4,179.2,-0.9,0,9',
    '2026-05-16T15:41:55.600,TRK,34.259101,-106.363108,5227.3,1450.2,179.2,1200.6,3,9',
  ].join('\n');
  const gr = parseGps(gpsCsv).rows;
  check('GPS Doppler velocities are parsed', gr.length === 2 && Math.abs(gr[1].upvel - 1200.6) < 1e-9 && Math.abs(gr[1].hvel - 1450.2) < 1e-9,
    `hvel=${gr[1].hvel} upvel=${gr[1].upvel}`);
  check('a no-fix GPS row is kept but marked', gr[0].fixType === 0, 'fix 0 retained for gap accounting');
}

// --- joining the two Blue Raven logs on the sync counter ----------------------
{
  const { syncAlignment } = await import('../src/lib/sync.ts');
  const offset = 0.137; // the high-rate log's own axis reads this much early
  const low: any[] = [], high: any[] = [];
  for (let i = 0; i < 5000; i++) {
    const tau = i * 0.02;
    low.push({ t: tau, sync: Math.round(tau * 1000) % 250 });
  }
  for (let i = 0; i < 50000; i++) {
    const tau = i * 0.002;
    high.push({ t: tau - offset, sync: Math.round(tau * 1000) % 250 });
  }
  const fit = syncAlignment(low, high);
  check('sync counter joins the two Blue Raven logs', !!fit && Math.abs(fit.offsetS - offset) <= 0.001,
    `got ${fit ? fit.offsetS.toFixed(3) : 'null'} s of ${offset} s, residual ${fit ? fit.residualMs.toFixed(2) : '-'} ms`);
  check('a shared-epoch export aligns to nothing', !!(() => {
    const same = high.map((r) => ({ ...r, t: r.t + offset }));
    const f = syncAlignment(low, same);
    return f && Math.abs(f.offsetS) <= 0.001;
  })());
  let s3 = 7;
  const jitter = () => ((s3 = (s3 * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff) * 250;
  check('logs from different flights refuse to align',
    syncAlignment(low, high.map((r) => ({ ...r, sync: Math.round(jitter()) }))) === null);
  check('an offset beyond the roll period is reported as ambiguous', !!fit && fit.aliased === true,
    `axes start ${(-offset).toFixed(3)} s apart`);
}

// --- which GPS rows count as measurements ------------------------------------
{
  const { gpsTrack } = await import('../src/lib/fusion.ts');
  const pad: any = { lat: 40.5, lon: -106.2, altFt: 5000 };
  const row = (o: any) => ({ t: 1, lat: 40.5001, lon: -106.2001, altFt: 5100, hvel: 300, heading: 90, upvel: 20, fixType: 3, sats: 9, ...o });
  const kept = (rows: any[]) => gpsTrack(rows, pad as any).t.length;
  check('a clean fix is a measurement', kept([row({})]) === 1);
  check('a row with no fix is not', kept([row({ fixType: 0 })]) === 0);
  check('three satellites is not a measurement', kept([row({ sats: 3 })]) === 0, 'four is the least that fixes a position');
  check('four satellites is', kept([row({ sats: 4 })]) === 1);
  check('HDOP 12 is not a measurement', kept([row({ hdop: 12 })]) === 0);
  check('HDOP 1.8 is', kept([row({ hdop: 1.8 })]) === 1);
  check('a log with no fix or satellite columns is assumed to be fixing', kept([row({ fixType: NaN, sats: NaN })]) === 1);
}

// --- which reading of the attitude quaternion a log is using ------------------
{
  const { resolveAttitude } = await import('../src/ui/attitude.ts');
  const D = Math.PI / 180;
  type Quat = [number, number, number, number];
  const qmul = (a: number[], b: number[]): Quat => [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
  const qAxis = (v: number[], thDeg: number): Quat => {
    const s = Math.sin((thDeg * D) / 2) / Math.hypot(v[0], v[1], v[2]);
    return [v[0] * s, v[1] * s, v[2] * s, Math.cos((thDeg * D) / 2)];
  };

  const az = 35;
  const low: any[] = [];
  const readingPointing: QuatEpoch[] = [];
  const readingBody: QuatEpoch[] = [];
  for (let i = 0; i <= 400; i++) {
    const t = i * 0.02;
    const tilt = 5 + 120 * (i / 400); // sweeps through vertical, so both signs of up appear
    const roll = 200 * t; // fast enough that the half-angle sign flips many times
    low.push({ t, tilt, roll });
    const point = [Math.sin(tilt * D) * Math.cos(az * D), Math.sin(tilt * D) * Math.sin(az * D), Math.cos(tilt * D)];
    // reading B: the quaternion is a roll about the pointing direction, so its vector part is it
    readingPointing.push({ t, quat: qAxis(point, roll) });
    // reading A: rotating body +z by the quaternion gives the pointing direction, and the roll is a
    // rotation about body +z, which leaves that direction alone
    const hinge = [-Math.sin(az * D), Math.cos(az * D), 0];
    readingBody.push({ t, quat: qmul(qAxis(hinge, tilt), qAxis([0, 0, 1], roll)) });
  }

  const b = resolveAttitude(readingPointing as any, low as any);
  check('the pointing-vector reading is chosen when it is the one that reproduces the tilt',
    b.source === 'axis-is-pointing', `${b.source}, median error ${b.agreementDeg.toFixed(3)} deg over ${Math.round(b.withinTol * 100)}% of epochs`);
  const a = resolveAttitude(readingBody as any, low as any);
  check('the body-to-world reading is chosen when it is the one that reproduces the tilt',
    a.source === 'body-to-world', `${a.source}, median error ${a.agreementDeg.toFixed(3)} deg`);
  let s4 = 11;
  const rnd = () => ((s4 = (s4 * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  const junk = resolveAttitude(
    readingPointing.map((r) => ({ t: r.t, quat: [rnd(), rnd(), rnd(), rnd()] as Quat })),
    low as any,
  );
  check('a quaternion that reproduces neither falls back to tilt plus track', junk.source === 'tilt-track', junk.source);

  const straightUp = resolveAttitude(
    low.map((r) => ({ t: r.t, quat: qAxis([0, 0, 1], r.roll) })),
    low.map((r) => ({ ...r, tilt: 0 })),
  );
  check('a rocket flying straight up cannot tell the two readings apart, so neither is claimed',
    straightUp.source === 'tilt-track', straightUp.source);

  // Past 90 degrees of tilt the nose is below the horizon. The half-angle in the quaternion loses
  // that sign, so it has to come back from the reported tilt, whichever way the roll has spun it.
  const up = b.at(1.0, { e: 1, n: 0 });
  const down = b.at(7.0, { e: 1, n: 0 });
  check('the marker is nose-up while the reported tilt is under 90 degrees', !!up && up.axis[2] > 0.9, `up component ${up ? up.axis[2].toFixed(2) : 'none'}`);
  check('and nose-down once the reported tilt is over 90 degrees', !!down && down.axis[2] < -0.3, `up component ${down ? down.axis[2].toFixed(2) : 'none'}`);
  const rollDeg = down ? (((down.roll / D) % 360) + 360) % 360 : NaN;
  check('roll is carried through from the low-rate log', !!down && Math.abs(rollDeg - (1400 % 360)) < 1,
    `${rollDeg.toFixed(0)} degrees reported, ${(1400 % 360).toFixed(0)} expected`);
}

// --- gyro saturation episodes ------------------------------------------------
{
  const { saturationEpisodes } = await import('../src/lib/fusion.ts');
  const times = [
    ...Array.from({ length: 251 }, (_, i) => i * 0.002), // 0 to 0.5 s at 500 Hz: a real tumble
    5.0, // one dropped field, which is a blip and not an episode
    ...Array.from({ length: 151 }, (_, i) => 10 + i * 0.002), // 10 to 10.3 s
  ];
  const sat = saturationEpisodes(times);
  check('two saturation episodes', sat.episodes.length === 2, JSON.stringify(sat.episodes));
  check('one blip counted rather than listed', sat.blips === 1, `${sat.blips}`);
  check('episode ends where the saturation stopped', Math.abs(sat.episodes[0].t1 - 0.5) < 1e-9);
  const split = saturationEpisodes([0, 0.002, 1, 1.002], 0.1, 0);
  check('a one second gap splits rather than stretches', split.episodes.length === 2);
}

// --- degraded inputs, end to end ---------------------------------------------
{
  const { reconstruct, derivePad } = await import('../src/lib/fusion.ts');
  const { parseBlueRaven } = await import('../src/lib/parsers/blueRaven.ts');
  const { parseGps } = await import('../src/lib/parsers/gps.ts');
  const { readFileSync } = await import('node:fs');
  const read = (f: string) => readFileSync(new URL(`../public/data/${f}`, import.meta.url), 'utf8');
  const low = parseBlueRaven(read('f52-tumble_blue_raven_low.csv'));
  const high = parseBlueRaven(read('f52-tumble_blue_raven_high.csv'));
  const gpsP = parseGps(read('f52-tumble_gps.csv'));
  const gps = gpsP.rows;
  const pad = derivePad(gps, low.low).pad;
  const meta = { brDialect: low.dialect, gpsDialect: gpsP.dialect, warnings: [...low.warnings, ...gpsP.warnings] };
  type Data = Parameters<typeof reconstruct>[0];
  type Rec = ReturnType<typeof reconstruct>;
  const full = reconstruct({ brLow: low.low, brHigh: high.high, gps, pad, meta });
  check('the full log reconstructs apogee', Math.abs(full.stats.maxAltFt - 9226) < 40, full.stats.maxAltFt.toFixed(0));

  const attempt = (label: string, data: Data, want: (r: Rec) => boolean, detail: (r: Rec) => string) => {
    try {
      const r = reconstruct(data);
      check(label, want(r), detail(r));
    } catch (e) {
      check(label, false, `threw ${e}`);
    }
  };

  attempt(
    "a flight whose high-rate log was never downloaded still reconstructs",
    { brLow: low.low, brHigh: undefined, gps, pad, meta },
    (r) => Math.abs(r.stats.maxAltFt - full.stats.maxAltFt) < 200,
    (r) => `apogee ${r.stats.maxAltFt.toFixed(0)} ft against ${full.stats.maxAltFt.toFixed(0)} ft, ${r.warnings.length} warnings`,
  );

  attempt(
    'a high-rate log with no rows in it does not stop it',
    { brLow: low.low, brHigh: [], gps, pad, meta },
    (r) => Number.isFinite(r.stats.maxAltFt) && r.stats.maxVelFps > 100,
    (r) => `apogee ${r.stats.maxAltFt.toFixed(0)} ft, max ${r.stats.maxVelFps.toFixed(0)} ft/s`,
  );

  attempt(
    'a GPS log with no fixes in it still reports a flight, on the IMU alone',
    { brLow: low.low, brHigh: high.high, gps: [], pad, meta },
    (r) => Number.isFinite(r.stats.maxAltFt) && r.fused.t.length > 1000,
    (r) => `${r.fused.t.length} epochs, apogee ${r.stats.maxAltFt.toFixed(0)} ft`,
  );

  let refused = false,
    why = '';
  for (const data of [
    { brLow: [], brHigh: high.high, gps, pad, meta },
    { brLow: [], brHigh: undefined, gps: [], pad, meta },
  ]) {
    try {
      reconstruct(data);
      why = 'returned a result instead of refusing';
    } catch (e) {
      refused = e instanceof Error && /low-rate log is empty/i.test(e.message);
      why = String(e).replace(/^Error: /, '');
    }
    if (!refused) break;
  }
  check('no flight-computer log is refused by name rather than by a crash', refused, why);
}

console.log(fails ? `\n${fails} check(s) failed` : '\nall checks passed');
process.exit(fails ? 1 : 0);
