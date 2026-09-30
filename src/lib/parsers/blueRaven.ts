import { columnIndex, findColumn, num, parseBitmask, parseCsv } from '../csv.ts';
import type { BrHighRow, BrLowRow, Dialect } from '../types.ts';

/** Flight-event register bits, from the Blue Raven manual's rocket-level event table. */
export const FER_BITS: { bit: number; name: string; label: string }[] = [
  { bit: 0, name: 'liftoff', label: 'Liftoff detected' },
  { bit: 1, name: 'apogee', label: 'Apogee detected (2-of-3 vote)' },
  { bit: 2, name: 'pressInc', label: 'Pressure increasing (descending)' },
  { bit: 3, name: 'apoFired', label: 'Apo channel fired' },
  { bit: 4, name: 'mainFired', label: 'Main channel fired' },
  { bit: 5, name: 'thirdFired', label: '3rd channel fired' },
  { bit: 6, name: 'fourthFired', label: '4th channel fired' },
  { bit: 7, name: 'eciNeg', label: 'ECI vertical velocity <= 0' },
  { bit: 8, name: 'accelNeg', label: 'Accel-only velocity <= 0' },
  { bit: 9, name: 'tilt90', label: 'Tilt exceeded 90 degrees' },
];

export const ferHas = (fer: number, bit: number) => (fer & (1 << bit)) !== 0;

export interface BrParseResult {
  low: BrLowRow[];
  high?: BrHighRow[];
  dialect: Dialect;
  kind: 'low' | 'high';
  flightDate?: string;
  warnings: string[];
}

export function looksLikeBlueRaven(text: string): boolean {
  const head = text.slice(0, 4000).toLowerCase();
  if (/@\s*log_(low|hir)/.test(head)) return true;
  const first = head.split('\n').find((l) => l.includes(','));
  if (!first) return false;
  const cols = first.split(',').map((c) => c.trim().toLowerCase());
  const has = (...names: string[]) => names.some((n) => cols.includes(n));
  return (has('vel_up_fps', 'vel_up', 'upward_velocity_fps', 'vel') && has('alt_nav_ft', 'pos_downrange_ft', 'inertial_nav_altitude_ft')) ||
    (has('quat_x', 'quatw') && has('gyro_x_dpps', 'accel_x_g'));
}

// --- native telemetry --------------------------------------------------------
const NUM = /-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?/g;

/**
 * Split an `@`-framed record into label -> payload, preserving order.
 *
 * Tokens are kept twice over: numeric, and raw. The raw copy exists because the flight-event
 * registers are hex bitmasks (`FER: 3A7`), which a numeric scan matches only as the leading `3`.
 */
function sections(line: string): { order: string[]; byLabel: Map<string, number[]>; tokens: Map<string, string[]>; head: number[] } {
  const tokens = line.trim().split(/\s+/);
  const byLabel = new Map<string, number[]>();
  const rawByLabel = new Map<string, string[]>();
  const order: string[] = [];
  const head: number[] = [];
  let cur: number[] | null = null;
  let curRaw: string[] | null = null;
  for (const raw of tokens) {
    if (raw.endsWith(':')) {
      const key = raw.slice(0, -1).toLowerCase();
      if (!byLabel.has(key)) {
        byLabel.set(key, []);
        rawByLabel.set(key, []);
        order.push(key);
      }
      cur = byLabel.get(key)!;
      curRaw = rawByLabel.get(key)!;
      continue;
    }
    const vals = raw.match(NUM);
    if (curRaw) curRaw.push(raw);
    if (!vals) continue;
    const nums = vals.map(Number);
    if (cur) cur.push(...nums);
    else head.push(...nums);
  }
  return { order, byLabel, tokens: rawByLabel, head };
}

/** Reconstruct monotonic time from the 250 ms-rolling ms sync counter. */
function timeFromSync(syncs: number[]): number[] {
  const t = [0];
  for (let i = 1; i < syncs.length; i++) {
    let d = syncs[i] - syncs[i - 1];
    if (d < -200) d += 250; // wrapped
    if (d < 0 || d > 250) d = 20; // implausible jump; assume nominal 20 ms
    t.push(t[i - 1] + d / 1000);
  }
  return t;
}

function parseTelemetryLow(text: string): { rows: BrLowRow[]; flightDate?: string } {
  let flightDate: string | undefined;
  const raw: Omit<BrLowRow, 't'>[] = [];
  const syncs: number[] = [];
  for (const line of text.split('\n')) {
    if (!line.includes(':')) continue;
    const isLog = /@\s*log_low/i.test(line);
    if (!isLog) continue;
    const { byLabel, tokens, head } = sections(line);
    if (!flightDate) {
      const y = head.find((v) => v >= 1990 && v <= 2100);
      if (y !== undefined) {
        const yi = head.indexOf(y);
        const [, mo, d] = head.slice(yi, yi + 6);
        if (mo !== undefined && d !== undefined) flightDate = `${y}-${String(mo).padStart(2, '0')}-${String(d).padStart(2, '0')}`;
      }
    }
    const bo = byLabel.get('bo') ?? [];
    const v = byLabel.get('v') ?? [];
    const vel = byLabel.get('vel') ?? [];
    const pos = byLabel.get('pos') ?? [];
    const ang = byLabel.get('ang') ?? [];
    const ferHex = (tokens.get('fer') ?? [])[0];
    const agl: number[] = byLabel.get('agl') ?? [];
    // First numeric token of the record is the ms sync code.
    const sync = head.length > 0 ? head[head.length - 1] : 0;
    syncs.push(sync);
    raw.push({
      sync,
      baroTempF: bo[0] ?? NaN,
      baroPressureAtm: (bo[1] ?? NaN) / 50000,
      batteryMv: v[0] ?? NaN,
      apoMv: v[1] ?? NaN,
      mainMv: v[2] ?? NaN,
      thirdMv: v[3] ?? NaN,
      fourthMv: v[4] ?? NaN,
      outputMa: v[5] ?? NaN,
      velUp: vel[0] ?? NaN,
      velDown: vel[1] ?? NaN,
      velCross: vel[2] ?? NaN,
      altNav: pos[0] ?? NaN,
      posDown: pos[1] ?? NaN,
      posCross: pos[2] ?? NaN,
      altBaroAgl: agl[0] ?? NaN,
      tilt: (ang[0] ?? NaN) / 10,
      roll: ang[1] ?? NaN,
      tiltFuture: (ang[2] ?? NaN) / 10,
      // Hex bitmask: take the token, not the numeric scan, which would read "3A7" as 3.
      fer: parseBitmask(ferHex) || 0,
    });
  }
  const t = timeFromSync(syncs);
  return { rows: raw.map((r, i) => ({ t: t[i], ...r })), flightDate };
}

function parseTelemetryHigh(text: string): BrHighRow[] {
  const syncs: number[] = [];
  const payload: number[][] = [];
  for (const line of text.split('\n')) {
    if (!/@\s*log_hir/i.test(line)) continue;
    const { head } = sections(line);
    // Layout is [packet length, y, mo, d, h, mi, s, sync, gx, gy, gz, ax, ay, az, qx, qy, qz, qw];
    // everything the reconstruction needs is the last eleven fields.
    const tail = head.slice(-11);
    if (tail.length < 11) continue;
    syncs.push(tail[0]);
    payload.push(tail);
  }
  const t = timeFromSync(syncs);
  return payload.map((p, i) => {
    const gx = p[1] / 100, gy = p[2] / 100, gz = p[3] / 100;
    const ax = p[4] / 100, ay = p[5] / 100, az = p[6] / 100;
    // Axis-first, x10000-scaled quaternion: the fourth term is w = cos(theta/2).
    const q = normaliseQuat([p[7] / 30000, p[8] / 30000, p[9] / 30000, p.length > 10 ? p[10] / 30000 : NaN]);
    return {
      t: t[i],
      sync: p[0],
      gyro: [gx, gy, gz] as [number, number, number],
      accel: [ax, ay, az] as [number, number, number],
      quat: q,
    };
  });
}

// --- CSV ---------------------------------------------------------------------
function parseCsvLow(text: string): { rows: BrLowRow[]; warnings: string[] } {
  const { header, rows } = parseCsv(text);
  const idx = columnIndex(header);
  const col = (...a: string[]) => findColumn(idx, a);
  const cT = col('t_s', 'time_s', 'seconds', 'time', 'elapsed_time_s', 't');
  const cSync = col('sync_code', 'sync', 'ms_counter');
  const cTms = col('t_ms', 'time_ms', 'millis');
  const cUp = col('vel_up_fps', 'vel_up', 'upward_velocity_fps', 'upward_velocity', 'velup');
  const cDn = col('vel_downrange_fps', 'vel_downrange', 'down_range_velocity_fps', 'downrange_velocity', 'velr1');
  const cCr = col('vel_crossrange_fps', 'vel_crossrange', 'cross_range_velocity_fps', 'crossrange_velocity', 'velr2');
  const cAltNav = col('alt_nav_ft', 'inertial_nav_altitude_ft', 'inertial_nav_altitude', 'alt_ft', 'altitude_ft');
  const cPosDn = col('pos_downrange_ft', 'downrange_feet', 'down_range_ft', 'downrange');
  const cPosCr = col('pos_crossrange_ft', 'cross_range_feet', 'cross_range_ft', 'crossrange');
  const cAltBaro = col('alt_baro_agl_ft', 'agl_ft', 'agl', 'baro_altitude_ft', 'altitude_agl_ft');
  const warnings: string[] = [];
  if (cUp < 0 && cAltNav < 0) warnings.push('Blue Raven file: no velocity or altitude columns recognised.');
  // If the file carries no time column at all, assume the documented 50 Hz cadence.
  const hasTime = cT >= 0 || cTms >= 0 || cSync >= 0;
  if (!hasTime && rows.length) warnings.push('Blue Raven file has no time column; assuming the nominal 50 Hz cadence.');
  const out: BrLowRow[] = [];
  for (let rowI = 0; rowI < rows.length; rowI++) {
    const r = rows[rowI];
    const g = (c: number) => (c >= 0 ? num(r[c]) : NaN);
    const ferRaw = (names: string[]) => {
      const c = col(...names);
      return c >= 0 ? parseBitmask(r[c]) : NaN;
    };
    const fer = ferRaw(['fer', 'flight_event_register', 'fer_rocket', 'event_register']);
    const t = cT >= 0 ? g(cT) : cTms >= 0 ? g(cTms) / 1000 : g(cSync) / 1000;
    out.push({
      t,
      sync: g(cSync),
      baroTempF: g(col('baro_temp_f', 'baro_temperature_f', 'baro_temp', 'temperature_f')),
      baroPressureAtm: g(col('baro_pressure_atm', 'pressure_atm', 'baro_pressure')),
      batteryMv: g(col('battery_mv', 'battery_millivolts', 'battery', 'vbat')),
      apoMv: g(col('apo_mv', 'apo_millivolts', 'apogee_mv')),
      mainMv: g(col('main_mv', 'main_millivolts')),
      thirdMv: g(col('third_mv', '3rd_mv', 'ch3_mv')),
      fourthMv: g(col('fourth_mv', '4th_mv', 'ch4_mv')),
      outputMa: g(col('output_ma', 'output_current_ma', 'output_current')),
      velUp: g(cUp),
      velDown: g(cDn),
      velCross: g(cCr),
      altNav: g(cAltNav),
      posDown: g(cPosDn),
      posCross: g(cPosCr),
      altBaroAgl: g(cAltBaro),
      tilt: g(col('tilt_deg', 'tilt_angle_deg', 'tilt')),
      roll: g(col('roll_deg', 'roll_angle_deg', 'roll')),
      tiltFuture: g(col('tilt_future_deg', 'future_tilt_angle_deg', 'tilt_predicted_deg')),
      fer: Number.isFinite(fer) ? fer : 0,
      ...(hasTime ? {} : { t: rowI / 50 }),
    });
  }
  return { rows: out.filter((r) => Number.isFinite(r.t)), warnings };
}

function parseCsvHigh(text: string): BrHighRow[] {
  const { header, rows } = parseCsv(text);
  const idx = columnIndex(header);
  const col = (...a: string[]) => findColumn(idx, a);
  const cT = col('t_s', 'time_s', 'time', 't');
  const cTms = col('t_ms', 'time_ms');
  const cSync = col('sync_code', 'sync');
  const cGyro = [col('gyro_x_dpps', 'gyro_x_degps', 'gyro_x'), col('gyro_y_dpps', 'gyro_y_degps', 'gyro_y'), col('gyro_z_dpps', 'gyro_z_degps', 'gyro_z')];
  const cAccel = [col('accel_x_g', 'accel_x'), col('accel_y_g', 'accel_y'), col('accel_z_g', 'accel_z')];
  const cQuat = [col('quat_x', 'quaternion_x', 'qx'), col('quat_y', 'quaternion_y', 'qy'), col('quat_z', 'quaternion_z', 'qz'), col('quat_w', 'quat_mag', 'quaternion_magnitude', 'qw', 'quat_m')];
  // A vendor CSV carries the telemetry's integer scalings (deg/s and G x100); a spreadsheet
  // re-export of the same file carries engineering units. Read the data, not the file name: across
  // a whole flight the median turn rate is tens of deg/s and the median specific force about 1 G,
  // so columns two orders of magnitude above that can only be the scaled form.
  const gyroScale = detectScaling(rows, cGyro, 40);
  const accelScale = detectScaling(rows, cAccel, 1.2);
  const out: BrHighRow[] = [];
  for (const r of rows) {
    const g = (c: number, d = 1) => (c >= 0 ? num(r[c]) / d : NaN);
    const t = cT >= 0 ? g(cT) : cTms >= 0 ? g(cTms) / 1000 : g(cSync) / 1000;
    if (!Number.isFinite(t)) continue;
    out.push({
      t,
      sync: g(cSync),
      gyro: cGyro.map((c) => g(c, gyroScale)) as [number, number, number],
      accel: cAccel.map((c) => g(c, accelScale)) as [number, number, number],
      quat: quatFrom(g(cQuat[0]), g(cQuat[1]), g(cQuat[2]), g(cQuat[3])),
    });
  }
  return out;
}

// --- scaling and attitude helpers --------------------------------------------

function sampledMedianAbs(rows: string[][], cols: number[]): number {
  const vals: number[] = [];
  const step = Math.max(1, Math.floor(rows.length / 400));
  for (let i = 0; i < rows.length; i += step) {
    for (const c of cols) {
      if (c < 0) continue;
      const v = Math.abs(num(rows[i][c]));
      if (Number.isFinite(v)) vals.push(v);
    }
  }
  if (!vals.length) return NaN;
  vals.sort((a, b) => a - b);
  return vals[Math.floor(vals.length / 2)];
}

/**
 * 1 when the columns already hold engineering units, 100 when they hold the telemetry's integer
 * form. The margin is a factor of ten either way, so a genuine 20 G log is never mistaken for a
 * scaled one and a scaled log whose flight was all coast is still caught.
 */
function detectScaling(rows: string[][], cols: number[], expectedTypical: number): number {
  const m = sampledMedianAbs(rows, cols);
  if (!Number.isFinite(m) || m <= 0) return 1;
  return m > expectedTypical * 10 ? 100 : 1;
}

/** Unit quaternion, normalised. `w` may be absent, in which case the triple is a rotation vector. */
export function normaliseQuat(q: [number, number, number, number]): [number, number, number, number] {
  const [x, y, z] = q;
  const v = Math.hypot(x, y, z);
  if (!Number.isFinite(q[3])) {
    const half = v / 2;
    const s = Math.sin(half);
    return v > 1e-9 ? [(x / v) * s, (y / v) * s, (z / v) * s, Math.cos(half)] : [0, 0, 0, 1];
  }
  const n = Math.hypot(x, y, z, q[3]);
  if (!Number.isFinite(n) || n < 1e-9) return [0, 0, 0, 1];
  return [x / n, y / n, z / n, q[3] / n];
}

/** Read the four quaternion terms whatever scaling the file used (a unit quaternion is <= 1). */
function quatFrom(qx: number, qy: number, qz: number, qw: number): [number, number, number, number] {
  if (!Number.isFinite(qx) && !Number.isFinite(qy) && !Number.isFinite(qz)) return [0, 0, 0, 1];
  const s = Math.hypot(qx, qy, qz) > 100 ? 30000 : 1;
  return normaliseQuat([qx / s, qy / s, qz / s, Number.isFinite(qw) ? qw / s : NaN]);
}

export function parseBlueRaven(text: string, _fileName = ''): BrParseResult {
  const warnings: string[] = [];
  const isTelemetry = /@\s*log_(low|hir)/i.test(text.slice(0, 20000));
  if (isTelemetry) {
    const isHigh = /@\s*log_hir/i.test(text.slice(0, 20000));
    if (isHigh) return { low: [], high: parseTelemetryHigh(text), dialect: 'telemetry', kind: 'high', warnings };
    const { rows, flightDate } = parseTelemetryLow(text);
    if (!rows.length) warnings.push('Blue Raven telemetry: found no @ LOG_LOW records.');
    return { low: rows, dialect: 'telemetry', kind: 'low', flightDate, warnings };
  }
  const { header } = parseCsv(text);
  const lower = header.map((h) => h.toLowerCase());
  const isHigh = lower.some((h) => h.startsWith('quat') || h.startsWith('gyro_'));
  if (isHigh) {
    const high = parseCsvHigh(text);
    if (!high.length) warnings.push('Blue Raven high-rate file parsed but contained no rows.');
    return { low: [], high, dialect: 'csv', kind: 'high', warnings };
  }
  const { rows, warnings: w } = parseCsvLow(text);
  if (!rows.length) warnings.push('Blue Raven CSV parsed but contained no usable rows.');
  return { low: rows, dialect: 'csv', kind: 'low', warnings: [...warnings, ...w] };
}
