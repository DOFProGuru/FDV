import { columnIndex, findColumn, firstNumericColumn, hasColumn, headerKeys, num, parseBitmask, parseCsv, readClock } from '../csv.ts';
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
  const cols = new Set(headerKeys(text));
  const has = (...names: string[]) => hasColumn(cols, names);
  return (has('vel_up_fps', 'vel_up', 'upward_velocity_fps', 'vel', 'velocity_up') &&
    has('alt_nav_ft', 'pos_downrange_ft', 'inertial_nav_altitude_ft', 'inertial_altitude')) ||
    (has('quat_x', 'quatw', 'quat_1') && has('gyro_x_dpps', 'gyro_x'));
}

/**
 * Whether a Blue Raven file is the 500 Hz attitude log rather than the 50 Hz altimeter log. The two
 * share the elapsed-time and sync columns; what separates them is the inertial measurement. Exported
 * because the loader has to make the same call as the parser, and two answers would be two bugs.
 */
export function isHighRateLog(text: string): boolean {
  const head = text.slice(0, 40_000);
  if (/@\s*log_hir/.test(head)) return true;
  if (/@\s*log_low/.test(head)) return false;
  return hasColumn(new Set(headerKeys(text)), ['quat_x', 'quat_1', 'quaternion_x', 'qx', 'gyro_x', 'gyro_x_dpps', 'gyro_x_degps']);
}

// --- native telemetry --------------------------------------------------------
const NUM = /-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?/g;

/** Tokens that name the record rather than a field within it. */
const FRAME_MARKERS = new Set(['@', 'log_low', 'log_hir', 'gps_stat']);

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
    // The frame marker is not a data section. Without this the header numbers (packet length, UTC
    // datestamp, sync code) would be filed under the marker's own name and `head` would stay empty.
    const bare = raw.endsWith(':') ? raw.slice(0, -1).toLowerCase() : raw.toLowerCase();
    if (FRAME_MARKERS.has(bare)) continue;
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
      // Positional, per the documented record layout: packet length, then year, month, day, hour,
      // minute, second. Some firmware writes a two-digit year, which is unambiguous for any device
      // sold this century; a scan for a four-digit number would instead lock onto whatever field
      // happens to look like a year.
      const [y, mo, d] = head.slice(1, 4);
      if (Number.isFinite(y) && Number.isFinite(mo) && Number.isFinite(d) && mo >= 1 && mo <= 12 && d >= 1 && d <= 31)
        flightDate = `${y < 100 ? 2000 + y : y}-${String(mo).padStart(2, '0')}-${String(d).padStart(2, '0')}`;
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
    // Axis-first unit quaternion, carried by some firmware as 30000-scaled integers: the fourth
    // term is w = cos(theta/2), not an angle.
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
/**
 * A millivolt column, from whichever of the two forms the file carries.
 *
 * The telemetry writes millivolts and the vendor's spreadsheet export writes volts, and the column
 * name is the only thing that says which. The magnitude decides rather than the name: a charge column
 * reads 0-2 V or 0-2,000 mV and a battery 4-25 V or 4,000-25,000 mV, so no honest file sits between
 * the two readings, and an export whose `volts` column already holds millivolts is not divided twice.
 */
function millivoltColumn(rows: string[][], cMv: number, cV: number): (r: string[]) => number {
  if (cMv >= 0 && firstNumericColumn(rows, [cMv]) >= 0) return (r) => num(r[cMv]);
  if (cV >= 0 && firstNumericColumn(rows, [cV]) >= 0) {
    const m = sampledMedianAbs(rows, [cV]);
    const alreadyMv = Number.isFinite(m) && m >= 100;
    return (r) => Math.round(num(r[cV]) * (alreadyMv ? 1 : 1000));
  }
  return () => NaN;
}

function parseCsvLow(text: string): { rows: BrLowRow[]; flightDate?: string; warnings: string[] } {
  const { header, rows } = parseCsv(text);
  const idx = columnIndex(header);
  const col = (...a: string[]) => findColumn(idx, a);
  // `Flight_Time_(s)` is the vendor export's name for the same elapsed seconds. It comes before the
  // bare `time`, which in both of the vendor's files is a time of day and not an interval.
  const cT = col('t_s', 'time_s', 'Flight_Time_(s)', 'seconds', 'elapsed_time_s', 'time', 't');
  const cSync = col('sync_code', 'sync', 'ms_counter');
  const cTms = col('t_ms', 'time_ms', 'millis');
  const cStamp = col('t_iso', 'timestamp', 'datetime', 'date_time');
  const cYear = col('year'), cMonth = col('month'), cDay = col('day');
  const cClock = col('time', 'clock', 'time_of_day');
  const cUp = col('vel_up_fps', 'vel_up', 'upward_velocity_fps', 'upward_velocity', 'velup', 'Velocity_Up');
  const cDn = col('vel_downrange_fps', 'vel_downrange', 'down_range_velocity_fps', 'downrange_velocity', 'velr1', 'Velocity_DR');
  const cCr = col('vel_crossrange_fps', 'vel_crossrange', 'cross_range_velocity_fps', 'crossrange_velocity', 'velr2', 'Velocity_CR');
  const cAltNav = col('alt_nav_ft', 'inertial_nav_altitude_ft', 'inertial_nav_altitude', 'Inertial_Altitude', 'alt_ft', 'altitude_ft');
  const cPosDn = col('pos_downrange_ft', 'downrange_feet', 'down_range_ft', 'downrange', 'Inertial_DR_Position');
  const cPosCr = col('pos_crossrange_ft', 'cross_range_feet', 'cross_range_ft', 'crossrange', 'Inertial_CR_Position');
  const cAltBaro = col('alt_baro_agl_ft', 'agl_ft', 'agl', 'baro_altitude_ft', 'altitude_agl_ft', 'Baro_Altitude_AGL_(feet)');
  const cBatt = col('battery_mv', 'battery_millivolts', 'battery', 'vbat');
  const cBattV = col('Batt_Volts', 'battery_volts', 'batt_v');
  const cApo = col('apo_mv', 'apo_millivolts', 'apogee_mv');
  const cApoV = col('Apo_Volts', 'apo_volts', 'apogee_volts');
  const cMain = col('main_mv', 'main_millivolts');
  const cMainV = col('Main_Volts', 'main_volts');
  const cThird = col('third_mv', '3rd_mv', 'ch3_mv');
  const cThirdV = col('3rd_Volts', 'third_volts', 'ch3_volts');
  const cFourth = col('fourth_mv', '4th_mv', 'ch4_mv');
  const cFourthV = col('4th_Volts', 'fourth_volts', 'ch4_volts');
  const warnings: string[] = [];
  if (cUp < 0 && cAltNav < 0) warnings.push('Blue Raven file: no velocity or altitude columns recognised.');
  // Elapsed seconds, then milliseconds, then the record's own clock, then the sync counter: whichever
  // of them is actually carrying numbers, because `time` is a name two exports share.
  const cSec = firstNumericColumn(rows, [cT, cTms]);
  const clockOf = readClock({ stamp: cStamp, year: cYear, month: cMonth, day: cDay, clock: cClock });
  const epochs = cSec < 0 && cSync < 0 ? rows.map((r) => { const iso = clockOf(r); return iso ? Date.parse(iso) : NaN; }) : [];
  const epoch0 = epochs.find((e) => Number.isFinite(e)) ?? NaN;
  // Two different stamps are enough to call it a clock: a file that repeats one stamp says nothing
  // about time and had better fall back to the nominal cadence.
  const hasClock = Number.isFinite(epoch0) && epochs.some((e) => Number.isFinite(e) && e !== epoch0);
  const hasTime = cSec >= 0 || cSync >= 0 || hasClock;
  if (!hasTime && rows.length) warnings.push('Blue Raven file has no time column; assuming the nominal 50 Hz cadence.');
  const mv = {
    battery: millivoltColumn(rows, cBatt, cBattV),
    apo: millivoltColumn(rows, cApo, cApoV),
    main: millivoltColumn(rows, cMain, cMainV),
    third: millivoltColumn(rows, cThird, cThirdV),
    fourth: millivoltColumn(rows, cFourth, cFourthV),
  };
  const cFer = col('fer', 'flight_event_register', 'fer_rocket', 'Rocket_FER_Hex', 'event_register');
  // The spreadsheet export decodes the event register into named columns as well as printing the
  // hex, and the two do not agree on where the bits are: the export counts a `Burnout_Coast` flag
  // ahead of the burn channels that the manual's table does not, which shifts every channel one
  // place. Read by the manual's numbers, this file's first row (`600` hex) comes out as "tilt
  // exceeded 90 degrees" where the file's own flags say vertical velocity and accel-only velocity
  // have each fallen to zero, which on the pad before liftoff is what they do. Where the export
  // names its flags, the named flags are the event record. The four burn channels are what the two
  // tables disagree about, so those are what must be present before the names are trusted: a partial
  // set of them tells less than the documented bit table does.
  const flagCols: { bit: number; c: number }[] = [
    { bit: 0, c: col('liftoff') },
    { bit: 1, c: col('apogee') },
    { bit: 2, c: col('press_increasing') },
    { bit: 3, c: col('apo_fired') },
    { bit: 4, c: col('main_fired') },
    { bit: 5, c: col('3rd_fired') },
    { bit: 6, c: col('4th_fired') },
    { bit: 7, c: col('eci_vvel_le_0', 'eci_v_vel_le_0') },
    { bit: 8, c: col('accel_vel_le_0', 'accel_vel_le_0') },
    { bit: 9, c: col('tilt_exceeded_90deg', 'tilt_exceeded_90_deg', 'tilt_90') },
  ];
  const namedFlags = flagCols.filter((f) => f.bit >= 3 && f.bit <= 6).every((f) => f.c >= 0);
  const out: BrLowRow[] = [];
  let flightDate: string | undefined;
  for (let rowI = 0; rowI < rows.length; rowI++) {
    const r = rows[rowI];
    const g = (c: number, d = 1) => (c >= 0 ? num(r[c]) / d : NaN);
    if (!flightDate) {
      const iso = clockOf(r);
      if (iso) flightDate = iso.slice(0, 10);
    }
    const fer = namedFlags
      ? flagCols.reduce((m, f) => (f.c >= 0 && num(r[f.c]) > 0 ? m | (1 << f.bit) : m), 0)
      : cFer >= 0
        ? parseBitmask(r[cFer])
        : NaN;
    const t = cSec >= 0 ? g(cSec, cSec === cTms && cTms >= 0 ? 1000 : 1) : hasClock ? (epochs[rowI] - epoch0) / 1000 : g(cSync) / 1000;
    out.push({
      t,
      sync: g(cSync),
      baroTempF: g(col('baro_temp_f', 'baro_temperature_f', 'baro_temp', 'Temperature_(F)')),
      baroPressureAtm: g(col('baro_pressure_atm', 'pressure_atm', 'baro_pressure', 'Baro_Press_(atm)')),
      batteryMv: mv.battery(r),
      apoMv: mv.apo(r),
      mainMv: mv.main(r),
      thirdMv: mv.third(r),
      fourthMv: mv.fourth(r),
      outputMa: g(col('output_ma', 'output_current_ma', 'output_current', 'Current')),
      velUp: g(cUp),
      velDown: g(cDn),
      velCross: g(cCr),
      altNav: g(cAltNav),
      posDown: g(cPosDn),
      posCross: g(cPosCr),
      altBaroAgl: g(cAltBaro),
      tilt: g(col('tilt_deg', 'Tilt_Angle_(deg)', 'tilt_angle_deg', 'tilt')),
      roll: g(col('roll_deg', 'Roll_Angle_(deg)', 'roll_angle_deg', 'roll')),
      tiltFuture: g(col('tilt_future_deg', 'Future_Angle_(deg)', 'future_angle_deg', 'future_tilt_angle_deg', 'tilt_predicted_deg')),
      fer: Number.isFinite(fer) ? fer : 0,
      ...(hasTime ? {} : { t: rowI / 50 }),
    });
  }
  return { rows: out.filter((r) => Number.isFinite(r.t)), flightDate, warnings };
}

function parseCsvHigh(text: string): BrHighRow[] {
  const { header, rows } = parseCsv(text);
  const idx = columnIndex(header);
  const col = (...a: string[]) => findColumn(idx, a);
  const cT = col('t_s', 'time_s', 'Flight_Time_(s)', 'time', 't');
  const cTms = col('t_ms', 'time_ms');
  const cSync = col('sync_code', 'sync');
  const cGyro = [col('gyro_x_dpps', 'gyro_x_degps', 'gyro_x'), col('gyro_y_dpps', 'gyro_y_degps', 'gyro_y'), col('gyro_z_dpps', 'gyro_z_degps', 'gyro_z')];
  const cAccel = [col('accel_x_g', 'accel_x'), col('accel_y_g', 'accel_y'), col('accel_z_g', 'accel_z')];
  // `Quat_1..4` numbers the four terms without saying which is the scalar one; `quatOrder` decides
  // that from the rows at rest rather than from the numbering.
  const cQuat = [col('quat_x', 'quaternion_x', 'qx', 'Quat_1', 'quat_1'), col('quat_y', 'quaternion_y', 'qy', 'Quat_2', 'quat_2'), col('quat_z', 'quaternion_z', 'qz', 'Quat_3', 'quat_3'), col('quat_w', 'quat_mag', 'quaternion_magnitude', 'qw', 'quat_m', 'Quat_4', 'quat_4')];
  const qAt = quatOrder(rows, cQuat, cGyro);
  const cSec = firstNumericColumn(rows, [cT, cTms, cSync]);
  // A vendor CSV carries the telemetry's integer scalings (deg/s and G x100); a spreadsheet
  // re-export of the same file carries engineering units. Read the data, not the file name: across
  // a whole flight the median turn rate is tens of deg/s and the median specific force about 1 G,
  // so columns two orders of magnitude above that can only be the scaled form.
  const gyroScale = detectScaling(rows, cGyro, 40);
  const accelScale = detectScaling(rows, cAccel, 1.2);
  const out: BrHighRow[] = [];
  for (const r of rows) {
    const g = (c: number, d = 1) => (c >= 0 ? num(r[c]) / d : NaN);
    const t = cSec >= 0 ? g(cSec, cSec === cTms && cTms >= 0 ? 1000 : 1) : NaN;
    if (!Number.isFinite(t)) continue;
    out.push({
      t,
      sync: g(cSync),
      gyro: cGyro.map((c) => g(c, gyroScale)) as [number, number, number],
      accel: cAccel.map((c) => g(c, accelScale)) as [number, number, number],
      quat: quatFrom(g(qAt[0]), g(qAt[1]), g(qAt[2]), g(qAt[3])),
    });
  }
  return out;
}

/**
 * Which column holds each quaternion term, in the internal `[x, y, z, w]` order.
 *
 * A header that writes `Quat_1..4` numbers the terms without saying which is `cos(theta/2)`, and the
 * orderings are not equivalent: taken the wrong way round, an airframe sitting at rest on the pad
 * becomes one turned 180 degrees about its own axis. The rows before liftoff settle it. At rest the
 * attitude is the identity, in which exactly one term is +-1 and the other three are 0, and the term
 * that is not zero is the scalar one. A log that is already turning in its first rows, or whose
 * sensor is mounted at a fixed angle to the airframe, says nothing either way, and the documented
 * order is what is left.
 */
function quatOrder(rows: string[][], cols: number[], cGyro: number[]): number[] {
  const r = rows[0];
  // deg/s: a resting airframe does not turn, so a log whose first sample is already turning cannot
  // be asked which end of the quaternion is which.
  if (!r || cGyro.some((c) => { const v = Math.abs(num(r[c])); return Number.isFinite(v) && v > 2; })) return cols;
  const q = cols.map((c) => num(r[c]));
  if (q.some((v) => !Number.isFinite(v))) return cols;
  const w = q.findIndex((v, j) => Math.abs(Math.abs(v) - 1) < 0.02 && q.every((u, k) => k === j || Math.abs(u) < 0.02));
  if (w < 0) return cols;
  return [...cols.filter((_, j) => j !== w), cols[w]];
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
  const isHigh = isHighRateLog(text);
  if (isHigh) {
    const high = parseCsvHigh(text);
    if (!high.length) warnings.push('Blue Raven high-rate file parsed but contained no rows.');
    return { low: [], high, dialect: 'csv', kind: 'high', warnings };
  }
  const { rows, flightDate, warnings: w } = parseCsvLow(text);
  if (!rows.length) warnings.push('Blue Raven CSV parsed but contained no usable rows.');
  return { low: rows, dialect: 'csv', kind: 'low', flightDate, warnings: [...warnings, ...w] };
}
