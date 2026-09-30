import { columnIndex, findColumn, num, parseCsv } from '../csv.ts';
import type { Dialect, GpsRow } from '../types.ts';

export interface GpsParseResult {
  rows: GpsRow[];
  dialect: Dialect;
  /** UTC date of the first fix, when the file carried wall-clock stamps */
  flightDate?: string;
  warnings: string[];
}

export function looksLikeGps(text: string): boolean {
  const head = text.slice(0, 4000).toLowerCase();
  if (/@\s*gps_stat|@frst_fix|\$g[png](gga|rmc|vtg)/.test(head)) return true;
  const first = head.split('\n').find((l) => l.includes(','));
  if (!first) return false;
  const cols = new Set(first.split(',').map((c) => c.trim().toLowerCase()));
  const hasLat = ['lat_deg', 'lat', 'latitude', 'latitude_deg', 'gps_lat'].some((c) => cols.has(c));
  const hasLon = ['lon_deg', 'lon', 'lng', 'longitude', 'longitude_deg', 'gps_lon'].some((c) => cols.has(c));
  return hasLat && hasLon;
}

const FT_M = 3.28084;

/** NMEA positions are degrees + decimal minutes; split on the minute boundary. */
function dmsToDeg(raw: number, hemi: string): number {
  const deg = Math.floor(Math.abs(raw) / 100);
  const v = deg + (Math.abs(raw) - deg * 100) / 60;
  return hemi === 'S' || hemi === 'W' ? -v : v;
}

/** true when a token is the tracker's `####` "no value yet" placeholder */
const missing = (s: string) => s === '####' || s === '###';

// --- native telemetry / NMEA -------------------------------------------------
/**
 * Parse the documented `@GPS_STAT` packet:
 *   @GPS_STAT 203 2020 11 15 01:20:21.986 CRC_OK TRK second TrkAlt5655 lt39.55612 ln-105.1032
 *   Vel0 -1550 Fix3 #9 42 0 0 000_00_00 000_00_00 000_00_00 000_00_00
 * plus NMEA GGA/RMC as a convenience for other loggers.
 */
function parseTelemetry(text: string): { rows: GpsRow[]; flightDate?: string } {
  const out: GpsRow[] = [];
  let flightDate: string | undefined;
  let epochMs = NaN;

  for (const line of text.split('\n')) {
    const l = line.trim();
    if (!l) continue;

    if (/@\s*GPS_STAT/i.test(l)) {
      const stamp = /(?:^|\s)(20\d{2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?/.exec(l);
      const alt = /TrkAlt\s*(\S+)/i.exec(l);
      const lt = /lt\s*(-?\d+(?:\.\d+)?|####)/i.exec(l);
      const ln = /ln\s*(-?\d+(?:\.\d+)?|####)/i.exec(l);
      const vel = /Vel\s*(\S+)\s+(\S+)/i.exec(l);
      const fix = /Fix\s*(\d)/i.exec(l);
      const sats = /#\s*(\d+)/.exec(l);
      if (!lt || !ln || missing(lt[1]) || missing(ln[1])) continue;

      const iso = stamp
        ? `${stamp[1]}-${stamp[2].padStart(2, '0')}-${stamp[3].padStart(2, '0')}T${stamp[4].padStart(2, '0')}:${stamp[5]}:${stamp[6]}.${(stamp[7] ?? '000').padEnd(3, '0')}Z`
        : undefined;
      if (iso && !Number.isFinite(epochMs)) {
        epochMs = Date.parse(iso);
        flightDate = iso.slice(0, 10);
      }
      out.push({
        t: 0,
        iso,
        lat: num(lt[1]),
        lon: num(ln[1]),
        altFt: alt && !missing(alt[1]) ? num(alt[1]) : NaN,
        // Documented order is horizontal velocity then upward velocity; heading is reported
        // only when the ground track exceeds 10 ft/s and is often absent.
        hvel: vel && !missing(vel[1]) ? num(vel[1]) : NaN,
        heading: NaN,
        upvel: vel && !missing(vel[2]) ? num(vel[2]) : NaN,
        fixType: fix ? num(fix[1]) : 3,
        sats: sats ? num(sats[1]) : 0,
      });
      continue;
    }

    const gga = /\$G[PNG]GA,(?:\d{6}(?:\.\d+)?,)?(\d+\.?\d*)([NS]),(\d+\.?\d*)([EW]),(\d),(\d+),[^,]*,[^,]*,(-?\d+\.?\d*)/.exec(l);
    if (gga) {
      out.push({
        t: 0, lat: dmsToDeg(parseFloat(gga[1]), gga[2]), lon: dmsToDeg(parseFloat(gga[3]), gga[4]),
        altFt: num(gga[7]) * FT_M, hvel: NaN, heading: NaN, upvel: NaN,
        fixType: num(gga[5]), sats: num(gga[6]),
      });
      continue;
    }

    const rmc = /\$G[PNG]RMC,(?:\d{6}(?:\.\d+)?,)?([AV]),(\d+\.?\d*)([NS]),(\d+\.?\d*)([EW]),(?:[^,]*,){2}(?:\d{6})?/.exec(l);
    if (rmc) {
      const spdKt = num(/(?:[^,]*,){6}(\d+\.?\d*)/.exec(l)?.[1]);
      const crs = num(/(?:[^,]*,){7}(\d+\.?\d*)/.exec(l)?.[1]);
      out.push({
        t: 0, lat: dmsToDeg(parseFloat(rmc[2]), rmc[3]), lon: dmsToDeg(parseFloat(rmc[4]), rmc[5]),
        altFt: NaN,
        hvel: (Number.isFinite(spdKt) ? spdKt : 0) * 1.68781,
        heading: Number.isFinite(crs) ? crs : NaN,
        upvel: NaN, fixType: rmc[1] === 'A' ? 3 : 0, sats: 0,
      });
    }
  }

  const rows: GpsRow[] = out.map((r, i) => ({ ...r, t: Number.isFinite(epochMs) && r.iso ? (Date.parse(r.iso) - epochMs) / 1000 : i * 0.1 }));
  rows.sort((a, b) => a.t - b.t);
  return { rows, flightDate };
}

// --- CSV ---------------------------------------------------------------------
function parseCsvGps(text: string): { rows: GpsRow[]; flightDate?: string; warnings: string[] } {
  const { header, rows } = parseCsv(text);
  const idx = columnIndex(header);
  const col = (...a: string[]) => findColumn(idx, a);
  const warnings: string[] = [];

  const cIso = col('t_iso', 'time_iso', 'utc', 'utc_time', 'timestamp', 'datetime', 'date_time', 'gps_time', 'time_utc');
  const cT = col('t_s', 'time_s', 'elapsed_time_s', 'time', 'seconds', 't');
  const cTms = col('t_ms', 'time_ms', 'millis');
  const cLat = col('lat_deg', 'lat', 'latitude', 'latitude_deg', 'gps_lat');
  const cLon = col('lon_deg', 'lon', 'lng', 'longitude', 'longitude_deg', 'gps_lon');
  const cAlt = col('alt_ft', 'altitude_ft', 'alt_asl_ft', 'alt', 'gps_altitude_ft', 'altitude_ft_asl');
  const cAltM = col('alt_m', 'altitude_m', 'alt_asl_m', 'elev_m');
  const cHvel = col('hvel_fps', 'horizontal_velocity_fps', 'ground_speed_fps', 'hvel', 'speed_fps');
  const cHvelMs = col('hvel_mps', 'ground_speed_mps', 'speed_mps', 'horizontal_velocity_mps');
  const cHead = col('heading_deg', 'heading', 'course_deg', 'course', 'track_deg');
  const cUp = col('upvel_fps', 'upward_velocity_fps', 'vertical_velocity_fps', 'climb_fps', 'upvel');
  const cUpMs = col('upvel_mps', 'vertical_velocity_mps', 'climb_mps');
  const cFix = col('fix_type', 'fix', 'gps_fix', 'position_type');
  const cSats = col('sats_total', 'satellites', 'sats', 'sv_count', 'num_sv', 'sat_count');
  const cHdop = col('hdop', 'gps_hdop', 'hdop_100', 'dop');
  if (cLat < 0 || cLon < 0) warnings.push('GPS file: latitude/longitude columns not found.');

  const out: GpsRow[] = [];
  let epochMs = NaN;
  for (const r of rows) {
    const g = (c: number) => (c >= 0 ? num(r[c]) : NaN);
    const isoRaw = cIso >= 0 ? (r[cIso] ?? '').trim() : '';
    let iso: string | undefined;
    if (/[0-9]/.test(isoRaw)) {
      const norm = (isoRaw.includes('T') ? isoRaw : isoRaw.replace(' ', 'T'));
      iso = /Z|[+-]\d{2}:?\d{2}$/.test(norm) ? norm : norm + 'Z';
      if (!Number.isFinite(Date.parse(iso))) iso = undefined;
      else if (!Number.isFinite(epochMs)) epochMs = Date.parse(iso);
    }

    // Metric columns are only consulted when the feet column is absent, so a file that carries
    // both is never scaled twice.
    let alt = g(cAlt);
    if (!Number.isFinite(alt)) alt = g(cAltM) * FT_M;
    let hvel = g(cHvel);
    if (!Number.isFinite(hvel)) hvel = g(cHvelMs) * FT_M;
    let upvel = g(cUp);
    if (!Number.isFinite(upvel)) upvel = g(cUpMs) * FT_M;

    out.push({
      t: cT >= 0 ? g(cT) : cTms >= 0 ? g(cTms) / 1000 : NaN,
      iso,
      lat: g(cLat),
      lon: g(cLon),
      altFt: alt,
      hvel,
      heading: g(cHead),
      upvel,
      fixType: g(cFix),
      sats: g(cSats),
      hdop: cHdop >= 0 ? g(cHdop) : undefined,
    });
  }

  const usable = out.filter((r) => Number.isFinite(r.lat) && Number.isFinite(r.lon));
  if (Number.isFinite(epochMs)) {
    for (const r of usable) {
      const ms = r.iso ? Date.parse(r.iso) : NaN;
      r.t = Number.isFinite(ms) ? (ms - epochMs) / 1000 : NaN;
    }
  }
  if (usable.some((r) => !Number.isFinite(r.t))) {
    let last = 0;
    for (const r of usable) {
      if (!Number.isFinite(r.t)) r.t = last + 0.1;
      last = r.t;
    }
    if (!Number.isFinite(epochMs) && cT < 0 && cTms < 0) {
      warnings.push('GPS file has no usable time column; assuming the documented 10 Hz cadence.');
    }
  }
  usable.sort((a, b) => a.t - b.t);
  if (!usable.length) warnings.push('GPS file parsed but contained no usable positions.');
  return { rows: usable, flightDate: usable.find((r) => r.iso)?.iso?.slice(0, 10), warnings };
}

export function parseGps(text: string): GpsParseResult {
  if (/@\s*GPS_STAT|\$G[PNG]?(GGA|RMC|VTG)/i.test(text.slice(0, 20000))) {
    const { rows, flightDate } = parseTelemetry(text);
    return { rows, dialect: 'telemetry', flightDate, warnings: rows.length ? [] : ['GPS telemetry parsed but contained no fixes.'] };
  }
  const { rows, flightDate, warnings } = parseCsvGps(text);
  return { rows, dialect: 'csv', flightDate, warnings };
}
