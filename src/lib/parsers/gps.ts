import { columnIndex, findColumn, firstNumericColumn, hasColumn, headerKeys, num, parseCsv, readClock } from '../csv.ts';
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
  const keys = new Set(headerKeys(text));
  // The tracker's position, or a generic lat/lon. A ground-station log carries two positions, the
  // receiver's own and the tracker's, and only one of them is the rocket: see `parseCsvGps`.
  return hasColumn(keys, LAT_COLS) && hasColumn(keys, LON_COLS);
}

const FT_M = 3.28084;

/** Position column names, most specific first: the tracker's own, then the generic ones. */
const LAT_COLS = ['lat_deg', 'trk_lat', 'tracker_lat', 'rocket_lat', 'vehicle_lat', 'lat', 'latitude', 'latitude_deg', 'gps_lat'];
const LON_COLS = ['lon_deg', 'trk_lon', 'tracker_lon', 'rocket_lon', 'vehicle_lon', 'lon', 'lng', 'longitude', 'longitude_deg', 'gps_lon'];

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
  // Date and time of day, the way a tracker log writes its clock when it has no single ISO column.
  // `time` also appears in the elapsed list below: which of the two it is gets settled from the
  // column's contents, since 9600.0 and 06:43:42.313 are not the same quantity under one name.
  const cDate = col('date', 'gps_date', 'fix_date', 'date_utc');
  const cClock = col('time', 'time_of_day', 'gps_clock', 'clock');
  const cT = col('t_s', 'time_s', 'elapsed_time_s', 'time', 'seconds', 't');
  const cTms = col('t_ms', 'time_ms', 'millis');
  const cLat = col(...LAT_COLS);
  const cLon = col(...LON_COLS);
  const cAlt = col('alt_asl_ft', 'alt_asl', 'tracker_alt_asl', 'trk_alt_asl', 'gps_altitude_ft_asl', 'altitude_ft_asl', 'alt_ft', 'altitude_ft', 'gps_altitude_ft', 'alt');
  const cAltM = col('alt_m', 'altitude_m', 'alt_asl_m', 'elev_m');
  const cHvel = col('hvel_fps', 'horizontal_velocity_fps', 'ground_speed_fps', 'hvel', 'horzv', 'horz_fps', 'gps_horz_speed_fps', 'speed_fps');
  const cHvelMs = col('hvel_mps', 'ground_speed_mps', 'speed_mps', 'horizontal_velocity_mps');
  const cHead = col('heading_deg', 'heading', 'head', 'course_deg', 'course', 'track_deg');
  const cUp = col('upvel_fps', 'upward_velocity_fps', 'vertical_velocity_fps', 'climb_fps', 'upvel', 'vertv', 'vert_fps', 'gps_vert_speed_fps');
  const cUpMs = col('upvel_mps', 'vertical_velocity_mps', 'climb_mps');
  const cFix = col('fix_type', 'fix', 'gps_fix', 'position_type');
  const cSats = col('sats_total', 'satellites', 'sats', 'sv_count', 'num_sv', 'sat_count', 'tot');
  const cHdop = col('hdop', 'gps_hdop', 'hdop_100', 'dop');
  if (cLat < 0 || cLon < 0) warnings.push('GPS file: latitude/longitude columns not found.');
  // A log of the ground station records where the receiver is, which is not where the rocket is. The
  // vendor's own export names both positions and the tracker's wins; when only the station's is
  // present, the track being drawn is the vehicle's, and that has to be said out loud rather than
  // plotted as a rocket that never left the launcher.
  if (col('gs_lat') >= 0 && !hasColumn(new Set(headerKeys(text)), ['trk_lat', 'tracker_lat', 'rocket_lat', 'vehicle_lat'])) {
    warnings.push('GPS file gives the ground station its own position and no tracker position; the ground is not the rocket.');
  }
  const clockOf = readClock({ stamp: cIso, date: cDate, clock: cClock });
  const cElapsed = firstNumericColumn(rows, [cT, cTms]);

  const out: GpsRow[] = [];
  let epochMs = NaN;
  for (const r of rows) {
    const g = (c: number) => (c >= 0 ? num(r[c]) : NaN);
    const iso = clockOf(r);
    let stamp: string | undefined;
    if (iso && Number.isFinite(Date.parse(iso))) {
      stamp = iso;
      const ms = Date.parse(iso);
      if (!Number.isFinite(epochMs)) epochMs = ms;
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
      t: cElapsed >= 0 ? num(r[cElapsed]) / (cElapsed === cTms && cTms >= 0 ? 1000 : 1) : NaN,
      iso: stamp,
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
    if (!Number.isFinite(epochMs) && cElapsed < 0) {
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
