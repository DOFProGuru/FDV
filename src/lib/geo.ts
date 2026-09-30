/** WGS84 geodesy: geodetic <-> ECEF <-> local East-North-Up, output in feet. */

export const FEET_PER_M = 3.28084;
export const WGS84_A = 6378137.0;
export const WGS84_F = 1 / 298.257223563;
const E2 = WGS84_F * (2 - WGS84_F);
const D2R = Math.PI / 180;

export interface Pad {
  lat: number;
  lon: number;
  altFt: number;
}

export interface Ecef {
  x: number;
  y: number;
  z: number;
}

export function geodeticToEcef(latDeg: number, lonDeg: number, altM: number): Ecef {
  const la = latDeg * D2R;
  const lo = lonDeg * D2R;
  const sLa = Math.sin(la);
  const n = WGS84_A / Math.sqrt(1 - E2 * sLa * sLa);
  return {
    x: (n + altM) * Math.cos(la) * Math.cos(lo),
    y: (n + altM) * Math.cos(la) * Math.sin(lo),
    z: (n * (1 - E2) + altM) * sLa,
  };
}

/** Rotate an ECEF offset into the pad-local East/North/Up frame. */
export function ecefOffsetToEnu(d: Ecef, lat0Deg: number, lon0Deg: number): [number, number, number] {
  const la = lat0Deg * D2R;
  const lo = lon0Deg * D2R;
  const sLa = Math.sin(la), cLa = Math.cos(la);
  const sLo = Math.sin(lo), cLo = Math.cos(lo);
  return [
    -sLo * d.x + cLo * d.y,
    -sLa * cLo * d.x - sLa * sLo * d.y + cLa * d.z,
    cLa * cLo * d.x + cLa * sLo * d.y + sLa * d.z,
  ];
}

/** Geodetic -> local ENU in feet, relative to `pad`. */
export function geodeticToEnu(latDeg: number, lonDeg: number, altFt: number, pad: Pad): [number, number, number] {
  const p = geodeticToEcef(latDeg, lonDeg, altFt / FEET_PER_M);
  const p0 = geodeticToEcef(pad.lat, pad.lon, pad.altFt / FEET_PER_M);
  const e = ecefOffsetToEnu({ x: p.x - p0.x, y: p.y - p0.y, z: p.z - p0.z }, pad.lat, pad.lon);
  return [e[0] * FEET_PER_M, e[1] * FEET_PER_M, e[2] * FEET_PER_M];
}

/** Local ENU feet -> geodetic (for reporting where the rocket actually landed). */
export function enuToGeodetic(eFt: number, nFt: number, uFt: number, pad: Pad): { lat: number; lon: number; altFt: number } {
  const la = pad.lat * D2R;
  const lo = pad.lon * D2R;
  const sLa = Math.sin(la), cLa = Math.cos(la);
  const sLo = Math.sin(lo), cLo = Math.cos(lo);
  const e = eFt / FEET_PER_M, n = nFt / FEET_PER_M, u = uFt / FEET_PER_M;
  const d: Ecef = {
    x: -sLo * e - sLa * cLo * n + cLa * cLo * u,
    y: cLo * e - sLa * sLo * n + cLa * sLo * u,
    z: cLa * n + sLa * u,
  };
  const p0 = geodeticToEcef(pad.lat, pad.lon, pad.altFt / FEET_PER_M);
  // Iterate the ECEF inverse (Bowring) — two steps is far below a millimetre here.
  let lat = pad.lat * D2R;
  let lon = Math.atan2(p0.y + d.y, p0.x + d.x);
  let alt = pad.altFt / FEET_PER_M;
  const X = p0.x + d.x, Y = p0.y + d.y, Z = p0.z + d.z;
  for (let i = 0; i < 4; i++) {
    const sLa2 = Math.sin(lat);
    const nR = WGS84_A / Math.sqrt(1 - E2 * sLa2 * sLa2);
    alt = Math.hypot(X, Y) / Math.cos(lat) - nR;
    lat = Math.atan2(Z, Math.hypot(X, Y) * (1 - (E2 * nR) / (nR + alt)));
  }
  return { lat: lat / D2R, lon: lon / D2R, altFt: alt * FEET_PER_M };
}

/** Horizontal speed + compass heading -> East/North velocity components (ft/s). */
export function headingToEnu(hvel: number, headingDeg: number): [number, number] {
  const h = headingDeg * D2R;
  return [hvel * Math.sin(h), hvel * Math.cos(h)];
}

export function enuToHeading(ve: number, vn: number): { hvel: number; heading: number } {
  return { hvel: Math.hypot(ve, vn), heading: (Math.atan2(ve, vn) / D2R + 360) % 360 };
}
