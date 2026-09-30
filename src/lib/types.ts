/** Shared types. Units: feet, feet/second, feet/second^2, seconds, degrees, Fahrenheit. */

export interface BrLowRow {
  /** seconds from Blue-Raven t=0 (its own clock) */
  t: number;
  sync: number;
  baroTempF: number;
  baroPressureAtm: number;
  batteryMv: number;
  apoMv: number;
  mainMv: number;
  thirdMv: number;
  fourthMv: number;
  outputMa: number;
  /** ground-relative inertial velocity, per the manual (not rocket axes) */
  velDown: number;
  velCross: number;
  velUp: number;
  posDown: number;
  posCross: number;
  altNav: number;
  altBaroAgl: number;
  tilt: number;
  roll: number;
  tiltFuture: number;
  fer: number;
}

export interface BrHighRow {
  t: number;
  sync: number;
  gyro: [number, number, number];
  accel: [number, number, number];
  /** unit quaternion, rotation axis first, magnitude fourth */
  quat: [number, number, number, number];
}

export interface GpsRow {
  /** seconds from the first GPS sample (its own clock) */
  t: number;
  iso?: string;
  lat: number;
  lon: number;
  /** altitude above mean sea level, feet */
  altFt: number;
  hvel: number;
  heading: number;
  upvel: number;
  fixType: number;
  sats: number;
  /** horizontal dilution of precision, when the log carries it */
  hdop?: number;
}

export interface FlightData {
  brLow: BrLowRow[];
  brHigh?: BrHighRow[];
  gps: GpsRow[];
  /** origin (launch pad) recovered from on-pad GPS samples */
  pad: { lat: number; lon: number; altFt: number };
  meta: {
    brDialect: Dialect;
    gpsDialect: Dialect;
    brFileName?: string;
    gpsFileName?: string;
    flightDate?: string;
    /** ms-clock sync offset between the two Blue Raven logs, from sync_code */
    hrSyncOffsetMs?: number;
    /** estimated clock offset applied to the GPS time base, seconds */
    gpsTimeOffsetS?: number;
    warnings: string[];
  };
}

export type Dialect = 'csv' | 'telemetry';

/** A fused trajectory sample. ENU feet relative to the pad; East=x, North=y, Up=z. */
export interface FusedSample {
  t: number;
  e: number;
  n: number;
  u: number;
  ve: number;
  vn: number;
  vu: number;
  /** attitude, when a high-rate log was supplied */
  q?: [number, number, number, number];
  /** tilt/roll from the low-rate log (always available) */
  tilt: number;
  roll: number;
  /** 0..1, how much the filter trusted the inertial nav at this epoch */
  brConfidence: number;
  baroAgl: number;
  altNav: number;
}

export interface FlightEvents {
  t: number;
  code:
    | 'liftoff'
    | 'burnout'
    | 'apogee'
    | 'drogue'
    | 'main'
    | 'landing'
    | 'channel'
    | 'gps-loss'
    | 'gyro-saturation'
    | 'tilt-over-90';
  label: string;
  altFt: number;
}

export interface FlightStats {
  maxAltFt: number;
  maxAltT: number;
  maxVelFps: number;
  maxMach: number;
  maxAccelG: number;
  flightTimeS: number;
  drogueVelFps: number;
  landingVelFps: number;
  driftFt: number;
  padAltFt: number;
  brOnlyMaxErrFt?: number;
  fusedMaxErrFt?: number;
}
