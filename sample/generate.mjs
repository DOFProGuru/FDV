/**
 * Physics-based sample flight generator.
 *
 * Runs a 3-DOF point-mass trajectory with ISA atmosphere and wind shear, then
 * *emulates* the two sensors (Blue Raven 50 Hz inertial nav + 500 Hz IMU, and a
 * 10 Hz u-blox-style GPS) including the failure modes the Blue Raven manual
 * describes. Field names, units and scalings follow FORMATS.md.
 *
 * Outputs, per flight, into public/data/:
 *   <id>_blue_raven_low.csv     50 Hz  low-rate log
 *   <id>_blue_raven_high.csv    500 Hz high-rate log (optional input for the app)
 *   <id>_gps.csv                10 Hz  GPS tracker log
 * and into sample/truth/:
 *   <id>_truth.json             truth state, for the verification harness only
 *
 * The nav/GPS models are engineering emulations, not the devices' real filters;
 * they reproduce the documented *behaviour* (drift, gyro-saturation divergence,
 * GPS vertical noise, dropouts) so the reconstruction pipeline has something
 * honest to chew on.
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const OUT = join(ROOT, 'public', 'data');
const TRUTH = join(ROOT, 'sample', 'truth');
mkdirSync(OUT, { recursive: true });
mkdirSync(TRUTH, { recursive: true });

const FT = 1 / 0.3048; // metres -> feet
const MPH2FPS = 1.46667;

// --- deterministic RNG -------------------------------------------------------
function mulberry32(seed) {
  return function () {
    seed |= 0; seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const gaussFactory = (rnd) => () => {
  let u = 0, v = 0;
  while (u === 0) u = rnd();
  while (v === 0) v = rnd();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
};

// --- atmosphere (ISA, troposphere + isothermal stratopause region) ------------
const T0 = 288.15, LAPSE = -0.0065, P0 = 101325, G = 9.80665, R_AIR = 287.05, T_TR = 216.65, H_TR = 11000;
const P_TR = P0 * Math.pow(T_TR / T0, -G / (LAPSE * R_AIR));

function isa(hM) {
  let T, p;
  if (hM <= H_TR) {
    T = Math.max(T0 + LAPSE * hM, T_TR);
    p = P0 * Math.pow(T / T0, -G / (LAPSE * R_AIR));
  } else {
    T = T_TR;
    p = P_TR * Math.exp((-G * (hM - H_TR)) / (R_AIR * T_TR));
  }
  return { T, p, rho: Math.max(0, p / (R_AIR * T)) };
}
// Invert pressure altitude (what a baro actually reads) back to a geometric height.
// h = (T0/L) * ((p/P0)^(L R / g) - 1) below the tropopause, barometric formula in the isothermal layer.
function hFromPressure(p) {
  if (p >= P_TR) return (T0 * Math.pow(p / P0, -LAPSE * R_AIR / G) - T0) / LAPSE;
  return H_TR - (R_AIR * T_TR / G) * Math.log(p / P_TR);
}

// --- helpers -----------------------------------------------------------------
const norm = (v) => Math.hypot(v[0], v[1], v[2]);
const unit = (v) => { const n = norm(v) || 1; return [v[0] / n, v[1] / n, v[2] / n]; };
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const lerp = (a, b, t) => a + (b - a) * t;

/** Small-angle misalignment matrix about East(x) then North(y): rail not perfectly vertical. */
function rotSmall(eastRad, northRad) {
  return [
    [1, 0, -northRad],
    [0, 1, eastRad],
    [northRad, -eastRad, 1],
  ];
}
const matVec = (m, v) => [
  m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
  m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
  m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
];

function thrustAt(curve, t) {
  if (t <= curve[0][0]) return 0;
  for (let i = 1; i < curve.length; i++) {
    if (t <= curve[i][0]) {
      const [t0, f0] = curve[i - 1], [t1, f1] = curve[i];
      return lerp(f0, f1, (t - t0) / (t1 - t0));
    }
  }
  return 0;
}

/** Total impulse of a thrust curve in Ns (trapezoid). */
function curveImpulse(curve) {
  let e = 0;
  for (let i = 1; i < curve.length; i++) e += ((curve[i][1] + curve[i - 1][1]) / 2) * (curve[i][0] - curve[i - 1][0]);
  return e;
}

// --- flight configurations ---------------------------------------------------
// Thrust curves are [t (s from stage ignition), thrust in NEWTONS] — Cesaroni/Estes style,
// where the number in e.g. "M1670-9" is average thrust in N. `diamIn` sets the airframe drag
// area (Cd 0.5); chute `cdA` is in ft^2 and is sized for a realistic descent rate.
// caseKg is ejected at that stage's burnout (booster / motor casing).
const FLIGHTS = [
  {
    id: 'f17-nominal',
    // Truth-time of each log's first sample. The tracker stamps itself with its own wall clock and
    // nothing ties the two logs' t=0 markers together, so `gps - br` (+0.0, -3.5, +6.75 s here) is
    // the unknown the app must recover from the signals alone.
    padHoldS: 10, brLogStartS: -4.5, gpsLogStartS: -4.5,
    name: "Adrian's BRav",
    pad: { lat: 34.2591, lon: -106.3631, altFt: 5212 }, // Elephant Butte-ish, NM
    wind: [[0, 6], [2000, 12], [6000, 22], [12000, 30]], // ft, mph toward East
    windDirDeg: 78,
    launchTiltDeg: 1.2, launchAzDeg: 0,
    dryMassKg: 2.6, diamIn: 4.0,
    // H1260-1200: 1260 N avg x 2.2 s = 2.8 kNs on a 4" / 4.5 kg airframe
    stages: [{ t0: 0, propKg: 1.40, caseKg: 0.55, isp: 190, curve: [[0, 60], [0.05, 1490], [0.9, 1230], [1.6, 790], [2.0, 400], [2.2, 40], [2.35, 0]] }],
    deploys: [{ kind: 'drogue', at: 'apogee', cdA: 1.5, impulseMs: 25, impulseG: 9 },
              { kind: 'main', altAgl: 700, cdA: 20, impulseMs: 35, impulseG: 14 }],
    gpsQuality: 'good',
    seed: 1711,
    duration: 200,
  },
  {
    id: 'f31-twostage',
    padHoldS: 10, brLogStartS: -3.0, gpsLogStartS: -6.5,
    name: 'Two-Stage Dusk',
    pad: { lat: 34.2591, lon: -106.3631, altFt: 5212 },
    wind: [[0, 8], [3000, 18], [10000, 34], [20000, 48]],
    windDirDeg: 95,
    launchTiltDeg: 2.0, launchAzDeg: 6,
    dryMassKg: 4.4, diamIn: 5.4,
    stages: [
      // booster: M2468, 2468 N avg x 2.0 s = 5.1 kNs, 2.6 kg prop, 1.0 kg case ejected at burnout
      { t0: 0, propKg: 2.60, caseKg: 1.00, isp: 192, curve: [[0, 80], [0.06, 2790], [1.0, 2520], [1.6, 1180], [2.0, 150], [2.15, 0]] },
      // sustainer lights on the predicted-tilt trigger 2.6 s after burnout: L1350, 1350 N avg x 3.2 s
      { t0: 'sep+2.6', propKg: 1.75, caseKg: 0.0, isp: 200, curve: [[0, 40], [0.05, 1500], [1.4, 1330], [2.7, 640], [3.2, 110], [3.35, 0]] },
    ],
    deploys: [
      { kind: 'drogue', at: 'apogee', cdA: 1.7, impulseMs: 25, impulseG: 8 },
      { kind: 'main', altAgl: 850, cdA: 26, impulseMs: 40, impulseG: 12 },
    ],
    gpsQuality: 'good',
    seed: 31337,
    duration: 260,
  },
  {
    id: 'f52-tumble',
    padHoldS: 10, brLogStartS: -8.0, gpsLogStartS: -1.25,
    name: 'Tumble + Degraded GPS',
    pad: { lat: 40.6892, lon: -117.9732, altFt: 4478 }, // Black Rock, NV
    wind: [[0, 4], [2000, 9], [8000, 16], [16000, 24]],
    windDirDeg: 250,
    launchTiltDeg: 1.0, launchAzDeg: 0,
    dryMassKg: 3.4, diamIn: 4.0,
    // L1440: 1440 N avg x 2.55 s = 3.8 kNs
    stages: [{ t0: 0, propKg: 1.95, caseKg: 0.70, isp: 192, curve: [[0, 65], [0.05, 1780], [1.2, 1500], [1.9, 880], [2.4, 210], [2.55, 0]] }],
    // Loose drogue: airframe tumbles hard behind it for a few seconds after ejection — the gyro
    // exceeds its +/-2000 deg/s limit and the nav loses track of "up". It then settles into a
    // nose-down oscillation, but the accumulated nav error does not recover on its own.
    deploys: [{ kind: 'drogue', at: 'apogee', cdA: 1.6, impulseMs: 22, impulseG: 11, tumbleFrom: 0.35, tumbleFor: 7.5 },
              { kind: 'main', altAgl: 650, cdA: 22, impulseMs: 35, impulseG: 15 }],
    gpsQuality: 'degraded',
    seed: 5202,
    duration: 220,
  },
];

// --- trajectory integration --------------------------------------------------
/** Pendulum sway of a parachute-suspended airframe, in radians from vertical. */
function pendulum(tAfter, amp) {
  const a = amp * Math.exp(-tAfter / 12) * Math.sin(tAfter * 1.9) + 0.02 * Math.sin(tAfter * 0.7);
  return unit([Math.sin(a), Math.cos(a) * 0.3 * Math.sin(a), Math.cos(a)]);
}

function simulate(cfg) {
  const rnd = mulberry32(cfg.seed);
  const gauss = gaussFactory(rnd);
  const dt = 0.002;
  const N = Math.floor(cfg.duration / dt);
  const g = G;
  const padAltM = cfg.pad.altFt * 0.3048;

  // Launch heading in the ENU-ish frame we integrate in: x=East, y=North, z=Up.
  const azr = (cfg.launchAzDeg * Math.PI) / 180;
  const tiltR = (cfg.launchTiltDeg * Math.PI) / 180;
  const launchDir = unit([Math.sin(tiltR) * Math.sin(azr), Math.sin(tiltR) * Math.cos(azr), Math.cos(tiltR)]);
  const airCdA = 0.5 * Math.PI * Math.pow((cfg.diamIn * 0.0254) / 2, 2); // m^2, Cd = 0.5

  const stages = cfg.stages.map((s) => {
    const impulseNs = curveImpulse(s.curve);
    return {
      ...s,
      // Propellant mass is derived from the curve so the motor can never fail to burn out.
      propKg: impulseNs / ((s.isp ?? 190) * G) * 0.93,
      impulseNs,
      burned: 0,
      tIgnite: typeof s.t0 === 'number' ? s.t0 : null,
    };
  });

  let pos = [0, 0, 0];       // metres relative to pad
  let vel = [0, 0, 0];
  let mass = cfg.dryMassKg + stages.reduce((a, s) => a + s.propKg, 0);
  const dryTotal = mass;

  const rec = [];            // truth samples at 500 Hz
  const events = [];         // {t, type}
  let sepTime = null;
  let drogueTime = null, mainTime = null, landTime = null;
  let tumbleStart = null, tumbleEnd = null;
  let apogeeAlt = 0, apogeeT = 0;
  let prevVelMag = 0, maxMotorAccel = 0, maxDragAccel = 0, maxDeployG = 0, maxLandingG = 0;
  let burnoutTilt = null, ascentRoll = 0;
  let landed = false;
  let impactVel = null;
  let impulse = { g: 0, until: 0 };

  // Both loggers are armed at the pad, so a real log opens several seconds before ignition. The
  // airframe is static on the rail: the accelerometer reads 1 g up, nothing moves.
  const padHoldS = cfg.padHoldS ?? 10;
  const padState = isa(padAltM);
  const padTilt = (Math.acos(clamp(launchDir[2], -1, 1)) * 180) / Math.PI;
  for (let t = -padHoldS; t < -1e-9; t += dt) {
    rec.push({
      t, p: [0, 0, 0], v: [0, 0, 0], a: [0, 0, 0], sf: [0, 0, 9.80665],
      axis: launchDir, bodyRate: 0, rollRate: 0, tilt: padTilt,
      pBaro: padState.p, rho: padState.rho, landed: false, onPad: true,
    });
  }

  for (let i = 0; i <= N; i++) {
    const t = i * dt;

    // Resolve delayed ignitions once separation is known.
    for (const s of stages) {
      if (s.tIgnite === null && typeof s.t0 === 'string' && sepTime !== null) {
        const m = s.t0.match(/sep\+(\d+\.?\d*)/);
        s.tIgnite = sepTime + (m ? parseFloat(m[1]) : 0);
      }
    }

    // Stage ignition event.
    for (const s of stages) {
      if (s.tIgnite !== null && !s.ignited && t >= s.tIgnite) { s.ignited = true; events.push({ t, type: 'ignition' }); }
    }
    // Burnout events.
    for (const s of stages) {
      if (s.ignited && !s.done && s.burned >= s.propKg) {
        s.done = true;
        events.push({ t, type: stages.filter((q) => q.done).length === 1 ? 'burnout' : 'burnout2' });
        // Separation is the FIRST stage's burnout — the sustainer's ignition time is referenced to it.
        if (s === stages[0] && cfg.stages.length > 1) sepTime = t;
      }
    }

    // Mass state: propellant is consumed continuously, empty cases are ejected at burnout.
    let remaining = 0;
    for (const s of stages) {
      if (!s.done) remaining += Math.max(0, s.propKg - s.burned);
      else if (!s.ejected) { s.ejected = true; events.push({ t, type: 'sep' }); }
    }
    const ejectedKg = stages.filter((s) => s.done).reduce((a, s) => a + (s.caseKg ?? 0), 0);
    mass = dryTotal - stages.reduce((a, s) => a + Math.min(s.propKg, s.burned), 0) - ejectedKg;

    // Thrust along current attitude (fins keep airframe pointed into the flow during boost).
    const vRelWind = [vel[0], vel[1], vel[2]];
    let attitude = norm(vRelWind) > 1 ? unit(vRelWind) : launchDir;
    if (!events.some((e) => e.type === 'liftoff')) attitude = launchDir;
    else if (mainTime !== null) attitude = pendulum(t - mainTime, 0.12);      // hangs nose-up under main
    else if (drogueTime !== null) attitude = unit([vel[0], vel[1], -Math.abs(vel[2]) - 1]); // nose-down on drogue

    let thrust = 0;
    for (const s of stages) {
      if (!s.ignited || s.done) continue;
      const tau = t - s.tIgnite;
      const fN = thrustAt(s.curve, tau);
      thrust += fN;
      s.burned += (fN / ((s.isp ?? 190) * 9.80665)) * dt;
    }

    // Wind profile (toward windDirDeg, clockwise from North).
    const hFt = pos[2] * FT;
    let wMag = 0;
    {
      const w = cfg.wind;
      if (hFt <= w[0][0]) wMag = w[0][1];
      else if (hFt >= w[w.length - 1][0]) wMag = w[w.length - 1][1];
      else for (let k = 1; k < w.length; k++) if (hFt <= w[k][0]) { wMag = lerp(w[k - 1][1], w[k][1], (hFt - w[k - 1][0]) / (w[k][0] - w[k - 1][0])); break; }
    }
    const wR = (cfg.windDirDeg * Math.PI) / 180;
    const wind = [wMag * MPH2FPS * Math.sin(wR), wMag * MPH2FPS * Math.cos(wR), 0];
    const vRel = [vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2]];
    const vRelM = norm(vRel);

    const { rho, p } = isa(padAltM + pos[2]);
    const ft2 = 0.092903;
    let cdA = airCdA; // bare airframe, m^2
    if (mainTime !== null && t > mainTime) cdA = cfg.deploys[1].cdA * ft2;
    else if (drogueTime !== null && t > drogueTime) cdA = cfg.deploys[0].cdA * ft2;

    const dragMag = 0.5 * rho * vRelM * vRelM * cdA;
    const drag = vRelM > 1e-6 ? vRel.map((c) => (-dragMag / vRelM) * c) : [0, 0, 0];

    const F = [
      thrust * attitude[0] + drag[0],
      thrust * attitude[1] + drag[1],
      thrust * attitude[2] + drag[2] - mass * g,
    ];
    const accel = F.map((c) => c / mass);

    // Specific force the accelerometers would read (thrust + drag, no gravity).
    const specific = [accel[0], accel[1], accel[2] + g];

    maxMotorAccel = Math.max(maxMotorAccel, thrust > 100 ? norm(accel) / 9.80665 : 0);
    maxDragAccel = Math.max(maxDragAccel, dragMag / mass / 9.80665);

    // Deployment triggers (evaluated on truth state).
    const agl = pos[2] * FT;
    // Physical trigger: the pad lets go once thrust-to-weight passes 1.05 (velocity-based
    // detection would deadlock against the ground support below).
    if (!events.some((e) => e.type === 'liftoff') && thrust * attitude[2] > mass * g * 1.05) events.push({ t, type: 'liftoff' });
    if (vel[2] > 0 && agl > apogeeAlt) { apogeeAlt = agl; apogeeT = t; }
    if (events.some((e) => e.type === 'liftoff') && drogueTime === null && vel[2] < -0.5 && t > 3) {
      drogueTime = t; events.push({ t, type: 'apogee' }); events.push({ t, type: 'drogue' });
    }
    if (drogueTime !== null && mainTime === null && landTime === null && agl < cfg.deploys[1].altAgl && vel[2] < 0) {
      mainTime = t; events.push({ t, type: 'main' });
    }
    if (mainTime !== null && landTime === null && pos[2] <= 0.02 && vel[2] > -1.5) {
      landTime = t; events.push({ t, type: 'landing' }); landed = true;
    }

    // Ejection impulse (short, high-G, opposite velocity).
    let impulseAccel = [0, 0, 0];
    for (const d of cfg.deploys) {
      const trig = d.kind === 'drogue' ? drogueTime : mainTime;
      if (trig !== null && t >= trig && t < trig + d.impulseMs / 1000) {
        const dir = unit([vel[0], vel[1], Math.abs(vel[2]) + 1]);
        impulseAccel = dir.map((c) => -c * d.impulseG * 9.80665);
        maxDeployG = Math.max(maxDeployG, d.impulseG);
      }
    }
    maxLandingG = Math.max(maxLandingG, landTime !== null && t - landTime < 0.1 ? norm(accel) / 9.80665 : 0);

    // Integrate. The launch pad supports the airframe until thrust exceeds weight (otherwise the
    // first step, before the thrust curve ramps up, would dig the rocket into the ground).
    if (!landed) {
      if (!events.some((e) => e.type === 'liftoff')) {
        for (let k = 0; k < 3; k++) { pos[k] = 0; vel[k] = 0; }
      } else {
        for (let k = 0; k < 3; k++) {
          vel[k] += (accel[k] + impulseAccel[k] * 0.001) * dt;
          pos[k] += vel[k] * dt;
          if (pos[2] < 0 && vel[2] < 0) {
            if (drogueTime !== null) impactVel = Math.max(impactVel ?? 0, Math.abs(vel[2]));
            vel[2] = 0; pos[2] = 0;
          }
        }
      }
    }

    // Body rate: quiet on the rail/boost, violent briefly after a loose drogue ejection, then a
    // settled nose-down oscillation under drogue and a gentle swing under main.
    let bodyRate = 0, rollRate = 1.2;
    const dep = cfg.deploys[0];
    if (dep.tumbleFrom !== undefined && drogueTime !== null) {
      if (t >= drogueTime + dep.tumbleFrom && t < drogueTime + dep.tumbleFrom + (dep.tumbleFor ?? 6)) bodyRate = 2650;
    }
    if (drogueTime !== null && t > drogueTime && t < (mainTime ?? cfg.duration) && bodyRate === 0) bodyRate = 45;
    if (mainTime !== null && t > mainTime) bodyRate = 12;
    if (t < 3) bodyRate = Math.abs(gauss()) * 0.6;
    ascentRoll += rollRate * dt;

    // Attitude for the marker: along velocity, or tumbling.
    let axis = attitude;
    if (bodyRate > 300) {
      const ph = t * (bodyRate * Math.PI) / 180;
      axis = unit([Math.sin(ph) * 0.85, Math.cos(ph) * 0.35, Math.cos(ph) * 0.5]);
    } else if (landed) axis = launchDir;

    const tilt = (Math.acos(clamp(axis[2], -1, 1)) * 180) / Math.PI;

    if (burnoutTilt === null && stages.every((s) => s.done) && t > (stages[stages.length - 1].tIgnite ?? 0) + 3) {
      burnoutTilt = tilt;
    }

    rec.push({
      t,
      p: [pos[0], pos[1], pos[2]],
      v: [vel[0], vel[1], vel[2]],
      a: accel,
      sf: [specific[0] + impulseAccel[0], specific[1] + impulseAccel[1], specific[2] + impulseAccel[2]],
      axis,
      bodyRate,
      rollRate,
      tilt,
      pBaro: p,
      rho,
      landed,
    });

    if (landTime !== null && t > landTime + 6) break;
  }

  const out = { rec, events, impactVel, apogeeAlt, apogeeT, maxMotorAccel, maxDragAccel, maxDeployG, maxLandingG, dryTotal };
  return out;
}

// --- Blue Raven emulation ----------------------------------------------------
function blueRavenSensors(cfg, sim) {
  const rnd = mulberry32(cfg.seed + 991);
  const gauss = gaussFactory(rnd);

  // Rail-direction misalignment (the BR derives this from first motion, so it is small)
  // plus an integrated gyro bias: this is what makes the inertial nav drift.
  const mis = rotSmall((cfg.launchTiltDeg * 0.35 + 0.6) * Math.PI / 180 * (rnd() > 0.5 ? 1 : -1),
                       (0.8 + rnd() * 0.6) * Math.PI / 180);
  const accelBias = [0.004 + rnd() * 0.004, 0.0035 + rnd() * 0.004, -0.006 - rnd() * 0.005].map((b) => b * 9.80665);
  const gyroBias = (0.35 + rnd() * 0.25) * (Math.PI / 180); // rad/s

  let navP = [0, 0, 0], navV = [0, 0, 0];
  let leakP = [0, 0, 0], leakV = [0, 0, 0]; // gravity-leak divergence while tumbling
  // The altimeter's barometric altitude is ASL by construction; the log reports it as AGL, so the
  // pad height is subtracted here rather than left for the reader to guess. Starting the av-bay
  // lag filter on the pad pressure avoids a spurious ramp-up in the first second of the log.
  let baroAlt = hFromPressure(isa(cfg.pad.altFt / FT).p) * FT;
  const tauVent = 0.16; // av-bay vent lag, s
  let roll = 0;
  const dt = sim.rec[1].t - sim.rec[0].t;
  const low = [];
  const high = [];
  let lastSync = 0;

  const fer = { liftoff: false, apogee: false, pressInc: false, apo: false, main: false, third: false, fourth: false, eciNeg: false, accelNeg: false, tilt90: false };
  const evAt = (type) => { const e = sim.events.find((x) => x.type === type); return e ? e.t : Infinity; };

  const logStart = cfg.brLogStartS ?? 0;
  for (const s of sim.rec) {
    const t = s.t;
    if (t < logStart) continue;
    // 500 Hz channel: accelerometers (with bias + noise) and gyros (clipped at the +/-2000 deg/s limit).
    const gyroLim = 2000;
    const rawRate = s.bodyRate;
    const clipped = Math.min(rawRate, gyroLim);
    const sat = rawRate > gyroLim;

    const ax = s.sf[0] / 9.80665 + accelBias[0] / 9.80665 + gauss() * 0.012;
    const ay = s.sf[1] / 9.80665 + accelBias[1] / 9.80665 + gauss() * 0.012;
    const az = s.sf[2] / 9.80665 + accelBias[2] / 9.80665 + gauss() * 0.012;
    const spin = t * (clipped * Math.PI) / 180;
    const gx = sat ? gyroBias * 180 / Math.PI + gauss() * 40 : Math.cos(spin) * clipped * 0.6 + gauss() * 1.2;
    const gy = sat ? gyroBias * 180 / Math.PI * 0.7 + gauss() * 40 : Math.sin(spin) * clipped * 0.6 + gauss() * 1.2;
    const gz = clipped + gauss() * 1.5;

    if (Math.round(t / dt) % Math.round(1 / (50 * dt)) === 0) {
      // Mechanise the nav at 50 Hz using the *measured* specific force, in a frame that is
      // misaligned by `mis` and drifting by the gyro bias -> exactly the failure the manual describes.
      const dtL = low.length === 0 ? 0 : t - low[low.length - 1].t;
      const frameRot = rotSmall(gyroBias * t * 0.55, gyroBias * t * 0.45);
      const sfNav = matVec(mis, matVec(frameRot, [s.sf[0], s.sf[1], s.sf[2] - 9.80665]));

      if (dtL > 0) {
        for (let k = 0; k < 3; k++) navV[k] += sfNav[k] * dtL;
        for (let k = 0; k < 3; k++) navP[k] += navV[k] * dtL;

        // When the gyros saturate the airframe "loses track of which way is up": gravity leaks
        // into the horizontal channels and the estimate diverges hard.
        if (sat) {
          const leak = [gauss() * 0.11, gauss() * 0.11, -0.55 * (0.5 + rnd() * 0.5)];
          for (let k = 0; k < 3; k++) leakV[k] += leak[k] * dtL;
          for (let k = 0; k < 3; k++) leakP[k] += leakV[k] * dtL;
        }
        // Mild constant-scale divergence from accelerometer bias, always present.
        baroAlt = baroAlt + (((hFromPressure(s.pBaro) * FT) - baroAlt) / tauVent) * dtL;
      }

      // Flight-event registers.
      fer.liftoff = fer.liftoff || t > evAt('liftoff');
      fer.apogee = fer.apogee || t > evAt('apogee');
      fer.pressInc = fer.pressInc || s.v[2] < 0;
      fer.eciNeg = fer.eciNeg || navV[2] <= 0;
      fer.accelNeg = fer.accelNeg || s.v[2] <= 0;
      fer.tilt90 = fer.tilt90 || s.tilt > 90;
      // The ejection channels latch when the charge fires, which is what makes bits 3..6 usable as
      // event timestamps; only bit 1 (apogee / pressure increase) is an altimeter conclusion and so
      // arrives late.
      fer.apo = fer.apo || t > evAt('drogue');
      fer.fourth = fer.fourth || (t > evAt('drogue') && t < evAt('main'));
      fer.main = fer.main || t > evAt('main');
      fer.third = fer.third || fer.apogee;

      const rollRateHz = s.rollRate;
      roll = (roll + rollRateHz * dtL) % 360;

      low.push({
        t,
        sync: Math.round((t * 1000) % 250),
        baroTempF: 78 + 14 * rnd() - Math.min(26, (s.p[2] * FT) / 600) + (s.sf[0] > 40 ? 6 : 0),
        baroP: s.pBaro / 101325,
        batteryMv: 4020 - t * 1.1 - (s.sf[0] > 10 ? 130 : 0) + gauss() * 8,
        apoMv: t > evAt('drogue') && t < evAt('drogue') + 0.5 ? 3600 : 0,
        mainMv: t > evAt('main') && t < evAt('main') + 0.5 ? 3550 : 0,
        thirdMv: t > evAt('apogee') && t < evAt('apogee') + 0.35 ? 3580 : 0,
        fourthMv: t > evAt('drogue') + 1.5 && t < evAt('main') ? 3410 : 0,
        outputMa: (t > evAt('drogue') && t < evAt('drogue') + 0.5 ? 420 : 0) + 12 + rnd() * 6,
        velUp: navV[2] * FT + leakV[2],
        velDown: navV[0] * FT + leakV[0],
        velCross: navV[1] * FT + leakV[1],
        altNav: navP[2] * FT + leakP[2],
        posDown: navP[0] * FT + leakP[0],
        posCross: navP[1] * FT + leakP[1],
        altBaroAgl: baroAlt - cfg.pad.altFt + gauss() * 0.4,
        tilt: clamp(s.tilt + gauss() * 0.15, 0, 180),
        roll,
        tiltFuture: clamp(s.tilt + (s.v[2] > 0 ? (1 - Math.cos(Math.min(3, (s.p[2] * FT) / 400))) * 18 : 0) + gauss() * 0.3, 0, 180),
        fer: { ...fer },
      });
    }

    high.push({
      t, sync: Math.round((t * 1000) % 250),
      gx, gy, gz, ax, ay, az,
      quat: quatFromAxis(s.axis, spin),
      saturated: sat,
    });
  }
  return { low, high };
}

function quatFromAxis(axis, angleRad) {
  const n = unit(axis);
  const s = Math.sin(angleRad / 2);
  return { x: n[0] * s, y: n[1] * s, z: n[2] * s, w: Math.cos(angleRad / 2) };
}

// --- GPS emulation -----------------------------------------------------------
function gpsSensors(cfg, sim, padAltM) {
  const rnd = mulberry32(cfg.seed + 4242);
  const gauss = gaussFactory(rnd);
  const degraded = cfg.gpsQuality === 'degraded';
  const out = [];
  const period = 0.1; // 10 Hz
  // Slowly-varying satellite-geometry bias (a few metres, correlated over tens of seconds).
  let biasN = 0, biasE = 0, biasU = 0;
  let gapUntil = -1, fixHold = 3, sats = degraded ? 7 : 10;
  let acc = -Infinity;
  const logStart = cfg.gpsLogStartS ?? 0;

  for (const s of sim.rec) {
    const t = s.t;
    if (t < logStart || t < acc) continue;
    acc = t + period;

    biasN += gauss() * 0.02; biasE += gauss() * 0.02; biasU += gauss() * 0.03;
    biasN = clamp(biasN, -4, 4); biasE = clamp(biasE, -4, 4); biasU = clamp(biasU, -7, 7);

    const moving = Math.hypot(s.v[0], s.v[1], s.v[2]) > 2 && t > 2.5;
    // Dropouts: occasional on the pad, and (in the degraded case) hard in-flight gaps where the
    // receiver loses lock entirely.
    if (gapUntil < 0 && rnd() < (moving ? (degraded ? 0.0022 : 0.0002) : 0.003)) {
      gapUntil = t + (moving ? 0.7 + rnd() * 1.6 : 0.5 + rnd());
    }
    if (degraded && rnd() < 0.02 && t > 8 && fixHold !== 3) fixHold = 3;
    if (degraded && rnd() < 0.0016 && t > 3) fixHold = rnd() < 0.4 ? 2 : 0;
    if (fixHold === 0 && t > 25 && rnd() < 0.02) fixHold = 2;
    if (gapUntil > 0 && t > gapUntil) gapUntil = -1;
    if (gapUntil > 0) continue; // no fix at all this epoch

    // On the pad the receiver wanders; in flight it is velocity-aided and much tighter.
    const sigH = moving ? (degraded ? 2.2 : 1.3) : 2.4;
    const sigU = moving ? (degraded ? 7.5 : 4.0) : 9.0;
    const pe = s.p[0] + biasE + gauss() * sigH;
    const pn = s.p[1] + biasN + gauss() * sigH;
    const pu = s.p[2] + biasU + gauss() * sigU;

    const ve = s.v[0] + gauss() * (moving ? 0.55 : 0.12);
    const vn = s.v[1] + gauss() * (moving ? 0.55 : 0.12);
    const vu = s.v[2] + gauss() * (moving ? 1.1 : 0.2);

    const satsTotal = clamp(Math.round((degraded ? 6 : 9) + gauss() * 1.4), 3, 14);
    const fixType = fixHold === 0 ? 0 : degraded && rnd() < 0.07 ? 2 : 3;

    out.push({
      t,
      lat: cfg.pad.lat + (pn * FT) / 364000,
      lon: cfg.pad.lon + (pe * FT) / (364000 * Math.cos((cfg.pad.lat * Math.PI) / 180)),
      altFt: (padAltM + pu) * FT,
      hvel: Math.hypot(ve, vn) * FT,
      heading: ((Math.atan2(ve, vn) * 180) / Math.PI + 360) % 360,
      upvel: vu * FT,
      fixType,
      sats: satsTotal,
      sats24: clamp(satsTotal - (degraded ? 3 : 1), 0, 16),
      sats32: clamp(satsTotal - (degraded ? 6 : 4), 0, 16),
      sats40: clamp(satsTotal - (degraded ? 9 : 8), 0, 16),
    });
  }
  return out;
}

// --- encoding ----------------------------------------------------------------
const ferBits = (f) => {
  let m = 0;
  const map = [['liftoff', 0], ['apogee', 1], ['pressInc', 2], ['apo', 3], ['main', 4], ['third', 5], ['fourth', 6], ['eciNeg', 7], ['accelNeg', 8], ['tilt90', 9]];
  for (const [k, b] of map) if (f[k]) m |= 1 << b;
  return m.toString(16).toUpperCase().padStart(3, '0');
};

function writeCsv(path, header, rows, fmt) {
  const lines = [header.join(',')];
  for (const r of rows) lines.push(fmt(r).join(','));
  writeFileSync(path, lines.join('\n') + '\n');
}

const n = (x, d = 2) => (Number.isFinite(x) ? Number(x).toFixed(d) : '0');
const i0 = (x) => String(Math.round(x));

// pad ASL -> geodetic, and truth for the harness
function geodeticFromEnu(pad, e, nn, upFt) {
  const dNorth = nn / 364000;
  const dEast = e / (364000 * Math.cos((pad.lat * Math.PI) / 180));
  return { lat: pad.lat + dNorth, lon: pad.lon + dEast, altFt: pad.altFt + upFt };
}

let summary = [];
for (const cfg of FLIGHTS) {
  const sim = simulate(cfg);
  const padAltM = cfg.pad.altFt * 0.3048;
  const br = blueRavenSensors(cfg, sim);
  const gps = gpsSensors(cfg, sim, padAltM);

  const flightDate = new Date(Date.UTC(2026, 4, 16, 15, 42, 0));
  const iso = (t) => new Date(flightDate.getTime() + t * 1000).toISOString().replace('Z', '');

  writeCsv(
    join(OUT, `${cfg.id}_blue_raven_low.csv`),
    ['t_s', 'sync_code', 'baro_temp_f', 'baro_pressure_atm', 'battery_mv', 'apo_mv', 'main_mv', 'third_mv', 'fourth_mv', 'output_ma',
     'vel_up_fps', 'vel_downrange_fps', 'vel_crossrange_fps', 'alt_nav_ft', 'pos_downrange_ft', 'pos_crossrange_ft', 'alt_baro_agl_ft',
     'tilt_deg', 'roll_deg', 'tilt_future_deg', 'fer', 'fer_apo', 'fer_main', 'fer_third', 'fer_fourth'],
    br.low,
    (r) => [n(r.t, 3), r.sync, n(r.baroTempF, 1), n(r.baroP, 6), i0(r.batteryMv), i0(r.apoMv), i0(r.mainMv), i0(r.thirdMv), i0(r.fourthMv),
            i0(r.outputMa), n(r.velUp, 1), n(r.velDown, 1), n(r.velCross, 1), n(r.altNav, 1), n(r.posDown, 1), n(r.posCross, 1),
            n(r.altBaroAgl, 1), n(r.tilt, 1), n(r.roll, 1), n(r.tiltFuture, 1), ferBits(r.fer), ferBits(r.fer), ferBits(r.fer), ferBits(r.fer), ferBits(r.fer)],
  );

  writeCsv(
    join(OUT, `${cfg.id}_blue_raven_high.csv`),
    ['t_s', 'sync_code', 'gyro_x_dpps', 'gyro_y_dpps', 'gyro_z_dpps', 'accel_x_g', 'accel_y_g', 'accel_z_g', 'quat_x', 'quat_y', 'quat_z', 'quat_w'],
    br.high,
    // CSV carries engineering units (see FORMATS.md); only the native `@` dialect carries the
    // vendor's x100 / x30000 scalings.
    (r) => [n(r.t, 3), r.sync, n(r.gx, 2), n(r.gy, 2), n(r.gz, 2), n(r.ax, 3), n(r.ay, 3), n(r.az, 3),
            n(r.quat.x, 5), n(r.quat.y, 5), n(r.quat.z, 5), n(r.quat.w, 5)],
  );

  writeCsv(
    join(OUT, `${cfg.id}_gps.csv`),
    ['t_iso', 'gps_unit', 'lat_deg', 'lon_deg', 'alt_ft', 'hvel_fps', 'heading_deg', 'upvel_fps', 'fix_type', 'sats_total', 'sats_24', 'sats_32', 'sats_40'],
    gps,
    (r) => [iso(r.t), 'TRK', n(r.lat, 6), n(r.lon, 6), n(r.altFt, 1), n(r.hvel, 1), n(r.heading, 1), n(r.upvel, 1), r.fixType, r.sats, r.sats24, r.sats32, r.sats40],
  );

  writeFileSync(
    join(TRUTH, `${cfg.id}_truth.json`),
    JSON.stringify({
      id: cfg.id,
      pad: cfg.pad,
      apogeeFt: sim.apogeeAlt,
      apogeeT: sim.apogeeT,
      events: sim.events,
      duration: sim.rec[sim.rec.length - 1].t,
      brStartS: cfg.brLogStartS ?? 0,
      gpsStartS: cfg.gpsLogStartS ?? 0,
      // Enu in FEET, East/North/Up, relative to pad; plus truth velocity.
      samples: sim.rec.filter((_, k) => k % 10 === 0).map((s) => ({ t: +s.t.toFixed(3), p: s.p.map((x) => x * FT), v: s.v.map((x) => x * FT) })),
    }),
  );

  const landE = sim.rec[sim.rec.length - 1].p;
  const at = (k) => sim.events.find((e) => e.type === k);
  const velAt = (k) => {
    const e = at(k);
    if (!e) return NaN;
    const s = sim.rec.reduce((b, r) => (Math.abs(r.t - e.t) < Math.abs(b.t - e.t) ? r : b), sim.rec[0]);
    return Math.hypot(s.v[0], s.v[1], s.v[2]) * FT;
  };
  const maxV = sim.rec.reduce((a, r) => Math.max(a, norm(r.v)), 0);
  const satSec = br.high.filter((r) => r.saturated).length * 0.002;
  const gpsT = gps.map((g2) => g2.t);
  let maxGap = 0;
  for (let k = 1; k < gpsT.length; k++) maxGap = Math.max(maxGap, gpsT[k] - gpsT[k - 1]);
  const badFix = gps.filter((g2) => g2.fixType !== 3).length;
  summary.push({
    id: cfg.id,
    maxAltFt: Math.round(sim.apogeeAlt),
    maxVel: Math.round(maxV * FT),
    mach: +(maxV / 340.3).toFixed(2),
    maxG: Math.round(sim.maxMotorAccel),
    drogueVelFps: Math.round(velAt('main')),
    landingFps: Math.round((sim.impactVel ?? 0) * FT),
    dur: Math.round(sim.rec[sim.rec.length - 1].t),
    brRows: br.low.length,
    hrRows: br.high.length,
    gpsRows: gps.length,
    gyroSatS: +satSec.toFixed(1),
    gpsMaxGapS: +maxGap.toFixed(2),
    gpsBadFix: badFix,
    driftMi: +((Math.hypot(landE[0], landE[1]) * FT) / 5280).toFixed(2),
    landed: !!at('landing'),
  });
}

// A small index so the app can list the bundled flights.
writeFileSync(
  join(OUT, 'index.json'),
  JSON.stringify(
    summary.map((s, idx) => ({
      ...s,
      label: FLIGHTS[idx].name,
      low: `${s.id}_blue_raven_low.csv`,
      high: `${s.id}_blue_raven_high.csv`,
      gps: `${s.id}_gps.csv`,
    })),
    null,
    2,
  ),
);
console.table(summary);
