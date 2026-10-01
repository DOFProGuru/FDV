# Data formats

This app reconstructs a rocket trajectory from two Featherweight Altimeters data files.
Everything below is traceable to the vendor documentation; the parts that are **not** are
flagged and listed in [Assumptions](#assumptions).

Sources used:
- *Blue Raven Altimeter User's Manual* (rev. 2026‑may‑12) — §"Downloading flight data",
  "Low Rate Data", "High Rate Data", "Rocket events", `<summary>`.
- *Featherweight GPS Tracker Manual* (rev. 2025‑feb‑20) — Appendix A, `@GPS_STAT` packet table.

## Important context: the native format is *not* CSV

The devices emit line-oriented ASCII telemetry over USB/Bluetooth, captured with a terminal
emulator (RealTerm "Direct Capture"), and the phone app exports files that "can be opened in
Excel". So two input dialects are supported everywhere in this app:

| Dialect | Blue Raven | GPS tracker |
|---|---|---|
| **Native telemetry** (`@`-framed lines, as captured from USB) | `@ LOG_LOW …`, `@ LOG_HIR …` | `@GPS_STAT …` |
| **CSV** (spreadsheet-friendly export, one row per sample) | `blue_raven_low_*.csv`, `blue_raven_high_*.csv` | `gps_*.csv` |

The native reader is what makes this usable with a raw RealTerm capture; the CSV reader is the
requested primary path. Dialect is auto-detected from the first non-blank line.

## Blue Raven — low rate (50 Hz)

Native line, verbatim from the manual:

```
[sync code] Bo: [baro temp] [pressure atm x 50,000]
V: [Battery mV] [Apo mV] [Main mV] [3rd mV] [4th mV] [output current, mA]
Vel: [Upward velocity ft/sec] [Down-range ft/sec] [Cross-range ft/sec]
Pos: [Inertial Nav Altitude (ft)] [Down-range feet] [Cross-range feet]
ang: [tilt angle deg x10] [roll angle deg] [future tilt angle deg x10]
FER: [flight event register][Apo FER][Main FER][3rd FER][4th FER] CRC: [16b CRC]
```

CSV columns (engineering units; the manual's ×1000/×10 style scalings are decoded on ingest so
both dialects land in the same units):

| Column | Meaning | Native field |
|---|---|---|
| `t_s` | seconds since liftoff (from the record datestamp + `sync_code`) | derived |
| `sync_code` | ms counter shared with the high-rate log, rolls every 250 ms | `[sync code]` |
| `baro_temp_f` | baro sensor temperature, °F | `Bo:` field 1 |
| `baro_pressure_atm` | static pressure, atm | `Bo:` field 2 ÷ 50 000 |
| `battery_mv` | battery voltage | `V:` 1 |
| `apo_mv` `main_mv` `third_mv` `fourth_mv` | ejection charge voltages | `V:` 2–5 |
| `output_ma` | total output current | `V:` 6 |
| `vel_up_fps` `vel_downrange_fps` `vel_crossrange_fps` | inertial-nav velocity, **ground-relative inertial frame** (not rocket axes) | `Vel:` |
| `alt_nav_ft` | inertial-nav altitude | `Pos:` 1 |
| `pos_downrange_ft` `pos_crossrange_ft` | inertial-nav horizontal position | `Pos:` 2,3 |
| `alt_baro_agl_ft` | barometric altitude AGL | `AGL:` (status line) |
| `tilt_deg` | angle between rocket axis and vertical | `ang:` 1 ÷ 10 |
| `roll_deg` | integrated roll about the rocket axis | `ang:` 2 |
| `tilt_future_deg` | predicted tilt in +3 s (staging trigger) | `ang:` 3 ÷ 10 |
| `fer` `fer_apo` `fer_main` `fer_third` `fer_fourth` | flight-event registers, hex bitmasks (see below) | `FER:` |

## Blue Raven — high rate (500 Hz)

```
[sync code] [gyroX][gyroY][gyroZ] (deg/s x100) [accelX][accelY][accelZ] (G x100)
[quatX][quatY][quatZ][quatM] (x30,000)  CRC:
```

CSV: `t_s, sync_code, gyro_x_dpps, gyro_y_dpps, gyro_z_dpps, accel_x_g, accel_y_g, accel_z_g,
quat_x, quat_y, quat_z, quat_w`. Rotation **axis** first, magnitude term fourth, scaled by
30 000 to fit signed int16. The high-rate file is optional; it is used only for the rocket
attitude marker and the acceleration chart.

### Flight-event register bits

Bit meanings come from the "Rocket events" table (rocket-level) plus the per-channel table:

| Bit | Event |
|---|---|
| 0 | Liftoff detected |
| 1 | Apogee detected (2-of-3 vote: baro rising, total vel < 0, tilt > 90°) |
| 2 | Pressure increasing (descending) |
| 3 | Apo channel fired |
| 4 | Main channel fired |
| 5 | 3rd channel fired |
| 6 | 4th channel fired |
| 7 | ECI vertical velocity ≤ 0 |
| 8 | Accel-only velocity ≤ 0 |
| 9 | Tilt exceeded 90° |

Bits are sticky: once set they stay set for the rest of the log, so an event's time is the *first*
sample in which the bit reads set, and the resolution is one 20 ms low-rate frame. Bits 3-6 latch
when the corresponding charge fires, which is what makes them usable as deployment timestamps; bit
1 is the altimeter's own conclusion and can arrive a second or two after the airframe actually
reaches apogee, so it is a cross-check rather than a measurement.

Bits 7-9 need care. On the pad the airframe is level and stationary, but the accelerometers are
noisy enough that the internally-integrated velocity wanders through zero, which sets bits 7 and 8
before the rocket has moved - a pad `FER` of `180` is exactly this, and is normal. Because the bits
are sticky that pad latch can never be seen again, so the reconstruction derives "tilt exceeded 90°"
from the reported tilt angle rather than from bit 9, and does not use bits 7-9 at all as an
inertial-health signal.

## GPS tracker (10 Hz)

Native packet:

```
@ GPS_STAT 203 2020 11 15 01:20:21.986 CRC_OK TRK second
TrkAlt5655 lt39.55612 ln-105.1032 Vel0 -1550 Fix3 #9 42 0 0
000_00_00 000_00_00 000_00_00 000_00_00 000_00_00 CRC:6A1D
```

| Column | Unit | Native field |
|---|---|---|
| `t_iso` | UTC wall clock, ms precision | year/month/date + `HH:MM:SS.mmm` |
| `gps_unit` | `TRK` tracker / `GS` ground station / `FND` lost-rocket relay | parse type |
| `lat_deg` `lon_deg` | decimal degrees | `lt` / `ln` |
| `alt_ft` | altitude **ASL**, feet | `TrkAlt` |
| `hvel_fps` | horizontal velocity, ft/s | `Vel` 1 |
| `heading_deg` | horizontal heading, deg | — |
| `upvel_fps` | upward velocity, ft/s | `Vel` 2 |
| `fix_type` | 0 = no fix, 2 = 2-D, 3 = 3-D | `Fix` |
| `sats_total` `sats_24` `sats_32` `sats_40` | SV counts by dB strength gate | `#9 42 0 0` |
| `sats` | up to 5 `azimuth_elevation_strength` triplets | trailing block |

Altitude is ASL; the flight computer reports AGL. The pad elevation is recovered from the GPS
on-pad samples, which is one reason both files are needed.

## Encoding rules the parsers apply

These are the places where a file can be read two different ways, and what decides it.

**Bitmask columns are hexadecimal.** The registers are documented as hex bitmasks written without a
prefix, so `180` in a `fer` column is `0x180` (384), not 180. A token is read as hex when it carries
a prefix or suffix (`0x1F`, `1Fh`) or contains a letter (`3A7`); a token made only of digits is read
as hex too, because that is what the device writes. Write `180d` if you ever need decimal. In the
native telemetry the raw token is used rather than a numeric scan, which would otherwise read `3A7`
as `3`.

**High-rate CSV columns are engineering units** - gyro in deg/s, accel in g, quaternion components
dimensionless. Vendors' raw exports instead carry centi-unit integers (`2387` = 23.87 deg/s) and
30000-scaled quaternion terms. Which one a file is gets decided from the data, not from the filename:
if the median absolute gyro reading is more than ten times what an airframe plausibly does (40 deg/s)
the gyro columns are divided by 100, likewise the accel columns against 1.2 g. This survives a
spreadsheet re-export that stripped the column headers' meaning.

**The quaternion is a unit quaternion**, stored axis-first as `[axis * sin(theta/2), cos(theta/2)]`,
and it is renormalised on the way in, so a drifted or clipped estimate still points the right way.
Where only three quaternion-like terms are present they are read as a rotation vector
(`axis * theta/2`) and the fourth term is reconstructed. A raw export that multiplies the terms by
30000 is detected per row from the vector magnitude and divided back down.

## Assumptions

Things the manuals do **not** pin down, stated so they can be corrected against a real file:

1. **There is no published CSV column list.** The vendor documents the `@`-telemetry grammar and
   says the app "exports files you can open in Excel". The CSV headers above are a faithful
   1:1 projection of the documented telemetry fields, in the documented units and scalings. The
   parsers accept common aliases and unknown extra columns, so a real export will most likely
   parse as-is; add a line to `alias` maps in `src/lib/parsers/*` if it doesn't.
2. **Down-range/cross-range axis assignment.** The manual says these are ground-relative inertial
   axes but not their compass orientation, because it depends on the rail direction at launch.
   Default mapping is down-range→East, cross-range→North. The registration step (below) solves a
   full 3-D rigid transform, so a rail pointing anywhere but straight-up is still handled.
3. **"Sparrow" GPS.** Featherweight's trackers are the *Featherweight GPS Tracker* and the newer
   *Swift*; there is no "Sparrow" model. The GPS reader implements the documented `@GPS_STAT`
   schema plus generic NMEA-ish aliases (`lat`, `lon`, `alt`, `speed`, `course`, `hdop`), so
   third-party GPS logs also load.
4. The two devices have unsynchronised clocks; `sync_code` only aligns the two Blue Raven logs to
   *each other*. GPS↔Blue Raven alignment is estimated by cross-correlating the vertical-velocity
   profiles (±4 s window).

## Reconstruction pipeline

`parse → pad → time-align → register → noise → fuse → events`, all of it in `src/lib/fusion.ts`.

1. **Pad** — the launch point is not given in either log. It is taken from the GPS rows logged
   before the vehicle moved, with the tracker's own static offset against the launch point removed
   (`derivePad`). Everything downstream is stated relative to it, in East-North-Up.
2. **Time-align** — the two Blue Raven logs are joined on the shared millisecond counter
   (`sync.ts`), which is exact to a millisecond where the two overlap and is flagged `aliased` when
   the counter's own roll period leaves a second candidate offset. GPS against Blue Raven is a
   cross-correlation of the altitude traces over a ±10 s window, cross-checked against two physical
   anchors (burnout and apogee) that must agree; the winning score, the runner-up and the anchor
   disagreement are all reported in the diagnostics panel.
3. **Register** — the tracker's local tangent plane is put onto the pad's East-North-Up by a
   Huber-weighted yaw fit to the vertical turns in both tracks (`yawFitWeighted`), which is what a
   ground-relative inertial frame needs when the rail was not pointing north. Reported as the yaw,
   its standard error and the horizontal/vertical residuals.
4. **Noise** — what the filter is told to expect of the GPS, estimated from the residuals of a
   straight-line fit through successive fixes, rather than assumed. If too few fixes survive, a
   documented fallback is used and marked `(fallback)` in the panel.
   That velocity figure is an upper bound and knows it: on the sample logs it comes out at 9–140 ft/s,
   which is largely the inertial solution's own error being blamed on the tracker. The tracker's real
   velocity noise is measurable on the pad, where the answer is zero, and is 0.5–0.9 ft/s. Substituting
   the smaller number improves the velocity against truth (f17 4.7 → 2.9 ft/s RMS) and worsens the
   position, taking the apogee error from 1 ft to 10 ft and the tumbling flight's from 0 ft to 40 ft: a
   Doppler solution that is quiet while bolted to a rail is a different instrument under 35 g. The
   ascent keeps the pessimistic figure; the pad figure is reported in the panel as a property of the
   tracker, not as a sigma.
5. **Fuse** — error-state Kalman filter plus a Rauch-Tung-Striebel smoother, solved independently
   per axis: state `[δp, δv, δb]`, the Blue Raven velocity as the control input (its *shape* at
   50 Hz, which is where its value is), and GPS position and velocity as measurements.
   - The airframe sits motionless on the rail for the first seconds of a log, which is the only stretch
     where the velocity is known to be zero without asking either sensor, and therefore the only place
     an accelerometer *bias* can be told apart from motion — the one error that integrates twice, into
     velocity and then into position. The velocity is pinned to zero over that stretch (`padRestEnd`)
     and the bias state absorbs whatever the inertial solution accumulated there. The constraint stops
     at the first hint of movement less 0.25 s, not at the proof of it: Doppler confirms motion a few
     tenths of a second late, and holding a zero-velocity constraint one fix into a 35 g boost costs
     the whole ascent about 20 ft/s. It refuses to run at all unless the log starts stationary and the
     fixes stay within 60 ft of one another — a platform that wanders is a truck, not a pad. On the
     bundled flights the constraint holds to 0.2 s before liftoff; how long it can hold is bounded by
     where the *GPS* log starts, which on the tumbling flight is barely a second before the rail lets
     go. Post-landing stillness is not used, and should be: it would catch a barometric drift that
     nothing else can see.
   - A GPS row is a measurement only if it reports a fix, at least four satellites and a dilution
     below 10. A row the tracker itself does not believe in is worse than no row: it pulls the
     trajectory toward a position that may be miles out. Rows that fail are counted in the panel.
   - Process noise on `δb` is raised wherever the inertial solution is known to be bad: gyro rates
     near the ±2000 deg/s limit (the whole saturation window, widened a second either side) and
     tilt past 90° *while still climbing* — every airframe turns nose-down at apogee on purpose, so
     tilt alone says nothing once the altimeter is falling.
   - The vertical-rate and negative-acceleration register bits latch on first downward motion and
     stay latched, so they are never consulted as a health signal; only their edges mean anything.
6. **Events** — decoded from the FER bitmasks where the firmware can be trusted, and from the
   reconstructed trajectory where it cannot: apogee is the top of the fused track with the barometric
   register as a cross-check, burnout is the top of the speed trace below apogee, landings come from
   the altimeter with a sustained standstill as the fallback. A coast in the middle of the speed
   trace — speed rises, drag takes a few percent of it back off, then it rises higher than the first
   burn ever got — is reported as a staged motor with two burnouts, because a staged motor whose
   second stage is weaker than its first would otherwise show as a single one.

## Checking this

| command | what it establishes |
| --- | --- |
| `npm run check` | types, over `src` and `tools` alike — the check that turned up three debuggers still calling a parser signature from two revisions ago |
| `npm test` | the numerics in isolation: eigendecomposition, the Huber fits, clock alignment, the filter and smoother, the parser encoding rules, the GPS measurement gate, the quaternion-convention detector, the saturation episode merger, the pad rest detection, and the degraded input paths |
| `npm run verify -- f17-nominal` | the whole pipeline against the simulator's truth, which the app never reads: apogee, clock offset, position and velocity RMS against truth |
| `npm run smoke` | the page in a real headless Chrome over the DevTools protocol: WebGL came up, the panels filled in, nothing threw, no NaN reached the DOM |

`sample/verify.mjs` on the three bundled flights currently reconstructs apogee to within 0–2 ft,
the GPS clock offset to within 12 ms, and the track to 35–105 ft RMS against truth, where the
inertial solution on its own is 2,400–7,600 ft RMS. That gap is the argument for fusing at all.
