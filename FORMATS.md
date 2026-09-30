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
| `fer` `fer_apo` `fer_main` `fer_third` `fer_fourth` | flight-event registers, hex-encoded bitmasks | `FER:` |

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
| 3 | Apo channel fired (+1.5 s) |
| 4 | Main channel fired (+1.5 s) |
| 5 | 3rd channel fired (+1.5 s) |
| 6 | 4th channel fired (+1.5 s) |
| 7 | ECI vertical velocity ≤ 0 |
| 8 | Accel-only velocity ≤ 0 |
| 9 | Tilt exceeded 90° |

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

`parse → decimate → time-align → register → fuse → events`

1. **Time-align** — coarse clock offset from cross-correlation of `|vel_up|`; residual per-axis
   offset from registration.
2. **Register** — weighted orthogonal Procrustes (rotation + translation, Huber IRLS) fits the
   Blue-Raven inertial frame onto the GPS frame. Handles rail azimuth/tilt misalignment that a
   simple offset cannot.
3. **Fuse** — error-state Kalman filter + RTS smoother, solved independently per axis:
   state `[δp, δv, δb]`, Blue-Raven velocity as the control input (its *shape* at 50 Hz),
   GPS position and velocity as measurements. Process noise on `δb` is raised automatically
   wherever the manual says the inertial nav becomes untrustworthy — gyro rates near the ±2000 deg/s
   limit and tilt past 90° (tumbling after chute deployment), where gravity direction is lost and
   position error diverges. There the filter falls back to GPS.
4. **Events** — decoded from the FER bitmasks, cross-checked against baro/accel so a lost bit does
   not hide an ejection.
