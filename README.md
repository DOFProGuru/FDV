# Flight reconstruction

Rocket flight logs in, one defensible trajectory out — with the uncertainty and the caveats shown
next to the result rather than filed away.

It reads [Blue Raven](FORMATS.md) flight-computer logs (low-rate and high-rate CSV) and a separate GPS
tracker log (CSV or NMEA), puts them on one clock and one frame, fuses them, and presents the
afterwards: key numbers, four profile charts, a 3-D replay with an attitude marker, an event list, a
list of everything that went wrong with the logs, and a panel that says how much of it to believe.

## Run it

```
npm install
npm run dev        # http://localhost:5173
```

Three bundled flights are in the picker — a nominal one, a two-stage one and one that tumbles. Drag
your own logs onto the window to read them instead: any Blue Raven low-rate CSV, an optional high-rate
CSV, and a GPS log. `npm run build && npm run preview` for the production bundle.

## What the reconstruction does

`parse → pad → time-align → register → noise → fuse → events`. The fuse is an error-state Kalman
filter with an RTS smoother, solved per axis: the Blue Raven's velocity gives the *shape* at 50 Hz and
the GPS gives absolute position and velocity. Where the two disagree, the filter is told which one to
believe by rules taken from the hardware manual — gyro rates near the ±2000 deg/s clipping limit, tilt
past 90° while still climbing, GPS rows with no fix or too few satellites.

The detail is in [FORMATS.md](FORMATS.md): every header alias accepted, every encoding rule, every
undocumented assumption, and what each check is for. It is the part worth reading.

## What is measured and what is not

The app distinguishes these wherever it can, and the diagnostics panel says which is which:

- **Clock offset** — cross-correlated, with the winning score, the runner-up and an independent
  anchor check displayed. Below about 55% it is a guess and says so.
- **Registration** — the yaw between the tracker's frame and the pad's, fitted to vertical turns, with
  its standard error and residuals.
- **GPS quality** — how many rows were rejected as measurements and why. A row with three satellites is
  not a position.
- **Attitude marker** — the manual does not say what vector the quaternion's imaginary part is. Both
  readings are tested against the independently reported tilt angle, per file, and the winner is used
  and named in the panel with its median disagreement. When neither fits, the marker is drawn from the
  tilt and the direction of travel and labelled nominal. Roll is shown with a parallel-transported
  datum, because there is no yaw reference to hang it on.
- **Track colour** — teal where the fixes and the accelerometers agree, amber where the inertial
  solution is running alone, blue where it is GPS-only because the airframe's own idea of up has failed.

Against the simulator's truth — which the app itself never sees — the bundled flights come out with
apogee within 0–2 ft and the track within 35–105 ft RMS, where the inertial solution alone is
2,400–7,600 ft RMS.

## Checks

| command | what it establishes |
| --- | --- |
| `npm test` | the numerics on their own: eigendecomposition, the Huber fits, clock alignment, filter and smoother, parser encodings, the quaternion-reading detector |
| `npm run verify` | the whole pipeline against `sample/truth/`, per flight |
| `npm run smoke` | the page in a real headless Chrome: WebGL up, panels filled, nothing thrown, no NaN in the DOM (needs `npm run build && npm run preview` first) |
| `npm run check` | types |

## Layout

```
src/lib/        parsers, geodesy, sync, the filter and smoother, events  (no DOM, no three.js)
src/ui/         charts, 3-D replay, attitude reading, formatting, loading
src/main.ts     the wiring
public/data/    three bundled flights and their manifest
sample/         the simulator that generated them, and the truth they are scored against
tools/          numerics tests, browser smoke test
```
