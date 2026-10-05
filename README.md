# Flight reconstruction

Rocket flight logs in, one defensible trajectory out — with the uncertainty and the caveats shown next
to the result rather than filed away.

It reads [Blue Raven](FORMATS.md) flight-computer logs (the `@ LOG_LOW` / `@ LOG_HIR` telemetry the
device writes, or the CSV exported from it) and a separate GPS tracker log (CSV or NMEA), puts them on
one clock and one frame, fuses them, and presents the result: key numbers, four profile charts, a 3-D
replay with an attitude marker, an event list, everything that went wrong with the logs, and a panel
that says how much of it to believe.

## Run it

```
npm install
npm run dev        # http://localhost:5173
```

Three bundled flights are in the picker — a nominal one, a two-stage one and one that tumbles. Drag
your own logs onto the window to read them instead: any Blue Raven low-rate CSV, an optional high-rate
CSV, and a GPS log. `npm run build && npm run preview` for the production bundle.

## The hosted version

**<https://dofproguru.github.io/FDV/>** — the same app, no server behind it, nothing to install.

Pushing to `main` publishes it. [`.github/workflows/deploy.yml`](.github/workflows/deploy.yml) runs
`check`, `test` and `verify` first and refuses to publish if any of them fail, then builds and uploads
`dist/` as a Pages artifact. The browser smoke test runs against the built bundle too, and reports
without gating the deploy: the runner has no GPU, so its WebGL is the software renderer, which is a
fact about the machine rather than a verdict on the code.

`dist/` is built and never committed, so there is no second copy of the 30 MB of bundled CSVs sitting
in a `gh-pages` branch. The one thing that is not in the repository is the Pages *source* setting:
Settings → Pages → Source must read **GitHub Actions**, and if it is ever switched back to "Deploy from
a branch" the deploy fails with "Pages not enabled", which sounds like a permissions problem and is not
one.

The base is relative (`base: './'` in [`vite.config.ts`](vite.config.ts)), which is why the bundle can
live under `/FDV/` today and under a domain later without a rebuild that differs in any other way.
Moving to a custom domain is then a DNS record, the field in Settings → Pages, and a `CNAME` in
`public/` so the record survives the next deploy — not a change to the build. One consequence worth
knowing before you share it: choosing the two-stage or tumble flight in the picker pulls a ~10 MB
high-rate log, so the page is heavier than it looks.

## What the reconstruction does

`parse → pad → time-align → register → noise → fuse → events`. The fuse is an error-state Kalman filter
with an RTS smoother, solved per axis: the Blue Raven's velocity gives the *shape* at 50 Hz, the GPS
gives absolute position and velocity. Where the two disagree, rules taken from the hardware manual tell
the filter which one to believe — gyro rates near the ±2000 deg/s clipping limit, tilt past 90° while
still climbing, GPS rows with no fix or too few satellites. The seconds at the head of a log, when the
airframe is demonstrably not moving, count as a measurement of their own: the velocity is pinned to zero
there, which is the only place in a flight an accelerometer bias can be told apart from motion.

The detail is in [FORMATS.md](FORMATS.md) — every header alias accepted, every encoding rule, every
undocumented assumption, and what each check is for. It is the part worth reading.

## What is measured and what is not

The app distinguishes these wherever it can, and the diagnostics panel says which is which.

- **Clock offset** — cross-correlated, with the winning score, the runner-up and an independent anchor
  check displayed. Below about 55% it is a guess and says so.
- **Registration** — the yaw between the tracker's frame and the pad's, fitted to vertical turns, with
  its standard error and residuals.
- **GPS quality** — how many rows were rejected as measurements, and why. A row with three satellites is
  not a position. The panel separates the tracker's velocity noise, measured on the pad where the truth
  is zero, from the 9–140 ft/s the two solutions disagree by in flight; that difference belongs to the
  airframe, not the tracker, and using the smaller number as the in-flight sigma makes apogee worse.
- **Rest on the pad** — how long the log starts out stationary, since the bias is estimated against that
  stretch. When no such stretch exists the panel says so: the bias then has nothing to be measured
  against and rides through the ascent.
- **Attitude marker** — the manual does not say what vector the quaternion's imaginary part is. Both
  readings are tested per file against the independently reported tilt angle; the winner is used and
  named in the panel with its median disagreement. When neither fits, the marker is drawn from the tilt
  and the direction of travel and labelled nominal. Roll uses a parallel-transported datum, because
  there is no yaw reference to hang it on.
- **Track colour** — teal where the fixes and the accelerometers agree, amber where the inertial solution
  runs alone, blue where it is GPS-only because the airframe's own idea of up has failed.

Against the simulator's truth — which the app itself never sees — the bundled flights come out with
apogee within 0–2 ft and the track within 35–105 ft RMS, where the inertial solution alone is
2,400–7,600 ft RMS.

## Checks

| command | what it establishes |
| --- | --- |
| `npm test` | the numerics on their own: eigendecomposition, the Huber fits, clock alignment, filter and smoother, parser encodings, the quaternion-reading detector, the GPS measurement gate, saturation episodes, pad rest detection, logs with pieces missing |
| `npm run verify` | the whole pipeline against `sample/truth/`, per flight |
| `npm run smoke` | the page in a real headless Chrome: WebGL up, panels filled, nothing thrown, no NaN in the DOM (needs `npm run build && npm run preview` first) |
| `npm run check` | types, over the app and the tools alike |

## Layout

```
.github/        the Pages deploy
src/lib/        parsers, geodesy, sync, the filter and smoother, events  (no DOM, no three.js)
src/ui/         charts, 3-D replay, attitude reading, formatting, loading
src/main.ts     the wiring
public/data/    three bundled flights and their manifest
sample/         the simulator that generated them, and the truth they are scored against
tools/          numerics tests, browser smoke test
```

The repository is `FDV`, to sit with the others; the npm package inside it is lowercase `fdv`, because
npm enforces that at publish.

## Authorship

This project was drafted by the [pi](https://pi.dev) coding agent, running `qwen3.8-flash-next` over
Ollama, under Josef Spjut's direction. The requirements, the approach at each fork and the flight-domain
judgement are his; the drafting, the code and the debugging were the agent's.

## License

MIT-0 (MIT No Attribution), `Copyright 2026 Josef Spjut` — full text in [LICENSE](LICENSE). Use the
code, tools and documents for anything, credit nobody, no notice to carry along. What survives is the
warranty disclaimer, which in a tool that prints an apogee next to an uncertainty is not a footnote: the
output is a reconstruction of a log, and the judgement about whether to trust it stays with whoever is
standing at the pad. **Not for flight termination, range safety, or a go/no-go call** — nothing here is
qualified for that.

The synthetic flights under `public/data/` and their truth files are CC0 instead
([LICENSE-CC0](LICENSE-CC0)), dedicated to the public domain, since CC0 is the tool built for data.

Neither license grants a patent, neither reaches what `npm install` pulls in (`three` and the rest stay
under their own terms), and neither reaches the vendor's protocol: [FORMATS.md](FORMATS.md) is our own
writing about somebody else's format, so what is licensed is our sentences, not Featherweight's manuals
or the packets they describe.
