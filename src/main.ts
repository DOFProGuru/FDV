/**
 * The app. Files in, `reconstruct` in the middle, four charts, a 3-D replay, an event list and a
 * panel of evidence out.
 *
 * Everything shown comes from the reconstruction itself, including the claims about how much of it
 * to believe: the diagnostics panel reports the clock offset that was fitted, how many fixes survived
 * as measurements, what noise the filter was told to expect, and which reading of the attitude
 * quaternion was chosen and on what evidence.
 */
import './style.css';

import { reconstruct, speedOfSoundFps, type Reconstruction, type Vec3 } from './lib/fusion.ts';
import type { FlightData, FlightStats, FusedSample, GpsRow } from './lib/types.ts';
import { Chart, type ChartMarker, type ChartSeries } from './ui/chart.ts';
import { resolveAttitude, type AttitudeResolver, type QuatEpoch } from './ui/attitude.ts';
import { bundleFiles, fetchFlight, fetchManifest, readAll, type Bundle, type FlightEntry } from './ui/load.ts';
import { C } from './ui/palette.ts';
import { EVENT_COLOR, Replay3D } from './ui/replay.ts';
import { deg, descentSpeed, distance, fps, ft, num, offsetS, pct, tLabel } from './ui/format.ts';

const $ = <T extends HTMLElement>(id: string): T => document.getElementById(id) as T;
const el = <K extends keyof HTMLElementTagNameMap>(tag: K, cls?: string, text?: string): HTMLElementTagNameMap[K] => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};

const app = $('app');
const statusEl = $('status');
const flightSel = $('flight') as unknown as HTMLSelectElement;
const fileInput = $('files') as unknown as HTMLInputElement;
const kpiGrid = $('kpis');
const eventList = $('events') as unknown as HTMLOListElement;
const diagList = $('diag') as unknown as HTMLDListElement;
const warnList = $('warnings');
const readout = $('replay-readout');
const playBtn = $('play') as unknown as HTMLButtonElement;
const scrub = $('scrub') as unknown as HTMLInputElement;
const rateSel = $('rate') as unknown as HTMLSelectElement;
const camBtn = $('cam') as unknown as HTMLButtonElement;

let manifest: FlightEntry[] = [];
let rec: Reconstruction | null = null;
let attitude: AttitudeResolver | null = null;
let replay: Replay3D | null = null;
let charts: Record<'alt' | 'vel' | 'acc' | 'att', Chart> | null = null;

let t0 = 0;
let t1 = 1;
let cur = 0;
let playing = false;
let playbackRate = 10;
let lastFrame = 0;

// --- helpers -----------------------------------------------------------------
function setStatus(text: string, cls = ''): void {
  statusEl.textContent = text;
  statusEl.className = `status${cls ? ` ${cls}` : ''}`;
}

const twoFrames = (): Promise<void> => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(() => r())));

const speed = (v: Vec3): number => Math.hypot(v.e, v.n, v.u);

function nearest(xs: number[], t: number): number {
  const n = xs.length;
  if (!n) return -1;
  if (t <= xs[0]) return 0;
  if (t >= xs[n - 1]) return n - 1;
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (xs[m] <= t) lo = m; else hi = m;
  }
  return t - xs[lo] <= xs[hi] - t ? lo : hi;
}

function hzOf(rows: { t: number }[]): number {
  if (rows.length < 2) return 0;
  const dt = rows[rows.length - 1].t - rows[0].t;
  return dt > 0 ? Math.round((rows.length - 1) / dt) : 0;
}

/** Central difference over ~0.1 s: differencing 50 Hz samples directly drowns in its own noise. */
function accelSeries(t: number[], v: Vec3[]): number[] {
  const n = t.length;
  if (n < 2) return [];
  const dt = (t[n - 1] - t[0]) / (n - 1);
  const k = Math.max(1, Math.round(0.1 / dt));
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - k);
    const b = Math.min(n - 1, i + k);
    const d = t[b] - t[a];
    const va = v[a];
    const vb = v[b];
    out[i] = d > 0 ? Math.hypot(vb.e - va.e, vb.n - va.n, vb.u - va.u) / d : NaN;
  }
  return out.map((a) => a / 32.174);
}

/** Break the line where an angle wraps, so 359 to 1 does not draw as a sweep across the panel. */
function gapAtWraps(xs: number[]): number[] {
  const out = xs.slice();
  for (let i = 1; i < out.length; i++)
    if (Number.isFinite(xs[i]) && Number.isFinite(xs[i - 1]) && Math.abs(xs[i] - xs[i - 1]) > 180) out[i] = NaN;
  return out;
}

function statRow(label: string, value: string, hint = '', title = ''): HTMLElement {
  const s = el('div', `stat${hint ? ` ${hint}` : ''}`);
  if (title) s.title = title;
  s.append(el('span', 'k', label), el('b', undefined, value));
  return s;
}

function diagRow(label: string, value: string, cls = '', title = ''): HTMLElement {
  const a = el('dt', undefined, label);
  const b = el('dd', cls, value);
  if (title) {
    a.title = title;
    b.title = title;
  }
  const w = el('div');
  w.append(a, b);
  return w;
}

// --- panels ------------------------------------------------------------------
function renderKpis(st: FlightStats, samples: FusedSample[]): void {
  const land = samples[samples.length - 1];
  kpiGrid.replaceChildren(
    statRow('Apogee', `${ft(st.maxAltFt)} ft`, st.maxAltT > 0 && st.maxAltT < 600 ? '' : 'warn', `Peak altitude above the pad, reached at T+${st.maxAltT.toFixed(1)} s.`),
    statRow('Max velocity', `${ft(st.maxVelFps)} ft/s`, '', `Mach ${st.maxMach.toFixed(2)} at that moment, at the altitude where it happened.`),
    statRow('Max acceleration', `${st.maxAccelG.toFixed(1)} g`, st.maxAccelG > 14 ? 'warn' : '', 'Net acceleration of the reconstructed track, not what the accelerometer felt: gravity is not in it.'),
    statRow('Flight time', `${st.flightTimeS.toFixed(1)} s`, '', 'Liftoff to landing.'),
    statRow(
      'Drift',
      distance(st.driftFt),
      '',
      `Landing ${ft(land.e)} ft east and ${ft(land.n)} ft north of the pad, ${ft(st.driftFt)} away.`,
    ),
    statRow('Drogue', descentSpeed(st.drogueVelFps), st.drogueVelFps === undefined ? 'dim' : st.drogueVelFps > 750 ? 'warn' : '', 'Terminal descent speed before the main ejection.'),
    statRow('Landing', `${fps(st.landingVelFps)} ft/s`, st.landingVelFps > 25 ? 'warn' : '', 'Vertical speed in the second before touchdown.'),
  );
}

function chartSeries(d: FlightData): Record<'alt' | 'vel' | 'acc' | 'att', ChartSeries[]> {
  const r = rec!;
  const t = r.fused.t;
  const alt = r.fused.p.map((p) => p.u);
  const spd = r.fused.v.map(speed);
  const mach = spd.map((s, i) => s / speedOfSoundFps(alt[i]));
  const gpsT = r.gpsOnly.t;
  const hi = d.brHigh ?? [];
  const tiltFuture = d.brLow.map((l) => l.tiltFuture);

  const empty: ChartSeries = { x: [], y: [], color: C.muted };
  return {
    alt: [
      { x: r.brOnly.t, y: r.brOnly.p.map((p) => p.u), color: C.br, width: 1.4, dash: [4, 3], label: 'inertial only' },
      { x: gpsT, y: r.gpsOnly.p.map((p) => p.u), color: C.gps, width: 1.4, dash: [1, 3], label: 'GPS only' },
      { x: t, y: r.samples.map((s) => s.baroAgl), color: C.baro, width: 1.2, dash: [2, 3], label: 'barometric' },
      { x: t, y: alt, color: C.fused, width: 2, label: 'fused altitude' },
    ],
    vel: [
      { x: gpsT, y: r.gpsOnly.v.map(speed), color: C.gps, width: 1.4, dash: [1, 3], label: 'GPS speed' },
      { x: t, y: r.fused.v.map((v) => v.u), color: '#3f9d90', width: 1.4, dash: [5, 3], label: 'vertical speed' },
      { x: t, y: mach, color: C.dim, width: 1.2, dash: [2, 4], axis: 'right', label: 'Mach' },
      { x: t, y: spd, color: C.fused, width: 2, label: 'speed' },
    ],
    acc: [
      hi.length
        ? {
            x: hi.map((h) => h.t + (r.brSync ? r.brSync.offsetS : 0)),
            y: hi.map((h) => Math.hypot(h.accel[0], h.accel[1], h.accel[2])),
            color: C.imu,
            width: 1.2,
            dash: [3, 3],
            cap: 20,
            label: 'accelerometer',
          }
        : empty,
      { x: t, y: r.samples.map((s) => s.brConfidence), color: C.dim, width: 1.2, dash: [2, 4], axis: 'right', label: 'inertial trust' },
      { x: t, y: accelSeries(t, r.fused.v), color: C.fused, width: 2, label: 'net acceleration' },
    ],
    att: [
      { x: d.brLow.map((l) => l.t), y: tiltFuture, color: C.dim, width: 1.2, dash: [2, 4], label: 'predicted tilt' },
      { x: t, y: gapAtWraps(r.samples.map((s) => s.roll)), color: C.gps, width: 1.4, dash: [4, 3], axis: 'right', label: 'roll' },
      { x: t, y: r.samples.map((s) => s.tilt), color: C.fused, width: 2, label: 'tilt' },
    ],
  };
}

function ensureCharts(): Record<'alt' | 'vel' | 'acc' | 'att', Chart> {
  if (charts) return charts;
  const mk = (id: string, yLabel: string, yDomain?: [number, number], right?: { label: string; min: number; max: number }) =>
    new Chart($(id), { yLabel, yDomain, right, onSeek: (t) => seek(t, true) });
  charts = {
    alt: mk('chart-alt', 'ft AGL'),
    vel: mk('chart-vel', 'ft/s', undefined, { label: 'Mach', min: 0, max: 1.2 }),
    acc: mk('chart-acc', 'g', [0, 20], { label: 'inertial trust', min: 0, max: 1 }),
    att: mk('chart-att', 'tilt °', [0, 180], { label: 'roll °', min: 0, max: 360 }),
  };
  return charts;
}

function markers(): ChartMarker[] {
  return rec!.events
    .filter((e) => e.code !== 'channel' && e.code !== 'gps-loss')
    .map((e) => ({ t: e.t, color: EVENT_COLOR[e.code] ?? C.muted, label: e.code.toUpperCase() }));
}

/** Which solution is carrying each epoch: that is what the track is coloured by. */
function kindAt(gpsT: number[]): (i: number) => 0 | 1 | 2 {
  return (i) => {
    const s = rec!.samples[i];
    if (1 - s.brConfidence > 0.6) return 2; // the accelerometers are not to be believed: the fixes are carrying it
    const j = nearest(gpsT, s.t);
    if (j < 0 || Math.abs(gpsT[j] - s.t) > 1.5) return 1; // no fix nearby: coasting on the inertial solution
    return 0;
  };
}

function codeCell(code: string): HTMLElement {
  const c = el('span', 'code', code.toUpperCase());
  c.style.color = EVENT_COLOR[code as keyof typeof EVENT_COLOR] ?? C.muted;
  return c;
}

function renderEvents(): void {
  eventList.replaceChildren();
  for (const e of rec!.events) {
    const li = el('li');
    li.dataset.t = String(e.t);
    li.append(el('span', 't', tLabel(e.t)), codeCell(e.code), el('span', 'what', e.label));
    li.addEventListener('click', () => seek(e.t, true));
    eventList.append(li);
  }
}

function renderDiagnostics(b: Bundle, st: FlightStats): void {
  const r = rec!;
  const reg = r.registration;
  const gpsKept = r.gpsOnly.t.length;
  // The same three tests the fusion applies, so the counts below add up to the rows above.
  const hasFix = (g: GpsRow): boolean => !Number.isFinite(g.fixType) || g.fixType >= 1;
  const noFix = b.data.gps.filter((g) => !hasFix(g)).length;
  const weak = b.data.gps.filter(
    (g) => hasFix(g) && ((Number.isFinite(g.sats) && g.sats >= 1 && g.sats < 4) || (g.hdop ?? 0) >= 10),
  ).length;
  let gap = 0;
  for (let i = 1; i < r.gpsOnly.t.length; i++) gap = Math.max(gap, r.gpsOnly.t[i] - r.gpsOnly.t[i - 1]);
  const distrust = r.samples.filter((s) => 1 - s.brConfidence > 0.6).length / Math.max(1, r.samples.length);
  const att = attitude;
  const attText =
    !att || att.source === 'tilt-track'
      ? 'tilt + track (nominal)'
      : att.source === 'axis-is-pointing'
        ? `quaternion axis is the pointing vector (${deg(att.agreementDeg)} vs tilt)`
        : `quaternion body→ENU (${deg(att.agreementDeg)} vs tilt)`;

  diagList.replaceChildren(
    diagRow(
      'GPS clock offset',
      `${offsetS(r.clock.gpsOffsetS)} (${pct(r.clock.score)})`,
      r.clock.score > 0.8 ? 'ok' : r.clock.score > 0.55 ? 'warn' : 'bad',
      `Cross-correlation of the two altitude channels against a delayed version of itself. The best match won ${pct(r.clock.score)} confidence` +
        (Number.isFinite(r.clock.runnerUpScore) ? `, the runner-up ${pct(r.clock.runnerUpScore)}` : '') +
        (r.clock.anchorAgreementS !== null ? `, and the independent event anchor agrees to ${offsetS(r.clock.anchorAgreementS, 2)}` : '') +
        '. Below about 55% the offset is a guess and the fusion will be poor.',
    ),
    r.brSync
      ? diagRow(
          'Blue Raven logs joined',
          `${(r.brSync.offsetS * 1000).toFixed(1)} ms · ${r.brSync.residualMs.toFixed(2)} ms rms`,
          r.brSync.aliased || r.brSync.residualMs > 2 ? 'warn' : 'ok',
          r.brSync.aliased
            ? `The millisecond counter repeats every few seconds, so the join is only good to about that period; ${r.brSync.residualMs.toFixed(2)} ms rms over the samples where both logs were open.`
            : `Both logs carry the same millisecond counter; the residual is the scatter left after the offset.`,
        )
      : diagRow('Blue Raven logs joined', 'not joined', 'warn', 'The two Blue Raven logs share no overlapping counter value, so the high-rate log was left on its own clock.'),
    diagRow(
      'Frame registration',
      `${deg(reg.yawDeg)} ± ${deg(reg.yawSigmaDeg)} · ${ft(reg.rmsHorizFt)} ft / ${ft(reg.rmsVertFt)} ft`,
      reg.pairs === 0 || reg.inlierFraction < 0.35 ? 'bad' : reg.inlierFraction < 0.6 ? 'warn' : 'ok',
      `Rotation from the tracker's local tangent plane onto the launch point, fitted to ${reg.pairs} vertical turns by Huber-weighted least squares. ` +
        `${pct(reg.inlierFraction)} of them agreed within a few hundred feet; the residuals are horizontal and vertical.`,
    ),
    diagRow(
      'GPS noise assumed',
      `${ft(r.noise.sigmaPosFt)} ft · ${fps(r.noise.sigmaVelFps, 1)} ft/s${r.noise.estimated ? '' : ' (fallback)'}`,
      r.noise.estimated ? 'ok' : 'warn',
      'Estimated from the residuals of a straight-line fit to successive fixes, which is what the filter is then told to expect of them.',
    ),
    diagRow(
      'Inertial trust',
      `${pct(1 - distrust)} of the flight`,
      distrust > 0.25 ? 'bad' : distrust > 0.02 ? 'warn' : 'ok',
      'Fraction of epochs where the fitted attitude or the gyro health let the inertial solution run, rather than the fixes carrying the position.',
    ),
    diagRow(
      'Fixes used',
      `${num(gpsKept)} of ${num(b.data.gps.length)} kept · ${n0(noFix)} with no fix, ${n0(weak)} too weak`,
      gpsKept < b.data.gps.length * 0.5 || gap > 3 ? 'warn' : 'ok',
      'A row is a measurement only if it has a fix, at least four satellites and a dilution under 10. ' +
        `Longest hole in the used series: ${gap.toFixed(1)} s.`,
    ),
    diagRow(
      'Pad',
      `${b.data.pad.lat.toFixed(5)}, ${b.data.pad.lon.toFixed(5)} · ${ft(b.data.pad.altFt)} ft MSL`,
      'dim',
      'From the GPS rows logged before the vehicle moved, with the tracker\'s static offset against the launch point removed.',
    ),
    diagRow(
      'Attitude marker',
      attText,
      !att || att.source === 'tilt-track' ? 'warn' : 'ok',
      `The manual does not say what vector the quaternion's imaginary part is, so the reading that reproduces the independently reported tilt angle is used: ` +
        `${att ? pct(att.withinTol) : '0%'} of epochs agreed within 5 degrees. The 3-D marker is indicative, not a measurement.`,
    ),
    diagRow(
      'Logs',
      `${num(b.data.brLow.length)} @${hzOf(b.data.brLow)} Hz · ${num(b.data.brHigh?.length ?? 0)} @${hzOf(b.data.brHigh ?? [])} Hz · ${num(b.data.gps.length)} @${hzOf(b.data.gps)} Hz`,
      'dim',
      [`Blue Raven low-rate: ${b.lowName}`, b.highName ? `Blue Raven high-rate: ${b.highName}` : 'no high-rate log', `GPS: ${b.gpsName}`].join(' · '),
    ),
    diagRow('Ascent and descent', `apogee at ${tLabel(st.maxAltT)} · drogue ${descentSpeed(st.drogueVelFps)}`, 'dim', `Total flight time ${st.flightTimeS.toFixed(1)} s.`),
  );
}

const n0 = (v: number): string => num(v);

function renderWarnings(): void {
  const w = rec!.warnings;
  warnList.replaceChildren(
    ...(w.length
      ? w.map((x) => {
          const li = el('li', 'warn', x);
          li.title = 'The track is drawn regardless; read this before using it.';
          return li;
        })
      : [el('li', 'ok', 'Nothing anomalous in these logs.')]),
  );
}

// --- transport ---------------------------------------------------------------
function seek(t: number, pause = false): void {
  if (!rec) return;
  if (pause && playing) setPlaying(false);
  cur = Math.min(t1, Math.max(t0, t));
  replay?.setTime(cur);
  charts?.alt.setCursor(cur);
  charts?.vel.setCursor(cur);
  charts?.acc.setCursor(cur);
  charts?.att.setCursor(cur);
  scrub.value = String(cur);
  const i = nearest(rec.fused.t, cur);
  const s = rec.samples[i];
  if (s) {
    const v = rec.fused.v[i];
    readout.textContent =
      `${tLabel(cur)} · ${ft(s.u)} ft AGL · ${ft(speed(v))} ft/s · Mach ${(speed(v) / speedOfSoundFps(s.u)).toFixed(2)} · ` +
      `${deg(s.tilt)} tilt · ${pct(s.brConfidence)} inertial trust`;
  }
  let active: HTMLElement | null = null;
  for (const li of Array.from(eventList.children) as HTMLElement[]) {
    if (Number(li.dataset.t) <= cur + 1e-9) active = li;
    else li.classList.remove('active');
  }
  active?.classList.add('active');
}

function setPlaying(on: boolean): void {
  if (!rec) return;
  playing = on && t1 > t0;
  playBtn.textContent = playing ? '❚❚' : '▶';
  playBtn.setAttribute('aria-pressed', String(playing));
  if (!playing) return;
  if (cur >= t1 - 1e-6) seek(t0);
  lastFrame = performance.now();
  requestAnimationFrame(tick);
}

function tick(now: number): void {
  if (!playing) return;
  const dt = Math.min(0.25, (now - lastFrame) / 1000);
  lastFrame = now;
  const nt = cur + dt * playbackRate;
  if (nt >= t1) {
    seek(t1);
    setPlaying(false);
    return;
  }
  seek(nt);
  requestAnimationFrame(tick);
}

// --- loading -----------------------------------------------------------------
function quaternions(samples: FusedSample[]): QuatEpoch[] | undefined {
  const out: QuatEpoch[] = [];
  for (const s of samples) if (s.q) out.push({ t: s.t, quat: s.q });
  return out.length ? out : undefined;
}

async function show(b: Bundle, title: string): Promise<void> {
  setStatus(`reconstructing ${title}…`, 'work');
  await twoFrames();
  const started = performance.now();
  let r: Reconstruction;
  try {
    r = reconstruct(b.data);
  } catch (err) {
    rec = null;
    setStatus(`${title}: could not be reconstructed — ${(err as Error).message}`, 'error');
    return;
  }
  const ms = performance.now() - started;
  rec = r;
  attitude = resolveAttitude(quaternions(r.samples), b.data.brLow);
  t0 = r.fused.t[0] ?? 0;
  t1 = r.fused.t[r.fused.t.length - 1] ?? 1;
  scrub.min = String(t0);
  scrub.max = String(t1);
  scrub.step = '0.02';
  scrub.disabled = false;
  playBtn.disabled = false;

  const cs = chartSeries(b.data);
  const m = markers();
  ensureCharts();
  for (const k of ['alt', 'vel', 'acc', 'att'] as const) charts![k].setData(cs[k], [t0, t1], m);

  renderKpis(r.stats, r.samples);
  renderEvents();
  renderDiagnostics(b, r.stats);
  renderWarnings();
  ensureReplay()?.setFlight({ samples: r.samples, kinds: kindAt(r.gpsOnly.t), events: r.events, attitude, showBaro: true });

  app.hidden = false;
  setStatus(
    `${title} — ${num(b.data.brLow.length)} Blue Raven rows at ${hzOf(b.data.brLow)} Hz, ` +
      `${num(b.data.brHigh?.length ?? 0)} high-rate rows, ${num(b.data.gps.length)} GPS rows · ` +
      `${num(r.samples.length)} fused epochs, ${num(r.gpsOnly.t.length)} fixes kept as measurements · ${ms.toFixed(0)} ms`,
  );
  seek(t0);
  try {
    history.replaceState(null, '', `#${encodeURIComponent(title)}`);
  } catch {
    /* a file:// origin refuses to rewrite the URL; the deep link is a nicety */
  }
}

function ensureReplay(): Replay3D | null {
  if (replay) return replay;
  try {
    replay = new Replay3D($('replay'));
    replay.setFollow(true);
  } catch (err) {
    readout.textContent = `3-D view unavailable: ${(err as Error).message}`;
    return null;
  }
  return replay;
}

async function loadSample(e: FlightEntry): Promise<void> {
  setStatus(`loading ${e.id}…`, 'work');
  try {
    await show(bundleFiles(await fetchFlight(e)), e.id);
  } catch (err) {
    setStatus(`${e.id}: ${(err as Error).message}`, 'error');
  }
}

async function loadFiles(list: FileList | File[]): Promise<void> {
  setStatus('reading files…', 'work');
  try {
    const files = await readAll(list);
    await show(bundleFiles(files), files.map((f) => f.name).join(' + '));
  } catch (err) {
    setStatus((err as Error).message, 'error');
  }
}

function setFollow(on: boolean): void {
  replay?.setFollow(on);
  camBtn.textContent = on ? 'follow' : 'overview';
  camBtn.setAttribute('aria-pressed', String(on));
}

// --- wiring ------------------------------------------------------------------
function wire(): void {
  flightSel.addEventListener('change', () => {
    const e = manifest.find((m) => m.id === flightSel.value);
    if (e) void loadSample(e);
  });
  fileInput.addEventListener('change', () => {
    if (fileInput.files?.length) void loadFiles(fileInput.files);
  });
  playBtn.addEventListener('click', () => setPlaying(!playing));
  scrub.addEventListener('input', () => seek(Number(scrub.value), true));
  rateSel.addEventListener('change', () => {
    playbackRate = Number(rateSel.value);
  });
  camBtn.addEventListener('click', () => setFollow(!(replay?.follow ?? true)));
  $('replay-overview').addEventListener('click', () => {
    replay?.overview();
    setFollow(false);
  });

  window.addEventListener('dragover', (ev) => {
    ev.preventDefault();
    document.body.classList.add('dragging');
  });
  window.addEventListener('dragleave', (ev) => {
    if (ev.relatedTarget === null) document.body.classList.remove('dragging');
  });
  window.addEventListener('drop', (ev) => {
    ev.preventDefault();
    document.body.classList.remove('dragging');
    if (ev.dataTransfer?.files?.length) void loadFiles(ev.dataTransfer.files);
  });

  window.addEventListener('keydown', (ev) => {
    if (!rec) return;
    const tag = (ev.target as HTMLElement | null)?.tagName;
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    if (ev.code === 'Space') {
      ev.preventDefault();
      setPlaying(!playing);
    } else if (ev.code === 'ArrowRight' || ev.code === 'ArrowLeft') {
      ev.preventDefault();
      seek(cur + (ev.shiftKey ? 0.1 : 1) * (ev.code === 'ArrowRight' ? 1 : -1), true);
    } else if (ev.key === 'f' || ev.key === 'F') {
      setFollow(!(replay?.follow ?? true));
    }
  });
}

async function boot(): Promise<void> {
  wire();
  playbackRate = Number(rateSel.value) || 10;
  setStatus('reading the bundled sample flights…', 'work');
  try {
    manifest = await fetchManifest();
  } catch (err) {
    manifest = [];
    setStatus(`${(err as Error).message} Drop a Blue Raven log and a GPS log to continue.`, 'error');
  }
  const options: HTMLOptionElement[] = [];
  if (manifest.length) {
    for (const e of manifest) {
      const o = el('option', undefined, `${e.id} — ${e.label} (${ft(e.maxAltFt)} ft)`);
      o.value = e.id;
      options.push(o);
    }
    flightSel.replaceChildren(...options);
    flightSel.disabled = false;
    const want = decodeURIComponent(location.hash.slice(1));
    const e = manifest.find((m) => m.id === want) ?? manifest[0];
    flightSel.value = e.id;
    await loadSample(e);
  } else {
    const o = el('option', undefined, 'no bundled flights');
    o.value = '';
    flightSel.replaceChildren(o);
  }
}

void boot();
