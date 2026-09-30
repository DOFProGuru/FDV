/**
 * A small purpose-built line chart on canvas.
 *
 * Deliberately not a charting library: the traces here are 10k-100k samples of an unequal-spaced
 * series, several per panel, sharing one time cursor, and the panels redraw while the replay runs.
 * Two canvases (data, overlay), one min/max pair per pixel column, and a dirty-flag redraw, come to
 * about this many lines and need no dependency.
 */
import { C, MONO } from './palette.ts';

export interface ChartSeries {
  /** sample times, seconds; ascending */
  x: number[];
  /** values, aligned to x; NaN is a gap, the line is broken rather than interpolated across */
  y: number[];
  color: string;
  width?: number;
  dash?: number[];
  alpha?: number;
  axis?: 'left' | 'right';
  /** clipped here for the axis, still drawn to the top of the plot; for saturation spikes */
  cap?: number;
  /** name in the readout; the axis label is used when absent */
  label?: string;
}

export interface ChartMarker {
  t: number;
  color: string;
  label?: string;
}

export interface ChartOptions {
  yLabel: string;
  yDomain?: [number, number];
  /** right-hand axis, for a second quantity in the same panel */
  right?: { label: string; min: number; max: number };
  onSeek?: (t: number) => void;
}

interface Prepared {
  s: ChartSeries;
  t: Float64Array;
  lo: Float64Array;
  hi: Float64Array;
  has: Uint8Array;
}

const PAD = { l: 8, r: 10, t: 20, b: 6 };

/** Bucket a series into min/max pairs per pixel column, so a 500 kHz trace costs 900 line segments. */
function prepare(s: ChartSeries, xLo: number, xHi: number, buckets: number): Prepared {
  const n = Math.min(s.x.length, s.y.length);
  const t = new Float64Array(buckets);
  const lo = new Float64Array(buckets).fill(NaN);
  const hi = new Float64Array(buckets).fill(NaN);
  const has = new Uint8Array(buckets);
  const span = xHi - xLo || 1;
  for (let i = 0; i < n; i++) {
    const x = s.x[i];
    if (!(x >= xLo && x <= xHi)) continue;
    const y = s.y[i];
    if (!Number.isFinite(y)) continue;
    const b = Math.min(buckets - 1, Math.max(0, Math.floor(((x - xLo) / span) * buckets)));
    if (!has[b]) {
      has[b] = 1;
      t[b] = x;
      lo[b] = y;
      hi[b] = y;
    } else {
      if (y < lo[b]) lo[b] = y;
      if (y > hi[b]) hi[b] = y;
    }
  }
  return { s, t, lo, hi, has };
}

function nearest(xs: Float64Array, t: number): number {
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

function niceStep(x: number): number {
  if (!(x > 0) || !Number.isFinite(x)) return 1;
  const e = Math.pow(10, Math.floor(Math.log10(x)));
  const r = x / e;
  return (r >= 5 ? 5 : r >= 2 ? 2 : r >= 1 ? 1 : 0.5) * e;
}

export class Chart {
  private readonly host: HTMLElement;
  private readonly opts: ChartOptions;
  private readonly canvas: HTMLCanvasElement;
  private readonly overlay: HTMLCanvasElement;
  private readonly tip: HTMLDivElement;
  private readonly ro: ResizeObserver;
  private readonly g: CanvasRenderingContext2D;
  private readonly go: CanvasRenderingContext2D;
  private w = 10;
  private h = 10;
  private series: ChartSeries[] = [];
  private markers: ChartMarker[] = [];
  private xDomain: [number, number] = [0, 1];
  private yLo = 0;
  private yHi = 1;
  private prepared: Prepared[] = [];
  private hoverT: number | null = null;
  private cursorT: number | null = null;
  private dirtyData = true;
  private dirtyOverlay = true;

  constructor(host: HTMLElement, opts: ChartOptions) {
    this.host = host;
    this.opts = opts;
    this.canvas = document.createElement('canvas');
    this.overlay = document.createElement('canvas');
    this.tip = document.createElement('div');
    this.tip.className = 'chart-tip';
    host.append(this.canvas, this.overlay, this.tip);
    this.g = this.canvas.getContext('2d')!;
    this.go = this.overlay.getContext('2d')!;
    for (const ev of ['pointerenter', 'pointermove', 'pointerleave'] as const)
      this.overlay.addEventListener(ev, (e) => this.onPointer(e as PointerEvent));
    this.overlay.addEventListener('pointerdown', (e) => {
      const r = this.canvas.getBoundingClientRect();
      const i = nearest(this.xForHover(), this.tAt(e.clientX - r.left));
      if (i >= 0) this.opts.onSeek?.(this.xForHover()[i]);
    });
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(host);
    this.resize();
    this.loop();
  }

  setData(series: ChartSeries[], xDomain: [number, number], markers: ChartMarker[] = []): void {
    this.series = series.filter((s) => s.x.length > 1);
    this.prepared = [];
    this.xDomain = xDomain;
    this.markers = markers;
    this.yLo = Infinity;
    this.yHi = -Infinity;
    for (const s of this.series) {
      if (s.axis === 'right') continue;
      for (const y of s.y) {
        if (!Number.isFinite(y)) continue;
        if (y < this.yLo) this.yLo = y;
        if (y > this.yHi) this.yHi = y;
      }
    }
    if (!Number.isFinite(this.yLo)) {
      this.yLo = 0;
      this.yHi = 1;
    }
    for (const m of this.markers) {
      if (m.t < this.xDomain[0]) this.xDomain = [m.t, this.xDomain[1]];
      if (m.t > this.xDomain[1]) this.xDomain = [this.xDomain[0], m.t];
    }
    this.dirtyData = true;
    this.dirtyOverlay = true;
  }

  setCursor(t: number | null): void {
    this.cursorT = t;
    this.dirtyOverlay = true;
  }

  resize(): void {
    const r = this.host.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.w = Math.max(40, Math.floor(r.width));
    this.h = Math.max(40, Math.floor(r.height));
    for (const c of [this.canvas, this.overlay]) {
      c.width = Math.floor(this.w * dpr);
      c.height = Math.floor(this.h * dpr);
      c.style.width = `${this.w}px`;
      c.style.height = `${this.h}px`;
    }
    this.g.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.go.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.prepared = [];
    this.dirtyData = true;
    this.dirtyOverlay = true;
  }

  // --- drawing --------------------------------------------------------------
  private plot(): { x: number; y: number; w: number; h: number } {
    return { x: PAD.l, y: PAD.t, w: Math.max(10, this.w - PAD.l - PAD.r), h: Math.max(10, this.h - PAD.t - PAD.b) };
  }

  private X(t: number): number {
    const p = this.plot();
    const span = this.xDomain[1] - this.xDomain[0] || 1;
    return p.x + ((t - this.xDomain[0]) / span) * p.w;
  }

  private tAt(px: number): number {
    const p = this.plot();
    const span = this.xDomain[1] - this.xDomain[0] || 1;
    return this.xDomain[0] + Math.min(1, Math.max(0, (px - p.x) / p.w)) * span;
  }

  private Y(v: number, axis: 'left' | 'right' = 'left'): number {
    const p = this.plot();
    const [lo, hi] = axis === 'right' && this.opts.right ? [this.opts.right.min, this.opts.right.max] : [this.yLo, this.yHi];
    const f = (v - lo) / (hi - lo || 1);
    return p.y + p.h - Math.min(1.08, Math.max(-0.08, f)) * p.h;
  }

  private ensurePrepared(): void {
    const p = this.plot();
    if (this.prepared.length !== this.series.length) {
      this.prepared = this.series.map((s) => prepare(s, this.xDomain[0], this.xDomain[1], Math.max(4, Math.floor(p.w))));
    }
  }

  private drawData(): void {
    const g = this.g;
    const p = this.plot();
    g.clearRect(0, 0, this.w, this.h);
    this.ensurePrepared();

    const step = niceStep((this.yHi - this.yLo) / 4);
    const lo = Math.floor(this.yLo / step) * step;
    const hi = Math.ceil(this.yHi / step) * step;
    const tStep = niceStep((this.xDomain[1] - this.xDomain[0]) / Math.max(2, p.w / 78));
    const t0 = Math.ceil(this.xDomain[0] / tStep) * tStep;
    g.font = `500 9.5px ${MONO}`;
    g.textBaseline = 'middle';
    for (let v = lo; v <= hi + step * 1e-6; v += step) {
      const y = Math.round(this.Y(v)) + 0.5;
      if (y < p.y - 1 || y > p.y + p.h + 1) continue;
      g.strokeStyle = Math.abs(v) < step * 1e-6 ? C.line : C.grid;
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(p.x, y);
      g.lineTo(p.x + p.w, y);
      g.stroke();
      g.fillStyle = C.dim;
      g.textAlign = 'right';
      g.fillText(Math.abs(v) >= 10000 ? `${(v / 1000).toFixed(v % 1000 ? 1 : 0)}k` : v.toFixed(Math.abs(v) >= 10 ? 0 : 1), p.x + p.w - 4, y - 6);
    }
    g.textAlign = 'center';
    for (let t = t0; t <= this.xDomain[1] + 1e-9; t += tStep) {
      const x = Math.round(this.X(t)) + 0.5;
      g.strokeStyle = C.grid;
      g.beginPath();
      g.moveTo(x, p.y);
      g.lineTo(x, p.y + p.h);
      g.stroke();
      g.fillStyle = C.dim;
      g.fillText(String(Math.round(t * 10) / 10), x, p.y + p.h - 7);
    }
    if (this.opts.right) {
      g.textAlign = 'left';
      g.fillStyle = C.dim;
      for (const v of [this.opts.right.min, (this.opts.right.min + this.opts.right.max) / 2, this.opts.right.max])
        g.fillText(String(Math.round(v * 10) / 10), p.x + 4, this.Y(v, 'right'));
    }

    for (const m of this.markers) {
      if (m.t < this.xDomain[0] || m.t > this.xDomain[1]) continue;
      const x = Math.round(this.X(m.t)) + 0.5;
      g.strokeStyle = m.color;
      g.globalAlpha = 0.5;
      g.lineWidth = 1;
      g.setLineDash([2, 3]);
      g.beginPath();
      g.moveTo(x, p.y);
      g.lineTo(x, p.y + p.h);
      g.stroke();
      g.setLineDash([]);
      g.globalAlpha = 1;
    }

    for (const pr of this.prepared) {
      const s = pr.s;
      const clipCap = s.cap !== undefined;
      if (clipCap) {
        g.save();
        g.beginPath();
        g.rect(p.x, this.Y(s.cap!), p.w, p.y + p.h - this.Y(s.cap!));
        g.clip();
      }
      g.strokeStyle = s.color;
      g.lineWidth = s.width ?? 1.8;
      g.globalAlpha = s.alpha ?? 1;
      g.setLineDash(s.dash ?? []);
      g.lineJoin = 'round';
      g.beginPath();
      let open = false;
      for (let b = 0; b < pr.has.length; b++) {
        if (!pr.has[b]) {
          open = false;
          continue;
        }
        const x = this.X(pr.t[b]);
        const y0 = this.Y(pr.lo[b], s.axis);
        const y1 = this.Y(pr.hi[b], s.axis);
        if (!open) {
          g.moveTo(x, y1);
          open = true;
        } else g.lineTo(x, y1);
        if (y0 !== y1) {
          g.lineTo(x, y0);
          g.moveTo(x, y0);
        }
      }
      g.stroke();
      g.setLineDash([]);
      g.globalAlpha = 1;
      if (clipCap) g.restore();
    }

    // Saturation is worth seeing even when it destroys the scale, so it is drawn as ticks at the
    // top edge rather than being quietly clipped away.
    if (this.series.some((sp) => sp.cap !== undefined)) {
      g.fillStyle = C.bad;
      for (const sp of this.prepared) {
        if (sp.s.cap === undefined) continue;
        for (let b = 0; b < sp.has.length; b++) {
          if (!sp.has[b] || sp.hi[b] <= sp.s.cap) continue;
          g.fillRect(this.X(sp.t[b]) - 1, p.y - 4, 2, 3);
        }
      }
    }

    g.textAlign = 'right';
    g.textBaseline = 'top';
    g.fillStyle = C.muted;
    g.fillText(this.opts.yLabel, p.x + p.w - 4, 4);
    if (this.opts.right) {
      g.textAlign = 'left';
      g.fillStyle = C.dim;
      g.fillText(this.opts.right.label, p.x + 4, 4);
    }
  }

  private drawOverlay(): void {
    const g = this.go;
    const p = this.plot();
    g.clearRect(0, 0, this.w, this.h);
    if (this.cursorT === null && this.hoverT === null) return;

    const line = (t: number, color: string, dash: number[] | null, width: number): void => {
      if (!(t >= this.xDomain[0] && t <= this.xDomain[1])) return;
      const x = Math.round(this.X(t)) + 0.5;
      g.strokeStyle = color;
      g.globalAlpha = 0.9;
      g.lineWidth = width;
      g.setLineDash(dash ?? []);
      g.beginPath();
      g.moveTo(x, p.y);
      g.lineTo(x, p.y + p.h);
      g.stroke();
      g.setLineDash([]);
      g.globalAlpha = 1;
    };
    if (this.cursorT !== null) line(this.cursorT, C.fused, null, 1.5);
    if (this.hoverT !== null && this.hoverT !== this.cursorT) line(this.hoverT, C.muted, [3, 3], 1);

    // Dots on every trace at whichever time is being inspected, hover taking precedence.
    const at = this.hoverT ?? this.cursorT;
    if (at !== null) {
      for (const sp of this.prepared) {
        const i = nearest(sp.t, at);
        if (i < 0 || !sp.has[i]) continue;
        const y = this.Y(sp.hi[i], sp.s.axis);
        if (!Number.isFinite(y)) continue;
        g.fillStyle = sp.s.color;
        g.globalAlpha = 0.85;
        g.beginPath();
        g.arc(this.X(sp.t[i]), y, 2.6, 0, Math.PI * 2);
        g.fill();
        g.globalAlpha = 1;
      }
    }
  }

  private xForHover(): Float64Array {
    return this.prepared.length ? this.prepared.reduce((a, b) => (count(b) > count(a) ? b : a)).t : new Float64Array(0);
  }

  private tipHtml(t: number): string {
    const rows = this.prepared
      .map((sp) => {
        const i = nearest(sp.t, t);
        if (i < 0 || !sp.has[i]) return '';
        const v = sp.hi[i];
        const unit = sp.s.axis === 'right' ? (this.opts.right?.label ?? '') : this.opts.yLabel;
        return (
          `<div class="row"><span class="sw" style="background:${sp.s.color}"></span>` +
          `<span class="lb">${sp.s.label ?? unit}</span><span class="vl">${v.toFixed(Math.abs(v) >= 100 ? 0 : 1)}</span></div>`
        );
      })
      .filter(Boolean)
      .join('');
    const near = this.markers.find((m) => Math.abs(m.t - t) < (this.xDomain[1] - this.xDomain[0]) * 0.004);
    return `<div class="hd">T+${t.toFixed(2)} s${near ? ` &middot; <span style="color:${near.color}">${near.label ?? 'event'}</span>` : ''}</div>${rows}`;
  }

  private onPointer(e: PointerEvent): void {
    if (e.type === 'pointerleave') {
      this.hoverT = null;
      this.tip.classList.remove('show');
      this.dirtyOverlay = true;
      return;
    }
    const r = this.canvas.getBoundingClientRect();
    const px = e.clientX - r.left;
    const t = this.tAt(px);
    const xs = this.xForHover();
    const i = nearest(xs, t);
    this.hoverT = i >= 0 ? xs[i] : null;
    this.dirtyOverlay = true;
    if (this.hoverT === null) {
      this.tip.classList.remove('show');
      return;
    }
    this.tip.innerHTML = this.tipHtml(this.hoverT);
    this.tip.classList.add('show');
    const tw = this.tip.offsetWidth || 140;
    const flip = px > this.w * 0.6;
    this.tip.style.left = flip ? `${Math.max(4, px - tw - 10)}px` : `${Math.min(this.w - tw - 4, px + 10)}px`;
    this.tip.style.top = `${Math.max(4, Math.min(this.h - this.tip.offsetHeight - 4, e.clientY - r.top - 12))}px`;
  }

  // Data and overlay are dirtied separately: the cursor moves every frame of a replay, and the
  // thousand-segment traces underneath it do not.
  private loop = () => {
    requestAnimationFrame(this.loop);
    if (this.dirtyData) {
      this.dirtyData = false;
      this.drawData();
      this.dirtyOverlay = true;
    }
    if (this.dirtyOverlay) {
      this.dirtyOverlay = false;
      this.drawOverlay();
    }
  };

  destroy(): void {
    this.ro.disconnect();
    this.canvas.remove();
    this.overlay.remove();
    this.tip.remove();
  }
}

function count(p: Prepared): number {
  let n = 0;
  for (let i = 0; i < p.has.length; i++) n += p.has[i];
  return n;
}
