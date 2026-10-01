/**
 * The 3-D reconstruction view.
 *
 * Trajectory data is ENU feet with the pad at the origin; three.js is Y-up, so the render frame is
 * X = east, Y = up, Z = -north - a proper rotation, so attitudes carry across by rotating the
 * pointing vector the same way.
 *
 * Feet are rescaled per flight to put the whole trajectory in a ~1000-unit box: the depth buffer
 * gets some room, the orbit limits mean something, and a scene the size of a missile range does not
 * have to be rendered at missile-range scale.
 *
 * Everything that decides what the panel shows when a flight appears lives under "framing" below.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { FlightEvents, FusedSample } from '../lib/types.ts';
import type { AttitudeResolver } from './attitude.ts';
import { axesOf, distanceToFrame, frameTrack, panIntoFrame, viewDirection, type Tan } from './framing.ts';
import { C, KIND_COLOR } from './palette.ts';


export const EVENT_COLOR: Record<FlightEvents['code'], string> = {
  liftoff: C.good,
  burnout: C.br,
  apogee: C.fused,
  drogue: C.gps,
  main: C.gps,
  landing: C.good,
  channel: C.muted,
  'gps-loss': C.warn,
  'gyro-saturation': C.bad,
  'tilt-over-90': C.bad,
};

export interface FlightGeometry {
  samples: FusedSample[];
  /** which solution carried each epoch: 0 fused, 1 inertial-only, 2 GPS-only */
  kinds: (i: number) => 0 | 1 | 2;
  events: FlightEvents[];
  attitude: AttitudeResolver | null;
  /** draw the barometric altitude as a ghost of the fused track */
  showBaro: boolean;
}

const FT_STEPS = [100, 200, 500, 1000, 2000, 2640, 5280, 10560, 21120, 42240];

/** how high the default view sits above the pad plane, in the same ballpark as the view before */
const FIT_ELEVATION = (27 * Math.PI) / 180;
/** headroom left around the track when framing it. 1.25 is not cosmetic: it is what keeps the part
 *  of the track already flown on screen while follow slides the camera around it. */
const FIT_MARGIN = 1.25;
/** the airframe is followed by moving the camera no more than it takes to keep it inside this box,
 *  as a fraction of the half-panel */
const FOLLOW_BOX_X = 0.75;
const FOLLOW_BOX_Y = 0.68;

/** the view's half-widths in tangent units, aspect ratio and all */
function tanOfView(camera: THREE.PerspectiveCamera): Tan {
  const y = Math.tan((camera.fov * Math.PI) / 360);
  return { x: y * (camera.aspect || 1), y };
}

function textSprite(text: string, color: string, height: number): THREE.Sprite {
  const cv = document.createElement('canvas');
  const ctx = cv.getContext('2d')!;
  const font = `600 24px ui-monospace, Menlo, monospace`;
  ctx.font = font;
  const w = Math.ceil(ctx.measureText(text).width) + 16;
  cv.width = w;
  cv.height = 34;
  const c = cv.getContext('2d')!;
  c.font = font;
  c.textBaseline = 'middle';
  c.fillStyle = color;
  c.fillText(text, 8, 18);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false }));
  sp.scale.set(height * (w / 34), height, 1);
  return sp;
}

export class Replay3D {
  follow = true;

  private readonly host: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly ro: ResizeObserver;
  private flight = new THREE.Group();
  private raf = 0;
  private dirty = true;
  private scale = 1;
  private mark = 1;
  /** a framing that could not be applied yet: a panel in a hidden drawer has no size to fit against */
  private fitWanted = false;
  /** the pointer has taken the camera, so nothing moves it again except the pointer */
  private userOrbit = false;
  /** the clock time the present framing was composed for; see setTime */
  private framedAt = NaN;

  private times = new Float64Array(0);
  private pos = new Float32Array(0);
  private samples: FusedSample[] = [];
  private traveled?: THREE.Line;
  private rocket?: THREE.Group;
  private attitude: AttitudeResolver | null = null;
  private lastIdx = -1;
  /** parallel-transported body datum, so roll reads as roll rather than as the world's azimuth */
  private datumX: THREE.Vector3 | null = null;
  private datumT = NaN;

  constructor(host: HTMLElement) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    host.append(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(46, 1, 1, 20000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.09;
    this.controls.minDistance = 6;
    this.controls.maxDistance = 6000;
    this.controls.addEventListener('change', () => { this.dirty = true; });
    // OrbitControls reports 'start' for a drag and for the wheel, which is the line between the app
    // framing the flight and the user framing it.
    this.controls.addEventListener('start', () => { this.userOrbit = true; });

    this.scene.add(new THREE.HemisphereLight(0xc3c9ff, 0x12101a, 1.4));
    const key = new THREE.DirectionalLight(0xffffff, 2.4);
    key.position.set(1, 1.5, 0.7);
    this.scene.add(key);
    this.scene.add(this.flight);

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(host);
    this.resize();
    this.loop();
  }

  // --- build ------------------------------------------------------------------
  setFlight(g: FlightGeometry): void {
    this.clearFlight();
    const s = g.samples;
    this.samples = s;
    this.attitude = g.attitude;
    const n = s.length;
    if (!n) return;
    this.times = new Float64Array(n);
    for (let i = 0; i < n; i++) this.times[i] = s[i].t;

    let lo = new THREE.Vector3(Infinity, Infinity, Infinity);
    let hi = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    this.scale = 1;
    for (let pass = 0; pass < 2; pass++) {
      lo.set(Infinity, Infinity, Infinity);
      hi.set(-Infinity, -Infinity, -Infinity);
      this.pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const r = s[i];
        const x = r.e * this.scale;
        const y = Math.max(0, r.u) * this.scale;
        const z = -r.n * this.scale;
        this.pos.set([x, y, z], i * 3);
        lo.min(new THREE.Vector3(x, y, z));
        hi.max(new THREE.Vector3(x, y, z));
      }
      if (pass === 0) {
        const spanFt = Math.max(
          Math.max(lo.x, hi.x) - Math.min(lo.x, hi.x),
          Math.max(lo.z, hi.z) - Math.min(lo.z, hi.z),
          hi.y,
          300,
        );
        this.scale = 1000 / spanFt;
      }
    }
    this.mark = Math.max(4, (hi.y - lo.y) * 0.012);

    const posAttr = new THREE.BufferAttribute(this.pos, 3) as THREE.Float32BufferAttribute;
    const colAttr = new THREE.BufferAttribute(new Float32Array(n * 3), 3) as THREE.Float32BufferAttribute;
    const c = new THREE.Color();
    for (let i = 0; i < n; i++) {
      c.set(KIND_COLOR[g.kinds(i)]);
      c.toArray(colAttr.array as Float32Array, i * 3);
    }

    const full = new THREE.BufferGeometry();
    full.setAttribute('position', posAttr);
    full.setAttribute('color', colAttr);
    this.flight.add(new THREE.Line(full, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.3 })));

    // The flown part at full strength. Same buffers, its own draw range, so scrubbing costs one
    // integer: a 500 kHz path would otherwise be re-uploaded on every frame of the replay.
    const flown = new THREE.BufferGeometry();
    flown.setAttribute('position', posAttr);
    flown.setAttribute('color', colAttr);
    flown.setDrawRange(0, 0);
    this.traveled = new THREE.Line(flown, new THREE.LineBasicMaterial({ vertexColors: true }));
    this.flight.add(this.traveled);

    const shadow = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) shadow.set([this.pos[i * 3], 0.05, this.pos[i * 3 + 2]], i * 3);
    const sg = new THREE.BufferGeometry();
    sg.setAttribute('position', new THREE.BufferAttribute(shadow, 3));
    this.flight.add(new THREE.Line(sg, new THREE.LineBasicMaterial({ color: 0x353043, transparent: true, opacity: 0.5 })));

    if (g.showBaro) {
      // A ghost at the barometric height, so the fused altitude can be read against the altimeter's
      // own answer. Missing samples carry the last finite value rather than being drawn as a gap:
      // a NaN vertex takes the whole line with it on some drivers.
      const bp = new Float32Array(n * 3);
      let last: [number, number, number] | null = null;
      for (let i = 0; i < n; i++) {
        const r = s[i];
        const u = Number.isFinite(r.baroAgl) && r.baroAgl < 1e5 && r.baroAgl > -1e4 ? Math.max(0, r.baroAgl) : NaN;
        if (Number.isFinite(u)) last = [this.pos[i * 3], u * this.scale, this.pos[i * 3 + 2]];
        bp.set(last ?? [this.pos[i * 3], 0, this.pos[i * 3 + 2]], i * 3);
      }
      const bg = new THREE.BufferGeometry();
      bg.setAttribute('position', new THREE.BufferAttribute(bp, 3));
      this.flight.add(new THREE.Line(bg, new THREE.LineBasicMaterial({ color: C.baro, transparent: true, opacity: 0.5 })));
    }

    for (const e of g.events) {
      if (e.code === 'channel' || e.code === 'gps-loss') continue;
      const i = this.indexAt(e.t);
      const m = new THREE.Mesh(
        new THREE.SphereGeometry(this.mark * 0.28, 12, 9),
        new THREE.MeshBasicMaterial({ color: EVENT_COLOR[e.code] ?? C.muted }),
      );
      m.position.set(this.pos[i * 3], this.pos[i * 3 + 1], this.pos[i * 3 + 2]);
      this.flight.add(m);
    }

    const grid = this.addGround();
    this.addRocket();
    this.addLabel('PAD', C.br, new THREE.Vector3(0, this.mark * 3.2, 0));
    const land = g.events.find((e) => e.code === 'landing');
    if (land) {
      const i = this.indexAt(land.t);
      this.addLabel('LAND', C.good, new THREE.Vector3(this.pos[i * 3], this.mark * 1.6, this.pos[i * 3 + 2]));
    }
    this.addLabel('N', C.dim, new THREE.Vector3(0, this.mark, -grid * 0.9));
    this.addLabel('E', C.dim, new THREE.Vector3(grid * 0.9, this.mark, 0));

    this.fitTrack();
    this.lastIdx = -1;
    this.datumX = null;
    this.dirty = true;
  }

  /** the ground the track lies on: a grid of round feet over the width of the scene, the pad and
   *  its rail. Returns the half-extent, which is where the compass labels go. */
  private addGround(): number {
    // The scene is ~1000 units wide and ~1000/this.scale feet wide, so a cell of `step` feet is
    // step*scale units across. Dividing rather than multiplying here is what left the grid tens of
    // times wider than the flight, with its two visible lines nowhere near the track.
    const spanFt = 1000 / this.scale; // the scene is ~1000 units wide, this.scale is units per foot
    const step = FT_STEPS.find((s) => spanFt / s <= 12) ?? FT_STEPS[FT_STEPS.length - 1];
    const cell = step * this.scale;
    const divs = Math.max(6, Math.min(12, Math.ceil(spanFt / step)));
    const width = cell * divs;
    const fine = new THREE.GridHelper(width, divs, 0x3a334a, 0x241f2e);
    (fine.material as THREE.Material).transparent = true;
    (fine.material as THREE.Material).opacity = 0.8;
    this.flight.add(fine);
    const major = new THREE.GridHelper(width, Math.max(1, Math.round(divs / 5)), 0x4d4363, 0x4d4363);
    (major.material as THREE.Material).transparent = true;
    (major.material as THREE.Material).opacity = 0.45;
    major.position.y = -0.05;
    this.flight.add(major);

    const ring = new THREE.Mesh(
      new THREE.RingGeometry(this.mark * 0.5, this.mark * 0.95, 44),
      new THREE.MeshBasicMaterial({ color: C.br, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.06;
    this.flight.add(ring);
    const rail = new THREE.Line(
      new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0, this.mark * 2.4, 0]), 3)),
      new THREE.LineBasicMaterial({ color: C.br, transparent: true, opacity: 0.65 }),
    );
    this.flight.add(rail);
    return width / 2;
  }

  private addLabel(text: string, color: string, at: THREE.Vector3): void {
    const sp = textSprite(text, color, this.mark * 1.1);
    sp.position.copy(at);
    this.flight.add(sp);
  }

  private addRocket(): void {
    const L = this.mark * 3.4;
    const r = L * 0.11;
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CylinderGeometry(r, r, L * 0.62, 20), new THREE.MeshLambertMaterial({ color: 0xd7d2e2 }));
    body.position.y = L * 0.31;
    const nose = new THREE.Mesh(new THREE.ConeGeometry(r, L * 0.38, 20), new THREE.MeshLambertMaterial({ color: C.fused }));
    nose.position.y = L * 0.81;
    g.add(body, nose);
    // Fins and a stripe: without something off-axis the roll angle would be invisible.
    for (let k = 0; k < 3; k++) {
      const fin = new THREE.Mesh(new THREE.BoxGeometry(r * 0.22, L * 0.2, r * 1.6), new THREE.MeshLambertMaterial({ color: 0x9d94ad }));
      fin.position.y = L * 0.13;
      fin.rotation.y = (k * 2 * Math.PI) / 3;
      fin.translateZ(r * 1.05);
      g.add(fin);
    }
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(r * 2.1, L * 0.06, r * 0.3), new THREE.MeshLambertMaterial({ color: C.br }));
    stripe.position.y = L * 0.55;
    stripe.position.z = r * 0.95;
    g.add(stripe);
    const trail = new THREE.Line(
      new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0, -L * 1.6, 0]), 3)),
      new THREE.LineBasicMaterial({ color: C.fused, transparent: true, opacity: 0.3 }),
    );
    g.add(trail);
    this.rocket = g;
    this.flight.add(g);
  }

  // --- playback ---------------------------------------------------------------
  private indexAt(t: number): number {
    const ts = this.times;
    const n = ts.length;
    if (n <= 1) return 0;
    if (t <= ts[0]) return 0;
    if (t >= ts[n - 1]) return n - 1;
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (ts[mid] <= t) lo = mid; else hi = mid;
    }
    return lo;
  }

  /** Position and attitude at a time, interpolating between fused epochs. */
  setTime(t: number): void {
    const n = this.samples.length;
    if (!this.rocket || !n) return;
    const i = this.indexAt(t);
    const j = Math.min(n - 1, i + 1);
    const dt = this.times[j] - this.times[i];
    const a = dt > 1e-9 ? Math.min(1, Math.max(0, (t - this.times[i]) / dt)) : 0;
    const p = new THREE.Vector3(
      this.pos[i * 3] + (this.pos[j * 3] - this.pos[i * 3]) * a,
      this.pos[i * 3 + 1] + (this.pos[j * 3 + 1] - this.pos[i * 3 + 1]) * a,
      this.pos[i * 3 + 2] + (this.pos[j * 3 + 2] - this.pos[i * 3 + 2]) * a,
    );
    this.rocket.position.copy(p);
    this.rocket.quaternion.copy(this.orientation(i));
    this.traveled?.geometry.setDrawRange(0, i + 1);
    this.lastIdx = i;

    if (this.follow) {
      // The framing set with a flight is composed around the clock position the flight arrives at,
      // and the caller immediately seeks to that same position. Following starts when the clock
      // starts moving, so that first seek cannot turn a view of the whole flight into a view of the
      // launch pad.
      if (Number.isFinite(this.framedAt) && Math.abs(t - this.framedAt) > 1e-9) this.panToAirframe(p);
      else this.framedAt = t;
    }
    this.dirty = true;
  }

  /**
   * Body frame from the pointing direction and the roll angle. The roll datum is transported
   * forward from epoch to epoch rather than re-derived from the world each epoch, so a change in
   * heading does not look like roll; a scrub (a jump of more than a few epochs) re-seeds it.
   */
  private orientation(i: number): THREE.Quaternion {
    const s = this.samples[i];
    const A = new THREE.Vector3(0, 1, 0);
    const att = this.attitude?.at(s.t, { e: s.ve, n: s.vn });
    if (att) A.set(att.axis[0], att.axis[2], -att.axis[1]).normalize();
    if (!Number.isFinite(A.x) || A.lengthSq() < 1e-6) A.set(0, 1, 0);

    if (!this.datumX || Math.abs(i - this.lastIdx) > 4 || !Number.isFinite(this.datumT) || Math.abs(s.t - this.datumT) > 0.5) {
      const ref = Math.abs(A.x) > 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(1, 0, 0);
      this.datumX = ref.sub(A.clone().multiplyScalar(ref.dot(A))).normalize();
      this.datumT = s.t;
    }
    const X = this.datumX.clone().sub(A.clone().multiplyScalar(this.datumX.dot(A))).normalize();
    this.datumX = X.clone();
    this.datumT = s.t;
    if (att) X.applyAxisAngle(A, att.roll);
    const Z = new THREE.Vector3().crossVectors(X, A).normalize();
    const m = new THREE.Matrix4().makeBasis(X, A, Z);
    return new THREE.Quaternion().setFromRotationMatrix(m);
  }

  // --- framing ----------------------------------------------------------------
  /**
   * The default view of a flight: the whole track, from the side it was flown from, as large as the
   * panel allows. framing.ts has the geometry and the reasons for it; what is decided here is when it
   * happens - when a flight loads, when 'reset view' is pressed, and whenever the panel changes shape
   * while the user has not taken the camera over.
   */
  private fitTrack(): void {
    this.userOrbit = false;
    this.framedAt = NaN;
    this.fitWanted = true;
    this.applyFit();
  }

  /** A flight is loaded while the replay panel is still in a hidden drawer, where it has no size to
   *  fit an aspect ratio against, so the request stands until resize() reports one. */
  private applyFit(): void {
    if (!this.samples.length || this.host.clientWidth < 10 || this.host.clientHeight < 10) return;
    this.fitWanted = false;
    const view = frameTrack(
      this.pos,
      tanOfView(this.camera),
      viewDirection(this.pos, FIT_ELEVATION),
      FIT_MARGIN,
      this.controls.maxDistance,
    );
    this.controls.target.copy(view.target);
    this.camera.position.copy(view.position);
    this.camera.lookAt(view.target);
    this.controls.update();
    this.dirty = true;
  }

  /**
   * Slide the camera across the track - no turn, no zoom - just far enough to bring the airframe
   * back inside FOLLOW_BOX. Following it all the way to the middle of the frame, which is what this
   * replaced, is the wrong trade: an airframe a few tens of units tall held in the centre of a view
   * framed 1.25 times around a 1000-unit flight is a track pushed off the edges of a panel 300 px
   * high, and the track is what the panel is for. The box is loose enough to hold the whole flight
   * from launch to landing, so the camera only moves when it has to and the shape of the flight stays
   * in picture.
   */
  private panToAirframe(p: THREE.Vector3): void {
    const tan = tanOfView(this.camera);
    for (let pass = 0; pass < 2; pass++) {
      const dir = this.camera.position.clone().sub(this.controls.target);
      if (dir.lengthSq() < 1e-12) return;
      const { look, right, up } = axesOf(dir.normalize());
      this.camera.updateMatrixWorld();
      const s = p.clone().project(this.camera);
      const depth = p.clone().sub(this.camera.position).dot(look);
      const pan = panIntoFrame(s, depth, tan, { x: FOLLOW_BOX_X, y: FOLLOW_BOX_Y }, { right, up });
      if (!pan) return;
      this.controls.target.add(pan);
      this.camera.position.add(pan);
      this.camera.lookAt(this.controls.target);
    }
  }

  /** A panel that has become narrower or shorter can crop a track that fitted before it; stand
   *  further back along the same line of sight to keep it whole. */
  private pullBackToFrame(): void {
    if (!this.samples.length) return;
    const need = distanceToFrame(this.pos, tanOfView(this.camera), this.camera.position, this.controls.target);
    if (need <= 0) return;
    const dir = this.camera.position.clone().sub(this.controls.target).normalize();
    this.camera.position
      .copy(this.controls.target)
      .addScaledVector(dir, Math.min(need * FIT_MARGIN, this.controls.maxDistance));
    this.camera.lookAt(this.controls.target);
  }

  setFollow(on: boolean): void {
    this.follow = on;
    this.dirty = true;
  }

  /** frame the whole track, and stop following: 'reset view' gives the view a flight loads with. */
  overview(): void {
    if (!this.samples.length) return;
    this.follow = false;
    this.fitTrack();
  }

  // --- plumbing ---------------------------------------------------------------
  resize(): void {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (w < 10 || h < 10) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    // A flight is loaded while the panel is still hidden, so the framing it was given was computed
    // against no aspect ratio at all; apply it once there is one. Until the pointer takes over, keep
    // the track framed across whatever panel it ends up in.
    if (this.fitWanted) this.applyFit();
    else if (!this.userOrbit) this.pullBackToFrame();
    this.dirty = true;
  }

  private loop = () => {
    this.raf = requestAnimationFrame(this.loop);
    this.controls.update();
    if (!this.dirty) return;
    this.dirty = false;
    this.renderer.render(this.scene, this.camera);
  };

  private clearFlight(): void {
    for (const o of [...this.flight.children]) {
      this.flight.remove(o);
      o.traverse((d) => {
        const res = d as unknown as { geometry?: THREE.BufferGeometry; material?: THREE.Material | THREE.Material[] };
        res.geometry?.dispose();
        const m = res.material;
        if (Array.isArray(m)) m.forEach((x) => x.dispose());
        else m?.dispose();
      });
    }
    this.traveled = undefined;
    this.rocket = undefined;
    this.lastIdx = -1;
    this.datumX = null;
  }

  destroy(): void {
    cancelAnimationFrame(this.raf);
    this.ro.disconnect();
    this.controls.dispose();
    this.clearFlight();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
