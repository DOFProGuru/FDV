/**
 * Where the 3-D view puts its camera.
 *
 * The replay panel is small - 408x300 css px on the desktop layout - and a flight path that fills a
 * third of it is a flight path you cannot see, so the framing is worth getting exactly right rather
 * than to within a factor of two. All of it is pure: a track's positions in scene units in, a
 * camera placement out, no renderer and no panel, which is also what makes it testable.
 */
import * as THREE from 'three';

const UP = new THREE.Vector3(0, 1, 0);

/** The view's half-widths in tangent units: y from the vertical fov, x = y * aspect. Keeping x and y
 *  apart is the whole point - a wide panel frames a wide track without standing back for height. */
export interface Tan {
  x: number;
  y: number;
}

export interface Placement {
  position: THREE.Vector3;
  target: THREE.Vector3;
}

/** The axes of a camera sitting on `dir` (track -> camera) and looking at the origin. */
export function axesOf(dir: THREE.Vector3): { look: THREE.Vector3; right: THREE.Vector3; up: THREE.Vector3 } {
  const look = dir.clone().negate();
  const right = new THREE.Vector3().crossVectors(look, UP);
  if (right.lengthSq() < 1e-12) right.set(0, 0, 1); // looking straight down a pole
  right.normalize();
  return { look, right, up: new THREE.Vector3().crossVectors(right, look).normalize() };
}

/** The bounding box of a track; non-finite epochs are ignored rather than believed. */
export function bboxOf(pos: Float32Array): { lo: THREE.Vector3; hi: THREE.Vector3 } {
  const lo = new THREE.Vector3(Infinity, Infinity, Infinity);
  const hi = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  const v = new THREE.Vector3();
  const n = pos.length / 3;
  for (let i = 0; i < n; i++) {
    v.set(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
    if (!Number.isFinite(v.x + v.y + v.z)) continue;
    lo.min(v);
    hi.max(v);
  }
  return { lo, hi };
}

/** The middle of the track's bounding box. */
export function trackMidpoint(pos: Float32Array): THREE.Vector3 {
  const { lo, hi } = bboxOf(pos);
  return lo.add(hi).multiplyScalar(0.5);
}

/**
 * How far back along `dir`, looking at `centre`, it takes for every epoch to clear the frustum.
 *
 * Each epoch constrains the distance linearly - it fits horizontally when the camera stands at
 * |across| / tanX minus its own depth - so the largest of those constraints is the exact answer,
 * not the estimate a scaled bounding-box diagonal gives. The horizontal term is usually the one
 * that wins on a wide panel, and the vertical term on a tall one.
 */
export function frameDistance(pos: Float32Array, dir: THREE.Vector3, centre: THREE.Vector3, tan: Tan): number {
  const { look, right, up } = axesOf(dir);
  const n = pos.length / 3;
  const v = new THREE.Vector3();
  let need = 1;
  for (let i = 0; i < n; i++) {
    v.set(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]).sub(centre);
    const across = v.dot(right), rise = v.dot(up), fore = v.dot(look);
    if (!Number.isFinite(across + rise + fore)) continue;
    need = Math.max(need, (Math.abs(across) - tan.x * fore) / tan.x, (Math.abs(rise) - tan.y * fore) / tan.y);
  }
  return need;
}

/**
 * The direction the default view looks from: broadside to the track's own dominant horizontal
 * direction, at `elevation` above the ground plane.
 *
 * A fixed azimuth is wrong for half of all flights - looked at from the south-east, a rocket that
 * went east is a stub along the view axis with no path left to see. So the azimuth comes from the
 * principal axis of the ground positions, used when the flight actually has a ground trace: long
 * compared with the flight itself (a quarter of it), and lying along something. A rocket that climbs
 * and comes back to the pad has a wind squiggle rather than a direction, its azimuth is worth
 * nothing visually, and taking the old south-east one keeps the view the same flight after flight
 * instead of following whichever way the wander happened to lean.
 *
 * Of the two broadside sides the camera always takes the one south of the pad - and the east one for
 * a flight that went due north or due south, which has no south side to look from - so north stays on
 * the far side of the picture and every flight reads left to right the same way round.
 */
export function viewDirection(pos: Float32Array, elevation: number): THREE.Vector3 {
  const n = pos.length / 3;
  let px = 0.57, pz = 0.82; // south-east, the azimuth the view had before it had an idea
  if (n > 0) {
    let mx = 0, mz = 0;
    for (let i = 0; i < n; i++) {
      mx += pos[i * 3];
      mz += pos[i * 3 + 2];
    }
    mx /= n;
    mz /= n;
    let sxx = 0, sxz = 0, szz = 0;
    for (let i = 0; i < n; i++) {
      const dx = pos[i * 3] - mx, dz = pos[i * 3 + 2] - mz;
      sxx += dx * dx;
      sxz += dx * dz;
      szz += dz * dz;
    }
    // The principal axis and the two spreads of a symmetric 2x2 covariance of the ground positions.
    const mean = 0.5 * (sxx + szz);
    const spread = Math.hypot(0.5 * (sxx - szz), sxz);
    const along = Math.sqrt(Math.max(0, mean + spread) / n); // the scale of the trace itself
    const elong = Math.sqrt((mean + spread) / Math.max(1e-9, mean - spread)); // and whether it is a line
    const { lo, hi } = bboxOf(pos);
    const size = Math.max(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
    if (elong > 1.8 && along > 0.25 * size) {
      const axis = 0.5 * Math.atan2(2 * sxz, sxx - szz);
      px = -Math.sin(axis);
      pz = Math.cos(axis);
    }
  }
  if (pz < 0 || (Math.abs(pz) < 0.15 && px < 0)) {
    px = -px;
    pz = -pz;
  }
  const h = Math.hypot(px, pz) || 1;
  return new THREE.Vector3((px / h) * Math.cos(elevation), Math.sin(elevation), (pz / h) * Math.cos(elevation));
}

/**
 * Sit the camera on `dir`, `margin` times the framing distance back, looking at the middle of the
 * track *as it lands on screen*.
 *
 * The bounding-box middle is only a starting guess: a camera looking down at 27 degrees does not put
 * the middle of a box in the middle of its picture, and the difference is a third of a panel. So
 * slide the camera across its own view until the projected box is centred - and one slide is not
 * enough, because a camera translation moves a close epoch further across the screen than a distant
 * one, in inverse proportion to their depth. Given which two epochs define an edge of the box, the
 * slide that centres that pair exactly is one offset weighted by the other's depth and vice versa;
 * recomputing which epochs define the edges on each pass settles it in two or three.
 */
export function frameTrack(
  pos: Float32Array,
  tan: Tan,
  dir: THREE.Vector3,
  margin: number,
  maxDistance = Infinity,
): Placement {
  const { look, right, up } = axesOf(dir);
  const n = pos.length / 3;
  const centre = trackMidpoint(pos);
  const v = new THREE.Vector3();
  let d = frameDistance(pos, dir, centre, tan);
  for (let pass = 0; pass < 4; pass++) {
    let xLo = Infinity, xHi = -Infinity, yLo = Infinity, yHi = -Infinity;
    let loAcross = 0, loDepthX = 0, hiAcross = 0, hiDepthX = 0;
    let loRise = 0, loDepthY = 0, hiRise = 0, hiDepthY = 0;
    for (let i = 0; i < n; i++) {
      v.set(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]).sub(centre);
      const across = v.dot(right), rise = v.dot(up), fore = v.dot(look);
      const depth = d + fore;
      if (!(depth > 1e-6) || !Number.isFinite(across + rise + fore)) continue;
      const sx = across / (tan.x * depth), sy = rise / (tan.y * depth);
      if (sx < xLo) { xLo = sx; loAcross = across; loDepthX = depth; }
      if (sx > xHi) { xHi = sx; hiAcross = across; hiDepthX = depth; }
      if (sy < yLo) { yLo = sy; loRise = rise; loDepthY = depth; }
      if (sy > yHi) { yHi = sy; hiRise = rise; hiDepthY = depth; }
    }
    if (!(xLo <= xHi) || !(yLo <= yHi)) break;
    centre.addScaledVector(right, (hiDepthX * loAcross + loDepthX * hiAcross) / (loDepthX + hiDepthX));
    centre.addScaledVector(up, (hiDepthY * loRise + loDepthY * hiRise) / (loDepthY + hiDepthY));
    d = frameDistance(pos, dir, centre, tan);
  }
  return {
    target: centre,
    position: centre.clone().addScaledVector(dir, Math.min(d * margin, maxDistance)),
  };
}

/**
 * What the distance from a camera to its look-at point would have to become for the whole track to
 * clear the frame, seen from where it is now. Used when the panel changes shape: the answer to a
 * cropped track is to stand further back along the same line of sight, never to move or zoom the
 * camera somewhere its owner did not put it.
 */
export function distanceToFrame(pos: Float32Array, tan: Tan, from: THREE.Vector3, target: THREE.Vector3): number {
  const dir = from.clone().sub(target);
  const span = dir.length();
  if (span < 1e-6) return 0;
  dir.normalize();
  const need = frameDistance(pos, dir, target, tan);
  return need > span * 1.001 ? need : 0;
}

/**
 * How far to slide a camera sideways - no turn, no zoom - to bring a point back inside a box centred
 * on the view, or null when it is already inside. `screen` is where the point lands (0,0 the middle,
 * +/-1 the edge of the panel), `depth` how far ahead of the camera it is along the line of sight, and
 * `box` the margin it is allowed within, in the same units.
 *
 * A pan of d across the view moves a point by d / (depth * tan) on screen, so the pan that lands it
 * exactly on the edge of the box is the overshoot multiplied back the other way - which is why a deep
 * pan costs so little on screen and a close one so much.
 */
export function panIntoFrame(
  screen: { x: number; y: number },
  depth: number,
  tan: Tan,
  box: { x: number; y: number },
  axes: { right: THREE.Vector3; up: THREE.Vector3 },
): THREE.Vector3 | null {
  const overX = Math.abs(screen.x) - box.x;
  const overY = Math.abs(screen.y) - box.y;
  if (overX <= 0 && overY <= 0) return null;
  const d = Math.max(1e-6, depth);
  return new THREE.Vector3()
    .addScaledVector(axes.right, Math.sign(screen.x) * Math.max(0, overX) * tan.x * d)
    .addScaledVector(axes.up, Math.sign(screen.y) * Math.max(0, overY) * tan.y * d);
}