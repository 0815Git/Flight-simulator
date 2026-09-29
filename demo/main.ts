/**
 * Israel Flight Simulator — a ring-course game built on the threetiles engine.
 *
 * A login screen collects the participant's full name and subject number, then
 * the simulator anchors the world over Kiryat Shmona and lays out a dense
 * course of rings running north toward Lake Qaraoun. Every ring
 * faces straight down the course (its opening squares up to the aircraft's
 * nose); the rings only "crab" left/right and up/down along the way, they
 * never rotate away. Rings are orange until flown through, then turn green.
 * A large ring scores 5 points, a small precision ring scores 10. The running
 * score is shown at the top; a fighter-style HUD (W datum, flight path marker,
 * pitch ladder) helps aim through each hoop.
 *
 * Keyboard: **A/D** roll, **Q/E** yaw, **W/S** pitch, **+/-** throttle,
 * **R** restart the course, **E** after a crash continues from the crash spot
 * (costs CRASH_PENALTY points). Touch: one-finger drag steers (right = bank right,
 * down = nose up), two-finger pinch is the throttle, tap resets after a crash.
 */
import * as THREE from 'three';
import { Terrain, ZOOM_LEVELS, type WorldConfig, worldFromLatLon } from '../src';
import { FM, SPEED_RULE, applyStick, speedRuleTrimThrottle, stepDynamics } from './flight';

/** Sky/fog color (raylib's SKYBLUE, for parity with the original demo). */
const SKY = new THREE.Color().setRGB(70 / 255, 130 / 255, 190 / 255, THREE.SRGBColorSpace);
/** User-space drift (meters) that triggers a large-world rebase. */
const REBASE_THRESHOLD = 4096;
/** Touch-drag distance (px) for full stick deflection. */
const STEER_RADIUS = 100;
/** Airspeed limits (m/s). Pilot controls throttle with +/- keys (or pinch). */
const SPEED_START = 160; // initial airspeed at spawn (m/s) — a front-side cruise

// Throttle is an engine COMMAND (0..1); the point-mass flight model in
// ./flight.ts turns it into thrust and evolves the airspeed from the forces.
// initial engine command (0..1): holds SPEED_START in level flight
const THROTTLE_START = SPEED_RULE.enabled ? speedRuleTrimThrottle() : 0.4;
const THROTTLE_RATE = 0.5;   // how fast the +/- keys move the throttle (per second)

// -- Gamepad mapping (Thrustmaster HOTAS Warthog) ----------------------------
// The Warthog is TWO separate USB devices, each with its own axes numbered from
// 0. We tell them apart by their id: the throttle's id contains GP_THROTTLE_ID,
// anything else is treated as the stick. Axis indices below are per-device, as
// read from Gamepad Tester. Stick = roll (X) + pitch (Y), mirroring touch/keys;
// the two linked throttle levers set speed (+1 = slowest, −1 = fastest). Flip an
// INVERT flag if a direction feels reversed. Yaw stays on Q/E (stick has 2 axes).
const GP_THROTTLE_ID = 'throttle';  // substring (lowercase) in the throttle's id
const GP_AXIS_ROLL = 0;    // stick device: left/right → roll  (like A/D)
const GP_AXIS_PITCH = 1;   // stick device: fwd/back   → pitch (like W/S)
const GP_AXIS_THR_A = 2;   // throttle device: lever A ┐ linked pair, averaged
const GP_AXIS_THR_B = 5;   // throttle device: lever B ┘ +1 → idle, −1 → full power
const GP_DEADZONE = 0.06;  // ignore tiny stick noise near centre
const GP_INVERT_ROLL = false;
const GP_INVERT_PITCH = false;

/** Where the aircraft spawns: over the south of the Kinneret, facing north. */
const START = { lat: 33.20796358450835, lon: 35.56987082847023, altitude: 600 };

/**
 * Course endpoints: the ring path runs from Kiryat Shmona up to Lake Qaraoun.
 * The straight line between them is the "centerline" (the 0 reference); every
 * ring in COURSE is placed as an offset from it. (PATH_END only sets the
 * direction; the rings extend as far as their cumulative gaps reach.)
 */
const PATH_START = { lat: 33.20796358450835, lon: 35.56987082847023, alt: 600 }; // Kiryat Shmona
const PATH_END = { lat: 33.571503158789284, lon: 35.696654819118166, alt: 1800 }; // Lake Qaraoun

/** Ring size → radius (m). Bigger = easier (+5); small/precision = +10. HAZ = red. */
type RingSize = 'XL' | 'L' | 'M' | 'S' | 'XS' | 'HAZ';
const SIZE_RADIUS: Record<RingSize, number> = { XL: 205, L: 140, M: 92, S: 56, XS: 34, HAZ: 115 };
const HAZARD_POINTS = -10; // red ring flown through

/**
 * THE COURSE — one row per ring, easy to hand-edit. Fields:
 *   gap  = distance (m) along the course from the PREVIOUS ring (gap 0 ≈ a ring
 *          stacked/beside the previous one at the same station — a pair).
 *   side = offset from the centerline: + = right, − = left (m).
 *   up   = offset from the centerline: + = above, − = below (m).
 *   size = XL | L | M | S | XS  (green) or HAZ (red).
 *   kind = 'green' (fly through, scores) or 'red' (avoid, −10 if entered).
 * Green points come from size automatically (XL/L/M = +5, S/XS = +10).
 * Reorder / edit / add / delete rows freely; the course updates directly.
 */
interface RingSpec { gap: number; side: number; up: number; size: RingSize; kind: 'green' | 'red' }
/** Distance (m) from the course start to the LAST station — it ends at Lake
 *  Qaraoun. The gaps below are stretched proportionally so the course always
 *  ends exactly here: deleting rows spreads the remaining rings out, adding
 *  rows packs them tighter. Rings sharing a station (gap 0) stay side by side. */
const COURSE_END = 39512;
/** Flight time (s, at SPEED_START) from the spawn point to the first station —
 *  a lead-in to settle before the rings begin. Places the first station, so
 *  the first row's gap below is ignored. */
const FIRST_RING_SECONDS = 15;
/** The spawn sits this far (m) behind the course start. */
const SPAWN_BACK = 250;
const COURSE: RingSpec[] = [
  { gap: 0, side: 38, up: 126, size: 'M', kind: 'green' },
  { gap: 0, side: -211, up: 186, size: 'HAZ', kind: 'red' },
  { gap: 3843, side: -198, up: -140, size: 'S', kind: 'green' },
  { gap: 0, side: 15, up: -110, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -411, up: -180, size: 'HAZ', kind: 'red' },
  { gap: 1815, side: 250, up: 170, size: 'L', kind: 'green' },
  { gap: 2993, side: -410, up: -230, size: 'XL', kind: 'green' },
  { gap: 4066, side: 200, up: 130, size: 'XS', kind: 'green' },
  { gap: 0, side: 391, up: 160, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 9, up: 90, size: 'HAZ', kind: 'red' },
  { gap: 2107, side: -360, up: -190, size: 'S', kind: 'green' },
  { gap: 0, side: -147, up: -160, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -573, up: -230, size: 'HAZ', kind: 'red' },
  { gap: 2174, side: 280, up: 240, size: 'L', kind: 'green' },
  { gap: 3888, side: -440, up: -110, size: 'S', kind: 'green' },
  { gap: 0, side: -227, up: -80, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -653, up: -150, size: 'HAZ', kind: 'red' },
  { gap: 2611, side: 190, up: 210, size: 'L', kind: 'green' },
  { gap: 1643, side: -330, up: -140, size: 'L', kind: 'green' },
  { gap: 3357, side: 250, up: 170, size: 'M', kind: 'green' },
  { gap: 0, side: 1, up: 230, size: 'HAZ', kind: 'red' },
  { gap: 4521, side: -410, up: -230, size: 'XL', kind: 'green' },
  { gap: 2620, side: 200, up: 130, size: 'L', kind: 'green' },
];
const COLOR_TARGET = 0x00e5ff; // bright cyan-blue — fly through these (stands out against the sky)
const COLOR_HAZARD = 0xe23a3a; // red — avoid these (drawn as squares)
const COLOR_DONE = 0x24507f; // dimmed blue — a target already flown through
const COLOR_MISS = 0xb23b3b; // a green target that was missed

/** Scoring. Missing a green ring costs nothing; red hazards use HAZARD_POINTS. */
const CRASH_PENALTY = -30; // crashed and continued with E

/**
 * Progressive reveal: only the nearest rings ahead are drawn solid; the rest
 * stay transparent and fade in as the aircraft advances — so the course can't
 * be scanned and pre-planned far ahead. `FULL` nearest are opaque; they fade to
 * invisible by the `VIS`-th ahead.
 */
const REVEAL_FULL = 3;
const REVEAL_VIS = 6;
/**
 * Ring opacity — 0 = invisible, 1 = fully solid. Set separately for the blue
 * circles and the red squares. "next" = the ring(s) you must fly through now
 * (the nearest station not yet passed); "later" = every ring after them.
 *   outline = the torus / square frame,  fill = the tinted inside.
 */
const OPACITY = {
  blue: {
    next:  { outline: 1,    fill: 0.75 },
    later: { outline: 0.35, fill: 0.05 },
  },
  red: {
    next:  { outline: 1,    fill: 0.61 },
    later: { outline: 0.35, fill: 0.05 },
  },
};

// -- DOM ---------------------------------------------------------------------

const canvas = document.getElementById('threetiles') as HTMLCanvasElement;
const loadingEl = document.getElementById('loading')!;
const crashEl = document.getElementById('crash')!;
const scorebarEl = document.getElementById('scorebar')!;
const scoreValEl = document.getElementById('scoreVal')!;
const ringsValEl = document.getElementById('ringsVal')!;
const spdTapeEl = document.getElementById('spdTape') as HTMLCanvasElement;
const altTapeEl = document.getElementById('altTape') as HTMLCanvasElement;
const compassEl = document.getElementById('compass') as HTMLCanvasElement;
const compassCtx = compassEl.getContext('2d')!;
const popEl = document.getElementById('pop')!;
const attitudeEl = document.getElementById('attitude') as HTMLCanvasElement;
const attitudeCtx = attitudeEl.getContext('2d')!;
const designatorEl = document.getElementById('designator') as HTMLCanvasElement;
const designatorCtx = designatorEl.getContext('2d')!;
const loginEl = document.getElementById('login')!;
const loginForm = document.getElementById('loginForm') as HTMLFormElement;
const startBtn = loginForm.querySelector('button[type="submit"]') as HTMLButtonElement;
const loginErr = document.getElementById('loginErr')!;
const fullNameEl = document.getElementById('fullName') as HTMLInputElement;
const subjectIdEl = document.getElementById('subjectId') as HTMLInputElement;

// -- renderer / scene --------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(SKY);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 1, 400_000);

const world = worldFromLatLon(START.lat, START.lon);
world.skirtOverlap = new Array(ZOOM_LEVELS).fill(1.01);
world.maxZoom = 17; // extra imagery/height detail over the course

const terrain = new Terrain(
  camera,
  {
    world,
    rendering: {
      fogColor: SKY,
            ambient: new THREE.Color().setRGB(150 / 255, 150 / 255, 150 / 255, THREE.SRGBColorSpace),
      sunScale: 0.35,
    },
    network: { concurrency: 8 },
  },
  scene,
);

/**
 * Absolute (rebase-independent) world position of a geographic coordinate,
 * in the same frame as the terrain's baked tile centers. Mercator tile math
 * mirrors {@link worldFromLatLon}; the anchor's `tileSize` is uniform in
 * tile-space, so this is accurate across the course.
 */
function worldPoint(w: WorldConfig, lat: number, lon: number, alt: number): THREE.Vector3 {
  const n = 2 ** w.baseZoom;
  const latRad = (lat * Math.PI) / 180;
  const x = ((lon + 180) / 360) * n;
  const y = ((1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2) * n;
  return new THREE.Vector3((x - w.anchorX) * w.tileSize, alt, (y - w.anchorZ) * w.tileSize);
}

// -- rings -------------------------------------------------------------------

interface Ring {
  center: THREE.Vector3; // ABSOLUTE position (independent of large-world rebase)
  s: number; // distance along the course (for progressive reveal ordering)
  radius: number;
  points: number;
  hazard: boolean;
  mesh: THREE.Mesh;
  halo: THREE.Mesh;
  done: boolean; // already flown through (scored or penalized) — ignore after
}

/** Group holds every ring; its position tracks the large-world offset. */
const ringGroup = new THREE.Group();
scene.add(ringGroup);

const rings: Ring[] = [];
/** Total number of green target rings (red hazards are excluded from the tally). */
let totalTargets = 0;
/** Constant course direction; every ring's opening faces along it (nose-on). */
const courseNormal = new THREE.Vector3(0, 0, -1);
/** Absolute centerline start of the course (spawn reference). */
const courseStart = new THREE.Vector3();

/** A flat square "annulus" (a square frame / square ring) in the XY plane, for
 *  the red hazards — same footprint as a circle of the given half-size. */
function squareAnnulus(outerHalf: number, innerHalf: number): THREE.ShapeGeometry {
  const shape = new THREE.Shape();
  shape.moveTo(-outerHalf, -outerHalf);
  shape.lineTo(outerHalf, -outerHalf);
  shape.lineTo(outerHalf, outerHalf);
  shape.lineTo(-outerHalf, outerHalf);
  shape.closePath();
  const hole = new THREE.Path();
  hole.moveTo(-innerHalf, -innerHalf);
  hole.lineTo(-innerHalf, innerHalf);
  hole.lineTo(innerHalf, innerHalf);
  hole.lineTo(innerHalf, -innerHalf);
  hole.closePath();
  shape.holes.push(hole);
  return new THREE.ShapeGeometry(shape);
}

function buildCourse(): void {
  const a = worldPoint(world, PATH_START.lat, PATH_START.lon, PATH_START.alt);
  const b = worldPoint(world, PATH_END.lat, PATH_END.lon, PATH_END.alt);

  // Every ring faces along the straight centerline (the opening squares up to
  // the aircraft's nose); the per-ring side/up offsets below only shift the
  // CENTER, never the facing — a "crab walk", not a turn.
  courseNormal.copy(b).sub(a).normalize();
  courseStart.copy(a);
  const up = new THREE.Vector3(0, 1, 0);
  const right = new THREE.Vector3(-courseNormal.z, 0, courseNormal.x).normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), courseNormal);
  // Squares need an explicit in-plane orientation (edges level with right/up),
  // which a circle didn't care about.
  const upInPlane = new THREE.Vector3().crossVectors(courseNormal, right).normalize();
  const quatSquare = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(right, upInPlane, courseNormal),
  );

  // Place each ring from COURSE: walk along the centerline by `gap`, then step
  // `side` (right/left) and `up` (above/below) off the line.
  // First station a fixed flight time from the spawn; the remaining gaps are
  // stretched so the last station lands exactly on COURSE_END.
  const firstAlong = FIRST_RING_SECONDS * SPEED_START - SPAWN_BACK;
  const restGaps = COURSE.slice(1).reduce((sum, spec) => sum + spec.gap, 0);
  const gapScale = (COURSE_END - firstAlong) / restGaps;
  let along = 0;
  for (const [i, spec] of COURSE.entries()) {
    along += i === 0 ? firstAlong : spec.gap * gapScale;
    const radius = SIZE_RADIUS[spec.size];
    const hazard = spec.kind === 'red';
    const points = hazard ? HAZARD_POINTS : radius >= 80 ? 5 : 10;
    const center = a
      .clone()
      .addScaledVector(courseNormal, along)
      .addScaledVector(right, spec.side)
      .addScaledVector(up, spec.up);

    const color = hazard ? COLOR_HAZARD : COLOR_TARGET;
    const tube = Math.max(6, radius * 0.07);
    // Blue targets = round (torus + ring fill); red hazards = square, same size.
    const outlineGeo = hazard
      ? squareAnnulus(radius + tube, radius - tube)
      : new THREE.TorusGeometry(radius, tube, 16, 44);
    const fillGeo = hazard
      ? squareAnnulus(radius * 0.94, radius * 0.18)
      : new THREE.RingGeometry(radius * 0.18, radius * 0.94, 44);
    const mesh = new THREE.Mesh(
      outlineGeo,
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, side: THREE.DoubleSide }),
    );
    mesh.quaternion.copy(hazard ? quatSquare : quat); // opening faces down the course
    mesh.position.copy(center);
    ringGroup.add(mesh);
    const halo = new THREE.Mesh(
      fillGeo,
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, side: THREE.DoubleSide }),
    );
    mesh.add(halo);
    rings.push({ center, s: along, radius, points, hazard, mesh, halo, done: false });
    if (!hazard) totalTargets++;
  }
}

function setRingColor(ring: Ring, color: number): void {
  (ring.mesh.material as THREE.MeshBasicMaterial).color.set(color);
  (ring.halo.material as THREE.MeshBasicMaterial).color.set(color);
}

/** Flew THROUGH a ring: score a green target or penalize a red hazard. */
function passRing(ring: Ring): void {
  ring.done = true;
  score += ring.points;
  if (ring.hazard) {
    setRingColor(ring, 0x7a2020); // red entered by mistake
    showPop(`${ring.points}`, '#ff6b6b'); // "-10"
  } else {
    setRingColor(ring, COLOR_DONE);
    targetsCleared++;
    showPop(`+${ring.points}`, HUD_GREEN);
  }
  updateScore();
}

/** Crossed a ring's plane outside it. No penalty: a missed green is just marked, a red is safely dodged. */
function skipRing(ring: Ring): void {
  ring.done = true;
  if (!ring.hazard) setRingColor(ring, COLOR_MISS);
}

let popTimer = 0;
function showPop(text: string, color: string): void {
  popEl.textContent = text;
  popEl.style.color = color;
  popEl.style.opacity = '1';
  popTimer = 0.9;
}

// -- game / flight state -----------------------------------------------------

let started = false;
let participant = { name: '', subject: '' };
let score = 0;
let targetsCleared = 0;

const flight = {
  speed: SPEED_START,          // |velocity| in m/s (for the HUD; set by the model)
  vel: new THREE.Vector3(),    // world velocity vector — the actual flight path
  throttle: THROTTLE_START,    // engine command 0..1 (drives thrust, not speed)
  angVel: new THREE.Vector3(), // rad/s: x=pitch, y=yaw, z=roll
  crashed: false,
};

function updateScore(): void {
  scoreValEl.textContent = String(score);
  ringsValEl.textContent = `${targetsCleared} / ${totalTargets}`;
}

function resetCamera(): void {
  // Spawn on the course centerline, just behind the start, aimed straight down
  // the course. The first ring sits a bit ahead (not dead on the start).
  const startAbs = courseStart.clone().addScaledVector(courseNormal, -SPAWN_BACK);
  camera.position.copy(startAbs).add(terrain.anchor.worldOffset);
  camera.up.set(0, 1, 0);
  camera.lookAt(courseStart.clone().addScaledVector(courseNormal, 1200).add(terrain.anchor.worldOffset));
  // Launch already moving straight down the nose at the spawn airspeed.
  camera.getWorldDirection(forward);
  flight.vel.copy(forward).multiplyScalar(SPEED_START);
  flight.speed = SPEED_START;
}

function resize(): void {
  const w = window.innerWidth;
  const h = window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  sizeCompass();
  sizeDesignator();
  sizeAttitude();
  sizeTape(spdTape);
  sizeTape(altTape);
}
window.addEventListener('resize', resize);

function crash(): void {
  flight.crashed = true;
  crashEl.style.display = 'block';
}

/**
 * Restart the whole run from the start of the course: reset the aircraft,
 * the score, and every ring. Bound to R (any time) and to a tap after a crash.
 * It never returns to the login screen — the participant stays signed in.
 */
function resetRun(): void {
  flight.crashed = false;
  flight.angVel.set(0, 0, 0);
  flight.speed = SPEED_START;
  flight.throttle = THROTTLE_START;
  score = 0;
  targetsCleared = 0;
  for (const ring of rings) {
    ring.done = false;
    const color = ring.hazard ? COLOR_HAZARD : COLOR_TARGET;
    (ring.mesh.material as THREE.MeshBasicMaterial).color.set(color);
    (ring.halo.material as THREE.MeshBasicMaterial).color.set(color);
  }
  updateScore();
  havePrev = false; // drop the stale ring-crossing history after the jump
  resetCamera();
  crashEl.style.display = 'none';
}

/**
 * Continue after a crash (E): same spot over the ground, back up at the
 * course's start altitude, nose level along the course. Rings and score are
 * kept, minus CRASH_PENALTY.
 */
const _contLook = new THREE.Vector3();
function continueAfterCrash(): void {
  flight.crashed = false;
  flight.angVel.set(0, 0, 0);
  flight.throttle = THROTTLE_START;
  // Start altitude, but never inside a hill that rises above it.
  const ground = terrain.groundHeight(camera.position) ?? 0;
  camera.position.y = Math.max(courseStart.y + terrain.anchor.worldOffset.y, ground + 150);
  camera.up.set(0, 1, 0);
  _contLook.set(courseNormal.x, 0, courseNormal.z).normalize();
  camera.lookAt(_contLook.multiplyScalar(1000).add(camera.position));
  camera.getWorldDirection(forward);
  flight.vel.copy(forward).multiplyScalar(SPEED_START);
  flight.speed = SPEED_START;
  score += CRASH_PENALTY;
  updateScore();
  showPop(`${CRASH_PENALTY}`, '#ff6b6b'); // "-30"
  havePrev = false; // don't count the vertical jump as a ring crossing
  crashEl.style.display = 'none';
}

// -- input: keyboard ---------------------------------------------------------

const keys = new Set<string>();
const justPressed = new Set<string>();
window.addEventListener('keydown', (e) => {
  if (!started) return;
  if (!keys.has(e.code)) justPressed.add(e.code);
  keys.add(e.code);
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

// -- input: touch ------------------------------------------------------------

const touchSteer = { x: 0, y: 0 };
let steerId: number | null = null;
let steerStart = { x: 0, y: 0 };
let pinchDist = 0;
let tapReset = false;

const touchDist = (a: Touch, b: Touch): number => Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
const clearSteer = (): void => { steerId = null; touchSteer.x = 0; touchSteer.y = 0; };
const anchorSteer = (t: Touch): void => {
  steerId = t.identifier; steerStart = { x: t.clientX, y: t.clientY }; touchSteer.x = 0; touchSteer.y = 0;
};

canvas.addEventListener('touchstart', (e) => {
  if (!started) return;
  e.preventDefault();
  if (flight.crashed) { tapReset = true; return; }
  if (e.touches.length === 1) anchorSteer(e.touches[0]);
  else if (e.touches.length === 2) { clearSteer(); pinchDist = touchDist(e.touches[0], e.touches[1]); }
}, { passive: false });

canvas.addEventListener('touchmove', (e) => {
  if (!started) return;
  e.preventDefault();
  if (e.touches.length >= 2) {
    const d = touchDist(e.touches[0], e.touches[1]);
    if (pinchDist > 0) flight.throttle = Math.min(Math.max(flight.throttle + (d - pinchDist) / 300, 0), 1);
    pinchDist = d;
    return;
  }
  const t = e.touches[0];
  if (t === undefined || t.identifier !== steerId) return;
  const clamp1 = (v: number): number => Math.min(Math.max(v, -1), 1);
  touchSteer.x = clamp1((t.clientX - steerStart.x) / STEER_RADIUS);
  touchSteer.y = clamp1((t.clientY - steerStart.y) / STEER_RADIUS);
}, { passive: false });

const touchEnd = (e: TouchEvent): void => {
  e.preventDefault();
  pinchDist = 0;
  if (e.touches.length === 1) anchorSteer(e.touches[0]);
  else if (e.touches.length === 0) clearSteer();
};
canvas.addEventListener('touchend', touchEnd, { passive: false });
canvas.addEventListener('touchcancel', touchEnd, { passive: false });

// -- input: gamepad (Thrustmaster HOTAS Warthog) -----------------------------

/** Warthog is two devices — split them by id (throttle's id has GP_THROTTLE_ID). */
function warthogPads(): { stick: Gamepad | null; throttle: Gamepad | null } {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let stick: Gamepad | null = null;
  let throttle: Gamepad | null = null;
  for (const p of pads) {
    if (!p) continue;
    if (p.id.toLowerCase().includes(GP_THROTTLE_ID)) throttle = p;
    else stick = p;
  }
  return { stick, throttle };
}
// Log each device's id + axis count on connect, so the mapping above can be
// checked/adjusted against what this machine actually reports (open the console).
window.addEventListener('gamepadconnected', (e) => {
  const g = e.gamepad;
  console.log(`[gamepad] "${g.id}" — ${g.axes.length} axes, ${g.buttons.length} buttons (index ${g.index})`);
});

/** Read an axis by index, 0 if that axis doesn't exist on this device. */
const axis = (a: readonly number[], i: number): number => (i < a.length ? a[i] : 0);
/** Suppress tiny centre noise so a resting stick doesn't drift. */
const deadzone = (v: number): number => (Math.abs(v) < GP_DEADZONE ? 0 : v);

// -- systems -----------------------------------------------------------------

const target = new THREE.Vector3();
const forward = new THREE.Vector3();
// Inputs are sampled once per rendered frame; the physics then runs in fixed
// sub-steps (see the frame loop), so behaviour is identical at any frame rate.
let throttleDir = 0; // −1 / 0 / +1 from the throttle keys, applied per physics step
let leverThrottle: number | null = null; // absolute HOTAS lever setting, if present

function gatherInputs(): void {
  // Stick / rudder: body-axis RATE commands (x = pitch, y = yaw, z = roll).
  // Holding deflects a rate; releasing an axis sets its rate to 0 → the aircraft
  // stops rotating on that axis and HOLDS attitude (no auto-level).
  target.set(0, 0, 0);
  if (keys.has('KeyA')) target.z += FM.ROLL_RATE;
  if (keys.has('KeyD')) target.z -= FM.ROLL_RATE;
  if (keys.has('KeyQ')) target.y += FM.YAW_RATE;
  if (keys.has('KeyE')) target.y -= FM.YAW_RATE;
  if (keys.has('KeyW')) target.x -= FM.PITCH_RATE;
  if (keys.has('KeyS')) target.x += FM.PITCH_RATE;
  target.z -= FM.ROLL_RATE * touchSteer.x;
  target.x += FM.PITCH_RATE * touchSteer.y;

  throttleDir = 0;
  if (keys.has('Equal') || keys.has('NumpadAdd')) throttleDir += 1;
  if (keys.has('Minus') || keys.has('NumpadSubtract')) throttleDir -= 1;

  leverThrottle = null;
  const { stick, throttle } = warthogPads();
  if (stick) {
    target.z -= FM.ROLL_RATE * deadzone(axis(stick.axes, GP_AXIS_ROLL)) * (GP_INVERT_ROLL ? -1 : 1);
    target.x += FM.PITCH_RATE * deadzone(axis(stick.axes, GP_AXIS_PITCH)) * (GP_INVERT_PITCH ? -1 : 1);
  }
  if (throttle && throttle.axes.length > GP_AXIS_THR_B) {
    const lever = (axis(throttle.axes, GP_AXIS_THR_A) + axis(throttle.axes, GP_AXIS_THR_B)) / 2;
    leverThrottle = Math.min(Math.max((1 - lever) / 2, 0), 1);
  }
}

/** One fixed physics sub-step: throttle, pose (rate command), then flight path. */
function physicsStep(h: number): void {
  if (flight.crashed) return;
  flight.throttle = Math.min(Math.max(flight.throttle + throttleDir * THROTTLE_RATE * h, 0), 1);
  if (leverThrottle !== null) flight.throttle = leverThrottle; // absolute lever wins
  applyStick(camera.quaternion, flight.angVel, target, h, flight.speed);
  const out = stepDynamics(camera.quaternion, flight.vel, flight.throttle, h);
  flight.speed = out.speed;
  camera.position.addScaledVector(flight.vel, h);
}

const shift = new THREE.Vector3();
function rebaseLargeWorld(): void {
  shift.set(0, 0, 0);
  if (Math.abs(camera.position.x) > REBASE_THRESHOLD) shift.x = -Math.sign(camera.position.x) * REBASE_THRESHOLD;
  if (Math.abs(camera.position.z) > REBASE_THRESHOLD) shift.z = -Math.sign(camera.position.z) * REBASE_THRESHOLD;
  if (shift.x !== 0 || shift.z !== 0) {
    camera.position.add(shift);
    terrain.rebase(shift);
    ringGroup.position.copy(terrain.anchor.worldOffset); // keep rings glued to terrain
  }
}

/** Detect flying through each open ring: plane crossing within the radius. */
const camAbs = new THREE.Vector3();
const prevCamAbs = new THREE.Vector3();
const crossing = new THREE.Vector3();
let havePrev = false;
function ringCheck(): void {
  // Absolute camera position = user-space − worldOffset (rebase-independent).
  camAbs.copy(camera.position).sub(terrain.anchor.worldOffset);
  if (!havePrev) { prevCamAbs.copy(camAbs); havePrev = true; return; }

  for (const ring of rings) {
    if (ring.done) continue;
    const fPrev = prevCamAbs.clone().sub(ring.center).dot(courseNormal);
    const fCur = camAbs.clone().sub(ring.center).dot(courseNormal);
    if (fPrev === fCur) continue;
    if ((fPrev <= 0 && fCur > 0) || (fPrev >= 0 && fCur < 0)) {
      const t = fPrev / (fPrev - fCur); // param of plane crossing along the step
      crossing.lerpVectors(prevCamAbs, camAbs, t);
      if (crossing.distanceTo(ring.center) <= ring.radius) passRing(ring); // through it
      else skipRing(ring); // crossed its depth outside the rim → miss/dodge
    }
  }
  prevCamAbs.copy(camAbs);
}

/**
 * Progressive reveal: draw only the nearest rings ahead solid, fading farther
 * ones to transparent so the pilot can't scan the whole course in advance.
 */
const revTmp = new THREE.Vector3();
function revealUpdate(): void {
  const sPlane = revTmp.copy(camera.position).sub(terrain.anchor.worldOffset).sub(courseStart).dot(courseNormal);
  // the "next" station: the nearest ring not yet flown through/past
  let nextS = Infinity;
  for (const ring of rings) if (!ring.done && ring.s < nextS) nextS = ring.s;
  for (const ring of rings) {
    const ahead = ring.s - sPlane;
    let op: number;
    if (ahead < -80) op = 0; // just passed → gone
    else if (ahead < 140) op = 1; // at the ring's depth → full
    else {
      let rank = 0; // how many upcoming rings are nearer than this one
      for (const o of rings) {
        const oa = o.s - sPlane;
        if (oa >= 140 && oa < ahead) rank++;
      }
      op = rank < REVEAL_FULL ? 1 : rank < REVEAL_VIS ? 1 - (rank - REVEAL_FULL) / (REVEAL_VIS - REVEAL_FULL) : 0;
    }
    // The ring(s) to fly through now read solid; everything after is see-through.
    const isNext = !ring.done && ring.s === nextS;
    const { outline, fill } = OPACITY[ring.hazard ? 'red' : 'blue'][isNext ? 'next' : 'later'];
    (ring.mesh.material as THREE.MeshBasicMaterial).opacity = op * outline;
    (ring.halo.material as THREE.MeshBasicMaterial).opacity = op * fill;
  }
}

function crashCheck(): void {
  if (flight.crashed) return; // R / tap handling lives in the frame loop
  const ground = terrain.groundHeight(camera.position) ?? 0;
  if (ground > camera.position.y) crash();
}

/**
 * The terrain is streamed live from map servers (it is not bundled), so it must
 * download around the spawn before the run looks right. We do that WHILE the
 * participant is on the login screen: the Start button shows the progress and is
 * disabled until the map is ready — so the big "loading" splash never sits over
 * the flight. (Ongoing streaming as you fly north is normal and silent.)
 */
function loadingUi(): void {
  loadingEl.style.display = 'none'; // no center splash during gameplay
  if (started) return;
  if (terrain.status.loading) {
    startBtn.disabled = true;
    startBtn.textContent = `Loading map… ${(terrain.status.progress * 100).toFixed(0)}%`;
  } else {
    startBtn.disabled = false;
    startBtn.textContent = 'Start';
  }
}

function hud(): void {
  // Live cockpit tapes: airspeed left of the centre, altitude right of it.
  drawTape(spdTape, flight.speed * 1.94384);      // m/s → knots
  drawTape(altTape, camera.position.y * 3.28084); // m → feet
}

// -- compass heading tape ----------------------------------------------------
// The world's north is -Z and east is +X (the course runs south→north), so the
// aircraft's compass heading is the bearing of its forward direction.
const _heading = new THREE.Vector3();
function currentHeading(): number {
  camera.getWorldDirection(_heading);
  const deg = Math.atan2(_heading.x, -_heading.z) * 180 / Math.PI;
  return (deg % 360 + 360) % 360;
}

/** Tape look, matched to the reference: px per degree + tick/label metrics. */
const CMP_DEG = 2.2;           // px per degree (tick density)
let cmpW = 0, cmpH = 0;
function sizeCompass(): void {
  const r = compassEl.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  cmpW = r.width; cmpH = r.height;
  compassEl.width = Math.round(cmpW * dpr);
  compassEl.height = Math.round(cmpH * dpr);
  compassCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
function headingLabel(d: number): string {
  const dd = (d % 360 + 360) % 360;
  if (dd === 0) return 'N';
  if (dd === 90) return 'E';
  if (dd === 180) return 'S';
  if (dd === 270) return 'W';
  return String(dd / 10).padStart(2, '0'); // 300→"30", 60→"06"
}
// ── Compass fill colours ─ set these (the black outline stays black either way).
//    Each controls the FILL/line colour only, not the outline stroke.
// F-15 style HUD green (matches --hud in index.html).
const HUD_GREEN = '#3dff5c';
const HUD_OUTLINE = 'rgba(0, 0, 0, .7)'; // thin dark rim so symbols read over bright sky
const CMP_COLOR_MINOR = HUD_GREEN;    // short ticks (minor, every 10°)
const CMP_COLOR_MAJOR = HUD_GREEN;    // long ticks (every 30°) + their label (number/letter)
const CMP_COLOR_POINTER = HUD_GREEN;  // centre pointer line + the big heading number above it

const CMP_COLOR_TARGET = '#' + COLOR_TARGET.toString(16).padStart(6, '0'); // same blue as the rings

/** Hide the compass dot once the next ring is this close (m, along the course):
 *  its bearing swings wildly as you fly through it. The following ring's dot
 *  appears as soon as this one is passed or skipped. */
const CMP_TARGET_HIDE_DIST = 400;

/** Compass bearing (deg) from the aircraft to the next blue ring, or null when
 *  none is left or it's about to be flown through. */
const _tgtVec = new THREE.Vector3();
function nextTargetBearing(): number | null {
  let next: Ring | null = null;
  for (const ring of rings) if (!ring.done && !ring.hazard && (!next || ring.s < next.s)) next = ring;
  if (!next) return null;
  _tgtVec.copy(next.center).add(terrain.anchor.worldOffset).sub(camera.position);
  if (_tgtVec.dot(courseNormal) < CMP_TARGET_HIDE_DIST) return null; // distance ahead along the course
  const deg = Math.atan2(_tgtVec.x, -_tgtVec.z) * 180 / Math.PI;
  return (deg % 360 + 360) % 360;
}

function drawCompass(): void {
  const ctx = compassCtx, W = cmpW, H = cmpH;
  if (!W) return;
  ctx.clearRect(0, 0, W, H);
  const cx = W / 2;
  const midY = H - 18;                 // tick centre line, near the bottom
  const heading = currentHeading();
  ctx.textAlign = 'center';

  // Black outline thickness (px), tuned separately because text and lines
  // render their outline differently: OUT_TEXT = visible rim around glyphs,
  // OUT_LINE = visible rim on each side of a tick / the pointer.
  const OUT_TEXT = 0.9;
  const OUT_LINE = 0.5;
  const strokedText = (t: string, x: number, y: number, color: string): void => {
    ctx.lineJoin = 'round';
    ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = OUT_TEXT * 2; ctx.strokeText(t, x, y);
    ctx.fillStyle = color; ctx.fillText(t, x, y);
  };
  const strokedLine = (x0: number, y0: number, x1: number, y1: number, w: number, color: string): void => {
    ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = w + OUT_LINE * 2; // black underlay
    ctx.beginPath(); ctx.moveTo(x0, y0 - OUT_LINE); ctx.lineTo(x1, y1 + OUT_LINE); ctx.stroke();
    ctx.strokeStyle = color; ctx.lineWidth = w;                 // fill colour on top
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  };

  // ticks + labels: every 10°, major (taller/thicker) + labelled every 30°
  const halfDeg = (W / 2) / CMP_DEG + 20;
  const startD = Math.ceil((heading - halfDeg) / 10) * 10;
  const endD = Math.floor((heading + halfDeg) / 10) * 10;
  for (let d = startD; d <= endD; d += 10) {
    const diff = ((d - heading + 540) % 360) - 180; // shortest signed offset
    const x = cx + diff * CMP_DEG;
    const major = ((d % 30) + 30) % 30 === 0;
    const half = major ? 8 : 5;
    strokedLine(x, midY - half, x, midY + half, major ? 1.6 : 1.0, major ? CMP_COLOR_MAJOR : CMP_COLOR_MINOR);
    if (major && Math.abs(x - cx) > 16) {   // no label under the centre readout
      ctx.font = '400 16px Roboto, system-ui, sans-serif';
      ctx.textBaseline = 'alphabetic';
      strokedText(headingLabel(d), x, midY - half - 6, CMP_COLOR_MAJOR);
    }
  }

  // fixed centre pointer, from just under the readout down past the ticks
  strokedLine(cx, 35, cx, midY + 21, 2.0, CMP_COLOR_POINTER);

  // next blue target's bearing: a small blue dot on the tick line (pinned to
  // the tape's end, half-faded, when it's outside the visible span)
  const tgt = nextTargetBearing();
  if (tgt !== null) {
    const diff = ((tgt - heading + 540) % 360) - 180;
    const lim = W / 2 - 8;
    const x = cx + Math.max(-lim, Math.min(lim, diff * CMP_DEG));
    ctx.globalAlpha = Math.abs(diff * CMP_DEG) > lim ? 0.5 : 1;
    ctx.beginPath(); ctx.arc(x, midY, 5, 0, Math.PI * 2);
    ctx.fillStyle = CMP_COLOR_TARGET; ctx.fill();
    ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = 1.2; ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // big current-heading readout, centred at top
  ctx.font = '500 20px Roboto, system-ui, sans-serif';
  ctx.textBaseline = 'top';
  strokedText(String(Math.round(heading) % 360).padStart(3, '0'), cx, 10, CMP_COLOR_POINTER);
}

// -- airspeed / altitude tapes -----------------------------------------------
// Fighter-HUD moving scales: the scale slides past a fixed boxed readout at the
// tape's vertical centre. Ticks sit on the edge facing the centre and the
// readout box has a caret pointing inward, mirrored for the two sides.

interface Tape {
  el: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  inner: 'left' | 'right'; // which canvas edge faces the screen centre
  unit: string;
  minor: number;           // value between minor ticks
  major: number;           // value between labelled major ticks
  pxPerMinor: number;      // spacing between minor ticks
  min: number;             // lowest value drawn on the scale (-Infinity = none)
  readoutStep: number;     // boxed number is rounded to this step (1 = exact)
  w: number; h: number;
}
function makeTape(el: HTMLCanvasElement, inner: Tape['inner'], unit: string,
                  minor: number, major: number, min: number, readoutStep = 1): Tape {
  return { el, ctx: el.getContext('2d')!, inner, unit, minor, major, pxPerMinor: 12, min, readoutStep, w: 0, h: 0 };
}
const spdTape = makeTape(spdTapeEl, 'right', 'KT', 10, 50, 0);           // left of centre
// Altitude readout in 100 ft steps (the scale behind it still slides smoothly).
const ALT_READOUT_STEP = 100;
const altTape = makeTape(altTapeEl, 'left', 'FT', 100, 500, -Infinity, ALT_READOUT_STEP); // right of centre

function sizeTape(t: Tape): void {
  const r = t.el.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  t.w = r.width; t.h = r.height;
  t.el.width = Math.round(t.w * dpr);
  t.el.height = Math.round(t.h * dpr);
  t.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function drawTape(t: Tape, value: number): void {
  const { ctx, w: W, h: H } = t;
  if (!W) return;
  ctx.clearRect(0, 0, W, H);
  // Author everything as if the inner edge is on the right, then mirror x.
  const X = (x: number): number => (t.inner === 'right' ? x : W - x);
  const align = (a: 'left' | 'right'): CanvasTextAlign =>
    t.inner === 'right' ? a : (a === 'left' ? 'right' : 'left');

  const unitH = 22;                 // room for the unit label at the bottom
  const top = 4, bot = H - unitH;
  const cy = (top + bot) / 2;
  const edge = W - 3;               // scale spine, on the inner edge
  const pxPerUnit = t.pxPerMinor / t.minor;

  ctx.lineCap = 'butt';
  ctx.lineJoin = 'round';
  const line = (pts: [number, number][], width: number, close = false): void => {
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), y) : ctx.moveTo(X(x), y)));
    if (close) ctx.closePath();
    ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = width + 1.2; ctx.stroke();
    ctx.strokeStyle = HUD_GREEN; ctx.lineWidth = width; ctx.stroke();
  };
  const text = (s: string, x: number, y: number, a: 'left' | 'right', font: string): void => {
    ctx.font = font; ctx.textAlign = align(a); ctx.textBaseline = 'middle';
    ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = 2; ctx.strokeText(s, X(x), y);
    ctx.fillStyle = HUD_GREEN; ctx.fillText(s, X(x), y);
  };

  // moving scale, clipped to the tape window
  ctx.save();
  ctx.beginPath(); ctx.rect(0, top, W, bot - top); ctx.clip();
  line([[edge, top], [edge, bot]], 1.4);
  const span = (bot - top) / 2 / pxPerUnit + t.minor;
  const first = Math.ceil((value - span) / t.minor) * t.minor;
  for (let v = first; v <= value + span; v += t.minor) {
    if (v < t.min) continue;
    const y = cy - (v - value) * pxPerUnit;
    const isMajor = Math.round(v) % t.major === 0;
    const len = isMajor ? 14 : 7;
    line([[edge - len, y], [edge, y]], isMajor ? 1.6 : 1.1);
    if (isMajor) text(String(Math.round(v)), edge - len - 5, y, 'right', '400 13px Roboto, system-ui, sans-serif');
  }
  ctx.restore();

  // fixed readout box with a caret pointing at the scale
  const bh = 24, caret = 8;
  const bx0 = 6, bx1 = edge - 18;
  const by0 = cy - bh / 2, by1 = cy + bh / 2;
  ctx.beginPath();
  [[bx0, by0], [bx1, by0], [bx1 + caret, cy], [bx1, by1], [bx0, by1]]
    .forEach(([x, y], i) => (i ? ctx.lineTo(X(x), y) : ctx.moveTo(X(x), y)));
  ctx.closePath();
  ctx.fillStyle = 'rgba(0, 14, 4, .78)'; ctx.fill(); // hide the scale behind it
  line([[bx0, by0], [bx1, by0], [bx1 + caret, cy], [bx1, by1], [bx0, by1]], 1.6, true);
  text(String(Math.round(value / t.readoutStep) * t.readoutStep), bx1 - 5, cy + 1, 'right', '500 16px Roboto, system-ui, sans-serif');

  // unit label under the tape
  text(t.unit, edge, H - unitH / 2, 'right', '500 11px Roboto, system-ui, sans-serif');
}

// -- target designator ---------------------------------------------------------
// HUD brackets around the next ring(s) to fly through, drawn in screen space so
// they're readable at any range — the ring itself is only a few pixels wide
// kilometres out. Moves to the next station the instant the current one is
// passed. Red hazards at that station get an "X" box (avoid), greens a
// bracket box (fly through).
const SHOW_DESIGNATOR = false;
const DESIG_MIN_HALF = 14; // px — never smaller than this, however far the ring
let desW = 0, desH = 0;
function sizeDesignator(): void {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  desW = window.innerWidth; desH = window.innerHeight;
  designatorEl.width = Math.round(desW * dpr);
  designatorEl.height = Math.round(desH * dpr);
  designatorCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
const _desP = new THREE.Vector3();
const _desView = new THREE.Vector3();
function drawDesignator(): void {
  const ctx = designatorCtx;
  ctx.clearRect(0, 0, desW, desH);
  if (!SHOW_DESIGNATOR || flight.crashed) return;
  let nextS = Infinity;
  for (const ring of rings) if (!ring.done && ring.s < nextS) nextS = ring.s;
  const off = terrain.anchor.worldOffset;
  const pxPerRad = desH / 2 / Math.tan((camera.fov * Math.PI) / 360);
  for (const ring of rings) {
    if (ring.done || ring.s !== nextS) continue;
    _desP.copy(ring.center).add(off);
    _desView.copy(_desP).applyMatrix4(camera.matrixWorldInverse);
    if (_desView.z > -1) continue; // behind the aircraft
    _desP.project(camera);
    const x = (_desP.x + 1) / 2 * desW, y = (1 - _desP.y) / 2 * desH;
    const half = Math.max(DESIG_MIN_HALF, (ring.radius / -_desView.z) * pxPerRad * 1.15);
    const c = Math.min(half * 0.45, 12); // bracket arm length
    ctx.lineWidth = 2; ctx.lineCap = 'square';
    const path = (): void => {
      ctx.beginPath();
      for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
        const cx = x + sx * half, cy = y + sy * half;
        ctx.moveTo(cx - sx * c, cy); ctx.lineTo(cx, cy); ctx.lineTo(cx, cy - sy * c);
      }
      if (ring.hazard) { // X through the middle: avoid
        const k = Math.min(half * 0.5, 10);
        ctx.moveTo(x - k, y - k); ctx.lineTo(x + k, y + k);
        ctx.moveTo(x + k, y - k); ctx.lineTo(x - k, y + k);
      }
    };
    path(); ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = 3.4; ctx.stroke();
    path(); ctx.strokeStyle = HUD_GREEN; ctx.lineWidth = 2; ctx.stroke();
  }
}

// -- attitude symbology (F-15 / F-16 style) ------------------------------------
// Everything is placed by projecting real directions through the camera (the
// camera IS the aircraft: its forward axis is the nose), so pitch and roll come
// out exactly right with no separate attitude maths:
//   • W aircraft datum — fixed at screen centre = where the nose points.
//   • Flight path marker — the direction of flight.vel (the physics' world
//     velocity), so it sits below the W when the nose is up but the aircraft
//     climbs less steeply (angle of attack), and drifts sideways in a skid.
//   • Horizon line (0°, solid) + pitch ladder every PITCH_STEP° (dashed), end
//     ticks pointing toward the horizon, numbered both ends. Only rungs within
//     LADDER_RANGE° of the nose's current pitch are drawn.

const PITCH_STEP = 5;       // degrees between ladder rungs
const LADDER_RANGE = 15;    // show rungs only within ± this many degrees of the nose
const LADDER_DASH = [7, 5]; // rung dash pattern (px on, px off); [] = solid lines
const RUNG_HALF = 70;       // px from the ladder centre to each rung's outer end
const RUNG_GAP = 32;        // px of empty centre on each side (keeps W/FPM clear)
const HORIZON_HALF = 230;   // px — the horizon line is longer than a rung
const HORIZON_GAP = 40;
const RUNG_TICK = 9;        // px end tick pointing toward the horizon

let attW = 0, attH = 0;
function sizeAttitude(): void {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  attW = window.innerWidth; attH = window.innerHeight;
  attitudeEl.width = Math.round(attW * dpr);
  attitudeEl.height = Math.round(attH * dpr);
  attitudeCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

const _attFwd = new THREE.Vector3();
const _attDir = new THREE.Vector3();
const _attP = new THREE.Vector3();
const _attQ = new THREE.Vector3();
const _attH = new THREE.Vector3();
const _attR = new THREE.Vector3();
/** Screen position (CSS px) of a world DIRECTION from the aircraft, or null if behind. */
function projectDir(dir: THREE.Vector3, out: { x: number; y: number }): boolean {
  if (dir.dot(_attFwd) < 0.05) return false; // behind / at the edge of view
  _attP.copy(camera.position).addScaledVector(dir, 1000).project(camera);
  out.x = (_attP.x + 1) / 2 * attW;
  out.y = (1 - _attP.y) / 2 * attH;
  return true;
}

function drawAttitude(): void {
  const ctx = attitudeCtx;
  ctx.clearRect(0, 0, attW, attH);
  if (!attW) return;
  camera.updateMatrixWorld();
  camera.getWorldDirection(_attFwd);

  // Horizontal heading axes: along the nose (flattened) and to its right.
  _attH.set(_attFwd.x, 0, _attFwd.z);
  if (_attH.lengthSq() < 1e-6) _attH.set(0, 1, 0).applyQuaternion(camera.quaternion).setY(0); // nose vertical
  _attH.normalize();
  _attR.set(-_attH.z, 0, _attH.x); // right of heading (north → east)

  const stroke = (draw: () => void, width: number, dash: number[] = []): void => {
    ctx.setLineDash(dash);
    ctx.lineCap = 'butt'; ctx.lineJoin = 'miter';
    ctx.beginPath(); draw(); ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = width + 1.4; ctx.stroke();
    ctx.beginPath(); draw(); ctx.strokeStyle = HUD_GREEN; ctx.lineWidth = width; ctx.stroke();
    ctx.setLineDash([]);
  };

  // Ladder + horizon, clipped to the window between the speed/alt tapes and
  // below the compass, like a real HUD's field of view.
  const sp = spdTapeEl.getBoundingClientRect(), al = altTapeEl.getBoundingClientRect();
  const cm = compassEl.getBoundingClientRect();
  const clipL = sp.right + 8, clipR = al.left - 8, clipT = cm.bottom + 6, clipB = attH - 30;
  ctx.save();
  ctx.beginPath(); ctx.rect(clipL, clipT, clipR - clipL, clipB - clipT); ctx.clip();

  const c = { x: 0, y: 0 }, cr = { x: 0, y: 0 };
  ctx.font = '400 13px Roboto, system-ui, sans-serif';
  ctx.textBaseline = 'middle';
  const nosePitch = (Math.asin(Math.max(-1, Math.min(1, _attFwd.y))) * 180) / Math.PI;
  for (let deg = -90; deg <= 90; deg += PITCH_STEP) {
    if (Math.abs(deg - nosePitch) > LADDER_RANGE) continue;
    const a = (deg * Math.PI) / 180;
    _attDir.copy(_attH).multiplyScalar(Math.cos(a)).addScaledVector(_WORLD_UP, Math.sin(a));
    if (!projectDir(_attDir, c)) continue;
    // on-screen rung direction: nudge the direction a hair to the right
    _attQ.copy(_attDir).addScaledVector(_attR, 0.01);
    if (!projectDir(_attQ, cr)) continue;
    let ux = cr.x - c.x, uy = cr.y - c.y;
    const len = Math.hypot(ux, uy) || 1; ux /= len; uy /= len;
    const nx = -uy, ny = ux; // screen "down" relative to the rung (toward − pitch)
    if (c.x < clipL - 300 || c.x > clipR + 300 || c.y < clipT - 300 || c.y > clipB + 300) continue;

    const half = deg === 0 ? HORIZON_HALF : RUNG_HALF;
    const gap = deg === 0 ? HORIZON_GAP : RUNG_GAP;
    const tick = deg > 0 ? RUNG_TICK : deg < 0 ? -RUNG_TICK : 0; // toward the horizon
    stroke(() => {
      for (const s of [-1, 1]) {
        const x0 = c.x + ux * gap * s, y0 = c.y + uy * gap * s;
        const x1 = c.x + ux * half * s, y1 = c.y + uy * half * s;
        ctx.moveTo(x0, y0); ctx.lineTo(x1, y1);
        if (tick) ctx.lineTo(x1 + nx * tick, y1 + ny * tick);
      }
    }, deg === 0 ? 1.8 : 1.5, deg < 0 ? LADDER_DASH : []); // climb rungs solid, dive rungs dashed

    if (deg !== 0) {
      const label = String(deg);
      const ang = Math.atan2(uy, ux);
      for (const s of [-1, 1]) {
        ctx.save();
        ctx.translate(c.x + ux * (half + 6) * s, c.y + uy * (half + 6) * s);
        ctx.rotate(ang);
        ctx.textAlign = s > 0 ? 'left' : 'right';
        // labels sit level with the tick end, so nudge toward it
        const ty = tick * 0.55;
        ctx.lineJoin = 'round';
        ctx.strokeStyle = HUD_OUTLINE; ctx.lineWidth = 2; ctx.strokeText(label, 0, ty);
        ctx.fillStyle = HUD_GREEN; ctx.fillText(label, 0, ty);
        ctx.restore();
      }
    }
  }
  ctx.restore();

  // W aircraft datum — fixed at the boresight (screen centre).
  const wx = attW / 2, wy = attH / 2;
  stroke(() => {
    ctx.moveTo(wx - 26, wy); ctx.lineTo(wx - 13, wy);
    ctx.lineTo(wx - 6.5, wy + 9); ctx.lineTo(wx, wy);
    ctx.lineTo(wx + 6.5, wy + 9); ctx.lineTo(wx + 13, wy);
    ctx.lineTo(wx + 26, wy);
  }, 2);

  // Flight path marker — where the aircraft is actually going.
  if (flight.speed > 1) {
    _attDir.copy(flight.vel).normalize();
    const f = { x: 0, y: 0 };
    if (projectDir(_attDir, f)) {
      // keep it inside the HUD window (a real FPM is caged to the HUD's FOV)
      f.x = Math.min(Math.max(f.x, clipL + 20), clipR - 20);
      f.y = Math.min(Math.max(f.y, clipT + 20), clipB - 20);
      const r = 6;
      stroke(() => {
        ctx.moveTo(f.x + r, f.y); ctx.arc(f.x, f.y, r, 0, Math.PI * 2);
        ctx.moveTo(f.x - r, f.y); ctx.lineTo(f.x - r - 11, f.y); // left wing
        ctx.moveTo(f.x + r, f.y); ctx.lineTo(f.x + r + 11, f.y); // right wing
        ctx.moveTo(f.x, f.y - r); ctx.lineTo(f.x, f.y - r - 7);  // tail
      }, 1.8);
    }
  }
}
const _WORLD_UP = new THREE.Vector3(0, 1, 0);

// -- frame loop --------------------------------------------------------------

const PHYS_DT = 1 / 120; // fixed physics step — decoupled from the render rate
let physAcc = 0;
let last = performance.now();
function frame(now: number): void {
  const dt = Math.min((now - last) / 1000, 0.1);
  last = now;

  if (started) {
    // R (any layout — it's the physical key) or a post-crash tap restarts the
    // run from the start of the course; it never returns to the login screen.
    if (justPressed.has('KeyR') || tapReset) resetRun();
    // E is the physical key (KeyE), so it works on Hebrew (ק) and English layouts.
    else if (flight.crashed && justPressed.has('KeyE')) continueAfterCrash();
    // Sample inputs once, then advance the physics in fixed PHYS_DT sub-steps so
    // the result is frame-rate independent (the leftover time carries over).
    gatherInputs();
    physAcc = Math.min(physAcc + dt, 0.25); // cap to avoid a spiral of death
    while (physAcc >= PHYS_DT) {
      physicsStep(PHYS_DT);
      physAcc -= PHYS_DT;
    }
    rebaseLargeWorld();
    ringCheck();
    crashCheck();
    hud();
    drawCompass();
    drawDesignator();
    drawAttitude();
  }
  terrain.update();
  revealUpdate();
  loadingUi();

  if (popTimer > 0) {
    popTimer -= dt;
    if (popTimer <= 0) popEl.style.opacity = '0';
  }

  justPressed.clear();
  tapReset = false;
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

// -- boot: login → simulator -------------------------------------------------

function startSimulation(): void {
  started = true;
  loginEl.style.display = 'none';
  scorebarEl.style.display = 'flex';
  attitudeEl.style.display = 'block';
  designatorEl.style.display = 'block';
  spdTapeEl.style.display = 'block';
  altTapeEl.style.display = 'block';
  compassEl.style.display = 'block';
  sizeCompass();
  sizeDesignator();
  sizeTape(spdTape);
  sizeTape(altTape);
  updateScore();
  resetCamera();
  canvas.focus();
}

loginForm.addEventListener('submit', (e) => {
  e.preventDefault();
  if (terrain.status.loading) return; // map not ready yet (button is disabled anyway)
  const name = fullNameEl.value.trim();
  const subject = subjectIdEl.value.trim();
  if (!name) { loginErr.textContent = 'Please enter your full name.'; return; }
  if (!subject) { loginErr.textContent = 'Please enter your subject number.'; return; }
  loginErr.textContent = '';
  participant = { name, subject };
  startSimulation();
});

if ('ontouchstart' in window || navigator.maxTouchPoints > 0) {
  crashEl.textContent = 'You crashed! Tap to reset.';
}

// Anchor the world, build the course, and start rendering immediately so the
// Kinneret terrain is already loading behind the login screen.
buildCourse();
resetCamera();
resize();
requestAnimationFrame(frame);

// dev convenience: expose live state for debugging in the console
Object.defineProperty(window, '__sim', {
  configurable: true,
  get: () => ({ terrain, flight, camera, rings, score, participant, courseNormal, courseStart }),
});
