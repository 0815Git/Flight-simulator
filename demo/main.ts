/**
 * Israel Flight Simulator — a ring-course game built on the threetiles engine.
 *
 * SESSION FLOW
 *   1. The login screen collects the participant's name and id, and whether
 *      they fly the NORTH region (Hula valley / Bekaa) or the SOUTH one (Eilat
 *      mountains). The world is anchored on the chosen region and its terrain
 *      streams while the details are typed.
 *   2. "Start session" drops the aircraft SPAWN_BACK metres behind the course,
 *      with a lead-in to settle down. Nothing is measured yet.
 *   3. Crossing the wide amber START GATE on the centerline begins the run:
 *      the clock starts and everything from there is recorded.
 *   4. **Esc** stops the route. The simulator freezes exactly where it is and
 *      asks whether to move on. "No" resumes from the identical state; "Yes"
 *      banks the route's row and loads the next route of the same region,
 *      which again starts far back with its own start gate.
 *   5. After the third route a summary appears with a button that downloads
 *      the session as a CSV — one row per route (name, id, region, route,
 *      seconds from the gate to Esc, crashes, score, rings passed/total).
 *
 * THE COURSE
 * Every ring faces straight down the course (its opening squares up to the
 * aircraft's nose); the rings only "crab" left/right and up/down along the way,
 * they never rotate away. Rings are scored like a target board: a large ring is
 * worth up to 5 points and a small precision ring up to 10, but only a crossing
 * through the middle pays the full amount — it falls off towards the rim. A red
 * square hazard costs 10 wherever it is entered. The score, rings and route are
 * shown at the top; a fighter-style HUD (W datum, flight path marker, pitch
 * ladder) helps aim through each hoop.
 *
 * Keyboard: **A/D** roll, **Q/E** yaw, **W/S** pitch, **+/-** throttle,
 * **Esc** stop the route, **1/2/3** jump to a route (a manual override — it
 * does not record a row), **R** restart the route, **E** after a crash
 * continues from the crash spot (costs CRASH_PENALTY points). Touch:
 * one-finger drag steers (right = bank right, down = nose up), two-finger
 * pinch is the throttle, tap resets after a crash.
 */import * as THREE from 'three';
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

// -- Gamepad mapping (HOTAS: a stick and a throttle on separate USB ports) ---
// Each device has its OWN axes numbered from 0, so the indices below are read
// per device. Telling them apart by name is unreliable — the Warthog's stick is
// called "Joystick - HOTAS Warthog", with no "throttle" in it, and a stick from
// another set has another name again — so they are matched by USB product id
// (all Thrustmaster, vendor 044f), with the name as a fallback. Stick = roll (X)
// + pitch (Y), mirroring touch/keys; the two linked throttle levers set speed
// (+1 = slowest, −1 = fastest). Flip an INVERT flag if a direction feels
// reversed. Yaw stays on Q/E (the stick's axes here are roll and pitch only).
/** USB product ids (vendor 044f) that are THROTTLES. */
const GP_THROTTLE_PRODUCTS = ['0404']; // HOTAS Warthog Throttle
/** USB product ids that are STICKS. */
const GP_STICK_PRODUCTS = ['0402', '0422']; // Warthog Joystick, Solaris Base
/** If several sticks are plugged in, this one wins. */
const GP_STICK_PREFERRED = '0422'; // Solaris Base
/** Fallback for any device not in the lists above: a name containing this is a
 *  throttle, anything else is a stick. */
const GP_THROTTLE_ID = 'throttle';
const GP_AXIS_ROLL = 0;    // stick device: left/right → roll  (like A/D)
const GP_AXIS_PITCH = 1;   // stick device: fwd/back   → pitch (like W/S)
const GP_AXIS_THR_A = 2;   // throttle device: lever A ┐ linked pair, averaged
const GP_AXIS_THR_B = 5;   // throttle device: lever B ┘ +1 → idle, −1 → full power
const GP_DEADZONE = 0.06;  // ignore tiny stick noise near centre
const GP_INVERT_ROLL = false;
const GP_INVERT_PITCH = false;

/**
 * Two REGIONS, picked on the login screen: north (the Hula valley up the Bekaa)
 * and south (the Eilat mountains), each holding the same three routes in a
 * different place. The world is anchored at the region's own anchor point —
 * Mercator scale changes with latitude, so one anchor cannot serve both. The
 * region is therefore chosen before the run starts, not during it.
 */
type RegionId = 'north' | 'south';

/** Ring size → radius (m). Bigger = easier (+5); small/precision = +10. HAZ = red. */
type RingSize = 'XL' | 'L' | 'M' | 'S' | 'XS' | 'HAZ';
const SIZE_RADIUS: Record<RingSize, number> = { XL: 205, L: 140, M: 92, S: 56, XS: 34, HAZ: 115 };
const HAZARD_POINTS = -10; // red ring flown through

/**
 * THE RING TABLES — one row per ring, easy to hand-edit. There is one table
 * per route (RINGS_1 / RINGS_2 / RINGS_3 below) and they are independent, so
 * editing one changes only that route. Fields:
 *   gap  = distance (m) along the course from the PREVIOUS ring (gap 0 ≈ a ring
 *          stacked/beside the previous one at the same station — a pair).
 *   side = offset from the centerline: + = right, − = left (m).
 *   up   = offset from the centerline: + = above, − = below (m).
 *   size = XL | L | M | S | XS  (green) or HAZ (red).
 *   kind = 'green' (fly through, scores) or 'red' (avoid, −10 if entered).
 * Green points come from size automatically (XL/L/M = up to +5, S/XS = up to
 * +10) and are then scaled by how centred the crossing was — see
 * BULLSEYE_FRACTION / RIM_SHARE.
 * Reorder / edit / add / delete rows freely; that route updates directly. The
 * gaps are relative, so they are stretched to fit whatever length the route's
 * start/end coordinates give — a table works at any route length.
 */
interface RingSpec { gap: number; side: number; up: number; size: RingSize; kind: 'green' | 'red' }
/** Where the FIRST ring sits (m from the course start). Independent of the
 *  spawn: moving the spawn back does not move the rings. The first row's gap in
 *  the tables below is ignored — this places it. */
const FIRST_RING_ALONG = 2150;
/** How far (m) BEHIND the course start the aircraft spawns. This is the only
 *  lead-in knob: raise it to give the participant more room to settle before
 *  reaching the start gate; the course itself does not move. */
const SPAWN_BACK = 2500;

/**
 * START GATE — a single wide gate on the centerline, flown through shortly
 * before the first ring. It scores nothing: crossing it is what starts the run
 * for measurement purposes (see `courseStartedAt`), so every participant is
 * timed from the same point in space rather than from the spawn.
 */
/** How far (m) before the FIRST RING the start gate sits. */
const START_GATE_BACK = 900;
/** Half-width and half-height (m) of the start gate's rectangle. It is drawn
 *  as a wide rectangular frame rather than a ring so it reads as a gateway, and
 *  it is deliberately huge: it should be impossible to miss or to have to aim
 *  for. Widening it costs nothing; raising the height lowers its bottom edge
 *  towards the ground, so re-check the terrain after changing it. */
const START_GATE_HALF_W = 520;
const START_GATE_HALF_H = 210;
/** Lift (m) of the gate's centre above the centerline, if the bottom edge needs
 *  to clear rising ground. */
const START_GATE_UP = 60;
/** Route 1's rings — the original course. */
const RINGS_1: RingSpec[] = [
  { gap: 0, side: 38, up: 126, size: 'M', kind: 'green' },
  { gap: 0, side: -211, up: 186, size: 'HAZ', kind: 'red' },
  { gap: 3843, side: -198, up: -120, size: 'S', kind: 'green' },
  { gap: 0, side: 15, up: -60, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -411, up: -60, size: 'HAZ', kind: 'red' },
  { gap: 1815, side: 250, up: 170, size: 'L', kind: 'green' },
  { gap: 2993, side: -410, up: -180, size: 'XL', kind: 'green' },
  { gap: 4066, side: 200, up: 130, size: 'XS', kind: 'green' },
  { gap: 0, side: 391, up: 160, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 9, up: 90, size: 'HAZ', kind: 'red' },
  { gap: 2107, side: -360, up: 60, size: 'S', kind: 'green' },
  { gap: 0, side: -147, up: 80, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -573, up: 120, size: 'HAZ', kind: 'red' },
  { gap: 2174, side: 280, up: 240, size: 'L', kind: 'green' },
  { gap: 3888, side: -440, up: 10, size: 'S', kind: 'green' },
  { gap: 0, side: -227, up: 80, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -653, up: 20, size: 'HAZ', kind: 'red' },
  { gap: 2611, side: 190, up: 210, size: 'L', kind: 'green' },
  { gap: 1643, side: -330, up: -140, size: 'L', kind: 'green' },
  { gap: 3357, side: 250, up: 170, size: 'M', kind: 'green' },
  { gap: 0, side: 1, up: 230, size: 'HAZ', kind: 'red' },
  { gap: 4521, side: -410, up: -230, size: 'XL', kind: 'green' },
  { gap: 2620, side: 200, up: 130, size: 'L', kind: 'green' },
];
/**
 * Route 2's rings: route 1 mirrored left↔right, with the vertical offsets
 * halved — a flatter course that banks the other way. Same 23 stations and the
 * same mix of sizes as route 1, so the maximum score is identical.
 */
const RINGS_2: RingSpec[] = [
  { gap: 0, side: -38, up: 63, size: 'M', kind: 'green' },
  { gap: 0, side: 211, up: 93, size: 'HAZ', kind: 'red' },
  { gap: 3267, side: 198, up: -70, size: 'S', kind: 'green' },
  { gap: 0, side: -15, up: -55, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 411, up: -90, size: 'HAZ', kind: 'red' },
  { gap: 2087, side: -250, up: 85, size: 'L', kind: 'green' },
  { gap: 2544, side: 410, up: -115, size: 'XL', kind: 'green' },
  { gap: 4676, side: -200, up: 65, size: 'XS', kind: 'green' },
  { gap: 0, side: -391, up: 80, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -9, up: 45, size: 'HAZ', kind: 'red' },
  { gap: 1791, side: 360, up: -95, size: 'S', kind: 'green' },
  { gap: 0, side: 147, up: -80, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 573, up: -115, size: 'HAZ', kind: 'red' },
  { gap: 2500, side: -280, up: 120, size: 'L', kind: 'green' },
  { gap: 3305, side: 440, up: -55, size: 'S', kind: 'green' },
  { gap: 0, side: 227, up: -40, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 653, up: -75, size: 'HAZ', kind: 'red' },
  { gap: 3003, side: -190, up: 105, size: 'L', kind: 'green' },
  { gap: 1397, side: 330, up: -70, size: 'L', kind: 'green' },
  { gap: 3861, side: -250, up: 85, size: 'M', kind: 'green' },
  { gap: 0, side: -1, up: 115, size: 'HAZ', kind: 'red' },
  { gap: 5199, side: 410, up: -115, size: 'XL', kind: 'green' },
  { gap: 2227, side: -200, up: 65, size: 'L', kind: 'green' },
];
/**
 * Route 3's rings: the lateral offsets pulled ~30% toward the centerline and
 * the vertical offsets widened ~40% — less weaving, more climbing and diving.
 * Again the same 23 stations and sizes, so all three routes score the same.
 */
const RINGS_3: RingSpec[] = [
  { gap: 0, side: 27, up: 176, size: 'M', kind: 'green' },
  { gap: 0, side: -148, up: 260, size: 'HAZ', kind: 'red' },
  { gap: 4419, side: -139, up: -196, size: 'S', kind: 'green' },
  { gap: 0, side: 10, up: -154, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -288, up: -160, size: 'HAZ', kind: 'red' },
  { gap: 1543, side: 175, up: 238, size: 'L', kind: 'green' },
  { gap: 3442, side: -287, up: -140, size: 'XL', kind: 'green' },
  { gap: 3456, side: 140, up: 182, size: 'XS', kind: 'green' },
  { gap: 0, side: 274, up: 224, size: 'HAZ', kind: 'red' },
  { gap: 0, side: 6, up: 126, size: 'HAZ', kind: 'red' },
  { gap: 2423, side: -252, up: -266, size: 'S', kind: 'green' },
  { gap: 0, side: -103, up: -110, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -401, up: -90, size: 'HAZ', kind: 'red' },
  { gap: 1848, side: 196, up: 336, size: 'L', kind: 'green' },
  { gap: 4471, side: -308, up: -154, size: 'S', kind: 'green' },
  { gap: 0, side: -159, up: -112, size: 'HAZ', kind: 'red' },
  { gap: 0, side: -457, up: -210, size: 'HAZ', kind: 'red' },
  { gap: 2219, side: 133, up: 294, size: 'L', kind: 'green' },
  { gap: 1889, side: -231, up: 30, size: 'L', kind: 'green' },
  { gap: 2853, side: 175, up: 238, size: 'M', kind: 'green' },
  { gap: 0, side: 1, up: 322, size: 'HAZ', kind: 'red' },
  { gap: 3843, side: -287, up: -160, size: 'XL', kind: 'green' },
  { gap: 3013, side: 140, up: 182, size: 'L', kind: 'green' },
];

/**
 * THE ROUTES, three per region. Keys 1 / 2 / 3 switch between them in flight
 * (after the participant has logged in); each switch rebuilds the rings and
 * starts that route's score from zero — the participant never re-enters their
 * details.
 *
 *   start / end = the course centerline's endpoints. `alt` is height above sea
 *     level (m): the centerline runs straight between them, so the pair also
 *     sets how the course climbs. The last ring lands exactly on `end`, and the
 *     altitudes are chosen so the rings clear the terrain in between.
 *   rings = that route's own ring table (RINGS_1 / RINGS_2 / RINGS_3, above).
 *     The tables are SHARED by both regions, so editing one changes that route
 *     in the north and in the south alike.
 */
interface GeoPoint { lat: number; lon: number; alt: number }
interface Route { name: string; start: GeoPoint; end: GeoPoint; rings: RingSpec[] }
interface Region { id: RegionId; label: string; anchor: { lat: number; lon: number }; routes: Route[] }

const REGIONS: Record<RegionId, Region> = {
  north: {
    id: 'north',
    label: 'North',
    anchor: { lat: 33.24206139858175, lon: 35.57500193888974 },
    routes: [
      {
        // North of Kiryat Shmona -> north of Lake Qaraoun. The original course.
        name: 'Route 1',
        start: { lat: 33.24206139858175, lon: 35.57500193888974, alt: 600 },
        end: { lat: 33.74957984835403, lon: 35.62881758358311, alt: 1800 },
        rings: RINGS_1,
      },
      {
        // Starts high on the Hermon's western slopes, runs north up the Bekaa.
        name: 'Route 2',
        start: { lat: 33.30315436941508, lon: 35.71088091141093, alt: 1820 },
        end: { lat: 33.6213440989546, lon: 35.74088402818536, alt: 1800 },
        rings: RINGS_2,
      },
      {
        // Same start as route 1, but ends further east and ~1200 m higher up.
        name: 'Route 3',
        start: { lat: 33.24206139858175, lon: 35.57500193888974, alt: 600 },
        end: { lat: 33.683902024452436, lon: 35.69045560640456, alt: 2300 },
        rings: RINGS_3,
      },
    ],
  },
  south: {
    id: 'south',
    label: 'South',
    anchor: { lat: 29.639129635266595, lon: 34.92709481959459 },
    // All three southern routes leave the SAME point in the Eilat mountains:
    // route 1 runs 25 km north, route 2 25 km east, route 3 25 km west. The
    // start altitudes differ because the ground east of the start is far higher.
    routes: [
      {
        name: 'Route 1',
        start: { lat: 29.639129635266595, lon: 34.92709481959459, alt: 1100 },
        end: { lat: 29.86370742901435, lon: 34.92709481959459, alt: 1200 },
        rings: RINGS_1,
      },
      {
        name: 'Route 2',
        start: { lat: 29.639129635266595, lon: 34.92709481959459, alt: 1500 },
        end: { lat: 29.6391296352666, lon: 35.18548046458278, alt: 1500 },
        rings: RINGS_2,
      },
      {
        name: 'Route 3',
        start: { lat: 29.639129635266595, lon: 34.92709481959459, alt: 1200 },
        end: { lat: 29.6391296352666, lon: 34.66870917460641, alt: 1300 },
        rings: RINGS_3,
      },
    ],
  },
};

/** The region being flown; chosen on the login screen before the run starts. */
let regionId: RegionId = 'north';
const region = (): Region => REGIONS[regionId];
const routes = (): Route[] => REGIONS[regionId].routes;

/** The route currently flown — index into the active region's routes. */
let routeIndex = 0;

const COLOR_TARGET = 0x00e5ff; // bright cyan-blue — fly through these (stands out against the sky)
const COLOR_HAZARD = 0xe23a3a; // red — avoid these (drawn as squares)
const COLOR_START = 0xffc53d; // amber — the start gate, before it is crossed
const COLOR_START_DONE = 0x6b5518; // dimmed amber — start gate already crossed
const COLOR_DONE = 0x24507f; // dimmed blue — a target already flown through
const COLOR_MISS = 0xb23b3b; // a green target that was missed

/**
 * TARGET-BOARD SCORING — a green ring's award depends on HOW CLOSE to its
 * centre you cross, like the rings of a target. Crossing anywhere inside
 * BULLSEYE_FRACTION of the radius scores the ring's full value; from there the
 * award falls off linearly to RIM_SHARE of it right at the rim. So a ring's
 * listed points are its best case, reached only by flying through the middle.
 * Red hazards are unaffected — entering one costs the full HAZARD_POINTS
 * wherever it is crossed.
 */
const BULLSEYE_FRACTION = 0.2; // inner 20% of the radius = a perfect hit
const RIM_SHARE = 0.3; // share of the full value awarded at the very rim

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
const routeValEl = document.getElementById('routeVal')!;
const pauseEl = document.getElementById('pause')!;
const pauseTitleEl = document.getElementById('pauseTitle')!;
const pauseInfoEl = document.getElementById('pauseInfo')!;
const pauseYesEl = document.getElementById('pauseYes') as HTMLButtonElement;
const pauseNoEl = document.getElementById('pauseNo') as HTMLButtonElement;
const finishEl = document.getElementById('finish')!;
const finishInfoEl = document.getElementById('finishInfo')!;
const downloadCsvEl = document.getElementById('downloadCsv') as HTMLButtonElement;
const regionBtns = Array.from(document.querySelectorAll<HTMLButtonElement>('.segBtn'));
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

/** World config for a region: anchored on its own point, with extra detail. */
function worldFor(r: Region): WorldConfig {
  const w = worldFromLatLon(r.anchor.lat, r.anchor.lon);
  w.skirtOverlap = new Array(ZOOM_LEVELS).fill(1.01);
  w.maxZoom = 17; // extra imagery/height detail over the course
  return w;
}

function makeTerrain(w: WorldConfig): Terrain {
  return new Terrain(
    camera,
    {
      world: w,
      rendering: {
        fogColor: SKY,
        ambient: new THREE.Color().setRGB(150 / 255, 150 / 255, 150 / 255, THREE.SRGBColorSpace),
        sunScale: 0.35,
      },
      network: { concurrency: 8 },
    },
    scene,
  );
}

// Re-anchored when the region changes (see `selectRegion`), so these are `let`.
let world = worldFor(region());
let terrain = makeTerrain(world);

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
  isStart: boolean; // the start gate: scores nothing, starts the run's clock
  /** Rectangular rings (the start gate) carry their half-sizes; a crossing is
   *  "through" when it falls inside this box rather than inside `radius`. */
  halfW?: number;
  halfH?: number;
  baseColor: number; // colour when untouched (restored on a reset)
}

/** Group holds every ring; its position tracks the large-world offset. */
const ringGroup = new THREE.Group();
scene.add(ringGroup);

const rings: Ring[] = [];
/**
 * MEASUREMENT of one route attempt. The clock runs from the moment the start
 * gate's plane is crossed until Esc is pressed, and nothing else is timed: the
 * lead-in before the gate, and any time spent in the pause dialog, are excluded.
 *   runStartedAt — when the clock last started ticking (null = not ticking)
 *   runMs        — time already banked from earlier ticking stretches
 *   crashes      — crashes during this attempt
 */
let runStartedAt: number | null = null;
let runMs = 0;
let crashes = 0;
/** Whether the start gate has been crossed on this attempt. */
let runArmed = false;
/** Seconds measured so far on this attempt. */
function courseElapsed(): number {
  return (runMs + (runStartedAt === null ? 0 : performance.now() - runStartedAt)) / 1000;
}
/** Stop the clock, banking the time run so far. */
function holdClock(): void {
  if (runStartedAt !== null) { runMs += performance.now() - runStartedAt; runStartedAt = null; }
}
/** Start/resume the clock (only once the gate has been crossed). */
function resumeClock(): void {
  if (runArmed && runStartedAt === null) runStartedAt = performance.now();
}
/** Clear the whole measurement — a fresh attempt at a route. */
function clearRun(): void {
  runStartedAt = null; runMs = 0; crashes = 0; runArmed = false;
}

/** One finished route attempt — a row in the exported CSV. */
interface RunRow {
  name: string; subject: string; region: RegionId; route: number;
  seconds: number; crashes: number; score: number; rings: number; targets: number;
}
const sessionRows: RunRow[] = [];
/** Total number of green target rings (red hazards are excluded from the tally). */
let totalTargets = 0;
/** Constant course direction; every ring's opening faces along it (nose-on). */
const courseNormal = new THREE.Vector3(0, 0, -1);
/** Absolute centerline start of the course (spawn reference). */
const courseStart = new THREE.Vector3();
/** The course's horizontal "right" axis — used to test rectangular rings. */
const courseRight = new THREE.Vector3(1, 0, 0);

/** A flat square "annulus" (a square frame / square ring) in the XY plane, for
 *  the red hazards — same footprint as a circle of the given half-size. */
function rectAnnulus(outW: number, outH: number, inW: number, inH: number): THREE.ShapeGeometry {
  const shape = new THREE.Shape();
  shape.moveTo(-outW, -outH);
  shape.lineTo(outW, -outH);
  shape.lineTo(outW, outH);
  shape.lineTo(-outW, outH);
  shape.closePath();
  const hole = new THREE.Path();
  hole.moveTo(-inW, -inH);
  hole.lineTo(-inW, inH);
  hole.lineTo(inW, inH);
  hole.lineTo(inW, -inH);
  hole.closePath();
  shape.holes.push(hole);
  return new THREE.ShapeGeometry(shape);
}

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

/** Drop the current route's rings (geometry included) before building another. */
function clearCourse(): void {
  for (const ring of rings) {
    ringGroup.remove(ring.mesh);
    ring.mesh.geometry.dispose();
    (ring.mesh.material as THREE.Material).dispose();
    ring.halo.geometry.dispose();
    (ring.halo.material as THREE.Material).dispose();
  }
  rings.length = 0;
  totalTargets = 0;
}

function buildCourse(route: Route): void {
  clearCourse();
  const a = worldPoint(world, route.start.lat, route.start.lon, route.start.alt);
  const b = worldPoint(world, route.end.lat, route.end.lon, route.end.alt);
  /** Distance (m) from the course start to the LAST station: the straight-line
   *  distance to the route's end, so the course finishes exactly on it. The
   *  gaps below are stretched proportionally to fill it — deleting rows spreads
   *  the remaining rings out, adding rows packs them tighter. Rings sharing a
   *  station (gap 0) stay side by side. */
  const courseEnd = b.distanceTo(a);

  // Every ring faces along the straight centerline (the opening squares up to
  // the aircraft's nose); the per-ring side/up offsets below only shift the
  // CENTER, never the facing — a "crab walk", not a turn.
  courseNormal.copy(b).sub(a).normalize();
  courseStart.copy(a);
  const up = new THREE.Vector3(0, 1, 0);
  const right = new THREE.Vector3(-courseNormal.z, 0, courseNormal.x).normalize();
  courseRight.copy(right);
  const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), courseNormal);
  // Squares need an explicit in-plane orientation (edges level with right/up),
  // which a circle didn't care about.
  const upInPlane = new THREE.Vector3().crossVectors(courseNormal, right).normalize();
  const quatSquare = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(right, upInPlane, courseNormal),
  );

  // Place each ring from the route's table: walk along the centerline by
  // `gap`, then step
  // `side` (right/left) and `up` (above/below) off the line.
  // First station a fixed flight time from the spawn; the remaining gaps are
  // stretched so the last station lands exactly on the route's end.
  const firstAlong = FIRST_RING_ALONG;
  const restGaps = route.rings.slice(1).reduce((sum, spec) => sum + spec.gap, 0);
  const gapScale = (courseEnd - firstAlong) / restGaps;
  let along = 0;
  for (const [i, spec] of route.rings.entries()) {
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
    addRing(center, along, radius, points, hazard, false, color, quat, quatSquare);
    if (!hazard) totalTargets++;
  }

  // The start gate: dead on the centerline (no side/up offset) a fixed distance
  // before the first ring, so it is crossed without manoeuvring.
  const gateAlong = firstAlong - START_GATE_BACK;
  addRing(
    a.clone().addScaledVector(courseNormal, gateAlong).addScaledVector(up, START_GATE_UP),
    gateAlong, Math.min(START_GATE_HALF_W, START_GATE_HALF_H), 0, false, true, COLOR_START,
    quat, quatSquare, { halfW: START_GATE_HALF_W, halfH: START_GATE_HALF_H },
  );
}

/** Build one ring's meshes and register it. Shared by the course and the start gate. */
function addRing(
  center: THREE.Vector3, s: number, radius: number, points: number,
  hazard: boolean, isStart: boolean, color: number,
  quat: THREE.Quaternion, quatSquare: THREE.Quaternion,
  rect?: { halfW: number; halfH: number },
): void {
  const tube = Math.max(6, radius * 0.07);
  // Blue targets = round (torus + ring fill); hazards = square; the start gate
  // = a wide rectangular frame. Squares and rectangles use `quatSquare`, which
  // keeps their edges level with the course's right/up axes.
  let outlineGeo: THREE.BufferGeometry;
  let fillGeo: THREE.BufferGeometry;
  if (rect) {
    const bar = Math.max(14, rect.halfH * 0.07); // frame thickness
    outlineGeo = rectAnnulus(rect.halfW + bar, rect.halfH + bar, rect.halfW - bar, rect.halfH - bar);
    fillGeo = rectAnnulus(rect.halfW * 0.97, rect.halfH * 0.97, rect.halfW * 0.04, rect.halfH * 0.1);
  } else if (hazard) {
    outlineGeo = squareAnnulus(radius + tube, radius - tube);
    fillGeo = squareAnnulus(radius * 0.94, radius * 0.18);
  } else {
    outlineGeo = new THREE.TorusGeometry(radius, tube, 16, 44);
    fillGeo = new THREE.RingGeometry(radius * 0.18, radius * 0.94, 44);
  }
  const mesh = new THREE.Mesh(
    outlineGeo,
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, side: THREE.DoubleSide }),
  );
  mesh.quaternion.copy(hazard || rect ? quatSquare : quat); // opening faces down the course
  mesh.position.copy(center);
  ringGroup.add(mesh);
  const halo = new THREE.Mesh(
    fillGeo,
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, side: THREE.DoubleSide }),
  );
  mesh.add(halo);
  rings.push({
    center, s, radius, points, hazard, mesh, halo, done: false, isStart, baseColor: color,
    halfW: rect?.halfW, halfH: rect?.halfH,
  });
}

function setRingColor(ring: Ring, color: number): void {
  (ring.mesh.material as THREE.MeshBasicMaterial).color.set(color);
  (ring.halo.material as THREE.MeshBasicMaterial).color.set(color);
}

/**
 * What a ring is worth for a crossing `miss` of the way out from its centre
 * (0 = dead centre, 1 = on the rim). Full value inside the bullseye, then a
 * linear fall-off to RIM_SHARE of it; never less than 1 point.
 */
function ringAward(ring: Ring, miss: number): number {
  if (ring.hazard) return ring.points; // a hazard costs the same wherever it is hit
  const t = Math.min(1, Math.max(0, (miss - BULLSEYE_FRACTION) / (1 - BULLSEYE_FRACTION)));
  return Math.max(1, Math.round(ring.points * (1 - (1 - RIM_SHARE) * t)));
}

/** Flew THROUGH a ring: score a green target or penalize a red hazard. */
function passRing(ring: Ring, miss: number): void {
  ring.done = true;
  if (ring.isStart) { startRun(ring); return; }
  const award = ringAward(ring, miss);
  score += award;
  if (ring.hazard) {
    setRingColor(ring, 0x7a2020); // red entered by mistake
    showPop(`${award}`, '#ff6b6b'); // "-10"
  } else {
    setRingColor(ring, COLOR_DONE);
    targetsCleared++;
    showPop(`+${award}`, HUD_GREEN);
  }
  updateScore();
}

/** Crossed a ring's plane outside it. No penalty: a missed green is just marked, a red is safely dodged. */
function skipRing(ring: Ring): void {
  ring.done = true;
  // The start gate is a measurement trigger, not a target: the run begins the
  // moment its plane is crossed, even if the pilot went around the rim.
  if (ring.isStart) { startRun(ring); return; }
  if (!ring.hazard) setRingColor(ring, COLOR_MISS);
}

/** Crossed the start gate — the run (and everything measured about it) begins. */
function startRun(gate: Ring): void {
  runArmed = true;
  runMs = 0;
  runStartedAt = performance.now();
  setRingColor(gate, COLOR_START_DONE);
  showPop('GO', '#ffc53d');
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
  routeValEl.textContent = String(routeIndex + 1);
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
  crashes++;
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
  clearRun(); // a restart is a fresh attempt: clock, crashes and gate all reset
  for (const ring of rings) {
    ring.done = false;
    setRingColor(ring, ring.baseColor);
  }
  updateScore();
  havePrev = false; // drop the stale ring-crossing history after the jump
  resetCamera();
  crashEl.style.display = 'none';
}

/**
 * Switch to another route (keys 1 / 2 / 3). The participant stays signed in:
 * only the rings are rebuilt and the score starts again from zero, so the same
 * pilot can fly all three routes one after another. Pressing the key of the
 * route already being flown does nothing (use R to restart a run).
 */
function switchRoute(index: number): void {
  if (index === routeIndex || index < 0 || index >= routes().length) return;
  routeIndex = index;
  buildCourse(routes()[routeIndex]);
  resetRun(); // zeroes the score, the clock and the crash count for the new route
  showPop(routes()[routeIndex].name, CMP_COLOR_TARGET);
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
  // Esc stops the route; it is handled here rather than in the frame loop
  // because the loop stops stepping as soon as the simulator is paused.
  if (e.code === 'Escape') { e.preventDefault(); if (!paused) pauseRoute(); return; }
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

/** The USB product id a browser reports inside a gamepad's name, e.g. "0404". */
function padProduct(p: Gamepad): string | null {
  return /product:\s*([0-9a-f]{4})/i.exec(p.id)?.[1]?.toLowerCase() ?? null;
}

/** Is this device the throttle? By product id first, then by name. */
function isThrottlePad(p: Gamepad): boolean {
  const product = padProduct(p);
  if (product && GP_THROTTLE_PRODUCTS.includes(product)) return true;
  if (product && GP_STICK_PRODUCTS.includes(product)) return false;
  return p.id.toLowerCase().includes(GP_THROTTLE_ID);
}

/** A HOTAS is two devices on two USB ports — sort them into stick and throttle. */
function warthogPads(): { stick: Gamepad | null; throttle: Gamepad | null } {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let stick: Gamepad | null = null;
  let throttle: Gamepad | null = null;
  for (const p of pads) {
    if (!p) continue;
    if (isThrottlePad(p)) { throttle ??= p; continue; }
    // Prefer the configured stick when more than one is connected.
    if (!stick || padProduct(p) === GP_STICK_PREFERRED) stick = p;
  }
  return { stick, throttle };
}
// Log each device's id + axis count on connect, so the mapping above can be
// checked/adjusted against what this machine actually reports (open the console).
window.addEventListener('gamepadconnected', (e) => {
  const g = e.gamepad;
  const role = isThrottlePad(g) ? 'THROTTLE' : 'STICK';
  console.log(`[gamepad] ${role}: "${g.id}" — ${g.axes.length} axes, ${g.buttons.length} buttons (index ${g.index})`);
});

/**
 * Axis probe. While the login screen is still up, every axis that moves is
 * logged with its device and index — so a new stick can be mapped by waggling
 * it and reading the console, instead of guessing the numbering. It stops the
 * moment the run starts, so it never logs during a flight.
 */
const gpProbe = new Map<string, number[]>();
function probeGamepads(): void {
  if (started) return;
  for (const p of navigator.getGamepads ? navigator.getGamepads() : []) {
    if (!p) continue;
    const last = gpProbe.get(p.id) ?? Array.from(p.axes, () => 0);
    for (let i = 0; i < p.axes.length; i++) {
      if (Math.abs(p.axes[i] - last[i]) > 0.35) {
        const role = isThrottlePad(p) ? 'THROTTLE' : 'STICK';
        console.log(`[gamepad] ${role} "${p.id}" → axis ${i} = ${p.axes[i].toFixed(2)}`);
        last[i] = p.axes[i];
      }
    }
    gpProbe.set(p.id, last);
  }
}

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
/** Is `crossing` inside the ring's opening? Circles use the radius, the
 *  rectangular start gate its own half-sizes. */
const _ringLocal = new THREE.Vector3();
function insideRing(ring: Ring): boolean {
  if (ring.halfW === undefined || ring.halfH === undefined) {
    return crossing.distanceTo(ring.center) <= ring.radius;
  }
  _ringLocal.copy(crossing).sub(ring.center);
  return Math.abs(_ringLocal.dot(courseRight)) <= ring.halfW && Math.abs(_ringLocal.y) <= ring.halfH;
}

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
      if (insideRing(ring)) passRing(ring, crossing.distanceTo(ring.center) / ring.radius);
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
    if (ring.isStart) { // always solid until crossed — it must never be missed
      const op = ring.done ? 0 : 1;
      (ring.mesh.material as THREE.MeshBasicMaterial).opacity = op;
      (ring.halo.material as THREE.MeshBasicMaterial).opacity = op * 0.12;
      continue;
    }
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

// -- pause, route hand-over and the session's data ---------------------------

/** True while Esc has frozen the simulator and the dialog is up. */
let paused = false;
/** True once all three routes are done — the session is over. */
let sessionDone = false;

/**
 * Esc — stop this route. The simulator freezes exactly where it is (nothing is
 * stepped while `paused`), the clock stops, and the participant is asked
 * whether to move on. Answering "No" resumes from the identical state.
 */
function pauseRoute(): void {
  if (!started || paused || sessionDone) return;
  holdClock();
  paused = true;
  const r = routes()[routeIndex];
  pauseTitleEl.textContent = `${r.name} stopped`;
  pauseInfoEl.textContent = runArmed
    ? `Time ${courseElapsed().toFixed(1)} s · score ${score} · rings ${targetsCleared}/${totalTargets} · crashes ${crashes}`
    : 'The start gate has not been crossed yet, so nothing was measured on this route.';
  pauseEl.classList.add('is-open');
}

/** "No" — carry on exactly where the aircraft was frozen. */
function resumeRoute(): void {
  paused = false;
  pauseEl.classList.remove('is-open');
  resumeClock();
  last = performance.now(); // don't bill the paused wall-clock to the physics
  canvas.focus();
}

/** "Yes" — bank this route's row, then move to the next one (or finish). */
function nextRoute(): void {
  pauseEl.classList.remove('is-open');
  paused = false;
  recordRoute();
  if (routeIndex < routes().length - 1) {
    switchRoute(routeIndex + 1);
    last = performance.now();
    canvas.focus();
  } else {
    sessionDone = true;
    finishInfoEl.textContent = sessionRows
      .map((r) => `${REGIONS[r.region].label} route ${r.route}: ${r.seconds.toFixed(1)} s · score ${r.score} · rings ${r.rings}/${r.targets} · crashes ${r.crashes}`)
      .join('\n');
    finishEl.classList.add('is-open');
  }
}

/** Append the finished attempt to the session, and back it up locally. */
function recordRoute(): void {
  sessionRows.push({
    name: participant?.name ?? '',
    subject: participant?.subject ?? '',
    region: regionId,
    route: routeIndex + 1,
    seconds: courseElapsed(),
    crashes,
    score,
    rings: targetsCleared,
    targets: totalTargets,
  });
  // A copy in localStorage, so a refresh or a closed tab cannot lose the data.
  try {
    localStorage.setItem('flightsim.session', JSON.stringify(sessionRows));
  } catch { /* private mode / storage disabled — the in-memory rows still stand */ }
}

const CSV_HEADER = ['name', 'subject_id', 'region', 'route', 'seconds', 'crashes', 'score', 'rings_passed', 'rings_total'];
/** RFC-4180 quoting: wrap in quotes and double any quote inside. */
function csvCell(v: string | number): string {
  const t = String(v);
  return /[",\n]/.test(t) ? `"${t.replace(/"/g, '""')}"` : t;
}
function sessionCsv(): string {
  const rows = sessionRows.map((r) => [
    r.name, r.subject, r.region, r.route, r.seconds.toFixed(2), r.crashes, r.score, r.rings, r.targets,
  ].map(csvCell).join(','));
  return [CSV_HEADER.join(','), ...rows].join('\r\n') + '\r\n';
}

function downloadCsv(): void {
  const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');
  const safe = (participant?.subject || 'participant').replace(/[^\w-]/g, '_');
  const blob = new Blob([sessionCsv()], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `flight-${safe}-${regionId}-${stamp}.csv`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

pauseYesEl.addEventListener('click', nextRoute);
pauseNoEl.addEventListener('click', resumeRoute);
downloadCsvEl.addEventListener('click', downloadCsv);

// -- frame loop --------------------------------------------------------------

const PHYS_DT = 1 / 120; // fixed physics step — decoupled from the render rate
let physAcc = 0;
let last = performance.now();
function frame(now: number): void {
  const dt = Math.min((now - last) / 1000, 0.1);
  last = now;

  if (started && !paused && !sessionDone) {
    // R (any layout — it's the physical key) or a post-crash tap restarts the
    // run from the start of the course; it never returns to the login screen.
    if (justPressed.has('KeyR') || tapReset) resetRun();
    // 1 / 2 / 3 (top row or numpad) swap routes without a new login.
    else if (justPressed.has('Digit1') || justPressed.has('Numpad1')) switchRoute(0);
    else if (justPressed.has('Digit2') || justPressed.has('Numpad2')) switchRoute(1);
    else if (justPressed.has('Digit3') || justPressed.has('Numpad3')) switchRoute(2);
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
  probeGamepads();

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

/**
 * Pick a region on the login screen. The world is re-anchored on the spot and
 * the new terrain starts streaming straight away, so by the time the
 * participant has typed their details the map is usually ready; the Start
 * button keeps showing the progress either way. Only selectable before the run
 * begins — the anchor cannot move once the participant is flying.
 */
function selectRegion(id: RegionId): void {
  if (started || id === regionId) return;
  regionId = id;
  routeIndex = 0;
  for (const b of regionBtns) {
    const on = b.dataset.region === id;
    b.classList.toggle('is-on', on);
    b.setAttribute('aria-checked', String(on));
  }
  // Drop the old region's terrain and re-anchor on the new one.
  clearCourse();
  terrain.dispose();
  world = worldFor(region());
  terrain = makeTerrain(world);
  ringGroup.position.set(0, 0, 0); // the new terrain starts at offset 0
  buildCourse(routes()[routeIndex]);
  resetRun();
}

for (const b of regionBtns) {
  b.addEventListener('click', () => selectRegion(b.dataset.region as RegionId));
}

loginForm.addEventListener('submit', (e) => {
  e.preventDefault();
  if (terrain.status.loading) return; // map not ready yet (button is disabled anyway)
  const name = fullNameEl.value.trim();
  const subject = subjectIdEl.value.trim();
  if (!name) { loginErr.textContent = 'Please enter your full name.'; return; }
  if (!subject) { loginErr.textContent = 'Please enter your participant ID.'; return; }
  loginErr.textContent = '';
  participant = { name, subject };
  sessionRows.length = 0; // a fresh session for this participant
  startSimulation();
});

if ('ontouchstart' in window || navigator.maxTouchPoints > 0) {
  crashEl.textContent = 'You crashed! Tap to reset.';
}

// Anchor the world, build the course, and start rendering immediately so the
// region's terrain is already loading behind the login screen.
buildCourse(routes()[routeIndex]);
resetCamera();
resize();
requestAnimationFrame(frame);

// dev convenience: expose live state for debugging in the console
Object.defineProperty(window, '__sim', {
  configurable: true,
  get: () => ({ terrain, flight, camera, rings, score, participant, courseNormal, courseStart, route: routes()[routeIndex], routeIndex,
    elapsed: courseElapsed(), crashes, runArmed, sessionRows, regionId }),
});
