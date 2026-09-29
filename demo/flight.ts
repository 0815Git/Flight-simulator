/**
 * Point-mass flight model (F-15/F-16-ish feel, NOT a replica of either).
 *
 * Everything here is in SPECIFIC-FORCE form: every coefficient yields an
 * ACCELERATION (m/s²) = force / mass, so mass cancels and never appears. The
 * numbers below are tuned MODEL ASSUMPTIONS chosen for consistent, teachable
 * behaviour — they are not measured data from a real jet.
 *
 * The model separates three things that the old code conflated:
 *   • pose      — where the airframe points (nose / bank), a RATE command from
 *                 the stick. Releasing the stick stops the rotation; it does NOT
 *                 auto-level. There is no stability augmentation ("assist").
 *   • velocity  — the actual flight path, a free 3-D vector evolved from forces.
 *   • AoA (α)   — the angle between them. Lift comes from α, so the pilot flies
 *                 the nose and the path follows via lift.
 *
 * Forces on the velocity each step:
 *   thrust  = throttle · THRUST_MAX, along the NOSE.
 *   lift    = q · CL, perpendicular to velocity in the pose's up-plane;
 *             CL = clamp(CL_ALPHA · α, ±CL_MAX)  (linear up to stall).
 *   drag    = q · (CD0 + IND_K · CL²), opposing velocity  (parasitic + induced).
 *   weight  = G, straight down.
 * with q = QS · V²  (a specific dynamic pressure).
 *
 * Consequences that make it teachable:
 *   • Each throttle setting + AoA settles at its own speed; climbing trades
 *     speed for height, diving trades back.
 *   • A level turn needs BANK (to point lift sideways) AND back-stick (to raise
 *     α so the vertical lift still equals weight) — and that extra lift raises
 *     induced drag, so a hard turn bleeds speed.
 *   • Accelerating in level flight needs forward-stick (less α) as V rises; the
 *     controls are always coupled.
 *   • Below the stall speed, max CL can no longer supply 1 g, so the aircraft
 *     sinks — no artificial speed floor holds it up.
 */
import * as THREE from 'three';

export const FM = {
  G: 9.81, // gravity (m/s²)
  QS: 1.658e-3, // 0.5·ρ·(S/m): lift/drag accel = QS·V²·C_. Tuned so stall ≈ 65 m/s.
  CL_ALPHA: 4.5, // lift-curve slope (per rad)
  CL_MAX: 1.4, // stall — max |CL|
  CD0: 0.05, // parasitic (zero-lift) drag coefficient
  IND_K: 0.143, // induced-drag factor: CD = CD0 + IND_K·CL² (min-drag speed ≈ 100 m/s)
  THRUST_MAX: 7, // specific thrust at full throttle (thrust/mass, m/s²; T/W ≈ 0.7)
  // Speed limiting. The PHYSICS cap is a compressibility-like drag rise above
  // V_HIGH (a "drag wall") that makes speed asymptote on its own — no energy is
  // removed by hand. VNE is only a last-resort safety clamp (a game protection),
  // and with the drag wall in place it is essentially never reached in flight.
  V_HIGH: 280, // m/s — where the extra transonic-style drag begins
  MACH_K: 0.02, // extra drag accel = MACH_K·(V − V_HIGH)²
  VNE: 300, // hard safety clamp only (NOT a physical effect)
  V_EPS: 1, // divide-by-zero guard for the velocity direction ONLY — never scales forces
  // Control effectiveness: aero controls need airflow, so commanded rates scale
  // with (V/CTRL_V_REF)², clamped to CTRL_MIN so control is reduced — not lost —
  // when slow. This is a simplified control model, not an F-15 control law.
  CTRL_V_REF: 110,
  CTRL_MIN: 0.2,
  // Full-deflection body-axis RATE commands (rad/s) and how fast they're reached.
  ROLL_RATE: 1.2,
  PITCH_RATE: 0.6,
  YAW_RATE: 0.8,
  CONTROL_RESPONSE: 5, // 1/s — commanded rate is approached, not instant
};

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  SPEED RULE — simple, tunable airspeed law (all values in KNOTS).
 * ═══════════════════════════════════════════════════════════════════════════
 * When `enabled`, the airspeed is NOT computed from thrust/drag physics. Each
 * second it changes by:
 *
 *   Δspeed [kt/s] = IDLE_RATE + (FULL_RATE − IDLE_RATE) · throttle
 *                   − PITCH_RATE_AT_90 · (pitch° / 90)
 *
 * both terms linear, then held between MIN_SPEED and MAX_SPEED. Examples with
 * the defaults: level at full → +20, level at idle → −5, straight up at full →
 * 20 − 40 = −20, straight down at idle → −5 + 40 = +35.
 * Lift and gravity still bend the flight path (turns, sinking when slow), so
 * only the SPEED follows this rule. Set `enabled: false` to return to the
 * original physics model (thrust vs. drag, FM below).
 */
export const SPEED_RULE = {
  /** true = use this rule; false = the original thrust/drag physics. */
  enabled: true,

  /** kt/s in level flight at FULL throttle (lever all the way forward).
   *  Higher → the jet accelerates faster at full power. */
  FULL_RATE: 20,

  /** kt/s in level flight at IDLE (lever all the way back). Keep it negative
   *  so idle slows you; more negative → idle slows you harder. */
  IDLE_RATE: -5,

  /** kt/s LOST when the nose points straight up (+90°); gained the same way
   *  straight down (−90°), linear in between (e.g. +45° → half of it).
   *  Higher → climbing costs more speed and diving gives more. 0 = pitch has
   *  no effect on speed. */
  PITCH_RATE_AT_90: 40,

  /** Lowest airspeed (kt) — speed never drops below this. Keep it above the
   *  stall (~126 kt) or the jet will sink while holding this speed. */
  MIN_SPEED: 150,

  /** Highest airspeed (kt) — speed never goes above this. */
  MAX_SPEED: 550,
};

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  BANK TURN — banking alone turns the jet (no back-stick needed).
 * ═══════════════════════════════════════════════════════════════════════════
 * While banked, the nose AND the flight path swing toward the low wing at
 *
 *   turn rate [°/s] = RATE_AT_90 · sin(bank)
 *
 * e.g. 90° → 10°/s, 45° → ~7°/s, 30° → 5°/s, wings level → 0. Pulling the
 * stick adds its own (lift) turn on top, as before. Altitude is NOT held — a
 * steep bank without pulling still sinks.
 */
export const BANK_TURN = {
  /** °/s of turn at a 90° bank. Higher → banking turns faster; 0 = off. */
  RATE_AT_90: 10,
};

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  SIDESLIP — the flight path re-aligns sideways with the nose.
 * ═══════════════════════════════════════════════════════════════════════════
 * Without this, a sideways gap between the W (nose) and the flight path marker
 * (e.g. after yawing with Q/E) never closed. Now the nose weathervanes toward
 * the direction of motion (like a real jet's fin), so the gap closes. The path
 * is not forced, so a banked jet still sinks — its nose just follows it down.
 * Up/down gap (angle of attack) is NOT touched: that one is real and needed.
 */
export const SIDESLIP = {
  /** Seconds for the sideways gap to shrink to ~37% (≈ gone after 3×).
   *  Smaller → snaps back faster; larger → drifts back slower; 0 = off. */
  DECAY_TIME: 0.7,
};

const KT = 0.514444; // 1 knot in m/s

/** The rule's speed change (kt/s) for a throttle 0..1 and nose pitch (deg, + = up). */
export function speedRuleRate(throttle: number, pitchDeg: number): number {
  const r = SPEED_RULE;
  return r.IDLE_RATE + (r.FULL_RATE - r.IDLE_RATE) * throttle - r.PITCH_RATE_AT_90 * (pitchDeg / 90);
}

/** Throttle (0..1) at which the rule holds speed in level flight. */
export function speedRuleTrimThrottle(): number {
  const r = SPEED_RULE;
  return Math.min(1, Math.max(0, -r.IDLE_RATE / (r.FULL_RATE - r.IDLE_RATE)));
}

export interface StepOut {
  speed: number; // |velocity| after the step (m/s)
  alpha: number; // angle of attack (rad)
  nz: number; // load factor (lift / weight), in g
}

const X = new THREE.Vector3(1, 0, 0);
const Y = new THREE.Vector3(0, 1, 0);
const Z = new THREE.Vector3(0, 0, 1);
const _q = new THREE.Quaternion();
const _nose = new THREE.Vector3();
const _up = new THREE.Vector3();
const _velHat = new THREE.Vector3();
const _lift = new THREE.Vector3();
const _acc = new THREE.Vector3();
const _t = new THREE.Vector3();

/** Control effectiveness 0..1 — full when fast, faded (not gone) when slow. */
export function controlAuthority(speed: number): number {
  const f = (speed / FM.CTRL_V_REF) ** 2;
  return Math.max(FM.CTRL_MIN, Math.min(1, f));
}

/**
 * Integrate the commanded body-axis rates into the pose. `target` holds the
 * commanded rates (x = pitch, y = yaw, z = roll, rad/s); they are first scaled
 * by the airspeed-dependent control authority, then `angVel` is eased toward the
 * result so control isn't instant. This is a RATE command (a deliberate
 * simplification of a real control system, NOT an F-15 control law): hold the
 * stick to keep rotating; RELEASE any axis and its target rate is 0, so the
 * aircraft stops rotating on that axis and HOLDS its attitude — it never
 * auto-levels.
 */
export function applyStick(
  pose: THREE.Quaternion,
  angVel: THREE.Vector3,
  target: THREE.Vector3,
  dt: number,
  speed: number,
): void {
  _t.copy(target).multiplyScalar(controlAuthority(speed));
  angVel.lerp(_t, 1 - Math.exp(-FM.CONTROL_RESPONSE * dt));
  pose.multiply(_q.setFromAxisAngle(Z, angVel.z * dt)); // roll (body Z)
  pose.multiply(_q.setFromAxisAngle(Y, angVel.y * dt)); // yaw  (body Y)
  pose.multiply(_q.setFromAxisAngle(X, angVel.x * dt)); // pitch(body X)
  pose.normalize();
}

/** Advance the velocity one step from thrust, lift, drag and gravity (mutates `vel`). */
export function stepDynamics(
  pose: THREE.Quaternion,
  vel: THREE.Vector3,
  throttle: number,
  dt: number,
): StepOut {
  _nose.set(0, 0, -1).applyQuaternion(pose); // camera looks down −Z
  _up.set(0, 1, 0).applyQuaternion(pose);

  // Bank turn: swing nose + path about world-up toward the low wing. Faded
  // out near vertical (cos pitch), where "bank" has no meaning.
  if (BANK_TURN.RATE_AT_90 !== 0) {
    _t.set(1, 0, 0).applyQuaternion(pose); // body right
    const bank = Math.atan2(-_t.y, _up.y); // + = right wing down
    const level = Math.sqrt(Math.max(0, 1 - _nose.y * _nose.y));
    const yaw = -THREE.MathUtils.degToRad(BANK_TURN.RATE_AT_90) * Math.sin(bank) * level * dt;
    _q.setFromAxisAngle(Y, yaw);
    pose.premultiply(_q).normalize();
    vel.applyQuaternion(_q);
    _nose.applyQuaternion(_q);
    _up.applyQuaternion(_q);
  }

  // Weathervane: yaw the nose (about body-up) toward the velocity, so the
  // sideslip decays. The path itself is untouched — a banked jet still sinks,
  // and its nose follows the sink down.
  if (SIDESLIP.DECAY_TIME > 0 && vel.lengthSq() > FM.V_EPS * FM.V_EPS) {
    _t.set(1, 0, 0).applyQuaternion(pose); // body right
    const beta = Math.atan2(vel.dot(_t), vel.dot(_nose)); // + = moving to the right
    _q.setFromAxisAngle(Y, -beta * (1 - Math.exp(-dt / SIDESLIP.DECAY_TIME)));
    pose.multiply(_q).normalize();
    _nose.set(0, 0, -1).applyQuaternion(pose);
    _up.set(0, 1, 0).applyQuaternion(pose);
  }
  const speed = vel.length(); // ACTUAL airspeed drives every force below
  _velHat.copy(vel).multiplyScalar(1 / Math.max(speed, FM.V_EPS)); // guard direction only

  // Signed AoA in the aircraft's longitudinal (pitch) plane: atan2(w, u) with
  // u = forward body velocity (vel·nose) and w = downward body velocity
  // (−vel·up). It keeps its sign and ignores the sideways component (sideslip).
  const alpha = Math.atan2(-vel.dot(_up), vel.dot(_nose));
  const CL = Math.max(-FM.CL_MAX, Math.min(FM.CL_MAX, FM.CL_ALPHA * alpha));
  const q = FM.QS * speed * speed; // → 0 as speed → 0, so V_EPS never adds force
  const liftAcc = q * CL;
  // parasitic + induced drag, plus a compressibility-style rise near VNE
  const dragAcc = q * (FM.CD0 + FM.IND_K * CL * CL) + FM.MACH_K * Math.max(0, speed - FM.V_HIGH) ** 2;

  // Lift acts ⟂ to velocity, along the body-up projected off the velocity — so
  // banking tilts it and its sideways part is what turns the flight path.
  _lift.copy(_up).addScaledVector(_velHat, -_up.dot(_velHat));
  const ll = _lift.length();
  if (ll > 1e-6) _lift.multiplyScalar(1 / ll);
  else _lift.set(0, 0, 0);

  if (SPEED_RULE.enabled) {
    // Lift + weight only bend the path: keep their part ⟂ to the velocity...
    _acc.copy(_lift).multiplyScalar(liftAcc);
    _acc.y -= FM.G;
    _acc.addScaledVector(_velHat, -_acc.dot(_velHat));
    vel.addScaledVector(_acc, dt);
    // ...and the speed follows the rule, from the nose pitch shown on the HUD.
    const pitchDeg = (Math.asin(Math.max(-1, Math.min(1, _nose.y))) * 180) / Math.PI;
    const kt = speed / KT + speedRuleRate(throttle, pitchDeg) * dt;
    const next = Math.min(SPEED_RULE.MAX_SPEED, Math.max(SPEED_RULE.MIN_SPEED, kt)) * KT;
    if (vel.lengthSq() > 1e-9) vel.setLength(next);
    else vel.copy(_nose).multiplyScalar(next);
    return { speed: next, alpha, nz: liftAcc / FM.G };
  }

  _acc.set(0, 0, 0);
  _acc.addScaledVector(_nose, FM.THRUST_MAX * throttle); // thrust along the nose
  _acc.addScaledVector(_lift, liftAcc); // lift
  _acc.addScaledVector(_velHat, -dragAcc); // drag
  _acc.y -= FM.G; // weight

  vel.addScaledVector(_acc, dt);
  // Safety clamp only. The drag rise above V_HIGH is what really limits speed;
  // this just guarantees the number can never run away past VNE.
  const sp = vel.length();
  if (sp > FM.VNE) vel.multiplyScalar(FM.VNE / sp);

  return { speed: vel.length(), alpha, nz: liftAcc / FM.G };
}
