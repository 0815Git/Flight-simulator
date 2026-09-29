/**
 * Flight-model scenarios. These drive the SAME functions the simulator calls
 * (applyStick + stepDynamics from demo/flight.ts) through a harness that mirrors
 * the sim's loop exactly: inputs are sampled once per RENDER frame, and the
 * physics is advanced in fixed PHYS-second sub-steps via an accumulator. So a
 * passing test reflects real in-sim behaviour, at any frame rate.
 */
import { describe, it, expect, beforeEach } from 'vitest';
import * as THREE from 'three';
import { BANK_TURN, FM, SIDESLIP, SPEED_RULE, applyStick, speedRuleRate, stepDynamics } from '../demo/flight';

const PHYS = 1 / 120; // must match PHYS_DT in main.ts

interface AC {
  pose: THREE.Quaternion;
  vel: THREE.Vector3;
  angVel: THREE.Vector3;
  pos: THREE.Vector3;
  throttle: number;
}
interface Input { roll?: number; pitch?: number; yaw?: number; throttle?: number }

/** Trimmed: nose pitched up by the AoA that gives 1 g at `speed`, velocity level. */
function makeAC(speed = 160, throttle = 0.4): AC {
  const clTrim = Math.min(FM.CL_MAX, FM.G / (FM.QS * speed * speed));
  const alphaTrim = clTrim / FM.CL_ALPHA;
  const pose = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), alphaTrim);
  return { pose, vel: new THREE.Vector3(0, 0, -speed), angVel: new THREE.Vector3(), pos: new THREE.Vector3(), throttle };
}

/** One fixed physics sub-step of size h (mirrors main.ts physicsStep). */
function sub(ac: AC, inp: Input, h: number) {
  const target = new THREE.Vector3(
    (inp.pitch ?? 0) * FM.PITCH_RATE,
    (inp.yaw ?? 0) * FM.YAW_RATE,
    (inp.roll ?? 0) * FM.ROLL_RATE,
  );
  if (inp.throttle !== undefined) ac.throttle = inp.throttle;
  applyStick(ac.pose, ac.angVel, target, h, ac.vel.length());
  const out = stepDynamics(ac.pose, ac.vel, ac.throttle, h);
  ac.pos.addScaledVector(ac.vel, h);
  return out;
}

/** Render loop: sample inputs each frame at `fps`, step physics at fixed PHYS. */
function fly(ac: AC, inpFn: (t: number, ac: AC) => Input, secs: number, fps = 120) {
  const rdt = 1 / fps;
  const frames = Math.round(secs * fps);
  let acc = 0;
  let out = { speed: ac.vel.length(), alpha: 0, nz: 1 };
  for (let f = 0; f < frames; f++) {
    const inp = inpFn(f * rdt, ac); // inputs sampled once per render frame
    acc = Math.min(acc + rdt, 0.25);
    while (acc >= PHYS) { out = sub(ac, inp, PHYS); acc -= PHYS; }
  }
  return out;
}

const speed = (ac: AC) => ac.vel.length();
const heading = (ac: AC) => (Math.atan2(ac.vel.x, -ac.vel.z) * 180 / Math.PI + 360) % 360;
const bankDeg = (ac: AC) => {
  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(ac.pose);
  const up = new THREE.Vector3(0, 1, 0).applyQuaternion(ac.pose);
  return THREE.MathUtils.radToDeg(Math.atan2(-right.y, up.y));
};
/** Competent pilot holding altitude h0; the lead term damps the slow phugoid. */
const altHold = (h0: number) => (_t: number, ac: AC): Input => {
  const prev = (ac as AC & { _vy?: number })._vy;
  const ay = prev === undefined ? 0 : (ac.vel.y - prev) / PHYS;
  (ac as AC & { _vy?: number })._vy = ac.vel.y;
  return { pitch: Math.max(-1, Math.min(1, -0.03 * (ac.pos.y - h0) - 0.6 * ac.vel.y - 3 * ay)) };
};

// Scenarios 1–8 exercise the original thrust/drag physics.
beforeEach(() => { SPEED_RULE.enabled = false; });

describe('1. level flight: throttle sets speed, pitch holds altitude', () => {
  it('more throttle → faster, less → slower, altitude held throughout', () => {
    const ac = makeAC(160, 0.4);
    fly(ac, altHold(0), 20);
    const vTrim = speed(ac);
    fly(ac, (t, a) => ({ ...altHold(0)(t, a), throttle: 0.9 }), 40);
    const vFast = speed(ac);
    expect(vFast).toBeGreaterThan(vTrim + 30);
    expect(Math.abs(ac.pos.y)).toBeLessThan(80);
    fly(ac, (t, a) => ({ ...altHold(0)(t, a), throttle: 0.4 }), 40);
    expect(speed(ac)).toBeLessThan(vFast - 30);
    expect(Math.abs(ac.pos.y)).toBeLessThan(80);
  });
});

describe('2. same throttle: climb trades speed for height, dive the reverse', () => {
  it('nose up → gains altitude, loses speed; nose down → opposite', () => {
    const climb = makeAC(180, 0.5);
    fly(climb, () => ({ pitch: 0.5 }), 6);
    expect(climb.pos.y).toBeGreaterThan(60);
    expect(speed(climb)).toBeLessThan(180);

    const dive = makeAC(180, 0.5);
    fly(dive, () => ({ pitch: -0.5 }), 6);
    expect(dive.pos.y).toBeLessThan(-60);
    expect(speed(dive)).toBeGreaterThan(180);
  });
});

describe('3. roll in, release stick (no auto-level), turn, roll out', () => {
  it('a released bank persists and keeps turning; it does not self-level', () => {
    const ac = makeAC(200, 0.6);
    fly(ac, () => ({ roll: 1 }), 0.8);
    const bankIn = bankDeg(ac);
    expect(Math.abs(bankIn)).toBeGreaterThan(25);

    const hdg0 = heading(ac);
    fly(ac, altHold(ac.pos.y), 3); // roll released
    expect(Math.abs(bankDeg(ac))).toBeGreaterThan(25); // still banked
    expect(Math.abs(heading(ac) - hdg0)).toBeGreaterThan(10); // kept turning

    fly(ac, () => ({ roll: -1 }), 0.9);
    expect(Math.abs(bankDeg(ac))).toBeLessThan(14); // wings ~level
    const hdgA = heading(ac);
    fly(ac, altHold(ac.pos.y), 2);
    expect(Math.abs(heading(ac) - hdgA)).toBeLessThan(8);
  });
});

describe('4. level turn needs back-pressure; pulling in a turn costs speed', () => {
  it('bank-only descends & keeps energy; bank+pull holds altitude & bleeds speed', () => {
    const noPull = makeAC(220, 0.6);
    fly(noPull, () => ({ roll: 1 }), 0.8);
    fly(noPull, () => ({}), 5);

    const withPull = makeAC(220, 0.6);
    fly(withPull, () => ({ roll: 1 }), 0.8);
    fly(withPull, altHold(0), 5);

    expect(noPull.pos.y).toBeLessThan(-40);
    expect(withPull.pos.y).toBeGreaterThan(noPull.pos.y + 40);
    expect(speed(withPull)).toBeLessThan(speed(noPull));
  });
});

describe('5. a high-drag maneuver bleeds speed at the same throttle', () => {
  it('a sustained hard pull ends slower than coasting at the same power', () => {
    const hard = makeAC(240, 0.6);
    fly(hard, () => ({ pitch: 1 }), 4);
    const coast = makeAC(240, 0.6);
    fly(coast, () => ({ pitch: 0 }), 4);
    expect(speed(hard)).toBeLessThan(speed(coast) - 15);
  });
});

describe('6. idle / low speed: no artificial speed maintenance', () => {
  it('idle level attitude cannot hold 1 g — it sinks (lift < weight)', () => {
    const ac = makeAC(160, 0);
    let sank = false;
    let minNz = Infinity;
    for (let i = 0; i < 120 * 25; i++) {
      const out = sub(ac, {}, PHYS);
      minNz = Math.min(minNz, out.nz);
      if (ac.vel.y < -3) sank = true;
    }
    expect(sank).toBe(true);
    expect(minNz).toBeLessThan(1);
    expect(ac.pos.y).toBeLessThan(-50);
  });

  it('coasting steeply nose-high at idle bleeds speed below the old 50 m/s floor', () => {
    const pose = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(80));
    const nose = new THREE.Vector3(0, 0, -1).applyQuaternion(pose);
    const ac: AC = { pose, vel: nose.multiplyScalar(120), angVel: new THREE.Vector3(), pos: new THREE.Vector3(), throttle: 0 };
    let minSpeed = Infinity;
    for (let i = 0; i < 120 * 12; i++) {
      sub(ac, {}, PHYS);
      minSpeed = Math.min(minSpeed, speed(ac));
    }
    expect(minSpeed).toBeLessThan(50);
  });
});

describe('7. low-speed control authority', () => {
  it('the same roll input rolls the jet far less when slow than when fast', () => {
    const fast = makeAC(220, 0.7);
    fly(fast, () => ({ roll: 1 }), 1);
    const bankFast = Math.abs(bankDeg(fast));

    // A genuinely slow airframe (below CTRL_V_REF), same 1 s of full roll input.
    const slow = makeAC(45, 0);
    fly(slow, () => ({ roll: 1 }), 1);
    const bankSlow = Math.abs(bankDeg(slow));

    expect(bankFast).toBeGreaterThan(30); // authority at speed
    expect(bankSlow).toBeLessThan(bankFast * 0.6); // markedly weaker when slow
    expect(bankSlow).toBeGreaterThan(2); // reduced, not fully lost (CTRL_MIN)
  });
});

describe('8. frame-rate independence (fixed physics step)', () => {
  it('30, 60 and 240 fps land in nearly the same state for one timed maneuver', () => {
    const maneuver = (t: number): Input => ({ roll: t < 1 ? 1 : 0, pitch: 0.3, throttle: 0.6 });
    const run = (fps: number) => { const ac = makeAC(180, 0.6); fly(ac, maneuver, 8, fps); return ac; };
    const a = run(30);
    const b = run(60);
    const c = run(240);
    const near = (x: number, y: number) => Math.abs(x - y);
    // reference = 240 fps
    for (const r of [a, b]) {
      expect(near(speed(r), speed(c))).toBeLessThan(speed(c) * 0.02); // < 2 %
      expect(near(r.pos.y, c.pos.y)).toBeLessThan(30); // < 30 m over 8 s
      expect(near(heading(r), heading(c))).toBeLessThan(4); // < 4°
    }
  });
});

describe('9. speed rule (SPEED_RULE.enabled)', () => {
  const KT = 0.514444;
  const kt = (ac: AC) => speed(ac) / KT;
  const pitched = (deg: number, knots: number, throttle: number): AC => {
    const pose = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(deg));
    const vel = new THREE.Vector3(0, 0, -1).applyQuaternion(pose).multiplyScalar(knots * KT);
    return { pose, vel, angVel: new THREE.Vector3(), pos: new THREE.Vector3(), throttle };
  };
  beforeEach(() => { SPEED_RULE.enabled = true; });

  it('rate is linear in throttle and pitch', () => {
    expect(speedRuleRate(1, 0)).toBeCloseTo(20);
    expect(speedRuleRate(0, 0)).toBeCloseTo(-5);
    expect(speedRuleRate(0.5, 0)).toBeCloseTo(7.5);
    expect(speedRuleRate(1, 90)).toBeCloseTo(-20);
    expect(speedRuleRate(0, -90)).toBeCloseTo(35);
    expect(speedRuleRate(0.2, 45)).toBeCloseTo(-20);
  });

  it('level full throttle gains ~20 kt/s, idle loses ~5 kt/s', () => {
    const full = pitched(0, 300, 1);
    fly(full, () => ({}), 2);
    expect(kt(full)).toBeCloseTo(340, 0);
    const idle = pitched(0, 300, 0);
    fly(idle, () => ({}), 2);
    expect(kt(idle)).toBeCloseTo(290, 0);
  });

  it('speed is held between MIN_SPEED and MAX_SPEED', () => {
    const fast = pitched(0, 540, 1);
    fly(fast, () => ({}), 5);
    expect(kt(fast)).toBeCloseTo(SPEED_RULE.MAX_SPEED, 3);
    const slow = pitched(0, 160, 0);
    fly(slow, () => ({}), 5);
    expect(kt(slow)).toBeCloseTo(SPEED_RULE.MIN_SPEED, 3);
  });
});

describe('10. bank turn: banking alone turns the jet', () => {
  const noseHdg = (ac: AC) => {
    const n = new THREE.Vector3(0, 0, -1).applyQuaternion(ac.pose);
    return Math.atan2(n.x, -n.z) * 180 / Math.PI;
  };
  beforeEach(() => { SPEED_RULE.enabled = true; });

  it('90° bank, no pull → ~RATE_AT_90 °/s to the low wing, nose and path together, and it sinks', () => {
    const ac = makeAC(160, 0.2);
    fly(ac, () => ({ roll: -1 }), 1.45); // roll right to ~90°
    expect(bankDeg(ac)).toBeGreaterThan(80);
    expect(bankDeg(ac)).toBeLessThan(100);
    const n0 = noseHdg(ac), p0 = heading(ac), y0 = ac.pos.y;
    fly(ac, () => ({}), 3);
    expect(noseHdg(ac) - n0).toBeGreaterThan(BANK_TURN.RATE_AT_90 * 3 * 0.85);
    expect(noseHdg(ac) - n0).toBeLessThan(BANK_TURN.RATE_AT_90 * 3 * 1.15);
    expect(heading(ac) - p0).toBeGreaterThan(BANK_TURN.RATE_AT_90 * 3 * 0.85);
    expect(ac.pos.y).toBeLessThan(y0 - 20);
  });

  it('wings level → no bank turn', () => {
    const ac = makeAC(160, 0.2);
    fly(ac, altHold(0), 3);
    expect(Math.abs(heading(ac))).toBeLessThan(0.5);
  });
});

describe('11. sideslip: the flight path re-aligns sideways with the nose', () => {
  const slipDeg = (ac: AC) => {
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(ac.pose);
    const nose = new THREE.Vector3(0, 0, -1).applyQuaternion(ac.pose);
    return Math.atan2(ac.vel.dot(right), ac.vel.dot(nose)) * 180 / Math.PI;
  };
  const yawThenRelease = () => {
    const ac = makeAC(160, 0.2);
    fly(ac, (t, a) => ({ ...altHold(0)(t, a), yaw: 1 }), 1);
    const peak = Math.abs(slipDeg(ac));
    fly(ac, altHold(0), 3);
    return { peak, after: Math.abs(slipDeg(ac)) };
  };
  beforeEach(() => { SPEED_RULE.enabled = true; });

  it('after yawing, the sideways gap closes within a few DECAY_TIMEs', () => {
    const { peak, after } = yawThenRelease();
    expect(peak).toBeGreaterThan(3);
    expect(after).toBeLessThan(0.5);
  });

  it('with DECAY_TIME = 0 (the old behaviour) the gap stays open', () => {
    const saved = SIDESLIP.DECAY_TIME;
    SIDESLIP.DECAY_TIME = 0;
    try {
      const { peak, after } = yawThenRelease();
      expect(after).toBeGreaterThan(peak * 0.8);
    } finally { SIDESLIP.DECAY_TIME = saved; }
  });
});
