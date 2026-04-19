#pragma once
#include "servo_driver.h"
#include "angleToPwm.h"

// ---------------------------------------------------------------------------
// MehdiWalk parameters — stored in degrees, converted to µs via calibration.
//
//   MW_LEAN  — offset in degrees from heel neutral (MW_LEAN_CENTER)
//   MW_LIFT  — offset in degrees from lift neutral (MW_LIFT_CENTER)
//   MW_FALL  — offset in degrees from fall neutral (MW_FALL_CENTER)
//   MW_TIME  — duration of each sub-phase in ms
//
// Servo roles per leg:
//   Right leg → heel=0 (lean), hip=1 (fall), knee=2 (lift)
//   Left  leg → heel=3 (lean), hip=4 (fall), knee=5 (lift)
// ---------------------------------------------------------------------------
const int MW_LEAN_CENTER = 87;   // neutral angle of heel servos (°)
const int MW_LIFT_CENTER = 91;   // neutral angle of knee servos  (°)
const int MW_FALL_CENTER = 84;   // neutral angle of hip servos   (°)

int MW_LEAN = 15;    // degrees
int MW_LIFT = 25;    // degrees
int MW_FALL = 12;    // degrees
int MW_TIME = 400;   // ms

const int MW_STEPS = 30;

// ---------------------------------------------------------------------------
// Compute a fully-neutral pose for all 6 leg servos using calibrated angles.
// Servos 6-8 stay at CENTER (no calibration table for them).
// ---------------------------------------------------------------------------
static void _mehdiNeutralPose(int t[]) {
  for (int i = 0; i < SERVO_COUNT; i++) t[i] = CENTER;
  t[0] = pulseForServoForAngle(0, MW_LEAN_CENTER);   // right heel
  t[1] = pulseForServoForAngle(1, MW_FALL_CENTER);   // right hip
  t[2] = pulseForServoForAngle(2, MW_LIFT_CENTER);   // right knee
  t[3] = pulseForServoForAngle(3, MW_LEAN_CENTER);   // left heel
  t[4] = pulseForServoForAngle(4, MW_FALL_CENTER);   // left hip
  t[5] = pulseForServoForAngle(5, MW_LIFT_CENTER);   // left knee
}

// ---------------------------------------------------------------------------
// MehdiWalk cycle (two symmetric phases):
//
//   Phase 1 — RIGHT supports, LEFT swings:
//     1a. Left heel (3) leans  → body tilts right
//     1b. Right knee (2) lifts → right leg rises
//     1c. Left hip   (4) falls → propulsion
//     1d. Reset to calibrated neutral
//
//   Phase 2 — LEFT supports (exact mirror):
//     2a. Right heel (0) leans up   → body tilts left
//     2b. Left knee  (5) lifts      → left leg rises
//     2c. Right hip  (1) falls      → propulsion
//     2d. Reset to calibrated neutral
// ---------------------------------------------------------------------------

void mehdiWalkCycle() {

  // ── Phase 1: RIGHT supports ──────────────────────────────────────────────

  // 1a. Left heel leans → body tilts right
  {
    int t[SERVO_COUNT];
    _mehdiNeutralPose(t);
    t[3] = pulseForServoForAngle(3, MW_LEAN_CENTER - MW_LEAN);  // left heel leans down
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 1b. Right knee lifts
  {
    int t[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) t[i] = servoUs[i];   // keep lean from 1a
    t[2] = pulseForServoForAngle(2, MW_LIFT_CENTER + MW_LIFT);  // right knee up
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 1c. Left hip falls → propulsion
  {
    int t[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) t[i] = servoUs[i];   // keep lean + lift
    t[4] = pulseForServoForAngle(4, MW_FALL_CENTER + MW_FALL);  // left hip falls
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 1d. Reset to calibrated neutral
  {
    int t[SERVO_COUNT];
    _mehdiNeutralPose(t);
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // ── Phase 2: LEFT supports (mirror of Phase 1) ───────────────────────────

  // 2a. Right heel leans up → body tilts left
  {
    int t[SERVO_COUNT];
    _mehdiNeutralPose(t);
    t[0] = pulseForServoForAngle(0, MW_LEAN_CENTER + MW_LEAN);  // right heel leans up
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 2b. Left knee lifts (mirrored direction)
  {
    int t[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) t[i] = servoUs[i];   // keep lean from 2a
    t[5] = pulseForServoForAngle(5, MW_LIFT_CENTER - MW_LIFT);  // left knee up (mirrored)
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 2c. Right hip falls (mirrored direction) → propulsion
  {
    int t[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) t[i] = servoUs[i];   // keep lean + lift
    t[1] = pulseForServoForAngle(1, MW_FALL_CENTER - MW_FALL);  // right hip falls (mirrored)
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }

  // 2d. Reset to calibrated neutral
  {
    int t[SERVO_COUNT];
    _mehdiNeutralPose(t);
    movePoseSmooth(t, MW_TIME, MW_STEPS);
  }
}
