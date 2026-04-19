#pragma once
#include "servo_driver.h"
#include "angleToPwm.h"

// CENTER in µs — kept for mehdi_gait.h compatibility
const int   CENTER       = 1500;
const float CENTER_ANGLE = 90.0f;

// Gait parameters in degrees (runtime-adjustable via app config packets)
//   0x01 = STEP   0x02 = LIFT   0x03 = LEAN
//   0x04 = MOVE_TIME (ms)       0x05 = MOVE_STEPS
float STEP       = 20.0f;
float LIFT       = 20.0f;
float LEAN       = 15.0f;
int   MOVE_TIME  = 150;
int   MOVE_STEPS = 40;

// ---------------------------------------------------------------------------
// Internal degree state — tracks current servo angles during gait execution.
// Only servos 0-5 (leg servos) are driven by angle; 6-8 stay at CENTER.
// ---------------------------------------------------------------------------
static float gaitDeg[SERVO_COUNT];

static void _gaitApplyPose() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    int us = (i < 6) ? pulseForServoForAngle(i, gaitDeg[i]) : CENTER;
    writeUs(i, us);
  }
}

static void _movePoseSmoothDeg(float targetDeg[], int durationMs, int steps) {
  float startDeg[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) startDeg[i] = gaitDeg[i];

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    float k = easeInOut(t);
    for (int i = 0; i < SERVO_COUNT; i++) {
      gaitDeg[i] = startDeg[i] + (targetDeg[i] - startDeg[i]) * k;
    }
    _gaitApplyPose();
    delay(durationMs / steps);
  }
  for (int i = 0; i < SERVO_COUNT; i++) gaitDeg[i] = targetDeg[i];
  _gaitApplyPose();
}

// ---------------------------------------------------------------------------
// Gait functions (ported from walk1.ino)
//   Servo mapping:
//     Right leg: heel=0, hip=1, knee=2
//     Left  leg: heel=3, hip=4, knee=5
// ---------------------------------------------------------------------------

void goStandPose() {
  for (int i = 0; i < SERVO_COUNT; i++) gaitDeg[i] = CENTER_ANGLE;
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = CENTER_ANGLE;
  _movePoseSmoothDeg(target, MOVE_TIME, MOVE_STEPS);
}

void leanRight() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[0] = CENTER_ANGLE - LEAN;
  target[3] = CENTER_ANGLE - LEAN;
  _movePoseSmoothDeg(target, MOVE_TIME * 2, MOVE_STEPS);
}

void leanLeft() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[0] = CENTER_ANGLE + LEAN;
  target[3] = CENTER_ANGLE + LEAN;
  _movePoseSmoothDeg(target, MOVE_TIME * 2, MOVE_STEPS);
}

void liftLeftLeg() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[4] = CENTER_ANGLE - (LIFT / 2.0f);
  target[5] = CENTER_ANGLE - LIFT;
  _movePoseSmoothDeg(target, MOVE_TIME, MOVE_STEPS);
}

void liftRightLeg() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[1] = CENTER_ANGLE + (LIFT / 2.0f);
  target[2] = CENTER_ANGLE + LIFT;
  _movePoseSmoothDeg(target, MOVE_TIME, MOVE_STEPS);
}

// Swing left leg forward while right leg pushes back 35% (counter-swing)
void moveLeftLegForwardSyncRightBack() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[4] = CENTER_ANGLE - STEP;
  target[5] = CENTER_ANGLE - STEP;
  target[1] = CENTER_ANGLE - STEP * 0.35f;
  target[2] = CENTER_ANGLE - STEP * 0.35f;
  _movePoseSmoothDeg(target, MOVE_TIME, MOVE_STEPS);
}

// Swing right leg forward while left leg pushes back 35% (counter-swing)
void moveRightLegForwardSyncLeftBack() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = gaitDeg[i];
  target[1] = CENTER_ANGLE + STEP;
  target[2] = CENTER_ANGLE + STEP;
  target[4] = CENTER_ANGLE + STEP * 0.35f;
  target[5] = CENTER_ANGLE + STEP * 0.35f;
  _movePoseSmoothDeg(target, MOVE_TIME, MOVE_STEPS);
}

// One complete walk cycle (Button 3)
void oneWalkCycle() {
  // goStandPose();
  leanLeft();
  liftRightLeg();
  moveRightLegForwardSyncLeftBack();
  leanRight();
  liftLeftLeg();
  moveLeftLegForwardSyncRightBack();
}
