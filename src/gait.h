#pragma once
#include "servo_driver.h"

const int CENTER    = 1500;
const int STEP      = 170;
const int LIFT      = 120;
const int LEAN      = 200;

const int MOVE_TIME = 300;
const int MOVE_STEPS = 40;

void goStandPose() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void leanRight() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[0] = CENTER;
  target[3] = CENTER - LEAN;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void leanLeft() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[0] = CENTER + LEAN;
  target[3] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void backToCenterFromRightLean() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[3] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void backToCenterFromLeftLean() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[0] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void liftLeftLeg() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[4] = CENTER + (LIFT / 2);
  target[5] = CENTER + LIFT;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void moveLeftLegForward() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[4] = CENTER - STEP;
  target[5] = CENTER - STEP;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void lowerLeftLeg() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[4] = CENTER;
  target[5] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void liftRightLeg() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[1] = CENTER - (LIFT / 2);
  target[2] = CENTER - LIFT;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void moveRightLegForward() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[1] = CENTER + STEP;
  target[2] = CENTER + STEP;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void lowerRightLeg() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoUs[i];
  target[1] = CENTER;
  target[2] = CENTER;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void oneWalkCycle() {
  leanRight();
  liftLeftLeg();
  moveLeftLegForward();
  backToCenterFromRightLean();
  lowerLeftLeg();

  leanLeft();
  liftRightLeg();
  moveRightLegForward();
  backToCenterFromLeftLean();
  lowerRightLeg();
}
