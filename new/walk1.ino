#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

const int SERVO_COUNT = 6;
const int CALIB_COUNT = 7;

const int calibAngles[SERVO_COUNT][CALIB_COUNT] = {
  {0, 30, 60, 90, 120, 150, 180},
  {0, 30, 60, 90, 120, 150, 180},
  {0, 30, 60, 90, 120, 150, 180},
  {0, 30, 60, 90, 120, 150, 180},
  {0, 30, 60, 90, 120, 150, 180},
  {0, 30, 60, 90, 120, 150, 180}
};

const int calibUs[SERVO_COUNT][CALIB_COUNT] = {
  {375, 601, 893, 1316, 1687, 2007, 2257},
  {366, 595, 882, 1342, 1747, 2059, 2299},
  {366, 665, 972, 1367, 1747, 2023, 2308},
  {366, 605, 942, 1350, 1742, 2018, 2283},
  {366, 620, 937, 1325, 1713, 2018, 2268},
  {366, 620, 897, 1325, 1703, 1998, 2253}
};

float servoDeg[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};

float CENTER_ANGLE = 90.0;

float STEP_ANGLE_LEFT = 20.0;
float STEP_ANGLE_RIGHT = 20.0;

float LIFT_ANGLE_LEFT = 20.0;
float LIFT_ANGLE_RIGHT = 20.0;

float LEAN_ANGLE_RIGHT = 15.0;
float LEAN_ANGLE_LEFT = 10.0;

int MOVE_TIME = 150;
int MOVE_STEPS = 40;

int stepIndex = 0;
bool autoRun = false;

int usToTicks(int us) {
  return (int)((us * 4096.0) / 20000.0);
}

int pulseUsForAngle(int servo, float angleDeg) {
  if (servo < 0 || servo >= SERVO_COUNT) return 1500;

  if (angleDeg <= calibAngles[servo][0]) return calibUs[servo][0];
  if (angleDeg >= calibAngles[servo][CALIB_COUNT - 1]) return calibUs[servo][CALIB_COUNT - 1];

  for (int i = 0; i < CALIB_COUNT - 1; i++) {
    int a0 = calibAngles[servo][i];
    int a1 = calibAngles[servo][i + 1];
    int u0 = calibUs[servo][i];
    int u1 = calibUs[servo][i + 1];

    if (angleDeg >= a0 && angleDeg <= a1) {
      float t = (angleDeg - a0) / float(a1 - a0);
      return (int)(u0 + t * (u1 - u0));
    }
  }

  return calibUs[servo][0];
}

void writeServoAngle(int ch, float angleDeg) {
  if (ch < 0 || ch > 15) return;
  int us = pulseUsForAngle(ch, angleDeg);
  int ticks = usToTicks(us);
  pca.setPWM(ch, 0, ticks);
}

float easeInOut(float t) {
  return t * t * (3.0 - 2.0 * t);
}

void applyPose() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeServoAngle(i, servoDeg[i]);
  }
}

void movePoseSmooth(float targetDeg[], int durationMs, int steps) {
  float startDeg[SERVO_COUNT];

  for (int i = 0; i < SERVO_COUNT; i++) {
    startDeg[i] = servoDeg[i];
  }

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    float k = easeInOut(t);

    for (int i = 0; i < SERVO_COUNT; i++) {
      servoDeg[i] = startDeg[i] + (targetDeg[i] - startDeg[i]) * k;
    }

    applyPose();
    delay(durationMs / steps);
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    servoDeg[i] = targetDeg[i];
  }

  applyPose();
}

void goStandPose() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = CENTER_ANGLE;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void leanRight() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];
  target[0] = CENTER_ANGLE - LEAN_ANGLE_RIGHT;
  target[3] = CENTER_ANGLE - LEAN_ANGLE_RIGHT;
  movePoseSmooth(target, MOVE_TIME * 2, MOVE_STEPS);
}

void leanLeft() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];
  target[0] = CENTER_ANGLE + LEAN_ANGLE_LEFT;
  target[3] = CENTER_ANGLE + LEAN_ANGLE_LEFT;
  movePoseSmooth(target, MOVE_TIME * 2, MOVE_STEPS);
}

void liftLeftLeg() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];
  target[4] = CENTER_ANGLE - (LIFT_ANGLE_LEFT / 2.0);
  target[5] = CENTER_ANGLE - LIFT_ANGLE_LEFT;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void liftRightLeg() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];
  target[1] = CENTER_ANGLE + (LIFT_ANGLE_RIGHT / 2.0);
  target[2] = CENTER_ANGLE + LIFT_ANGLE_RIGHT;
  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void moveLeftLegForwardSyncRightBack() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];

  target[4] = CENTER_ANGLE - STEP_ANGLE_LEFT;
  target[5] = CENTER_ANGLE - STEP_ANGLE_LEFT;

  target[1] = CENTER_ANGLE - STEP_ANGLE_RIGHT * 0.35;
  target[2] = CENTER_ANGLE - STEP_ANGLE_RIGHT * 0.35;

  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void moveRightLegForwardSyncLeftBack() {
  float target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) target[i] = servoDeg[i];

  target[1] = CENTER_ANGLE + STEP_ANGLE_RIGHT;
  target[2] = CENTER_ANGLE + STEP_ANGLE_RIGHT;

  target[4] = CENTER_ANGLE + STEP_ANGLE_LEFT * 0.35;
  target[5] = CENTER_ANGLE + STEP_ANGLE_LEFT * 0.35;

  movePoseSmooth(target, MOVE_TIME, MOVE_STEPS);
}

void printMenu() {
  Serial.println();
  Serial.println("Press key then Enter:");
  Serial.println("n = next debug action");
  Serial.println("g = start auto loop");
  Serial.println("s = stop auto loop");
  Serial.println("r = reset stand pose");
  Serial.println("0 = leanLeft");
  Serial.println("1 = liftRightLeg");
  Serial.println("2 = moveRightLegForwardSyncLeftBack");
  Serial.println("3 = leanRight");
  Serial.println("4 = liftLeftLeg");
  Serial.println("5 = moveLeftLegForwardSyncRightBack");
}

void doNextStep() {
  switch (stepIndex) {
    case 0:
      Serial.println("Step 0: leanLeft");
      leanLeft();
      break;
    case 1:
      Serial.println("Step 1: liftRightLeg");
      liftRightLeg();
      break;
    case 2:
      Serial.println("Step 2: moveRightLegForwardSyncLeftBack");
      moveRightLegForwardSyncLeftBack();
      break;
    case 3:
      Serial.println("Step 3: leanRight");
      leanRight();
      break;
    case 4:
      Serial.println("Step 4: liftLeftLeg");
      liftLeftLeg();
      break;
    case 5:
      Serial.println("Step 5: moveLeftLegForwardSyncRightBack");
      moveLeftLegForwardSyncRightBack();
      break;
  }

  stepIndex++;
  if (stepIndex > 5) stepIndex = 0;
}

void oneLoopCycle() {
  leanLeft();
  liftRightLeg();
  moveRightLegForwardSyncLeftBack();

  leanRight();
  liftLeftLeg();
  moveLeftLegForwardSyncRightBack();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') continue;

    if (c == 'n') {
      autoRun = false;
      doNextStep();
    } else if (c == 'g') {
      autoRun = true;
      Serial.println("Auto loop started");
    } else if (c == 's') {
      autoRun = false;
      Serial.println("Auto loop stopped");
    } else if (c == 'r') {
      autoRun = false;
      Serial.println("Reset to stand pose");
      goStandPose();
      stepIndex = 0;
    } else if (c == '0') {
      autoRun = false;
      Serial.println("leanLeft");
      leanLeft();
    } else if (c == '1') {
      autoRun = false;
      Serial.println("liftRightLeg");
      liftRightLeg();
    } else if (c == '2') {
      autoRun = false;
      Serial.println("moveRightLegForwardSyncLeftBack");
      moveRightLegForwardSyncLeftBack();
    } else if (c == '3') {
      autoRun = false;
      Serial.println("leanRight");
      leanRight();
    } else if (c == '4') {
      autoRun = false;
      Serial.println("liftLeftLeg");
      liftLeftLeg();
    } else if (c == '5') {
      autoRun = false;
      Serial.println("moveLeftLegForwardSyncRightBack");
      moveLeftLegForwardSyncRightBack();
    }

    printMenu();
  }
}

void setup() {
  Serial.begin(115200);
  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  goStandPose();
  delay(1000);

  Serial.println("Debug stepping mode ready.");
  printMenu();
}

void loop() {
  handleSerial();

  if (true) {
    oneLoopCycle();
    handleSerial();
  }
}
