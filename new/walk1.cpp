#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

const int SERVO_COUNT = 6;

int servoUs[SERVO_COUNT] = {
  1500, 1500, 1500, 1500, 1500, 1500
};

int CENTER = 1500;
int STEP = 170;
int LIFT = 120;
int LEAN = 200;

int MOVE_TIME = 300;
int MOVE_STEPS = 40;

int usToTicks(int us) {
  return (int)((us * 4096.0) / 20000.0);
}

void writeUs(int ch, int us) {
  if (ch < 0 || ch > 15) return;
  int ticks = usToTicks(us);
  pca.setPWM(ch, 0, ticks);
}

float easeInOut(float t) {
  return t * t * (3.0 - 2.0 * t);
}

void applyPose() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    writeUs(i, servoUs[i]);
  }
}

void movePoseSmooth(int targetUs[], int durationMs, int steps) {
  int startUs[SERVO_COUNT];

  for (int i = 0; i < SERVO_COUNT; i++) {
    startUs[i] = servoUs[i];
  }

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    float k = easeInOut(t);

    for (int i = 0; i < SERVO_COUNT; i++) {
      servoUs[i] = startUs[i] + (int)((targetUs[i] - startUs[i]) * k);
    }

    applyPose();
    delay(durationMs / steps);
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    servoUs[i] = targetUs[i];
  }
  applyPose();
}

void goStandPose() {
  int target[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) {
    target[i] = CENTER;
  }
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

void setup() {
  Serial.begin(115200);
  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  goStandPose();
  delay(1000);
}

void loop() {
  oneWalkCycle();
}
