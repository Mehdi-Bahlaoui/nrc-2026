#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int SERVO_COUNT = 9;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

int servoUs[SERVO_COUNT] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

int usToTicks(int us) {
  return (int)((us * 4096.0) / 20000.0);
}

void writeUs(int ch, int us) {
  if (ch < 0 || ch > 15) return;
  servoUs[ch] = us;
  pca.setPWM(ch, 0, usToTicks(us));
}

float easeInOut(float t) {
  return t * t * (3.0 - 2.0 * t);
}

void applyPose() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    pca.setPWM(i, 0, usToTicks(servoUs[i]));
  }
}

void movePoseSmooth(int targetUs[], int durationMs, int steps) {
  int startUs[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) startUs[i] = servoUs[i];

  for (int s = 1; s <= steps; s++) {
    float k = easeInOut((float)s / (float)steps);
    for (int i = 0; i < SERVO_COUNT; i++) {
      servoUs[i] = startUs[i] + (int)((targetUs[i] - startUs[i]) * k);
    }
    applyPose();
    delay(durationMs / steps);
  }

  for (int i = 0; i < SERVO_COUNT; i++) servoUs[i] = targetUs[i];
  applyPose();
}
