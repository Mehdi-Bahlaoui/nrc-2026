#include <Arduino.h>
#include <SoftwareSerial.h>
#include "input.h"
#include "angleToPwm.h"
#include "servo_driver.h"
#include "gait.h"
#include "mehdi_gait.h"
#include "ultrason.h"

SoftwareSerial btSerial(4, 5); // RX=10, TX=11
const int TrigPin = 7;
const int EchoPin = 6;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  btSerial.begin(9600);

  // pca
  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  // ultrasonic
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // initial pose
  // goStandPose();
}

void applyConfig(const ConfigPacket& cfg) {
  switch (cfg.paramId) {
    case 0x01: STEP       = cfg.value; break;
    case 0x02: LIFT       = cfg.value; break;
    case 0x03: LEAN       = cfg.value; break;
    case 0x04: MOVE_TIME  = cfg.value; break;
    case 0x05: MOVE_STEPS = cfg.value; break;
    case 0x20: MW_LEAN = cfg.value; break;
    case 0x21: MW_LIFT = cfg.value; break;
    case 0x22: MW_FALL = cfg.value; break;
    case 0x23: MW_TIME = cfg.value; break;
    default:
      if (cfg.paramId >= 0x10 && cfg.paramId <= 0x15)
        centerOffsets[cfg.paramId - 0x10] = cfg.value;
      break;
  }
}

void loop() {
  Packet pkt;
  ConfigPacket cfg;
  static bool prevBtn3 = false;
  static bool prevBtn4 = false;

  switch (readAnyPacket(pkt, cfg, btSerial)) {

    case PKT_CONFIG:
      applyConfig(cfg);
      break;

    case PKT_CONTROL:
      // Button 1 — built-in LED indicator
      digitalWrite(LED_BUILTIN, pkt.buttonStates[0] ? HIGH : LOW);

      // Button 3 (index 2) — oneWalkCycle, edge-triggered
      {
        bool btn3 = pkt.buttonStates[2];
        if (btn3 && !prevBtn3) oneWalkCycle();
        prevBtn3 = btn3;
      }

      // Button 4 (index 3) — MehdiWalk cycle, edge-triggered
      {
        bool btn4 = pkt.buttonStates[3];
        if (btn4 && !prevBtn4) mehdiWalkCycle();
        prevBtn4 = btn4;
      }

      // Sliders — servo 1–6 (indices 0–5)
      for (int i = 0; i < SERVO_COUNT; i++) {
        int pulse = pulseForServoForAngle(i, pkt.servoValues[i]);
        writeUs(i, pulse);
      }
      break;

    case PKT_NONE:
      break;
  }

  send_ultrasonic_distance(TrigPin, EchoPin, btSerial);
}
