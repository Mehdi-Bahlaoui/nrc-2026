#include <Arduino.h>
#include <SoftwareSerial.h>
#include "input.h"
#include "angleToPwm.h"
#include "servo_driver.h"
#include "gait.h"
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

  // untrason
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // initial pose
  // goStandPose();
}

void loop() {
  Packet pkt;
  static bool prevBtn3 = false;

  if (readPacket(pkt, btSerial)) {

    // Button 1 — built-in LED indicator
    digitalWrite(LED_BUILTIN, pkt.buttonStates[0] ? HIGH : LOW);
    // if (pkt.buttonStates[0]) {
    //   btSerial.println("Button 1 pressed!");
    // }

    // Button 3 (index 2) — run one walk cycle (edge-triggered)
    bool btn3 = pkt.buttonStates[2];
    if (btn3 && !prevBtn3) {
      oneWalkCycle();
    }
    prevBtn3 = btn3;




    

    // Sliders — servo 1–6 (indices 0–5) drive the 6 servos directly
    for (int i = 0; i < SERVO_COUNT; i++) {
      int pulse = pulseForServoForAngle(i, pkt.servoValues[i]);
      writeUs(i, pulse);
    }
  }

  send_ultrasonic_distance(TrigPin, EchoPin, btSerial);

}
