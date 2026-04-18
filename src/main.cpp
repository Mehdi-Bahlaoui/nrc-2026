#include <Arduino.h>
#include <SoftwareSerial.h>
#include "packet.h"
#include "angleToPwm.h"

SoftwareSerial btSerial(10, 11); // RX=10, TX=11

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);      // PC monitor only
  btSerial.begin(9600);    // HC-05
}

void loop() {
  Packet pkt;

  if (readPacket(pkt, btSerial)) {

    if (pkt.buttonStates[0]) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }

    if (pkt.buttonStates[1]) {

      btSerial.println("Received button click");


      for (int i = 0; i < 6; i++) {
        int pulse = pulseForServoForAngle(i, pkt.servoValues[i]);
        Serial.print("Servo ");
        Serial.print(i + 1);
        Serial.print(": angle=");
        Serial.print(pkt.servoValues[i]);
        Serial.print(" → pulse=");
        Serial.println(pulse);
      }
    }
  }
}
