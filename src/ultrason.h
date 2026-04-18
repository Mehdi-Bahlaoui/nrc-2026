#pragma once
#include <Arduino.h>

inline void send_ultrasonic_distance(int trigPin, int echoPin, Stream& port) {
    static unsigned long lastSend = 0;
    if (millis() - lastSend < 200) return;
    lastSend = millis();

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000);

    float distanceCm = duration * 0.01715;

    port.print("Distance: ");
    port.print(int(distanceCm));
    port.println(" cm");
}
