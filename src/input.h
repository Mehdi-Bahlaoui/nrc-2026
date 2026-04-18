#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Packet format (12 bytes, sent every ~100 ms from the app):
//   [0xFF][S1][S2][S3][S4][S5][S6][S7][S8][S9][BTN_LOW][BTN_HIGH]
//   0xFF      – sync / start-of-frame marker
//   S1–S9     – servo angles, each 0–180
//   BTN_LOW   – buttons 1–8 packed as bits (bit 0 = btn1 … bit 7 = btn8)
//   BTN_HIGH  – button 9 in bit 0
// ---------------------------------------------------------------------------

struct Packet {
  uint8_t servoValues[9];   // index 0–8  →  servo 1–9, range 0–180
  bool    buttonStates[9];  // index 0–8  →  button 1–9, true = pressed
};

// ---------------------------------------------------------------------------
// readPacket — call every loop iteration (non-blocking).
//
// Reads whatever bytes are available on Serial and accumulates them into an
// internal buffer.  Returns true *once* per complete, valid packet; the
// parsed data is written into `pkt`.  Returns false otherwise.
// ---------------------------------------------------------------------------
inline bool readPacket(Packet& pkt, Stream& port) {
  static const int     PACKET_SIZE = 12;
  static const uint8_t SYNC        = 0xFF;
  static uint8_t buf[PACKET_SIZE];
  static int     bufIdx = 0;

  while (port.available() > 0) {
    uint8_t b = (uint8_t)port.read();

    if (b == SYNC) {
      buf[0] = SYNC;
      bufIdx = 1;
      continue;
    }

    if (bufIdx == 0) continue;

    buf[bufIdx++] = b;

    if (bufIdx == PACKET_SIZE) {
      bufIdx = 0;

      for (int i = 0; i < 9; i++) {
        pkt.servoValues[i] = buf[1 + i];
      }
      uint8_t btnLow  = buf[10];
      uint8_t btnHigh = buf[11];
      for (int i = 0; i < 8; i++) {
        pkt.buttonStates[i] = (btnLow >> i) & 0x01;
      }
      pkt.buttonStates[8] = btnHigh & 0x01;

      return true;
    }
  }

  return false;
}
