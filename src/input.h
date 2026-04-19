#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Config packet (4 bytes):
//   [0xFE][PARAM_ID][VAL_LOW][VAL_HIGH]
//   value = int16, little-endian (VAL_LOW | VAL_HIGH << 8)
//
//   PARAM_ID:
//     0x01 = STEP        0x02 = LIFT        0x03 = LEAN
//     0x04 = MOVE_TIME   0x05 = MOVE_STEPS
//     0x10 = centerOffsets[0]  …  0x15 = centerOffsets[5]
//     0x20 = MW_LEAN   0x21 = MW_LIFT   0x22 = MW_FALL   0x23 = MW_TIME
// ---------------------------------------------------------------------------
//
// Both packet types share a single unified reader (readAnyPacket) so that
// they can coexist on the same Stream without stealing each other's bytes.
// ---------------------------------------------------------------------------

enum PacketType { PKT_NONE, PKT_CONTROL, PKT_CONFIG };

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
struct ConfigPacket {
  uint8_t paramId;
  int16_t value;
};

// Unified reader — handles both 0xFF control packets (12 bytes) and
// 0xFE config packets (4 bytes) from the same stream without dropping bytes.
inline PacketType readAnyPacket(Packet& pkt, ConfigPacket& cfg, Stream& port) {
  static const int CTRL_SIZE = 12;
  static const int CFG_SIZE  = 4;
  static uint8_t buf[12];
  static int     bufIdx   = 0;
  static uint8_t syncByte = 0;

  while (port.available() > 0) {
    uint8_t b = (uint8_t)port.read();

    if (b == 0xFF || b == 0xFE) {
      buf[0]   = b;
      bufIdx   = 1;
      syncByte = b;
      continue;
    }

    if (bufIdx == 0) continue;

    buf[bufIdx++] = b;

    const int expected = (syncByte == 0xFF) ? CTRL_SIZE : CFG_SIZE;

    if (bufIdx == expected) {
      bufIdx = 0;

      if (syncByte == 0xFF) {
        for (int i = 0; i < 9; i++) pkt.servoValues[i] = buf[1 + i];
        uint8_t btnLow  = buf[10];
        uint8_t btnHigh = buf[11];
        for (int i = 0; i < 8; i++) pkt.buttonStates[i] = (btnLow >> i) & 0x01;
        pkt.buttonStates[8] = btnHigh & 0x01;
        return PKT_CONTROL;
      } else {
        cfg.paramId = buf[1];
        cfg.value   = (int16_t)(buf[2] | (buf[3] << 8));
        return PKT_CONFIG;
      }
    }
  }

  return PKT_NONE;
}
