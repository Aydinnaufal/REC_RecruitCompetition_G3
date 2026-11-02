#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  void begin(int, int) {}
  void setClock(unsigned long) {}
  void beginTransmission(int) {}
  size_t write(uint8_t) { return 1; }
  int endTransmission(bool = true) { return 0; }
  int requestFrom(int, int n) { idx = 0; return n; }
  int read() { int16_t v = (int16_t)fake_gz_raw; return idx++ == 0 ? ((v >> 8) & 0xFF) : (v & 0xFF); }
 private: int idx = 0;
};
extern TwoWire Wire;
