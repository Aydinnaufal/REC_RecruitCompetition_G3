#pragma once
#include "Arduino.h"
extern int servo_deg[2]; extern int servo_count;
class Servo {
 public:
  Servo() { idx = servo_count++ % 2; }
  void setPeriodHertz(int) {}
  int attach(int, int, int) { return 1; }
  void write(int d) { servo_deg[idx] = d; }
 private: int idx;
};
