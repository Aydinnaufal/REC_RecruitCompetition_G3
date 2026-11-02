#pragma once
#include "Arduino.h"
#include <fcntl.h>
class BluetoothSerial : public Stream {   // the "Bluetooth link" is file descriptor 3 = one end of a pty; Python opens the other end like a COM port
 public:
  bool begin(const char*) { fcntl(3, F_SETFL, fcntl(3, F_GETFL) | O_NONBLOCK); return true; }
  bool hasClient() { return true; }
  size_t write(const uint8_t* b, size_t n) override { ssize_t r = ::write(3, b, n); return r < 0 ? 0 : (size_t)r; }
  int available() override { peek_fill(); return have; }
  int read() override { peek_fill(); if (!have) return -1; have = 0; return ch; }
 private:
  int have = 0; unsigned char ch = 0;
  void peek_fill() { if (!have) { ssize_t r = ::read(3, &ch, 1); have = r == 1 ? 1 : 0; } }
};
