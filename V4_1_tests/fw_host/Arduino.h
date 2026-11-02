// Host-side stand-in for the Arduino-ESP32 core: just enough to compile esp32_firmware_V_4_1.ino on a PC and run it against the Python link code.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <unistd.h>
#include <chrono>
typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define constrain(a, lo, hi) ((a) < (lo) ? (lo) : ((a) > (hi) ? (hi) : (a)))
inline long map(long x, long in_min, long in_max, long out_min, long out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
inline uint32_t millis() { using namespace std::chrono; static auto t0 = steady_clock::now(); return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count() + 1; }
inline uint32_t micros() { using namespace std::chrono; static auto t0 = steady_clock::now(); return (uint32_t)duration_cast<microseconds>(steady_clock::now() - t0).count(); }
extern int pin_val[64];
extern int fake_gz_raw;
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { pin_val[p] = v; }
inline void analogWrite(int p, int v) { pin_val[p] = v; }
inline uint32_t analogReadMilliVolts(int) { return 2560; }   // x 147/47 = about 8.0 V
class Print {
 public:
  virtual size_t write(const uint8_t* b, size_t n) = 0;
  size_t write(uint8_t c) { return write(&c, 1); }
  size_t print(const char* s) { return write((const uint8_t*)s, strlen(s)); }
  size_t println(const char* s) { size_t n = print(s); return n + write((const uint8_t*)"\n", 1); }
  size_t printf(const char* fmt, ...) { char b[300]; va_list a; va_start(a, fmt); int n = vsnprintf(b, sizeof b, fmt, a); va_end(a); return write((const uint8_t*)b, n > 0 ? n : 0); }
};
class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
};
class HardwareSerial : public Stream {
 public:
  void begin(unsigned long) {}
  size_t write(const uint8_t* b, size_t n) override { fwrite(b, 1, n, stdout); fflush(stdout); return n; }
  int available() override { return 0; }
  int read() override { return -1; }
};
extern HardwareSerial Serial;
