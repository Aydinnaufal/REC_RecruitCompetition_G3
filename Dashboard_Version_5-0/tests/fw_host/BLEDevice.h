// Host-side stand-in for the ESP32 BLE Arduino library: only the calls esp32_firmware_V_5_0.ino makes, with the API shape it assumes.
// The BLE "radio" is fd 3 (a pty). The harness calls ble_stub_poll() every loop: it reads what the laptop wrote (20 bytes at a time, like MTU 23),
// reports a connection on the first byte, and hands the bytes to the sketch's onWrite() callback exactly as the BLE stack would.
#pragma once
#include "Arduino.h"
#include <string>
#include <fcntl.h>
class BLEDescriptor {};
class BLE2902 : public BLEDescriptor {};
class BLECharacteristic;
class BLECharacteristicCallbacks { public: virtual ~BLECharacteristicCallbacks() {} virtual void onWrite(BLECharacteristic*) {} };
class BLECharacteristic {
 public:
  enum { PROPERTY_NOTIFY = 1, PROPERTY_WRITE = 2, PROPERTY_WRITE_NR = 4 };
  void setValue(uint8_t* d, size_t n) { v.assign((const char*)d, n); }
  void notify() { if (::write(3, v.data(), v.size()) < 0) {} notifies++; }
  void addDescriptor(BLEDescriptor*) {}
  void setCallbacks(BLECharacteristicCallbacks* c) { cb = c; }
  std::string getValue() { return v; }
  std::string v; BLECharacteristicCallbacks* cb = nullptr; int notifies = 0;
};
class BLEService {
 public:
  BLECharacteristic* createCharacteristic(const char*, int) { chars[n] = new BLECharacteristic(); return chars[n++]; }
  void start() {}
  BLECharacteristic* chars[4]; int n = 0;
};
class BLEServer;
class BLEServerCallbacks { public: virtual ~BLEServerCallbacks() {} virtual void onConnect(BLEServer*) {} virtual void onDisconnect(BLEServer*) {} };
class BLEServer {
 public:
  void setCallbacks(BLEServerCallbacks* c) { cb = c; }
  BLEService* createService(const char*) { svc = new BLEService(); return svc; }
  BLEServerCallbacks* cb = nullptr; BLEService* svc = nullptr;
};
class BLEAdvertising { public: void addServiceUUID(const char*) {} void setScanResponse(bool) {} };
extern BLEServer* g_ble_server; extern int g_ble_adv_started; extern bool g_ble_connected; extern bool g_ble_dead;
class BLEDevice {
 public:
  static void init(const char*) {}
  static void setMTU(int) {}
  static BLEServer* createServer() { g_ble_server = new BLEServer(); return g_ble_server; }
  static BLEAdvertising* getAdvertising() { static BLEAdvertising a; return &a; }
  static void startAdvertising() { g_ble_adv_started++; }
};
inline void ble_stub_poll() {
  if (!g_ble_server || !g_ble_server->svc) return;
  static bool nb = false; if (!nb) { fcntl(3, F_SETFL, fcntl(3, F_GETFL) | O_NONBLOCK); nb = true; }
  uint8_t b[20]; ssize_t r = ::read(3, b, sizeof b);
  if (r > 0 && g_ble_dead) return;                          // the link is down: nothing gets through until the test says "bleup"
  if (r > 0) {
    if (!g_ble_connected) { g_ble_connected = true; if (g_ble_server->cb) g_ble_server->cb->onConnect(g_ble_server); }
    BLECharacteristic* rx = g_ble_server->svc->chars[1];
    rx->v.assign((const char*)b, r);
    if (rx->cb) rx->cb->onWrite(rx);
  }
}
inline void ble_stub_drop() { g_ble_dead = true; if (g_ble_connected) { g_ble_connected = false; if (g_ble_server && g_ble_server->cb) g_ble_server->cb->onDisconnect(g_ble_server); } }
