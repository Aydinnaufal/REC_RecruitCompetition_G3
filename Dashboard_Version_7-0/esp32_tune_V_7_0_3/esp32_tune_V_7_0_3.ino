// REC robot - WIRELESS TEST + TUNING firmware (V_7_0_3) for Robot Station V 7.0 (Tune page). Motors, 2 servos, MPU6050 gyro/accel, battery.
// Board "ESP32 Dev Module" (core 3.x). No extra library: servos and motors both use the ESP32's own PWM (LEDC) on separate channels. Serial Monitor 115200 / Newline. Link: Bluetooth LE ("REC-Robot", no pairing) AND the USB cable.
// Pins: L298N IN1 26 IN2 27 | IN3 32 IN4 33 (ENA/ENB jumpers stay ON, speed = PWM on the IN pins) | servo1 18, servo2 19 | MPU6050 SDA 21 SCL 22 | battery 34
// Protocol rec1 ($TYPE,f,f*HH): see comms_V_7_0_3.py. Nothing moves until the dashboard ARMS the output; no command for 400 ms or a lost link = stop.
// Tuning values (minpwm, invert, servo limits, battery ratio) are changed from the dashboard and kept in flash. NOT TESTED on the real robot: wheels in the air first.
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const char* BT_NAME = "REC-Robot";
const int IN1 = 26, IN2 = 27, IN3 = 32, IN4 = 33, SP1 = 18, SP2 = 19, VB = 34, LED = 2;
const uint32_t FAILSAFE_MS = 400;
struct Port { Stream* s; uint32_t lastRx; char buf[120]; size_t len; uint32_t period; };   // defined before any function: the Arduino IDE adds prototypes above the first one
// ---- PWM: every output has its own LEDC channel. Motors: channels 0-3 at 4 kHz (timers 0, 1). Servos: channels 4-5 at 50 Hz (timer 2).
// (Two earlier versions used ESP32Servo + analogWrite: they fought over the same channel and moved the servo whenever a wheel changed.)
const int MOTOR_PIN[4] = {IN1, IN2, IN3, IN4}, SERVO_PIN[2] = {SP1, SP2};
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int ch, int hz, int bits) { ledcAttachChannel(pin, hz, bits, ch); }
void pwmOut(int pin, int ch, uint32_t duty) { ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch, int hz, int bits) { ledcSetup(ch, hz, bits); ledcAttachPin(pin, ch); }
void pwmOut(int pin, int ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// ---- tuning values, same order as the TCFG frame
float minpwm = 0, invl = 0, invr = 0, lo1 = 0, hi1 = 180, lo2 = 0, hi2 = 180, ratio = 147.0f / 47.0f;
struct { const char* n; float* v; } T[] = {{"minpwm", &minpwm}, {"invl", &invl}, {"invr", &invr}, {"lo1", &lo1}, {"hi1", &hi1}, {"lo2", &lo2}, {"hi2", &hi2}, {"ratio", &ratio}};
Preferences pf;

// ---- Bluetooth LE as a stream (Nordic UART service)
#define NUS "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
class BleStream : public Stream {
 public:
  uint8_t rx[512]; volatile uint16_t head = 0, tail = 0; volatile bool connected = false; volatile int chunk = 20; BLECharacteristic* tx = nullptr;   // chunk = bytes per notification (MTU - 3)
  int available() override { return (head - tail) & 511; }
  int read() override { if (head == tail) return -1; uint8_t c = rx[tail]; tail = (tail + 1) & 511; return c; }
  int peek() override { return head == tail ? -1 : rx[tail]; }
  void flush() override {}
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t* d, size_t n) override {
    if (!connected || !tx) return 0;
    for (size_t i = 0; i < n; i += chunk) { size_t m = n - i < (size_t)chunk ? n - i : chunk; tx->setValue((uint8_t*)d + i, m); tx->notify(); if (n > (size_t)chunk) delay(3); }   // one notification per frame when the MTU allows
    return n;
  }
  void push(uint8_t c) { uint16_t nh = (head + 1) & 511; if (nh != tail) { rx[head] = c; head = nh; } }
} bt;
volatile int btReason = -1;
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bt.chunk = 20; bt.connected = true; }
  void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* p) override { bt.chunk = constrain((int)p->mtu.mtu - 3, 20, 180); }
  void onDisconnect(BLEServer*, esp_ble_gatts_cb_param_t* p) override { btReason = p->disconnect.reason; bt.connected = false; bt.chunk = 20; esp_ble_remove_bond_device(p->disconnect.remote_bda); BLEDevice::startAdvertising(); }
};
class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override { auto v = c->getValue(); for (size_t i = 0; i < v.length(); i++) bt.push((uint8_t)v[i]); }
};
void btBegin() {
  BLEDevice::init(BT_NAME); BLEDevice::setMTU(185);
  BLESecurity* sec = new BLESecurity(); sec->setAuthenticationMode(ESP_LE_AUTH_NO_BOND); sec->setCapability(ESP_IO_CAP_NONE);
  int n = esp_ble_get_bond_device_num();                              // forget old pairings: a stale one makes the link drop after 1 s
  if (n > 0) { auto* l = (esp_ble_bond_dev_t*)malloc(n * sizeof(esp_ble_bond_dev_t)); if (l && esp_ble_get_bond_device_list(&n, l) == ESP_OK) for (int i = 0; i < n; i++) esp_ble_remove_bond_device(l[i].bd_addr); free(l); }
  BLEServer* sv = BLEDevice::createServer(); sv->setCallbacks(new ServerCb());
  BLEService* svc = sv->createService(NUS);
  bt.tx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY); bt.tx->addDescriptor(new BLE2902());
  svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR)->setCallbacks(new RxCb());
  svc->start(); BLEAdvertising* a = BLEDevice::getAdvertising(); a->addServiceUUID(NUS); a->setScanResponse(true); BLEDevice::startAdvertising();
}

// ---- state
Port P[2] = {{&Serial, 0, {0}, 0, 50}, {&bt, 0, {0}, 0, 100}};
int ang[2] = {-1, -1};                                                // servo angle, -1 = limp (no pulses)
int pct[2], pwmv[2]; bool armed = false, imuOk = false, cal = true; uint8_t who;
int cmdL, cmdR, cmdS1 = -2, cmdS2 = -2; uint16_t lastSeq, telSeq; uint32_t lastCmd, lastTel, tImu, calT0, ledUntil; int calN, blink;
float g[7], bias, yaw; double calSum;                                 // g: ax ay az (g)  temp (C)  gx gy gz (deg/s)

uint8_t xs(const char* s) { uint8_t c = 0; while (*s) c ^= (uint8_t)*s++; return c; }
void send(Port& p, const char* body) { char o[220]; int n = snprintf(o, sizeof o, "$%s*%02X\n", body, xs(body)); if (n > 0 && n < (int)sizeof o) p.s->write((const uint8_t*)o, n); }
bool usable(Port& p) { return p.lastRx && millis() - p.lastRx < 3000 && (&p != &P[1] || bt.connected); }
Port* active() { Port* b = nullptr; for (auto& p : P) if (usable(p) && (!b || (int32_t)(p.lastRx - b->lastRx) > 0)) b = &p; return b; }

// ---- outputs
void drive(int s, int p) {                                            // wheel s (0 left, 1 right), percent; the driver is always enabled, PWM goes on the active IN pin
  p = constrain(p, -100, 100); pct[s] = p;
  int d = (s ? invr : invl) > 0 ? -p : p;
  int pw = p ? map(abs(p), 1, 100, (int)minpwm, 255) : 0; pwmv[s] = d < 0 ? -pw : pw;
  pwmOut(MOTOR_PIN[s * 2], s * 2, d > 0 ? pw : 0); pwmOut(MOTOR_PIN[s * 2 + 1], s * 2 + 1, d < 0 ? pw : 0);
}
void stopAll() { drive(0, 0); drive(1, 0); }
void servo(int i, int deg) {                                          // -1 limp (no pulses), -2 keep, else degrees inside the limits
  if (deg == -2) return;
  if (deg < 0) { if (ang[i] >= 0) pwmOut(SERVO_PIN[i], 4 + i, 0); ang[i] = -1; return; }
  deg = constrain(deg, (int)(i ? lo2 : lo1), (int)(i ? hi2 : hi1));
  if (deg != ang[i]) pwmOut(SERVO_PIN[i], 4 + i, (uint32_t)(500 + 1900L * deg / 180) * 16384UL / 20000UL);   // 0..180 deg = 0.5..2.4 ms pulse in a 20 ms frame
  ang[i] = deg;
}
void disarm() { armed = false; cmdL = cmdR = 0; stopAll(); }

// ---- MPU6050
uint8_t rd(uint8_t r) { Wire.beginTransmission(0x68); Wire.write(r); Wire.endTransmission(false); Wire.requestFrom(0x68, 1); return Wire.read(); }
void imuInit() {
  Wire.begin(21, 22); Wire.setClock(400000);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); imuOk = Wire.endTransmission() == 0; who = imuOk ? rd(0x75) : 0;
}
bool imuRead() {
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false) || Wire.requestFrom(0x68, 14) != 14) return false;
  int16_t r[7]; for (auto& x : r) { uint8_t h = Wire.read(); uint8_t l = Wire.read(); x = (h << 8) | l; }
  for (int i = 0; i < 3; i++) { g[i] = r[i] / 16384.0f; g[4 + i] = r[4 + i] / 131.0f; }
  g[3] = r[3] / 340.0f + 36.53f; return true;
}
void startCal() { cal = true; calN = 0; calSum = 0; calT0 = millis(); }  // keep the robot still for one second

// ---- settings
void tcfg(Port& p) { char o[120]; snprintf(o, sizeof o, "TCFG,%g,%g,%g,%g,%g,%g,%g,%g", minpwm, invl, invr, lo1, hi1, lo2, hi2, ratio); send(p, o); }
void loadCfg() { pf.begin("tune", false); for (auto& t : T) *t.v = pf.getFloat(t.n, *t.v); }

void frame(Port& p, char* line) {
  char* star = strrchr(line, '*'); if (line[0] != '$' || !star) return;
  *star = 0; if ((uint8_t)strtol(star + 1, nullptr, 16) != xs(line + 1)) return;
  char* b = line + 1; p.lastRx = millis() | 1;
  if (!strncmp(b, "CMD,", 4)) {
    int sq, l, r, s1, s2, en;
    if (sscanf(b + 4, "%d,%d,%d,%d,%d,%d", &sq, &l, &r, &s1, &s2, &en) == 6) { lastSeq = sq; cmdL = constrain(l, -100, 100); cmdR = constrain(r, -100, 100); cmdS1 = s1; cmdS2 = s2; armed = en == 1; lastCmd = millis(); }
  } else if (!strncmp(b, "PING,", 5)) { char o[48]; snprintf(o, sizeof o, "PONG,%s,%lu", b + 5, (unsigned long)millis()); send(p, o); }
  else if (!strncmp(b, "HELLO?", 6)) { send(p, "HELLO,1,rec-tune,tune,V_7_0_3"); tcfg(p); }
  else if (!strncmp(b, "STOP", 4)) { disarm(); send(p, "ACK,STOP"); }
  else if (!strncmp(b, "CAL", 3)) { stopAll(); startCal(); send(p, "ACK,CAL"); }
  else if (!strncmp(b, "TCFG,", 5)) {
    char k[8]; float v;
    if (sscanf(b + 5, "%7[^,],%f", k, &v) == 2) for (auto& t : T) if (!strcmp(k, t.n)) { *t.v = v; pf.putFloat(t.n, v); }
    tcfg(p);
  }
}
void pump(Port& p) {
  while (p.s->available()) {
    int c = p.s->read(); if (c < 0) break;
    if (c == '\n') {
      p.buf[p.len] = 0;
      if (p.len) { if (p.buf[0] == '$') frame(p, p.buf); else { ledUntil = millis() + 2000; blink = 3; send(p, "ACK,TEXT"); } }   // typed text: LED lights 2 s and the robot answers
      p.len = 0;
    } else if (c != '\r') { if (p.len < sizeof p.buf - 1) p.buf[p.len++] = c; else p.len = 0; }
  }
}
void telemetry(Port& p) {
  uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(VB);
  int vb = (int)(mv / 8 * ratio);
  bool fs = millis() - lastCmd > FAILSAFE_MS;
  int fl = (armed ? 1 : 0) | (fs ? 2 : 0) | (imuOk && !cal ? 4 : 0) | (cal ? 8 : 0) | (vb > 1000 && vb < 6400 ? 16 : 0);
  char o[200];
  snprintf(o, sizeof o, "TUN,%u,%u,%lu,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d", (unsigned)++telSeq, (unsigned)lastSeq, (unsigned long)millis(), pwmv[0], pwmv[1], ang[0], ang[1],
           g[4], g[5], g[6] - bias, g[0], g[1], g[2], g[3], yaw, vb, fl, who);
  send(p, o);
}

void setup() {
  pinMode(LED, OUTPUT);
  for (int k = 0; k < 4; k++) { pwmAttach(MOTOR_PIN[k], k, 4000, 8); pwmOut(MOTOR_PIN[k], k, 0); }      // motors off
  for (int k = 0; k < 2; k++) { pwmAttach(SERVO_PIN[k], 4 + k, 50, 14); pwmOut(SERVO_PIN[k], 4 + k, 0); }  // servos limp until told
  Serial.begin(115200); analogReadResolution(12); analogSetPinAttenuation(VB, ADC_11db);
  loadCfg(); imuInit(); startCal(); btBegin();
  Serial.printf("\nREC tune firmware V_7_0_3 ready. BLE name '%s'. Gyro %s (id 0x%02X)\n", BT_NAME, imuOk ? "found" : "NOT found - check 3V3 SDA 21 SCL 22", who);
}

void loop() {
  uint32_t t = millis();
  pump(P[0]); pump(P[1]);
  bool fs = t - lastCmd > FAILSAFE_MS, live = armed && !fs;
  if (!live) { if (pct[0] || pct[1]) stopAll(); } else { drive(0, cmdL); drive(1, cmdR); servo(0, cmdS1); servo(1, cmdS2); }
  if (imuOk && t - tImu >= 10) {                                      // 100 Hz
    float dt = (t - tImu) * 1e-3f; tImu = t;
    if (imuRead()) {
      if (cal) { calSum += g[6]; calN++; if (t - calT0 > 1000 && calN > 20) { bias = calSum / calN; cal = false; yaw = 0; } }
      else { float z = g[6] - bias; if (fabsf(z) > 0.15f) yaw += z * dt; }
    }
  }
  static bool seen; if (bt.connected != seen) {                       // link change: never stay armed across it
    seen = bt.connected; if (seen) blink = 5; else { Serial.printf("[bt] disconnected, reason 0x%02X\n", btReason); if (P[1].lastRx > P[0].lastRx) disarm(); }
  }
  P[1].period = bt.chunk >= 120 ? 100 : 250;                          // a frame fits one notification: 10 per second; otherwise slower
  Port* p = active(); if (p && t - lastTel >= p->period) { lastTel = t; telemetry(*p); }
  static uint32_t bt0; static int bs; if (blink && t - bt0 > 150) { bt0 = t; bs++; if (bs >= blink * 2) { blink = bs = 0; } }
  digitalWrite(LED, blink ? bs % 2 == 0 : ((int32_t)(ledUntil - t) > 0 || bt.connected));
}
