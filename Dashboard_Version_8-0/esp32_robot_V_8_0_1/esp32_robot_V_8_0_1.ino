// REC robot - FULL ROBOT firmware (V_8_0_1) for the Robot Station dashboard (Game mode: Xbox controller -> motors, grip/lift servos, gyro feedback).
// Board "ESP32 Dev Module" (core 3.x), no extra library. Serial Monitor 115200 / Newline. Link: Bluetooth LE ("REC-Robot", no pairing) AND the USB cable.
// Pins: L298N IN1 26 IN2 27 | IN3 32 IN4 33 (ENA/ENB jumpers ON, speed = PWM on the IN pins) | servo1 (grip) 18, servo2 (lift) 19 | MPU6050 SDA 21 SCL 22 | battery divider 34 (optional)
// Protocol rec1 ($TYPE,f,f*HH): CMD,seq,vl,vr,grip,lift,enable every 50 ms = heartbeat. Nothing moves until the dashboard arms; no command for 400 ms or a lost link = wheels stop.
// Gyro: the yaw angle is sent to the dashboard AND used to hold a straight line (heading hold) when both sticks ask for the same speed.
// Tuning from the Tune page (min PWM, wheel invert, servo limits) is kept in flash by esp32_tune_V_7_0_4 and is read here, so flash that one first, tune, then flash this.
// NOT TESTED on the real robot: wheels in the air first, then on the floor with a low speed limit.
#include <Wire.h>
#include <esp_system.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------------------------------------------------------- settings you may change
const char* BT_NAME = "REC-Robot";
const char* ROBOT_TYPE = "simple";          // "simple" = grip servo only on GPIO 18. "tower" = grip on 18 + lift servo on 19 (lift stick moves it like a speed)
const char* FW_VERSION = "V_8_0_1";
const int IN1 = 26, IN2 = 27, IN3 = 32, IN4 = 33, SP1 = 18, SP2 = 19, VB = 34, LED = 2;
const uint32_t FAILSAFE_MS = 400;           // no valid CMD for this long = wheels stop
const float VBAT_RATIO = 0;                 // 0 = no battery divider fitted (battery shows 0). With the 100k/47k divider on GPIO 34 use 147.0/47.0
const float GYRO_SIGN = 1.0f;               // +1: turning LEFT (counter-clockwise from above) raises yaw. If it is the other way round (or the sensor is upside down) use -1
const float GYRO_DEADBAND = 0.15f;          // deg/s ignored while integrating yaw
const float RAMP_UP = 300, RAMP_DOWN = 600; // wheel speed may change by at most this many % per second (softer starts = smaller current spikes). Stopping by failsafe is instant
const float SERVO_DPS = 150.0f;             // servos move at most this fast (deg/s)
const float LIFT_RATE = 0.4f;               // tower: % of lift travel per second for a full lift stick
const int LIFT_MIN_DEG = 20, LIFT_MAX_DEG = 160;
const float HOLD_KP = 1.2f, HOLD_KD = 0.15f, HOLD_MAX = 25;   // heading hold: % of wheel speed added per degree of error, per deg/s of turning, and its limit
struct Port { Stream* s; uint32_t lastRx; char buf[120]; size_t len; uint32_t period; };   // defined before any function: the Arduino IDE adds prototypes above the first one

// ---- PWM: every output has its own LEDC channel. Motors: channels 0-3 at 4 kHz. Servos: channels 4-5 at 50 Hz.
const int MOTOR_PIN[4] = {IN1, IN2, IN3, IN4}, SERVO_PIN[2] = {SP1, SP2};
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmAttach(int pin, int ch, int hz, int bits) { ledcAttachChannel(pin, hz, bits, ch); }
void pwmOut(int pin, int ch, uint32_t duty) { ledcWrite(pin, duty); }
#else
void pwmAttach(int pin, int ch, int hz, int bits) { ledcSetup(ch, hz, bits); ledcAttachPin(pin, ch); }
void pwmOut(int pin, int ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// ---- settings kept in flash. The first seven are the same names/namespace as the tune firmware; go, gc, hold are new.
float minpwm = 0, invl = 0, invr = 0, lo1 = 0, hi1 = 180, lo2 = 0, hi2 = 180, go = 90, gc = 30, hold = 1;
struct { const char* n; float* v; } T[] = {{"minpwm", &minpwm}, {"invl", &invl}, {"invr", &invr}, {"lo1", &lo1}, {"hi1", &hi1}, {"lo2", &lo2}, {"hi2", &hi2}, {"go", &go}, {"gc", &gc}, {"hold", &hold}};
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
    for (size_t i = 0; i < n; i += chunk) { size_t m = n - i < (size_t)chunk ? n - i : chunk; tx->setValue((uint8_t*)d + i, m); tx->notify(); if (n > (size_t)chunk) delay(3); }
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
bool armed = false, imuOk = false, cal = true, tower;
int cmdL, cmdR, cmdGrip, cmdLift; uint16_t lastSeq, telSeq; uint32_t lastCmd, lastTel, tImu, calT0, ledUntil; int calN, blink;
float curL, curR, liftPos, yaw, gz, bias, holdYaw; double calSum; bool holding; uint32_t straightSince;
int ang[2] = {-1, -1}, tgt[2] = {-1, -1}; float spos[2];            // servo: angle now (-1 = limp), wanted angle, angle on its way

uint8_t xs(const char* s) { uint8_t c = 0; while (*s) c ^= (uint8_t)*s++; return c; }
void send(Port& p, const char* body) { char o[200]; int n = snprintf(o, sizeof o, "$%s*%02X\n", body, xs(body)); if (n > 0 && n < (int)sizeof o) p.s->write((const uint8_t*)o, n); }
bool usable(Port& p) { return p.lastRx && millis() - p.lastRx < 3000 && (&p != &P[1] || bt.connected); }
Port* active() { Port* b = nullptr; for (auto& p : P) if (usable(p) && (!b || (int32_t)(p.lastRx - b->lastRx) > 0)) b = &p; return b; }
const char* rstText() {                                               // BROWNOUT = the supply dipped
  switch (esp_reset_reason()) { case ESP_RST_POWERON: return "poweron"; case ESP_RST_SW: return "software"; case ESP_RST_BROWNOUT: return "BROWNOUT"; case ESP_RST_PANIC: return "CRASH"; case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "watchdog"; default: return "other"; }
}

// ---- outputs
void wheel(int s, float p) {                                          // wheel s (0 left, 1 right), percent
  int pc = constrain((int)lroundf(p), -100, 100), d = (s ? invr : invl) > 0 ? -pc : pc;
  int pw = pc ? map(abs(pc), 1, 100, (int)minpwm, 255) : 0;
  pwmOut(MOTOR_PIN[s * 2], s * 2, d > 0 ? pw : 0); pwmOut(MOTOR_PIN[s * 2 + 1], s * 2 + 1, d < 0 ? pw : 0);
}
void stopWheels() { curL = curR = 0; wheel(0, 0); wheel(1, 0); holding = false; }
void disarm() { armed = false; cmdL = cmdR = 0; stopWheels(); }
void servoPulse(int i, int deg) { pwmOut(SERVO_PIN[i], 4 + i, (uint32_t)(500 + 1900L * deg / 180) * 16384UL / 20000UL); }   // 0..180 deg = 0.5..2.4 ms in a 20 ms frame
void servo(int i, int deg) {                                          // degrees inside the limits; reached at SERVO_DPS
  tgt[i] = constrain(deg, (int)(i ? lo2 : lo1), (int)(i ? hi2 : hi1));
  if (ang[i] < 0) { spos[i] = tgt[i]; ang[i] = tgt[i]; servoPulse(i, ang[i]); }   // first command after power-up: the real position is unknown, start there
}
void servoStep(float dt) {
  for (int i = 0; i < 2; i++) if (ang[i] >= 0 && spos[i] != tgt[i]) {
    float d = tgt[i] - spos[i], m = SERVO_DPS * dt; spos[i] += d > m ? m : d < -m ? -m : d;
    int a = (int)(spos[i] + 0.5f); if (a != ang[i]) { ang[i] = a; servoPulse(i, a); }
  }
}
float ramp(float cur, float want, float dt) { float d = want - cur, up = (fabsf(want) > fabsf(cur) ? RAMP_UP : RAMP_DOWN) * dt; return d > up ? cur + up : d < -up ? cur - up : want; }

void drive(float dt) {                                                // from the stick command to the wheels: heading hold, then ramp
  float l = cmdL, r = cmdR;
  bool straight = hold > 0 && imuOk && !cal && abs(cmdL - cmdR) < 8 && abs(cmdL + cmdR) > 30;   // both sticks agree and the robot is really driving
  if (!straight) { holding = false; straightSince = millis(); }
  else if (millis() - straightSince > 300) {                          // wait 300 ms so the turn the driver just finished does not become the target
    if (!holding) { holding = true; holdYaw = yaw; }
    float c = constrain(HOLD_KP * (yaw - holdYaw) + HOLD_KD * gz, -HOLD_MAX, HOLD_MAX);   // drifted left (yaw up) -> left wheel faster -> turns right again
    l += c; r -= c;
  }
  curL = ramp(curL, constrain(l, -100.0f, 100.0f), dt); curR = ramp(curR, constrain(r, -100.0f, 100.0f), dt);
  wheel(0, curL); wheel(1, curR);
}

// ---- MPU6050
uint8_t rd(uint8_t r) { Wire.beginTransmission(0x68); Wire.write(r); Wire.endTransmission(false); Wire.requestFrom(0x68, 1); return Wire.read(); }
void imuInit() {
  Wire.begin(21, 22); Wire.setClock(400000);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); imuOk = Wire.endTransmission() == 0;
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0); Wire.endTransmission();   // gyro +-250 deg/s
}
bool gyroZ(float& dps) {
  Wire.beginTransmission(0x68); Wire.write(0x47);
  if (Wire.endTransmission(false) || Wire.requestFrom(0x68, 2) != 2) return false;
  uint8_t h = Wire.read(); uint8_t l = Wire.read(); dps = (int16_t)((h << 8) | l) / 131.0f; return true;
}
void startCal() { cal = true; calN = 0; calSum = 0; calT0 = millis(); }  // keep the robot still for one second

// ---- frames
void frame(Port& p, char* line) {
  char* star = strrchr(line, '*'); if (line[0] != '$' || !star) return;
  *star = 0; if ((uint8_t)strtol(star + 1, nullptr, 16) != xs(line + 1)) return;
  char* b = line + 1; p.lastRx = millis() | 1;
  if (!strncmp(b, "CMD,", 4)) {
    int sq, l, r, g, lf, en;
    if (sscanf(b + 4, "%d,%d,%d,%d,%d,%d", &sq, &l, &r, &g, &lf, &en) == 6) { lastSeq = sq; cmdL = constrain(l, -100, 100); cmdR = constrain(r, -100, 100); cmdGrip = constrain(g, 0, 100); cmdLift = constrain(lf, -100, 100); armed = en == 1; lastCmd = millis(); }
  } else if (!strncmp(b, "PING,", 5)) { char o[48]; snprintf(o, sizeof o, "PONG,%s,%lu", b + 5, (unsigned long)millis()); send(p, o); }
  else if (!strncmp(b, "HELLO?", 6)) { char o[72]; snprintf(o, sizeof o, "HELLO,1,rec-robot,%s,%s", ROBOT_TYPE, FW_VERSION); send(p, o); snprintf(o, sizeof o, "RST,%s,%lu", rstText(), (unsigned long)(millis() / 1000)); send(p, o); }
  else if (!strncmp(b, "STOP", 4)) { disarm(); send(p, "ACK,STOP"); }
  else if (!strncmp(b, "ZERO", 4)) { yaw = 0; holding = false; send(p, "ACK,ZERO"); }
  else if (!strncmp(b, "CAL", 3)) { stopWheels(); startCal(); send(p, "ACK,CAL"); }
  else if (!strncmp(b, "CFG,", 4)) {                                  // CFG,go,90  CFG,gc,30  CFG,hold,0  CFG,minpwm,50 ... saved in flash
    char k[8]; float v;
    if (sscanf(b + 4, "%7[^,],%f", k, &v) == 2) for (auto& t : T) if (!strcmp(k, t.n)) { *t.v = v; pf.putFloat(t.n, v); char o[32]; snprintf(o, sizeof o, "ACK,CFG,%s", k); send(p, o); }
  }
}
void pump(Port& p) {
  while (p.s->available()) {
    int c = p.s->read(); if (c < 0) break;
    if (c == '\n') {
      p.buf[p.len] = 0;
      if (p.len) {
        if (p.buf[0] == '$') frame(p, p.buf);
        else if (!strcmp(p.buf, "status") && &p == &P[0]) Serial.printf("[status] %s %s yaw %.1f gyro %s armed %d hold %d bt %d restart %s\n", FW_VERSION, ROBOT_TYPE, yaw, imuOk ? "ok" : "MISSING", armed, (int)hold, (int)bt.connected, rstText());
        else { ledUntil = millis() + 2000; blink = 3; send(p, "ACK,TEXT"); }   // typed text: LED lights 2 s and the robot answers
      }
      p.len = 0;
    } else if (c != '\r') { if (p.len < sizeof p.buf - 1) p.buf[p.len++] = c; else p.len = 0; }
  }
}
void telemetry(Port& p) {
  int vb = 0;
  if (VBAT_RATIO > 0) { uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(VB); vb = (int)(mv / 8 * VBAT_RATIO); }
  bool fs = millis() - lastCmd > FAILSAFE_MS;
  int fl = (armed ? 1 : 0) | (fs ? 2 : 0) | (imuOk && !cal ? 4 : 0) | (cal ? 8 : 0) | (vb > 1000 && vb < 6400 ? 16 : 0);
  char o[160];
  snprintf(o, sizeof o, "TEL,%u,%u,%lu,%.1f,%.1f,%d,%d,%d,%d,%d,%d", (unsigned)++telSeq, (unsigned)lastSeq, (unsigned long)millis(), yaw, gz, (int)lroundf(curL), (int)lroundf(curR), cmdGrip, (int)liftPos, vb, fl);
  send(p, o);
}

void setup() {
  pinMode(LED, OUTPUT); tower = !strcmp(ROBOT_TYPE, "tower");
  for (int k = 0; k < 4; k++) { pwmAttach(MOTOR_PIN[k], k, 4000, 8); pwmOut(MOTOR_PIN[k], k, 0); }      // motors off
  for (int k = 0; k < 2; k++) { pwmAttach(SERVO_PIN[k], 4 + k, 50, 14); pwmOut(SERVO_PIN[k], 4 + k, 0); }  // servos limp until the first armed command
  Serial.begin(115200); analogReadResolution(12); analogSetPinAttenuation(VB, ADC_11db);
  pf.begin("tune", false); for (auto& t : T) *t.v = pf.getFloat(t.n, *t.v);
  imuInit(); startCal(); btBegin();
  Serial.printf("\nLast restart: %s.\nREC robot firmware %s (%s) ready. BLE name '%s'. Gyro %s. Keep the robot still for 1 s: gyro calibration.\n", rstText(), FW_VERSION, ROBOT_TYPE, BT_NAME, imuOk ? "found" : "NOT found - check 3V3 SDA 21 SCL 22");
}

void loop() {
  uint32_t t = millis(); static uint32_t tLoop; float dt = (t - tLoop) * 1e-3f; tLoop = t;
  pump(P[0]); pump(P[1]);
  bool fs = t - lastCmd > FAILSAFE_MS, live = armed && !fs;
  if (imuOk && t - tImu >= 10) {                                      // gyro at 100 Hz
    float gdt = (t - tImu) * 1e-3f, d; tImu = t;
    if (gyroZ(d)) {
      if (cal) { calSum += d; calN++; gz = 0; if (t - calT0 > 1000 && calN > 20) { bias = calSum / calN; cal = false; yaw = 0; } }
      else { gz = (d - bias) * GYRO_SIGN; if (fabsf(gz) > GYRO_DEADBAND) yaw += gz * gdt; }
    }
  }
  if (!live) { if (curL != 0 || curR != 0 || holding) stopWheels(); }   // disarmed, link silent or failsafe: wheels stop at once, servos hold their position
  else {
    drive(dt);
    servo(0, (int)map(cmdGrip, 0, 100, (int)go, (int)gc));
    if (tower) { liftPos = constrain(liftPos + cmdLift * LIFT_RATE * dt, 0.0f, 100.0f); servo(1, (int)map((int)liftPos, 0, 100, LIFT_MIN_DEG, LIFT_MAX_DEG)); }
  }
  servoStep(dt);
  static bool seen; if (bt.connected != seen) {                       // link change: never stay armed across it
    seen = bt.connected; if (seen) blink = 5; else { Serial.printf("[bt] disconnected, reason 0x%02X\n", btReason); if (P[1].lastRx > P[0].lastRx) disarm(); }
  }
  P[1].period = bt.chunk >= 120 ? 100 : 250;                          // a frame fits one notification: 10 per second; otherwise slower
  Port* p = active(); if (p && t - lastTel >= p->period) { lastTel = t; telemetry(*p); }
  static uint32_t bt0; static int bs; if (blink && t - bt0 > 150) { bt0 = t; bs++; if (bs >= blink * 2) { blink = bs = 0; } }
  digitalWrite(LED, blink ? bs % 2 == 0 : (armed && fs ? (t / 100) % 2 : ((int32_t)(ledUntil - t) > 0 || bt.connected)));   // fast blink = armed but no commands
}
