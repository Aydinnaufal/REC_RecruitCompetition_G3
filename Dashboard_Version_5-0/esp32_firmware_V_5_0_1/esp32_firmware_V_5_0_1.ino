// REC robot firmware V_5_0 - ESP32 NodeMCU-32S (38 pin). Talks to Robot Station V 5.0 over the USB cable AND one wireless link of your choice.
// Protocol "rec1": text frames  $TYPE,f1,f2*HH\n  (HH = XOR of the characters between $ and *). See COMMS_ARCHITECTURE_V_4_0.md and README_V_5_0.md.
//
// >>> THE ONE LINE TO CHECK: BT_MODE below. <<<
//     BT_BLE     (default) Bluetooth LE. The dashboard scans for "REC-Robot" and connects. NO Windows pairing, NO COM port. Needs `pip install bleak`.
//     BT_CLASSIC Bluetooth Classic serial (SPP). Windows must pair it first and makes a COM port (use the Outgoing one), or use the MAC address mode.
//     BT_NONE    USB cable only. Smallest sketch, use it to prove the board, motors and dashboard work before adding wireless.
//
// Board : "ESP32 Dev Module" (Arduino-ESP32 core 3.x).  Library: ESP32Servo (Library Manager).  Serial Monitor: 115200, Newline.
//         "Sketch too big" on upload: Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)". Bluetooth makes the sketch large.
// Needs a classic ESP32 chip (WROOM-32 / NodeMCU-32S). ESP32-S2 has no Bluetooth; S3 and C3 have BLE only (BT_BLE works, BT_CLASSIC does not).
//
// Pins (same as perfboard_build_notes.md / the L298N wiring):
//   L298N  ENA 25  IN1 26  IN2 27  |  ENB 23  IN3 32  IN4 33        left motor = A, right motor = B
//   Servo  grip 18, lift 19 (the lift servo is only used by the tower robot)
//   MPU6050 SDA 21  SCL 22 (address 0x68)       battery divider 100k / 47k on GPIO 34
//   LED    GPIO 2 (built in): slow flash = waiting for the laptop, solid = laptop connected, fast blink = armed but no commands
//
// Safety: nothing moves until the laptop sends enable=1, the motors stop when no valid CMD arrived for FAILSAFE_MS, and losing the
// wireless connection disarms the robot at once.
// NOT TESTED ON THE REAL ROBOT. The protocol and the USB / Classic paths were run on a PC against the Python dashboard code; the BLE
// part was only compiled against stand-in headers. Wheels in the air for the first run.
#define BT_NONE 0
#define BT_CLASSIC 1
#define BT_BLE 2
#ifndef BT_MODE
#define BT_MODE BT_BLE                                  // <<< choose BT_BLE, BT_CLASSIC or BT_NONE
#endif

#include <ESP32Servo.h>
#include <Wire.h>
#include <esp_system.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>                                  // core 3.x: esp_read_mac / ESP_MAC_BT live here
#endif
#if BT_MODE == BT_CLASSIC
#include <BluetoothSerial.h>
#elif BT_MODE == BT_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

// ---------------------------------------------------------------- settings you may change
const char* BT_NAME = "REC-Robot";                  // the name the dashboard shows (BLE scan list or Windows pairing list). Keep it starting with REC
const char* ROBOT_TYPE = "simple";                  // "simple" = claw robot, "tower" = lift + pad robot
const char* FW_NAME = "rec-esp32";
const char* FW_VERSION = "V_5_0";
const int ENA = 25, IN1 = 26, IN2 = 27, ENB = 23, IN3 = 32, IN4 = 33;
const int SERVO_GRIP = 18, SERVO_LIFT = 19, VBAT_PIN = 34, LED_PIN = 2;
const bool INVERT_LEFT = false, INVERT_RIGHT = false;   // flip if a wheel turns the wrong way (or swap the motor wires)
const int MIN_PWM = 0;                                  // 0..255: lowest duty that still turns the wheel (TT motors: try 40-60)
int gripOpenDeg = 90, gripClosedDeg = 30;               // tune on the real gripper (CFG,go,<deg> and CFG,gc,<deg> change them live)
const int LIFT_MIN_DEG = 20, LIFT_MAX_DEG = 160;        // tower lift servo range for 0..100 %
const float LIFT_RATE = 0.4f;                           // % per second per 1 % of lift command (matches the dashboard simulation)
const float GYRO_SIGN = 1.0f;                           // +1: turning left (counter-clockwise from above) raises yaw. Use -1 if it is the other way.
const float GYRO_DEADBAND = 0.15f;                      // deg/s ignored while integrating yaw (stops slow drift)
const uint32_t GYRO_CAL_MS = 1000;                      // keep the robot still this long after power-up
uint32_t failsafeMs = 400;                              // motors stop when no valid CMD for this long (CFG,fs,<ms>)
const uint32_t TEL_PERIOD_MS = 50;                      // 20 Hz telemetry (USB, Classic)
const uint32_t TEL_PERIOD_BLE_MS = 100;                 // 10 Hz over BLE: every notification is a radio packet, and 10 Hz is plenty for a dashboard
const float VBAT_RATIO = 147.0f / 47.0f;                // divider (100k + 47k) / 47k. Set to 0 if no divider is fitted.

// Port is defined first on purpose: the Arduino IDE inserts function prototypes above the first function, and they mention Port.
struct Port {
  Stream* s;
  const char* name;
  uint32_t lastRx;       // millis() of the last valid frame from this port (0 = never)
  char buf[120];
  size_t len;
  uint32_t telPeriod;
};

// ---------------------------------------------------------------- the wireless stream
#if BT_MODE == BT_BLE
// Nordic UART service: RX (laptop writes), TX (robot notifies). The dashboard looks for exactly these.
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
const size_t BLE_CHUNK = 20;                           // every BLE link can carry 20 bytes per notification, whatever MTU was negotiated

class BleStream : public Stream {                      // makes the BLE characteristics look like a serial port
 public:
  uint8_t rx[512];
  volatile uint16_t head = 0, tail = 0;
  volatile bool connected = false;
  BLECharacteristic* tx = nullptr;
  int available() override { return (head - tail) & 511; }
  int read() override { if (head == tail) return -1; uint8_t c = rx[tail]; tail = (tail + 1) & 511; return c; }
  int peek() override { return head == tail ? -1 : rx[tail]; }
  void flush() override {}
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t* d, size_t n) override {
    if (!connected || !tx) return 0;
    for (size_t i = 0; i < n; i += BLE_CHUNK) {
      size_t m = (n - i < BLE_CHUNK) ? (n - i) : BLE_CHUNK;
      tx->setValue((uint8_t*)d + i, m);
      tx->notify();
      if (n > BLE_CHUNK) delay(2);                     // let the radio stack drain between the pieces of one frame
    }
    return n;
  }
  void push(uint8_t c) { uint16_t nh = (head + 1) & 511; if (nh != tail) { rx[head] = c; head = nh; } }
};
BleStream SerialBT;                                    // same name as the Classic object so the rest of the sketch does not care
bool btHasClient() { return SerialBT.connected; }

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { SerialBT.connected = true; }
  void onDisconnect(BLEServer*) override { SerialBT.connected = false; BLEDevice::startAdvertising(); }
};
class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();                            // std::string on core 2.x, String on core 3.x: both have length() and []
    for (size_t i = 0; i < v.length(); i++) SerialBT.push((uint8_t)v[i]);
  }
};

bool btBegin() {
  BLEDevice::init(BT_NAME);
  BLEDevice::setMTU(185);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService* svc = server->createService(NUS_SERVICE);
  SerialBT.tx = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  SerialBT.tx->addDescriptor(new BLE2902());
  BLECharacteristic* rxc = svc->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxc->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);                          // the name goes in the scan response (the service UUID fills the first packet)
  BLEDevice::startAdvertising();
  return true;
}
const char* BT_KIND = "Bluetooth LE";
#elif BT_MODE == BT_CLASSIC
BluetoothSerial SerialBT;
bool btHasClient() { return SerialBT.hasClient(); }
bool btBegin() { return SerialBT.begin(BT_NAME); }
const char* BT_KIND = "Bluetooth Classic";
#else
const char* BT_KIND = "none (USB only)";
#endif

// ---------------------------------------------------------------- state
#if BT_MODE == BT_NONE
Port ports[2] = {{&Serial, "usb", 0, {0}, 0, TEL_PERIOD_MS}, {nullptr, "bt", 0, {0}, 0, TEL_PERIOD_MS}};
#else
Port ports[2] = {{&Serial, "usb", 0, {0}, 0, TEL_PERIOD_MS}, {&SerialBT, "bt", 0, {0}, 0, BT_MODE == BT_BLE ? TEL_PERIOD_BLE_MS : TEL_PERIOD_MS}};
#endif
Servo servoGrip, servoLift;

int cmdVl = 0, cmdVr = 0, cmdGrip = 0, cmdLift = 0;
bool armed = false;
int appliedVl = 0, appliedVr = 0, appliedGrip = 0;
float liftPos = 0;                                     // 0..100 %
uint16_t lastCmdSeq = 0, telSeq = 0;
uint32_t lastCmdMs = 0, lastLoopUs = 0;
bool btReady = false, btClientSeen = false;
uint32_t lastTelMs = 0, lastWaitPrint = 0;

// gyro
float yawDeg = 0, gzDps = 0, gyroBias = 0;
bool gyroOk = false, gyroCalibrating = true;
uint32_t gyroStartMs = 0, gyroLastUs = 0, gyroCalN = 0, gyroFails = 0;
double gyroCalSum = 0;

// ---------------------------------------------------------------- helpers
uint8_t xorSum(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

void sendFrame(Port& p, const char* body) {            // body = "TYPE,f1,f2"; one write so Bluetooth sends one packet
  char out[170];
  int n = snprintf(out, sizeof out, "$%s*%02X\n", body, xorSum(body, strlen(body)));
  if (n > 0 && n < (int)sizeof out) p.s->write((const uint8_t*)out, (size_t)n);
}

bool portUsable(Port& p) {
  if (!p.s || p.lastRx == 0 || millis() - p.lastRx > 3000) return false;
#if BT_MODE != BT_NONE
  if (&p == &ports[1] && !btHasClient()) return false;
#endif
  return true;
}

Port* activePort() {                                   // the port that spoke last: telemetry goes only there
  Port* best = nullptr;
  for (int i = 0; i < 2; i++)
    if (portUsable(ports[i]) && (!best || (int32_t)(ports[i].lastRx - best->lastRx) > 0)) best = &ports[i];
  return best;
}

// ---------------------------------------------------------------- outputs
void driveOne(int en, int a, int b, int pct, bool invert) {
  if (invert) pct = -pct;
  int mag = constrain(abs(pct), 0, 100);
  int pwm = mag == 0 ? 0 : map(mag, 1, 100, MIN_PWM, 255);
  digitalWrite(a, pct > 0 ? HIGH : LOW);
  digitalWrite(b, pct < 0 ? HIGH : LOW);
  analogWrite(en, pwm);
}

void setMotors(int l, int r) {
  static bool first = true;
  if (!first && l == appliedVl && r == appliedVr) return;   // same as before: leave the pins alone
  first = false;
  appliedVl = l; appliedVr = r;
  driveOne(ENA, IN1, IN2, l, INVERT_LEFT);
  driveOne(ENB, IN3, IN4, r, INVERT_RIGHT);
}

void applyOutputs(bool live, float dt) {
  if (!live) {                                         // failsafe or disarmed: wheels stop, grip and lift hold their last position
    setMotors(0, 0);
    return;
  }
  setMotors(cmdVl, cmdVr);
  appliedGrip = cmdGrip;
  static int lastGripDeg = -1;
  int gripDeg = map(cmdGrip, 0, 100, gripOpenDeg, gripClosedDeg);
  if (gripDeg != lastGripDeg) { lastGripDeg = gripDeg; servoGrip.write(gripDeg); }
  if (strcmp(ROBOT_TYPE, "tower") == 0) {              // lift command is a speed (-100..100): integrate it into a position
    liftPos = constrain(liftPos + cmdLift * LIFT_RATE * dt, 0.0f, 100.0f);
    static int lastLiftDeg = -1;
    int liftDeg = map((int)liftPos, 0, 100, LIFT_MIN_DEG, LIFT_MAX_DEG);
    if (liftDeg != lastLiftDeg) { lastLiftDeg = liftDeg; servoLift.write(liftDeg); }
  }
}

// ---------------------------------------------------------------- gyro (MPU6050, raw registers, no library)
bool gyroReadZ(float& dps) {
  Wire.beginTransmission(0x68);
  Wire.write(0x47);                                    // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 2) != 2) return false;
  uint8_t hi = Wire.read();                            // two statements: C++ does not define the order inside one expression
  uint8_t lo = Wire.read();
  int16_t raw = (int16_t)((hi << 8) | lo);
  dps = raw / 131.0f;                                  // +-250 deg/s range
  return true;
}

void gyroBegin() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);                  // wake up
  gyroOk = Wire.endTransmission() == 0;
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x00);                  // gyro range +-250 deg/s
  Wire.endTransmission();
  gyroStartMs = millis();
  gyroLastUs = micros();
  gyroCalibrating = true;
}

void gyroUpdate() {
  uint32_t nowUs = micros();
  if (nowUs - gyroLastUs < 5000) return;               // 200 Hz
  float dt = (nowUs - gyroLastUs) / 1e6f;
  gyroLastUs = nowUs;
  float dps;
  if (!gyroReadZ(dps)) {
    if (++gyroFails > 20) gyroOk = false;
    return;
  }
  gyroFails = 0; gyroOk = true;
  if (gyroCalibrating) {
    gyroCalSum += dps; gyroCalN++;
    if (millis() - gyroStartMs >= GYRO_CAL_MS && gyroCalN > 20) {
      gyroBias = (float)(gyroCalSum / gyroCalN);
      gyroCalibrating = false;
    }
    gzDps = 0;
    return;
  }
  gzDps = (dps - gyroBias) * GYRO_SIGN;
  if (fabsf(gzDps) > GYRO_DEADBAND) yawDeg += gzDps * dt;
}

// ---------------------------------------------------------------- frames:  $TYPE,f,f*HH
void handleFrame(Port& p, char* line) {
  char* star = strrchr(line, '*');
  if (line[0] != '$' || !star) return;
  *star = 0;
  if ((uint8_t)strtol(star + 1, nullptr, 16) != xorSum(line + 1, strlen(line + 1))) return;   // bad checksum: ignore silently
  char* body = line + 1;
  p.lastRx = millis();
  if (p.lastRx == 0) p.lastRx = 1;
  if (!strncmp(body, "CMD,", 4)) {
    int seq, vl, vr, g, l, en;
    if (sscanf(body + 4, "%d,%d,%d,%d,%d,%d", &seq, &vl, &vr, &g, &l, &en) == 6) {
      lastCmdSeq = (uint16_t)seq;
      cmdVl = constrain(vl, -100, 100); cmdVr = constrain(vr, -100, 100);
      cmdGrip = constrain(g, 0, 100); cmdLift = constrain(l, -100, 100);
      armed = (en == 1);
      lastCmdMs = millis();
    }
  } else if (!strncmp(body, "PING,", 5)) {
    char out[48]; snprintf(out, sizeof out, "PONG,%s,%lu", body + 5, (unsigned long)millis()); sendFrame(p, out);
  } else if (!strncmp(body, "HELLO?", 6)) {
    char out[72]; snprintf(out, sizeof out, "HELLO,1,%s,%s,%s", FW_NAME, ROBOT_TYPE, FW_VERSION); sendFrame(p, out);
  } else if (!strncmp(body, "ZERO", 4)) {
    yawDeg = 0; sendFrame(p, "ACK,ZERO");
  } else if (!strncmp(body, "STOP", 4)) {
    armed = false; cmdVl = cmdVr = 0; setMotors(0, 0); sendFrame(p, "ACK,STOP");
  } else if (!strncmp(body, "CFG,", 4)) {              // CFG,go,90   CFG,gc,30   CFG,fs,400
    char key[8]; int v;
    if (sscanf(body + 4, "%7[^,],%d", key, &v) == 2) {
      if (!strcmp(key, "go")) gripOpenDeg = constrain(v, 0, 180);
      else if (!strcmp(key, "gc")) gripClosedDeg = constrain(v, 0, 180);
      else if (!strcmp(key, "fs")) failsafeMs = constrain(v, 100, 2000);
      char out[32]; snprintf(out, sizeof out, "ACK,CFG,%s", key); sendFrame(p, out);
    }
  }                                                    // unknown frame types are ignored on purpose (newer laptop, older robot)
}

void printStatus() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("[status] %s %s name '%s' address %02X:%02X:%02X:%02X:%02X:%02X  client %s  armed %d  yaw %.1f  gyro %s\n", FW_VERSION, BT_KIND, BT_NAME,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
#if BT_MODE == BT_NONE
                "n/a",
#else
                btHasClient() ? "YES" : "no",
#endif
                armed ? 1 : 0, yawDeg, gyroOk ? "ok" : "MISSING");
}

void pump(Port& p) {
  if (!p.s) return;
  while (p.s->available()) {
    int c = p.s->read();
    if (c < 0) break;
    if (c == '\n') {
      p.buf[p.len] = 0;
      if (p.len) {
        if (p.buf[0] == '$') handleFrame(p, p.buf);
        else if (!strcmp(p.buf, "status") && &p == &ports[0]) printStatus();    // type  status  in the Serial Monitor
      }
      p.len = 0;
    } else if (c != '\r') {
      if (p.len < sizeof p.buf - 1) p.buf[p.len++] = (char)c;
      else p.len = 0;                                  // runaway line: drop it
    }
  }
}

void sendTelemetry(Port& p) {
  int vbat = VBAT_RATIO > 0 ? (int)(analogReadMilliVolts(VBAT_PIN) * VBAT_RATIO) : 0;
  bool failsafe = (millis() - lastCmdMs) > failsafeMs;
  int flags = (armed ? 1 : 0) | (failsafe ? 2 : 0) | (gyroOk && !gyroCalibrating ? 4 : 0) | (gyroCalibrating ? 8 : 0) | ((vbat > 1000 && vbat < 6600) ? 16 : 0);
  char out[130];
  snprintf(out, sizeof out, "TEL,%u,%u,%lu,%.1f,%.1f,%d,%d,%d,%d,%d,%d", (unsigned)++telSeq, (unsigned)lastCmdSeq, (unsigned long)millis(),
           yawDeg, gzDps, appliedVl, appliedVr, appliedGrip, (int)liftPos, vbat, flags);
  sendFrame(p, out);
}

void updateLed(bool failsafe) {
  uint32_t t = millis();
  bool on;
  if (failsafe && armed) on = (t / 100) % 2;           // fast blink: armed but no commands
#if BT_MODE != BT_NONE
  else if (btHasClient()) on = true;                   // solid: wireless client connected
#endif
  else on = (t % 1000) < 100;                          // short flash each second: waiting
  digitalWrite(LED_PIN, on);
}

// ---------------------------------------------------------------- Arduino
void setup() {
  Serial.begin(115200);
  int outs[] = {ENA, IN1, IN2, ENB, IN3, IN4, LED_PIN};
  for (int i = 0; i < 7; i++) pinMode(outs[i], OUTPUT);
  setMotors(0, 0);
  servoGrip.setPeriodHertz(50); servoGrip.attach(SERVO_GRIP, 500, 2400); servoGrip.write(gripOpenDeg);
  if (strcmp(ROBOT_TYPE, "tower") == 0) {
    servoLift.setPeriodHertz(50); servoLift.attach(SERVO_LIFT, 500, 2400); servoLift.write(LIFT_MIN_DEG);
  }
  gyroBegin();
#if BT_MODE != BT_NONE
  btReady = btBegin();
#endif
  Serial.printf("\nREC robot firmware %s (%s) ready. Wireless: %s '%s' %s. Gyro: %s\n", FW_VERSION, ROBOT_TYPE, BT_KIND, BT_NAME,
                BT_MODE == BT_NONE ? "" : (btReady ? "started" : "FAILED to start"), gyroOk ? "found" : "NOT found (check SDA 21 / SCL 22)");
  printStatus();
  lastLoopUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;
  pump(ports[0]);
  pump(ports[1]);
  gyroUpdate();
  bool failsafe = (millis() - lastCmdMs) > failsafeMs;
  bool live = armed && !failsafe;
  applyOutputs(live, dt);
  updateLed(failsafe);
#if BT_MODE != BT_NONE
  bool client = btHasClient();
  if (client != btClientSeen) {
    btClientSeen = client;
    Serial.println(client ? "[bt] laptop connected" : "[bt] laptop disconnected");
    if (!client && armed && ports[1].lastRx && (!ports[0].lastRx || (int32_t)(ports[1].lastRx - ports[0].lastRx) > 0)) {
      armed = false; cmdVl = cmdVr = 0; setMotors(0, 0);   // the wireless link was the one in charge and it is gone: disarm
    }
  }
  if (!client && millis() - lastWaitPrint > 5000) {      // while waiting, say how to find the robot
    lastWaitPrint = millis();
    Serial.println("[bt] waiting for the laptop. Type  status  to see the name and address.");
  }
#endif
  Port* p = activePort();
  if (p && millis() - lastTelMs >= p->telPeriod) {
    lastTelMs = millis();
    sendTelemetry(*p);
  }
}
