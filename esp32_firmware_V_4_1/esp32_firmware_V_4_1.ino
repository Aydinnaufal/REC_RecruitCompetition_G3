// REC robot firmware V_4_1 - ESP32 NodeMCU-32S (38 pin), talks to Robot Station V 4.1 over Bluetooth Classic (SPP) or the USB cable.
// Protocol "rec1": text frames  $TYPE,f1,f2*HH\n  (HH = XOR of the characters between $ and *). See COMMS_ARCHITECTURE_V_4_0.md and README_V_4_1.md.
//
// Board : "ESP32 Dev Module" (Arduino-ESP32 core 3.x).  Library: ESP32Servo (Library Manager).  Serial Monitor: 115200, Newline.
//         If the upload says "Sketch too big": Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)". Bluetooth makes the sketch large.
// Needs a classic ESP32 chip (WROOM-32 / NodeMCU-32S). ESP32-S2, S3 and C3 have no Bluetooth Classic and cannot run this sketch.
//
// Pins (same as perfboard_build_notes.md / the L298N wiring):
//   L298N  ENA 25  IN1 26  IN2 27  |  ENB 23  IN3 32  IN4 33        left motor = A, right motor = B
//   Servo  grip 18, lift 19 (the lift servo is only used by the tower robot)
//   MPU6050 SDA 21  SCL 22 (address 0x68)       battery divider 100k / 47k on GPIO 34
//   LED    GPIO 2 (built in): slow blink = waiting for the laptop, solid = Bluetooth connected, fast blink = failsafe
//
// Safety: nothing moves until the laptop sends enable=1, and the motors stop when no valid CMD arrived for FAILSAFE_MS.
// NOT TESTED ON THE REAL ROBOT. The protocol part was tested against the Python dashboard code on a PC. Wheels in the air for the first run.
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ---------------------------------------------------------------- settings you may change
const char* BT_NAME = "REC-Robot";                  // the name Windows shows when you pair
const char* ROBOT_TYPE = "simple";                  // "simple" = claw robot, "tower" = lift + pad robot
const char* FW_NAME = "rec-esp32";
const char* FW_VERSION = "V_4_1";
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
const uint32_t TEL_PERIOD_MS = 50;                      // 20 Hz telemetry
const float VBAT_RATIO = 147.0f / 47.0f;                // divider (100k + 47k) / 47k. Set to 0 if no divider is fitted.

// ---------------------------------------------------------------- state
struct Port {
  Stream* s;
  const char* name;
  uint32_t lastRx;       // millis() of the last valid frame from this port (0 = never)
  char buf[120];
  size_t len;
};
BluetoothSerial SerialBT;
Port ports[2] = {{&Serial, "usb", 0, {0}, 0}, {&SerialBT, "bt", 0, {0}, 0}};
Servo servoGrip, servoLift;

int cmdVl = 0, cmdVr = 0, cmdGrip = 0, cmdLift = 0;
bool armed = false;
int appliedVl = 0, appliedVr = 0, appliedGrip = 0;
float liftPos = 0;                                     // 0..100 %
uint16_t lastCmdSeq = 0, telSeq = 0;
uint32_t lastCmdMs = 0, lastTelMs = 0, lastLoopUs = 0;
bool btReady = false, btClientSeen = false;

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
  if (p.lastRx == 0 || millis() - p.lastRx > 3000) return false;
  if (&p == &ports[1] && !SerialBT.hasClient()) return false;
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

void pump(Port& p) {
  while (p.s->available()) {
    int c = p.s->read();
    if (c < 0) break;
    if (c == '\n') {
      p.buf[p.len] = 0;
      if (p.len) handleFrame(p, p.buf);
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
  else if (SerialBT.hasClient()) on = true;            // solid: Bluetooth client connected
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
  btReady = SerialBT.begin(BT_NAME);
  Serial.printf("\nREC robot firmware %s (%s) ready. Bluetooth '%s': %s. Gyro: %s\n", FW_VERSION, ROBOT_TYPE, BT_NAME,
                btReady ? "started, pair it in Windows" : "FAILED to start", gyroOk ? "found" : "NOT found (check SDA 21 / SCL 22)");
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
  bool client = SerialBT.hasClient();
  if (client != btClientSeen) {
    btClientSeen = client;
    Serial.println(client ? "[bt] laptop connected" : "[bt] laptop disconnected");
  }
  if (millis() - lastTelMs >= TEL_PERIOD_MS) {
    lastTelMs = millis();
    Port* p = activePort();
    if (p) sendTelemetry(*p);
  }
}
