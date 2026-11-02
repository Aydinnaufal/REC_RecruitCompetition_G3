// REC robot firmware skeleton V_4_0: speaks the "rec1" protocol of the laptop dashboard (see COMMS_ARCHITECTURE_V_4_0.md).
// NOT COMPILED OR TESTED ON HARDWARE. Check every pin and library call against your board before powering motors.
// Board: ESP32 DevKit, Arduino-ESP32 core 3.x.  Libraries: ESP32Servo.  Bluetooth Classic serial (appears as a COM port on the laptop) + USB serial.
// Pin map is taken from perfboard_build_notes.md: ENA 25, IN1 26, IN2 27, IN3 32, IN4 33, ENB 23, servo1 18, servo2 19, MPU6050 SDA 21 / SCL 22, battery sense 34.
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <Wire.h>

BluetoothSerial SerialBT;
Servo servoGrip, servoLift;

const int ENA = 25, IN1 = 26, IN2 = 27, ENB = 23, IN3 = 32, IN4 = 33, SERVO1 = 18, SERVO2 = 19, VBAT_PIN = 34;
const uint32_t FAILSAFE_MS = 300, TEL_PERIOD_MS = 50;       // 20 Hz telemetry
const int GRIP_OPEN = 90, GRIP_CLOSED = 30;                  // tune on the real gripper
const char* FW = "rec-esp32", *VER = "V_4_0", *ROBOT = "simple";   // set ROBOT to "tower" for the tower robot

// ---- state
int cmdVl = 0, cmdVr = 0, cmdGrip = 0, cmdLift = 0; bool armed = false;
uint16_t lastCmdSeq = 0, telSeq = 0;
uint32_t lastCmdMs = 0, lastTelMs = 0;
float yaw = 0, gz = 0;                 // from the gyro code in robot_phase1_plan.md: gyroBegin() / gyroUpdate() fill these (yaw CCW positive)
bool gyroOk = false;
Stream* replyTo = &Serial;             // answer on the port the last valid frame came from

// ---- helpers
uint8_t xorSum(const char* s, size_t n) { uint8_t c = 0; for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i]; return c; }

void sendFrame(const char* body) {      // body = "TYPE,f1,f2"
  char tail[8]; snprintf(tail, sizeof tail, "*%02X\n", xorSum(body, strlen(body)));
  replyTo->print('$'); replyTo->print(body); replyTo->print(tail);
}

void motor(int en, int a, int b, int pct) {       // pct -100..100
  int pwm = constrain(abs(pct), 0, 100) * 255 / 100;
  digitalWrite(a, pct > 0); digitalWrite(b, pct < 0); analogWrite(en, pwm);
}
void setMotors(int l, int r) { motor(ENA, IN1, IN2, l); motor(ENB, IN3, IN4, r); }

void applyOutputs(bool live) {
  if (!live) { setMotors(0, 0); return; }                     // failsafe or disarmed: stop (grip and lift hold their last position)
  setMotors(cmdVl, cmdVr);
  servoGrip.write(map(cmdGrip, 0, 100, GRIP_OPEN, GRIP_CLOSED));
  // tower lift: cmdLift is a speed (-100..100); integrate it into an angle / rack position here
}

// ---- frame handling:  $TYPE,f,f*HH
void handleFrame(char* line) {
  char* star = strrchr(line, '*');
  if (line[0] != '$' || !star) return;
  *star = 0;
  if ((uint8_t)strtol(star + 1, nullptr, 16) != xorSum(line + 1, strlen(line + 1))) return;   // bad checksum: ignore silently
  char* body = line + 1;
  if (!strncmp(body, "CMD,", 4)) {
    int seq, vl, vr, g, l, en;
    if (sscanf(body + 4, "%d,%d,%d,%d,%d,%d", &seq, &vl, &vr, &g, &l, &en) == 6) {
      lastCmdSeq = seq; cmdVl = constrain(vl, -100, 100); cmdVr = constrain(vr, -100, 100);
      cmdGrip = constrain(g, 0, 100); cmdLift = constrain(l, -100, 100); armed = en == 1; lastCmdMs = millis();
    }
  } else if (!strncmp(body, "PING,", 5)) {
    char out[40]; snprintf(out, sizeof out, "PONG,%s,%lu", body + 5, millis()); sendFrame(out);
  } else if (!strncmp(body, "HELLO?", 6)) {
    char out[60]; snprintf(out, sizeof out, "HELLO,1,%s,%s,%s", FW, ROBOT, VER); sendFrame(out);
  } else if (!strncmp(body, "ZERO", 4)) {
    yaw = 0; sendFrame("ACK,ZERO");
  } else if (!strncmp(body, "STOP", 4)) {
    armed = false; cmdVl = cmdVr = 0; sendFrame("ACK,STOP");
  }                                                          // unknown frame types are ignored on purpose (newer laptop, older robot)
}

char lineBuf[120]; size_t lineLen = 0;
void pump(Stream& s) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n') { lineBuf[lineLen] = 0; if (lineLen) { replyTo = &s; handleFrame(lineBuf); } lineLen = 0; }
    else if (c != '\r' && lineLen < sizeof lineBuf - 1) lineBuf[lineLen++] = c;
    else if (lineLen >= sizeof lineBuf - 1) lineLen = 0;     // runaway line: drop it
  }
}

void sendTelemetry() {
  int vbat_mV = (int)(analogReadMilliVolts(VBAT_PIN) * 147.0f / 47.0f);   // divider R1 100k / R2 47k from perfboard_build_notes.md
  bool failsafe = (millis() - lastCmdMs) > FAILSAFE_MS;
  int flags = (armed ? 1 : 0) | (failsafe ? 2 : 0) | (gyroOk ? 4 : 8) | (vbat_mV < 6600 ? 16 : 0);
  char out[120];
  snprintf(out, sizeof out, "TEL,%u,%u,%lu,%.1f,%.1f,%d,%d,%d,%d,%d,%d", ++telSeq, lastCmdSeq, millis(), yaw, gz, cmdVl, cmdVr, cmdGrip, 0, vbat_mV, flags);
  sendFrame(out);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("REC-Robot");
  int p[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int i = 0; i < 6; i++) pinMode(p[i], OUTPUT);
  setMotors(0, 0);
  servoGrip.setPeriodHertz(50); servoGrip.attach(SERVO1, 500, 2400); servoGrip.write(GRIP_OPEN);
  servoLift.setPeriodHertz(50); servoLift.attach(SERVO2, 500, 2400);
  // gyroBegin();  gyroOk = true;     // from robot_phase1_plan.md; keep the robot still for about 1 s
}

void loop() {
  pump(Serial); pump(SerialBT);
  // gyroUpdate();                    // updates yaw and gz every loop
  bool live = armed && (millis() - lastCmdMs) <= FAILSAFE_MS;
  applyOutputs(live);
  if (millis() - lastTelMs >= TEL_PERIOD_MS) { lastTelMs = millis(); sendTelemetry(); }
}
