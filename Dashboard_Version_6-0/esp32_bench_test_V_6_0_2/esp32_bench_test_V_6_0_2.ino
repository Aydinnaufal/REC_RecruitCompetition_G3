// REC robot - ALL-IN-ONE BENCH TEST + TUNING  (V_6_0_2)   motors, 2 servos, gyro/accel (MPU6050), battery.
// Board "ESP32 Dev Module" (Arduino-ESP32 core 3.x), library ESP32Servo. Serial Monitor 115200 baud, line ending "Newline". Type h for help.
// Pins: L298N IN1 26 IN2 27 | IN3 32 IN4 33 (ENA/ENB jumpers stay ON, speed = PWM on the IN pins) | servo1 18, servo2 19 | MPU6050 SDA 21 SCL 22 | battery divider 100k/47k on 34
#include <Wire.h>
#include <ESP32Servo.h>

const int IN1 = 26, IN2 = 27, IN3 = 32, IN4 = 33, SP1 = 18, SP2 = 19, VB = 34, LED = 2;
const uint32_t MOTOR_TMO = 3000;                       // motors stop by themselves this long after the last command

// ---- tuning values: change with  set <name> <value>, then copy the printed line into the robot firmware
float minpwm = 0, invl = 0, invr = 0, lo1 = 0, hi1 = 180, lo2 = 0, hi2 = 180, ratio = 147.0f / 47.0f;
struct { const char* n; float* v; } T[] = {{"minpwm", &minpwm}, {"invl", &invl}, {"invr", &invr}, {"lo1", &lo1}, {"hi1", &hi1},
                                           {"lo2", &lo2}, {"hi2", &hi2}, {"ratio", &ratio}};

Servo sv[2]; int ang[2] = {-1, -1}; bool sweep = false, stream = false, imuOk = false;
int pct[2], pwmv[2]; uint32_t lastCmd, tImu, tStream, tSweep; uint8_t who;
float g[7], bias, yaw;                                   // g: ax ay az (g)  temp (C)  gx gy gz (deg/s)

// ---------------------------------------------------------------- motors
void drive(int s, int p) {                              // s 0 = left , 1 = right; p -100..100 %
  p = constrain(p, -100, 100); pct[s] = p; lastCmd = millis();
  int a = s ? IN3 : IN1, b = s ? IN4 : IN2, d = (s ? invr : invl) > 0 ? -p : p;   // driver always enabled: PWM goes on the active IN pin
  pwmv[s] = p ? map(abs(p), 1, 100, (int)minpwm, 255) : 0;
  analogWrite(a, d > 0 ? pwmv[s] : 0); analogWrite(b, d < 0 ? pwmv[s] : 0);
}
void ramp(int s) {                                      // raise PWM slowly; send any key the moment the wheel starts turning
  int a = s ? IN3 : IN1, b = s ? IN4 : IN2;
  analogWrite(b, 0); Serial.println("ramping, press Enter when the wheel starts...");
  while (Serial.available()) Serial.read();
  int p = 0;
  for (; p < 255 && !Serial.available(); p++) { analogWrite(a, p); delay(120); }
  analogWrite(a, 0); while (Serial.available()) Serial.read();
  minpwm = p; Serial.printf("wheel started at PWM %d -> minpwm set to %d\n", p, p);
}
void stopAll() { drive(0, 0); drive(1, 0); }

// ---------------------------------------------------------------- servos (attached on the first command, so they stay limp at boot)
void srv(int i, int deg) {
  float lo = i ? lo2 : lo1, hi = i ? hi2 : hi1; deg = constrain(deg, (int)lo, (int)hi);
  if (ang[i] < 0) sv[i].attach(i ? SP2 : SP1, 500, 2400);
  sv[i].write(deg); ang[i] = deg;
}

// ---------------------------------------------------------------- MPU6050 (raw registers, no library)
uint8_t rd(uint8_t r) { Wire.beginTransmission(0x68); Wire.write(r); Wire.endTransmission(false); Wire.requestFrom(0x68, 1); return Wire.read(); }
void imuInit() {
  Wire.begin(21, 22); Wire.setClock(400000);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); imuOk = Wire.endTransmission() == 0;
  who = imuOk ? rd(0x75) : 0;
}
bool imuRead() {
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false) || Wire.requestFrom(0x68, 14) != 14) return imuOk = false;
  int16_t r[7]; for (auto& x : r) { uint8_t h = Wire.read(); uint8_t l = Wire.read(); x = (h << 8) | l; }
  for (int i = 0; i < 3; i++) { g[i] = r[i] / 16384.0f; g[4 + i] = r[4 + i] / 131.0f; }
  g[3] = r[3] / 340.0f + 36.53f; return imuOk = true;
}
void zeroGyro() {                                       // keep the robot still for one second
  float s = 0; int n = 0; stopAll();
  for (int i = 0; i < 200; i++) if (imuRead()) { s += g[6]; n++; delay(5); }
  bias = n ? s / n : 0; yaw = 0; Serial.printf("gyro zeroed, bias %.2f deg/s (%d samples)\n", bias, n);
}

// ---------------------------------------------------------------- status
float battV() { return analogReadMilliVolts(VB) * ratio / 1000.0f; }
void status() {
  Serial.println("----------------------------------------------");
  Serial.printf("MOTOR L %+4d%%  pwm %3d  (IN1 26 IN2 27)\n", pct[0], pwmv[0]);
  Serial.printf("MOTOR R %+4d%%  pwm %3d  (IN3 32 IN4 33)\n", pct[1], pwmv[1]);
  for (int i = 0; i < 2; i++) { Serial.printf("SERVO %d  ", i + 1); if (ang[i] < 0) Serial.println("limp (no command yet)"); else Serial.printf("%d deg  (limits %d..%d)\n", ang[i], (int)(i ? lo2 : lo1), (int)(i ? hi2 : hi1)); }
  if (imuOk) Serial.printf("GYRO    OK id 0x%02X  yaw %+.1f deg  rate %+.2f deg/s  temp %.1f C\nACCEL   x %+.2f y %+.2f z %+.2f g  %s\n",
                           who, yaw, g[6] - bias, g[3], g[0], g[1], g[2], fabsf(g[2]) > 0.8f ? "(flat)" : "(tilted)");
  else Serial.println("GYRO    NOT FOUND - check 3V3, GND, SDA 21, SCL 22");
  float v = battV(); Serial.printf("BATTERY %.2f V  (%d mV on pin 34)  %s\n", v, (int)analogReadMilliVolts(VB), v < 1 ? "no battery sense" : v < 6.4f ? "LOW" : "ok");
}
void tuning() { Serial.print("TUNING "); for (auto& t : T) Serial.printf("%s=%g  ", t.n, *t.v); Serial.println(); }
void help() {
  Serial.println("l <-100..100>  left motor %      r <-100..100>  right motor %      m <l> <r>  both      s  stop all");
  Serial.println("1 <deg>  servo 1      2 <deg>  servo 2      sw  sweep both servos on/off");
  Serial.println("?  status      p  stream status on/off      z  zero gyro (keep still)      v  battery      auto  self-test");
  Serial.println("ramp l|r  find the lowest PWM that turns the wheel (sets minpwm)      set <name> <value>  change a tuning value      set  list them");
  Serial.println("names: minpwm invl invr (1 = reverse that motor) lo1 hi1 lo2 hi2 (servo limits) ratio (battery divider)");
  Serial.println("motors stop by themselves 3 s after the last command - resend to keep them running");
}
void autotest() {
  Serial.println("AUTO TEST: wheels off the ground! L fwd, L back, R fwd, R back, then both servos");
  int seq[4][2] = {{50, 0}, {-50, 0}, {0, 50}, {0, -50}};
  for (auto& q : seq) { Serial.printf("  motors L %d R %d\n", q[0], q[1]); drive(0, q[0]); drive(1, q[1]); delay(900); stopAll(); delay(300); }
  for (int i = 0; i < 2; i++) { int lo = i ? lo2 : lo1, hi = i ? hi2 : hi1; Serial.printf("  servo %d %d -> %d -> %d\n", i + 1, (lo + hi) / 2, lo, hi);
    srv(i, (lo + hi) / 2); delay(600); srv(i, lo); delay(700); srv(i, hi); delay(700); srv(i, (lo + hi) / 2); delay(400); }
  imuRead(); status(); Serial.println("AUTO TEST done");
}

void cmd(char* s) {
  char c[8] = "", w[12] = ""; float a = 0, b = 0;
  if (!strncmp(s, "set", 3)) { int n = sscanf(s, "set %11s %f", w, &a); if (n == 2) for (auto& t : T) if (!strcmp(w, t.n)) *t.v = a; tuning(); return; }
  int n = sscanf(s, "%7s %f %f", c, &a, &b);
  if (n < 1) return;
  if (!strcmp(c, "l")) drive(0, a); else if (!strcmp(c, "r")) drive(1, a);
  else if (!strcmp(c, "m")) { drive(0, a); drive(1, b); } else if (!strcmp(c, "s")) stopAll();
  else if (!strcmp(c, "1")) { sweep = false; srv(0, a); } else if (!strcmp(c, "2")) { sweep = false; srv(1, a); }
  else if (!strcmp(c, "sw")) sweep = !sweep;
  else if (!strcmp(c, "z")) zeroGyro();
  else if (!strcmp(c, "v")) Serial.printf("battery %.2f V\n", battV());
  else if (!strcmp(c, "p")) stream = !stream;
  else if (!strcmp(c, "ramp")) { sscanf(s, "ramp %7s", w); ramp(w[0] == 'r'); }
  else if (!strcmp(c, "auto")) autotest();
  else if (!strcmp(c, "h")) help();
  else if (!strcmp(c, "?")) status();
  else Serial.println("unknown command, h = help");
}

void setup() {
  int o[] = {IN1, IN2, IN3, IN4, LED}; for (int p : o) { pinMode(p, OUTPUT); digitalWrite(p, 0); }
  Serial.begin(115200); delay(300); analogReadResolution(12); analogSetPinAttenuation(VB, ADC_11db);
  imuInit(); if (imuOk) zeroGyro();
  Serial.println("\nREC bench test V_6_0_1 ready"); help(); tuning(); status();
}

void loop() {
  static char buf[40]; static int len;
  while (Serial.available()) { char ch = Serial.read(); if (ch == '\n' || ch == '\r') { if (len) { buf[len] = 0; cmd(buf); } len = 0; } else if (len < 39) buf[len++] = ch; }
  uint32_t t = millis();
  if ((pct[0] || pct[1]) && t - lastCmd > MOTOR_TMO) { stopAll(); Serial.println("(motors auto-stopped)"); }
  if (imuOk && t - tImu >= 10) { float dt = (t - tImu) * 1e-3f; tImu = t; if (imuRead()) yaw += (g[6] - bias) * dt; }
  if (sweep && t - tSweep >= 20) { tSweep = t; float ph = (t % 4000) / 2000.0f; if (ph > 1) ph = 2 - ph; srv(0, lo1 + (hi1 - lo1) * ph); srv(1, lo2 + (hi2 - lo2) * ph); }
  if (stream && t - tStream >= 500) { tStream = t; status(); }
  digitalWrite(LED, (t / 500) % 2 && (pct[0] || pct[1]));        // on-board LED blinks while a motor runs
}
