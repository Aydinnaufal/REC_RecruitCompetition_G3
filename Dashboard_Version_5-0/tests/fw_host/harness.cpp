// V_5_0: also builds with -DHAVE_BLE_STUB (BT_MODE = BT_BLE), stdin "bledrop" = the BLE link drops (traffic is discarded), "bleup" = it can come back.
// Runs the firmware's setup() / loop() on a PC. fd 3 = the Bluetooth link (a pty). stdin: "gz N" sets the fake gyro raw value. stdout: "PINS ..." when an output changes.
#include "Arduino.h"
#include "Wire.h"
#include "ESP32Servo.h"
#include <fcntl.h>
#include <string>
#ifdef HAVE_BLE_STUB
#include "BLEDevice.h"
BLEServer* g_ble_server = nullptr; int g_ble_adv_started = 0; bool g_ble_connected = false; bool g_ble_dead = false;
#endif
int pin_val[64]; int fake_gz_raw = 0; int servo_deg[2] = {-1, -1}; int servo_count = 0;
HardwareSerial Serial; TwoWire Wire;
void setup(); void loop();
int main() {
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  setup();
  std::string in; int last[10] = {-9, -9, -9, -9, -9, -9, -9, -9, -9, -9};
  for (;;) {
    loop();
#ifdef HAVE_BLE_STUB
    ble_stub_poll();
#endif
    char b[64]; ssize_t r = read(0, b, sizeof b);
    if (r > 0) { in.append(b, r); size_t p; while ((p = in.find('\n')) != std::string::npos) { std::string l = in.substr(0, p); in.erase(0, p + 1);
      if (l.rfind("gz ", 0) == 0) fake_gz_raw = atoi(l.c_str() + 3); if (l == "quit") return 0;
#ifdef HAVE_BLE_STUB
      if (l == "bledrop") ble_stub_drop();
      if (l == "bleup") g_ble_dead = false;
#endif
 } }
    else if (r == 0) return 0;
    int cur[10] = {pin_val[25], pin_val[26], pin_val[27], pin_val[23], pin_val[32], pin_val[33], servo_deg[0], servo_deg[1], 0, 0};
    if (memcmp(cur, last, sizeof cur)) { memcpy(last, cur, sizeof cur);
      printf("PINS ena=%d in1=%d in2=%d enb=%d in3=%d in4=%d grip=%d lift=%d\n", cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], cur[6], cur[7]); fflush(stdout); }
    usleep(300);
  }
}
