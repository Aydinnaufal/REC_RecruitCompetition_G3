# esp32_tune_V_7_0_4 - what changed from _3

Flash this instead of _3. Keep using run_dashboard_V_7_0_2.bat (no dashboard change).

1. Restart reason. After connecting, the dashboard Messages log shows `$RST,<reason>,<seconds>`, and the Serial Monitor (115200) prints `Last restart: <reason>.`
   - BROWNOUT = the supply dipped (battery missing, or USB alone cannot feed the servo). This is the usual cause of "STALE after turning a servo".
   - poweron = normal start. CRASH / watchdog = a code problem, send me the line.
2. Servo speed limit: servos glide at most 150 deg/s (SERVO_DPS at the top of the file) instead of jumping, so the current spike is smaller.

## Checks for "servo goes STALE, motors do not run"
- Battery card should read about 7-8 V. It read 0.44 V = battery not connected to the divider on GPIO 34 (100k/47k), so the battery is not what powers things.
- Measure the L298N motor-supply terminal with a multimeter: it needs the 7.4 V battery. USB cannot run motors.
- Servos need their own 5-6 V supply with grounds joined to the ESP32 GND. Do not power them from the ESP32 3.3 V pin.
- Not tested on real hardware, only on a PC stand-in.
