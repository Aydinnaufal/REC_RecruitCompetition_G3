# Robot Station V 7.0 - fix 2 (read README_V_7_0_1.md for the Tune page itself)

Run `run_dashboard_V_7_0_2.bat` and flash `esp32_tune_V_7_0_2/esp32_tune_V_7_0_2.ino`. The other V_7_0_1 files stay as they were (this version still uses them).

## What was wrong, what changed
- **A wheel slider moved the servo on pin 18, motors and servos misbehaved.** The first tuning firmware used the ESP32Servo library and `analogWrite` together; both grab the same hardware PWM channel, so every motor change rewrote the servo signal (and the motor signal). Now the firmware drives everything through the ESP32's own PWM with one channel per output: motors on channels 0-3 (4 kHz), servos on channels 4-5 (50 Hz). The ESP32Servo library is no longer needed. Servo pulse: 0 deg = 0.5 ms, 180 deg = 2.4 ms. Limp = no pulses.
- **Lag (600 ms), bad frames and a drop after 2-3 minutes.** The robot sent every reading as 5-6 tiny 20-byte Bluetooth packets, which piled up. It now reads the negotiated MTU and sends each frame as ONE notification (10 per second); if the MTU is small it slows to 4 per second instead of flooding.
- **Battery showed nothing.** The page now always shows what the pin reads. 0.38 V means nothing is feeding GPIO 34: the pack is off or the divider (100k from the 7.4 V bus, 47k to GND, middle to GPIO 34) is not wired. The battery value is also averaged over 8 readings.
- **Motors need the battery.** On USB power alone the L298N has no motor supply: the wheels cannot turn however the PWM looks. Check the dashboard reads about 7 to 8 V before testing the wheels.

## Honest status
Checked on a PC: the firmware compiles against stand-in headers and, driven with the dashboard's own frames, gives separate motor and servo outputs (servo 0 / 90 / 180 deg = 409 / 1187 / 1966 of 16384, unaffected by wheel changes). Not tested on the real ESP32 or over real Bluetooth.
