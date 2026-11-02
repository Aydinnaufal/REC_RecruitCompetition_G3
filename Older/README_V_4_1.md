# Robot Station V 4.1 (dashboard + ESP32 Bluetooth firmware)

V 4.1 is V 4.0 with a Bluetooth link that can be set up, diagnosed and trusted. Game, Monitor, controls and settings are unchanged.

## Files
Put these next to the V 4.0 files (V 4.1 still imports `controller_dashboard_v3_2.py`, `robot_sim3.py`, `dash_ui_V_4_0.py` and the older ones).

| File | What it is |
|---|---|
| `controller_dashboard_V_4_1.py` | the dashboard. Run: `python controller_dashboard_V_4_1.py` (needs `pip install pygame pyserial`) |
| `comms_V_4_1.py` | link layer, Bluetooth / USB transport, Auto-find, virtual robot (replaces `comms_V_4_0.py`) |
| `esp32_firmware_V_4_1.ino` | firmware for the ESP32 NodeMCU-32S (38 pin): Bluetooth + USB, motors, servos, gyro, battery |
| `test_V_4_1.py` + `fw_host/` | 20 PC tests, including the real firmware logic talking to the real link code (optional) |

## What changed from V 4.0
| Problem in V 4.0 | V 4.1 |
|---|---|
| `HELLO?` was sent once. A robot that was not ready yet was never noticed and the link looked "connected" | `HELLO?` repeats every 0.5 s. No answer in time gives state **NO REPLY** (port open, robot silent) and it keeps asking until the robot answers |
| Opening a Bluetooth COM port could freeze the window for seconds | opening runs in a background thread; Bluetooth ports retry 3 times with plain-language errors |
| Opening a port could pulse DTR / RTS and reset a USB ESP32 | DTR / RTS are kept low when opening |
| "Reconnecting never re-arms" was written down but not done | any error, reconnect or disconnect sets the output to DISARMED (tested) |
| A full Bluetooth buffer raised an error and dropped the link | one command is dropped, the next one is 50 ms later |
| No help finding the right COM port | **Auto-find** tries every Bluetooth / USB port and connects to the one that answers; Bluetooth ports are marked in the list |
| Port, baud and protocol forgotten each run | remembered in `~/.rec_station_v41.json` |
| Robot failsafe 300 ms | 400 ms, so a late Bluetooth burst does not stop the robot |

Protocol is unchanged (`rec1`, see `COMMS_ARCHITECTURE_V_4_0.md`); the V 4.1 firmware answers `HELLO,1,rec-esp32,simple,V_4_1`.

## 1. Flash the ESP32
1. Arduino IDE, board **ESP32 Dev Module**, core 3.x. Install the **ESP32Servo** library.
2. If the upload says "Sketch too big": Tools > Partition Scheme > **Huge APP (3MB No OTA/1MB SPIFFS)**.
3. Open `esp32_firmware_V_4_1.ino`, check the pins at the top against your wiring (ENA 25, IN1 26, IN2 27, ENB 23, IN3 32, IN4 33, servos 18 / 19, MPU6050 SDA 21 / SCL 22, battery sense 34), upload.
4. Serial Monitor at 115200: you should see `REC robot firmware V_4_1 (simple) ready. Bluetooth 'REC-Robot': started`.
5. The built-in LED: short flash every second = waiting for the laptop, solid = Bluetooth client connected, fast blink = armed but no commands.
6. Wheels in the air for the first run. Servo power comes from the 5 V buck rail, not from USB or 3V3.

Tune in the sketch: `INVERT_LEFT` / `INVERT_RIGHT` (a wheel turns the wrong way), `GYRO_SIGN` (yaw goes the wrong way), `MIN_PWM`, `gripOpenDeg` / `gripClosedDeg`. Set `ROBOT_TYPE` to `"tower"` for the lift robot.

## 2. Pair it with Windows (once)
1. Power the robot. Windows Settings > Bluetooth & devices > Add device > Bluetooth > **REC-Robot**. No PIN.
2. Settings > Bluetooth & devices > Devices > More Bluetooth settings > **COM Ports** tab. Note the port marked **Outgoing** (for example COM5). The Incoming one is not used.
3. If you changed the Bluetooth name or re-flashed with another name, remove the device and pair again.
4. The ESP32 accepts one Bluetooth client at a time. Close PuTTY or the Arduino serial monitor on that COM port first.

## 3. Connect from the dashboard
1. LINK tab, Transport = **USB / Bluetooth COM**, press **Refresh**.
2. Either pick the Outgoing COM port and press **Connect**, or press **Auto-find** and let it search.
3. Expected: CONNECTING (Bluetooth takes up to a few seconds) > HANDSHAKE > **CONNECTED**, the terminal shows `$HELLO 1 rec-esp32 simple V_4_1`, TX and RX near 20 per second, telemetry fills in.
4. Move the sticks: nothing moves. Press **ARM OUTPUT** to let the robot follow the sticks. **Space** or E-STOP disarms and stops.
5. Test the failsafe with the wheels in the air: walk the laptop away or turn Bluetooth off; the wheels stop within about 0.4 s.

## If it does not connect
| What you see | Likely cause |
|---|---|
| `port not found` | wrong COM number, or the robot is not paired: pair again and press Refresh |
| `port busy or access denied` | another program has the port (Arduino Serial Monitor, PuTTY, a second dashboard) |
| `Bluetooth did not answer` | robot off, out of range, or already connected to another device |
| state **NO REPLY** | the port opened but nothing answers: wrong COM port (try the other one or Auto-find), sketch not running, or the old `esp32_remote_test` sketch (use Protocol = Legacy) |
| Auto-find finds nothing | the robot is not paired, or pyserial is missing (`pip install pyserial`) |
| `bad` counter rising | baud mismatch on a USB cable (use 115200); Bluetooth ignores the baud number |
| wheel turns the wrong way | `INVERT_LEFT` / `INVERT_RIGHT` in the sketch |

## What was and was not tested
- **Tested, 20 of 20 repeated 5 times, on real pyserial 3.5:** framing, handshake retries, NO REPLY recovery, disarm on error / reconnect / disconnect, stale detection, Auto-find (including ports that fail to open or never open), and the real `.ino` logic compiled on a PC talking to the real `Link` and `SerialTransport` through a pseudo-terminal: HELLO, 20 Hz telemetry, battery reading, motor pin states for forward / spin, claw angle, 400 ms failsafe, STOP, gyro zeroing and sign, garbage and wrong-checksum input, reconnect.
- **Tested with a pygame stand-in, 8 of 8 repeated 5 times:** the V 4.1 dashboard logic on top of the real V3 / V4 modules: settings, connect / arm / E-STOP through the virtual robot, Auto-find, Stop scan, hint text fits in every state, Link tab buttons fit, and every screen draws in every link state.
- **Not tested:** the look of the window in real pygame (text width with the real fonts), the real Bluetooth stack and pairing, the ESP32 itself (the sketch was compiled against stand-in headers, not the real core, so a compile error on your core is possible), the real MPU6050, motors and servos, and real controller input. Send me the first error or screenshot and I will fix it.
- Not covered: low battery only shows a flag (it does not cut the motors); the lift servo is only used when `ROBOT_TYPE` is `"tower"`.
