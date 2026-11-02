# Robot Station V 5.0  (folder: Dashboard_Version_5-0)

V 5.0 fixes "no ports found / cannot connect to the ESP32 over Bluetooth". It does not hide errors any more, it explains them, and it gives you three ways to reach the robot. Everything from V 4.1 (game, monitor, controls, timer, settings) is unchanged.

**Rule from now on:** every version lives in its own folder (`Dashboard_Version_5-0`, then `Dashboard_Version_5-1` ...) and contains everything it needs. This folder runs on its own; the older folders are not needed.

## Why V 4.1 found nothing (what I could and could not confirm)
- V 4.1 swallowed every error while listing ports. If **pyserial** is not installed in the Python that starts the dashboard (very common with a `.venv`), the list is empty and it only said "no ports found". I cannot see your PC, so I cannot prove this was your case. The new **DOCTOR** tab tells you.
- **Bluetooth Classic on Windows** is fragile: Windows must pair the board, then create an *Outgoing* COM port. Often it creates only the *Incoming* one, or none. V 4.1 had no way around that.
- V 5.0 adds **Bluetooth LE**: the dashboard scans, lists `REC-Robot`, and connects. No Windows pairing, no COM port.

## Do this, in this order
**1. Check the PC (2 minutes).** Double-click `run_port_doctor_V_5_0.bat`. It installs pyserial and bleak into your `.venv` (or the Python on your PATH) and prints a report with numbered next steps. It also scans for Bluetooth LE devices. If a step is not clear, paste the report to me.

**2. Flash the ESP32** (`esp32_firmware_V_5_0/esp32_firmware_V_5_0.ino`).
- Arduino IDE: board **ESP32 Dev Module**, Arduino-ESP32 core **3.x**, library **ESP32Servo**.
- Tools > Partition Scheme > **Huge APP (3MB No OTA/1MB SPIFFS)**. Bluetooth makes the sketch too big for the default.
- Line near the top: `#define BT_MODE BT_BLE`. Choices: `BT_BLE` (default, recommended), `BT_CLASSIC`, `BT_NONE` (USB cable only).
- Open the Serial Monitor (115200, Newline). You should see `REC robot firmware V_5_0 ... Wireless: Bluetooth LE 'REC-Robot' started` and a `[status]` line with the Bluetooth address. Type `status` any time to see it again.
- Wheels in the air for the first run.

**3. Prove the board first, over the USB cable (works in any BT_MODE).**
Run `run_dashboard_V_5_0.bat`. LINK tab: Transport = **USB / Bluetooth COM**, press Refresh, pick the CP210x / CH340 port, press **Connect**. Expect CONNECTED within 3 seconds and TX / RX around 20 per second (10 per second for RX over BLE). If this fails the problem is the cable, driver (CP210x or CH340), the port being open in the Arduino Serial Monitor, or the sketch, not Bluetooth. Close the Serial Monitor before connecting.

**4. Bluetooth LE.** Unplug USB if you like (power the robot from its battery). Transport = **Bluetooth LE (scan)**, press **Scan** (5 seconds). `REC-Robot` appears in the BLE DEVICE list and is selected. Press **Connect**. Or press **Auto-find**: it scans and connects by itself.
Do not pair the robot in Windows settings for BLE. If you paired it earlier, remove it there.

**5. Only if BLE is impossible: Bluetooth Classic.** Flash with `BT_MODE BT_CLASSIC`, pair `REC-Robot` in Windows (wait for "Paired"). Then either Transport = USB / Bluetooth COM and pick the *Outgoing* port (the list marks Incoming ports "skip"), or Transport = **Bluetooth Classic (MAC)**: it connects by address without any COM port (the address is in the DOCTOR tab list, or type it in "Bluetooth address by hand"; the firmware prints it with `status`).

**6. Arm.** The robot never receives drive commands until you press **ARM OUTPUT**. Space = E-STOP. Losing the wireless link disarms it, and the robot also stops by itself 400 ms after the last command.

## What is new in the program
| Where | What |
|---|---|
| DOCTOR tab | check pyserial, COM ports, paired Bluetooth devices (with MAC), bleak; numbered next steps; **Save report**; type a COM port or a Bluetooth address by hand |
| LINK tab | Transport: Virtual robot / USB or Bluetooth COM / Bluetooth LE (scan) / Bluetooth Classic (MAC). The second dropdown becomes PORT, BLE DEVICE or PAIRED ROBOT. Scan / Refresh and Auto-find work per transport. Cut cable / Lossy link only show for the virtual robot |
| COM ports | Windows registry is read as a second source; Outgoing / Incoming Bluetooth ports are recognised; the Incoming one is never used or probed |
| Messages | plain sentences with the exact install command for the Python that is running (for example `"...\.venv\Scripts\python.exe" -m pip install pyserial`) |
| Firmware | one sketch, three modes; BLE Nordic-UART service; `status` command; disarms the instant the wireless link drops |
| Settings | remembered in `~/.rec_station_v50.json` (transport, port, typed port, BLE device, address) |

## Files
| File | What |
|---|---|
| `controller_dashboard_V_5_0.py` | the program |
| `comms_V_5_0.py` | link, protocol, serial / BLE / Classic transports, Port Doctor, virtual robot |
| `port_doctor_V_5_0.py` | the same check without a window (`--ble` scans, `--probe COM7` tests one port) |
| `esp32_firmware_V_5_0/esp32_firmware_V_5_0.ino` | robot firmware |
| `run_dashboard_V_5_0.bat`, `run_port_doctor_V_5_0.bat`, `requirements_V_5_0.txt` | installers / launchers |
| `tests/` | `test_V_5_0.py` (44 tests) and the stand-ins it uses; `test_sim2.py`, `test_sim3.py` for the simulator |
| the other `.py` files | unchanged modules from V 3 / V 4 that the program imports (copied so the folder is self-contained) |

Protocol (`rec1`) is unchanged, see `COMMS_ARCHITECTURE_V_4_0.md` in the Project. In BLE mode telemetry is 10 per second instead of 20.

## If it does not connect
| What you see | Likely cause |
|---|---|
| "pyserial is missing" / "bleak is not installed" | run `run_port_doctor_V_5_0.bat`, or the command shown in the DOCTOR tab |
| BLE scan finds devices but none called REC | the robot is off, out of range, or flashed with BT_NONE / BT_CLASSIC |
| "Bluetooth is off or missing on this PC" | switch Bluetooth on in Windows; a desktop PC without a Bluetooth adapter needs a USB dongle |
| "connected, but this device has no REC serial service" | the board runs another sketch, or BT_MODE is not BT_BLE |
| "the robot did not answer in time" | something else is connected to the robot (phone app, another dashboard); only one BLE client is allowed |
| state NO REPLY | link open but the robot is silent: wrong device, old sketch (use Protocol = Legacy for `esp32_remote_test`) |
| "that is the Incoming Bluetooth port" | pick the Outgoing port, or use Bluetooth Classic (MAC) |
| port busy / access denied | close the Arduino Serial Monitor, PuTTY, or a second dashboard |
| `bad` counter rising on USB | baud is not 115200 |

## What was and was not tested
- **Run, 44 of 44, repeated 3 times, headless:** framing, handshake, failsafe and disarm rules, Auto-find, the Port Doctor (missing pyserial, registry-only ports, Incoming / Outgoing ports, paired-device parsing, all the verdicts), the Bluetooth LE transport and scanner against a pretend robot, Classic-by-address over a socket pair, the dashboard logic and every screen drawn in every transport, and the real `.ino` compiled on a PC in three modes (USB only, Classic, BLE) talking to the real Python link code. The sketch compiles with no warnings under `g++ -Wall`.
- **Stand-ins, not the real thing:** this test machine could not install `pyserial` or `bleak`, so a tiny pyserial replacement and a fake bleak were used (they live in `tests/testshim`; if your real packages are installed, the tests use those instead). pygame was also a drawing stand-in, so text widths in the real window are unchecked.
- **Not tested at all:** real Windows Bluetooth (BLE scan, Classic sockets, pairing), the real ESP32 BLE and Bluetooth stacks (the sketch was compiled against stand-in headers that I wrote from my memory of the Arduino-ESP32 API: a compile error on your core, for example around `BLEServerCallbacks`, is possible), the MPU6050, motors and servos on the real robot, and a real gamepad. If the Arduino IDE shows an error, send me the first red line.
- BLE throughput on a real radio is untested: telemetry is chunked into 20 byte notifications and limited to 10 per second to stay safe.
- Low battery only shows a flag; it does not cut the motors.
