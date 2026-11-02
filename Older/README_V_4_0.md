# Dashboard V 4.0

Run: `python controller_dashboard_V_4_0.py` (needs pygame; `pip install pyserial` for real serial ports).
Keep these in the same folder: `controller_dashboard_V_4_0.py`, `comms_V_4_0.py`, `dash_ui_V_4_0.py`, plus the older `controller_dashboard_v3_2.py`, `robot_sim3.py`, `sim_tower.py`, `sim_physics.py`, `dash_ui.py`, `controller_viewer.py`, `xbox_input.py`, `robot_sim2.py`, `robot_sim.py`.

## What changed
- Layout on a 1440x860 canvas (scales to any window, F11 / F = fullscreen): left column = controller + driving info, centre = viewport + compass and gauges, right = tabbed panel LINK / SETTINGS / HELP.
- Raw axes are now six slim bars stacked vertically with a numbered button grid.
- Gyro under the viewport is a rotating compass (Y/B/A/X on the card, red pointer, gyro rate bar) plus gauge bars.
- All settings, camera/trail/shadow/arrow options, reset buttons and descriptions are in the SETTINGS and HELP tabs; the top bar only keeps Game/Monitor, Robot, Team, link chip, ARMED badge and the timer.
- New LINK tab: connection setup, link status and quality, ARM OUTPUT and E-STOP, robot telemetry, serial terminal.
- Monitor mode can take its picture from the simulation or from robot telemetry (dead reckoning).
- Game, both robots, timer, mouse camera: unchanged.

## Using the link without hardware
LINK tab -> Transport: Virtual robot -> Connect. Then ARM OUTPUT to let the virtual robot follow the sticks; try Cut cable and Lossy link to see STALE and the failsafe. Space = E-STOP.

## Files
| File | Purpose |
|---|---|
| `comms_V_4_0.py` | Link, protocol, transports, virtual robot, telemetry pose (no pygame) |
| `dash_ui_V_4_0.py` | tabs, text input, compass, compact raw panel, drawing helpers |
| `controller_dashboard_V_4_0.py` | the dashboard |
| `test_V_4_0.py` | 19 tests (headless) |
| `COMMS_ARCHITECTURE_V_4_0.md` | communication design and bring-up plan |
| `esp32_firmware_V_4_0.ino` | robot-side skeleton (not compiled) |

## Caveats
Tests ran headless against a PIL stand-in for pygame (19/19 V4, 24/24 sim3, 18/18 sim2). Not yet run on real pygame, a real gamepad, a real serial port or the robot. Check the layout and text input on your machine first.
