# Robot Station V 6.0 (lighter rebuild)

Everything is in this one folder. Nothing is imported from V3/V4/V5.

## Run
Double-click `run_dashboard_V_6_0_1.bat` (uses `..\.venv` if it exists, installs `requirements_V_6_0.txt`).

## Files
- controller_dashboard_V_6_0_1.py  main program (UI, link tab, settings)
- ui_V_6_0_1.py        widgets and drawing helpers
- input_V_6_0_1.py     Xbox controller reading (loads `xbox_mapping.json` next to it if present; no learn screen)
- sim_V_6_0_1.py       tower robot physics, cubes, snap turning
- render_V_6_0_1.py    3D picture (Game and Monitor)
- comms_V_6_0_1.py     rec1 protocol, USB serial, Bluetooth LE

## Kept
Tower robot sim, Arcade and Arcade+Snap, Game and Monitor modes, LT turn toggle (square indicators), timer, extra camera views, terminal with Send and Clear, saved settings.

## Removed
DOCTOR tab, Classic Bluetooth, virtual robot / cut cable / lossy link, protocol and baud menus, Demo input, learn/save/default map, HELP tab, team choice, Auto-find, Retry, HELLO / PING / Zero-gyro buttons, all tests.

## Changed
- Overlays are true opacity.
- Timer arms and starts on any input (button, D-pad, stick, trigger).
- Link tab widgets stay inside their cards.

## Controls
Space E-STOP, F11 fullscreen, V view, T drive mode, L turn, 0 re-zero, P timer, C cubes, X robot, Z all, ESC quit.
LB/RB snap +-90 deg, Y/B/A/X face directions, START re-zero, LT toggles stick turning, RT speed 20 to 30 cm/s, right stick X = pad, right stick Y = box lift.

## Robot link
- Firmware: `esp32_firmware_V_5_0_3.ino` (protocol unchanged).
- Do NOT pair REC-Robot in Windows Bluetooth settings; the dashboard connects without pairing.
- Output is disarmed on every connect or link loss. Press ARM OUTPUT to drive.

## Honest status
Tested only with a stand-in pygame and fake BLE/serial robot (26 checks pass). Not yet run on real pygame, Windows or hardware. No tests ship; a new test/tuning routine comes later.
