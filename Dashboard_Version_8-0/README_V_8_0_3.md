# V 8.0 - PID tab (dashboard V_8_0_2) + firmware esp32_robot_V_8_0_3

Run `run_dashboard_V_8_0_2.bat`. Flash `esp32_robot_V_8_0_3/esp32_robot_V_8_0_3.ino` (replaces _2). The other dashboard files (ui, sim, input, render) are unchanged, `comms_V_8_0_2.py` is new. Not tested on the real robot: checked on a PC against the firmware's control loop and a protocol stand-in.

## The new PID tab (top bar: Game, Monitor, Tune, PID)
- The GYRO HOLD tab beside LINK and SETTINGS is gone; the right column is back to LINK and SETTINGS (ARM OUTPUT stays on LINK).
- Orientation: top view of the robot turned to its real gyro heading (up = heading at power-up, turning left raises yaw), the amber line and wedge = the heading the PID wants, wheels glow green forward / amber reverse.
- Three live graphs of the last 12 s (10 samples per second from the robot): yaw vs target; error and correction (each at its own full scale: +-20 deg, +-25 %); left / right wheel commands. Pause and Clear buttons.
- PID settings, all saved in the robot: hold Kp / Ki / Kd, left / right wheel gain, left / right start PWM, and the snap-turn Turn strength / Turn damping (new in firmware _3). Plus Gyro hold on/off, Flip gyro direction, Reset balance, Re-zero heading.
- Step tests (need ARM OUTPUT): Straight 3 s at 50 %, and Turn +45 / -45 / +90 / -90 deg. Any stick movement, E-STOP, disarming or leaving the page stops a test. When it ends the page prints the result: biggest error and drift for the straight run; overshoot, settling time and final error for a turn.
- Sticks still drive the robot while this page is open.

## How to tune
1. Spin the robot by hand to the left: yaw must go up. If not, Flip gyro direction.
2. Straight test: the error should stay inside +-2 deg. Slow lean: raise Ki. Quick wobble: lower Kp or raise Kd. If one wheel is clearly faster, lower its gain (0.95).
3. Turn tests: overshoot: lower Turn strength or raise Turn damping. Stops short: raise Turn strength.
