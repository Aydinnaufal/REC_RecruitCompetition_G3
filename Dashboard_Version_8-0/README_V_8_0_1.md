# V 8.0 - full robot firmware (esp32_robot_V_8_0_1)

Dashboard: keep using `run_dashboard_V_7_0_2.bat` (Game mode). Nothing in the dashboard changed. Flash `esp32_robot_V_8_0_1/esp32_robot_V_8_0_1.ino` (board "ESP32 Dev Module", no extra library). Keep `esp32_tune_V_7_0_4` for tuning days.
NOT tested on the real robot, only compiled and run on a PC stand-in. Wheels in the air for the first run.

## What it does
- Xbox -> dashboard -> Bluetooth LE ("REC-Robot", no pairing) or USB -> wheels, grip servo, lift servo (tower only).
- Safety: disarmed on start; no command for 400 ms, a lost link, or STOP = wheels stop at once. Servos hold their last position.
- Soft start: wheels ramp (max 300 %/s up, 600 %/s down) so the battery and the ESP32 are not hit by a sudden current spike. Servos move at most 150 deg/s.
- Gyro: yaw and turn rate go to the dashboard (the yaw dial / plot). Heading hold: when both sticks ask for about the same speed and the robot is really driving, it remembers the heading after 300 ms and nudges the wheels to keep it. It switches off while you turn, stop, or if the gyro is missing or still calibrating.
- Settings in the sketch top: ROBOT_TYPE ("simple" = grip on GPIO 18; "tower" = lift servo on GPIO 19 too), GYRO_SIGN, VBAT_RATIO (0 = no battery divider, battery shows 0), HOLD_KP / HOLD_KD / HOLD_MAX.
- Min PWM, wheel invert and servo limits that you set on the Tune page are kept in flash by the tune firmware and are read by this one. Grip open/closed angles default 90 / 30: change with `CFG,go,<deg>` / `CFG,gc,<deg>` / `CFG,hold,0` (0 turns the heading hold off), saved in flash.
- Serial Monitor (115200, Newline): type `status`. It also prints `Last restart: ...` (BROWNOUT = supply dip).

## First run, in this order
1. Wheels off the ground, battery connected. Power on and leave the robot still for 1 s (gyro calibration; the LED/flag says so).
2. Dashboard -> connect -> Messages should show `$HELLO ... rec-robot` and `$RST ...`.
3. Arm. Push a stick slowly: check both wheels turn the right way (if not: Tune page invert, or swap the motor wires) and the grip servo opens/closes.
4. Gyro sign test: turn the robot by hand about 90 degrees to the LEFT (counter-clockwise from above). Yaw should go to about +90. If it goes to -90, set `GYRO_SIGN = -1` at the top of the sketch (otherwise heading hold would push the wrong way).
5. Drive on the floor at a low speed limit. Heading hold test: drive straight with both sticks equal and nudge the robot sideways by hand; it should swing back. If it wobbles, lower HOLD_KP (1.2 -> 0.6) and HOLD_KD; if it does nothing, raise HOLD_KP. Turn it off with `CFG,hold,0` to compare.

## Mounting the MPU6050 gyro
- Flat and level on the robot's chassis, chip face UP, so the Z axis points straight up. Yaw is measured around Z; if it is tilted, yaw reads too small and picks up roll/pitch.
- Position along the robot: yaw rate is the same anywhere on a rigid body, so no need to be at the exact centre. Choose a rigid spot away from the motors and the L298N heat, ideally over the axle line, and not on the lift or the gripper.
- Fix it with screws on short plastic standoffs (M2/M3), or hard double-sided tape on a flat rigid plate. A bare floppy board or loose foam adds vibration noise. If motor vibration shows in the gz graph, add a thin layer of foam tape under the screws.
- Align the board's X arrow with the robot's front, so the accel readings on the Tune page match the robot's axes.
- Wires: 3V3, GND, SDA to GPIO 21, SCL to GPIO 22, short (under 15 cm), twisted or bundled together, and away from the motor wires. If you have the module's AD0 pin, leave it unconnected or at GND (address 0x68).
- Power it from the ESP32 3V3 pin.
- The gyro measures turning rate, so yaw drifts slowly (a few degrees per minute). The dashboard `ZERO` resets yaw. Do not move the robot during the first second after power-up.
