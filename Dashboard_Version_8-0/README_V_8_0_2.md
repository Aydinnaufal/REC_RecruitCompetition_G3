# V 8.0 - dashboard V_8_0_1 + robot firmware esp32_robot_V_8_0_2

Run `run_dashboard_V_8_0_1.bat`. Flash `esp32_robot_V_8_0_2/esp32_robot_V_8_0_2.ino` (replaces `esp32_robot_V_8_0_1`). Not tested on the real robot: the firmware control loop was run on a PC against a simple wheel/gyro model, and the dashboard against a protocol stand-in.

## Why it felt delayed, and moved without a button
The wheel commands sent to the robot came from the SIMULATED robot, which has its own lag and acceleration limit, and the firmware then ramped them again. After you released the stick the real wheels kept going for about half a second. Now the sticks go straight to the wheels (the simulation is only the picture). The firmware ramp is faster too (0 to full in about 0.15 s, stopping in about 0.05 s), and commands go out at 30 per second instead of 20. Speed is still capped at 20 cm/s (67 %) until you pull RT.

## Gyro PID and the unequal wheels (firmware)
- Heading hold is now a PID: Kp (strength), Ki (learning), Kd (damping). It starts when both sticks agree and the robot really drives, after the last turn settles.
- The integral term becomes a learned balance (how much stronger one wheel is, in % of speed). It works forwards and backwards, is kept in flash when the robot stands still, and is applied even when the hold is off.
- Manual trims: left / right wheel gain and left / right start PWM (the lowest power that still turns that wheel). The start PWM begins as the value you found on the Tune page.
- Snap turns (Arcade + Snap mode) now run on the robot: the dashboard sends the target heading, the robot turns to it with its gyro and stops within 1.5 deg. Before, the snap was only the simulation spinning.

## Dashboard
- Removed: CUBE tile and OUTPUT / SAFE tile (Game and Monitor). OUTPUT is already the red ARMED badge in the top bar.
- New tiles: GYRO HOLD (OFF / READY / LOCK / TURN) and GYRO (OK / CAL / LOST).
- The compass, the 3D robot and the heading numbers now follow the robot's gyro whenever the robot is connected (label under the compass: ROBOT GYRO or SIMULATION). Re-zero heading (START button) sets the new zero.
- New tab GYRO HOLD (right column): state, yaw / target / error / correction / balance, live Kp Ki Kd, wheel gains, start PWM, Hold on/off, Flip gyro direction, Reset balance. All saved in the robot.

## First run, in this order
1. Flash the firmware, wheels in the air. Power on, keep it still 1 s.
2. Connect, open the GYRO HOLD tab. Spin the robot by hand to the left: yaw must go UP. If it goes down, press Flip gyro direction (no reflash needed). This decides everything else.
3. Arm, drive on the floor with the left stick straight. If it still curves a lot, watch Balance: it should move away from 0 and the curve should shrink within a few seconds. You can also lower the faster wheel's gain (try 0.95).
4. If it wobbles, lower Kp (1.2 to 0.6). If it sways slowly, lower Ki. Reset balance after you change a motor or a wheel.
5. Snap turns: Settings tab, Drive mode = Arcade + Snap, then Y / B / A / X or LB / RB. If it overshoots or stops short, tell me the numbers.
