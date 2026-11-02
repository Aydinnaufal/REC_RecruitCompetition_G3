# V 8.0.4 (dashboard controller_dashboard_V_8_0_3.py + firmware esp32_robot_V_8_0_4)

New files only. Everything else is as in V 8.0.2 / 8.0.3.
Needs: controller_dashboard_V_8_0_3.py, sim_V_8_0_2.py, render_V_8_0_2.py (the other modules are unchanged: comms_V_8_0_2, ui_V_8_0_1, input_V_8_0_1). Start with run_dashboard_V_8_0_3.bat.

## What is new
1. **BACK button = ARM / DISARM.**
   - Arming only works when connected, telemetry is fresh, and the sticks and D-pad are released (so the robot never jumps).
   - Disarming always works.
2. **Auto-reconnect** (on by default, Settings -> Auto-reconnect).
   - If the link drops, the dashboard retries every 2 s. A link stuck without data for 8 s is restarted.
   - A reconnected link is always DISARMED; press BACK to arm again.
   - The Disconnect button turns it off until you press Connect again.
3. **Speed without RT** (Settings, cm/s, default 20 = 67 %). Set 30 for full power without holding RT. Saved with the settings.
4. **Straight line, forward and backward** (firmware _4). The gyro hold now engages whenever both wheel commands are equal within 3 % (turn stick at 0 or LT turn off) and the speed is above 10 %. A deliberate small turn is not fought.

## Why the motor driver shows only 3-4 V
- The L298N loses about 1.5-2.5 V inside (bipolar outputs).
- The output is PWM; a multimeter shows the average = duty x (battery - drop).
- Without RT the speed cap is 67 %, so about 0.67 x (8 - 2) = 4 V. That matches your reading.
- Hold RT, or set "Speed without RT" to 30, then measure again: about 6 V.
- For more real voltage and torque: use a MOSFET driver (TB6612FNG or DRV8833), a fully charged pack, and watch for battery sag.

## Tested
Only on PC stand-ins (41 dashboard checks, firmware compiled against a plant model: straight, reverse, snap, failsafe). Not tested on the real robot.
