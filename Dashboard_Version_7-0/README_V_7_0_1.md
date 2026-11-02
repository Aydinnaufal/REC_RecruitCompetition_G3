# Robot Station V 7.0

V 6.0 plus a **Tune** page: wireless test and tuning of the motors, both servos, the MPU6050 gyro/accelerometer and the battery, with its own firmware.
Everything is in this one folder.

## Run
1. Flash `esp32_tune_V_7_0_1/esp32_tune_V_7_0_1.ino` (ESP32 Dev Module, core 3.x, library ESP32Servo; Serial Monitor 115200 / Newline).
   Wiring is the same as `esp32_bench_test_V_6_0_2` (L298N ENA/ENB jumpers stay ON): IN1 26, IN2 27, IN3 32, IN4 33, servos 18 / 19, MPU6050 SDA 21 / SCL 22 (3V3), battery divider on 34.
2. Double-click `run_dashboard_V_7_0_1.bat`.
3. LINK tab: Scan, Connect (robot name REC-Robot; do NOT pair it in Windows settings). Top bar: **Tune**.
4. Nothing moves until you press **ARM OUTPUT** (LINK tab). Space = E-STOP.

## Tune page
- **Motors**: two speedometers showing the PWM the robot is really sending (needle, %, raw PWM, FWD/REV). Slider per wheel (snaps to 0), Stop, and **Find start power**:
  arm, press it, the wheel speeds up slowly, press again the moment it turns: the PWM at that moment is saved as the robot's MIN_PWM.
- **Servos**: two dials with the allowed range lit, slider, Limp (no signal), Centre, Min / Max, and Sweep both.
- **Gyro + accelerometer**: a 3D board turning like the real sensor with its X / Y / Z axes, yaw / pitch / roll, angular rate and acceleration bars, temperature, chip id, **Zero gyro**.
- **Battery**: charge picture for the 2S pack (6.0 to 8.4 V), volts, percent, per-cell volts, LOW warning at 6.4 V.
- **Tuning values** (kept in the robot's flash, survive power off): start power, servo 1 / 2 limits, battery ratio, reverse left / right wheel. **Save values to a file** writes `tuning_values_V_7_0.txt` with lines to paste into the full robot firmware.

## Safety
New or lost link = disarmed. Disarmed: wheels get zero, servos keep where they are. Arming never starts a wheel (sliders are zeroed). E-STOP zeroes the sliders. The robot stops by itself after 400 ms without a command.

## Game / Monitor
Unchanged from V 6.0 (tower robot simulation, Arcade and Arcade+Snap, Monitor of the real robot with the normal firmware).
The Monitor needs the normal robot firmware; the Tune page needs the tuning firmware (the page says so if the robot runs the other one).

## Protocol additions (rec1)
`CMD,seq,vl,vr,s1,s2,en` (servo degrees, -1 limp, -2 keep) | `TCFG,name,value` | `CAL` | robot: `TUN,seq,ack,ms,pwmL,pwmR,s1,s2,gx,gy,gz,ax,ay,az,temp,yaw,vbat_mv,flags,id`, `TCFG,minpwm,invl,invr,lo1,hi1,lo2,hi2,ratio`.

## Honest status
Tested only with a stand-in pygame, a fake tuning robot in Python (20 checks) and the firmware compiled on a PC against stand-in headers and driven with the dashboard's own frames. Not yet run on real pygame, Windows or the ESP32. Wheels in the air first.
