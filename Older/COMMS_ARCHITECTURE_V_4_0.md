# Laptop <-> Robot communication architecture (V_4_0)

Status: the laptop side is built and tested against a virtual robot. The robot side (`esp32_firmware_V_4_0.ino`) is a skeleton that has NOT been compiled or run on hardware.

## 1. Goals
1. The robot is never driven by accident: nothing moves until you press ARM OUTPUT, and a dead link stops the robot on its own.
2. The same dashboard works with a virtual robot (no hardware), an ESP32 over USB or Bluetooth, or an Arduino with a legacy letter protocol.
3. The link is visible: state, quality, rates, latency, loss, battery, and a raw serial terminal.
4. Each layer can be replaced without touching the others.

## 2. Layers (laptop side, all in `comms_V_4_0.py`)

```
 Dashboard (controller_dashboard_V_4_0.py)    draws panels, owns the buttons
        |  Command(vl, vr, grip, lift, enable)         ^  telemetry dict, link stats, log lines
        v                                              |
 Link      state machine, 20 Hz heartbeat, handshake, 1 Hz ping, stats, auto-reconnect, terminal log
        |  Profile: 'rec1' (framed) | 'legacy' (F/B/L/R/S, C/O)
        v
 FrameParser   bytes -> lines -> checked frames   (rec1 only; other lines go to the terminal as plain text)
        |
        v
 Transport     SerialTransport (pyserial: USB or Bluetooth COM port) | VirtualRobotTransport (FakeRobot, fault injection)
```

Rules: the dashboard never touches a port; the Link never draws; a Transport only moves bytes (open, write, read, close).

## 3. Link state machine
DISCONNECTED -> CONNECTING (open the port) -> HANDSHAKE (send `HELLO?` every 0.5 s, wait up to 2 s) -> CONNECTED.
CONNECTED -> STALE if no valid frame for 0.6 s (comes back to CONNECTED on the next frame). Any open or write failure -> ERROR, then auto-retry if enabled. The legacy profile skips the handshake because those boards say nothing.

## 4. rec1 protocol
Text lines, so they can be read in the terminal and typed by hand.

`$TYPE,field,field*HH\n` where `HH` is 2 hex digits of the XOR of every character between `$` and `*`.

Laptop -> robot
| Frame | Fields | Sent |
|---|---|---|
| `CMD` | seq, vl, vr, grip, lift, enable | every 50 ms (20 Hz), always, even when disarmed (zeros) |
| `PING` | seq | 1 Hz |
| `HELLO?` | none | during handshake and from the terminal button |
| `CFG` | key, value | on demand (reserved: gains, limits) |
| `ZERO` | none | zero the gyro heading |
| `STOP` | none | E-STOP and on disconnect |

`vl`, `vr` are -100..100 (% of full speed), `grip` 0..100 (0 open, 100 closed), `lift` -100..100 (tower lift rate), `enable` 1 only when ARM OUTPUT is on.

Robot -> laptop
| Frame | Fields | Rate |
|---|---|---|
| `HELLO` | proto, firmware, robot (`simple`/`tower`), version | on request |
| `TEL` | tseq, ack, t_ms, yaw, gz, vl, vr, grip, lift, vbat_mV, flags | 20 Hz |
| `PONG` | seq, t_ms | per PING |
| `ACK` | what | per ZERO / STOP |
| `LOG` | text | optional |

`ack` is the last CMD seq the robot accepted, so the laptop can measure command latency. `yaw` is degrees CCW positive, `gz` deg/s.
Flags: ARMED 1, FAILSAFE 2, GYRO_OK 4, GYRO_CAL 8, LOWBAT 16 (below 6600 mV on a 2S pack).
Unknown frame types must be ignored (a newer laptop can talk to an older robot). A frame with a bad checksum is dropped and counted (`bad`).

## 5. Safety model
- Output starts DISARMED. Disarmed means the laptop still sends CMD at 20 Hz but with zeros and `enable=0`.
- Robot failsafe: no valid CMD for 300 ms -> motors off. This is the real safety net, independent of the laptop.
- E-STOP (Space): sends STOP, disarms, and the robot also drops `armed`. Re-arming is a deliberate click.
- Disconnect sends STOP (or `S` on legacy).
- Reconnecting never re-arms.
- LOWBAT is shown, not acted on (decide later whether the robot should cut the motors).

## 6. Bandwidth
CMD ~30 bytes x 20 Hz = 0.6 kB/s, TEL ~70 bytes x 20 Hz = 1.4 kB/s. Roughly 2 kB/s total = 16 kbit/s.
- USB serial at 115200 baud (11.5 kB/s): plenty.
- ESP32 Bluetooth Classic (SPP): plenty. It shows as a COM port after pairing.
- Arduino Nano + HC-05 at 9600 baud (~960 B/s): too slow for 20 Hz TEL. Use the legacy profile, or lower the rates (CMD 10 Hz, TEL 5 Hz) and raise the HC-05 to 38400 or 115200 with AT commands.
- ESP32 BLE and WiFi are not covered; the Transport interface is the place to add them.

## 7. Legacy profile
Matches `esp32_remote_test.ino`: single letters F/B/L/R/S for drive and C/O when the gripper command changes; 400 ms failsafe on the board. No telemetry, so Monitor "Robot" picture and the battery panel show nothing.

## 8. Testing without hardware
Pick Transport = Virtual robot. `FakeRobot` answers HELLO/PING/CMD, honours the 300 ms failsafe, calibrates a gyro bias, sags the battery, sends TEL at 20 Hz. Buttons "Cut cable" and "Lossy link" inject faults, and latency/jitter/loss are constructor parameters. `test_V_4_0.py` covers all of this.

## 9. Bring-up plan on real hardware
1. Wheels off the ground (or motor supply disconnected). Flash the skeleton. Open the Serial Monitor: it should print nothing, then answer a typed `$HELLO?*HH` line.
2. Dashboard, Transport = Serial, pick the port, Protocol = rec1, Connect. Expect HELLO in the terminal, state CONNECTED, TEL rate ~20/s.
3. Move the sticks while DISARMED: robot stays still, TEL `vl/vr` stay 0. Press ARM: TEL follows the sticks. Unplug USB / walk away: motors stop within ~0.3 s.
4. Check yaw sign and scale by rotating the robot by hand 90 degrees; fix the sign in firmware if the compass turns the wrong way.
5. Calibrate speed: measure cm/s at 100% so Monitor dead reckoning (assumes 30 cm/s) is right; add encoders later if possible.
6. Only then put the robot on the floor, with a hand on E-STOP.

## 10. Open questions
- Real link: USB cable, ESP32 Bluetooth Classic, or Arduino + HC-05? (decides profile and baud)
- Which sensors really exist (gyro only, or also encoders)? Telemetry fields can be trimmed.
- Tower lift: speed command (as now) or absolute position? Needs a limit switch or known stroke.
- Should LOWBAT cut the motors?
- Competition rules on wireless: is Bluetooth/WiFi allowed during the run?
