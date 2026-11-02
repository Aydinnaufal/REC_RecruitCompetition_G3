# Dashboard v3.1

Run `python controller_dashboard_v3_1.py` (or `--demo`). Replaces `controller_dashboard_v3.py` (deleted). Other v3 files keep their names:
`robot_sim3.py`, `sim_tower.py`, `sim_physics.py`, `dash_ui.py`, `test_sim3.py` (20 tests), `test_sim2.py` (18). Still needs the v2/original files
(`robot_sim.py`, `robot_sim2.py`, `xbox_input.py`, `controller_viewer.py`, `controller_dashboard.py`). Tested headless only, not on real pygame or a real pad.

## Controls (Arcade and Arcade + Snap)
| Input | Action |
|---|---|
| Left stick Y | forward / back |
| Left stick X | turn, **only while LT is held** (without LT it does nothing) |
| RT | speed cap: not pressed = 20 cm/s, fully pressed = 30 cm/s, in between is proportional |
| D-pad | slow nudge, not affected by RT: up/down = 5 cm/s forward/back, left/right = slight turn (0.6 rad/s, about 35 deg/s) |
| Right stick X | gripper, both robots. Centre or left = open / released. Right = closes, as far as you push (hold it at the position you want). |
| Right stick Y | tower only: box up / down, how far you push = how fast (7 cm/s at full) |
| LB / RB | snap-turn 90 (Arcade + Snap) |
| Y / B / A / X | face up / right / down / left of the zero |
| START | new zero heading |

Tank mode: left Y / right Y are the wheels, so the tower box moves with LB (down) / RB (up) instead.

## Things I decided (tell me if you want them different)
- You said the tower ignores LT for the gripper. I used the same right-stick-X gripper control on the simple robot too, since LT is now the turn enable.
- Right stick X centred = gripper open. So to keep holding a cube, keep the stick pushed right (as you described). Left does the same as centre, since there is nothing more open than open.
- The tower pad (0-3 cm) closes proportionally, so half way push = pad half way out. The simple claw does the same.
- A D-pad press overrides the sticks while held.
