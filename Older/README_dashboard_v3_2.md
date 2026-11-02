# Dashboard v3.2

Run `python controller_dashboard_v3_2.py` (or `--demo`). Replaces `controller_dashboard_v3_1.py`. Files: `controller_dashboard_v3_2.py`, `robot_sim3.py`,
`sim_tower.py`, `sim_physics.py`, `dash_ui.py`, `test_sim3.py` (24 tests), `test_sim2.py` (18), `bench_robots.py`. It still needs `robot_sim.py`, `robot_sim2.py`,
`xbox_input.py`, `controller_viewer.py`, `controller_dashboard.py`. Tested headless only (stand-in for pygame), not on a real pad.

## What changed from v3.1
- **Timer** (top right, optional). `Timer` button shows / hides it. `Reset` = back to 0:00.0 and wait; it starts on the first controller button press (any button or D-pad).
  `Restart` = back to 0 and run now. `P` or a click on the time pauses / resumes. Turning the timer off clears it.
- **LT is a toggle**: press once = left stick X turns, press again = off. A button on the middle panel (`Turn: ON/OFF`, key `L`) shows and flips it. Reset robot sets it off.
- **Arcade + Snap** is the default drive mode (also after switching robot).

## Controls
Left stick Y forward / back; left stick X turns when Turn is ON; RT 20 cm/s (released) to 30 cm/s (full); D-pad slow nudge; right stick X gripper
(right = close, as far as you push; centre / left = open); right stick Y tower box up / down; LB / RB snap 90; Y/B/A/X face up/right/down/left of the zero; START = new zero.
Tank mode: wheels on left Y / right Y, tower box on LB (down) / RB (up).

## Which robot is faster? (`python bench_robots.py`, add `--sens` for the error test)
Job: bring blue cubes from the pillar column at x = -83 (y = -13, 0, 13) into the blue start & retry box. A scripted driver (full 30 cm/s, snap turns, no human delay) plays both robots.

| Cubes | Simple (1 claw) | Tower |
|---|---|---|
| 1 | 16.3 s | 10.9 s |
| 2 | 30.7 s | 14.9 s |
| 3 | 45.9 s | 27.7 s |

- The tower wins and the gap grows with more cubes, because it drives straight over the cubes (no going round), stacks two and carries both in one trip. The simple robot needs one round trip per cube (about 15 s each).
- Even for one cube the tower is faster (no turning around to line up from the side).
- The catch is accuracy. When lining up on a cube, the simple robot still delivered 2 of 2 with up to 2 cm error either way. The tower delivered 2 of 2 up to 1.5 cm error long or 1.0 cm short and failed with 2 cm error. A human on a joystick will be slower than this driver, and the tower needs more careful lining up.
- Caveats: it is my model of both robots. Assumptions are the ones in the README for v3 (box 3.8 cm, lift 7 cm/s, pad 14 cm/s), and the tower cannot dispense the stack one by one yet, so both cubes go into the box together. If the rules need cubes placed one by one in different spots, the tower needs the extra lift / re-grip time, and its lead shrinks.
