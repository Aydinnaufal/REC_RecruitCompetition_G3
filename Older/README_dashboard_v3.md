# Dashboard v3

Run: `pip install pygame`, then `python controller_dashboard_v3.py` (or `--demo` with no controller).
v2 files are untouched (`controller_dashboard_v2.py` still works). Tests: `python test_sim3.py` (15) and `python test_sim2.py` (18).
Tested headless with a PIL stand-in for pygame only. Nothing has run on real pygame or a real pad yet.

## What is new
| Request | What v3 does |
|---|---|
| Fullscreen and resizing | Window is resizable; the whole UI is drawn on a 1360x800 canvas that scales to any window (aspect kept). F11 / F or the Fullscreen button; ESC leaves fullscreen. |
| Better UI | Toolbar: dropdowns for Mode, Robot, Drive, Team, View; buttons for Re-zero, Reset cubes/robot/all, Demo, Learn/Save/Default map, Fullscreen. Keys still work. |
| Monitor mode | Mode dropdown -> Monitor. Robot only, no arena. It rotates in place, the floor grid slides under it, claw/pad/lift animate. Drawn from `sim.snapshot()` (a plain dict), so later `app.set_feed(fn)` can supply real ESP data instead. |
| Pillar | 5 mm high (4x4 cm top kept). Cubes sit at z 0.5 cm. |
| Gripper position | Claw moved to cube height (z 0.9-3.1 cm, pillar is only 0.5 cm), starts 1 cm in front of the chassis, tips reach 14 cm in front of the axle, whole robot fits the 20x20 cm limit. |
| START | Records the current heading as the new zero (heading only). Y/B/A/X face up/right/down/left of that zero. A yellow arrow on the floor shows the zero direction. |
| Sticks | Left stick Y = forward/back only. Right stick X = turn. Tank mode unchanged. |
| Views | Game: Arena, Chase, Top, Front, Right side, Left side. Monitor: Angled, Front, Right, Left, Back, Top. |
| Spawn / reset | Robot starts in its team's start box (Blue bottom-left facing up, Red top-right facing down). Reset cubes = cubes only. Reset robot = robot back in its box. Reset all = both. |
| Two robots | Simple (1 claw, LT) and Tower. Switching robot puts it back in the start box. |

## Added after first feedback
- **Mouse camera:** drag in the picture to orbit (switches to the Free view), scroll to zoom in every view, Reset camera button to undo.
- **Shadow** is now a see-through patch (was solid near-black, which hid the wheels). Wheels are lighter with 4 bright spokes and a hub so turning is easy to see.
- **Trail / Shadow / Arrow:** each has an on/off switch and a - / + strength (5% steps) under the picture.

## Tower robot (assumed numbers, please check)
- Box: 3.8 cm inside (30 mm cube + 4 mm each side), walls 2.5 mm, 8 cm tall, open at top and bottom, sliding in a 4.3 cm hole in the chassis.
- Chassis underside at 4.2 cm (a cube on a pillar is 3.5 cm tall), chassis top 8 cm, mast and servo up to 20 cm.
- Lift: D-pad up/down, hold to move, 7 cm/s. Box bottom 0.6 to 12 cm. Starts raised (7 cm) so it drives over cubes.
- Pad: one pad on the inside wall, LT 0-100% pushes sideways, 3 cm travel. It presses the cube against the far wall to hold it. LT back to 0 releases.
- Stacking: a cube released over another lands on it (3.5 cm). Lowering the box over both and gripping holds the bottom cube; the top one rides on it.

## Not decided yet (my questions)
1. **One by one dispensing.** With one low pad only the bottom cube is gripped. Releasing lets the whole stack down. Options: (a) a second pad higher up that holds the top cube while the lower one is released; (b) lift the box off so the stack stays on the floor, then re-grip only the top; (c) a shelf/finger under the stack. Which one is real?
2. **Simple gripper complaint.** I assumed it floated above the cube / sat too far from the body. What was wrong exactly?
3. Tower sizes above (chassis height, box height, lift range) are guesses.
4. Is the box open at the bottom (a sleeve)? Assumed yes.
