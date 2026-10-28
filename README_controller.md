# Xbox controller mapping and viewer

Files: `xbox_input.py` (reads the pad, reusable later by the bridge and the dashboard) and `controller_viewer.py` (picture + raw numbers).

## Run
1. `pip install pygame`
2. Windows: plug in the Xbox 360 wireless receiver, let Windows install its driver, press the receiver's sync button, then the pad's sync button.
3. `python controller_viewer.py`
   - No pad yet? `python controller_viewer.py --demo` plays fake input so you can see the screen.

## What you see
- Left: the controller. Buttons light up, the sticks move, the triggers fill from the bottom, the D-pad arms light.
- Right: the RAW axes, hats and buttons with their numbers, plus a log of every press and release.
- Anything the mapping does not know about still lights up in the raw panel (with a `?`), so you can find it.

## Keys
M learn mapping (guided, about a minute) · S save to `xbox_mapping.json` · R default mapping · D demo on/off · SPACE skip a step · ESC quit

## If a button is wrong
Raw numbers depend on the operating system and driver. Press M and follow the prompts. Press S to save: next start loads your saved mapping. The Guide (big X) button is often not reported on Windows; skip it with SPACE.

## Use it from other programs
```python
from xbox_input import XboxController
pad = XboxController()
s = pad.poll()      # s.buttons['A'], s.lx, s.ly, s.rx, s.ry (up and right are +), s.lt, s.rt (0..1), s.dpad (x, y)
```

## Tested / not tested
I tested the mapping and learn-mode logic with simulated controllers, and checked the drawing layout with a stand-in for pygame. I could not run real pygame or a real controller here, so the first run on your computer is the real test. Send me a screenshot or the error and I will fix it.
