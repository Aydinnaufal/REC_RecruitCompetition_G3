import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the program modules live one folder up
"""
bench_robots.py - which robot is faster? A scripted 'perfect driver' plays the same job on both robots in the game simulation.

Job: bring N blue cubes from their pillars (the column at x = -83, from the south: y = -13, 0, 13) into the blue start & retry zone
(the 20 x 20 cm box at x = -82, y = -92). The timer starts when the robot leaves the start box with the first command and stops
when the last cube is inside the box, on the floor and let go.

The driver only uses the controller inputs the dashboard has (left stick Y, the Y/B/A/X face buttons of Arcade + Snap, right stick X / Y,
RT at full speed = 30 cm/s). It is a tidy, repeatable driver with no human delay, so read the numbers as the best case for each robot.
    python bench_robots.py            # N = 1, 2, 3
"""
import math, sys
import xbox_input as xi
import robot_sim3 as r3
import sim_physics as ph

DT = 1 / 60.0
ZONE = (-82.0, -92.0)
LANE_X = -62.0                                   # simple robot: the corridor east of the pillar columns
COLUMN_X = -83.0
CUBE_YS = [-13.0, 0.0, 13.0]


class Pilot:
    def __init__(self, robot):
        self.sim = r3.RobotSim3(robot, 'blue')
        for _ in range(120): self.sim.world.step_cubes(DT)      # cubes settle on their pillars
        self.t = 0.0
        self.grip, self.lift = 0.0, 0.0                         # held commands: right stick X / Y

    def frame(self, ly=0.0, button=None, rt=1.0, grip=None, lift=None):
        s = xi.ControllerState()
        s.ly, s.rt = ly, rt
        s.rx = self.grip if grip is None else grip
        s.ry = self.lift if lift is None else lift
        if button: s.buttons[button] = True
        self.sim.drive(s, DT)
        self.t += DT
        if self.t > 600: raise RuntimeError('timeout')

    def wait(self, sec, **k):
        for _ in range(int(round(sec / DT))): self.frame(**k)

    def face(self, button):
        """Snap-turn with Y / B / A / X (north / east / south / west of the start heading)."""
        self.frame(button=button)
        self.frame()
        while self.sim.snapping: self.frame()
        self.wait(0.05)

    def go(self, axis, target):
        """Drive straight (forward or reverse) until the robot's x or y equals target. Heading must be one of the four directions."""
        sim = self.sim
        f = -math.sin(sim.th) if axis == 'x' else math.cos(sim.th)
        for _ in range(int(60 / DT)):
            pos = sim.x if axis == 'x' else sim.y
            rem = (target - pos) / f
            if abs(rem) < 0.25 and abs(sim.speed) < 1.5: break
            self.frame(ly=math.copysign(max(0.08, min(1.0, abs(rem) / 9.0)), rem))
        self.wait(0.05)

    def set_grip(self, v, sec): self.grip = v; self.wait(sec)

    def set_lift(self, v, sec): self.lift = v; self.wait(sec); self.lift = 0.0

    def delivered(self, ys):
        w = self.sim.world
        n = 0
        for c in w.cubes:
            if abs(c.x - ZONE[0]) < 10 and abs(c.y - ZONE[1]) < 10 and not c.held and c.z < 7 and abs(c.vx) + abs(c.vy) < 1:
                n += 1
        return n


def run_simple(n, log=None, err=0.0):
    p = Pilot('simple')
    ys = CUBE_YS[:n]
    drops = [(-68.0, -86.0), (-76.0, -86.0), (-72.0, -90.5)][:n]       # robot x, y when it lets go (cube is 10 cm ahead, to the west)
    p.face('B'); p.go('x', LANE_X)
    for cy, (dx, dy) in zip(ys, drops):
        p.face('Y'); p.go('y', cy)
        p.face('X'); p.go('x', COLUMN_X + 10.5 + err)
        p.set_grip(1.0, 0.9)
        p.go('x', LANE_X)
        p.face('A'); p.go('y', dy + 0.0)
        p.face('X'); p.go('x', dx)
        p.set_grip(0.0, 0.5)
        p.go('x', LANE_X)
        if log: log('cube at y=%.0f delivered by %.1f s' % (cy, p.t))
    p.wait(1.0)
    return p


def run_tower(n, log=None, err=0.0):
    p = Pilot('tower')
    left = list(CUBE_YS[:n])
    drops = {1: [-90.0], 2: [-90.0], 3: [-96.0, -84.0]}[n]
    trips = [left[i:i + 2] for i in range(0, n, 2)]
    for trip, dy in zip(trips, drops if n == 3 else drops):
        p.go('y', trip[0] + err)
        p.set_lift(-1.0, 1.0)
        p.set_grip(1.0, 0.8)
        if len(trip) == 2:
            p.lift = 0.0
            p.set_lift(1.0, 0.55)                       # up to 4.1 cm: cube clear of the pillars
            p.go('y', trip[1] + err)
            p.grip = 0.0; p.wait(0.4)                   # drop cube 1 on cube 2
            p.set_lift(-1.0, 0.9)                       # box down over both
            p.set_grip(1.0, 0.8)
        p.set_lift(1.0, 0.55)
        p.go('y', dy)
        p.set_lift(-1.0, 0.7)
        p.grip = 0.0; p.wait(0.5)
        p.set_lift(1.0, 1.0)
        if log: log('trip %s delivered by %.1f s' % (trip, p.t))
    return p


def main():
    for n in (1, 2, 3):
        res = {}
        for name, fn in (('simple', run_simple), ('tower', run_tower)):
            p = fn(n)
            res[name] = (p.t, p.delivered(None))
        print('N=%d  simple %5.1f s (%d in zone)   tower %5.1f s (%d in zone)' % (n, res['simple'][0], res['simple'][1], res['tower'][0], res['tower'][1]))


def sensitivity(n=2):
    """Stop short / long by err cm in the direction of travel when lining up on a cube."""
    print('N=%d, stopping error when lining up on a cube' % n)
    for err in (0.0, 0.5, 1.0, 1.5, 2.0, -0.5, -1.0, -1.5, -2.0):
        out = []
        for name, fn in (('simple', run_simple), ('tower', run_tower)):
            try:
                p = fn(n, err=err)
                out.append('%s %5.1f s (%d/%d)' % (name, p.t, p.delivered(None), n))
            except Exception as e:
                out.append('%s failed (%s)' % (name, type(e).__name__))
        print('  err %+.1f cm   ' % err + '   '.join(out))


if __name__ == '__main__':
    main()
    if '--sens' in sys.argv: sensitivity()
