"""sim_V_8_0_1.py - the simulated tower robot, its arena and its controls. No pygame, so it can be tested anywhere.

Units: cm and seconds. World: x right, y up the map, z up. Heading th = 0 faces +y and grows counter-clockwise; heading_deg is the compass
number on screen (0 = the zero direction, turning right makes it bigger). Robot frame: x to the robot's right, y forward, origin = middle of
the wheel axle.

    Robot    pose, wheels, snap turns, controller input     World    arena walls, pillars, cubes, collisions     Tower    the lifting box + pad
Controller (ControllerState from input_V_8_0_1.py)
    left stick Y   forward / back          left stick X   turn, only while TURN is on (press LT once = on, again = off)
    RT             speed: not pressed = 20 cm/s, fully pressed = 30 cm/s      D-pad   slow nudge: up / down = forward / back, left / right = slight turn
    right stick X  pad: right = push the cube against the far wall (how far you push = how far it moves), centre or left = release
    right stick Y  box up / down (how far you push = how fast)
    Arcade + Snap: LB / RB snap-turn 90 degrees left / right,  Y / B / A / X face up / right / down / left of the zero heading,  START = new zero
"""
import math
from collections import deque

# ---------------------------------------------------------------- numbers
G = 981.0                    # gravity, cm/s^2
CUBE_S = 3.0                 # cube side: 30 mm
PILLAR_W, PILLAR_H = 4.0, 0.5
WALL_T, WALL_H = 1.2, 10.0
FIELD = 105.0                # half of the 210 cm field
FRICTION, RESTITUTION, Z_EPS = 0.35, 0.25, 0.01
PILLAR_XS, PILLAR_YS = (-95.0, -83.0), (-13.0, 0.0, 13.0)          # blue side; red side is the mirror image
DROP_HEIGHT = 10.0

TRACK = 15.0                 # distance between the wheels
MAX_SPEED, SLOW_CM = 30.0, 20.0
TURN_GAIN = 0.55
ACCEL, DECEL, LAG = 150.0, 240.0, 9.0
W_MAX = 2.0 * MAX_SPEED / TRACK
SNAP_ACCEL = 30.0
ALIGN_TOL = math.radians(0.3)
MAX_SUB = 1.0 / 60.0
DPAD_SPEED, DPAD_TURN, LT_TURN = 5.0, 0.6, 0.5
SPAWN = (-82.0, -92.0, 0.0)                                         # centre of the blue start box, facing up the map
MODES = ['arcade', 'arcade_snap']
MODE_LABEL = {'arcade': 'Arcade', 'arcade_snap': 'Arcade + Snap'}

BODY_Z = (4.2, 8.0)                                                  # chassis underside clears a cube on a pillar
PARTS = [(0.0, 5.25, 6.0, 2.75, *BODY_Z), (0.0, -5.95, 6.0, 2.05, *BODY_Z), (-4.25, -0.7, 1.75, 3.2, *BODY_Z), (4.25, -0.7, 1.75, 3.2, *BODY_Z),
         (0.0, -3.3, 1.0, 0.6, 8.0, 20.0),                           # rack column
         (-7.5, 0.0, 1.2, 3.0, 0.0, 6.0), (7.5, 0.0, 1.2, 3.0, 0.0, 6.0)]   # wheels

# tower: a box on a rack and pinion, one servo pad inside it
BOX_IN, BOX_WALL = 3.8, 0.25
BOX_OUT = BOX_IN + 2 * BOX_WALL
BOX_H = 8.0
LIFT_MIN, LIFT_MAX, LIFT_START, LIFT_RATE = 0.6, 12.0, 7.0, 7.0
PAD_MAX, PAD_RATE, PAD_Y_HALF, PAD_Z0, PAD_Z1 = 3.0, 14.0, 1.0, 0.3, 2.7
TOWER_RELEASE_EPS = 0.10
FAR_X = -BOX_IN / 2 + CUBE_S / 2


def clamp(v, lo, hi): return max(lo, min(hi, v))


def wrap_pi(a):
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return math.pi if a <= -math.pi + 1e-12 else a


# ---------------------------------------------------------------- boxes and collisions
class OBB:
    """Oriented box: centre (cx, cy), half sizes hx / hy, rotation a, z range z0..z1."""
    __slots__ = ('cx', 'cy', 'hx', 'hy', 'ux', 'uy', 'z0', 'z1', 'ex', 'ey', 'tag')

    def __init__(self, cx, cy, hx, hy, a, z0, z1, tag=None):
        self.cx, self.cy, self.hx, self.hy = cx, cy, hx, hy
        self.ux, self.uy = math.cos(a), math.sin(a)
        self.z0, self.z1, self.tag = z0, z1, tag
        self.ex = abs(self.ux) * hx + abs(self.uy) * hy
        self.ey = abs(self.uy) * hx + abs(self.ux) * hy


def z_overlap(a, b): return a.z0 < b.z1 - Z_EPS and b.z0 < a.z1 - Z_EPS


def aabb_apart(a, b): return abs(a.cx - b.cx) > a.ex + b.ex or abs(a.cy - b.cy) > a.ey + b.ey


def sat(a, b):
    """Separating axis test. Returns (depth, nx, ny) with n pointing from a to b, or None when they do not overlap."""
    dx, dy = b.cx - a.cx, b.cy - a.cy
    best = None
    for ax, ay in ((a.ux, a.uy), (-a.uy, a.ux), (b.ux, b.uy), (-b.uy, b.ux)):
        ra = a.hx * abs(a.ux * ax + a.uy * ay) + a.hy * abs(-a.uy * ax + a.ux * ay)
        rb = b.hx * abs(b.ux * ax + b.uy * ay) + b.hy * abs(-b.uy * ax + b.ux * ay)
        d = dx * ax + dy * ay
        ov = ra + rb - abs(d)
        if ov < 0:
            return None
        if best is None or ov < best[0]:
            best = (ov, ax if d >= 0 else -ax, ay if d >= 0 else -ay)
    return best


class Cube:
    def __init__(self, cid, team, x, y, z=0.0):
        self.id, self.team = cid, team
        self.x, self.y, self.z, self.th = x, y, z, 0.0
        self.vx = self.vy = self.vz = 0.0
        self.held = self.pinned = False
        self.dx = self.dy = 0.0                # how far it was pushed this step
        self.px, self.py = x, y                # position at the start of the step
        self.on, self.off = None, (0.0, 0.0)   # the cube it rests on, and where on it

    def obb(self): return OBB(self.x, self.y, CUBE_S / 2, CUBE_S / 2, self.th, self.z, self.z + CUBE_S, self)


class World:
    """Walls, pillars and 12 cubes (a cube released over another one lands on it)."""

    def __init__(self):
        self.statics, self.pillars, self.cubes = [], [], []
        self._build()
        self.reset_cubes()

    def _build(self):
        S = self.statics
        def wall(x0, y0, x1, y1):
            hx = abs(x1 - x0) / 2 + WALL_T / 2 if y0 == y1 else WALL_T / 2
            hy = abs(y1 - y0) / 2 + WALL_T / 2 if x0 == x1 else WALL_T / 2
            S.append(OBB((x0 + x1) / 2, (y0 + y1) / 2, hx, hy, 0.0, 0.0, WALL_H, 'wall'))
        for a, b in WALLS:
            wall(a[0], a[1], b[0], b[1])
        big = FIELD + 20
        for cx, cy, hx, hy in ((-FIELD - 5, 0, 5, big), (FIELD + 5, 0, 5, big), (0, -FIELD - 5, big, 5), (0, FIELD + 5, big, 5)):
            S.append(OBB(cx, cy, hx, hy, 0.0, 0.0, WALL_H, 'boundary'))
        for sgn in (1, -1):
            for px in PILLAR_XS:
                for py in PILLAR_YS:
                    p = OBB(sgn * px, sgn * py, PILLAR_W / 2, PILLAR_W / 2, 0.0, 0.0, PILLAR_H, 'pillar')
                    S.append(p); self.pillars.append(p)

    def reset_cubes(self):
        self.cubes, n = [], 0
        for sgn, team in ((1, 'B'), (-1, 'R')):
            for py in PILLAR_YS:
                for px in PILLAR_XS:
                    n += 1
                    self.cubes.append(Cube(n, team, sgn * px, sgn * py, PILLAR_H + DROP_HEIGHT))

    def free_cubes(self): return [c for c in self.cubes if not c.held]

    def support_height(self, x, y):
        for p in self.pillars:
            if abs(x - p.cx) <= p.hx and abs(y - p.cy) <= p.hy:
                return PILLAR_H
        return 0.0

    def support_of(self, c):
        """(height, cube underneath or None): the floor, a pillar top, or the top of a cube under this one's centre."""
        h, base = self.support_height(c.x, c.y), None
        for o in self.cubes:
            if o is c or o.z + CUBE_S > c.z + 0.05:
                continue
            if abs(c.x - o.x) <= CUBE_S / 2 and abs(c.y - o.y) <= CUBE_S / 2 and o.z + CUBE_S > h:
                h, base = o.z + CUBE_S, o
        return h, base

    def mark(self):
        for c in self.cubes:
            c.px, c.py = c.x, c.y

    def step_cubes(self, dt):
        """Gravity, friction and landing. Call after resolve(). A cube that was pushed already moved; its speed is taken from the push."""
        for c in sorted(self.free_cubes(), key=lambda q: q.z):
            pushed = bool(c.dx or c.dy)
            if c.on is not None and c.on.held:                    # the cube under it is carried by the robot: it rides along
                c.x, c.y = c.on.x + c.off[0], c.on.y + c.off[1]
                c.z, c.vz = c.on.z + CUBE_S, 0.0
                c.dx = c.dy = c.vx = c.vy = 0.0
                continue
            sup, base = self.support_of(c)
            c.on = base
            if base is not None:
                c.off = (c.x - base.x, c.y - base.y)
            if base is not None and c.z <= sup + 0.05 and c.vz >= -1e-9:     # resting on a cube: moves with it
                if not pushed:
                    c.x += base.x - base.px
                    c.y += base.y - base.py
                c.z, c.vz = sup, 0.0
                c.dx = c.dy = c.vx = c.vy = 0.0
                continue
            if pushed:
                vx, vy = c.dx / dt, c.dy / dt
                sp = math.hypot(vx, vy)
                if sp > 90.0: vx, vy = vx * 90.0 / sp, vy * 90.0 / sp
                c.vx, c.vy = vx, vy
            c.dx = c.dy = 0.0
            if c.z > sup + 1e-6 or c.vz != 0.0:                       # in the air
                c.vz -= G * dt
                c.z += c.vz * dt
                if c.z <= sup:
                    c.z = sup
                    c.vz = -c.vz * RESTITUTION if c.vz < -25.0 else 0.0
            else:                                                      # resting: friction
                c.z = sup
                sp = math.hypot(c.vx, c.vy)
                if sp > 0 and not pushed:
                    k = max(0.0, sp - FRICTION * G * dt) / sp
                    c.vx *= k; c.vy *= k
                    if sp * k < 0.05: c.vx = c.vy = 0.0
            if not pushed:
                c.x += c.vx * dt
                c.y += c.vy * dt

    def resolve(self, robot_parts, shift_robot, iters=5):
        """Push apart everything that overlaps. The robot pushes cubes, walls and pillars stop both."""
        free = self.free_cubes()
        for c in free: c.pinned = False
        for _ in range(iters):
            moved = False
            for c in free:                                              # cubes against walls and pillars
                ob = c.obb()
                for s in self.statics:
                    if aabb_apart(ob, s) or not z_overlap(ob, s): continue
                    r = sat(ob, s)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        c.x -= nx * d; c.y -= ny * d; c.dx -= nx * d; c.dy -= ny * d
                        c.pinned = moved = True
                        ob = c.obb()
            for part in robot_parts():                                  # robot against walls and pillars
                for s in self.statics:
                    if aabb_apart(part, s) or not z_overlap(part, s): continue
                    r = sat(part, s)
                    if r and r[0] > 1e-6:
                        shift_robot(-r[1] * r[0], -r[2] * r[0]); moved = True
                        break
            for part in robot_parts():                                  # robot against cubes: the cube moves unless it is pinned
                for c in free:
                    ob = c.obb()
                    if aabb_apart(part, ob) or not z_overlap(part, ob): continue
                    r = sat(part, ob)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        if c.pinned: shift_robot(-nx * d, -ny * d)
                        else: c.x += nx * d; c.y += ny * d; c.dx += nx * d; c.dy += ny * d
                        moved = True
            for i in range(len(free)):                                  # cubes against cubes
                a = free[i]
                for j in range(i + 1, len(free)):
                    b = free[j]
                    oa, ob = a.obb(), b.obb()
                    if aabb_apart(oa, ob) or not z_overlap(oa, ob): continue
                    r = sat(oa, ob)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        wa = 0.0 if b.pinned and not a.pinned else 1.0
                        wb = 0.0 if a.pinned and not b.pinned else 1.0
                        tot = wa + wb
                        if tot == 0: continue
                        a.x -= nx * d * wa / tot; a.y -= ny * d * wa / tot; a.dx -= nx * d * wa / tot; a.dy -= ny * d * wa / tot
                        b.x += nx * d * wb / tot; b.y += ny * d * wb / tot; b.dx += nx * d * wb / tot; b.dy += ny * d * wb / tot
                        moved = True
            if not moved: break


# the inner walls of the arena (also drawn by render_V_8_0_1)
WALLS = [((-35, -105), (-35, -12.5)), ((-35, 12.5), (-35, 57.5)), ((-35, 82.5), (-35, 105)),
         ((35, -105), (35, -82.5)), ((35, -57.5), (35, -12.5)), ((35, 12.5), (35, 105)),
         ((-35, 35), (35, 35)), ((-35, -35), (35, -35)), ((-35, 105), (35, 105)), ((-35, -105), (35, -105))]


# ---------------------------------------------------------------- the tower
class Tower:
    """A storage box that slides up and down on a rack, open at the top and the bottom, with one servo pad on its +x inner wall.
    The pad pushes a cube inside the box against the far wall: it is then HELD and goes up and down and drives with the box."""

    def __init__(self):
        self.pad_cmd = self.lift_cmd = 0.0
        self.reset()

    def reset(self):
        self.lift, self.ext = LIFT_START, 0.0          # box bottom height, pad travel from the +x wall
        self.held = None
        self.rel = (0.0, 0.0, 0.0, 0.0)                 # held cube in the robot frame: x, y, height above the box bottom, yaw
        self.contact = False

    @property
    def pad_face(self): return BOX_IN / 2 - self.ext

    def parts(self, rob):
        o = BOX_IN / 2 + BOX_WALL / 2
        walls = ((0.0, o, BOX_OUT / 2, BOX_WALL / 2), (0.0, -o, BOX_OUT / 2, BOX_WALL / 2), (o, 0.0, BOX_WALL / 2, BOX_IN / 2), (-o, 0.0, BOX_WALL / 2, BOX_IN / 2))
        return [OBB(*rob.to_world(cx, cy), hx, hy, rob.th, self.lift, self.lift + BOX_H, 'boxwall') for cx, cy, hx, hy in walls]

    def set_cmd(self, pad, lift):
        self.pad_cmd, self.lift_cmd = clamp(pad, 0.0, 1.0), clamp(lift, -1.0, 1.0)

    def move(self, dt):
        self.lift = clamp(self.lift + self.lift_cmd * LIFT_RATE * dt, LIFT_MIN, LIFT_MAX)
        new = self.ext + clamp(self.pad_cmd * PAD_MAX - self.ext, -PAD_RATE * dt, PAD_RATE * dt)
        if self.held is not None and new > self.ext:
            new = min(new, BOX_IN / 2 - (self.rel[0] + CUBE_S / 2))        # a held cube stops the pad
        self.ext = clamp(new, 0.0, PAD_MAX)

    def sync_held(self, rob):
        if self.held is not None:
            c = self.held
            c.x, c.y = rob.to_world(self.rel[0], self.rel[1])
            c.th = rob.th + self.rel[3]
            c.z = self.lift + self.rel[2]

    def update_grip(self, rob, world):
        if self.held is not None:
            if self.pad_face > self.rel[0] + CUBE_S / 2 + TOWER_RELEASE_EPS:
                self.release(rob)
            return
        self.contact = False
        for c in world.free_cubes():
            rx, ry = rob.to_robot(c.x, c.y)
            if abs(rx) > BOX_IN / 2 + 0.2 or abs(ry) > BOX_IN / 2 + 0.2:
                continue                                                   # not in the box
            zrel = c.z - self.lift
            if not (zrel < PAD_Z1 and zrel + CUBE_S > PAD_Z0) or self.pad_face > rx + CUBE_S / 2 + 0.02:
                continue                                                   # the pad does not reach it (height) / has not reached it yet
            self.contact = True
            nrx = min(rx, self.pad_face - CUBE_S / 2)                      # the pad pushes the cube toward the far wall
            if nrx <= FAR_X + 0.02:                                        # squeezed against the far wall: held
                self.ext = BOX_IN / 2 - (FAR_X + CUBE_S / 2)
                self.grab(rob, c, FAR_X, ry, zrel)
                return
            c.x, c.y = rob.to_world(nrx, ry)
            c.vx = c.vy = 0.0

    def grab(self, rob, cube, rx, ry, zrel):
        self.held = cube
        cube.held = True
        cube.vx = cube.vy = cube.vz = 0.0
        self.rel = (rx, clamp(ry, -0.4, 0.4), zrel, cube.th - rob.th)
        self.sync_held(rob)

    def release(self, rob):
        c = self.held
        if c is None:
            return
        c.held = False
        c.vx, c.vy = rob.point_velocity(c.x, c.y)
        c.vz = 0.0
        self.held = None


# ---------------------------------------------------------------- the robot
class Robot:
    def __init__(self, arena=True):
        self.arena = arena
        self.world, self.tower = World(), Tower()
        self.mode = 'arcade_snap'
        self.trail = deque(maxlen=500)
        self.reset_robot()

    # ---- state
    def reset_robot(self):
        """Back in the start box, tower in its start pose. The cubes stay where they are."""
        self.tower.release(self)
        self.tower.reset()
        self.x, self.y, self.th = SPAWN
        self.vl = self.vr = 0.0                         # wheel speeds, -1 .. 1
        self.al = self.ar = 0.0                         # wheel angles for the spokes
        self.speed = self.omega = self.vx = self.vy = 0.0
        self.snap_target, self.snap_w = None, 0.0
        self.follow = False                                  # True while a real robot's gyro drives the heading (Game / Monitor with the robot connected)
        self.raw_l = self.raw_r = 0.0                        # what the sticks ask of each wheel, -1..1, with no simulated lag: this is what the real robot gets
        self.turn_on, self._lt_prev = False, False
        self._prev, self.rt, self._t = {}, 0.0, 0.0
        self.trail.clear()
        self.zero()

    def reset(self):
        self.reset_robot()
        self.world.reset_cubes()

    def reset_cubes(self):
        self.tower.release(self)
        self.world.reset_cubes()

    def set_arena(self, on):
        if on != self.arena:
            self.arena = on
            self.reset()

    def zero(self):
        """The current heading becomes the zero: Y / B / A / X face up / right / down / left from here."""
        self.ref_th = self.th

    @property
    def heading_deg(self): return (-math.degrees(self.th - self.ref_th)) % 360

    @property
    def snapping(self): return self.snap_target is not None

    @property
    def speed_scale(self): return (SLOW_CM + (MAX_SPEED - SLOW_CM) * clamp(self.rt, 0.0, 1.0)) / MAX_SPEED

    # ---- frames
    def to_world(self, x, y):
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + x * c - y * s, self.y + x * s + y * c

    def to_robot(self, wx, wy):
        dx, dy = wx - self.x, wy - self.y
        c, s = math.cos(self.th), math.sin(self.th)
        return dx * c + dy * s, -dx * s + dy * c

    def point_velocity(self, wx, wy): return self.vx - self.omega * (wy - self.y), self.vy + self.omega * (wx - self.x)

    def parts(self):
        out = [OBB(*self.to_world(cx, cy), hx, hy, self.th, z0, z1, 'body') for cx, cy, hx, hy, z0, z1 in PARTS]
        out += self.tower.parts(self)
        if self.tower.held is not None:
            out.append(self.tower.held.obb())
        return out

    def _shift(self, dx, dy): self.x += dx; self.y += dy

    # ---- input
    def _edge(self, st, name):
        now, was = bool(st.buttons.get(name)), self._prev.get(name, False)
        self._prev[name] = now
        return now and not was

    def face(self, ang):
        err = wrap_pi(self.ref_th + ang - self.th)
        if abs(err) >= ALIGN_TOL:
            self.snap_target = self.th + err

    def _buttons(self, st):
        edges = {n: self._edge(st, n) for n in ('START', 'LB', 'RB', 'Y', 'B', 'A', 'X')}
        if edges['START']:
            self.zero()
        if self.mode != 'arcade_snap':
            self.snap_target = None
            return
        if self.snapping:
            return                                                         # presses during a turn are ignored: one turn per press
        if edges['LB']: self.snap_target = self.th + math.pi / 2
        elif edges['RB']: self.snap_target = self.th - math.pi / 2
        elif edges['Y']: self.face(0.0)
        elif edges['B']: self.face(-math.pi / 2)
        elif edges['A']: self.face(math.pi)
        elif edges['X']: self.face(math.pi / 2)

    def _targets(self, st):
        """Wheel targets in cm/s. The D-pad nudges slowly (not affected by RT); otherwise the left stick, turning only while TURN is on."""
        dx, dy = st.dpad
        if dx or dy:
            v, w = dy * DPAD_SPEED, -dx * DPAD_TURN
            return v - w * TRACK / 2.0, v + w * TRACK / 2.0
        s = self.speed_scale
        turn = st.lx if self.turn_on else 0.0
        return clamp(st.ly + TURN_GAIN * turn, -1, 1) * s * MAX_SPEED, clamp(st.ly - TURN_GAIN * turn, -1, 1) * s * MAX_SPEED

    @staticmethod
    def _approach(v, target, h):
        """One wheel: first-order lag toward the target, with limited acceleration and deceleration."""
        nv = v + (target - v) * min(1.0, h * LAG)
        lim = (ACCEL if abs(nv) > abs(v) else DECEL) * h
        return v + clamp(nv - v, -lim, lim)

    # ---- one frame
    def drive(self, st, dt):
        self._st = st
        self.rt = st.rt
        lt = st.lt >= LT_TURN
        if lt and not self._lt_prev:
            self.turn_on = not self.turn_on                                 # each press of LT switches turning on / off
        self._lt_prev = lt
        self.tower.set_cmd(clamp(st.rx, 0.0, 1.0), st.ry)
        self._buttons(st)
        n = max(1, int(math.ceil(dt / MAX_SUB - 1e-9)))
        for _ in range(n):
            self._substep(st, dt / n)

    def _substep(self, st, h):
        self.world.mark()
        if self.follow:
            self._follow_step(h)
        elif self.snapping:
            self._snap_step(h)
        else:
            tl, tr = self._targets(st)
            self.raw_l, self.raw_r = tl / MAX_SPEED, tr / MAX_SPEED
            vl, vr = self._approach(self.vl * MAX_SPEED, tl, h), self._approach(self.vr * MAX_SPEED, tr, h)
            self.vl, self.vr = vl / MAX_SPEED, vr / MAX_SPEED
            self.speed = (vl + vr) / 2.0
            self.omega = (vr - vl) / TRACK
            self.th += self.omega * h
            self.vx, self.vy = -math.sin(self.th) * self.speed, math.cos(self.th) * self.speed
            self.x += self.vx * h
            self.y += self.vy * h
        self.al += self.vl * MAX_SPEED * h / 3.0
        self.ar += self.vr * MAX_SPEED * h / 3.0
        self.tower.move(h)
        self.tower.sync_held(self)
        if self.arena:
            self.world.resolve(self.parts, self._shift)
            self.tower.sync_held(self)
            self.world.step_cubes(h)
            self.tower.update_grip(self, self.world)
        self._t += h
        if self._t > 0.06 and abs(self.speed) > 0.5:
            self._t = 0.0
            self.trail.append((self.x, self.y))

    def follow_robot(self, on, yaw_deg=0.0, gz_dps=0.0, vl_pct=0.0, vr_pct=0.0):
        """Real robot connected: its gyro IS the heading (turning left raises yaw, same as th). Wheel speeds come from its telemetry."""
        self.follow = bool(on)
        if on:
            self.th, self.omega = math.radians(yaw_deg), math.radians(gz_dps)
            self.vl, self.vr = vl_pct / 100.0, vr_pct / 100.0
            self.speed = (self.vl + self.vr) / 2.0 * MAX_SPEED

    def _follow_step(self, h):
        tl, tr = self._targets(self._st) if getattr(self, '_st', None) is not None else (0.0, 0.0)
        self.raw_l, self.raw_r = (0.0, 0.0) if self.snapping else (tl / MAX_SPEED, tr / MAX_SPEED)
        if self.snapping and abs(wrap_pi(self.snap_target - self.th)) < math.radians(1.5) and abs(self.omega) < math.radians(12):
            self.snap_target = None                                          # the robot arrived: its own gyro says so
        self.vx, self.vy = -math.sin(self.th) * self.speed, math.cos(self.th) * self.speed
        self.x += self.vx * h
        self.y += self.vy * h

    def _snap_step(self, h):
        """Spin in place about the wheel midpoint: accelerate, cruise, brake so the heading arrives exactly on target with zero speed."""
        err = self.snap_target - self.th
        w_des = math.copysign(min(W_MAX, math.sqrt(2.0 * 0.8 * SNAP_ACCEL * abs(err))), err)
        self.snap_w += clamp(w_des - self.snap_w, -SNAP_ACCEL * h, SNAP_ACCEL * h)
        step = self.snap_w * h
        if abs(step) >= abs(err) or (err != 0 and step * err < 0):
            self.th, self.snap_target, self.snap_w = self.snap_target, None, 0.0
        else:
            self.th += step
        self.raw_l = self.raw_r = 0.0
        w = self.snap_w
        self.vl, self.vr = -w * TRACK / 2.0 / MAX_SPEED, w * TRACK / 2.0 / MAX_SPEED
        self.speed, self.omega, self.vx, self.vy = 0.0, w, 0.0, 0.0

    # ---- what the picture and the panels need (a plain dict, so real robot data can fill the same shape)
    def snapshot(self):
        t = self.tower
        return dict(x=self.x, y=self.y, th=self.th, ref_th=self.ref_th, heading=self.heading_deg, vl=self.vl, vr=self.vr, al=self.al, ar=self.ar,
                    speed=self.speed, omega=self.omega, lift=t.lift, pad=t.ext, cmd=t.pad_cmd, holding=t.held is not None, touching=t.contact,
                    turn_on=self.turn_on, snapping=self.snapping, mode=self.mode, rt=self.rt, speed_scale=self.speed_scale, source='sim')
