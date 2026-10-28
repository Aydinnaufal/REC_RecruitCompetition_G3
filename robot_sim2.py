"""
robot_sim2.py - alternate version of the robot simulator: cubes on pillars, a proportional claw, variable speed, snap turns.

RobotSim2 extends RobotSim (robot_sim.py is not changed). The physics lives in sim_physics.py; this file connects it to the
controller input and draws it with the camera and helpers of robot_sim.py.

Controller (ControllerState from xbox_input.py):
    left stick            drive and steer (the same arcade mixing as RobotSim; tank mode is unchanged too)
    LT   0..1             claw: 0 = fully open, 1 = fully closed (proportional). Both arms touching one cube = grab.
    RT   0..1             speed: 0 = SPEED_MIN (slightly below default), 1 = default speed (MAX_SPEED)
  mode 'arcade_snap' = arcade plus:
    LB / RB               snap-turn 90 degrees left / right about the middle of the wheels, once per press
    Y / B / A / X         face up / right / down / left of the map, as the robot stood at the start (shortest turn)
"""
import math
import pygame
import sim_physics as ph
from robot_sim import (RobotSim, MAX_SPEED, TRACK, TURN_GAIN, clamp, poly, line, rect3, NEAR, LIGHT, BODY, DECK, NOSE, WALL,
                       draw_floor, set_view, _box, _wheel)

MODES = ['arcade', 'arcade_snap', 'tank']
SPEED_MIN = 0.80                       # RT = 0 gives 80% of the default speed
ACCEL, DECEL = 150.0, 240.0            # wheel acceleration limits, cm/s^2 (full speed in about 0.2 s)
LAG = 9.0                              # same first-order lag as RobotSim, applied before the limits
W_MAX = 2.0 * MAX_SPEED / TRACK        # rad/s: the wheels at full speed in opposite directions
SNAP_ACCEL = 30.0                      # rad/s^2
ALIGN_TOL = math.radians(0.3)          # already facing the wanted way: do not turn
REF_TH = 0.0                           # "initial orientation": facing up the map. Never changes.
MAX_SUB = 1.0 / 60.0

# robot body as collision boxes in the robot frame: cx, cy, half x, half y, z0, z1 (the wheels are centred on the origin)
BODY_PARTS = [(0.0, 0.0, 6.0, 7.0, 2.0, 5.0), (0.0, 8.75, 4.0, 1.75, 2.6, 4.4),
              (-7.5, 0.0, 1.2, 3.0, 0.0, 6.0), (7.5, 0.0, 1.2, 3.0, 0.0, 6.0)]


class RobotSim2(RobotSim):
    def __init__(self):
        self.world = ph.World()
        self.claw = ph.Claw()
        super().__init__()
        self.mode = 'arcade'

    # ---------------------------------------------------------- state
    def reset(self):
        super().reset()
        self.vx = self.vy = self.omega = 0.0
        self.snap_target, self.snap_w = None, 0.0
        self._prev = {}
        self.rt = 0.0
        self.claw.reset()
        self.world.reset_cubes()

    def reset_cubes(self):
        self.claw.release(self)
        self.world.reset_cubes()

    @property
    def speed_scale(self):
        return SPEED_MIN + (1.0 - SPEED_MIN) * clamp(self.rt, 0.0, 1.0)

    @property
    def snapping(self):
        return self.snap_target is not None

    # ---------------------------------------------------------- frames
    def to_world(self, x, y):
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + x * c - y * s, self.y + x * s + y * c

    def to_robot(self, wx, wy):
        dx, dy = wx - self.x, wy - self.y
        c, s = math.cos(self.th), math.sin(self.th)
        return dx * c + dy * s, -dx * s + dy * c

    def point_velocity(self, wx, wy):
        return self.vx - self.omega * (wy - self.y), self.vy + self.omega * (wx - self.x)

    def parts(self):
        out = [ph.OBB(*self.to_world(cx, cy), hx, hy, self.th, z0, z1, 'body') for cx, cy, hx, hy, z0, z1 in BODY_PARTS]
        out += [self.claw.arm_obb(self, -1), self.claw.arm_obb(self, 1)]
        if self.claw.held is not None:
            out.append(self.claw.held.obb())
        return out

    def _shift(self, dx, dy):
        self.x += dx; self.y += dy

    # ---------------------------------------------------------- input
    def _edge(self, st, name):
        now, was = bool(st.buttons.get(name)), self._prev.get(name, False)
        self._prev[name] = now
        return now and not was

    def start_snap(self, delta):
        self.snap_target = self.th + delta

    def face(self, ang):
        """Turn to face the absolute direction ang (0 up, -pi/2 right, pi down, +pi/2 left) by the shortest way; no turn if aligned."""
        err = ph.wrap_pi(REF_TH + ang - self.th)
        if abs(err) >= ALIGN_TOL:
            self.snap_target = self.th + err

    def _buttons(self, st):
        edges = {n: self._edge(st, n) for n in ('LB', 'RB', 'Y', 'B', 'A', 'X')}
        if self.mode != 'arcade_snap':
            self.snap_target = None
            return
        if self.snapping:
            return                                   # presses during a turn are ignored: one turn per press
        if edges['LB']: self.start_snap(math.pi / 2)
        elif edges['RB']: self.start_snap(-math.pi / 2)
        elif edges['Y']: self.face(0.0)
        elif edges['B']: self.face(-math.pi / 2)
        elif edges['A']: self.face(math.pi)
        elif edges['X']: self.face(math.pi / 2)

    def _stick_targets(self, st):
        s = self.speed_scale
        if self.mode == 'tank':
            l, r = st.ly, st.ry
        else:
            l, r = clamp(st.ly + TURN_GAIN * st.lx, -1, 1), clamp(st.ly - TURN_GAIN * st.lx, -1, 1)
        return l * s * MAX_SPEED, r * s * MAX_SPEED

    @staticmethod
    def _approach(v, target, h):
        """One wheel: first-order lag toward the target, with limited acceleration (speeding up) and deceleration (braking)."""
        nv = v + (target - v) * min(1.0, h * LAG)
        lim = (ACCEL if abs(nv) > abs(v) else DECEL) * h
        return v + clamp(nv - v, -lim, lim)

    # ---------------------------------------------------------- one frame
    def drive(self, st, dt):
        """st: ControllerState. Same call as RobotSim.drive, so the dashboard loop does not change."""
        self.rt = st.rt
        self.claw.cmd = clamp(st.lt, 0.0, 1.0)
        self._buttons(st)
        n = max(1, int(math.ceil(dt / MAX_SUB - 1e-9)))
        for _ in range(n):
            self._substep(st, dt / n)

    def _substep(self, st, h):
        if self.snapping:
            self._snap_step(h)
        else:
            tl, tr = self._stick_targets(st)
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
        self.claw.move_arms(self, h)
        self.claw.sync_held(self)
        self.world.resolve(self.parts, self._shift)
        self.claw.sync_held(self)
        self.world.step_cubes(h)
        self.claw.update_grip(self, self.world, h)
        self._t += h
        if self._t > 0.06 and abs(self.speed) > 0.5:
            self._t = 0.0
            self.trail.append((self.x, self.y))

    def _snap_step(self, h):
        """Spin in place about the wheel midpoint along an exact motion profile: accelerate, cruise at the top rotation speed,
        brake so the heading arrives at the target with zero speed. The last step lands on the target exactly (no overshoot)."""
        err = self.snap_target - self.th
        w_des = math.copysign(min(W_MAX, math.sqrt(2.0 * 0.8 * SNAP_ACCEL * abs(err))), err)
        self.snap_w += clamp(w_des - self.snap_w, -SNAP_ACCEL * h, SNAP_ACCEL * h)
        step = self.snap_w * h
        if abs(step) >= abs(err) or (err != 0 and step * err < 0):
            self.th, self.snap_target, self.snap_w = self.snap_target, None, 0.0
        else:
            self.th += step
        w = self.snap_w
        self.vl, self.vr = -w * TRACK / 2.0 / MAX_SPEED, w * TRACK / 2.0 / MAX_SPEED   # wheels turn opposite ways (for the display)
        self.speed, self.omega, self.vx, self.vy = 0.0, w, 0.0, 0.0


# ================================================================ drawing
CUBE_COL = {'B': (48, 96, 232), 'R': (226, 66, 62)}
PILLAR_COL = (214, 216, 220)
ARM_COL = (238, 168, 58)
BASE_COL = (84, 92, 108)


def _xform(faces, ang, ox, oy, oz=0.0):
    """Rotate faces (points, normal, colour) by ang about z, then move them by (ox, oy, oz)."""
    c, s = math.cos(ang), math.sin(ang)
    return [([(ox + p[0] * c - p[1] * s, oy + p[0] * s + p[1] * c, oz + p[2]) for p in pts], (n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]), col)
            for pts, n, col in faces]


def robot_faces2(sim):
    """Robot faces in the robot frame: the original body with the wheels centred on the axle, plus the claw."""
    f = _box(-6.0, -7.0, 2.0, 6.0, 7.0, 5.0, BODY)
    f += _box(-4.0, 7.0, 2.6, 4.0, 10.5, 4.4, NOSE)
    for fc in _wheel(-1, sim.al) + _wheel(1, sim.ar):
        f.append(([(p[0], p[1] + 0.5, p[2]) for p in fc[0]], fc[1], fc[2]))
    f.append(([(-2.5, -3, 5.02), (2.5, -3, 5.02), (0, 4.5, 5.02)], (0, 0, 1), DECK))
    f += _box(-ph.ARM_HINGE - 1.4, ph.ARM_HINGE_Y - 1.8, 5.0, ph.ARM_HINGE + 1.4, ph.ARM_HINGE_Y + 0.6, 6.2, BASE_COL)      # claw mount
    for side in (-1, 1):
        cx, cy, hx, hy, a = sim.claw.arm_local(side, sim.claw.ang[side])
        f += _xform(_box(0.0, -ph.ARM_T / 2, ph.ARM_Z0, ph.ARM_LEN, ph.ARM_T / 2, ph.ARM_Z1, ARM_COL), a, side * ph.ARM_HINGE, ph.ARM_HINGE_Y)
    return f


def _items(cam, faces, W, N, out):
    for pts, n, col in faces:
        wp, wn = [W(p) for p in pts], N(n)
        ctr = tuple(sum(p[k] for p in wp) / len(wp) for k in range(3))
        view = (cam.eye[0] - ctr[0], cam.eye[1] - ctr[1], cam.eye[2] - ctr[2])
        if wn[0] * view[0] + wn[1] * view[1] + wn[2] * view[2] <= 0:
            continue
        shade = 0.5 + 0.5 * max(0.0, wn[0] * LIGHT[0] + wn[1] * LIGHT[1] + wn[2] * LIGHT[2])
        out.append((cam.cam(ctr)[2], tuple(int(v * shade) for v in col), wp))


def _box_world(cx, cy, hx, hy, ang, z0, z1, col, cam, out, seg=None):
    """A box at a world position. With seg it is cut into pieces about seg long (along its x) so long walls depth-sort correctly."""
    n = max(1, round(2 * hx / seg)) if seg else 1
    h = hx / n
    c, s = math.cos(ang), math.sin(ang)
    for k in range(n):
        off = -hx + (k + 0.5) * 2 * hx / n
        _items(cam, _box(-h, -hy, z0, h, hy, z1, col), lambda p: (cx + (p[0] + off) * c - p[1] * s, cy + (p[0] + off) * s + p[1] * c, p[2]),
               lambda nv: (nv[0] * c - nv[1] * s, nv[0] * s + nv[1] * c, nv[2]), out)


def draw_scene2(surf, cam, sim, view):
    """Floor, walls, pillars, cubes and the robot with its claw, depth sorted together."""
    surf.fill((20, 23, 28))
    set_view(cam, view, sim)
    draw_floor(surf, cam)
    for i, (tx, ty) in enumerate(sim.trail):
        cc = cam.cam((tx, ty, 0.1))
        if cc[2] > NEAR:
            a = 0.25 + 0.75 * i / max(1, len(sim.trail))
            pygame.draw.circle(surf, (int(0 * a + 20), int(200 * a), int(170 * a)), cam.screen(cc), 2)
    w = sim.world
    for c in w.cubes:                                                    # shadow of a raised cube on the floor / pillar top
        sup = w.support_height(c.x, c.y)
        if c.z > sup + 0.05:
            poly(surf, cam, (18, 20, 24), rect3(c.x - 1.5, c.y - 1.5, c.x + 1.5, c.y + 1.5, sup + 0.07))
    s, co = math.sin(sim.th), math.cos(sim.th)
    shadow = [sim.to_world(x, y) for x, y in ((-8, -9), (8, -9), (8, 12), (-8, 12))]
    poly(surf, cam, (22, 25, 29), [(x, y, 0.1) for x, y in shadow])
    items = []
    for st in w.statics:
        if st.tag == 'wall':
            _box_world(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, ph.WALL_H, WALL, cam, items, seg=8.0)
        elif st.tag == 'pillar':
            _box_world(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, ph.PILLAR_H, PILLAR_COL, cam, items)
    for c in w.cubes:
        _box_world(c.x, c.y, ph.CUBE_S / 2, ph.CUBE_S / 2, c.th, c.z, c.z + ph.CUBE_S, CUBE_COL[c.team], cam, items)
    fwd, rgt = (-s, co), (co, s)
    Wr = lambda p: (sim.x + p[0] * rgt[0] + p[1] * fwd[0], sim.y + p[0] * rgt[1] + p[1] * fwd[1], p[2])
    Nr = lambda n: (n[0] * rgt[0] + n[1] * fwd[0], n[0] * rgt[1] + n[1] * fwd[1], n[2])
    _items(cam, robot_faces2(sim), Wr, Nr, items)
    for _, col, wp in sorted(items, key=lambda t: -t[0]):
        poly(surf, cam, col, wp)
        poly(surf, cam, tuple(int(v * 0.6) for v in col), wp, 1)
    cc = cam.cam(sim.to_world(0, -5.5) + (1.0,))                         # rear ball caster
    if cc[2] > NEAR:
        pygame.draw.circle(surf, (180, 186, 194), cam.screen(cc), max(2, int(cam.fl * 1.0 / cc[2])))
    for c in w.cubes:                                                    # far away a 3 cm cube is only a pixel or two: keep it visible
        cc = cam.cam((c.x, c.y, c.z + ph.CUBE_S / 2))
        if cc[2] > NEAR and cam.fl * ph.CUBE_S / cc[2] < 7:
            sx, sy = cam.screen(cc)
            pygame.draw.rect(surf, CUBE_COL[c.team], (sx - 3, sy - 3, 7, 7))
            pygame.draw.rect(surf, (235, 235, 235), (sx - 3, sy - 3, 7, 7), 1)
        if c.held:
            cc = cam.cam((c.x, c.y, c.z + ph.CUBE_S + 1.2))
            if cc[2] > NEAR:
                pygame.draw.circle(surf, (255, 235, 90), cam.screen(cc), 3)
