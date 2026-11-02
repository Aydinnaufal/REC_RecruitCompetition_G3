"""
robot_sim3.py - v3 of the robot simulator. Builds on robot_sim2.py (which is not changed).

New in v3
  * two robot models, both inside a 20 x 20 x 20 cm box:
        'simple'  one claw (two arms) at the front, as v2 but moved to cube height and fitted to the 20 cm limit
        'tower'   no front claw. A storage box on a rack and pinion goes up and down through a hole in a raised chassis;
                  one servo pad inside it holds a cube against the far wall. The robot drives over a cube, lowers the box, grips, lifts.
  * the robot starts in its team's start box (blue or red), not in the middle; reset buttons for cubes / robot / everything
  * left stick = forward and back only, right stick X = turn; START records a new zero heading
  * camera views from the front, right and left of the robot
  * monitor mode: the robot stays in place, no arena; the picture is drawn from a plain dict (snapshot) so real data can feed it later

Controller (ControllerState from xbox_input.py)
    left stick Y   forward / back            right stick X   turn (arcade modes)        tank mode: left Y = left wheel, right Y = right wheel
    RT             speed 80% .. 100%         LT              simple: claw 0 open .. 1 closed;   tower: pad 0 retracted .. 1 pushing
    D-pad up/down  tower box up / down (hold)
    LB / RB        snap-turn 90 left / right (arcade + snap mode)         Y / B / A / X   face up / right / down / left of the zero heading
    START          make the current heading the new zero
"""
import math
import pygame
import sim_physics as ph
import sim_tower as tw
from robot_sim import (RobotSim, Camera, MAX_SPEED, TRACK, TURN_GAIN, clamp, poly, line, rect3, NEAR, LIGHT, BODY, DECK, NOSE, WALL,
                       draw_floor, _box, _wheel)
from robot_sim2 import (RobotSim2, ACCEL, DECEL, ALIGN_TOL, MAX_SUB, CUBE_COL, PILLAR_COL, ARM_COL, BASE_COL, _xform)

SLOW_CM = 20.0                               # speed cap without RT (cm/s); RT fully pressed gives 30
DPAD_SPEED = 5.0                             # D-pad nudge: forward / back, cm/s
DPAD_TURN = 0.6                              # D-pad nudge: turn, rad/s (about 35 deg/s)
LT_TURN = 0.5                                # LT pressed this far counts as a press (toggles the turning on / off)
PILLAR_H3 = 0.5                              # the white pillar a cube stands on is only 5 mm high
MODES = ['arcade', 'arcade_snap', 'tank']
MODE_LABEL = {'arcade': 'Arcade', 'arcade_snap': 'Arcade + Snap', 'tank': 'Tank'}
ROBOTS = ['simple', 'tower']
ROBOT_LABEL = {'simple': 'Simple (1 claw)', 'tower': 'Tower (rack + pad)'}
TEAMS = ['blue', 'red']

# ---- the two designs. Robot frame: x right, y forward, origin = middle of the wheel axle. All within 20 x 20 x 20 cm.
CLAW = dict(hinge=1.9, hinge_y=7.0, length=7.0, z0=0.9, z1=3.1)      # arms start 1 cm in front of the chassis, tips at y = 14 cm (rear is at -6)
WHEEL_PARTS = [(-7.5, 0.0, 1.2, 3.0, 0.0, 6.0), (7.5, 0.0, 1.2, 3.0, 0.0, 6.0)]
BODY_Z = (4.2, 8.0)                          # tower chassis: the underside (4.2) clears a cube on a pillar (3.5)
HOLE = BOX_HOLE = 2.5                        # half size of the square hole in the tower chassis (box outside is 4.3 across)
MAST = (0.0, -3.3, 1.0, 0.6, 8.0, 20.0)      # rack column behind the box, in a notch of the chassis ring
PARTS = {
    'simple': [(0.0, 0.0, 6.0, 6.0, 2.0, 5.0), (0.0, 6.8, 3.4, 0.8, 0.9, 3.3)] + WHEEL_PARTS,
    'tower': [(0.0, 5.25, 6.0, 2.75, *BODY_Z), (0.0, -5.95, 6.0, 2.05, *BODY_Z),
              (-4.25, -0.7, 1.75, 3.2, *BODY_Z), (4.25, -0.7, 1.75, 3.2, *BODY_Z), MAST] + WHEEL_PARTS,
}
SPAWN = {'blue': (-82.0, -92.0, 0.0), 'red': (82.0, 92.0, math.pi)}       # centre of the start box and the way the robot faces
SPAWN_SHIFT = {'simple': -4.0, 'tower': 0.0}                              # move back so the whole 20 cm footprint is inside the 20 x 20 box


def spawn_pose(team, robot):
    sx, sy, th = SPAWN[team]
    k = SPAWN_SHIFT[robot]
    return sx - math.sin(th) * k, sy + math.cos(th) * k, th


class SimpleGrip(ph.Claw):
    """The v2 claw behind the same small interface the tower uses."""
    kind = 'claw'

    def set_cmd(self, pad, lift): self.cmd = pad
    def move(self, rob, dt): self.move_arms(rob, dt)
    def parts(self, rob): return [self.arm_obb(rob, -1), self.arm_obb(rob, 1)]


class TowerGrip(tw.Tower):
    kind = 'tower'


def make_grip(robot):
    return SimpleGrip(**CLAW) if robot == 'simple' else TowerGrip()


class RobotSim3(RobotSim2):
    def __init__(self, robot='simple', team='blue', arena=True):
        self.robot_type, self.team, self.arena = robot, team, arena
        self.world = ph.World(pillar_h=PILLAR_H3, stacking=True)
        self.grip = make_grip(robot)
        self.ref_th = 0.0
        self.zero_count = 0
        RobotSim.__init__(self)                       # sets mode 'arcade' and calls reset()
        self.mode = 'arcade_snap'                     # v3.2 default: arcade + snap

    # ---------------------------------------------------------- state
    def place_at_spawn(self):
        self.x, self.y, self.th = spawn_pose(self.team, self.robot_type)

    def _stop(self):
        self.vl = self.vr = 0.0
        self.vx = self.vy = self.omega = self.speed = 0.0
        self.snap_target, self.snap_w = None, 0.0
        self.turn_on, self._lt_prev = False, False          # LT toggles the left-stick turning

    def reset_robot(self):
        """Put the robot back in its start box with the gripper in its start pose. The cubes stay where they are."""
        self.grip.release(self)
        self.grip.reset()
        RobotSim.reset(self)
        self._stop()
        self._prev, self.rt = {}, 0.0
        self.place_at_spawn()
        self.zero()

    def reset(self):
        """The whole thing again: robot in its start box, gripper reset, every cube back on its pillar."""
        self.reset_robot()
        self.world.reset_cubes()

    def reset_cubes(self):
        self.grip.release(self)
        self.world.reset_cubes()

    def set_robot(self, robot):
        if robot != self.robot_type:
            self.grip.release(self)
            self.robot_type = robot
            self.grip = make_grip(robot)
            self.reset_robot()

    def set_team(self, team):
        if team != self.team:
            self.team = team
            self.reset_robot()

    def set_arena(self, on):
        if on != self.arena:
            self.arena = on
            self.grip.release(self)
            self.reset()

    def zero(self):
        """Record the current heading as the new absolute zero: Y / B / A / X face up / right / down / left from here."""
        self.ref_th = self.th
        self.zero_count += 1

    @property
    def heading_deg(self):
        """Compass style relative to the zero: 0 = the zero direction, turning right makes it bigger."""
        return (-math.degrees(self.th - self.ref_th)) % 360

    # ---------------------------------------------------------- collision parts
    def parts(self):
        out = [ph.OBB(*self.to_world(cx, cy), hx, hy, self.th, z0, z1, 'body') for cx, cy, hx, hy, z0, z1 in PARTS[self.robot_type]]
        out += self.grip.parts(self)
        if self.grip.held is not None:
            out.append(self.grip.held.obb())
        return out

    # ---------------------------------------------------------- input
    def face(self, ang):
        err = ph.wrap_pi(self.ref_th + ang - self.th)
        if abs(err) >= ALIGN_TOL:
            self.snap_target = self.th + err

    def _buttons(self, st):
        if self._edge(st, 'START'):
            self.zero()
        super()._buttons(st)

    @property
    def speed_scale(self):
        """Without RT the robot is limited to SLOW_CM; RT fully pressed gives MAX_SPEED (30 cm/s)."""
        return (SLOW_CM + (MAX_SPEED - SLOW_CM) * clamp(self.rt, 0.0, 1.0)) / MAX_SPEED

    def _stick_targets(self, st):
        """Wheel targets in cm/s.
        D-pad (any direction): slow nudge, forward / back / slight turn, not affected by RT.
        Left stick Y: forward and back.  Left stick X: turns, only while turning is switched on (LT toggles it).   Tank mode: left Y / right Y = left / right wheel."""
        dx, dy = st.dpad
        if dx or dy:
            w = -dx * DPAD_TURN                                      # right = clockwise = negative
            v = dy * DPAD_SPEED
            return v - w * TRACK / 2.0, v + w * TRACK / 2.0
        s = self.speed_scale
        if self.mode == 'tank':
            l, r = st.ly, st.ry
        else:
            turn = st.lx if self.turn_on else 0.0
            l, r = clamp(st.ly + TURN_GAIN * turn, -1, 1), clamp(st.ly - TURN_GAIN * turn, -1, 1)
        return l * s * MAX_SPEED, r * s * MAX_SPEED

    def grip_input(self, st):
        """Right stick X: gripper. 0 = open / released, right = closing (how far you push is how far it closes), left = open.
        Tower: right stick Y moves the box up / down (how far = how fast). In tank mode the right stick Y is a wheel, so LB / RB move the box."""
        pad = clamp(st.rx, 0.0, 1.0)
        if self.mode == 'tank':
            lift = (1.0 if st.buttons.get('RB') else 0.0) - (1.0 if st.buttons.get('LB') else 0.0)
        else:
            lift = st.ry
        self.grip.set_cmd(pad, lift)

    # ---------------------------------------------------------- one frame
    def drive(self, st, dt):
        self.rt = st.rt
        lt = st.lt >= LT_TURN
        if lt and not self._lt_prev:
            self.turn_on = not self.turn_on                          # each press of LT switches left-stick turning on / off
        self._lt_prev = lt
        self.grip_input(st)
        self._buttons(st)
        n = max(1, int(math.ceil(dt / MAX_SUB - 1e-9)))
        for _ in range(n):
            self._substep(st, dt / n)

    def _substep(self, st, h):
        self.world.mark()
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
        self.grip.move(self, h)
        self.grip.sync_held(self)
        if self.arena:
            self.world.resolve(self.parts, self._shift)
            self.grip.sync_held(self)
            self.world.step_cubes(h)
            self.grip.update_grip(self, self.world, h)
        self._t += h
        if self._t > 0.06 and abs(self.speed) > 0.5:
            self._t = 0.0
            self.trail.append((self.x, self.y))

    # ---------------------------------------------------------- state for drawing (and, later, for real data)
    def snapshot(self):
        g = self.grip
        d = dict(turn_on=self.turn_on, type=self.robot_type, x=self.x, y=self.y, th=self.th, ref_th=self.ref_th, heading=self.heading_deg,
                 vl=self.vl, vr=self.vr, al=self.al, ar=self.ar, speed=self.speed, omega=self.omega, holding=g.held is not None, cubes=[])
        if g.kind == 'claw':
            d.update(arm_l=g.ang[-1], arm_r=g.ang[1], cmd=g.cmd)
        else:
            d.update(lift=g.lift, pad=g.ext, cmd=g.pad_cmd)
        return d


# ================================================================ drawing
TOWER_COL = (60, 200, 190)
MAST_COL = (74, 82, 98)
SERVO_COL = (44, 48, 58)
POST_COL = (64, 70, 82)
MARK_COL = (255, 200, 90)
VIEWS3 = ['arena', 'chase', 'top', 'front', 'right', 'left', 'free']
MON_VIEWS = ['iso', 'front', 'right', 'left', 'back', 'top', 'free']
VIEW_LABEL = {'arena': 'Arena', 'chase': 'Chase (behind)', 'top': 'Top', 'front': 'Front', 'right': 'Right side', 'left': 'Left side',
              'iso': 'Angled', 'back': 'Back', 'free': 'Free (mouse)'}


TYRE3, RIM3, SPOKE3 = (62, 66, 76), (88, 94, 106), (0, 235, 195)


def _wheel3(side, angle, n=14):
    """Wheel as an n-sided prism (axis = robot x). Lighter than v2 so it does not vanish in the shadow, with 4 bright spokes and a hub."""
    from robot_sim import WHEEL_R, WHEEL_W
    xc = side * TRACK / 2.0
    xi, xo = xc - WHEEL_W / 2, xc + WHEEL_W / 2
    cy, cz = 0.0, WHEEL_R
    ring = [(cy + WHEEL_R * math.cos(2 * math.pi * k / n), cz + WHEEL_R * math.sin(2 * math.pi * k / n)) for k in range(n)]
    faces = []
    for k in range(n):
        (ya, za), (yb, zb) = ring[k], ring[(k + 1) % n]
        mid = 2 * math.pi * (k + 0.5) / n
        faces.append(([(xi, ya, za), (xo, ya, za), (xo, yb, zb), (xi, yb, zb)], (0, math.cos(mid), math.sin(mid)), TYRE3))
    for x, nx in ((xi, -1), (xo, 1)):
        faces.append(([(x, y, z) for y, z in ring], (nx, 0, 0), RIM3))
    xs = xo + 0.06 if side > 0 else xi - 0.06
    for k in range(4):
        a = angle + k * math.pi / 2
        p = lambda r, w: (xs, cy + r * math.cos(a) - w * math.sin(a), cz + r * math.sin(a) + w * math.cos(a))
        faces.append(([p(0, 0.55), p(WHEEL_R * 0.9, 0.55), p(WHEEL_R * 0.9, -0.55), p(0, -0.55)], (side, 0, 0), SPOKE3))
    faces.append(([(xs + 0.01 * side, cy + 0.9 * math.cos(2 * math.pi * k / 8), cz + 0.9 * math.sin(2 * math.pi * k / 8)) for k in range(8)], (side, 0, 0), (235, 240, 245)))
    return faces


def _wheels(d):
    return _wheel3(-1, d['al']) + _wheel3(1, d['ar'])


def _arm(side, deg, claw=CLAW):
    b = math.radians(deg)
    a = math.pi / 2 - b if side < 0 else math.pi / 2 + b
    return _xform(_box(0.0, -ph.ARM_T / 2, claw['z0'], claw['length'], ph.ARM_T / 2, claw['z1'], ARM_COL), a, side * claw['hinge'], claw['hinge_y'])


def faces_simple(d):
    f = _box(-6.0, -6.0, 2.0, 6.0, 6.0, 5.0, BODY)
    f += _box(-2.0, 5.0, 5.0, 2.0, 6.0, 5.4, NOSE)
    f += _wheels(d)
    f.append(([(-2.5, -3, 5.02), (2.5, -3, 5.02), (0, 4.5, 5.02)], (0, 0, 1), DECK))
    f += _box(-3.4, 6.0, 0.9, 3.4, 7.6, 3.3, BASE_COL)
    f += _arm(-1, d['arm_l']) + _arm(1, d['arm_r'])
    return f


def faces_tower(d):
    lift, ext = d['lift'], d['pad']
    f = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            f += _box(sx * 5.0 - 0.5, sy * 6.8 - 0.5, 0.0, sx * 5.0 + 0.5, sy * 6.8 + 0.5, BODY_Z[0], POST_COL)
    for cx, cy, hx, hy, z0, z1 in PARTS['tower'][:4]:
        f += _box(cx - hx, cy - hy, z0, cx + hx, cy + hy, z1, BODY)
    f += _wheels(d)
    f += _box(-1.0, -3.9, 8.0, 1.0, -2.7, 20.0, MAST_COL)                      # rack column
    f += _box(-1.7, -4.5, 18.0, 1.7, -2.2, 20.0, SERVO_COL)                    # lift servo at the top
    f += _box(-1.0, -2.7, lift + 5.0, 1.0, -2.15, lift + 7.5, ARM_COL)         # carriage from the rack to the box
    for cx, cy, hx, hy in ((0.0, tw.BOX_IN / 2 + tw.BOX_WALL / 2, tw.BOX_OUT / 2, tw.BOX_WALL / 2), (0.0, -tw.BOX_IN / 2 - tw.BOX_WALL / 2, tw.BOX_OUT / 2, tw.BOX_WALL / 2),
                           (tw.BOX_IN / 2 + tw.BOX_WALL / 2, 0.0, tw.BOX_WALL / 2, tw.BOX_IN / 2), (-tw.BOX_IN / 2 - tw.BOX_WALL / 2, 0.0, tw.BOX_WALL / 2, tw.BOX_IN / 2)):
        f += _box(cx - hx, cy - hy, lift, cx + hx, cy + hy, lift + tw.BOX_H, ('wire', TOWER_COL))     # the box walls are drawn as a see-through frame
    f += _box(tw.BOX_IN / 2 - ext, -tw.PAD_Y_HALF, lift + tw.PAD_Z0, tw.BOX_IN / 2, tw.PAD_Y_HALF, lift + tw.PAD_Z1, ARM_COL)    # the pad
    f += _box(tw.BOX_OUT / 2, -1.4, lift + 0.8, tw.BOX_OUT / 2 + 1.2, 1.4, lift + 3.6, SERVO_COL)                                 # pad servo
    f.append(([(-2.5, -6.5, 8.02), (2.5, -6.5, 8.02), (0, -3.6, 8.02)], (0, 0, 1), DECK))
    f += _box(-3.0, 7.2, 8.0, 3.0, 8.0, 8.4, NOSE)                              # front marker
    f += [([(cx + px, cy + py, lift + tw.BOX_H + 0.01) for px, py in ((-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3))], (0, 0, 1), TOWER_COL) for cx, cy in ((0, 0),)]
    return f


def robot_faces3(d):
    f = faces_simple(d) if d['type'] == 'simple' else faces_tower(d)
    for cx, cy, z, yaw, team in d.get('cubes', []):                              # cubes the robot reports it is carrying (real data later)
        c, s = math.cos(yaw), math.sin(yaw)
        f += _xform(_box(-1.5, -1.5, 0.0, 1.5, 1.5, 3.0, CUBE_COL.get(team, CUBE_COL['B'])), yaw, cx, cy, z)
    return f


def _items3(cam, faces, W, N, out):
    for pts, n, col in faces:
        wire = isinstance(col[0], str)
        if wire:
            col = col[1]
        wp, wn = [W(p) for p in pts], N(n)
        ctr = tuple(sum(p[k] for p in wp) / len(wp) for k in range(3))
        if not wire:
            view = (cam.eye[0] - ctr[0], cam.eye[1] - ctr[1], cam.eye[2] - ctr[2])
            if wn[0] * view[0] + wn[1] * view[1] + wn[2] * view[2] <= 0:
                continue
        shade = 0.5 + 0.5 * max(0.0, wn[0] * LIGHT[0] + wn[1] * LIGHT[1] + wn[2] * LIGHT[2])
        out.append((cam.cam(ctr)[2], tuple(int(v * shade) for v in col) if not wire else col, wp, wire))


def _box_world3(cx, cy, hx, hy, ang, z0, z1, col, cam, out, seg=None):
    n = max(1, round(2 * hx / seg)) if seg else 1
    h = hx / n
    c, s = math.cos(ang), math.sin(ang)
    for k in range(n):
        off = -hx + (k + 0.5) * 2 * hx / n
        _items3(cam, _box(-h, -hy, z0, h, hy, z1, col), lambda p: (cx + (p[0] + off) * c - p[1] * s, cy + (p[0] + off) * s + p[1] * c, p[2]),
                lambda nv: (nv[0] * c - nv[1] * s, nv[0] * s + nv[1] * c, nv[2]), out)


def _paint(surf, cam, items):
    for _, col, wp, wire in sorted(items, key=lambda t: -t[0]):
        if wire:
            poly(surf, cam, col, wp, 1)
        else:
            poly(surf, cam, col, wp)
            poly(surf, cam, tuple(int(v * 0.6) for v in col), wp, 1)


def _robot_items(cam, d, ox, oy, items):
    s, co = math.sin(d['th']), math.cos(d['th'])
    fwd, rgt = (-s, co), (co, s)
    Wr = lambda p: (ox + p[0] * rgt[0] + p[1] * fwd[0], oy + p[0] * rgt[1] + p[1] * fwd[1], p[2])
    Nr = lambda n: (n[0] * rgt[0] + n[1] * fwd[0], n[0] * rgt[1] + n[1] * fwd[1], n[2])
    _items3(cam, robot_faces3(d), Wr, Nr, items)
    return Wr


class Look:
    """What the person changed with the mouse and the options row: orbit angles, zoom, and the three overlays."""
    def __init__(self):
        self.reset_cam()
        self.trail_on, self.trail_i = True, 0.55
        self.shadow_on, self.shadow_i = True, 0.35
        self.arrow_on, self.arrow_i = True, 0.8

    def reset_cam(self):
        self.yaw, self.pitch, self.dist, self.zoom = 0.5, 0.55, 80.0, 1.0

    def drag(self, dx, dy):
        self.yaw -= dx * 0.008
        self.pitch = clamp(self.pitch + dy * 0.008, 0.05, 1.5)

    def wheel(self, n):
        self.zoom = clamp(self.zoom * (0.88 ** n), 0.15, 5.0)


def set_view3(cam, view, x, y, th, robot='simple', look=None):
    """arena / chase / top as before, plus front / right / left (and back) of the robot, which turn with it."""
    set_view_base(cam, view, x, y, th, robot, look or Look())


def _zl(cam, look, eye, target, up=(0, 0, 1), zoomed=True):
    """cam.look with the zoom applied: the eye moves toward / away from what it looks at."""
    z = look.zoom if zoomed else 1.0
    cam.look(tuple(t + (e - t) * z for e, t in zip(eye, target)), target, up=up)


def set_view_base(cam, view, x, y, th, robot, look):
    s, c = math.sin(th), math.cos(th)
    fwd, rgt = (-s, c), (c, s)
    zc = 4.5 if robot == 'simple' else 9.0
    if view == 'free':
        cam.fl = 400
        d = look.dist
        _zl(cam, look, (x + d * math.cos(look.pitch) * math.sin(look.yaw), y - d * math.cos(look.pitch) * math.cos(look.yaw), zc + d * math.sin(look.pitch)), (x, y, zc))
    elif view in ('front', 'right', 'left', 'back'):
        cam.fl = 400
        d = 50.0
        v = {'front': fwd, 'back': (-fwd[0], -fwd[1]), 'right': rgt, 'left': (-rgt[0], -rgt[1])}[view]
        _zl(cam, look, (x + v[0] * d, y + v[1] * d, zc + 24), (x, y, zc))
    elif view == 'chase':
        cam.fl = 430
        _zl(cam, look, (x + s * 52, y - c * 52, 34), (x - s * 14, y + c * 14, 3))
    elif view == 'top':
        cam.fl = 480
        _zl(cam, look, (0, 0, 265), (0, 0, 0), up=(0, 1, 0))
    elif view == 'iso':
        cam.fl = 400
        _zl(cam, look, (x + 46, y - 60, zc + 46), (x, y, zc))
    else:
        cam.fl = 395
        _zl(cam, look, (0, -175, 205), (0, 0, 0))


def _trail(surf, cam, sim, look):
    if not look.trail_on:
        return
    k = look.trail_i
    for i, (tx, ty) in enumerate(sim.trail):
        cc = cam.cam((tx, ty, 0.1))
        if cc[2] > NEAR:
            a = (0.25 + 0.75 * i / max(1, len(sim.trail))) * k * 1.6
            pygame.draw.circle(surf, (int(20 * a), min(255, int(200 * a)), min(255, int(170 * a))), cam.screen(cc), 2)


def _shadow(surf, cam, pts, look):
    """A see-through dark patch under the robot (alpha follows the Shadow setting), so the floor and the wheels stay visible."""
    if not look.shadow_on or look.shadow_i <= 0:
        return
    sp = []
    for p in pts:
        cc = cam.cam(p)
        if cc[2] <= NEAR:
            return
        sp.append(cam.screen(cc))
    x0, y0 = int(min(p[0] for p in sp)) - 1, int(min(p[1] for p in sp)) - 1
    w, h = int(max(p[0] for p in sp)) - x0 + 2, int(max(p[1] for p in sp)) - y0 + 2
    if w < 2 or h < 2 or w > 2000 or h > 2000:
        return
    layer = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.polygon(layer, (0, 0, 0, int(200 * look.shadow_i)), [(p[0] - x0, p[1] - y0) for p in sp])
    surf.blit(layer, (x0, y0))


def _zero_arrow(surf, cam, x, y, ref_th, look, z=0.2):
    if not look.arrow_on:
        return
    col = tuple(int(v * look.arrow_i) for v in MARK_COL)
    zx, zy = -math.sin(ref_th), math.cos(ref_th)
    tip = (x + zx * 30, y + zy * 30, z)
    line(surf, cam, col, (x, y, z), tip, 2)
    line(surf, cam, col, tip, (x + zx * 25 - zy * 3, y + zy * 25 + zx * 3, z), 2)
    line(surf, cam, col, tip, (x + zx * 25 + zy * 3, y + zy * 25 - zx * 3, z), 2)


def draw_scene3(surf, cam, sim, view, look=None):
    """The arena: floor, walls, 5 mm pillars, cubes and the robot, depth sorted together."""
    surf.fill((20, 23, 28))
    d = sim.snapshot()
    look = look or Look()
    set_view3(cam, view, sim.x, sim.y, sim.th, sim.robot_type, look)
    draw_floor(surf, cam)
    _trail(surf, cam, sim, look)
    _zero_arrow(surf, cam, sim.x, sim.y, sim.ref_th, look)
    w = sim.world
    for c in w.cubes:
        sup = w.support_of(c)[0]
        if c.z > sup + 0.05:
            poly(surf, cam, (18, 20, 24), rect3(c.x - 1.5, c.y - 1.5, c.x + 1.5, c.y + 1.5, sup + 0.07))
    _shadow(surf, cam, [sim.to_world(x, y) + (0.1,) for x, y in ((-8, -8), (8, -8), (8, 8), (-8, 8))] if sim.robot_type == 'tower' else [sim.to_world(x, y) + (0.1,) for x, y in ((-8, -7), (8, -7), (8, 14), (-8, 14))], look)
    items = []
    for st in w.statics:
        if st.tag == 'wall':
            _box_world3(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, ph.WALL_H, WALL, cam, items, seg=8.0)
        elif st.tag == 'pillar':
            _box_world3(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, w.pillar_h, PILLAR_COL, cam, items)
    for c in w.cubes:
        _box_world3(c.x, c.y, ph.CUBE_S / 2, ph.CUBE_S / 2, c.th, c.z, c.z + ph.CUBE_S, CUBE_COL[c.team], cam, items)
    _robot_items(cam, dict(d, x=sim.x), sim.x, sim.y, items)
    _paint(surf, cam, items)
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


def draw_monitor3(surf, cam, d, view, look=None):
    """Monitor: the robot stays at the origin, turning in place. The grid under it slides as the robot 'moves'. d is a snapshot() dict."""
    look = look or Look()
    surf.fill((20, 23, 28))
    set_view3(cam, view, 0.0, 0.0, d['th'], d['type'], look)
    R = 64.0
    poly(surf, cam, (30, 34, 40), rect3(-R, -R, R, R, 0.0))
    gx0 = int(math.floor((d['x'] - R) / 10.0)) * 10
    for g in range(gx0, int(d['x'] + R) + 10, 10):
        x = g - d['x']
        if -R <= x <= R:
            line(surf, cam, (92, 102, 114) if g % 50 == 0 else (52, 58, 66), (x, -R, 0.02), (x, R, 0.02), 1)
    gy0 = int(math.floor((d['y'] - R) / 10.0)) * 10
    for g in range(gy0, int(d['y'] + R) + 10, 10):
        y = g - d['y']
        if -R <= y <= R:
            line(surf, cam, (92, 102, 114) if g % 50 == 0 else (52, 58, 66), (-R, y, 0.02), (R, y, 0.02), 1)
    for a, b in (((-R, -R), (R, -R)), ((R, -R), (R, R)), ((R, R), (-R, R)), ((-R, R), (-R, -R))):
        line(surf, cam, (96, 106, 118), (a[0], a[1], 0.03), (b[0], b[1], 0.03), 2)
    line(surf, cam, (255, 120, 110), (0, R - 10, 0.05), (0, R, 0.05), 3)                      # north mark: the zero direction of the map
    _zero_arrow(surf, cam, 0.0, 0.0, d['ref_th'], look, 0.2)
    _shadow(surf, cam, [(-8, -8, 0.1), (8, -8, 0.1), (8, 8, 0.1), (-8, 8, 0.1)], look)
    items = []
    _robot_items(cam, d, 0.0, 0.0, items)
    _paint(surf, cam, items)
