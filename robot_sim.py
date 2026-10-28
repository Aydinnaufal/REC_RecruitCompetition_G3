"""
robot_sim.py - a simple differential-drive robot (a box with two wheels) and a small 3D view of it.

RobotSim   keeps the pose (x, y, heading) and the wheel speeds, moved by controller input.
draw_scene draws the arena floor, the robot and its trail with a basic perspective camera (pygame.draw only).

Units: centimetres. The world is the REC arena: x to the right, y up the map, z up. Internally the heading th is 0 when facing +y (up the map) and grows when turning left
(counter-clockwise seen from above); heading_deg is the compass-style number shown on screen.
"""
import math
from collections import deque
import pygame

TRACK = 15.0          # distance between the two wheels (cm)
WHEEL_R, WHEEL_W = 3.0, 2.4
MAX_SPEED = 30.0      # cm/s at full stick
TURN_GAIN = 0.55      # arcade mode: how much of the full turning speed the left/right stick gives
FIELD = 105.0         # half of the 210 cm field


def clamp(v, lo, hi): return max(lo, min(hi, v))


class RobotSim:
    def __init__(self):
        self.mode = 'arcade'               # 'arcade': left stick drives and steers; 'tank': left stick Y = left wheel, right stick Y = right wheel
        self.trail = deque(maxlen=500)
        self.reset()

    def reset(self):
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vl = self.vr = 0.0            # wheel speeds, -1 .. 1
        self.al = self.ar = 0.0            # wheel rotation angles (rad), for the spoke
        self.speed = 0.0
        self.trail.clear()
        self._t = 0.0

    def drive(self, st, dt):
        """st: ControllerState (stick up = +, stick right = +). dt in seconds."""
        if self.mode == 'arcade':
            l, r = clamp(st.ly + TURN_GAIN * st.lx, -1, 1), clamp(st.ly - TURN_GAIN * st.lx, -1, 1)
        else:
            l, r = st.ly, st.ry
        k = min(1.0, dt * 9.0)             # motors do not change speed instantly
        self.vl += (l - self.vl) * k
        self.vr += (r - self.vr) * k
        vl, vr = self.vl * MAX_SPEED, self.vr * MAX_SPEED
        self.speed = (vl + vr) / 2.0
        self.th += (vr - vl) / TRACK * dt                  # right wheel faster -> turns left
        self.x += -math.sin(self.th) * self.speed * dt
        self.y += math.cos(self.th) * self.speed * dt
        self.x, self.y = clamp(self.x, -FIELD + 8, FIELD - 8), clamp(self.y, -FIELD + 8, FIELD - 8)
        self.al += vl * dt / WHEEL_R
        self.ar += vr * dt / WHEEL_R
        self._t += dt
        if self._t > 0.06 and abs(self.speed) > 0.5:
            self._t = 0.0
            self.trail.append((self.x, self.y))

    @property
    def heading_deg(self):
        """Compass style: 0 = up the map, turning right (clockwise) increases it."""
        return (-math.degrees(self.th)) % 360


# ---------------------------------------------------------------- camera
class Camera:
    def __init__(self, w, h, fl):
        self.cx, self.cy, self.fl = w / 2.0, h / 2.0, fl
        self.eye = (0, -100, 100); self.f = (0, 1, 0); self.r = (1, 0, 0); self.u = (0, 0, 1)

    @staticmethod
    def _norm(v):
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return tuple(c / n for c in v)

    def look(self, eye, target, up=(0, 0, 1)):
        f = self._norm(tuple(t - e for t, e in zip(target, eye)))
        r = self._norm((f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]))
        u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
        self.eye, self.f, self.r, self.u = eye, f, r, u

    def cam(self, p):
        d = (p[0] - self.eye[0], p[1] - self.eye[1], p[2] - self.eye[2])
        dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
        return (dot(d, self.r), dot(d, self.u), dot(d, self.f))

    def screen(self, c):
        return (self.cx + self.fl * c[0] / c[2], self.cy - self.fl * c[1] / c[2])


NEAR = 2.0


def _clip_poly(pc):
    """Clip a polygon in camera space against the near plane."""
    out = []
    for i, a in enumerate(pc):
        b = pc[(i + 1) % len(pc)]
        ain, bin_ = a[2] >= NEAR, b[2] >= NEAR
        if ain: out.append(a)
        if ain != bin_:
            t = (NEAR - a[2]) / (b[2] - a[2])
            out.append(tuple(a[k] + (b[k] - a[k]) * t for k in range(3)))
    return out


def poly(surf, cam, color, pts, width=0):
    pc = _clip_poly([cam.cam(p) for p in pts])
    if len(pc) >= 3:
        pygame.draw.polygon(surf, color, [cam.screen(c) for c in pc], width)


def line(surf, cam, color, a, b, width=1):
    ca, cb = cam.cam(a), cam.cam(b)
    if ca[2] < NEAR and cb[2] < NEAR: return
    if ca[2] < NEAR: ca = tuple(ca[k] + (cb[k] - ca[k]) * (NEAR - ca[2]) / (cb[2] - ca[2]) for k in range(3))
    if cb[2] < NEAR: cb = tuple(cb[k] + (ca[k] - cb[k]) * (NEAR - cb[2]) / (ca[2] - cb[2]) for k in range(3))
    pygame.draw.line(surf, color, cam.screen(ca), cam.screen(cb), width)


def rect3(x0, y0, x1, y1, z=0.0): return [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]


# ---------------------------------------------------------------- arena floor
FLOOR = (34, 38, 44)
ZONES = [((-105, -105, -35, 105), (44, 58, 108)), ((35, -105, 105, 105), (112, 52, 56)),
         ((-35, 35, 35, 105), (62, 72, 118)), ((-35, -35, 35, 35), (74, 78, 84)), ((-35, -105, 35, -35), (104, 64, 68))]
WALL = (196, 154, 108)
WALLS = [((-35, -105), (-35, -12.5)), ((-35, 12.5), (-35, 57.5)), ((-35, 82.5), (-35, 105)),
         ((35, -105), (35, -82.5)), ((35, -57.5), (35, -12.5)), ((35, 12.5), (35, 105)),
         ((-35, 35), (35, 35)), ((-35, -35), (35, -35)), ((-35, 105), (35, 105)), ((-35, -105), (35, -105))]


def draw_floor(surf, cam):
    poly(surf, cam, FLOOR, rect3(-FIELD, -FIELD, FIELD, FIELD))
    for (x0, y0, x1, y1), c in ZONES:
        poly(surf, cam, c, rect3(x0, y0, x1, y1, 0.01))
    for g in range(-100, 101, 10):
        c = (58, 64, 72) if g % 30 else (74, 82, 92)
        line(surf, cam, c, (g, -FIELD, 0.02), (g, FIELD, 0.02), 1)
        line(surf, cam, c, (-FIELD, g, 0.02), (FIELD, g, 0.02), 1)
    # start boxes, markers, storing zones, pillars
    for sx, sy, col in ((-82, -92, (120, 150, 255)), (82, 92, (255, 130, 130))):
        poly(surf, cam, col, rect3(sx - 10, sy - 10, sx + 10, sy + 10, 0.03), 2)
    for my in (90, 0, -90):
        poly(surf, cam, (10, 10, 10), rect3(-5, my - 5, 5, my + 5, 0.04))
        poly(surf, cam, (200, 205, 210), rect3(-2.5, my - 2.5, 2.5, my + 2.5, 0.05))
    for sgn, col in ((1, (120, 150, 255)), (-1, (255, 130, 130))):
        poly(surf, cam, col, rect3(sgn * -102, -20, sgn * -76, 20, 0.03), 1)
        for px in (-95, -83):
            for py in (-13, 0, 13):
                poly(surf, cam, (235, 235, 235), rect3(sgn * px - 2, sgn * py - 2, sgn * px + 2, sgn * py + 2, 0.06))
    for a, b in WALLS:
        line(surf, cam, WALL, (a[0], a[1], 0.08), (b[0], b[1], 0.08), 3)
    for a, b in (((-FIELD, -FIELD), (FIELD, -FIELD)), ((FIELD, -FIELD), (FIELD, FIELD)), ((FIELD, FIELD), (-FIELD, FIELD)), ((-FIELD, FIELD), (-FIELD, -FIELD))):
        line(surf, cam, (150, 160, 172), (a[0], a[1], 0.08), (b[0], b[1], 0.08), 2)


# ---------------------------------------------------------------- the robot
BODY = (96, 112, 138)
DECK = (140, 156, 182)
NOSE = (0, 200, 170)
TYRE = (30, 32, 36)
LIGHT = (-0.35, -0.45, 0.82)


def _box(x0, y0, z0, x1, y1, z1, color):
    """Faces of an axis-aligned box in robot coordinates: (points, outward normal, colour)."""
    return [([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)], (0, 0, -1), color),
            ([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1), color),
            ([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0), color),
            ([(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)], (0, 1, 0), color),
            ([(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)], (-1, 0, 0), color),
            ([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0), color)]


def _wheel(side, angle, n=14):
    """A wheel as an n-sided prism whose axis is the robot's x axis; side = -1 left, +1 right."""
    xc = side * TRACK / 2.0
    xi, xo = xc - WHEEL_W / 2, xc + WHEEL_W / 2
    cy, cz = -0.5, WHEEL_R
    faces = []
    ring = [(cy + WHEEL_R * math.cos(2 * math.pi * k / n), cz + WHEEL_R * math.sin(2 * math.pi * k / n)) for k in range(n)]
    for k in range(n):
        (ya, za), (yb, zb) = ring[k], ring[(k + 1) % n]
        mid = 2 * math.pi * (k + 0.5) / n
        faces.append(([(xi, ya, za), (xo, ya, za), (xo, yb, zb), (xi, yb, zb)], (0, math.cos(mid), math.sin(mid)), TYRE))
    for x, nx in ((xi, -1), (xo, 1)):
        faces.append(([(x, y, z) for y, z in ring], (nx, 0, 0), (52, 56, 64)))
    xs = xo + 0.05 * side if side > 0 else xi - 0.05          # spoke on the outer face: shows the wheel turning
    for off in (0, math.pi):
        a = angle + off
        p = lambda r, w: (xs, cy + r * math.cos(a) - w * math.sin(a), cz + r * math.sin(a) + w * math.cos(a))
        faces.append(([p(0, 0.35), p(WHEEL_R * 0.92, 0.35), p(WHEEL_R * 0.92, -0.35), p(0, -0.35)], (side, 0, 0), (0, 200, 170)))
    return faces


def robot_faces(sim):
    f = []
    f += _box(-6.0, -7.0, 2.0, 6.0, 7.0, 5.0, BODY)                  # chassis
    f += _box(-4.0, 7.0, 2.6, 4.0, 10.5, 4.4, NOSE)                  # front bay (where the gripper will go)
    f += _wheel(-1, sim.al) + _wheel(1, sim.ar)
    f.append(([(-2.5, -3, 5.02), (2.5, -3, 5.02), (0, 4.5, 5.02)], (0, 0, 1), DECK))   # arrow on top: heading
    return f


def draw_robot(surf, cam, sim):
    s, c = math.sin(sim.th), math.cos(sim.th)
    fwd, rgt = (-s, c), (c, s)
    W = lambda p: (sim.x + p[0] * rgt[0] + p[1] * fwd[0], sim.y + p[0] * rgt[1] + p[1] * fwd[1], p[2])
    N = lambda n: (n[0] * rgt[0] + n[1] * fwd[0], n[0] * rgt[1] + n[1] * fwd[1], n[2])
    poly(surf, cam, (22, 25, 29), [W((-8, -9, 0.1)), W((8, -9, 0.1)), W((8, 12, 0.1)), W((-8, 12, 0.1))])   # shadow
    items = []
    for pts, n, col in robot_faces(sim):
        wp, wn = [W(p) for p in pts], N(n)
        ctr = tuple(sum(p[k] for p in wp) / len(wp) for k in range(3))
        view = (cam.eye[0] - ctr[0], cam.eye[1] - ctr[1], cam.eye[2] - ctr[2])
        if wn[0] * view[0] + wn[1] * view[1] + wn[2] * view[2] <= 0: continue      # facing away
        shade = 0.5 + 0.5 * max(0.0, wn[0] * LIGHT[0] + wn[1] * LIGHT[1] + wn[2] * LIGHT[2])
        depth = cam.cam(ctr)[2]
        items.append((depth, tuple(int(v * shade) for v in col), wp))
    for _, col, wp in sorted(items, key=lambda t: -t[0]):
        poly(surf, cam, col, wp)
        poly(surf, cam, tuple(int(v * 0.6) for v in col), wp, 1)
    cb = W((0, -5.5, 1.0))                                                          # rear ball caster
    cc = cam.cam(cb)
    if cc[2] > NEAR:
        pygame.draw.circle(surf, (180, 186, 194), cam.screen(cc), max(2, int(cam.fl * 1.0 / cc[2])))


VIEWS = ['arena', 'chase', 'top']


def set_view(cam, view, sim):
    s, c = math.sin(sim.th), math.cos(sim.th)
    if view == 'chase':
        cam.fl = 430
        cam.look((sim.x + s * 52, sim.y - c * 52, 34), (sim.x - s * 14, sim.y + c * 14, 3))
    elif view == 'top':
        cam.fl = 480
        cam.look((0, 0, 265), (0, 0, 0), up=(0, 1, 0))
    else:
        cam.fl = 395
        cam.look((0, -175, 205), (0, 0, 0))


def draw_scene(surf, cam, sim, view):
    surf.fill((20, 23, 28))
    set_view(cam, view, sim)
    draw_floor(surf, cam)
    for i, (tx, ty) in enumerate(sim.trail):
        cc = cam.cam((tx, ty, 0.1))
        if cc[2] > NEAR:
            a = 0.25 + 0.75 * i / max(1, len(sim.trail))
            pygame.draw.circle(surf, (int(0 * a + 20), int(200 * a), int(170 * a)), cam.screen(cc), 2)
    draw_robot(surf, cam, sim)
