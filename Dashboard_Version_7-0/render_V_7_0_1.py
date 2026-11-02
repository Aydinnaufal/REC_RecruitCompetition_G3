"""render_V_7_0_1.py - draws the 3D picture: the arena with the tower robot (Game) or the robot alone on a sliding grid (Monitor).
A small perspective camera and pygame.draw only. Overlays (trail, shadow, zero arrow) are drawn on a transparent layer, so their
setting is a true opacity."""
import math
import pygame
import sim_V_7_0_1 as sm

NEAR = 2.0
WHEEL_R, WHEEL_W = 3.0, 2.4
VIEWS_GAME = ['arena', 'chase', 'top', 'front', 'right', 'left', 'free']
VIEWS_MONITOR = ['iso', 'front', 'right', 'left', 'back', 'top', 'free']
VIEW_LABEL = {'arena': 'Arena', 'chase': 'Chase (behind)', 'top': 'Top', 'front': 'Front', 'right': 'Right side', 'left': 'Left side',
              'iso': 'Angled', 'back': 'Back', 'free': 'Free (mouse)'}

BACKGROUND = (20, 23, 28)
FLOOR = (34, 38, 44)
ZONES = [((-105, -105, -35, 105), (44, 58, 108)), ((35, -105, 105, 105), (112, 52, 56)),
         ((-35, 35, 35, 105), (62, 72, 118)), ((-35, -35, 35, 35), (74, 78, 84)), ((-35, -105, 35, -35), (104, 64, 68))]
WALL_COL, PILLAR_COL = (196, 154, 108), (214, 216, 220)
CUBE_COL = {'B': (48, 96, 232), 'R': (226, 66, 62)}
BODY, DECK, NOSE = (96, 112, 138), (140, 156, 182), (0, 200, 170)
TOWER_COL, MAST_COL, SERVO_COL, POST_COL, ARM_COL = (60, 200, 190), (74, 82, 98), (44, 48, 58), (64, 70, 82), (238, 168, 58)
TYRE, RIM, SPOKE = (62, 66, 76), (88, 94, 106), (0, 235, 195)
MARK_COL = (255, 200, 90)
LIGHT = (-0.35, -0.45, 0.82)


def clamp(v, lo, hi): return max(lo, min(hi, v))


# ---------------------------------------------------------------- camera
class Camera:
    def __init__(self, w, h, fl):
        self.cx, self.cy, self.fl = w / 2.0, h / 2.0, fl
        self.eye, self.f, self.r, self.u = (0, -100, 100), (0, 1, 0), (1, 0, 0), (0, 0, 1)

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

    def screen(self, c): return (self.cx + self.fl * c[0] / c[2], self.cy - self.fl * c[1] / c[2])


def _clip_poly(pc):
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
def draw_floor(surf, cam):
    F = sm.FIELD
    poly(surf, cam, FLOOR, rect3(-F, -F, F, F))
    for (x0, y0, x1, y1), c in ZONES:
        poly(surf, cam, c, rect3(x0, y0, x1, y1, 0.01))
    for g in range(-100, 101, 10):
        c = (58, 64, 72) if g % 30 else (74, 82, 92)
        line(surf, cam, c, (g, -F, 0.02), (g, F, 0.02), 1)
        line(surf, cam, c, (-F, g, 0.02), (F, g, 0.02), 1)
    for sx, sy, col in ((-82, -92, (120, 150, 255)), (82, 92, (255, 130, 130))):                    # start boxes
        poly(surf, cam, col, rect3(sx - 10, sy - 10, sx + 10, sy + 10, 0.03), 2)
    for my in (90, 0, -90):                                                                          # markers
        poly(surf, cam, (10, 10, 10), rect3(-5, my - 5, 5, my + 5, 0.04))
        poly(surf, cam, (200, 205, 210), rect3(-2.5, my - 2.5, 2.5, my + 2.5, 0.05))
    for sgn, col in ((1, (120, 150, 255)), (-1, (255, 130, 130))):                                   # storing zones and pillar squares
        poly(surf, cam, col, rect3(sgn * -102, -20, sgn * -76, 20, 0.03), 1)
        for px in (-95, -83):
            for py in (-13, 0, 13):
                poly(surf, cam, (235, 235, 235), rect3(sgn * px - 2, sgn * py - 2, sgn * px + 2, sgn * py + 2, 0.06))
    for a, b in sm.WALLS:
        line(surf, cam, WALL_COL, (a[0], a[1], 0.08), (b[0], b[1], 0.08), 3)
    for a, b in (((-F, -F), (F, -F)), ((F, -F), (F, F)), ((F, F), (-F, F)), ((-F, F), (-F, -F))):
        line(surf, cam, (150, 160, 172), (a[0], a[1], 0.08), (b[0], b[1], 0.08), 2)


# ---------------------------------------------------------------- shapes
def _box(x0, y0, z0, x1, y1, z1, color):
    """Faces of an axis-aligned box: (points, outward normal, colour)."""
    return [([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)], (0, 0, -1), color),
            ([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1), color),
            ([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0), color),
            ([(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)], (0, 1, 0), color),
            ([(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)], (-1, 0, 0), color),
            ([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0), color)]


def _wheel(side, angle, n=14):
    """A wheel as an n-sided prism (axis = robot x) with four bright spokes and a hub, so its turning can be seen."""
    xc = side * sm.TRACK / 2.0
    xi, xo = xc - WHEEL_W / 2, xc + WHEEL_W / 2
    cz = WHEEL_R
    ring = [(WHEEL_R * math.cos(2 * math.pi * k / n), cz + WHEEL_R * math.sin(2 * math.pi * k / n)) for k in range(n)]
    faces = []
    for k in range(n):
        (ya, za), (yb, zb) = ring[k], ring[(k + 1) % n]
        mid = 2 * math.pi * (k + 0.5) / n
        faces.append(([(xi, ya, za), (xo, ya, za), (xo, yb, zb), (xi, yb, zb)], (0, math.cos(mid), math.sin(mid)), TYRE))
    for x, nx in ((xi, -1), (xo, 1)):
        faces.append(([(x, y, z) for y, z in ring], (nx, 0, 0), RIM))
    xs = xo + 0.06 if side > 0 else xi - 0.06
    for k in range(4):
        a = angle + k * math.pi / 2
        p = lambda r, w: (xs, r * math.cos(a) - w * math.sin(a), cz + r * math.sin(a) + w * math.cos(a))
        faces.append(([p(0, 0.55), p(WHEEL_R * 0.9, 0.55), p(WHEEL_R * 0.9, -0.55), p(0, -0.55)], (side, 0, 0), SPOKE))
    faces.append(([(xs + 0.01 * side, 0.9 * math.cos(2 * math.pi * k / 8), cz + 0.9 * math.sin(2 * math.pi * k / 8)) for k in range(8)], (side, 0, 0), (235, 240, 245)))
    return faces


def _xform(faces, ang, ox, oy, oz=0.0):
    c, s = math.cos(ang), math.sin(ang)
    return [([(ox + p[0] * c - p[1] * s, oy + p[0] * s + p[1] * c, oz + p[2]) for p in pts], (n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]), col)
            for pts, n, col in faces]


def tower_faces(d):
    lift, ext = d['lift'], d['pad']
    f = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            f += _box(sx * 5.0 - 0.5, sy * 6.8 - 0.5, 0.0, sx * 5.0 + 0.5, sy * 6.8 + 0.5, sm.BODY_Z[0], POST_COL)          # chassis posts
    for cx, cy, hx, hy, z0, z1 in sm.PARTS[:4]:
        f += _box(cx - hx, cy - hy, z0, cx + hx, cy + hy, z1, BODY)                                                         # chassis plates
    f += _wheel(-1, d['al']) + _wheel(1, d['ar'])
    f += _box(-1.0, -3.9, 8.0, 1.0, -2.7, 20.0, MAST_COL)                                                                    # rack column
    f += _box(-1.7, -4.5, 18.0, 1.7, -2.2, 20.0, SERVO_COL)                                                                  # lift servo
    f += _box(-1.0, -2.7, lift + 5.0, 1.0, -2.15, lift + 7.5, ARM_COL)                                                       # carriage
    i, w, o = sm.BOX_IN, sm.BOX_WALL, sm.BOX_OUT
    for cx, cy, hx, hy in ((0.0, i / 2 + w / 2, o / 2, w / 2), (0.0, -i / 2 - w / 2, o / 2, w / 2), (i / 2 + w / 2, 0.0, w / 2, i / 2), (-i / 2 - w / 2, 0.0, w / 2, i / 2)):
        f += _box(cx - hx, cy - hy, lift, cx + hx, cy + hy, lift + sm.BOX_H, ('wire', TOWER_COL))                            # the box, see-through
    f += _box(i / 2 - ext, -sm.PAD_Y_HALF, lift + sm.PAD_Z0, i / 2, sm.PAD_Y_HALF, lift + sm.PAD_Z1, ARM_COL)               # the pad
    f += _box(o / 2, -1.4, lift + 0.8, o / 2 + 1.2, 1.4, lift + 3.6, SERVO_COL)                                              # pad servo
    f.append(([(-2.5, -6.5, 8.02), (2.5, -6.5, 8.02), (0, -3.6, 8.02)], (0, 0, 1), DECK))                                    # arrow on the chassis
    f += _box(-3.0, 7.2, 8.0, 3.0, 8.0, 8.4, NOSE)                                                                           # front marker
    return f


def _items(cam, faces, W, N, out):
    for pts, n, col in faces:
        wire = isinstance(col[0], str)
        if wire: col = col[1]
        wp, wn = [W(p) for p in pts], N(n)
        ctr = tuple(sum(p[k] for p in wp) / len(wp) for k in range(3))
        if not wire:
            view = (cam.eye[0] - ctr[0], cam.eye[1] - ctr[1], cam.eye[2] - ctr[2])
            if wn[0] * view[0] + wn[1] * view[1] + wn[2] * view[2] <= 0:
                continue
        shade = 0.5 + 0.5 * max(0.0, wn[0] * LIGHT[0] + wn[1] * LIGHT[1] + wn[2] * LIGHT[2])
        out.append((cam.cam(ctr)[2], col if wire else tuple(int(v * shade) for v in col), wp, wire))


def _box_world(cx, cy, hx, hy, ang, z0, z1, col, cam, out, seg=None):
    """A box at a world position. With seg a long wall is cut into pieces so it depth-sorts correctly."""
    n = max(1, round(2 * hx / seg)) if seg else 1
    h = hx / n
    c, s = math.cos(ang), math.sin(ang)
    for k in range(n):
        off = -hx + (k + 0.5) * 2 * hx / n
        _items(cam, _box(-h, -hy, z0, h, hy, z1, col), lambda p: (cx + (p[0] + off) * c - p[1] * s, cy + (p[0] + off) * s + p[1] * c, p[2]),
               lambda nv: (nv[0] * c - nv[1] * s, nv[0] * s + nv[1] * c, nv[2]), out)


def _robot_items(cam, d, ox, oy, out):
    s, co = math.sin(d['th']), math.cos(d['th'])
    fwd, rgt = (-s, co), (co, s)
    _items(cam, tower_faces(d), lambda p: (ox + p[0] * rgt[0] + p[1] * fwd[0], oy + p[0] * rgt[1] + p[1] * fwd[1], p[2]),
           lambda n: (n[0] * rgt[0] + n[1] * fwd[0], n[0] * rgt[1] + n[1] * fwd[1], n[2]), out)


def _paint(surf, cam, items):
    for _, col, wp, wire in sorted(items, key=lambda t: -t[0]):
        if wire:
            poly(surf, cam, col, wp, 1)
        else:
            poly(surf, cam, col, wp)
            poly(surf, cam, tuple(int(v * 0.6) for v in col), wp, 1)


# ---------------------------------------------------------------- camera views and what the person can change
class Look:
    """The mouse camera and the three overlays. trail / shadow / arrow: a switch and an OPACITY from 0 (invisible) to 1 (solid)."""

    def __init__(self):
        self.reset_cam()
        self.trail_on, self.trail_i = True, 0.9
        self.shadow_on, self.shadow_i = True, 0.35
        self.arrow_on, self.arrow_i = True, 0.8

    def reset_cam(self): self.yaw, self.pitch, self.dist, self.zoom = 0.5, 0.55, 80.0, 1.0

    def drag(self, dx, dy):
        self.yaw -= dx * 0.008
        self.pitch = clamp(self.pitch + dy * 0.008, 0.05, 1.5)

    def wheel(self, n): self.zoom = clamp(self.zoom * (0.88 ** n), 0.15, 5.0)


def _zl(cam, look, eye, target, up=(0, 0, 1)):
    cam.look(tuple(t + (e - t) * look.zoom for e, t in zip(eye, target)), target, up=up)


def set_view(cam, view, x, y, th, look):
    s, c = math.sin(th), math.cos(th)
    fwd, rgt = (-s, c), (c, s)
    zc = 9.0                                                      # the camera looks at the middle of the tower
    if view == 'free':
        cam.fl, d = 400, look.dist
        _zl(cam, look, (x + d * math.cos(look.pitch) * math.sin(look.yaw), y - d * math.cos(look.pitch) * math.cos(look.yaw), zc + d * math.sin(look.pitch)), (x, y, zc))
    elif view in ('front', 'right', 'left', 'back'):
        cam.fl = 400
        v = {'front': fwd, 'back': (-fwd[0], -fwd[1]), 'right': rgt, 'left': (-rgt[0], -rgt[1])}[view]
        _zl(cam, look, (x + v[0] * 50.0, y + v[1] * 50.0, zc + 24), (x, y, zc))
    elif view == 'chase':
        cam.fl = 430
        _zl(cam, look, (x + s * 52, y - c * 52, 34), (x - s * 14, y + c * 14, 3))
    elif view == 'top':
        cam.fl = 480
        _zl(cam, look, (0, 0, 265), (0, 0, 0), up=(0, 1, 0))
    elif view == 'iso':
        cam.fl = 400
        _zl(cam, look, (x + 46, y - 60, zc + 46), (x, y, zc))
    else:                                                          # 'arena'
        cam.fl = 395
        _zl(cam, look, (0, -175, 205), (0, 0, 0))


# ---------------------------------------------------------------- see-through overlays
def _overlays(surf, cam, look, trail, ref_th, ox, oy, shadow_pts):
    """Trail, robot shadow and zero arrow, all flat on the floor, on one transparent layer that is laid over the picture."""
    if not (look.trail_on or look.shadow_on or look.arrow_on):
        return
    layer = pygame.Surface((surf.get_width(), surf.get_height()), pygame.SRCALPHA)
    a255 = lambda k: int(255 * clamp(k, 0.0, 1.0))
    if look.shadow_on and look.shadow_i > 0:
        pts = [cam.cam(p) for p in shadow_pts]
        if all(p[2] > NEAR for p in pts):
            pygame.draw.polygon(layer, (0, 0, 0, a255(look.shadow_i)), [cam.screen(p) for p in pts])
    if look.trail_on and look.trail_i > 0:
        n = max(1, len(trail))
        for i, (tx, ty) in enumerate(trail):
            cc = cam.cam((tx, ty, 0.1))
            if cc[2] > NEAR:
                pygame.draw.circle(layer, (20, 200, 170, a255(look.trail_i * (0.25 + 0.75 * i / n))), cam.screen(cc), 2)
    if look.arrow_on and look.arrow_i > 0:
        col, z = MARK_COL + (a255(look.arrow_i),), 0.2
        zx, zy = -math.sin(ref_th), math.cos(ref_th)
        tip = (ox + zx * 30, oy + zy * 30, z)
        line(layer, cam, col, (ox, oy, z), tip, 2)
        line(layer, cam, col, tip, (ox + zx * 25 - zy * 3, oy + zy * 25 + zx * 3, z), 2)
        line(layer, cam, col, tip, (ox + zx * 25 + zy * 3, oy + zy * 25 - zx * 3, z), 2)
    surf.blit(layer, (0, 0))


# ---------------------------------------------------------------- the two pictures
def draw_scene(surf, cam, robot, view, look):
    """Game: floor, walls, pillars, cubes and the robot, depth sorted together."""
    surf.fill(BACKGROUND)
    d = robot.snapshot()
    set_view(cam, view, robot.x, robot.y, robot.th, look)
    draw_floor(surf, cam)
    _overlays(surf, cam, look, robot.trail, robot.ref_th, robot.x, robot.y, [robot.to_world(x, y) + (0.1,) for x, y in ((-8, -8), (8, -8), (8, 8), (-8, 8))])
    w = robot.world
    for c in w.cubes:                                                    # shadow of a raised cube
        sup = w.support_of(c)[0]
        if c.z > sup + 0.05:
            poly(surf, cam, (18, 20, 24), rect3(c.x - 1.5, c.y - 1.5, c.x + 1.5, c.y + 1.5, sup + 0.07))
    items = []
    for st in w.statics:
        if st.tag == 'wall': _box_world(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, sm.WALL_H, WALL_COL, cam, items, seg=8.0)
        elif st.tag == 'pillar': _box_world(st.cx, st.cy, st.hx, st.hy, 0.0, 0.0, sm.PILLAR_H, PILLAR_COL, cam, items)
    for c in w.cubes:
        _box_world(c.x, c.y, sm.CUBE_S / 2, sm.CUBE_S / 2, c.th, c.z, c.z + sm.CUBE_S, CUBE_COL[c.team], cam, items)
    _robot_items(cam, d, robot.x, robot.y, items)
    _paint(surf, cam, items)
    for c in w.cubes:                                                    # far away a 3 cm cube is a pixel: keep it visible
        cc = cam.cam((c.x, c.y, c.z + sm.CUBE_S / 2))
        if cc[2] > NEAR and cam.fl * sm.CUBE_S / cc[2] < 7:
            sx, sy = cam.screen(cc)
            pygame.draw.rect(surf, CUBE_COL[c.team], (sx - 3, sy - 3, 7, 7))
            pygame.draw.rect(surf, (235, 235, 235), (sx - 3, sy - 3, 7, 7), 1)
        if c.held:
            cc = cam.cam((c.x, c.y, c.z + sm.CUBE_S + 1.2))
            if cc[2] > NEAR:
                pygame.draw.circle(surf, (255, 235, 90), cam.screen(cc), 3)


def draw_monitor(surf, cam, d, view, look):
    """Monitor: the robot stays at the origin and turns in place; the grid under it slides as the robot moves. d is a snapshot() dict."""
    surf.fill(BACKGROUND)
    set_view(cam, view, 0.0, 0.0, d['th'], look)
    R = 64.0
    poly(surf, cam, (30, 34, 40), rect3(-R, -R, R, R, 0.0))
    for g in range(int(math.floor((d['x'] - R) / 10.0)) * 10, int(d['x'] + R) + 10, 10):          # grid lines slide with the robot's x and y
        x = g - d['x']
        if -R <= x <= R:
            line(surf, cam, (92, 102, 114) if g % 50 == 0 else (52, 58, 66), (x, -R, 0.02), (x, R, 0.02), 1)
    for g in range(int(math.floor((d['y'] - R) / 10.0)) * 10, int(d['y'] + R) + 10, 10):
        y = g - d['y']
        if -R <= y <= R:
            line(surf, cam, (92, 102, 114) if g % 50 == 0 else (52, 58, 66), (-R, y, 0.02), (R, y, 0.02), 1)
    for a, b in (((-R, -R), (R, -R)), ((R, -R), (R, R)), ((R, R), (-R, R)), ((-R, R), (-R, -R))):
        line(surf, cam, (96, 106, 118), (a[0], a[1], 0.03), (b[0], b[1], 0.03), 2)
    line(surf, cam, (255, 120, 110), (0, R - 10, 0.05), (0, R, 0.05), 3)                      # north mark: the zero direction of the map
    _overlays(surf, cam, look, (), d['ref_th'], 0.0, 0.0, [(-8, -8, 0.1), (8, -8, 0.1), (8, 8, 0.1), (-8, 8, 0.1)])
    items = []
    _robot_items(cam, d, 0.0, 0.0, items)
    _paint(surf, cam, items)

