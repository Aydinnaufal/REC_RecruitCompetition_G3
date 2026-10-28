"""
sim_physics.py - small 2.5D physics for the alternate robot simulator (no pygame, so it can be tested anywhere).

Units: cm, seconds. World as in robot_sim.py: x right, y up the map, z up; heading th = 0 faces +y and grows counter-clockwise.
Robot frame: x to the robot's right, y forward. A point (x, y) in robot frame is world pos + x*right + y*fwd, with
right = (cos th, sin th) and fwd = (-sin th, cos th).

Bodies are oriented boxes (OBB) in the xy plane with a z range. Two bodies only collide when their z ranges overlap, which is
what lets the claw arms (z 5.3 .. 7.7) pass over a pillar (z 0 .. 5) and still grab the cube standing on it.

Contents
    OBB, sat()        overlap test between two boxes (separating axis theorem)
    Cube              a 3 cm cube with gravity, friction and push physics
    World             pillars, walls, cubes; step() moves the cubes and resolves every collision
    Claw              two arms on hinges; grab = both arms touch the same cube, release = an arm lets go
"""
import math

G = 981.0                  # gravity, cm/s^2
CUBE_S = 3.0               # cube side (handbook: 30 mm)
PILLAR_W, PILLAR_H = 4.0, 5.0
WALL_T, WALL_H = 1.2, 10.0 # assumed, same as the arena STEP file (12 mm, 100 mm)
FIELD = 105.0
FRICTION = 0.35            # sliding friction coefficient of a cube on the floor / pillar top
RESTITUTION = 0.25         # vertical bounce of a falling cube
Z_EPS = 0.01
PILLAR_XS, PILLAR_YS = (-95.0, -83.0), (-13.0, 0.0, 13.0)     # the designated squares (blue side; red side is the mirror image)


def clamp(v, lo, hi): return max(lo, min(hi, v))


def wrap_pi(a):
    """Angle in (-pi, pi]."""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return math.pi if a <= -math.pi + 1e-12 else a


class OBB:
    """Oriented box: centre (cx, cy), half sizes hx along u and hy along v, u = (cos a, sin a), v = (-sin a, cos a), z range z0..z1."""
    __slots__ = ('cx', 'cy', 'hx', 'hy', 'ux', 'uy', 'z0', 'z1', 'ex', 'ey', 'tag')

    def __init__(self, cx, cy, hx, hy, a, z0, z1, tag=None):
        self.cx, self.cy, self.hx, self.hy = cx, cy, hx, hy
        self.ux, self.uy = math.cos(a), math.sin(a)
        self.z0, self.z1, self.tag = z0, z1, tag
        self.ex = abs(self.ux) * hx + abs(self.uy) * hy          # half size of the bounding box (axis aligned)
        self.ey = abs(self.uy) * hx + abs(self.ux) * hy


def z_overlap(a, b):
    return a.z0 < b.z1 - Z_EPS and b.z0 < a.z1 - Z_EPS


def aabb_apart(a, b, margin=0.0):
    return abs(a.cx - b.cx) > a.ex + b.ex + margin or abs(a.cy - b.cy) > a.ey + b.ey + margin


def sat(a, b, margin=0.0):
    """Separating axis test. Returns (depth, nx, ny) or None. n points from a to b. depth > 0: overlapping by that much;
    depth < 0 (only when margin > 0): a gap of that size along the best axis (a lower bound of the true distance)."""
    dx, dy = b.cx - a.cx, b.cy - a.cy
    best = None
    for ax, ay in ((a.ux, a.uy), (-a.uy, a.ux), (b.ux, b.uy), (-b.uy, b.ux)):
        ra = a.hx * abs(a.ux * ax + a.uy * ay) + a.hy * abs(-a.uy * ax + a.ux * ay)
        rb = b.hx * abs(b.ux * ax + b.uy * ay) + b.hy * abs(-b.uy * ax + b.ux * ay)
        d = dx * ax + dy * ay
        ov = ra + rb - abs(d)
        if ov < -margin:
            return None
        if best is None or ov < best[0]:
            best = (ov, ax if d >= 0 else -ax, ay if d >= 0 else -ay)
    return best


# ---------------------------------------------------------------- cubes
class Cube:
    def __init__(self, cid, team, x, y, z=0.0):
        self.id, self.team = cid, team
        self.x, self.y, self.z, self.th = x, y, z, 0.0
        self.vx = self.vy = self.vz = 0.0
        self.held = False
        self.pinned = False            # touching something that cannot move this step
        self.dx = self.dy = 0.0        # how far it was pushed this step (becomes its velocity)
        self.home = (x, y)

    def obb(self):
        return OBB(self.x, self.y, CUBE_S / 2, CUBE_S / 2, self.th, self.z, self.z + CUBE_S, self)


class World:
    def __init__(self, drop_height=10.0):
        self.statics = []
        self.pillars = []
        self.cubes = []
        self.drop_height = drop_height
        self._build()
        self.reset_cubes()

    # ---- layout
    def _build(self):
        S = self.statics
        def wall(x0, y0, x1, y1):            # axis-aligned wall segment, WALL_T thick
            hx = abs(x1 - x0) / 2 + WALL_T / 2 if y0 == y1 else WALL_T / 2
            hy = abs(y1 - y0) / 2 + WALL_T / 2 if x0 == x1 else WALL_T / 2
            S.append(OBB((x0 + x1) / 2, (y0 + y1) / 2, hx, hy, 0.0, 0.0, WALL_H, 'wall'))
        # inner walls: same list as robot_sim.WALLS (kept here so this module has no pygame import)
        for a, b in (((-35, -105), (-35, -12.5)), ((-35, 12.5), (-35, 57.5)), ((-35, 82.5), (-35, 105)),
                     ((35, -105), (35, -82.5)), ((35, -57.5), (35, -12.5)), ((35, 12.5), (35, 105)),
                     ((-35, 35), (35, 35)), ((-35, -35), (35, -35)), ((-35, 105), (35, 105)), ((-35, -105), (35, -105))):
            wall(a[0], a[1], b[0], b[1])
        big = FIELD + 20
        for cx, cy, hx, hy in ((-FIELD - 5, 0, 5, big), (FIELD + 5, 0, 5, big), (0, -FIELD - 5, big, 5), (0, FIELD + 5, big, 5)):
            S.append(OBB(cx, cy, hx, hy, 0.0, 0.0, WALL_H, 'boundary'))
        for sgn in (1, -1):
            for px in PILLAR_XS:
                for py in PILLAR_YS:
                    p = OBB(sgn * px, sgn * py, PILLAR_W / 2, PILLAR_W / 2, 0.0, 0.0, PILLAR_H, 'pillar')
                    S.append(p); self.pillars.append(p)

    def support_height(self, x, y):
        """Height of whatever a cube centred at (x, y) rests on: a pillar top if the centre is over a pillar, else the floor."""
        for p in self.pillars:
            if abs(x - p.cx) <= p.hx and abs(y - p.cy) <= p.hy:
                return PILLAR_H
        return 0.0

    def reset_cubes(self):
        self.cubes = []
        n = 0
        for sgn, team in ((1, 'B'), (-1, 'R')):
            for py in PILLAR_YS:
                for px in PILLAR_XS:
                    n += 1
                    self.cubes.append(Cube(n, team, sgn * px, sgn * py, PILLAR_H + self.drop_height))
        for c in self.cubes:
            c.home = (c.x, c.y)

    def free_cubes(self):
        return [c for c in self.cubes if not c.held]

    # ---- one physics step
    def step_cubes(self, dt):
        """Gravity, friction and landing for every free cube. Call after resolve(). A cube that was pushed this step already moved
        (resolve moved it), so its velocity is taken from the push; any other cube moves by its own velocity."""
        for c in self.free_cubes():
            pushed = bool(c.dx or c.dy)
            if pushed:
                vx, vy = c.dx / dt, c.dy / dt
                sp = math.hypot(vx, vy)
                if sp > 90.0: vx, vy = vx * 90.0 / sp, vy * 90.0 / sp
                c.vx, c.vy = vx, vy
            c.dx = c.dy = 0.0
            sup = self.support_height(c.x, c.y)
            if c.z > sup + 1e-6 or c.vz != 0.0:                # in the air: falling
                c.vz -= G * dt
                c.z += c.vz * dt
                if c.z <= sup:
                    c.z = sup
                    c.vz = -c.vz * RESTITUTION if c.vz < -25.0 else 0.0
            else:
                c.z = sup
                sp = math.hypot(c.vx, c.vy)                    # friction acts only while it rests on something
                if sp > 0 and not pushed:
                    k = max(0.0, sp - FRICTION * G * dt) / sp
                    c.vx *= k; c.vy *= k
                    if sp * k < 0.05: c.vx = c.vy = 0.0
            if not pushed:
                c.x += c.vx * dt
                c.y += c.vy * dt

    def resolve(self, robot_parts, shift_robot, iters=5):
        """Resolve overlaps. robot_parts() returns the robot's current OBBs (so it can be called again after a shift);
        shift_robot(dx, dy) moves the robot. Cubes are pushed by the robot, walls and pillars stop both."""
        free = self.free_cubes()
        for c in free: c.pinned = False
        for _ in range(iters):
            moved = False
            # 1. cubes against walls and pillars
            for c in free:
                ob = c.obb()
                for s in self.statics:
                    if aabb_apart(ob, s) or not z_overlap(ob, s): continue
                    r = sat(ob, s)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        c.x -= nx * d; c.y -= ny * d; c.dx -= nx * d; c.dy -= ny * d
                        c.pinned = True; moved = True
                        ob = c.obb()
            # 2. robot against walls and pillars
            for part in robot_parts():
                for s in self.statics:
                    if aabb_apart(part, s) or not z_overlap(part, s): continue
                    r = sat(part, s)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        shift_robot(-nx * d, -ny * d); moved = True
                        break
            # 3. robot against cubes: the cube moves, unless it is pinned (then the robot gives way)
            for part in robot_parts():
                for c in free:
                    ob = c.obb()
                    if aabb_apart(part, ob) or not z_overlap(part, ob): continue
                    r = sat(part, ob)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        if c.pinned:
                            shift_robot(-nx * d, -ny * d)
                        else:
                            c.x += nx * d; c.y += ny * d; c.dx += nx * d; c.dy += ny * d
                        moved = True
            # 4. cubes against cubes
            for i in range(len(free)):
                a = free[i]
                for j in range(i + 1, len(free)):
                    b = free[j]
                    oa, ob = a.obb(), b.obb()
                    if aabb_apart(oa, ob) or not z_overlap(oa, ob): continue
                    r = sat(oa, ob)
                    if r and r[0] > 1e-6:
                        d, nx, ny = r
                        wa = 0.0 if b.pinned and not a.pinned else 1.0     # a pinned cube does not move, the other one takes it all
                        wb = 0.0 if a.pinned and not b.pinned else 1.0
                        tot = wa + wb
                        if tot == 0: continue
                        a.x -= nx * d * wa / tot; a.y -= ny * d * wa / tot; a.dx -= nx * d * wa / tot; a.dy -= ny * d * wa / tot
                        b.x += nx * d * wb / tot; b.y += ny * d * wb / tot; b.dx += nx * d * wb / tot; b.dy += ny * d * wb / tot
                        moved = True
            if not moved: break


# ---------------------------------------------------------------- claw
ARM_HINGE = 1.9        # arm hinge x (robot frame), each side: closed on a 3 cm cube the arms are parallel, 1.9 +- 0.4 from the centre line
ARM_HINGE_Y = 8.5      # arm hinge y
ARM_LEN, ARM_T = 8.5, 0.8
ARM_Z0, ARM_Z1 = 5.3, 7.7        # just above the chassis and the pillar tops, level with a cube standing on a pillar
OPEN_DEG, CLOSED_DEG = -40.0, 6.0    # arm angle from straight ahead, positive = swung inwards (0 = parallel, 6 = tips almost touching)
ARM_RATE = 200.0       # servo speed, degrees per second
CONTACT_EPS = 0.08     # arm counts as touching a cube within this gap (cm)
RELEASE_EPS = 0.30     # a held cube is let go when an arm is further than this
HOLD_TIME = 0.0        # both arms touching a cube (while the claw is closing) counts as a grab at once; a delay lets the arms squeeze the cube out


class Claw:
    """Two arms. angle[side] in degrees from straight ahead, positive = swung inwards (toward the centre line)."""

    def __init__(self):
        self.cmd = 0.0                       # LT, 0 open .. 1 closed
        self.reset()

    def reset(self):
        self.ang = {-1: OPEN_DEG, 1: OPEN_DEG}
        self.held = None                     # the Cube being held
        self.rel = (0.0, 0.0, 0.0)           # held cube pose in the robot frame (x, y, yaw)
        self.timer = 0.0
        self.contact = {-1: False, 1: False}

    @staticmethod
    def target_deg(lt):
        return OPEN_DEG + clamp(lt, 0.0, 1.0) * (CLOSED_DEG - OPEN_DEG)

    @staticmethod
    def arm_local(side, deg):
        """Arm box in the robot frame: (cx, cy, hx, hy, angle). side -1 = left, +1 = right."""
        b = math.radians(deg)
        hx = side * ARM_HINGE
        # left arm points forward and swings toward +x; right arm toward -x
        ang = math.pi / 2 - b if side < 0 else math.pi / 2 + b
        cx, cy = hx + math.cos(ang) * ARM_LEN / 2, ARM_HINGE_Y + math.sin(ang) * ARM_LEN / 2
        return cx, cy, ARM_LEN / 2, ARM_T / 2, ang

    def arm_obb(self, rob, side, deg=None):
        cx, cy, hx, hy, a = self.arm_local(side, deg if deg is not None else self.ang[side])
        return OBB(*rob.to_world(cx, cy), hx, hy, a + rob.th, ARM_Z0, ARM_Z1, ('arm', side))

    # ---- motion
    def move_arms(self, rob, dt):
        tgt = self.target_deg(self.cmd)
        for side in (-1, 1):
            cur = self.ang[side]
            new = cur + clamp(tgt - cur, -ARM_RATE * dt, ARM_RATE * dt)
            if self.held is not None and new > cur:
                # squeezing a held cube: the arm stops where it touches it
                ob = self.held.obb()
                stepn = max(1, int(abs(new - cur) / 0.25))
                ok = cur
                for k in range(1, stepn + 1):
                    d = cur + (new - cur) * k / stepn
                    r = sat(self.arm_obb(rob, side, d), ob)
                    if r and r[0] > 1e-6: break
                    ok = d
                new = ok
            self.ang[side] = new

    def update_grip(self, rob, world, dt):
        """Decide grab and release. Returns nothing; sets self.held and the cube's held flag."""
        if self.held is not None:
            c = self.held
            ob = c.obb()
            touching = []
            for side in (-1, 1):
                r = sat(self.arm_obb(rob, side), ob, RELEASE_EPS)
                touching.append(r is not None)
            self.contact = {-1: touching[0], 1: touching[1]}
            if not (touching[0] and touching[1]):
                self.release(rob)
            return
        best = None
        for c in world.free_cubes():
            ob = c.obb()
            sides = {}
            for side in (-1, 1):
                arm = self.arm_obb(rob, side)
                if aabb_apart(arm, ob, CONTACT_EPS) or not z_overlap(arm, ob):
                    sides[side] = False; continue
                r = sat(arm, ob, CONTACT_EPS)
                if r is None:
                    sides[side] = False; continue
                nx, ny = r[1], r[2]
                lateral = nx * math.cos(rob.th) + ny * math.sin(rob.th)         # arm -> cube direction along the robot's right
                sides[side] = (lateral > 0.2) if side < 0 else (lateral < -0.2)  # left arm must press toward +x, right arm toward -x
            if sides[-1] and sides[1]:
                best = c; self.contact = dict(sides); break
            if best is None: self.contact = dict(sides)
        closing = self.target_deg(self.cmd) >= max(self.ang.values()) - 0.5
        if best is not None and closing:
            self.timer += dt
            if self.timer >= HOLD_TIME:
                self.grab(rob, best)
        else:
            self.timer = 0.0

    def grab(self, rob, cube):
        self.held = cube; cube.held = True
        cube.vx = cube.vy = cube.vz = 0.0
        x, y = rob.to_robot(cube.x, cube.y)
        self.rel = (x, y, cube.th - rob.th)
        self.timer = 0.0

    def release(self, rob):
        c = self.held
        if c is None: return
        c.held = False
        vx, vy = rob.point_velocity(c.x, c.y)
        c.vx, c.vy, c.vz = vx, vy, 0.0
        self.held = None
        self.timer = 0.0

    def sync_held(self, rob):
        """Put the held cube where the claw holds it, after the robot moved."""
        if self.held is not None:
            c = self.held
            c.x, c.y = rob.to_world(self.rel[0], self.rel[1])
            c.th = rob.th + self.rel[2]
