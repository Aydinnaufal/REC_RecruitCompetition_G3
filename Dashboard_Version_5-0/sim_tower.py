"""
sim_tower.py - the tower robot's intake: a storage box on a rack and pinion, with one mini gripper pad (no pygame, testable anywhere).

Robot frame as in sim_physics: x to the robot's right, y forward, origin at the middle of the wheel axle. The box sits at the origin,
in a hole through the chassis, and slides up and down. Units: cm.

  box        4 thin walls around a square cavity (BOX_IN across, a bit bigger than the 30 mm cube), open at the top and at the bottom.
             Bottom edge at height `lift` (LIFT_MIN .. LIFT_MAX). The right stick Y moves it (how far you push = how fast).
  pad        a servo pad on the +x inner wall, pushes sideways. right stick X 0..1 = pad out 0 .. PAD_MAX.
             A cube inside the box that the pad presses against the far wall is HELD: it moves up and down and drives with the box.
  stacking   a cube released over another cube lands on it (World(stacking=True)); a cube resting on the held one is carried with it.
"""
import math
import sim_physics as ph
from sim_physics import OBB, clamp, CUBE_S

BOX_IN = 3.8                        # cavity, 35 to 40 mm (a 30 mm cube leaves 4 mm each side)
BOX_WALL = 0.25
BOX_OUT = BOX_IN + 2 * BOX_WALL
BOX_H = 8.0                         # two cubes (6 cm) and some room
LIFT_MIN, LIFT_MAX = 0.6, 12.0      # box bottom; 12 + 8 = 20 cm: the top of the robot never passes the 20 cm limit
LIFT_START = 7.0                    # starts raised: the robot can drive over cubes (box bottom above the 3.5 cm cube-on-pillar top)
LIFT_RATE = 7.0                     # cm/s, rack and pinion servo
PAD_MAX, PAD_RATE = 3.0, 14.0       # pad travel (cm) and speed (cm/s)
PAD_Y_HALF = 1.0
PAD_Z0, PAD_Z1 = 0.3, 2.7           # pad height range, measured from the bottom edge of the box
RELEASE_EPS = 0.10                  # pad backs off this much from a held cube = released
FAR_X = -BOX_IN / 2 + CUBE_S / 2    # x of a cube pressed against the far (-x) wall: -0.4


class Tower:
    def __init__(self):
        self.pad_cmd = 0.0
        self.lift_cmd = 0.0
        self.reset()

    def reset(self):
        self.lift = LIFT_START
        self.ext = 0.0                      # pad travel from the +x wall, cm
        self.held = None
        self.rel = (0.0, 0.0, 0.0, 0.0)     # held cube in the robot frame: x, y, height above the box bottom, yaw
        self.contact = False

    # ---- geometry
    @property
    def pad_face(self):
        return BOX_IN / 2 - self.ext

    def wall_boxes(self):
        """The four box walls as (cx, cy, hx, hy) in the robot frame."""
        o = BOX_IN / 2 + BOX_WALL / 2
        return [(0.0, o, BOX_OUT / 2, BOX_WALL / 2), (0.0, -o, BOX_OUT / 2, BOX_WALL / 2),
                (o, 0.0, BOX_WALL / 2, BOX_IN / 2), (-o, 0.0, BOX_WALL / 2, BOX_IN / 2)]

    def parts(self, rob):
        return [OBB(*rob.to_world(cx, cy), hx, hy, rob.th, self.lift, self.lift + BOX_H, 'boxwall') for cx, cy, hx, hy in self.wall_boxes()]

    # ---- input and motion
    def set_cmd(self, pad, lift):
        self.pad_cmd = clamp(pad, 0.0, 1.0)                # pad: 0 open .. 1 pushing
        self.lift_cmd = clamp(lift, -1.0, 1.0)             # box: +1 up, -1 down, in between = slower

    def move(self, rob, dt):
        self.lift = clamp(self.lift + self.lift_cmd * LIFT_RATE * dt, LIFT_MIN, LIFT_MAX)
        tgt = self.pad_cmd * PAD_MAX
        new = self.ext + clamp(tgt - self.ext, -PAD_RATE * dt, PAD_RATE * dt)
        if self.held is not None and new > self.ext:
            new = min(new, BOX_IN / 2 - (self.rel[0] + CUBE_S / 2))        # a held cube stops the pad
        self.ext = clamp(new, 0.0, PAD_MAX)

    def sync_held(self, rob):
        if self.held is not None:
            c = self.held
            c.x, c.y = rob.to_world(self.rel[0], self.rel[1])
            c.th = rob.th + self.rel[3]
            c.z = self.lift + self.rel[2]

    # ---- grip
    def update_grip(self, rob, world, dt):
        if self.held is not None:
            if self.pad_face > self.rel[0] + CUBE_S / 2 + RELEASE_EPS:
                self.release(rob)
            return
        self.contact = False
        for c in world.free_cubes():
            rx, ry = rob.to_robot(c.x, c.y)
            if abs(rx) > BOX_IN / 2 + 0.2 or abs(ry) > BOX_IN / 2 + 0.2:
                continue                                    # the cube is not in the box
            zrel = c.z - self.lift
            if not (zrel < PAD_Z1 and zrel + CUBE_S > PAD_Z0):
                continue                                    # the pad does not reach this height
            if self.pad_face > rx + CUBE_S / 2 + 0.02:
                continue                                    # the pad has not reached the cube yet
            self.contact = True
            nrx = min(rx, self.pad_face - CUBE_S / 2)       # the pad pushes the cube toward the far wall
            if nrx <= FAR_X + 0.02:                         # squeezed against the far wall: held
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
