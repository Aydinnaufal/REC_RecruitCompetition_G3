"""
test_sim2.py - headless tests for the alternate simulator (sim_physics.py + robot_sim2.py). No window, no controller.

    python test_sim2.py            (needs pygame installed only because robot_sim.py imports it)

Each test drives the simulator with fake ControllerState values and checks numbers. Run the whole file: it prints PASS / FAIL per test.
"""
import math, sys
import xbox_input as xi
import sim_physics as ph
from robot_sim import RobotSim, MAX_SPEED, TRACK, TURN_GAIN
from robot_sim2 import RobotSim2, SPEED_MIN, ACCEL, DECEL, W_MAX

DT = 1 / 60


def S(**k):
    s = xi.ControllerState(connected=True)
    for a, v in k.items():
        if a in s.buttons: s.buttons[a] = v
        else: setattr(s, a, v)
    return s


def run(sim, st, sec, dt=DT):
    for _ in range(int(round(sec / dt))): sim.drive(st, dt)


def fresh():
    sim = RobotSim2()
    run(sim, S(), 1.0)                           # let the cubes drop onto their pillars
    return sim


def at(sim, x, y, heading_left_deg=0.0):
    sim.x, sim.y, sim.th = x, y, math.radians(heading_left_deg)


def dock(sim, cube_id=4, lt=0.0, lateral=0.0, heading_err=0.0):
    """Robot facing the cube's pillar from 20 cm away (lateral cm off to the robot's right, heading_err degrees), drive up and stop
    the moment the nose touches the pillar, the way a driver would."""
    c = sim.world.cubes[cube_id - 1]
    at(sim, c.x + 12.5 + 20.0, c.y + lateral, 90.0 + heading_err)
    sim.claw.cmd = lt
    for _ in range(int(6 / DT)):
        if sim.to_robot(c.x, c.y)[1] <= 12.5 + 0.25: break
        sim.drive(S(ly=0.6, lt=lt), DT)
    run(sim, S(lt=lt), 0.4)


def angdiff(a, b): return abs(ph.wrap_pi(a - b))


TESTS = []
def test(f): TESTS.append(f); return f


# ---------------------------------------------------------------- 1. cubes
@test
def cubes_on_designated_squares_with_gravity():
    w = ph.World()
    assert len(w.cubes) == 12 and all(c.z > ph.PILLAR_H + 5 for c in w.cubes), 'cubes start above their pillars'
    sim = fresh()
    for c in sim.world.cubes:
        assert abs(c.z - ph.PILLAR_H) < 1e-6, ('cube %d rests at z=%.3f' % (c.id, c.z))
        assert math.hypot(c.x - c.home[0], c.y - c.home[1]) < 1e-6, 'cube drifted off its square'
    xs = sorted({round(abs(c.x)) for c in sim.world.cubes}); assert xs == [83, 95], xs
    assert sum(c.team == 'B' for c in sim.world.cubes) == 6


@test
def cube_pushed_off_pillar_falls_and_lands_on_floor():
    sim = fresh()
    c = sim.world.cubes[3]
    c.vx = 60.0                                  # slide it off the pillar towards +x (at z = 5)
    for _ in range(120):
        sim.world.step_cubes(DT)
        if c.z < ph.PILLAR_H - 0.5: break
    assert c.z < ph.PILLAR_H, 'cube did not fall after leaving the pillar'
    for _ in range(120): sim.world.resolve(sim.parts, sim._shift); sim.world.step_cubes(DT)
    assert abs(c.z) < 1e-6 and c.vz == 0.0, 'cube should rest on the floor, z=%.3f' % c.z


@test
def robot_stops_at_pillar_and_wall_and_pushes_floor_cube():
    sim = fresh()
    dock(sim)
    assert abs(sim.x - (-83 + 2 + 10.5)) < 0.3, 'nose should touch the pillar, x=%.2f' % sim.x
    # wall: drive into the boundary
    sim = fresh(); at(sim, 0, 80, 0); run(sim, S(ly=1.0, rt=1.0), 4.0)
    assert sim.y < ph.FIELD, 'robot left the field'
    # a cube on the floor gets pushed
    sim = fresh(); c = sim.world.cubes[0]
    c.x, c.y, c.z, c.vz = 0.0, 20.0, 0.0, 0.0
    at(sim, 0, 0, 0); run(sim, S(ly=0.5), 3.0)
    assert c.y > 20.0 + 5 and abs(c.x) < 3, 'floor cube should be pushed ahead, y=%.1f' % c.y
    assert sim.y > 10, 'robot should have kept moving while pushing'


@test
def cube_pushed_into_wall_stops_robot():
    sim = fresh(); c = sim.world.cubes[0]
    c.x, c.y, c.z, c.vz = 0.0, 90.0, 0.0, 0.0
    at(sim, 0, 60, 0); run(sim, S(ly=1.0), 6.0)
    assert c.y <= 105 - ph.CUBE_S / 2 + 0.05, 'cube went through the wall: y=%.2f' % c.y
    assert sim.y < 105 - 15, 'robot should be held back by the cube: y=%.2f' % sim.y


# ---------------------------------------------------------------- 2. claw
@test
def claw_follows_lt_proportionally():
    sim = fresh(); at(sim, 0, 0, 0)
    got = {}
    for lt in (0.0, 0.25, 0.5, 0.75, 1.0):
        run(sim, S(lt=lt), 1.0)
        got[lt] = sim.claw.ang[-1]
        assert abs(sim.claw.ang[-1] - sim.claw.ang[1]) < 1e-9, 'arms should move together'
        want = ph.OPEN_DEG + lt * (ph.CLOSED_DEG - ph.OPEN_DEG)
        assert abs(sim.claw.ang[-1] - want) < 1e-6, (lt, sim.claw.ang[-1], want)
    assert got[0.0] == ph.OPEN_DEG and got[1.0] == ph.CLOSED_DEG
    # rate limited (a servo takes time), and smooth: no jump larger than the rate allows
    sim = fresh(); prev = sim.claw.ang[-1]; worst = 0
    for _ in range(40):
        sim.drive(S(lt=1.0), DT); worst = max(worst, abs(sim.claw.ang[-1] - prev)); prev = sim.claw.ang[-1]
    assert worst <= ph.ARM_RATE * DT + 1e-6


@test
def grab_needs_both_arms_and_holds_the_cube():
    sim = fresh(); dock(sim)
    run(sim, S(lt=0.5), 1.0)
    assert sim.claw.held is None, 'half closed claw must not hold'
    run(sim, S(lt=1.0), 1.0)
    c = sim.claw.held
    assert c is not None and c.id == 4 and c.held, 'both arms touching -> grab'
    # the cube moves with the robot: back off 20 cm
    x0, y0 = c.x, c.y
    run(sim, S(ly=-0.6, lt=1.0), 1.5)
    assert c.held and math.hypot(c.x - x0, c.y - y0) > 15, 'held cube should travel with the robot'
    rel = sim.to_robot(c.x, c.y)
    assert abs(rel[0] - sim.claw.rel[0]) < 1e-6 and abs(rel[1] - sim.claw.rel[1]) < 1e-6, 'held cube must keep its place in the claw'


@test
def grab_tolerance_and_one_arm_pushing():
    # lateral and heading errors a driver can reasonably keep: still a grab
    for lat, hd in ((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (0.0, 2.0), (0.0, -2.0), (1.0, 1.5)):
        sim = fresh(); dock(sim, lateral=lat, heading_err=hd); run(sim, S(lt=1.0), 1.5)
        assert sim.claw.held is not None and sim.claw.held.id == 4, ('should grab', lat, hd)
    # cube far to one side: one arm touches it, the other cannot: it is pushed, never held
    sim = fresh(); c = sim.world.cubes[3]
    at(sim, c.x + 12.5, c.y + 6.0, 90.0)                      # cube is 6 cm to the robot's left side, beyond the right arm's reach
    run(sim, S(lt=0.0), 0.3); home = (c.x, c.y)
    run(sim, S(lt=1.0), 1.5)
    assert sim.claw.held is None, 'a cube touched by one arm must not be held'
    assert math.hypot(c.x - home[0], c.y - home[1]) > 0.5, 'it should have been pushed by the arm'
    # both arms only reach half way: never a grab
    sim = fresh(); dock(sim); run(sim, S(lt=0.5), 1.5)
    assert sim.claw.held is None


@test
def release_drops_cube_with_gravity_and_pushes_nothing_else():
    sim = fresh(); dock(sim); run(sim, S(lt=1.0), 1.5)
    c = sim.claw.held; assert c is not None
    run(sim, S(ly=-0.8, lt=1.0), 1.2)                      # carry it off the pillar zone onto open floor
    assert c.z == ph.PILLAR_H or abs(c.z - ph.PILLAR_H) < 1e-6, 'held cube keeps its height'
    run(sim, S(lt=0.0), 0.3)                               # open the claw
    assert sim.claw.held is None and not c.held, 'cube released'
    run(sim, S(lt=0.0), 1.5)
    assert abs(c.z) < 1e-6 and c.vz == 0.0, 'cube must fall to the floor: z=%.3f' % c.z
    assert math.hypot(c.vx, c.vy) < 0.1, 'cube should have stopped'


@test
def place_cube_back_on_a_pillar():
    sim = fresh(); dock(sim); run(sim, S(lt=1.0), 1.5)
    c = sim.claw.held; assert c is not None
    # sideways to the next pillar row is blocked by walls of pillars; reverse out, drive in again and release over the same pillar
    run(sim, S(ly=-0.6, lt=1.0), 0.6)
    run(sim, S(ly=0.6, lt=1.0), 2.5)
    run(sim, S(lt=0.0), 1.5)
    assert sim.claw.held is None
    assert abs(c.z - ph.PILLAR_H) < 1e-6, 'cube released over the pillar should rest on it, z=%.3f' % c.z
    assert math.hypot(c.x - c.home[0], c.y - c.home[1]) < 3.0


# ---------------------------------------------------------------- 3. variable speed
@test
def rt_maps_to_speed_with_smooth_acceleration():
    for rt, want in ((0.0, SPEED_MIN * MAX_SPEED), (1.0, MAX_SPEED), (0.5, (SPEED_MIN + (1 - SPEED_MIN) / 2) * MAX_SPEED)):
        sim = fresh(); at(sim, 0, -20, 0)
        run(sim, S(ly=1.0, rt=rt), 0.9)
        assert abs(sim.speed - want) < 0.05, (rt, sim.speed, want)
    assert SPEED_MIN * MAX_SPEED < MAX_SPEED
    sim = fresh(); at(sim, 0, -90, 0)
    prev, acc, speeds = 0.0, [], []
    for _ in range(90):
        sim.drive(S(ly=1.0, rt=1.0), DT); acc.append((sim.speed - prev) / DT); prev = sim.speed; speeds.append(sim.speed)
    assert max(acc) <= ACCEL + 1e-6, 'acceleration limit %.1f' % max(acc)
    assert all(b >= a - 1e-9 for a, b in zip(speeds, speeds[1:])), 'speed should rise monotonically'
    assert speeds[3] < 0.5 * MAX_SPEED, 'no instant jump to full speed'
    dec = []
    for _ in range(90):
        sim.drive(S(ly=0.0, rt=1.0), DT); dec.append((sim.speed - prev) / DT); prev = sim.speed
    assert min(dec) >= -DECEL - 1e-6 and sim.speed < 0.01, 'braking limit / stops'
    # changing RT while driving changes speed smoothly (no jump)
    sim = fresh(); at(sim, 0, -80, 0); run(sim, S(ly=1.0, rt=0.0), 1.0)
    before = sim.speed; sim.drive(S(ly=1.0, rt=1.0), DT)
    assert abs(sim.speed - before) <= ACCEL * DT + 1e-6, 'RT step must not jerk the wheels'


# ---------------------------------------------------------------- 4. arcade + snap
@test
def arcade_mode_keeps_its_old_behaviour():
    a, b = RobotSim(), RobotSim2(); b.mode = 'arcade'
    run(b, S(), 1.0)
    st = S(ly=0.6, lx=0.4, rt=1.0)
    for _ in range(180): a.drive(st, DT); b.drive(st, DT)
    assert abs(a.vl - b.vl) < 1e-3 and abs(a.vr - b.vr) < 1e-3, (a.vl, b.vl, a.vr, b.vr)
    assert abs(a.speed - b.speed) < 0.05
    ta = [0.6 + TURN_GAIN * 0.4, 0.6 - TURN_GAIN * 0.4]
    assert abs(b.vl - ta[0]) < 1e-3 and abs(b.vr - ta[1]) < 1e-3, 'same stick mixing as RobotSim'
    # tank mode unchanged
    b2 = RobotSim2(); b2.mode = 'tank'; run(b2, S(ly=0.5, ry=-0.5, rt=1.0), 1.5)
    assert abs(b2.vl - 0.5) < 1e-3 and abs(b2.vr + 0.5) < 1e-3
    # snap buttons do nothing outside arcade_snap
    b3 = RobotSim2(); b3.mode = 'arcade'; run(b3, S(LB=True), 0.5); assert abs(b3.th) < 1e-12


def snap(sim, st_press, release_after=0.05, total=2.0):
    """Press for release_after seconds, then let go; returns the final theta and the list of thetas, positions."""
    ths, pos = [], []
    for i in range(int(total / DT)):
        st = st_press if i * DT < release_after else S()
        sim.drive(st, DT); ths.append(sim.th); pos.append((sim.x, sim.y))
    return ths, pos


@test
def snap_turn_90_left_and_right_exact():
    for key, sgn in (('LB', 1), ('RB', -1)):
        sim = fresh(); sim.mode = 'arcade_snap'; at(sim, -5, 7, 20.0)
        th0 = sim.th
        ths, pos = snap(sim, S(**{key: True}))
        want = th0 + sgn * math.pi / 2
        assert abs(sim.th - want) < 1e-12, (key, sim.th - want)
        assert not sim.snapping and sim.snap_w == 0.0 and abs(sim.vl) < 1e-12 and abs(sim.vr) < 1e-12
        assert all(sgn * (t - want) <= 1e-12 for t in ths), 'overshoot'
        assert all(abs(p[0] + 5) < 1e-9 and abs(p[1] - 7) < 1e-9 for p in pos), 'drift while turning on the spot'
        assert all(b * sgn >= a * sgn - 1e-12 for a, b in zip(ths, ths[1:])), 'heading must move one way only'
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    ths, _ = snap(sim, S(LB=True))
    n = next(i for i, t in enumerate(ths) if abs(t - math.pi / 2) < 1e-12)
    assert 0.3 < n * DT < 1.2, 'a 90 degree snap should take about half a second: %.2f s' % (n * DT)
    w_peak = max((b - a) / DT for a, b in zip(ths, ths[1:]))
    assert abs(w_peak - W_MAX) < 0.05, 'should reach the maximum rotation speed: %.2f vs %.2f' % (w_peak, W_MAX)


@test
def snap_triggers_once_per_press():
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    run(sim, S(LB=True), 2.0)                              # button held for two seconds
    assert abs(sim.th - math.pi / 2) < 1e-12, 'holding LB must give one turn only'
    run(sim, S(), 0.1); run(sim, S(LB=True), 0.05); run(sim, S(), 1.5)
    assert abs(sim.th - math.pi) < 1e-12, 'a second press gives a second turn'
    # presses during a turn are ignored
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    sim.drive(S(LB=True), DT); sim.drive(S(), DT); sim.drive(S(LB=True), DT); sim.drive(S(), DT)
    run(sim, S(), 2.0); assert abs(sim.th - math.pi / 2) < 1e-12


@test
def face_absolute_directions_by_the_shortest_turn():
    want = {'Y': 0.0, 'B': -math.pi / 2, 'A': math.pi, 'X': math.pi / 2}
    for start in (0, 30, 90, 135, 180, 269, 270, 355, -80):
        for key, tgt in want.items():
            sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 10, -10, start)
            th0 = sim.th
            err = ph.wrap_pi(tgt - th0)
            travelled, prev = 0.0, sim.th
            for i in range(int(2.5 / DT)):
                sim.drive(S(**{key: True}) if i < 3 else S(), DT)
                travelled += abs(sim.th - prev); prev = sim.th
            assert angdiff(sim.th, tgt) < 1e-9, (start, key, math.degrees(sim.th))
            assert abs(travelled - abs(err)) < 1e-9, ('unnecessary rotation', start, key, travelled, err)
            assert abs(sim.x - 10) < 1e-9 and abs(sim.y + 10) < 1e-9, 'drift'
    # already aligned: no motion at all
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    sim.drive(S(Y=True), DT); assert not sim.snapping and sim.th == 0.0
    # the reference is the START orientation: it does not follow the robot
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    snap(sim, S(LB=True)); snap(sim, S(LB=True))            # now facing down (180)
    snap(sim, S(Y=True)); assert angdiff(sim.th, 0.0) < 1e-9, 'Y = absolute up even after turning'


@test
def modes_cycle_and_snap_ignores_stick_until_done():
    sim = fresh(); sim.mode = 'arcade_snap'; at(sim, 0, 0, 0)
    sim.drive(S(LB=True), DT)
    for _ in range(10): sim.drive(S(ly=1.0), DT)
    assert abs(sim.x) < 1e-9 and abs(sim.y) < 1e-9, 'no driving during a snap'
    run(sim, S(ly=1.0), 1.0); assert sim.speed > 1.0, 'stick works again after the snap'
    sim.mode = 'arcade'; sim.drive(S(), DT)


# ---------------------------------------------------------------- 5. together
@test
def together_pick_turn_place():
    sim = fresh(); sim.mode = 'arcade_snap'
    dock(sim); run(sim, S(lt=1.0), 1.5)
    c = sim.claw.held; assert c is not None
    run(sim, S(ly=-0.6, lt=1.0, rt=0.3), 1.5)               # back away at reduced speed
    sim.drive(S(RB=True, lt=1.0), DT); run(sim, S(lt=1.0), 1.5)  # snap 90 degrees with the cube in the claw
    assert c.held and abs(ph.wrap_pi(sim.th - (math.pi / 2 - math.pi / 2))) < 1e-9
    rel = sim.to_robot(c.x, c.y)
    assert abs(rel[0] - sim.claw.rel[0]) < 1e-6 and abs(rel[1] - sim.claw.rel[1]) < 1e-6, 'cube stays in the claw through the turn'
    sim.drive(S(X=True, lt=1.0), DT); run(sim, S(lt=1.0), 2.0)
    assert angdiff(sim.th, math.pi / 2) < 1e-9 and c.held
    run(sim, S(lt=0.0), 2.0)
    assert sim.claw.held is None and abs(c.z) < 1e-6, 'released and landed on the floor'
    # nothing exploded: all numbers finite, no cube inside a wall or the robot
    for k in sim.world.cubes:
        assert all(math.isfinite(v) for v in (k.x, k.y, k.z, k.vx, k.vy, k.vz)), 'NaN'
        assert abs(k.x) < 105 and abs(k.y) < 105
    assert all(math.isfinite(v) for v in (sim.x, sim.y, sim.th))


@test
def long_random_drive_stays_sane():
    import random
    rnd = random.Random(3)
    sim = fresh(); sim.mode = 'arcade_snap'
    for i in range(60 * 90):
        if i % 20 == 0:
            st = S(lx=rnd.uniform(-1, 1), ly=rnd.uniform(-1, 1), lt=rnd.random() * rnd.choice((0, 1)), rt=rnd.random(),
                   LB=rnd.random() < 0.05, RB=rnd.random() < 0.05, Y=rnd.random() < 0.03, B=rnd.random() < 0.03,
                   A=rnd.random() < 0.03, X=rnd.random() < 0.03)
        sim.drive(st, DT)
        assert all(math.isfinite(v) for v in (sim.x, sim.y, sim.th)), 'NaN at step %d' % i
        assert abs(sim.x) < ph.FIELD + 1 and abs(sim.y) < ph.FIELD + 1, 'robot left the field at step %d' % i
        for k in sim.world.cubes:
            assert abs(k.x) < ph.FIELD + 1 and abs(k.y) < ph.FIELD + 1 and k.z > -0.01 and math.isfinite(k.z), ('cube %d out' % k.id, i)


@test
def reset_restores_everything():
    sim = fresh(); dock(sim); run(sim, S(lt=1.0), 1.5); sim.reset()
    assert sim.claw.held is None and sim.x == 0 and sim.th == 0
    assert all(c.z > ph.PILLAR_H for c in sim.world.cubes) and not any(c.held for c in sim.world.cubes)


if __name__ == '__main__':
    bad = 0
    for t in TESTS:
        try:
            t(); print('PASS', t.__name__)
        except Exception as e:
            bad += 1; print('FAIL', t.__name__, '-', type(e).__name__, e)
    print('%d of %d passed' % (len(TESTS) - bad, len(TESTS)))
    sys.exit(1 if bad else 0)
