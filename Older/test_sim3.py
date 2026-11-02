"""test_sim3.py - checks for dashboard v3 (run: python test_sim3.py). Needs no window. test_sim2.py still covers v2 physics."""
import math, sys
import xbox_input as xi
import robot_sim3 as r3, sim_physics as ph, sim_tower as tw
import dash_ui as ui
import controller_dashboard_v3_2 as d3

def st(**k):
    s = xi.ControllerState()
    for a, b in k.items(): setattr(s, a, b)
    return s

def run(sim, sec, **k):
    a = st(**k)
    for _ in range(int(sec * 60)): sim.drive(a, 1 / 60)

def settle(sim):
    for _ in range(120): sim.world.step_cubes(1 / 60)

TESTS = []
def test(f): TESTS.append(f); return f

@test
def spawn_in_start_zone():
    for team, (cx, cy) in (('blue', (-82, -92)), ('red', (82, 92))):
        for rb in r3.ROBOTS:
            s = r3.RobotSim3(rb, team)
            assert abs(s.x - cx) <= 10 and abs(s.y - cy) <= 10, (team, rb, s.x, s.y)
            for p in s.parts():
                assert abs(p.cx - cx) < 10 + p.hx + 1, 'body must be in the zone'

@test
def robot_within_20cm_cube():
    for rb in r3.ROBOTS:
        xs, ys, zs = [], [], []
        for cx, cy, hx, hy, z0, z1 in r3.PARTS[rb]:
            xs += [cx - hx, cx + hx]; ys += [cy - hy, cy + hy]; zs.append(z1)
        assert max(xs) - min(xs) <= 20.01 and max(ys) - min(ys) <= 20.01, (rb, 'footprint')
    t = tw.Tower()
    assert tw.LIFT_MAX + tw.BOX_H <= 20.0 and r3.MAST[5] <= 20.0

@test
def pillar_is_5mm():
    s = r3.RobotSim3()
    assert s.world.pillar_h == 0.5
    settle(s)
    assert all(abs(c.z - 0.5) < 1e-6 for c in s.world.cubes)
    assert all(p.z1 == 0.5 for p in s.world.pillars)

@test
def left_stick_turns_only_with_lt():
    s = r3.RobotSim3()
    run(s, 1.0, lx=1.0)
    assert abs(s.th) < 1e-9 and abs(s.x + 82) < 1e-6, 'left stick X does nothing without LT'
    run(s, 1.0, ly=1.0)
    assert s.y > -96 + 10 and abs(s.th) < 1e-9
    run(s, 0.5, lt=1.0, lx=1.0)
    assert s.th < -0.1, 'LT + left stick right turns clockwise'
    run(s, 1.0)
    th = s.th
    run(s, 0.5, rx=1.0)
    assert abs(s.th - th) < 1e-3, 'right stick X no longer turns'

@test
def speed_cap_20_and_30():
    s = r3.RobotSim3(arena=False)
    run(s, 2.0, ly=1.0)
    assert abs(s.speed - 20.0) < 0.3, s.speed
    run(s, 2.0, ly=1.0, rt=1.0)
    assert abs(s.speed - 30.0) < 0.3, s.speed
    run(s, 2.0, ly=1.0, rt=0.5)
    assert abs(s.speed - 25.0) < 0.3, s.speed

@test
def dpad_slow_nudge():
    s = r3.RobotSim3(arena=False)
    run(s, 2.0, dpad=(0, 1))
    assert abs(s.speed - r3.DPAD_SPEED) < 0.2 and abs(s.th) < 1e-9
    run(s, 1.0)
    run(s, 2.0, dpad=(0, -1)); assert s.speed < -r3.DPAD_SPEED + 0.3
    run(s, 1.0)
    th = s.th; run(s, 1.0, dpad=(1, 0)); assert s.th < th - 0.3 and abs(s.speed) < 0.5, 'right = clockwise, nearly in place'
    th = s.th; run(s, 1.0, dpad=(-1, 0)); assert s.th > th + 0.3
    run(s, 1.0, dpad=(0, 1), rt=1.0); assert abs(s.speed - r3.DPAD_SPEED) < 0.2, 'RT does not speed up the nudge'

@test
def gripper_follows_right_stick_x():
    t = r3.RobotSim3('tower', arena=False)
    run(t, 1.0, rx=0.5); assert abs(t.grip.ext - 0.5 * tw.PAD_MAX) < 0.05, 'half way push = half way closed'
    run(t, 1.0, rx=0.5); assert abs(t.grip.ext - 0.5 * tw.PAD_MAX) < 0.05, 'and it stays there'
    run(t, 1.0, rx=-1.0); assert t.grip.ext == 0.0, 'max left releases'
    run(t, 1.0, rx=1.0); assert abs(t.grip.ext - tw.PAD_MAX) < 1e-6
    s = r3.RobotSim3('simple', arena=False)
    run(s, 1.0, rx=1.0); assert s.grip.ang[1] > 5
    run(s, 1.0, rx=-1.0); assert s.grip.ang[1] < -39
    run(s, 1.0, lt=1.0); assert s.grip.ang[1] < -39, 'LT no longer touches the claw'

@test
def tower_lift_speed_follows_right_stick_y():
    t = r3.RobotSim3('tower', arena=False)
    l0 = t.grip.lift
    run(t, 0.5, ry=1.0); full = t.grip.lift - l0
    t.grip.lift = l0
    run(t, 0.5, ry=0.5); half = t.grip.lift - l0
    assert abs(half - full / 2) < 0.05
    t.mode = 'tank'; l1 = t.grip.lift
    a = st(); a.buttons['LB'] = True
    run(t, 0.0); t.drive(a, 0.5); assert t.grip.lift < l1

@test
def start_records_zero_and_faces_relative():
    s = r3.RobotSim3()
    s.mode = 'arcade_snap'
    run(s, 0.6, lt=1.0, lx=1.0)
    th0 = s.th
    run(s, 0.3)                                     # stop
    a = st(); a.buttons['START'] = True
    s.drive(a, 1 / 60); s.drive(st(), 1 / 60)
    assert abs(s.ref_th - s.th) < 0.2 and s.zero_count == 2
    ref = s.ref_th
    b = st(); b.buttons['B'] = True
    s.drive(b, 1 / 60)
    run(s, 2.0)
    assert abs(ph.wrap_pi(s.th - (ref - math.pi / 2))) < 0.01, 'B faces right of the NEW zero'
    c = st(); c.buttons['Y'] = True
    s.drive(c, 1 / 60); run(s, 2.0)
    assert abs(ph.wrap_pi(s.th - ref)) < 0.01 and s.heading_deg < 0.6 or s.heading_deg > 359.4

@test
def simple_claw_geometry_and_grab():
    s = r3.RobotSim3('simple', 'blue')
    for side in (-1, 1):
        o = s.grip.arm_obb(s, side)
        assert 0.9 <= o.z0 and o.z1 <= 3.5
    settle(s)
    c = s.world.cubes[1]                            # cube 2 on the blue side, (-83, -13)
    s.x, s.y, s.th = c.x, c.y - 17, 0.0
    for _ in range(600):
        s.drive(st(ly=0.3), 1 / 60)
        if s.y >= c.y - 10.5: break
    run(s, 0.3)
    run(s, 1.0, rx=1.0)
    assert s.grip.held is c
    run(s, 0.5, ly=-0.3, rx=1.0)
    assert c.held and abs(c.z - 0.5) < 1e-6
    run(s, 0.8, rx=0.0)
    assert s.grip.held is None

@test
def tower_drives_over_cube_when_raised():
    s = r3.RobotSim3('tower', 'blue')
    settle(s)
    c = [q for q in s.world.cubes if abs(q.x + 83) < 1 and abs(q.y + 13) < 1][0]
    s.x, s.y, s.th = -83.0, -26.0, 0.0
    for _ in range(900):
        s.drive(st(ly=0.3), 1 / 60)
        if s.y >= -9: break
    assert abs(c.x + 83) < 0.01 and abs(c.y + 13) < 0.01 and abs(c.z - 0.5) < 1e-6, 'raised box passes over without touching'

@test
def tower_grip_lift_stack_and_carry():
    s = r3.RobotSim3('tower', 'blue')
    settle(s)
    c2 = [q for q in s.world.cubes if abs(q.x + 83) < 1 and abs(q.y + 13) < 1][0]
    c4 = [q for q in s.world.cubes if abs(q.x + 83) < 1 and abs(q.y) < 1][0]
    s.x, s.y, s.th = -83.0, -24.0, 0.0
    for _ in range(600):
        s.drive(st(ly=0.25), 1 / 60)
        if s.y >= -13.0: break
    run(s, 0.5)
    run(s, 1.2, ry=-1.0); assert s.grip.lift < 1.0
    run(s, 0.8, rx=1.0); assert s.grip.held is c2
    run(s, 1.5, ry=1.0, rx=1.0); assert c2.z > 10 and c2.held
    for _ in range(600):
        s.drive(st(ly=0.25, rx=1.0), 1 / 60)
        if s.y >= -0.2: break
    run(s, 0.4, rx=1.0)
    assert abs(c4.z - 0.5) < 1e-6, 'raised held cube passes over the cube on the next pillar'
    run(s, 1.0, ry=-1.0, rx=1.0)
    run(s, 0.8, rx=0.0)
    assert s.grip.held is None and abs(c2.z - 3.5) < 0.05, ('cube 1 lands on cube 2', c2.z)
    run(s, 0.5, ry=-1.0); assert s.grip.lift < 1.0
    run(s, 0.8, rx=1.0); assert s.grip.held is c4
    run(s, 1.5, ry=1.0, rx=1.0)
    assert abs(c2.z - (c4.z + 3.0)) < 1e-6 and c2.z > 10, 'the stacked cube rides with the held one'

@test
def tower_limits():
    s = r3.RobotSim3('tower', 'blue')
    run(s, 4.0, ry=1.0); assert abs(s.grip.lift - tw.LIFT_MAX) < 1e-9
    run(s, 4.0, ry=-1.0); assert abs(s.grip.lift - tw.LIFT_MIN) < 1e-9

@test
def resets():
    s = r3.RobotSim3('simple', 'red')
    run(s, 1.0, ly=1.0)
    s.world.cubes[0].x += 20
    s.reset_robot()
    assert (round(s.x, 3), round(s.y, 3)) == (82.0, 96.0) and abs(s.world.cubes[0].x - s.world.cubes[0].home[0]) > 5
    s.reset_cubes()
    assert all(c.z > 5 for c in s.world.cubes)       # dropped from above again
    run(s, 1.0, ly=1.0); s.reset()
    assert (round(s.x, 3), round(s.y, 3)) == (82.0, 96.0) and s.ref_th == s.th

@test
def switch_robot_and_team():
    s = r3.RobotSim3('simple', 'blue')
    s.set_robot('tower'); assert s.grip.kind == 'tower' and abs(s.y + 92) < 1e-9
    s.set_team('red'); assert s.x > 0 and abs(ph.wrap_pi(s.th - math.pi)) < 1e-9

@test
def monitor_mode_no_arena():
    s = r3.RobotSim3('simple', 'blue', arena=False)
    run(s, 8.0, ly=1.0)
    assert s.y > 60, 'passes where the arena wall would be'
    d = s.snapshot()
    for k in ('type', 'x', 'y', 'th', 'heading', 'vl', 'vr', 'al', 'ar', 'arm_l', 'arm_r'): assert k in d
    t = r3.RobotSim3('tower', arena=False).snapshot()
    assert 'lift' in t and 'pad' in t

@test
def snapshot_drives_the_picture():
    import pygame
    from robot_sim import Camera
    surf, cam = pygame.Surface((300, 240)), Camera(300, 240, 400)
    d = r3.RobotSim3('tower', arena=False).snapshot()
    for v in r3.MON_VIEWS: r3.draw_monitor3(surf, cam, d, v)
    d = r3.RobotSim3('simple', arena=False).snapshot()
    for v in r3.MON_VIEWS: r3.draw_monitor3(surf, cam, d, v)
    s = r3.RobotSim3()
    for v in r3.VIEWS3: r3.draw_scene3(surf, cam, s, v)

@test
def widgets_and_window_mapping():
    app = d3.App()
    called = []
    u = d3.build_ui(app, dict(demo=lambda: called.append('demo'), learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: called.append('fs')))
    dd = app.dd_robot
    assert u.click(dd.rect.center) and dd.open
    assert u.click(dd.item_rect(1).center) and app.sim.robot_type == 'tower' and not dd.open
    app.dd_mode.click(app.dd_mode.rect.center); u.click(app.dd_mode.item_rect(1).center)
    assert app.mode == 'monitor' and not app.sim.arena
    d3.refresh_view_options(app); assert 'iso' in [v for v, _ in app.dd_view.options]
    b = [x for x in u.buttons if x.label == 'Fullscreen'][0]
    assert u.click(b.rect.center) and called == ['fs']
    assert not [x for x in u.buttons if x.label == 'Reset cubes'][0].is_enabled()
    assert ui.to_canvas((0, 0), (2720, 1600), 1360, 800) == (0, 0)
    assert ui.to_canvas((1360, 800), (2720, 1600), 1360, 800) == (680, 400)
    assert ui.to_canvas((5, 400), (2000, 800), 1360, 800) is None, 'in the side bar'

@test
def rendered_frame_all_modes():
    import pygame, controller_viewer as cv, controller_dashboard as cd
    from robot_sim import Camera
    fonts, big = cd.make_fonts()
    canvas, scene, cam = pygame.Surface((d3.CW, d3.CH)), pygame.Surface((d3.MW - 20, d3.SCENE_H3)), Camera(d3.MW - 20, d3.SCENE_H3, 400)
    app = d3.App(); u = d3.build_ui(app, dict(demo=lambda: 0, learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: 0))
    m = xi.default_mapping(); a, b, h = cv.demo_raw(1.0); s = xi.apply_mapping(m, a, b, h, 'DEMO')
    for mode in ('game', 'monitor'):
        app.set_mode(mode)
        for rb in r3.ROBOTS:
            app.sim.set_robot(rb)
            for v in app.views():
                app.set_view(v); d3.render_dashboard3(canvas, fonts, big, cam, scene, app, u, s, m, [])

@test
def mouse_camera_and_options():
    import pygame
    from robot_sim import Camera
    app = d3.App(); u = d3.build_ui(app, dict(demo=lambda: 0, learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: 0))
    assert app.cur_view() == 'arena' and app.in_scene((app.scene_rect()[0] + 5, app.scene_rect()[1] + 5)) and not app.in_scene((5, 5))
    y0 = app.look.yaw
    app.drag(50, 20)
    assert app.cur_view() == 'free' and app.look.yaw != y0
    for _ in range(200): app.drag(0, 50)
    assert 0.05 <= app.look.pitch <= 1.5
    z = app.look.zoom
    app.zoom(3); assert app.look.zoom < z
    for _ in range(100): app.zoom(-1)
    assert app.look.zoom <= 5.0
    cam = Camera(300, 240, 400); surf = pygame.Surface((300, 240))
    e1 = None
    for zoom in (0.5, 1.0, 2.0):
        app.look.zoom = zoom
        for v in r3.VIEWS3: r3.draw_scene3(surf, cam, app.sim, v, app.look)
        r3.set_view3(cam, 'arena', 0, 0, 0, 'simple', app.look)
        dist = math.sqrt(sum(c * c for c in cam.eye))
        if e1 is not None: assert dist > e1
        e1 = dist
    app.reset_cam(); assert app.look.zoom == 1.0
    L = app.look
    sts = {x.label: x for x in u.steppers}
    for name, on, val in (('Trail', 'trail_on', 'trail_i'), ('Shadow', 'shadow_on', 'shadow_i'), ('Arrow', 'arrow_on', 'arrow_i')):
        w = sts[name]
        assert u.click(w.r_tog.center) and getattr(L, on) is False
        assert u.click(w.r_tog.center) and getattr(L, on) is True
        v = getattr(L, val)
        assert u.click(w.r_plus.center) and getattr(L, val) > v
        for _ in range(40): u.click(w.r_minus.center)
        assert getattr(L, val) == 0.0
    L.trail_on = L.shadow_on = L.arrow_on = False
    r3.draw_scene3(surf, cam, app.sim, 'arena', L)
    r3.draw_monitor3(surf, cam, app.sim.snapshot(), 'iso', L)


@test
def lt_is_a_toggle_and_default_mode():
    s = r3.RobotSim3()
    assert s.mode == 'arcade_snap'
    s.mode = 'arcade'
    run(s, 0.5, lx=1.0); assert abs(s.th) < 1e-9 and not s.turn_on
    run(s, 0.2, lt=1.0); assert s.turn_on, 'first press: on'
    run(s, 0.2)                                    # LT released: stays on
    th = s.th; run(s, 0.5, lx=1.0); assert s.th < th - 0.05 and s.turn_on
    run(s, 0.2, lt=1.0); assert not s.turn_on, 'second press: off'
    run(s, 1.0)
    th = s.th; run(s, 0.5, lx=1.0); assert abs(s.th - th) < 1e-3
    run(s, 0.2, lt=1.0); s.reset_robot(); assert not s.turn_on, 'reset puts turning back to off'
    assert r3.RobotSim3('tower').mode == 'arcade_snap'

@test
def timer_states():
    app = d3.App()
    assert not app.timer_on
    app.timer_update(1.0, st()); assert app.timer_t == 0.0, 'off: nothing runs'
    app.timer_toggle(); assert app.timer_on and app.timer_state == 'armed' and app.timer_t == 0.0
    app.timer_update(1.0, st()); assert app.timer_t == 0.0, 'armed: waits'
    b = st(); b.buttons['A'] = True
    app.timer_update(0.5, b); assert app.timer_state == 'running'
    app.timer_update(1.0, st()); assert abs(app.timer_t - 1.5) < 1e-9
    app.timer_pause(); app.timer_update(5.0, st()); assert abs(app.timer_t - 1.5) < 1e-9 and app.timer_state == 'paused'
    app.timer_pause(); assert app.timer_state == 'running'
    app.timer_reset(); assert app.timer_t == 0.0 and app.timer_state == 'armed'
    d = st(); d.dpad = (0, 1); app.timer_update(0.1, d); assert app.timer_state == 'running', 'a D-pad press also starts it'
    app.timer_restart(); assert app.timer_t == 0.0 and app.timer_state == 'running'
    app.timer_t = 125.43; assert app.timer_text() == '2:05.4'
    app.timer_toggle(); assert not app.timer_on

@test
def timer_and_turn_buttons():
    app = d3.App(); u = d3.build_ui(app, dict(demo=lambda: 0, learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: 0))
    by = {b.label if isinstance(b.label, str) else 'turn': b for b in u.buttons}
    assert not by['Reset'].is_enabled() and not by['Restart'].is_enabled()
    u.click(by['Timer'].rect.center); assert app.timer_on and by['Reset'].is_enabled()
    u.click(by['Restart'].rect.center); assert app.timer_state == 'running'
    u.click(by['Reset'].rect.center); assert app.timer_state == 'armed'
    u.click(by['turn'].rect.center); assert app.sim.turn_on
    assert app.timer_rect()[0] + app.timer_rect()[2] <= d3.CW

@test
def benchmark_tower_faster_for_two_cubes():
    import bench_robots as b
    a, t = b.run_simple(2), b.run_tower(2)
    assert a.delivered(None) == 2 and t.delivered(None) == 2
    assert t.t < a.t, (t.t, a.t)

if __name__ == '__main__':
    bad = 0
    for t in TESTS:
        try: t(); print('PASS', t.__name__)
        except Exception as e:
            bad += 1; import traceback; print('FAIL', t.__name__); traceback.print_exc()
    print('%d of %d passed' % (len(TESTS) - bad, len(TESTS)))
    sys.exit(1 if bad else 0)
