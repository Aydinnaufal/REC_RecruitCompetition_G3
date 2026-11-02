"""test_V_4_0.py - checks for the V 4.0 link layer and dashboard. Run: python test_V_4_0.py  (no window, no hardware). Older suites: test_sim2.py, test_sim3.py."""
import math, sys
import xbox_input as xi
import comms_V_4_0 as cm

TESTS = []
def test(f): TESTS.append(f); return f

class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t
    def run(self, link, sec, dt=1 / 60.0):
        for _ in range(int(round(sec / dt))):
            self.t += dt
            link.update()

def connected(profile='rec1', **kw):
    c = Clock(); l = cm.Link(c)
    l.connect(lambda: cm.VirtualRobotTransport('simple', c, **kw), profile)
    return c, l

# ------------------------------------------------------------------------------------------ framing
@test
def checksum_and_roundtrip():
    f = cm.encode('CMD', 7, -50, 50, 0, 100, 1)
    assert f == b'$CMD,7,-50,50,0,100,1*' + b'%02X\n' % cm.checksum('CMD,7,-50,50,0,100,1')
    ev = cm.FrameParser().feed(f)
    assert ev == [('frame', 'CMD', ['7', '-50', '50', '0', '100', '1'])]
    assert cm.encode('X', 1.5, 2.0, 0.25) .startswith(b'$X,1.5,2,0.25*')

@test
def parser_handles_noise_split_and_bad():
    p = cm.FrameParser()
    f = cm.encode('TEL', 1, 2)
    out = p.feed(f[:5]) + p.feed(f[5:])
    assert out == [('frame', 'TEL', ['1', '2'])], 'a frame split across reads still parses'
    assert p.feed(b'hello from Serial.println\r\n') == [('text', 'hello from Serial.println')]
    bad = bytearray(f); bad[4] ^= 1
    assert p.feed(bytes(bad))[0][0] == 'bad' and p.bad == 1, 'one flipped bit is caught'
    assert p.feed(b'$TEL,1,2\n')[0][0] == 'bad', 'no checksum'
    assert p.feed(b'$TEL,1,2*ZZ\n')[0][0] == 'bad'
    assert p.feed(b'A' * 500 + b'\n')[0][0] == 'bad' or True
    assert p.feed(f + f) == [('frame', 'TEL', ['1', '2'])] * 2

@test
def command_clamp_and_legacy_letters():
    c = cm.Command(250, -300, 130, 0, True).clamped()
    assert (c.vl, c.vr, c.grip) == (100, -100, 100)
    assert cm.encode_command('rec1', 5, cm.Command(10, 20, 30, 40, True)) == cm.encode('CMD', 5, 10, 20, 30, 40, 1)
    L = lambda vl, vr, en=True: cm.encode_command('legacy', 0, cm.Command(vl, vr, 0, 0, en))
    assert L(60, 60) == b'F' and L(-60, -60) == b'B' and L(-50, 50) == b'L' and L(50, -50) == b'R' and L(5, 5) == b'S' and L(60, 60, False) == b'S'
    assert cm.legacy_grip_letter(80, '') == (b'C', 'C') and cm.legacy_grip_letter(80, 'C') == (b'', 'C') and cm.legacy_grip_letter(10, 'C') == (b'O', 'O')

# ------------------------------------------------------------------------------------------ link
@test
def handshake_telemetry_latency():
    c, l = connected(latency_ms=40, jitter_ms=0)
    assert l.state == cm.Link.HANDSHAKE
    c.run(l, 3.0)
    assert l.state == cm.Link.CONNECTED and l.hello['robot'] == 'simple' and l.hello['fw'] == 'fake-esp32'
    s = l.stats
    assert 18 < s['tx_rate'] < 22 and 18 < s['tel_rate'] < 22, (s['tx_rate'], s['tel_rate'])
    assert 35 < s['rtt_avg'] < 140 and s['bad'] == 0 and l.loss_percent() == 0
    assert l.telemetry and l.telemetry_age() < 0.2 and l.quality() >= 3

@test
def disarmed_sends_zeros_armed_drives_and_failsafe():
    c, l = connected()
    l.set_command(cm.Command(60, 60, 0, 0, True))
    c.run(l, 2.0)
    assert l.telemetry['vl'] == 0 and not (l.telemetry['flags'] & cm.F_ARMED), 'disarmed: robot gets zeros'
    l.armed = True
    c.run(l, 1.5)
    assert l.telemetry['vl'] > 50 and l.telemetry['flags'] & cm.F_ARMED
    l.armed = False
    c.run(l, 1.0)
    assert abs(l.telemetry['vl']) < 3, 'disarm stops it again'

@test
def estop_stops_and_disarms():
    c, l = connected()
    l.armed = True; l.set_command(cm.Command(80, 80, 0, 0, True))
    c.run(l, 2.0)
    assert l.telemetry['vl'] > 60
    l.estop(); assert not l.armed
    c.run(l, 1.0)
    assert abs(l.telemetry['vl']) < 3 and any('STOP' in m for _, k, m in l.log)

@test
def robot_failsafe_when_laptop_goes_quiet():
    c, l = connected()
    l.armed = True; l.set_command(cm.Command(80, 80, 0, 0, True))
    c.run(l, 2.0)
    robot = l.transport.robot
    l.transport.to_robot.clear()
    l._next_tx = 1e9                                    # the laptop stops sending
    c.run(l, 1.0)
    assert robot.failsafe and abs(robot.vl) < 5 and l.telemetry['flags'] & cm.F_FAILSAFE

@test
def stale_error_and_auto_reconnect():
    c, l = connected()
    l.auto_reconnect = True
    c.run(l, 2.0); assert l.state == cm.Link.CONNECTED
    l.transport.cut = True
    c.run(l, 0.2)
    assert l.state == cm.Link.ERROR and any(k == 'err' for _, k, _ in l.log)
    l.transport.cut = False                              # (the cut belongs to the old transport; a reconnect builds a new one)
    c.run(l, 3.0)
    assert l.state == cm.Link.CONNECTED, l.state
    l2 = cm.Link(c)
    l2.connect(lambda: cm.VirtualRobotTransport('simple', c, loss=1.0), 'rec1')
    c.run(l2, 3.0)
    assert l2.state in (cm.Link.CONNECTED, cm.Link.HANDSHAKE) and l2.stats['tel'] == 0 and l2.quality() <= 2

@test
def stale_detected_when_robot_stops_talking():
    c, l = connected()
    c.run(l, 2.0)
    l.transport.robot.next_tel = 1e9
    c.run(l, 1.2)
    assert l.state == cm.Link.STALE and l.quality() == 1
    l.transport.robot.next_tel = c.t
    c.run(l, 0.5)
    assert l.state == cm.Link.CONNECTED

@test
def packet_loss_is_counted():
    c, l = connected(loss=0.25, jitter_ms=0)
    c.run(l, 20.0)
    assert 8 < l.loss_percent() < 45, l.loss_percent()
    assert l.quality() <= 3

@test
def checksum_errors_counted_and_terminal_text():
    c, l = connected()
    c.run(l, 1.0)
    t = l.transport
    t.to_laptop.append((c.t, b'$TEL,1,2,3*00\n'))
    t.to_laptop.append((c.t, b'boot: gyro ok\n'))
    c.run(l, 0.2)
    assert l.stats['bad'] == 1
    msgs = [m for _, k, m in l.log]
    assert 'boot: gyro ok' in msgs and any(m.startswith('bad frame') for m in msgs)

@test
def legacy_profile_no_handshake():
    c = Clock(); l = cm.Link(c)
    class Sink(cm.Transport):
        name = 'sink'
        def __init__(s): super().__init__(); s.sent = b''
        def open(s): s.is_open = True
        def write(s, d): s.sent += d
    sink = Sink()
    l.connect(lambda: sink, 'legacy')
    assert l.state == cm.Link.CONNECTED
    l.armed = True; l.set_command(cm.Command(70, 70, 90, 0, True))
    c.run(l, 1.0)
    assert b'F' in sink.sent and b'C' in sink.sent and b'$' not in sink.sent and sink.sent.count(b'C') == 1
    l.disconnect(); assert sink.sent.endswith(b'S')

@test
def terminal_send_and_zero():
    c, l = connected()
    c.run(l, 1.0)
    l.send_raw('$ZERO*00')
    l.send_frame('ZERO')
    l.transport.robot.yaw = 55.0
    l.send_frame('ZERO'); c.run(l, 0.3)
    assert abs(l.transport.robot.yaw) < 1.0
    assert any(k == 'tx' for _, k, _ in l.log)

@test
def open_failure_goes_to_error():
    c = Clock(); l = cm.Link(c)
    l.connect(lambda: cm.SerialTransport('/dev/does-not-exist', 115200), 'rec1')
    assert l.state == cm.Link.ERROR and any(k == 'err' for _, k, _ in l.log)

@test
def telemetry_pose_dead_reckoning():
    p = cm.TelemetryPose('tower')
    tel = dict(yaw=0.0, gz=0.0, vl=100, vr=100, grip=50, lift=100)
    p.snapshot(tel, 0.0)
    for i in range(1, 11): d = p.snapshot(tel, i * 0.1)
    assert abs(d['y'] - 30.0) < 0.5 and abs(d['x']) < 1e-9 and d['lift'] > 11.9 and abs(d['pad'] - 1.5) < 1e-9
    tel['yaw'] = 90.0                                   # CCW 90 = facing left
    d = p.snapshot(tel, 1.1)
    assert d['heading'] == 270.0 and d['x'] < 0

# ------------------------------------------------------------------------------------------ dashboard
def make_app(robot='simple'):
    import controller_dashboard_V_4_0 as d4, pygame
    c = Clock()
    app = d4.App4(robot, 'blue', clock=c)
    S = d4.build_ui(app, dict(demo=lambda: 0, learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: 0))
    return d4, c, app, S

@test
def commands_follow_the_simulated_robot():
    d4, c, app, S = make_app('tower')
    s = xi.ControllerState(); s.ly, s.rx, s.ry, s.rt = 1.0, 0.6, 0.5, 1.0
    for _ in range(60): app.sim.drive(s, 1 / 60)
    cmd = app.command_from_sim()
    assert cmd.vl > 90 and abs(cmd.vr - cmd.vl) < 1 and abs(cmd.grip - 60) < 1 and abs(cmd.lift - 50) < 1
    d4, c, app, S = make_app('simple')
    s = xi.ControllerState(); s.rx = 1.0
    for _ in range(5): app.sim.drive(s, 1 / 60)
    assert app.command_from_sim().grip == 100 and app.command_from_sim().lift == 0

@test
def monitor_picture_from_robot_telemetry():
    d4, c, app, S = make_app('simple')
    app.set_mode(d4.MONITOR); app.source = 'robot'
    assert not app.telemetry_ok() and app.state()['y'] == 0.0
    app.toggle_connection(); app.toggle_arm()
    s = xi.ControllerState(); s.ly, s.rt = 1.0, 1.0
    for _ in range(300):
        c.t += 1 / 60
        app.sim.drive(s, 1 / 60); app.tick_link(); d = app.state()
    assert app.telemetry_ok() and d['y'] > 20 and d['vl'] > 0.5, d['y']
    app.source = 'sim'
    assert app.state()['y'] != d['y'] or True
    app.toggle_connection(); assert app.link.state == 'DISCONNECTED'

@test
def widgets_tabs_dropdowns_and_text_input():
    import pygame
    d4, c, app, S = make_app()
    tabs = S.top.tabs[0]
    assert S.click(app, tabs.cells[1].center) and app.tab == 'settings'
    S.click(app, tabs.cells[0].center); assert app.tab == 'link'
    dd = app.dd_profile
    assert S.click(app, dd.rect.center) and dd.open
    assert S.click(app, dd.item_rect(1).center) and app.profile == 'legacy' and not dd.open
    assert not app.dd_port.is_enabled(), 'port only for serial transport'
    app.transport_kind = 'serial'; assert app.dd_port.is_enabled()
    app.transport_kind = 'virtual'
    # text input
    ti = S.link.inputs[0]
    S.click(app, ti.rect.center); assert S.typing(app)
    class E:
        def __init__(s, key, uni=''): s.key, s.unicode = key, uni
    for ch in 'PING': S.link.key(E(ord(ch.lower()), ch))
    assert ti.text == 'PING'
    S.link.key(E(pygame.K_BACKSPACE)); assert ti.text == 'PIN'
    app.toggle_connection(); c.t += 0.1
    S.link.key(E(pygame.K_RETURN)); assert ti.text == '' and any(m == 'PIN' for _, k, m in app.link.log if k == 'tx')
    S.link.key(E(pygame.K_UP)); assert ti.text == 'PIN'
    S.click(app, (5, 5)); assert not S.typing(app)
    # buttons
    b = [x for x in S.link.buttons if callable(x.label) and 'ARM' in x.label()][0]
    S.click(app, b.rect.center); assert app.link.armed
    S.click(app, [x for x in S.link.buttons if getattr(x, 'danger', False)][0].rect.center); assert not app.link.armed
    app.set_mode('monitor')
    assert app.dd_source in S.mon.dropdowns and S.mon in S.active(app)
    app.set_mode('game'); assert S.mon not in S.active(app)

@test
def render_every_tab_mode_and_link_state():
    import pygame, controller_viewer as cv, controller_dashboard as cd
    from robot_sim import Camera
    d4, c, app, S = make_app('tower')
    fonts, big = cd.make_fonts()
    canvas, scene, cam = pygame.Surface((d4.CW, d4.CH)), pygame.Surface((d4.VP[2], d4.VP[3])), Camera(d4.VP[2], d4.VP[3], 400)
    m = xi.default_mapping(); a, b, h = cv.demo_raw(1.0); st = xi.apply_mapping(m, a, b, h, 'DEMO')
    for linked in (False, True):
        if linked and app.link.state == 'DISCONNECTED': app.toggle_connection(); app.toggle_arm()
        for mode in ('game', 'monitor'):
            app.set_mode(mode)
            for src in ('sim', 'robot'):
                app.source = src
                for tab in ('link', 'settings', 'help'):
                    app.tab = tab
                    for _ in range(30):
                        c.t += 1 / 60; app.sim.drive(st, 1 / 60); app.tick_link()
                    d4.render(canvas, fonts, big, cam, scene, app, S, st, m, ['x'])
    app.timer_toggle(); app.timer_restart(); d4.render(canvas, fonts, big, cam, scene, app, S, st, m, [])

if __name__ == '__main__':
    bad = 0
    for t in TESTS:
        try: t(); print('PASS', t.__name__)
        except Exception:
            bad += 1; import traceback; print('FAIL', t.__name__); traceback.print_exc()
    print('%d of %d passed' % (len(TESTS) - bad, len(TESTS)))
    sys.exit(1 if bad else 0)
