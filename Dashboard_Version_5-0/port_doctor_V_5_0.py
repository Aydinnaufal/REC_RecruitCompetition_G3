"""
port_doctor_V_5_0.py - finds out why the robot cannot be reached. No window, no pygame needed. Run it with the SAME Python that runs the dashboard:

    python port_doctor_V_5_0.py              check pyserial, COM ports, paired Bluetooth devices, bleak; saves port_report_V_5_0.txt
    python port_doctor_V_5_0.py --ble        also scan for Bluetooth LE devices for 6 seconds (the robot is called REC-Robot)
    python port_doctor_V_5_0.py --probe COM7 open COM7, send HELLO? and say whether a V_5_0 robot answers (USB or Bluetooth COM port)

Paste the printed text to Claude if it does not tell you what to do.
"""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import comms_V_5_0 as cm


def scan_ble(seconds=6.0):
    sc = cm.BleScanner(seconds)
    print('\nScanning for Bluetooth LE devices for %d seconds (switch the robot on first)...' % seconds)
    sc.start()
    while sc.running:
        time.sleep(0.2)
    if sc.error:
        print('[FAIL] ' + sc.error)
        return sc.error
    if not sc.devices:
        print('[warn] no named Bluetooth LE devices found. Is the PC Bluetooth on? Is the robot on and flashed with BT_MODE = BT_BLE?')
        return 'no BLE devices'
    for d in sc.devices:
        mark = '  <-- the robot' if (d.nus or d.name.upper().startswith(cm.ROBOT_NAME_PREFIX)) else ''
        print('   %-22s %s  signal %s dBm%s' % (d.name[:22], d.address, d.rssi, mark))
    r = sc.robot()
    print('[ ok ] REC-Robot found: %s' % r.address if r else '[warn] no device called REC-... in that list')
    return ''


def probe(port):
    print('\nOpening %s and asking HELLO? for up to 10 seconds...' % port)
    info = [p for p in cm.list_serial_ports() if p.device.upper() == port.upper()]
    bt = bool(info and info[0].bluetooth)
    if info and info[0].role == 'in':
        print('[FAIL] %s is an INCOMING Bluetooth port: it can never connect out. Use the Outgoing one.' % port)
        return
    sc = cm.PortScanner([cm.PortInfo(port, '', '', bt, info[0].role if info else '', '')], only_probeable=False)
    sc.start()
    while sc.running:
        time.sleep(0.2)
    print(('[ ok ] ' if sc.found else '[FAIL] ') + sc.message)
    for dev, ok, why in sc.results:
        print('       %s: %s' % (dev, 'robot answered' if ok else why))


def main():
    rep = cm.diagnose()
    print(rep.text())
    path = os.path.join(HERE, 'port_report_V_5_0.txt')
    try:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(rep.text())
        print('Report saved to ' + path)
    except OSError as e:
        print('(could not save the report: %s)' % e)
    if '--ble' in sys.argv:
        scan_ble()
    if '--probe' in sys.argv:
        i = sys.argv.index('--probe')
        if i + 1 < len(sys.argv):
            probe(sys.argv[i + 1])
        else:
            print('--probe needs a port, for example:  --probe COM7')


if __name__ == '__main__':
    main()
