#!/usr/bin/env python3
"""Run compiled SITL against a prescribed-attitude fixture; never connects to hardware.

Uses simulator-truth AHRS to isolate control signs from estimator calibration.
The fixture ignores motor forces: this is a controller test, not a hover test.
"""
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import subprocess
import tempfile
import threading
import time
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]

class Fixture(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 19302))
        self.sock.settimeout(.2)
        self.stop = False
        self.t = 0
        self.pose = (0., 0., 0.)
        self.rates = (0., 0., 0.)
        self.samples = []
        self.error = None

    def run(self):
        try:
            while not self.stop:
                try:
                    data, address = self.sock.recvfrom(2048)
                except socket.timeout:
                    continue
                magic, hz, count, *pwm = struct.unpack('<HHI16H', data)
                assert magic == 18458 and hz > 0
                self.t += 1/hz
                roll, pitch, yaw = self.pose
                self.samples.append((self.t, pwm[:8]))
                # Stationary specific force in body FRD, gravity excluded.
                accel = [9.80665*math.sin(pitch), -9.80665*math.sin(roll)*math.cos(pitch),
                         -9.80665*math.cos(roll)*math.cos(pitch)]
                state = {'timestamp': self.t, 'imu': {'gyro': self.rates, 'accel_body': accel},
                         'position': [0, 0, 0], 'velocity': [0, 0, 0],
                         'attitude': self.pose}
                self.sock.sendto(('\n'+json.dumps(state)+'\n').encode(), address)
        except Exception as error:
            self.error = error


def main():
    output = ROOT/'logs/attitude-response'
    output.mkdir(exist_ok=True, parents=True)
    fixture = Fixture()
    fixture.start()
    proc = link = None
    rows = []
    try:
        with tempfile.TemporaryDirectory(prefix='attitude-fixture-') as directory, (output/'sitl.log').open('w') as log:
            params = (ROOT/'base-drone.parm').read_text()
            # Apply the saved aircraft's rate/angle gains and motor thrust settings.
            for line in (ROOT.parent/'mav.parm').read_text().splitlines():
                name, value = line.split()
                if name.startswith(('ATC_', 'MOT_')):
                    params += f'\n{name} {value}'
            # Only this disposable simulator gets these fixture settings.
            params += '\nAHRS_EKF_TYPE 10\nLOG_DISARMED 1\nLOG_BITMASK 180222\nGCS_PID_MASK 7\nFS_GCS_ENABLE 0\nFS_THR_ENABLE 0\nBATT_FS_LOW_ACT 0\nBATT_FS_CRT_ACT 0\nFS_CRASH_CHECK 0\nDISARM_DELAY 0\nSIM_SPEEDUP 5\n'
            # ArduPilot's defaults loader does not use last-occurrence-wins.
            # Resolve overrides here and emit each parameter exactly once.
            values = {}
            for line in params.splitlines():
                if line.strip() and not line.startswith('#'):
                    key, value = line.split()
                    values[key] = value
            values['SIM_BATT_VOLTAGE'] = '25.2'
            params = '\n'.join(f'{k} {v}' for k,v in values.items())+'\n'
            default = Path(directory)/'fixture.parm'
            default.write_text(params)
            proc = subprocess.Popen([str(ROOT/'ardupilot/build/sitl/bin/arducopter'),
                '--model', 'JSON', '--instance', '23', '--sim-address', '127.0.0.1',
                '--sim-port-out', '19302', '--speedup', '5', '--defaults', str(default), '--wipe'],
                cwd=directory, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+30
            while time.monotonic()<deadline:
                if proc.poll() is not None:
                    raise RuntimeError((output/'sitl.log').read_text())
                try:
                    link = mavutil.mavlink_connection('tcp:127.0.0.1:5990', source_system=255)
                    break
                except OSError:
                    time.sleep(.1)
            assert link is not None
            latest = {}
            throttle = 1000
            def hold(seconds):
                end = fixture.t+seconds
                wall_end = time.monotonic()+max(15, seconds*2)
                next_send = 0
                while fixture.t<end:
                    if fixture.error:
                        raise fixture.error
                    assert time.monotonic()<wall_end, 'SITL fixture stalled'
                    if time.monotonic()>=next_send:
                        link.mav.heartbeat_send(6,8,0,0,0)
                        link.mav.rc_channels_override_send(1,1,1500,1500,throttle,1500,65535,65535,65535,65535)
                        next_send = time.monotonic()+.02
                    msg = link.recv_match(blocking=False)
                    if msg is not None:
                        latest[msg.get_type()] = msg
                        if msg.get_type() in ('STATUSTEXT', 'COMMAND_ACK'):
                            print(str(msg), flush=True)
                    else:
                        time.sleep(.001)
            hold(15)
            assert latest['HEARTBEAT'].autopilot == 3
            for message_id in (30, 36, 194):
                link.mav.command_long_send(1,1,511,0,message_id,50000,0,0,0,0,0)
            link.set_mode(0)
            hold(2)
            assert latest['HEARTBEAT'].custom_mode == 0
            # Force-arm only the child SITL process on a hardcoded loopback port.
            def arm(value):
                link.mav.command_long_send(1,1,400,0,int(value),21196,0,0,0,0,0)
                hold(2)
                assert bool(latest['HEARTBEAT'].base_mode & 128) == value
            cases = [('roll_right_down', (5,0,0)), ('roll_left_down', (-5,0,0)),
                     ('nose_down', (0,-5,0)), ('nose_up', (0,5,0)),
                     ('yaw_clockwise', (0,0,5)), ('yaw_anticlockwise', (0,0,-5))]
            for name, degrees in cases:
                fixture.pose = (0.,0.,0.)
                throttle = 1000
                hold(2)
                arm(True)
                throttle = 1300
                hold(4)
                assert latest['HEARTBEAT'].base_mode & 128, (name, 'disarmed before stimulus')
                assert min(fixture.samples[-1][1][i] for i in (0,2,4,6))>1100, (name, 'lift outputs not running')
                baseline = [statistics.mean(pwm[i] for t,pwm in fixture.samples if t>fixture.t-.3) for i in range(8)]
                fixture.pose = tuple(math.radians(v) for v in degrees)
                start = fixture.t
                hold(.8)
                pwm = [statistics.mean(p[i] for t,p in fixture.samples if start+.4<t<start+.8) for i in range(8)]
                att = latest['ATTITUDE']
                measured = [math.degrees(att.roll),math.degrees(att.pitch),math.degrees(att.yaw)]
                assert all(abs(a-b)<.2 for a,b in zip(measured,degrees)), (name,measured)
                delta = [a-b for a,b in zip(pwm,baseline)]
                row = {'case':name,'measured_deg':measured,'baseline_pwm':baseline,'response_pwm':pwm,
                       'right_minus_left_change':delta[2]+delta[4]-delta[0]-delta[6],
                       'front_minus_rear_change':delta[0]+delta[2]-delta[4]-delta[6],
                       'ccw_lateral_minus_cw_change':delta[1]+delta[5]-delta[3]-delta[7]}
                rows.append(row)
                print(json.dumps(row),flush=True)
                throttle=1000
                arm(False)
            (output/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
            for row in rows:
                name=row['case']
                if name.startswith('roll_'):
                    assert row['right_minus_left_change']*(1 if name=='roll_right_down' else -1)>5,row
                elif name.startswith('nose_'):
                    assert row['front_minus_rear_change']*(1 if name=='nose_down' else -1)>5,row
                else:
                    assert row['ccw_lateral_minus_cw_change']*(1 if name=='yaw_clockwise' else -1)>5,row
            print('PASS: compiled SITL attitude-to-motor correction signs for both roll/pitch/yaw directions')
    finally:
        if link:
            link.close()
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
        fixture.stop=True
        fixture.join(timeout=1)
        fixture.sock.close()

if __name__=='__main__':
    main()
