#!/usr/bin/env python3
"""Serial/Wi-Fi props-off bench checks. No parameter writes or forced arming."""
import argparse
import glob
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil

NEUTRAL = (1500, 1500, 1000, 1500)
TESTS = {
    'pitch-up': (1500, 1600, 1050, 1500),
    'pitch-down': (1500, 1400, 1050, 1500),
    'right': (1700, 1500, 1050, 1500),
    'left': (1300, 1500, 1050, 1500),
    'yaw-right': (1500, 1500, 1050, 1700),
    'yaw-left': (1500, 1500, 1050, 1300),
    'roll-right-down': (1500, 1500, 1050, 1500),
    'nose-down': (1500, 1500, 1050, 1500),
    'rc-loss': NEUTRAL,
    'link-loss': NEUTRAL,
}
REQUIRED = {
    'FRAME_CLASS': 16, 'SCR_ENABLE': 1, 'SCR_LD_CHECKSUM': 934390,
    'SCR_RUN_CHECKSUM': 934390, 'MOT_PWM_TYPE': 5, 'SERVO_BLH_MASK': 255,
    'SERVO_BLH_OTYPE': 5, 'ARMING_SKIPCHK': 0, 'MAV_GCS_SYSID': 255,
    'RC_OPTIONS': 1, 'RC_OVERRIDE_TIME': 0.5, 'RC_FS_TIMEOUT': 1,
    'FS_THR_ENABLE': 3, 'FS_GCS_ENABLE': 5, 'FS_GCS_TIMEOUT': 2,
    'DISARM_DELAY': 10,
    'RCMAP_ROLL': 1, 'RCMAP_PITCH': 2, 'RCMAP_THROTTLE': 3, 'RCMAP_YAW': 4,
    **{f'RC{i}_REVERSED': 0 for i in range(1, 5)},
    **{f'RC{i}_MIN': 1000 for i in range(1, 5)},
    **{f'RC{i}_MAX': 2000 for i in range(1, 5)},
    **{f'SERVO{i}_FUNCTION': 32+i for i in range(1, 9)},
}


def select_autopilot(link, timeout=10):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        msg = link.recv_match(type='HEARTBEAT', blocking=True,
                              timeout=min(1, max(0, deadline-time.monotonic())))
        if msg is not None and msg.get_srcComponent() == 1 and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            link.target_system = msg.get_srcSystem()
            link.target_component = msg.get_srcComponent()
            return
    raise RuntimeError('No ArduPilot FC heartbeat; close MAVProxy and check the connection')


class Bench:
    def __init__(self, link, log, clock=time.monotonic, sleep=time.sleep):
        self.link, self.log, self.clock, self.sleep = link, log, clock, sleep
        self.latest, self.params = {}, {}
        self.next_heartbeat = 0
        self.command_time = 0
        self.texts = []

    def poll(self):
        # Bounded draining prevents heavy telemetry from delaying the stop timer.
        for _ in range(200):
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != self.link.target_system:
                continue
            if msg.get_srcComponent() != self.link.target_component:
                continue  # the Wi-Fi bridge has its own heartbeat (component 68)
            kind = msg.get_type()
            if kind == 'BAD_DATA':
                continue
            self.latest[kind] = (self.clock(), msg)
            if kind == 'PARAM_VALUE':
                key = msg.param_id
                if isinstance(key, bytes):
                    key = key.decode()
                self.params[key.rstrip('\0')] = msg.param_value
            if kind in ('HEARTBEAT', 'ATTITUDE', 'SERVO_OUTPUT_RAW',
                        'RC_CHANNELS', 'SYS_STATUS', 'STATUSTEXT', 'COMMAND_ACK'):
                self.log.write(json.dumps({'received_monotonic': self.clock(),
                                           'message': msg.to_dict()}) + '\n')
            if kind == 'STATUSTEXT':
                self.texts.append((self.clock(), msg.text))
                print('FC:', msg.text)

    def send(self, channels=None):
        now = self.clock()
        if now >= self.next_heartbeat:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.next_heartbeat = now + 1
        if channels is not None:
            self.link.mav.rc_channels_override_send(
                self.link.target_system, self.link.target_component,
                *channels, 65535, 65535, 65535, 65535)

    def fresh(self, kind, age=2):
        item = self.latest.get(kind)
        if item is None or self.clock()-item[0] > age:
            raise RuntimeError(f'No recent {kind}; test stopped')
        return item[1]

    def armed(self):
        return bool(self.fresh('HEARTBEAT').base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def hold(self, seconds, channels=None, require_armed=False):
        deadline = self.clock() + seconds
        while self.clock() < deadline:
            self.poll()
            if require_armed and not self.armed():
                raise RuntimeError('FC disarmed during test; no automatic re-arm')
            if require_armed:
                if self.fresh('HEARTBEAT').custom_mode != 0:
                    raise RuntimeError('FC left STABILIZE; test stopped')
                att = self.fresh('ATTITUDE')
                if max(abs(att.roll), abs(att.pitch)) > math.radians(12):
                    raise RuntimeError('Frame moved beyond bench tilt limit; test stopped')
            self.send(channels)
            self.sleep(min(0.05, max(0, deadline-self.clock())))

    def arm_command(self, arm):
        self.command_time = self.clock()
        self.link.mav.command_long_send(
            self.link.target_system, self.link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            int(arm), 0, 0, 0, 0, 0, 0)  # param2=0: never force

    def check(self, test):
        if self.armed():
            raise RuntimeError('Already armed: this tool requires a disarmed FC')
        hb = self.fresh('HEARTBEAT')
        if hb.custom_mode != 0:
            raise RuntimeError('Select STABILIZE in MAVProxy first; this tool does not change mode')
        self.read_parameters()
        if self.armed() or self.fresh('HEARTBEAT').custom_mode != 0:
            raise RuntimeError('Armed state or mode changed during checks; test cancelled')
        mismatches = {k: (self.params[k], v) for k, v in REQUIRED.items()
                      if abs(self.params[k]-v) > 1e-5}
        if mismatches:
            raise RuntimeError(f'Mismatched parameters (actual, expected): {mismatches}')
        volts = self.fresh('SYS_STATUS').voltage_battery / 1000
        if not 21.6 <= volts <= 25.8:
            raise RuntimeError(f'Expected connected, healthy 6S battery; received {volts:.2f} V')
        att = self.fresh('ATTITUDE')
        roll, pitch = math.degrees(att.roll), math.degrees(att.pitch)
        print(f'Checks: battery {volts:.2f} V; roll {roll:.2f}°, pitch {pitch:.2f}°; disarmed, STABILIZE.')
        if abs(roll) > 12 or abs(pitch) > 12:
            raise RuntimeError('Bench tilt exceeds 12 degrees')
        if test == 'roll-right-down' and not (3 <= roll <= 10 and abs(pitch) <= 3):
            raise RuntimeError('Support the stationary frame right-side-down by 3–10 degrees before starting')
        if test == 'nose-down' and not (-10 <= pitch <= -3 and abs(roll) <= 3):
            raise RuntimeError('Support the stationary frame nose-down by 3–10 degrees before starting')
        if test not in ('roll-right-down', 'nose-down') and (abs(roll) > 3 or abs(pitch) > 3):
            raise RuntimeError('Command tests require the frame approximately level (within 3 degrees)')

    def read_parameters(self):
        print(f'Checking {len(REQUIRED)} parameters (paced requests; retries for missing replies)…')
        # Keep one request outstanding instead of overflowing the FC reply queue.
        # Clear cached values so the post-prompt check also requires fresh replies.
        self.params.clear()
        overall_deadline = self.clock() + 30
        missing = []
        for key in REQUIRED:
            for attempt in range(3):
                if self.clock() >= overall_deadline:
                    break
                self.link.mav.param_request_read_send(self.link.target_system,
                    self.link.target_component, key.encode(), -1)
                deadline = min(self.clock()+0.8, overall_deadline)
                while self.clock() < deadline and key not in self.params:
                    self.hold(0.05)
                if key in self.params:
                    break
            if key not in self.params:
                missing.append(key)
        if missing:
            raise RuntimeError('No parameter replies after retries: '+', '.join(missing)+
                               '. No arming request sent; settings were not changed.')

    def stop(self):
        # Repeated ordinary disarm requests; no forced disarm and no param changes.
        deadline = self.clock() + 3
        last_request = -float('inf')
        confirmed = False
        while self.clock() < deadline:
            self.send(NEUTRAL)
            if self.clock()-last_request >= 0.25:
                self.arm_command(False)
                last_request = self.clock()
            self.poll()
            hb = self.latest.get('HEARTBEAT')
            out = self.latest.get('SERVO_OUTPUT_RAW')
            if hb and out and hb[0] >= deadline-3 and out[0] >= deadline-3:
                stopped = all(getattr(out[1], f'servo{i}_raw') <= 1000 for i in range(1, 9))
                if not (hb[1].base_mode & 128) and stopped:
                    confirmed = True
                    break
            self.sleep(0.05)
        if confirmed:
            # Release overrides only after a fresh disarmed/stopped report.
            self.link.mav.rc_channels_override_send(self.link.target_system,
                self.link.target_component, 0, 0, 0, 0, 65535, 65535, 65535, 65535)
            print('STOP CONFIRMED: disarmed; motor outputs 1–8 stopped.')
        else:
            print('STOP NOT CONFIRMED: disconnect the battery now. Do not assume the motors stopped.')
        self.log.write(json.dumps({'event': 'stop', 'confirmed': confirmed,
                                   'received_monotonic': self.clock()})+'\n')
        return confirmed

    def loss_test(self, test):
        # Ground-only test: never request throttle above minimum. Do not send a
        # disarm command until after observing whether the FC itself disarms.
        self.hold(2.5, NEUTRAL, require_armed=True)
        start = self.clock()
        self.log.write(json.dumps({'event': 'loss_start', 'test': test,
                                   'received_monotonic': start})+'\n')
        print('Withholding RC inputs'+(' and all outgoing traffic.' if test == 'link-loss'
                                      else '; GCS heartbeats continue.'))
        first_disarmed = None
        while self.clock()-start < 4:
            self.poll()
            if not self.armed() and first_disarmed is None:
                first_disarmed = self.clock()-start
            if test == 'rc-loss':
                self.send()  # heartbeat only: no RC override or disarm packet
            self.sleep(0.05)
        texts = [text for timestamp, text in self.texts if timestamp >= start]
        radio = any(text.startswith('Radio Failsafe') and 'Cleared' not in text for text in texts)
        gcs = any(text.startswith('GCS Failsafe') and 'Cleared' not in text for text in texts)
        outputs = self.fresh('SERVO_OUTPUT_RAW')
        stopped = all(getattr(outputs, f'servo{i}_raw') <= 1000 for i in range(1, 9))
        passed = (first_disarmed is not None and not self.armed() and stopped and radio
                  and (gcs if test == 'link-loss' else not gcs))
        result = {'event': 'loss_result', 'test': test, 'passed': passed,
                  'disarmed_after_seconds': first_disarmed, 'radio_failsafe': radio,
                  'gcs_failsafe': gcs, 'outputs_stopped_before_cleanup': stopped,
                  'received_monotonic': self.clock()}
        self.log.write(json.dumps(result)+'\n')
        print('FC FAILSAFE RESULT:', result)
        if not passed:
            raise RuntimeError('Failsafe test not verified; cleanup will request disarm')
        # Resume only heartbeat traffic, never arm or restore active inputs.
        deadline = self.clock()+2
        while self.clock() < deadline:
            self.poll()
            if self.armed():
                raise RuntimeError('Unexpected armed state after heartbeat recovery')
            self.send()
            self.sleep(0.05)
        print('Heartbeat restored; FC remained disarmed. Ground test only; airborne LAND is untested.')

    def run(self, test):
        self.check(test)
        # Operator confirmation occurs before any RC overrides or arming request.
        description = 'ground failsafe test at minimum throttle' if test.endswith('loss') else '2-second motor command'
        response = input(f'Props REMOVED, frame secured, shafts/wires clear? Type RUN for {description}: ')
        if response != 'RUN':
            print('Cancelled; no arming request sent.')
            return
        attempted = False
        try:
            # Recheck after any time spent at the prompt; never use stale state.
            self.hold(1)
            self.check(test)
            self.hold(0.5, NEUTRAL)
            attempted = True
            self.arm_command(True)
            deadline = self.clock()+4
            while self.clock() < deadline:
                self.hold(0.05, NEUTRAL)
                ack = self.latest.get('COMMAND_ACK')
                if ack and ack[0] >= self.command_time and ack[1].command == 400 and ack[1].result != 0:
                    raise RuntimeError(f'Arming rejected: result {ack[1].result}')
                if self.armed():
                    break
            else:
                raise RuntimeError('No armed confirmation')
            print(f'Running {test}; Ctrl+C stops early.')
            self.log.write(json.dumps({'event': 'test_start', 'test': test,
                                       'received_monotonic': self.clock()})+'\n')
            if test in ('rc-loss', 'link-loss'):
                self.loss_test(test)
            else:
                self.hold(2, TESTS[test], require_armed=True)
        finally:
            if attempted:
                self.log.write(json.dumps({'event': 'command_end',
                                           'received_monotonic': self.clock()})+'\n')
                if not self.stop():
                    raise RuntimeError('Motor stop could not be verified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Serial path or tcp:IP:PORT; default: unique ArduPilot USB by-id path')
    parser.add_argument('--test', choices=TESTS, help='Explicitly select a powered test; omitted = status only')
    args = parser.parse_args()
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/usb-ArduPilot_StellarH7V2_*')
    if len(ports) != 1:
        parser.error('Specify --port: expected exactly one StellarH7V2 USB device')
    path = Path(__file__).resolve().parent/'logs'/f'bench-{time.time_ns()}.jsonl'
    path.parent.mkdir(exist_ok=True)
    print('Close MAVProxy first. Connecting to', ports[0])
    link = mavutil.mavlink_connection(ports[0], baud=115200, source_system=255)
    try:
        select_autopilot(link)
        with path.open('w', buffering=1) as log:
            bench = Bench(link, log)
            for message, interval in ((30, 50000), (36, 100000), (65, 100000), (1, 500000)):
                link.mav.command_long_send(link.target_system, link.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message, interval, 0, 0, 0, 0, 0)
            bench.hold(2)
            if args.test:
                bench.run(args.test)
            else:
                att = bench.fresh('ATTITUDE')
                print('STATUS ONLY: no RC overrides or arming request sent.')
                print('Armed:', bench.armed(), 'roll/pitch degrees:',
                      round(math.degrees(att.roll), 2), round(math.degrees(att.pitch), 2))
                print('Battery volts:', bench.fresh('SYS_STATUS').voltage_battery/1000)
    finally:
        link.close()
        print('Telemetry:', path)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, KeyboardInterrupt) as error:
        raise SystemExit(str(error) or 'Interrupted')
