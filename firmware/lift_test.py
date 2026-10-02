#!/usr/bin/env python3
"""Short tethered props-on test in STABILIZE; Space cuts throttle and disarms.

Not a flight interface: no position hold, no altitude hold, no automatic landing.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

from pymavlink import mavutil
from bench_test import Bench, NEUTRAL, select_autopilot


class Controls:
    """Bound pilot input; keep the selected throttle until changed or stopped."""
    def __init__(self, now):
        self.throttle = 1000
        self.last_step = -float('inf')
        self.reason = None

    def update(self, key, now):
        if key == 'STOP':
            self.reason = 'Stop key pressed'
        elif key in ('UP', 'DOWN') and now-self.last_step >= 0.4:
            self.last_step = now
            if key == 'DOWN':
                self.throttle = max(1000, self.throttle-25)
            else:
                self.throttle = min(1500, max(1050, self.throttle+25))
        return self.reason


class KeyDecoder:
    """Decode normal/application cursor keys, including split terminal reads."""
    sequences = {b'\x1b[A': 'UP', b'\x1b[B': 'DOWN',
                 b'\x1bOA': 'UP', b'\x1bOB': 'DOWN'}

    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        if not data or b' ' in data:
            self.buffer = b''
            return 'STOP'  # Space has priority over arrows; EOF also stops.
        self.buffer += data
        result = None
        while self.buffer:
            match = next((seq for seq in self.sequences if self.buffer.startswith(seq)), None)
            if match:
                result = self.sequences[match]
                self.buffer = self.buffer[len(match):]
            elif any(seq.startswith(self.buffer) for seq in self.sequences):
                break
            else:
                self.buffer = self.buffer[1:]
        return result


def drive(bench, read_key):
    controls = Controls(bench.clock())
    displayed = None
    while True:
        bench.poll()
        if controls.update(read_key(), bench.clock()):
            print('\n'+controls.reason)
            return
        if not bench.armed() or bench.fresh('HEARTBEAT').custom_mode != 0:
            raise RuntimeError('FC disarmed or left STABILIZE; stopping')
        att = bench.fresh('ATTITUDE')
        if max(abs(att.roll), abs(att.pitch)) > math.radians(12):
            raise RuntimeError('Tilt exceeded 12 degrees; stopping')
        bench.fresh('SERVO_OUTPUT_RAW')
        volts = bench.fresh('SYS_STATUS').voltage_battery/1000
        if not 21 <= volts <= 25.8:
            raise RuntimeError(f'Battery reading outside test range: {volts:.2f} V')
        bench.send((1500, 1500, controls.throttle, 1500))
        if displayed != controls.throttle:
            displayed = controls.throttle
            print(f'\rThrottle input: {displayed} | UP/DOWN = adjust | SPACE = CUT AND DISARM', flush=True)
            bench.log.write(json.dumps({'event': 'pilot_throttle', 'value': displayed,
                                       'received_monotonic': bench.clock()})+'\n')
        bench.sleep(0.05)


def execute(bench, confirm=input, read_key=None):
    bench.check('right')  # shared configuration checks; requires level STABILIZE
    print('TETHERED PROPS-ON TEST ONLY. Stabilisation active; all eight motors may run.')
    print('Space or Ctrl+C: CUT AND DISARM, not a landing.')
    print('Up arrow raises throttle input by 25; Down arrow lowers it.')
    print('Selected throttle persists. No operator-inactivity or test-duration timeout.')
    print('Throttle input ceiling 1500 is NOT a ceiling on individual motor outputs.')
    if confirm('Props fitted correctly, short tethers secured, area clear? Type RUN: ') != 'RUN':
        print('Cancelled; no arming request sent.')
        return
    bench.hold(1)
    bench.check('right')
    attempted = False
    try:
        bench.hold(0.5, NEUTRAL)
        attempted = True
        bench.arm_command(True)
        deadline = bench.clock()+4
        while bench.clock() < deadline:
            if read_key() == 'STOP':
                print('Stop pressed during arming; cancelling')
                return
            bench.hold(0.05, NEUTRAL)
            ack = bench.latest.get('COMMAND_ACK')
            if ack and ack[0] >= bench.command_time and ack[1].command == 400 and ack[1].result != 0:
                raise RuntimeError(f'Arming rejected: {ack[1].result}')
            if bench.armed():
                break
        else:
            raise RuntimeError('No armed confirmation')
        print('ARMED at minimum throttle. UP/DOWN adjust throttle. SPACE stops.')
        drive(bench, read_key)
    finally:
        if attempted and not bench.stop():
            raise RuntimeError('Stop not confirmed: disconnect battery now')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='tcp:192.168.2.1:5760')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Run in an interactive terminal so the stop key is available')
    path = Path(__file__).resolve().parent/'logs'/f'lift-{time.time_ns()}.jsonl'
    path.parent.mkdir(exist_ok=True)
    print('Close MAVProxy first. Connecting to', args.port)
    link = mavutil.mavlink_connection(args.port, baud=115200, source_system=255)
    original = None
    try:
        select_autopilot(link)
        with path.open('w', buffering=1) as log:
            bench = Bench(link, log)
            for message, interval in ((30, 50000), (36, 100000), (65, 100000), (1, 500000)):
                link.mav.command_long_send(link.target_system, link.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message, interval, 0, 0, 0, 0, 0)
            bench.hold(2)
            def confirm(prompt):
                nonlocal original
                answer = input(prompt)
                if answer == 'RUN':
                    original = termios.tcgetattr(sys.stdin)
                    tty.setcbreak(sys.stdin.fileno())
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                return answer
            decoder = KeyDecoder()
            def read_key():
                if select.select([sys.stdin], [], [], 0)[0]:
                    return decoder.feed(os.read(sys.stdin.fileno(), 256))
                return None
            execute(bench, confirm, read_key)
    finally:
        if original is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        link.close()
        print('Telemetry:', path)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, KeyboardInterrupt) as error:
        raise SystemExit(str(error) or 'Interrupted; stop cleanup attempted')
