#!/usr/bin/env python3
"""Enable the existing current ADC, or run a timed props-off response scan.

Readings are UNCALIBRATED. A response from both ESCs does not prove summation.
"""
import argparse
import glob
import json
from pathlib import Path
import statistics
import time
from pymavlink import mavutil
from bench_test import Bench, select_autopilot

ROOT = Path(__file__).resolve().parent
KEYS = ('BATT_MONITOR', 'BATT_CURR_PIN', 'BATT_AMP_PERVLT', 'BATT_AMP_OFFSET',
        'BATT_VOLT_MULT', 'BATT_LOW_MAH', 'BATT_CRT_MAH', 'BATT_ARM_MAH')

class CurrentBench(Bench):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_samples = []
        self.last_current_time = None

    def poll(self):
        super().poll()
        sample = self.latest.get('SYS_STATUS')
        if sample and sample[0] != self.last_current_time:
            self.last_current_time = sample[0]
            # MAVLink uses centiamps; -1 means unavailable, not negative current.
            if sample[1].current_battery != -1:
                self.current_samples.append((sample[0], sample[1].current_battery / 100.0))

    def parameter(self, key):
        self.params.pop(key, None)
        for _ in range(3):
            self.link.mav.param_request_read_send(self.link.target_system,
                self.link.target_component, key.encode(), -1)
            self.hold(.6)
            if key in self.params:
                return self.params[key]
        raise RuntimeError('No parameter reply: '+key)

    def set_monitor(self, value):
        self.link.mav.param_set_send(self.link.target_system, self.link.target_component,
            b'BATT_MONITOR', value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.hold(.5)
        if self.parameter('BATT_MONITOR') != value:
            raise RuntimeError('BATT_MONITOR write did not verify')

    def motor(self, number, pwm=1150, seconds=3):
        self.latest.pop('COMMAND_ACK', None)
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0, number, 1, pwm, seconds, 1, 0, 0)


def median_since(samples, since):
    values = [value for stamp, value in samples if stamp >= since]
    if len(values) < 3:
        raise RuntimeError('Not enough valid current samples; check monitor/reboot/telemetry')
    return statistics.median(values)


def scan(bench):
    # Existing commissioning checks include disarmed, STABILIZE, calibrated RC,
    # expected mixer, a connected 6S pack, and approximately level attitude.
    bench.check('right')
    if bench.parameter('BATT_MONITOR') != 4 or bench.parameter('BATT_CURR_PIN') != 11:
        raise RuntimeError('Run --enable, reboot the FC, then rerun --test')
    if input('ALL EIGHT PROPS REMOVED, frame secured? Type RUN: ') != 'RUN':
        return []
    bench.hold(1)
    bench.check('right')
    result = []
    active = False
    try:
        for motor in range(1, 9):
            if bench.armed():
                raise RuntimeError('FC must be disarmed before each motor test')
            baseline_start = bench.clock()
            bench.hold(2)
            baseline = median_since(bench.current_samples, baseline_start)
            print(f'M{motor}: 3 seconds at 1150; Ctrl+C stops. Observe which physical motor spins.', flush=True)
            active = True
            bench.motor(motor)
            start = bench.clock()
            bench.hold(.6)
            ack = bench.latest.get('COMMAND_ACK')
            if not ack or ack[1].command != 209 or ack[1].result != 0:
                raise RuntimeError('Motor-test command not accepted; no automatic retry')
            bench.hold(1.8)
            bench.fresh('SYS_STATUS', age=1)
            running = median_since(bench.current_samples, start+.6)
            row = {'motor': motor, 'baseline_uncalibrated_A': baseline,
                   'running_uncalibrated_A': running, 'delta_uncalibrated_A': running-baseline}
            result.append(row)
            bench.log.write(json.dumps({'event': 'current_result', **row})+'\n')
            print(f'  UNCALIBRATED: baseline {baseline:.2f}, running {running:.2f}, delta {running-baseline:+.2f} A-equivalent', flush=True)
            # End explicitly as well as relying on the FC's three-second timeout.
            bench.motor(motor, pwm=1000, seconds=0)
            if not bench.stop():
                raise RuntimeError('Stop not confirmed; no further motor tests')
            active = False
            bench.hold(1)
    finally:
        if active:
            bench.motor(1, pwm=1000, seconds=0)
            if not bench.stop():
                raise RuntimeError('STOP NOT CONFIRMED; no further motor tests')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='USB serial path, or tcp:192.168.2.1:5760')
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--enable', action='store_true', help='Save settings and enable existing current input; no motor commands')
    actions.add_argument('--test', action='store_true', help='Props-off scan of motors 1–8, one at a time')
    actions.add_argument('--disable', action='store_true', help='Return to voltage-only monitoring; no motor commands')
    args = parser.parse_args()
    ports = [args.port] if args.port else glob.glob('/dev/serial/by-id/usb-ArduPilot_StellarH7V2_*')
    if len(ports) != 1:
        parser.error('Specify --port: expected exactly one StellarH7V2 USB device')
    path = ROOT/'logs'/f'current-{time.time_ns()}.jsonl'
    path.parent.mkdir(exist_ok=True)
    link = mavutil.mavlink_connection(ports[0], baud=115200, source_system=255)
    try:
        select_autopilot(link)
        with path.open('w', buffering=1) as log:
            bench = CurrentBench(link, log)
            for message, interval in ((1, 100000), (30, 50000), (36, 100000)):
                link.mav.command_long_send(link.target_system, link.target_component,
                    511, 0, message, interval, 0, 0, 0, 0, 0)
            bench.hold(2)
            if bench.armed():
                raise RuntimeError('Disarm before running this tool')
            params = {key: bench.parameter(key) for key in KEYS}
            log.write(json.dumps({'event': 'initial_parameters', 'values': params})+'\n')
            if args.enable:
                if params['BATT_CURR_PIN'] != 11 or params['BATT_MONITOR'] not in (3,4):
                    raise RuntimeError('Unexpected current-input configuration; no changes made')
                if any(params[key] != 0 for key in ('BATT_LOW_MAH','BATT_CRT_MAH','BATT_ARM_MAH')):
                    raise RuntimeError('Capacity thresholds are enabled; cannot use uncalibrated current')
                bench.set_monitor(4)
                print('Enabled current ADC. Scale/offset and voltage calibration unchanged. No motor commands sent.')
                print('Disconnect battery and USB, reconnect, then run --test with ALL PROPS REMOVED.')
            elif args.disable:
                bench.set_monitor(3)
                print('Voltage-only monitoring restored. Power-cycle the FC. No motor commands sent.')
            else:
                scan(bench)
                print('Scan complete. No response is inconclusive at props-off current; do not increase power to chase it.')
                print('Readings are NOT calibrated total battery current. Both groups responding does not prove summation.')
    finally:
        link.close()
        print('Log:', path)

if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, KeyboardInterrupt) as error:
        raise SystemExit(str(error) or 'Interrupted; FC motor-test timeout is three seconds')
