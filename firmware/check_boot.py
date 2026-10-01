#!/usr/bin/env python3
"""Disarmed SITL boot/configuration check. Does not validate aircraft dynamics."""
import json
from pathlib import Path
import subprocess
import tempfile
import time
from pymavlink import mavutil

root = Path(__file__).resolve().parent
expected = {}
# BLHeli output hardware is compiled out of this stock SITL target.
hardware_only = {'SERVO_BLH_MASK', 'SERVO_BLH_OTYPE'}
for line in (root / 'base-drone.parm').read_text().splitlines():
    if line and not line.startswith('#'):
        key, value = line.split()
        expected[key] = float(value)

with tempfile.TemporaryDirectory(prefix='base-drone-boot-') as run:
    log_path = root / 'logs/boot.log'
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open('w') as log:
        proc = subprocess.Popen([
            str(root / 'ardupilot/build/sitl/bin/arducopter'),
            '--model', 'quad', '--speedup', '1', '--instance', '12',
            '--defaults', str(root / 'base-drone.parm'), '--wipe',
        ], cwd=run, stdout=log, stderr=subprocess.STDOUT)
        link = None
        messages = []
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(log_path.read_text())
                try:
                    link = mavutil.mavlink_connection('tcp:127.0.0.1:5880', source_system=255)
                    break
                except OSError:
                    time.sleep(0.2)
            assert link is not None, 'SITL TCP endpoint unavailable'
            assert link.wait_heartbeat(timeout=20), 'No heartbeat'
            def receive(kind, timeout=5):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    msg = link.recv_match(blocking=True, timeout=0.2)
                    if msg is None:
                        continue
                    if msg.get_type() == 'STATUSTEXT':
                        messages.append(msg.text)
                    if msg.get_type() == 'HEARTBEAT':
                        assert not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    if msg.get_type() == kind:
                        return msg
                raise TimeoutError(kind)
            def get_param(name):
                link.mav.param_request_read_send(1, 1, name.encode(), -1)
                end = time.monotonic() + 5
                while time.monotonic() < end:
                    msg = receive('PARAM_VALUE')
                    if msg.param_id == name:
                        return msg.param_value
                raise TimeoutError(name)
            for name, value in expected.items():
                if name in hardware_only:
                    continue
                actual = get_param(name)
                assert abs(actual - value) < max(1e-5, abs(value)*1e-6), (name, value, actual)
            # Let the embedded script initialise, then ask ArduPilot to save its checksums.
            time.sleep(5)
            link.mav.param_set_send(1, 1, b'SCR_DEBUG_OPTS', 32, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(3)
            loaded = get_param('SCR_LD_CHECKSUM')
            running = get_param('SCR_RUN_CHECKSUM')
            assert loaded > 0 and loaded == running, (loaded, running)
            assert loaded == expected['SCR_LD_CHECKSUM']
            assert running == expected['SCR_RUN_CHECKSUM']
            # Check mode selection only. A stock quad plant cannot validate our eight-motor flight.
            for mode in (2, 9):
                link.set_mode(mode)
                end = time.monotonic() + 8
                while True:
                    msg = receive('HEARTBEAT')
                    if msg.custom_mode == mode:
                        break
                    assert time.monotonic() < end, ('mode rejected', mode)
            errors = [s for s in messages if 'Lua:' in s and any(w in s.lower() for w in ('error', 'failed', 'panic'))]
            assert not errors, errors
            report = {'result': 'PASS', 'parameters_checked': len(set(expected)-hardware_only),
                      'hardware_only_parameters_not_checked': sorted(set(expected)&hardware_only),
                      'loaded_script_checksum': loaded, 'running_script_checksum': running,
                      'modes_selected_disarmed': ['ALT_HOLD', 'LAND'],
                      'scope': 'SITL boot only; no arming, motor dynamics, failsafe landing or hardware validation',
                      'status_messages': messages}
            (root / 'logs/boot-result.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2))
        finally:
            if link:
                link.close()
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
