"""Test timing and cleanup with a fake link; never access flight hardware."""
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('bench_test', Path(__file__).resolve().parents[1]/'bench_test.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeClock:
    now = 0
    interrupt_at = None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.interrupt_at is not None and self.now >= self.interrupt_at:
            self.interrupt_at = None
            raise KeyboardInterrupt()


class Message(SimpleNamespace):
    def get_srcSystem(self): return 1
    def get_srcComponent(self): return getattr(self, 'component', 1)
    def get_type(self): return self.kind
    def to_dict(self): return vars(self)


class Link:
    target_system, target_component = 1, 1

    def __init__(self, clock):
        self.clock = clock
        self.mav = self
        self.is_armed = False
        self.reject = False
        self.mode = 0
        self.calls = []
        self.channels = module.NEUTRAL
        self.pending = []
        self.last_sample = -1
        self.silent = False
        self.simulate_failsafes = False
        self.last_rc = self.last_gcs = clock.now
        self.radio_reported = self.gcs_reported = False

    def heartbeat_send(self, *args):
        self.last_gcs = self.clock.now
        self.calls.append(('heartbeat', self.clock.now))

    def rc_channels_override_send(self, sysid, component, *channels):
        self.calls.append(('rc', self.clock.now, channels))
        self.channels = channels[:4]
        self.last_rc = self.last_gcs = self.clock.now

    def param_request_read_send(self, sysid, component, name, index):
        key = name.decode()
        self.pending.append(Message(kind='PARAM_VALUE', param_id=key, param_value=module.REQUIRED[key]))

    def command_long_send(self, sysid, component, command, confirmation, *params):
        self.calls.append(('command', self.clock.now, command, params))
        assert command == 400
        assert params[1] == 0  # no forced arm/disarm magic value
        if not (params[0] and self.reject): self.is_armed = bool(params[0])
        self.pending.append(Message(kind='COMMAND_ACK', command=400, result=2 if params[0] and self.reject else 0))

    def recv_match(self, blocking=False):
        if self.silent: return None
        if self.pending: return self.pending.pop(0)
        if self.last_sample == self.clock.now: return None
        self.last_sample = self.clock.now
        if self.simulate_failsafes:
            if self.is_armed and self.clock.now-self.last_rc > 1.5 and not self.radio_reported:
                self.is_armed = False
                self.radio_reported = True
                self.pending.append(Message(kind='STATUSTEXT', text='Radio Failsafe - Disarming'))
            if self.clock.now-self.last_gcs > 2 and not self.gcs_reported:
                self.gcs_reported = True
                self.pending.append(Message(kind='STATUSTEXT', text='GCS Failsafe'))
        running = self.is_armed and self.channels[2] > 1000
        self.pending += [Message(kind='ATTITUDE', roll=0, pitch=0),
                        Message(kind='SYS_STATUS', voltage_battery=22700),
                        Message(kind='SERVO_OUTPUT_RAW', **{f'servo{i}_raw': 1200 if running else 1000 for i in range(1, 9)})]
        return Message(kind='HEARTBEAT', base_mode=128 if self.is_armed else 0, custom_mode=self.mode)


class Tests(unittest.TestCase):
    def setup_link(self):
        clock = FakeClock()
        link = Link(clock)
        bench = module.Bench(link, io.StringIO(), clock.monotonic, clock.sleep)
        bench.hold(0.1)
        return clock, link, bench

    def run_test(self, bench, test='right'):
        with patch('builtins.input', return_value='RUN'), patch('builtins.print'):
            bench.run(test)

    def test_two_second_limit_and_stop(self):
        clock, link, bench = self.setup_link()
        self.run_test(bench)
        active = [c[1] for c in link.calls if c[0]=='rc' and c[2][2]==1050]
        stop = next(c[1] for c in link.calls if c[0]=='rc' and c[1]>active[-1] and c[2][2]==1000)
        self.assertLessEqual(stop-active[0], 2.051)
        self.assertFalse(link.is_armed)
        self.assertEqual(link.channels, (0, 0, 0, 0))

    def test_interruption_disarms(self):
        clock, link, bench = self.setup_link()
        original = bench.hold
        def interrupted(seconds, channels=None, require_armed=False):
            if require_armed:
                clock.interrupt_at = clock.now+0.2
            return original(seconds, channels, require_armed)
        bench.hold = interrupted
        with self.assertRaises(KeyboardInterrupt): self.run_test(bench)
        self.assertFalse(link.is_armed)

    def test_arming_rejected_never_sends_running_throttle(self):
        clock, link, bench = self.setup_link()
        link.reject = True
        with self.assertRaisesRegex(RuntimeError, 'Arming rejected'): self.run_test(bench)
        self.assertFalse(any(c[0]=='rc' and c[2][2]==1050 for c in link.calls))

    def test_already_armed_is_not_taken_over(self):
        clock, link, bench = self.setup_link()
        link.is_armed = True
        bench.hold(0.1)
        with self.assertRaisesRegex(RuntimeError, 'Already armed'): self.run_test(bench)
        self.assertFalse(any(c[0]=='command' for c in link.calls))

    def test_lost_telemetry_stops_test(self):
        clock, link, bench = self.setup_link()
        original = bench.hold
        def lost(seconds, channels=None, require_armed=False):
            if require_armed:
                link.silent = True
                bench.latest['HEARTBEAT'] = (-100, bench.latest['HEARTBEAT'][1])
            return original(seconds, channels, require_armed)
        bench.hold = lost
        with self.assertRaisesRegex(RuntimeError, 'Motor stop could not be verified'):
            self.run_test(bench)
        self.assertFalse(link.is_armed)  # disarm was sent; silence prevents confirmation
        self.assertFalse(any(c[0]=='rc' and c[2][2]==1050 for c in link.calls))

    def test_prompt_cancel_sends_no_arm_or_override(self):
        clock, link, bench = self.setup_link()
        with patch('builtins.input', return_value=''), patch('builtins.print'):
            bench.run('right')
        self.assertFalse(any(c[0] in ('command', 'rc') for c in link.calls))

    def test_mode_change_stops_without_running_throttle(self):
        clock, link, bench = self.setup_link()
        original = bench.hold
        def change_mode(seconds, channels=None, require_armed=False):
            if require_armed:
                link.mode = 9
            return original(seconds, channels, require_armed)
        bench.hold = change_mode
        with self.assertRaisesRegex(RuntimeError, 'left STABILIZE'): self.run_test(bench)
        self.assertFalse(link.is_armed)
        self.assertFalse(any(c[0]=='rc' and c[2][2]==1050 for c in link.calls))

    def test_parameter_mismatch_prevents_arm(self):
        clock, link, bench = self.setup_link()
        original = link.param_request_read_send
        def mismatch(sysid, component, name, index):
            original(sysid, component, name, index)
            if name == b'ARMING_SKIPCHK': link.pending[-1].param_value = 1
        link.param_request_read_send = mismatch
        with self.assertRaisesRegex(RuntimeError, 'Mismatched parameters'): self.run_test(bench)
        self.assertFalse(any(c[0] in ('command', 'rc') for c in link.calls))

    def test_dropped_parameter_replies_are_retried(self):
        clock, link, bench = self.setup_link()
        original = link.param_request_read_send
        counts = {}
        def dropped(sysid, component, name, index):
            counts[name] = counts.get(name, 0)+1
            if counts[name] % 2 == 0:
                original(sysid, component, name, index)
        link.param_request_read_send = dropped
        # Test only a few keys so each first dropped reply fits the total deadline.
        with patch.dict(module.REQUIRED, {'FRAME_CLASS': 16, 'RC3_MIN': 1000}, clear=True), patch('builtins.print'):
            bench.read_parameters()
        self.assertEqual(counts, {b'FRAME_CLASS': 2, b'RC3_MIN': 2})
        self.assertFalse(any(c[0] in ('command', 'rc') for c in link.calls))

    def test_permanently_missing_parameter_prevents_arm(self):
        clock, link, bench = self.setup_link()
        original = link.param_request_read_send
        def missing(sysid, component, name, index):
            if name != b'SERVO8_FUNCTION': original(sysid, component, name, index)
        link.param_request_read_send = missing
        with self.assertRaisesRegex(RuntimeError, 'No parameter replies after retries: SERVO8_FUNCTION'):
            self.run_test(bench)
        self.assertFalse(any(c[0] in ('command', 'rc') for c in link.calls))

    def verify_loss(self, name):
        clock, link, bench = self.setup_link()
        link.simulate_failsafes = True
        self.run_test(bench, name)
        events = [json.loads(line) for line in bench.log.getvalue().splitlines()]
        start = next(e['received_monotonic'] for e in events if e.get('event')=='loss_start')
        result = next(e for e in events if e.get('event')=='loss_result')
        self.assertTrue(result['passed'])
        during = [c for c in link.calls if start<=c[1]<result['received_monotonic']]
        self.assertFalse(any(c[0] in ('rc', 'command') for c in during))
        self.assertEqual(any(c[0]=='heartbeat' for c in during), name=='rc-loss')
        self.assertFalse(any(c[0]=='rc' and c[2][2]>1000 for c in link.calls))
        self.assertFalse(link.is_armed)

    def test_rc_loss_does_not_disarm_for_the_fc(self):
        self.verify_loss('rc-loss')

    def test_complete_link_loss_sends_nothing_in_observation_window(self):
        self.verify_loss('link-loss')

    def test_missing_failsafe_fails_and_runs_cleanup(self):
        clock, link, bench = self.setup_link()
        with self.assertRaisesRegex(RuntimeError, 'Failsafe test not verified'):
            self.run_test(bench, 'rc-loss')
        self.assertFalse(link.is_armed)

    def test_bridge_heartbeat_does_not_override_fc_armed_state(self):
        clock, link, bench = self.setup_link()
        link.pending.append(Message(kind='HEARTBEAT', component=68, base_mode=128, custom_mode=99))
        bench.poll()
        self.assertFalse(bench.armed())
        self.assertEqual(bench.fresh('HEARTBEAT').custom_mode, 0)

    def test_connection_selects_autopilot_not_bridge(self):
        replies = iter([Message(kind='HEARTBEAT', component=68, autopilot=8),
                        Message(kind='HEARTBEAT', component=1, autopilot=3)])
        link = SimpleNamespace(recv_match=lambda **kwargs: next(replies))
        module.select_autopilot(link)
        self.assertEqual((link.target_system, link.target_component), (1, 1))


if __name__ == '__main__': unittest.main()
