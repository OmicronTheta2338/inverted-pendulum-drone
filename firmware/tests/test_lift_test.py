"""Software checks only; no FC connection or motor commands to real hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lift_test
from test_bench_test import FakeClock, Link
import bench_test
import io


class Tests(unittest.TestCase):
    def test_input_limits_and_step_rate(self):
        c = lift_test.Controls(0)
        c.update('UP', 0)
        self.assertEqual(c.throttle, 1050)
        c.update('UP', .1)
        self.assertEqual(c.throttle, 1050)
        for i in range(1, 30): c.update('UP', i*.41)
        self.assertEqual(c.throttle, 1500)

    def test_setting_persists_without_operator_input(self):
        c = lift_test.Controls(0)
        c.update('UP', 0)
        for now in (2, 5, 12, 60, 3600):
            self.assertIsNone(c.update(None, now))
            self.assertEqual(c.throttle, 1050)

    def test_no_start_timeout(self):
        c = lift_test.Controls(0)
        self.assertIsNone(c.update(None, 60))
        self.assertEqual(c.throttle, 1000)

    def test_stop_key(self):
        c = lift_test.Controls(0)
        c.update('UP', 0)
        self.assertIn('Stop key', c.update('STOP', .1))

    def test_arrow_decoding_and_split_sequences(self):
        decoder = lift_test.KeyDecoder()
        self.assertIsNone(decoder.feed(b'\x1b'))
        self.assertIsNone(decoder.feed(b'['))
        self.assertEqual(decoder.feed(b'A'), 'UP')
        self.assertEqual(decoder.feed(b'\x1b[B'), 'DOWN')
        self.assertEqual(decoder.feed(b'\x1bOA'), 'UP')
        self.assertEqual(decoder.feed(b'\x1bOB'), 'DOWN')

    def test_space_prioritised_over_repeated_arrows_and_eof_stops(self):
        decoder = lift_test.KeyDecoder()
        self.assertEqual(decoder.feed(b'\x1b[A\x1b[A \x1b[B'), 'STOP')
        self.assertEqual(decoder.feed(b''), 'STOP')

    def test_unrelated_keys_do_not_change_throttle(self):
        self.assertIsNone(lift_test.KeyDecoder().feed(b'hH+-x\n'))

    def test_lowering_throttle_cannot_go_below_minimum(self):
        c = lift_test.Controls(0)
        for i in range(5): c.update('DOWN', i*.5)
        self.assertEqual(c.throttle, 1000)

    def setup_link(self):
        clock = FakeClock()
        link = Link(clock)
        bench = bench_test.Bench(link, io.StringIO(), clock.monotonic, clock.sleep)
        bench.hold(.1)
        return clock, link, bench

    def test_stop_key_disarms_and_clears_override(self):
        clock, link, bench = self.setup_link()
        calls = 0
        def key():
            nonlocal calls
            calls += 1
            return 'UP' if calls == 2 else ('STOP' if calls == 500 else None)
        with patch('builtins.print'):
            lift_test.execute(bench, lambda prompt:'RUN', key)
        self.assertTrue(any(c[0]=='rc' and c[2][2]==1050 for c in link.calls))
        self.assertFalse(link.is_armed)
        self.assertEqual(link.channels, (0, 0, 0, 0))
        active = [c[1] for c in link.calls if c[0]=='rc' and c[2][2]==1050]
        self.assertGreater(active[-1]-active[0], 12)

    def test_interrupt_runs_disarm_cleanup(self):
        clock, link, bench = self.setup_link()
        calls = 0
        def key():
            nonlocal calls
            calls += 1
            if calls >= 3: raise KeyboardInterrupt()
            return 'UP' if calls == 2 else None
        with patch('builtins.print'), self.assertRaises(KeyboardInterrupt):
            lift_test.execute(bench, lambda prompt:'RUN', key)
        self.assertFalse(link.is_armed)

    def test_no_confirmation_does_not_arm(self):
        clock, link, bench = self.setup_link()
        with patch('builtins.print'):
            lift_test.execute(bench, lambda prompt:'', lambda:None)
        self.assertFalse(any(c[0]=='command' for c in link.calls))


if __name__ == '__main__': unittest.main()
