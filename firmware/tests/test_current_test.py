import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from current_test import scan, median_since, CurrentBench

class Fake:
    def __init__(self, reject=False, stop_ok=True):
        self.t=0; self.current_samples=[]; self.latest={}; self.log=io.StringIO()
        self.commands=[]; self.running=False; self.reject=reject; self.stop_ok=stop_ok; self.stops=0
    def check(self,*a): pass
    def parameter(self,k): return {'BATT_MONITOR':4,'BATT_CURR_PIN':11}[k]
    def armed(self): return False
    def clock(self): return self.t
    def hold(self,s):
        for _ in range(round(s*10)):
            self.t+=.1; self.current_samples.append((self.t,2 if self.running else .2))
    def motor(self,n,pwm=1150,seconds=3):
        self.commands.append((n,pwm,seconds)); self.running=seconds>0
        self.latest['COMMAND_ACK']=(self.t,SimpleNamespace(command=209,result=4 if self.reject else 0))
    def fresh(self,*a,**k): pass
    def stop(self): self.stops+=1; self.running=False; return self.stop_ok

class Tests(unittest.TestCase):
    @patch('builtins.input',return_value='RUN')
    def test_sequence(self,_):
        f=Fake(); result=scan(f)
        self.assertEqual([c for c in f.commands if c[2]],[(i,1150,3) for i in range(1,9)])
        self.assertEqual(f.stops,8)
        self.assertEqual(len(result),8)
        self.assertAlmostEqual(result[0]['delta_uncalibrated_A'],1.8)
    @patch('builtins.input',return_value='')
    def test_cancel(self,_):
        f=Fake(); self.assertEqual(scan(f),[]); self.assertEqual(f.commands,[])
    @patch('builtins.input',return_value='RUN')
    def test_reject_cleans_up(self,_):
        f=Fake(reject=True)
        with self.assertRaisesRegex(RuntimeError,'not accepted'): scan(f)
        self.assertEqual(f.commands,[(1,1150,3),(1,1000,0)])
        self.assertEqual(f.stops,1)
    @patch('builtins.input',return_value='RUN')
    def test_failed_stop_prevents_next_motor(self,_):
        f=Fake(stop_ok=False)
        with self.assertRaises(RuntimeError): scan(f)
        self.assertEqual(len([c for c in f.commands if c[2]]),1)
    @patch('builtins.input',return_value='RUN')
    def test_interruption_cleanup(self,_):
        f=Fake(); original=f.hold
        def hold(s):
            if f.running: raise KeyboardInterrupt()
            original(s)
        f.hold=hold
        with self.assertRaises(KeyboardInterrupt): scan(f)
        self.assertEqual(f.commands[-1],(1,1000,0)); self.assertEqual(f.stops,1)
    def test_missing_current(self):
        with self.assertRaises(RuntimeError): median_since([],0)
    def test_motor_packet(self):
        from unittest.mock import Mock
        link=Mock(target_system=1,target_component=1)
        bench=CurrentBench(link,io.StringIO()); bench.motor(5)
        link.mav.command_long_send.assert_called_once_with(1,1,209,0,5,1,1150,3,1,0,0)

if __name__=='__main__': unittest.main()
