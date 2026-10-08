"""Independent oracle from the existing MATLAB implementation."""
import json
import math
from pathlib import Path
import sys
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'controllers/scara_draw'))
from core.model import fk, ik
from core.control import EffortPID
from core.trajectory import build_reference


class MatlabInteropTests(unittest.TestCase):
    def setUp(self):
        self.p = json.loads((PROJECT.parent/'common/config/scara.json').read_text(encoding='utf-8'))
        self.oracle = json.loads((PROJECT/'tests/fixtures/matlab_oracle.json').read_text(encoding='utf-8'))

    def test_matlab_fk_ik_and_effort_pid(self):
        for case in self.oracle['cases']:
            self.assertLess(math.dist(fk(case['q'],self.p),case['xyz']),1e-12)
            self.assertLess(math.dist(ik(case['xyz'],self.p),case['q']),1e-10)
            result = EffortPID(self.p,True).command(case['q'],case['dq'],case['q'],case['dq'],case['ddq'],self.p['dt'])
            self.assertLess(math.dist(result,case['command']),1e-11)

    def test_matlab_trajectory_samples_and_durations(self):
        for path in self.oracle['paths']:
            ref = build_reference(path['name'],self.p,csv_path=PROJECT.parent/'common/trajectories/example_drawing.csv')
            self.assertEqual(len(ref.t),path['sampleCount'])
            self.assertAlmostEqual(ref.t[-1],path['duration'],places=10)
            for i,time in enumerate(path['t']):
                qr,_,_,xyz,_ = ref.sample(time)
                self.assertLess(math.dist(qr,path['q'][i]),1e-10)
                self.assertLess(math.dist(xyz,path['xyz'][i]),1e-10)


if __name__ == '__main__':
    unittest.main()
