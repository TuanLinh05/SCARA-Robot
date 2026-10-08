import copy
import json
import math
from pathlib import Path
import random
import sys
import unittest

PROJECT = Path(__file__).resolve().parents[1]
SIMULATION = PROJECT.parent
sys.path.insert(0, str(PROJECT/'controllers/scara_draw'))
from core.model import fk, ik, motor_coordinates
from core.control import EffortPID, ink_allowed
from core.trajectory import build_reference


class AlgorithmTests(unittest.TestCase):
    def setUp(self):
        self.p = json.loads((SIMULATION/'common/config/scara.json').read_text(encoding='utf-8'))
        self.settings = json.loads((PROJECT/'config/demo.json').read_text(encoding='utf-8'))

    def test_fk_ik_and_tool_offset(self):
        rng = random.Random(41)
        for offset in ([0,0], [.005,-.002]):
            p = copy.deepcopy(self.p)
            p['toolOffset'] = offset
            for _ in range(150):
                q = [math.radians(rng.uniform(-90,70)), math.radians(rng.uniform(15,160)), rng.uniform(0,.01)]
                recovered = ik(fk(q,p),p)
                self.assertLess(math.dist(q,recovered),1e-10)

    def test_invalid_workspace_singularity_limits(self):
        for xyz in ([1,1,0],[*self.p['base'],0], [0,self.p['base'][1]+self.p['L1']+self.p['L2'],0], [0,0,.2]):
            with self.assertRaises(ValueError):
                ik(xyz,self.p)
        for xyz in ([math.nan,0,0],[0,math.inf,0]):
            with self.assertRaises(ValueError):
                ik(xyz,self.p)
        q = [-.8,-1.3,.01]
        self.assertLess(math.dist(ik(fk(q,self.p),self.p,branch=-1,check_limits=False),q),1e-10)

    def test_paths_limits_contact_lift_and_smoothness(self):
        for kind in ('circle','flower','square','csv'):
            ref = build_reference(kind,self.p,csv_path=SIMULATION/'common/trajectories/example_drawing.csv')
            self.assertEqual(len(ref.t),len(ref.q))
            self.assertGreater(sum(ref.pen),100)
            self.assertFalse(ref.pen[0])
            self.assertFalse(ref.pen[-1])
            self.assertAlmostEqual(ref.xyz[-1][2],self.p['penLift'])
            for i in range(0,len(ref.q),7):
                self.assertLess(math.dist(fk(ref.q[i],self.p),ref.xyz[i]),1e-10)
            self.assertLess(max(abs(v) for row in ref.dq for v in row[:2]),3)
            self.assertLess(max(abs(row[2]) for row in ref.dq),.05)
            for row in (ref.sample(-1)[0], ref.sample(ref.t[-1]+1)[0]):
                self.assertTrue(all(math.isfinite(v) for v in row))

    def test_csv_pen_up_segment_and_invalid_input(self):
        ref = build_reference('csv',self.p,csv_path=PROJECT/'tests/fixtures/pen_lift.csv')
        # Entire first horizontal segment is pen-up, even after interpolation.
        for t in [.5,.8,1.2]:
            self.assertFalse(ref.sample(t)[4])
        with self.assertRaises(ValueError):
            build_reference('csv',self.p,csv_path=PROJECT/'tests/fixtures/invalid_pen.csv')

    def test_pid_saturation_antiwindup_and_gravity(self):
        pid = EffortPID(self.p,False)
        q = [-.8,1.3,.006]
        held = pid.command(q,[0]*3,q,[0]*3,[0]*3,.002)
        self.assertEqual(held[:2],[0,0])
        self.assertAlmostEqual(held[2],self.p['zMass']*self.p['gravity'])
        for _ in range(100):
            out = pid.command(q,[0]*3,[10,10,1],[0]*3,[0]*3,.002)
            self.assertEqual(out,self.p['actuatorLimits'])
        self.assertEqual(pid.integral,[0,0,0])
        with self.assertRaises(ValueError):
            pid.command(q,[math.nan]*3,q,[0]*3,[0]*3,.002)

    def test_ink_contact_gate_and_crosstalk(self):
        self.assertTrue(ink_allowed(True,[0,0,0],.48,self.p,self.settings))
        for requested, xyz, force in [(False,[0,0,0],.48),(True,[0,0,.006],.48),
                                       (True,[.06,0,0],.48),(True,[0,0,0],0)]:
            self.assertFalse(ink_allowed(requested,xyz,force,self.p,self.settings))
        axes, steps = motor_coordinates([.6,1.2,.01],self.p)
        self.assertAlmostEqual(axes[1],1.4)
        self.assertEqual(steps[2],4000)


if __name__ == '__main__':
    unittest.main()
