import math
from pathlib import Path
import sys
import tempfile
import unittest
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from test_cartesian_nc import fixture
from cartesian_nc_model import NCConfig
from cartesian_nc_workspace import Viewport, Setpoint, bounded_pose, pose_ranges, test_points, load_points, save_points, validate_program

class WorkspaceTests(unittest.TestCase):
    def setUp(self): self.model=NCConfig().model(fixture())
    def test_coordinate_axes_and_inverse(self):
        for size in ((800,400),(1200,700),(500,250)):
            view=Viewport(*size,196)
            self.assertEqual(view.point(0,0),(size[0]/2,size[1]/2))
            self.assertGreater(view.point(10,0)[0],size[0]/2)
            self.assertLess(view.point(0,10)[1],size[1]/2)
            for p in ((-100,150),(25,-25),(0,0)):
                for a,b in zip(view.world(*view.point(*p)),p): self.assertAlmostEqual(a,b)
    def test_interactive_step_is_bounded_and_converges(self):
        q=(0,45,10); target=(-60,70,20)
        for _ in range(100):
            next_q=bounded_pose(q,target)
            for a,b,limit in zip(q,next_q,(2,2,.5)): self.assertLessEqual(abs(a-b),limit+1e-10)
            self.model.check_pose(next_q); q=next_q
        self.assertEqual(q,target)
    def test_nine_interior_points_keep_current_z(self):
        points=test_points(self.model,22)
        self.assertEqual(len(points),9)
        self.assertEqual({p.x for p in points},{-25,0,25})
        self.assertEqual({p.y for p in points},{155,170,180})
        self.assertEqual({p.z for p in points},{22})
        plans=validate_program(self.model,(35200,0,1800),points)
        self.assertEqual(len(plans),9)
        for plan in plans: self.assertEqual(plan.steps[0],35200)
        with self.assertRaises(ValueError): self.model.ik((0,0,22))
    def test_elbow_branch_retained(self):
        negative=replace(NCConfig(),park_deg=(0,-45)).model(replace(fixture(),pos=(35200,0,-1800)))
        for model in (self.model,negative):
            limits=pose_ranges(model)
            self.assertGreater(limits[1][0]*model.branch if model.branch>0 else limits[1][1]*model.branch,0)
            for point in test_points(model,10): self.assertGreater(model.ik(point.xyz)[1]*model.branch,0)
    def test_whole_program_rejects_bad_later_target(self):
        points=[Setpoint("valid",*self.model.fk((0,45,22))),Setpoint("bad",0,0,22)]
        with self.assertRaisesRegex(ValueError,"Điểm2"):
            validate_program(self.model,(35200,0,1800),points)
        with self.assertRaises(ValueError): validate_program(self.model,(35200,0,1800),[])
        with self.assertRaisesRegex(ValueError,"bị hủy"):
            validate_program(self.model,(35200,0,1800),points,lambda:True)
    def test_csv_roundtrip_and_invalid_data(self):
        root=Path(__file__).resolve().parents[2]/"Firmware/ScaraCartesian/build_host"
        with tempfile.TemporaryDirectory(dir=root) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(root.resolve()))
            path=Path(temp)/"points.csv"; points=test_points(self.model,22)
            save_points(path,points); self.assertEqual(load_points(path),points)
            path.write_text("name,x,y,z,speed_mm_s,dwell_s\nbad,nan,170,10,5,0\n",encoding="utf-8")
            with self.assertRaises(ValueError): load_points(path)
            path.write_text("X,Y,Z\n0,170,10\n",encoding="utf-8")
            with self.assertRaises(ValueError): load_points(path)
        for p in (Setpoint("",0,170,10),Setpoint("x",0,170,10,-1),Setpoint("x",0,170,10,5,61)):
            with self.assertRaises(ValueError): p.validate()

if __name__=="__main__": unittest.main()
