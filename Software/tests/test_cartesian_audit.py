from pathlib import Path
import sys,json,tempfile,unittest
from dataclasses import asdict,replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from test_cartesian_nc import fixture
from cartesian_nc_model import NCConfig
from cartesian_nc_audit import audit_log,markdown_report

class AuditTests(unittest.TestCase):
    def test_endpoint_and_count_mismatch_are_reported_not_hidden(self):
        state=replace(fixture(),job=2,pos=(32000,0,1800))
        events=[dict(type="connection",demo=False,config=asdict(NCConfig())),
            dict(type="status",status=asdict(state)),
            dict(type="drawing_start",dry=False,settings=dict(size_mm=20,center_x=0,center_y=165,
                paper_z=20,lift_mm=2,draw_speed=12,travel_speed=12,step_mm=.75,tolerance_mm=.25,pattern="STAR",settle_s=.12)),
            dict(type="stroke_start",job=3,profiles=[dict(steps=[32000,2,1804],ticks=4,peak=50,entry=50,exit=50,accel=2000)]),
            dict(type="status",status=asdict(replace(state,job=3,pos=(32000,2,1805),total=(80000,7202,28805),timer_late=1))),
            dict(type="stroke_complete",job=3,pos=[32000,2,1805],timer_late=1,starved=0,elapsed_s=.1,estimated_s=.08)]
        base=Path(__file__).resolve().parents[2]/"Firmware/ScaraCartesian/build_host"
        with tempfile.TemporaryDirectory(dir=base) as temp:
            path=Path(temp)/"fixture.jsonl"; path.write_text("\n".join(map(json.dumps,events))+"\n",encoding="utf-8")
            result=audit_log(path)
        stroke=result["drawings"][0]["strokes"][0]
        self.assertEqual(stroke["step_endpoint_error"],[0,0,1])
        self.assertEqual(stroke["axis_pulse_error"],[0,0,1]); self.assertEqual(stroke["timer_late_delta"],1)
        self.assertTrue(stroke["constant_z"]); self.assertFalse(stroke["commanded_closed"])
        self.assertIn("0/0",markdown_report(result))

if __name__=="__main__": unittest.main()
