from pathlib import Path
import sys,math,json,subprocess,tempfile,unittest
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from test_cartesian_nc import fixture
from cartesian_nc_model import NCConfig
from cartesian_nc_drawing import BKSettings,compile_bk
from cartesian_nc_path import PathUpload,CAPACITY,PathSegment,smooth_square
from cartesian_nc_protocol import Ack

ROOT=Path(__file__).resolve().parents[2]

class PathTests(unittest.TestCase):
    def setUp(self): self.model=NCConfig().model(fixture())
    def test_whole_stroke_profiles_keep_exact_checked_positions_and_paper_z(self):
        for pattern,size in (("BK",20),("LINH",40),("THOA",40)):
            p=compile_bk(self.model,(35200,0,1800),BKSettings(pattern=pattern,size_mm=size,paper_z=10))
            self.assertEqual(len(p.runs),len(p.strokes))
            for run in p.runs:
                self.assertEqual(len(run.segments),run.end-run.first)
                start=run.start_steps
                for seg in run.segments:
                    self.assertEqual(seg.steps[0],16000)
                    self.assertEqual(seg.ticks,max(abs(a-b) for a,b in zip(start,seg.steps)))
                    self.assertTrue(50<=seg.entry<=seg.peak<=1600 and 50<=seg.exit<=seg.peak)
                    self.assertLessEqual(abs(seg.entry**2-seg.exit**2),2*seg.accel*seg.ticks)
                    self.assertLessEqual(seg.ramp_up+seg.ramp_down,seg.ticks)
                    start=seg.steps
                self.assertEqual(run.segments[0].entry,50); self.assertEqual(run.segments[-1].exit,50)
            self.assertTrue(any(s.exit>50 for r in p.runs for s in r.segments))
            # K's 90-degree vertex decelerates, without changing either line.
            if pattern=="BK":
                run=p.runs[-1]
                vertex=next(i for i in range(run.first+1,run.end) if p.operations[i].part!=p.operations[i-1].part)
                self.assertEqual(run.segments[vertex-run.first-1].exit,50)
                self.assertEqual(run.segments[vertex-run.first].entry,50)
    def test_credit_accounting_never_overfills_with_delayed_status(self):
        program=compile_bk(self.model,(35200,0,1800),BKSettings(paper_z=10))
        segments=program.runs[0].segments; self.assertGreater(len(segments),32)
        upload=PathUpload(7,1,segments)
        for i in range(CAPACITY):
            self.assertTrue(upload.next().startswith(f"SEG 7 1 {i} "))
            upload.ack(Ack(7,1,"SEG",1,"ok"))
        self.assertEqual(upload.next(),"GO 7 1"); upload.ack(Ack(7,1,"GO",1,"ok"))
        self.assertIsNone(upload.next())
        upload.status(replace(fixture(),job=1,path=(len(segments),32,8,8)))
        for i in range(8):
            self.assertIsNotNone(upload.next()); upload.ack(Ack(7,1,"SEG",1,"ok"))
        self.assertIsNone(upload.next())
        # A late status with lower credit is conservative; a wrong job is ignored.
        upload.status(replace(fixture(),job=2,path=(0,0,0,32)))
        self.assertIsNone(upload.next())
    def test_rejected_upload_and_wrong_job_ack(self):
        p=compile_bk(self.model,(35200,0,1800),BKSettings(paper_z=10))
        upload=PathUpload(7,1,p.runs[0].segments); upload.next()
        self.assertFalse(upload.ack(Ack(7,2,"SEG",1,"ok")))
        self.assertIsNone(upload.next())
        with self.assertRaises(ValueError): upload.ack(Ack(7,1,"SEG",0,"path_profile"))
    def test_actual_gpio_isr_continuous_ink_speed_and_exact_pulse_counts(self):
        executable=ROOT/"Firmware/ScaraCartesian/build_host/test_motor_io.exe"
        rows=[]
        base=executable.parent
        with tempfile.TemporaryDirectory(dir=base) as temp:
            for pattern,size in (("BK",20),("LINH",40),("THOA",40)):
                p=compile_bk(self.model,(35200,0,1800),BKSettings(pattern=pattern,size_mm=size,paper_z=10))
                old=new=0; all_pulses=[0,0,0]
                for index,run in enumerate(p.runs):
                    path=Path(temp)/f"{pattern}_{index}.txt"
                    path.write_text(" ".join(map(str,run.start_steps))+"\n"+"\n".join(
                        " ".join(map(str,(*s.steps,s.peak,s.entry,s.exit,s.accel))) for s in run.segments)+"\n",encoding="ascii")
                    result=json.loads(subprocess.check_output([str(executable),"--stroke-file",str(path)]))
                    self.assertEqual(result["timer_starts"],1)
                    self.assertEqual(result["rises"][0],0)
                    self.assertLess(result["continuous_seconds"],result["old_seconds"])
                    self.assertAlmostEqual(result["continuous_seconds"],run.seconds+.05,delta=.03)
                    old+=result["old_seconds"]; new+=result["continuous_seconds"]
                    all_pulses=[a+b for a,b in zip(all_pulses,result["rises"])]
                self.assertLess(new,old*.65) # quintic trades some time for zero boundary acceleration/jerk
                rows.append(dict(pattern=pattern,size_mm=size,strokes=len(p.runs),segments=sum(len(r.segments) for r in p.runs),
                    previous_ink_seconds=old,continuous_ink_seconds=new,speedup=old/new,rises=all_pulses,
                    note="Virtual actual GPIO/timer ISR; excludes pen lifts/travel, host/USB upload time and hardware mechanics"))
        (ROOT/"Software/reports/verification_path.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8")
        print("R12 ink timing:",json.dumps(rows))

    def test_quintic_profile_has_flat_boundaries_and_respects_acceleration(self):
        seg=PathSegment((0,1000,1000),1000,50,50,2000,1000)
        self.assertLessEqual(seg.ramp_up+seg.ramp_down,seg.ticks)
        square=[smooth_square(seg.entry,seg.peak,i,seg.ramp_up) for i in range(seg.ramp_up+1)]
        self.assertEqual(square[0],2500); self.assertEqual(square[-1],1000000)
        self.assertTrue(all(a<=b for a,b in zip(square,square[1:])))
        self.assertLess(square[1]-square[0],10)
        self.assertLess(square[-1]-square[-2],10)
        self.assertLessEqual(max(b-a for a,b in zip(square,square[1:])),2*seg.accel*1.04)
        for i,value in enumerate(square):
            u=i/seg.ramp_up; expected=2500+997500*(6*u**5-15*u**4+10*u**3)
            self.assertLess(abs(value-expected),400)

if __name__=="__main__": unittest.main()
