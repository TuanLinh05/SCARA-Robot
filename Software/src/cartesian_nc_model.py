"""NC-switch configuration and measured joint model, angles relative at elbow."""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math
from cartesian_model import RobotConfig

@dataclass(frozen=True)
class NCConfig:
    l1_mm: float = 98.0
    l2_mm: float = 98.0
    tool_offset_mm: tuple = (0.0,0.0)  # local +X: right, +Y: along arm 2
    lead_mm_rev: float = 2.0
    z_span_mm: float = 0.0  # 0: unmeasured, infer approximate mm from lead
    joint_limits_deg: tuple = ((-90.0, 90.0), (-90.0, 90.0))
    park_deg: tuple = (0.0, 45.0)
    z_clear_mm: float = 3.0
    microsteps: tuple = (16, 8, 8)
    ratios: tuple = (3.0, 9.0)
    positive_high: tuple = (True, True, False)  # Z, J1, J2; opposite physical arm motor signs
    coupling: float = 1.333333  # absolute passive arm-2 heading=-A/3; elbow=B-4A/3
    auto_coupling: bool = True
    coupling_probe_pulses: int = 128
    arm_home_pps: int = 200
    j2_home_pps: int = 800
    j2_latch_pps: int = 200
    z_home_speed_mm_s: float = 3.0
    z_latch_pps: int = 400
    home_in_background: bool = True
    motion_in_background: bool = True
    arm_scan_pulses: int = 40000
    z_scan_pulses: int = 300000
    arm_backoff_pulses: int = 512
    z_backoff_pulses: int = 1024
    max_master_rate: int = 1600
    max_joint_speed_deg_s: tuple = (15.0, 10.0)
    max_z_speed_mm_s: float = 1.0

    @property
    def z_home_pps(self):
        return round(200*self.microsteps[0]*self.z_home_speed_mm_s/self.lead_mm_rev)

    @property
    def reach_mm(self): return self.l1_mm+math.hypot(self.tool_offset_mm[0],self.l2_mm+self.tool_offset_mm[1])

    def validate(self):
        RobotConfig(l1_mm=self.l1_mm, l2_mm=self.l2_mm, tool_offset_mm=self.tool_offset_mm,home_deg=self.park_deg,
                    joint_limits_deg=self.joint_limits_deg, home_z_mm=0, z_limits_mm=(0, 1000),
                    ratios=(self.lead_mm_rev, *self.ratios), microsteps=self.microsteps,
                    positive_high=self.positive_high, coupling=self.coupling,
                    max_master_rate=self.max_master_rate, max_joint_speed_deg_s=self.max_joint_speed_deg_s,
                    max_z_speed_mm_s=self.max_z_speed_mm_s).validate()
        if abs(self.coupling) > 2: raise ValueError("Hệ số liên động cần trong -2 đến 2.")
        if type(self.auto_coupling) is not bool or type(self.coupling_probe_pulses) is not int or not 16<=self.coupling_probe_pulses<=256:
            raise ValueError("Đo bù tự động cần true/false và bước thử J1 trong 16–256 xung.")
        for lo,hi in self.joint_limits_deg:
            if not -180 <= lo < hi <= 180 or not 10 <= hi-lo <= 360:
                raise ValueError("Giới hạn góc mỗi khớp cần đúng vị trí công tắc; khoảng 10–360°.")
        for p,(lo,hi) in zip(self.park_deg,self.joint_limits_deg):
            if not lo+2 < p < hi-2: raise ValueError("Tư thế đỗ phải cách biên hơn 2°.")
        if not 10 <= abs(self.park_deg[1]) <= 170: raise ValueError("Góc đỗ J2 cần cách tư thế thẳng/gập.")
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (self.z_span_mm,self.z_clear_mm)):
            raise ValueError("Thông số Z cần hữu hạn.")
        if not 0 <= self.z_span_mm <= 2000 or not .1 <= self.z_clear_mm <= 100:
            raise ValueError("Sai hành trình Z hoặc khoảng lùi khỏi biên trên.")
        if type(self.z_home_speed_mm_s) not in (int,float) or not math.isfinite(self.z_home_speed_mm_s) or not .1<=self.z_home_speed_mm_s<=10:
            raise ValueError("Tốc độ calib Z cần 0,1–10 mm/s.")
        if self.z_home_pps>6400: raise ValueError(f"Calib Z cần {self.z_home_pps} xung/s, vượt giới hạn 6400; giảm tốc độ hoặc kiểm tra lead/vi bước.")
        for rate, hi in ((self.arm_home_pps,800),(self.z_home_pps,6400),(self.j2_home_pps,1600)):
            if type(rate) is not int or not 100 <= rate <= hi: raise ValueError("Tốc độ home ngoài phạm vi.")
        if type(self.z_latch_pps) is not int or not 50<=self.z_latch_pps<=self.z_home_pps:
            raise ValueError("Tốc độ chốt biên Z cần 50 xung/s đến tốc độ quét Z.")
        if type(self.j2_latch_pps) is not int or not 50<=self.j2_latch_pps<=self.j2_home_pps:
            raise ValueError("Tốc độ chốt biên J2 cần 50 xung/s đến tốc độ quét J2.")
        if type(self.home_in_background) is not bool: raise ValueError("home_in_background cần true/false.")
        if type(self.motion_in_background) is not bool: raise ValueError("motion_in_background cần true/false.")
        for scan,back in ((self.arm_scan_pulses,self.arm_backoff_pulses),(self.z_scan_pulses,self.z_backoff_pulses)):
            if type(scan) is not int or type(back) is not int or not 256 <= scan <= 1000000 or not 32 <= back <= 8192 or back >= scan:
                raise ValueError("Sai ngân sách tìm biên hoặc lùi công tắc.")
        arm_factors = [200*m*r/360 for m,r in zip(self.microsteps[1:],self.ratios)]
        if any(not 1 <= f <= 976 for f in arm_factors) or abs(self.coupling*arm_factors[1]/arm_factors[0])>64:
            raise ValueError("Tỷ số xung liên động vượt phạm vi firmware.")
        return self

    @classmethod
    def load(cls,path):
        data=json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls,data):
        if isinstance(data,dict):
            data=dict(data)
            data.pop("z_home_pps",None)
            data.setdefault("z_home_speed_mm_s",3.0)
            data.setdefault("tool_offset_mm",[0.0,0.0])
            data.setdefault("z_latch_pps",min(400,data.get("z_home_pps",1600)))
            data.setdefault("home_in_background",True)
            data.setdefault("motion_in_background",True)
            data.setdefault("j2_home_pps",800); data.setdefault("j2_latch_pps",200)
            data.setdefault("auto_coupling",True); data.setdefault("coupling_probe_pulses",128)
        if not isinstance(data,dict) or set(data)!=set(cls.__dataclass_fields__):
            raise ValueError("File cấu hình NC thiếu hoặc thừa trường.")
        return cls(**data).validate()

    def save(self,path):
        self.validate()
        Path(path).write_text(json.dumps(asdict(self),indent=2,ensure_ascii=False)+"\n",encoding="utf-8")

    def commands(self,session):
        self.validate(); s=session
        factors=(200*self.microsteps[0]/self.lead_mm_rev,
                 200*self.microsteps[1]*self.ratios[0]/360,200*self.microsteps[2]*self.ratios[1]/360)
        angles=[round(x*1000) for interval in self.joint_limits_deg for x in interval]
        return [
            f"GEOM {s} {round(self.coupling*1000000)} "+ " ".join(str(round(f*1024)) for f in factors)+
                " "+ " ".join(map(str,self.microsteps)),
            f"SPAN {s} "+ " ".join(map(str,angles))+f" {round(self.z_span_mm*1000)}",
            f"PARK {s} {round(self.park_deg[0]*1000)} {round(self.park_deg[1]*1000)} {round(self.z_clear_mm*1000)}",
            f"TUNE {s} {self.arm_home_pps} {self.z_home_pps} {self.arm_scan_pulses} {self.z_scan_pulses} {self.arm_backoff_pulses} {self.z_backoff_pulses} {self.z_latch_pps} {self.j2_home_pps} {self.j2_latch_pps}",
            *[f"POL {s} {a} {int(p)}" for a,p in zip(("Z","J1","J2"),self.positive_high)],
            f"COUPLE {s} {int(self.auto_coupling)} {self.coupling_probe_pulses}",
        ]

    def model(self,status):
        self.validate()
        if not status.referenced or any(f<=0 for f in status.factor) or any(n<64 for n in status.range):
            raise ValueError("Chưa hoàn thành HOME + CALIB.")
        factors=tuple(f/1024 for f in status.factor)
        upper=status.range[0]/factors[0]
        margin=max(.1,2/factors[0])
        if upper <= 2*margin: raise ValueError("Hành trình Z đo được quá ngắn.")
        delta=math.atan2(-self.tool_offset_mm[0],self.l2_mm+self.tool_offset_mm[1])
        branch=1 if math.sin(math.radians(self.park_deg[1])+delta)>0 else -1
        return MeasuredModel(l1_mm=self.l1_mm,l2_mm=self.l2_mm,tool_offset_mm=tuple(self.tool_offset_mm),
            home_deg=(0.0,0.0), home_z_mm=0,
            joint_limits_deg=tuple((lo+2,hi-2) for lo,hi in self.joint_limits_deg),
            z_limits_mm=(margin,upper-margin), ratios=(self.lead_mm_rev,*self.ratios),
            microsteps=self.microsteps,positive_high=self.positive_high,coupling=status.coupling_ppm/1000000,
            max_master_rate=self.max_master_rate,max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_z_speed_mm_s=self.max_z_speed_mm_s,measured_factors=factors,
            branch=branch)

@dataclass(frozen=True)
class MeasuredModel(RobotConfig):
    measured_factors: tuple = (1.0,1.0,1.0)
    branch: int = 1
    @property
    def factors(self): return self.measured_factors

    def ik(self,xyz,near=None):
        if len(xyz)!=3 or not all(math.isfinite(v) for v in xyz): raise ValueError("XYZ cần ba số hữu hạn.")
        x,y,z=xyz
        tx,ty=self.tool_offset_mm; length=math.hypot(tx,self.l2_mm+ty)
        delta=math.atan2(-tx,self.l2_mm+ty)
        cosine=(x*x+y*y-self.l1_mm**2-length**2)/(2*self.l1_mm*length)
        if abs(cosine)>1+1e-10 or math.hypot(x,y)<1e-8: raise ValueError("Điểm ngoài vùng với tới.")
        e=self.branch*math.acos(max(-1,min(1,cosine)))
        b=e-delta
        a=math.atan2(-x,y)-math.atan2(length*math.sin(e),self.l1_mm+length*math.cos(e))
        a=math.degrees(math.atan2(math.sin(a),math.cos(a)))
        candidates=[(a+360*k,math.degrees(b),z) for k in (-1,0,1)]
        valid=[]
        for q in candidates:
            try: self.check_pose(q)
            except ValueError: continue
            valid.append(q)
        if not valid: raise ValueError("Đích vượt giới hạn công tắc hoặc qua tư thế kỳ dị.")
        near=near or (0,45*self.branch,z)
        return min(valid,key=lambda q:abs(q[0]-near[0]))


