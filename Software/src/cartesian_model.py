"""Measured-coordinate SCARA model. Units: mm, degrees, signed emitted pulses.

Same angular convention as Simulation/matlab: zero points +Y; positive rotates
toward -X. Base XY is (0, 0) for the real robot, rather than the simulated paper.
Wire order: Z, J1, J2. q2 is RELATIVE elbow angle, not absolute tool heading.
"""

from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math


@dataclass(frozen=True)
class RobotConfig:
    l1_mm: float = 98.0
    l2_mm: float = 98.0
    tool_offset_mm: tuple = (0.0, 0.0)
    home_deg: tuple = (0.0, 90.0)
    home_z_mm: float = 0.0
    joint_limits_deg: tuple = ((-100.0, 100.0), (10.0, 170.0))
    z_limits_mm: tuple = (-5.0, 120.0)
    microsteps: tuple = (16, 8, 8)
    ratios: tuple = (2.0, 3.0, 9.0)  # Z lead mm/rev; J1/J2 motor:joint
    positive_high: tuple = (True, True, True)
    coupling: float = 0.0  # delta_q2 = delta_B - coupling * delta_A
    singularity_margin: float = 0.02
    max_master_rate: int = 800
    max_joint_speed_deg_s: tuple = (15.0, 10.0)
    max_z_speed_mm_s: float = 0.5

    def validate(self):
        for name, length in (
            ("tool_offset_mm", 2),
            ("home_deg", 2),
            ("joint_limits_deg", 2),
            ("z_limits_mm", 2),
            ("microsteps", 3),
            ("ratios", 3),
            ("positive_high", 3),
            ("max_joint_speed_deg_s", 2),
        ):
            if (
                not isinstance(getattr(self, name), (tuple, list))
                or len(getattr(self, name)) != length
            ):
                raise ValueError(f"{name}: sai số phần tử.")
        for interval in self.joint_limits_deg:
            if not isinstance(interval, (tuple, list)) or len(interval) != 2:
                raise ValueError("Mỗi giới hạn khớp cần hai số.")
        numbers = [
            self.l1_mm,
            self.l2_mm,
            self.home_z_mm,
            self.coupling,
            self.singularity_margin,
            self.max_z_speed_mm_s,
            *self.tool_offset_mm,
            *self.home_deg,
            *self.z_limits_mm,
            *self.ratios,
            *self.max_joint_speed_deg_s,
            *(x for pair in self.joint_limits_deg for x in pair),
        ]
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in numbers):
            raise ValueError("Cấu hình cần các số hữu hạn.")
        if not all(10 <= x <= 500 for x in (self.l1_mm, self.l2_mm)):
            raise ValueError("Chiều dài khâu cần trong 10–500 mm.")
        if math.hypot(self.tool_offset_mm[0], self.l2_mm + self.tool_offset_mm[1]) < 1:
            raise ValueError("Độ dài hiệu dụng của khâu 2 quá nhỏ.")
        if any(abs(x) > 200 for x in self.tool_offset_mm) or abs(self.coupling) > 2:
            raise ValueError("Tool offset hoặc hệ số liên động vượt phạm vi.")
        if any(
            type(x) is not int or x not in (1, 2, 4, 8, 16, 32) for x in self.microsteps
        ):
            raise ValueError("Vi bước phải khớp DIP: 1, 2, 4, 8, 16 hoặc 32.")
        if not 0.5 <= self.ratios[0] <= 20 or not all(
            1 <= x <= 100 for x in self.ratios[1:]
        ):
            raise ValueError("Sai hành trình vít me hoặc tỷ số truyền.")
        if (
            any(type(x) is not bool for x in self.positive_high)
            or not self.positive_high[0]
        ):
            raise ValueError("Z+ dùng DIR HIGH theo phần cứng đã xác nhận.")
        if (
            type(self.max_master_rate) is not int
            or not 50 <= self.max_master_rate <= 1600
        ):
            raise ValueError("Tốc độ master cần 50–1600 xung/s.")
        if (
            not 0 < self.singularity_margin < 0.5
            or not 0.01 <= self.max_z_speed_mm_s <= 4
        ):
            raise ValueError("Sai ngưỡng kỳ dị hoặc tốc độ Z.")
        if not all(0.1 <= x <= 60 for x in self.max_joint_speed_deg_s):
            raise ValueError("Tốc độ khớp cần 0,1–60 độ/s.")
        for lo, hi in (*self.joint_limits_deg, self.z_limits_mm):
            if lo >= hi or abs(lo) > 1000 or abs(hi) > 1000:
                raise ValueError("Sai giới hạn chuyển động.")
        self.check_pose((*self.home_deg, self.home_z_mm))
        return self

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or set(data) != set(cls.__dataclass_fields__):
            raise ValueError("File cấu hình thiếu hoặc thừa trường.")
        return cls(**data).validate()

    def save(self, path):
        self.validate()
        Path(path).write_text(
            json.dumps(asdict(self), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    @property
    def factors(self):
        return (
            200 * self.microsteps[0] / self.ratios[0],
            200 * self.microsteps[1] * self.ratios[1] / 360,
            200 * self.microsteps[2] * self.ratios[2] / 360,
        )

    def check_pose(self, q):
        if len(q) != 3 or not all(math.isfinite(x) for x in q):
            raise ValueError("Vị trí cần ba số hữu hạn.")
        for value, (lo, hi) in zip(q, (*self.joint_limits_deg, self.z_limits_mm)):
            if not lo - 1e-8 <= value <= hi + 1e-8:
                raise ValueError("Đích hoặc đường đi vượt giới hạn khớp/Z đã cấu hình.")
        delta = math.atan2(-self.tool_offset_mm[0], self.l2_mm + self.tool_offset_mm[1])
        if abs(math.sin(math.radians(q[1]) + delta)) < self.singularity_margin:
            raise ValueError("Đích hoặc đường đi quá gần tư thế kỳ dị.")

    def fk(self, q):
        a, b = math.radians(q[0]), math.radians(q[0] + q[1])
        tx, ty = self.tool_offset_mm
        return (
            -self.l1_mm * math.sin(a)
            + tx * math.cos(b)
            - (self.l2_mm + ty) * math.sin(b),
            self.l1_mm * math.cos(a)
            + tx * math.sin(b)
            + (self.l2_mm + ty) * math.cos(b),
            q[2],
        )

    def elbow(self, q):
        a = math.radians(q[0])
        return (-self.l1_mm * math.sin(a), self.l1_mm * math.cos(a))

    def ik(self, xyz, near=None):
        if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
            raise ValueError("XYZ cần ba số hữu hạn, đơn vị mm.")
        x, y, z = xyz
        tx, ty = self.tool_offset_mm
        length = math.hypot(tx, self.l2_mm + ty)
        delta = math.atan2(-tx, self.l2_mm + ty)
        cosine = (x * x + y * y - self.l1_mm**2 - length**2) / (2 * self.l1_mm * length)
        if abs(cosine) > 1 + 1e-10 or math.hypot(x, y) < 1e-8:
            raise ValueError("Điểm ngoài vùng với tới của robot.")
        # Retain the manual-home elbow branch; never switch across a singularity.
        home_e = math.radians(self.home_deg[1]) + delta
        e = math.copysign(math.acos(max(-1, min(1, cosine))), math.sin(home_e))
        a = math.atan2(-x, y) - math.atan2(
            length * math.sin(e), self.l1_mm + length * math.cos(e)
        )
        a = math.degrees(math.atan2(math.sin(a), math.cos(a)))
        candidates = [(a + 360 * k, math.degrees(e - delta), z) for k in (-1, 0, 1)]
        valid = []
        for q in candidates:
            try:
                self.check_pose(q)
            except ValueError:
                continue
            valid.append(q)
        if not valid:
            raise ValueError("Điểm vượt giới hạn hoặc gần kỳ dị trên nhánh home.")
        near = near or (*self.home_deg, self.home_z_mm)
        return min(valid, key=lambda q: abs(q[0] - near[0]))

    def to_steps(self, q):
        self.check_pose(q)
        a = q[0] - self.home_deg[0]
        axes = (q[2] - self.home_z_mm, a, q[1] - self.home_deg[1] + self.coupling * a)
        result = tuple(round(x * f) for x, f in zip(axes, self.factors))
        if any(abs(x) > 1_000_000 for x in result):
            raise ValueError("Tọa độ motor vượt phạm vi firmware.")
        return result

    def from_steps(self, steps):
        z, a, b = (x / f for x, f in zip(steps, self.factors))
        return (
            self.home_deg[0] + a,
            self.home_deg[1] + b - self.coupling * a,
            self.home_z_mm + z,
        )

    def plan(self, start_steps, xyz, speed_mm_s=5.0):
        if not math.isfinite(speed_mm_s) or not 0.05 <= speed_mm_s <= 50:
            raise ValueError("Tốc độ đầu công tác cần 0,05–50 mm/s.")
        q0 = self.from_steps(start_steps)
        self.check_pose(q0)
        q1 = self.ik(xyz, q0)
        steps = self.to_steps(q1)
        q1 = self.from_steps(steps)
        self.check_pose(q1)
        delta = tuple(t - s for t, s in zip(steps, start_steps))
        ticks = max(map(abs, delta))
        if ticks > 1_000_000:
            raise ValueError("Đoạn vượt 1.000.000 xung/trục; chọn đích gần hơn.")
        if not ticks:
            return MovePlan(steps, 50, (self.fk(q0),), q1, 0.0)
        # Motor interpolation is affine in joints with this linear coupling model.
        # Bounds and singularity are checked for the whole joint interval.
        effective_delta = math.degrees(
            math.atan2(-self.tool_offset_mm[0], self.l2_mm + self.tool_offset_mm[1])
        )
        elo, ehi = sorted((q0[1] + effective_delta, q1[1] + effective_delta))
        if math.floor(elo / 180) != math.floor(ehi / 180):
            raise ValueError("Đường đi cắt tư thế kỳ dị.")
        # Upper bound for tip speed along the joint path, not just endpoint distance.
        length = math.hypot(self.tool_offset_mm[0], self.l2_mm + self.tool_offset_mm[1])
        bound = self.l1_mm * abs(math.radians(q1[0] - q0[0])) + length * abs(
            math.radians(q1[0] + q1[1] - q0[0] - q0[1])
        )
        duration = max(
            math.hypot(bound, q1[2] - q0[2]) / speed_mm_s,
            abs(q1[2] - q0[2]) / self.max_z_speed_mm_s,
            *(abs(q1[i] - q0[i]) / self.max_joint_speed_deg_s[i] for i in (0, 1)),
        )
        rate = min(self.max_master_rate, math.floor(ticks / max(duration, 1e-9)))
        if rate < 50:
            raise ValueError(
                "Tốc độ yêu cầu thấp hơn nhịp tối thiểu 50 xung/s; chỉnh tốc độ/vi bước."
            )
        if ticks > 590 * rate:
            raise ValueError("Đoạn quá lâu (giới hạn 600 s); chọn đích gần hơn.")
        # Check each emitted DDA state too: rounding and belt compensation can
        # move the elbow slightly outside the ideal affine joint interval.
        position, accum = list(start_steps), [0, 0, 0]
        for _ in range(ticks):
            for a in range(3):
                accum[a] += abs(delta[a])
                if accum[a] >= ticks:
                    accum[a] -= ticks
                    position[a] += 1 if delta[a] > 0 else -1
            self.check_pose(self.from_steps(position))
        path = []
        for i in range(101):
            q = tuple(a + (b - a) * i / 100 for a, b in zip(q0, q1))
            self.check_pose(q)
            path.append(self.fk(q))
        return MovePlan(steps, rate, tuple(path), q1, ticks / rate)


@dataclass(frozen=True)
class MovePlan:
    steps: tuple
    rate: int
    path: tuple
    joints: tuple
    minimum_seconds: float


def measured_coupling(shoulder_delta, relative_elbow_delta):
    """Hold motor B fixed: k = -observed relative elbow change / shoulder change."""
    if (
        not all(math.isfinite(x) for x in (shoulder_delta, relative_elbow_delta))
        or abs(shoulder_delta) < 1
    ):
        raise ValueError("Cần đo DOF1 ít nhất 1° và nhập hai góc có dấu.")
    result = -relative_elbow_delta / shoulder_delta
    if abs(result) > 2:
        raise ValueError("Hệ số đo vượt phạm vi cấu hình.")
    return result
