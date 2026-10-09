"""Coordinate view, bounded interactive targets and portable setpoint files."""

from dataclasses import dataclass
from pathlib import Path
import csv
import math

POINT_FIELDS = ("name", "x", "y", "z", "speed_mm_s", "dwell_s")


@dataclass(frozen=True)
class Setpoint:
    name: str
    x: float
    y: float
    z: float
    speed_mm_s: float = 5.0
    dwell_s: float = 0.3

    @property
    def xyz(self):
        return (self.x, self.y, self.z)

    def validate(self):
        if (
            not isinstance(self.name, str)
            or not self.name.strip()
            or len(self.name) > 64
        ):
            raise ValueError("Tên điểm cần 1–64 ký tự.")
        values = (*self.xyz, self.speed_mm_s, self.dwell_s)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("Điểm cần các số hữu hạn.")
        if (
            any(abs(v) > 2000 for v in self.xyz)
            or not 0.05 <= self.speed_mm_s <= 50
            or not 0 <= self.dwell_s <= 60
        ):
            raise ValueError(
                "XYZ ngoài phạm vi hoặc tốc độ/thời gian nghỉ không hợp lệ."
            )
        return self


@dataclass(frozen=True)
class Viewport:
    width: float
    height: float
    reach: float

    @property
    def scale(self):
        return max(0.05, min(self.width - 100, self.height - 90) / (2.12 * self.reach))

    def point(self, x, y):
        return (self.width / 2 + x * self.scale, self.height / 2 - y * self.scale)

    def world(self, x, y):
        return ((x - self.width / 2) / self.scale, (self.height / 2 - y) / self.scale)


def bounded_pose(current, target, angle_step=2.0, z_step=0.5):
    """One affine joint segment toward the newest target; keeps elbow branch."""
    fraction = min(
        1.0,
        *(
            limit / abs(b - a) if b != a else 1.0
            for a, b, limit in zip(current, target, (angle_step, angle_step, z_step))
        ),
    )
    return tuple(a + (b - a) * fraction for a, b in zip(current, target))


def pose_ranges(model):
    a = model.joint_limits_deg[0]
    lo, hi = model.joint_limits_deg[1]
    minimum = math.degrees(math.asin(model.singularity_margin))
    b = (
        (max(lo, minimum), min(hi, 180 - minimum))
        if model.branch > 0
        else (max(lo, -180 + minimum), min(hi, -minimum))
    )
    if b[0] >= b[1]:
        raise ValueError("Nhánh khuỷu không có khoảng chỉnh hợp lệ.")
    return (a, b, model.z_limits_mm)


def test_points(model, z, speed=5):
    """Interior XY grid at the current Z; filter against calibrated limits."""
    ratio = (model.l1_mm + model.l2_mm) / 196
    points = []
    for y in (170, 180, 155):
        for x in (0, -25, 25):
            xyz = (x * ratio, y * ratio, z)
            try:
                model.ik(xyz)
            except ValueError:
                continue
            points.append(
                Setpoint(f"P{len(points)+1:02d}", *xyz, speed, 0.3).validate()
            )
    return points


def save_points(path, points):
    if len(points) > 500:
        raise ValueError("Tối đa500 điểm trong một chuỗi.")
    for point in points:
        point.validate()
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(POINT_FIELDS)
        for p in points:
            writer.writerow((p.name, p.x, p.y, p.z, p.speed_mm_s, p.dwell_s))


def load_points(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(POINT_FIELDS):
            raise ValueError("CSV cần cột: " + ", ".join(POINT_FIELDS))
        points = []
        for row in reader:
            if len(points) >= 500:
                raise ValueError("Tối đa500 điểm trong một chuỗi.")
            if None in row:
                raise ValueError(f"Dòng{reader.line_num}: thừa cột.")
            try:
                point = Setpoint(
                    row["name"], *(float(row[k]) for k in POINT_FIELDS[1:])
                ).validate()
            except (TypeError, ValueError, KeyError) as e:
                raise ValueError(f"Dòng{reader.line_num}: {e}") from e
            points.append(point)
    return points


def validate_program(model, start_steps, points, cancelled=lambda: False):
    """Validate every leg before the first MOVE; controller rechecks each leg."""
    if not points or len(points) > 500:
        raise ValueError("Chuỗi cần 1–500 điểm.")
    plans = []
    position = start_steps
    for i, point in enumerate(points, 1):
        if cancelled():
            raise ValueError("Chuỗi bị hủy.")
        try:
            point.validate()
            plan = model.plan(position, point.xyz, point.speed_mm_s)
        except ValueError as e:
            raise ValueError(f"Điểm{i} ({point.name}): {e}") from e
        plans.append(plan)
        position = plan.steps
    return tuple(plans)
