"""Checked Cartesian chords and continuous fixed-Z strokes for firmware R8."""

from dataclasses import dataclass, replace
from functools import lru_cache
import math
from cartesian_nc_workspace import Setpoint
from cartesian_nc_font import word_geometry
from cartesian_nc_path import lookahead, StrokeRun
from cartesian_nc_curves import ellipse_arc, ellipse_radial_error
from cartesian_nc_shapes import (
    PATTERNS,
    SHAPES,
    shape_dimensions,
    shape_vertices,
    shape_ink_distance,
)
from cartesian_nc_geometry import segment_distance


@dataclass(frozen=True)
class BKSettings:
    size_mm: float = 20.0
    center_x: float = 0.0
    center_y: float = 165.0
    paper_z: float | None = None
    lift_mm: float = 2.0
    draw_speed: float = 12.0
    travel_speed: float = 12.0
    step_mm: float = 0.75
    tolerance_mm: float = 0.25
    pattern: str = "BK"
    settle_s: float = 0.12

    def validate(self, paper=False):
        if self.pattern not in PATTERNS:
            raise ValueError("Chọn một mẫu trong thư viện hình/chữ.")
        values = (
            self.size_mm,
            self.center_x,
            self.center_y,
            self.lift_mm,
            self.draw_speed,
            self.travel_speed,
            self.step_mm,
            self.tolerance_mm,
            self.settle_s,
        )
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("Thông số BK cần số hữu hạn.")
        if not 4 <= self.size_mm <= 180 or any(
            abs(v) > 1000 for v in (self.center_x, self.center_y)
        ):
            raise ValueError("Chiều rộng cần4–180mm; tâm XY ngoài phạm vi.")
        if (
            not 0.25 <= self.lift_mm <= 10
            or not 0.05 <= self.draw_speed <= 50
            or not 0.05 <= self.travel_speed <= 50
        ):
            raise ValueError("Nhấc bút cần0,25–10mm; tốc độ cần0,05–50mm/s.")
        if not 0.2 <= self.step_mm <= 2 or not 0.05 <= self.tolerance_mm <= 1:
            raise ValueError("Bước nét cần0,2–2mm; sai lệch mô hình cần0,05–1mm.")
        if not 0 <= self.settle_s <= 1:
            raise ValueError("Thời gian chờ bút cần 0–1 giây.")
        if self.paper_z is not None and (
            type(self.paper_z) not in (int, float) or not math.isfinite(self.paper_z)
        ):
            raise ValueError("Z giấy cần số hữu hạn.")
        if paper and self.paper_z is None:
            raise ValueError("Cần xác nhận Z đầu bút vừa chạm giấy.")
        return self


@dataclass(frozen=True)
class Vertex:
    xy: tuple
    part: str


def bk_strokes(settings):
    s = settings.validate()
    scale = s.size_mm / 0.5

    def xy(x, y):
        return (s.center_x + (x - 0.25) * scale, s.center_y + (y - 0.25) * scale)

    def line(start, end, part):
        n = max(1, math.ceil(math.dist(start, end) * scale / s.step_mm))
        return [
            Vertex(xy(*(a + (b - a) * i / n for a, b in zip(start, end))), part)
            for i in range(1, n + 1)
        ]

    b = [Vertex(xy(0, 0.5), "B_stem")] + line((0, 0.5), (0, 0), "B_stem")
    for cy, part in ((0.125, "B_lower"), (0.375, "B_upper")):
        center = xy(0, cy)
        radius = s.size_mm / 4
        b.extend(
            Vertex(point, part)
            for point in ellipse_arc(
                *center,
                radius,
                radius,
                -math.pi / 2,
                math.pi / 2,
                s.step_mm,
                s.tolerance_mm / 10,
            )
        )
    stem = [Vertex(xy(0.25, 0.5), "K_stem")] + line((0.25, 0.5), (0.25, 0), "K_stem")
    diagonal = (
        [Vertex(xy(0.5, 0.5), "K_upper")]
        + line((0.5, 0.5), (0.25, 0.25), "K_upper")
        + line((0.25, 0.25), (0.5, 0), "K_lower")
    )
    return tuple(tuple(stroke) for stroke in (b, stem, diagonal))


def pattern_dimensions(settings):
    s = settings.validate()
    if s.pattern in SHAPES:
        return shape_dimensions(s)
    return (
        (s.size_mm, s.size_mm)
        if s.pattern == "BK"
        else word_geometry(s.pattern, s.size_mm, s.center_x, s.center_y)[:2]
    )


@lru_cache(maxsize=32)
def text_edges(settings):
    polylines = word_geometry(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2]
    return {
        f"TXT_{i}_{j}": (a, b)
        for i, line in enumerate(polylines)
        for j, (a, b) in enumerate(zip(line, line[1:]))
    }


def pattern_strokes(settings):
    s = settings.validate()
    if s.pattern in SHAPES:
        return tuple(
            tuple(Vertex(xy, part) for xy, part in stroke)
            for stroke in shape_vertices(s)
        )
    if s.pattern == "BK":
        return bk_strokes(s)
    lines = word_geometry(s.pattern, s.size_mm, s.center_x, s.center_y)[2]
    strokes = []
    for index, line in enumerate(lines):
        if s.pattern == "THOA" and index == 5:
            scale = s.size_mm / 3.7
            cx = s.center_x - s.size_mm / 2 + 2.35 * scale
            vertices = [Vertex((cx, s.center_y + scale / 2), "O_curve")]
            vertices.extend(
                Vertex(point, "O_curve")
                for point in ellipse_arc(
                    cx,
                    s.center_y,
                    0.35 * scale,
                    0.5 * scale,
                    math.pi / 2,
                    -3 * math.pi / 2,
                    s.step_mm,
                    s.tolerance_mm / 10,
                )
            )
            strokes.append(tuple(vertices))
            continue
        vertices = [Vertex(line[0], f"TXT_{index}_0")]
        for j, (a, b) in enumerate(zip(line, line[1:])):
            n = max(1, math.ceil(math.dist(a, b) / s.step_mm))
            vertices.extend(
                Vertex(
                    tuple(x + (y - x) * i / n for x, y in zip(a, b)), f"TXT_{index}_{j}"
                )
                for i in range(1, n + 1)
            )
        strokes.append(tuple(vertices))
    return tuple(strokes)


def ink_distance(xy, part, s):
    if s.pattern in SHAPES:
        return shape_ink_distance(xy, part, s)
    if part == "O_curve":
        scale = s.size_mm / 3.7
        cx = s.center_x - s.size_mm / 2 + 2.35 * scale
        return ellipse_radial_error(xy, cx, s.center_y, 0.35 * scale, 0.5 * scale)
    if s.pattern != "BK":
        return segment_distance(xy, *text_edges(s)[part])
    x0 = s.center_x - s.size_mm / 2
    y0 = s.center_y - s.size_mm / 2
    y1 = y0 + s.size_mm
    if part == "B_stem":
        return segment_distance(xy, (x0, y0), (x0, y1))
    if part == "K_stem":
        return segment_distance(xy, (s.center_x, y0), (s.center_x, y1))
    if part in ("K_upper", "K_lower"):
        end = (s.center_x + s.size_mm / 2, y1 if part == "K_upper" else y0)
        return segment_distance(xy, (s.center_x, s.center_y), end)
    cy = s.center_y + (-1 if part == "B_lower" else 1) * s.size_mm / 4
    r = s.size_mm / 4
    if xy[0] >= x0:
        return abs(math.dist(xy, (x0, cy)) - r)
    return min(math.dist(xy, (x0, cy - r)), math.dist(xy, (x0, cy + r)))


def dda_xyz(model, start, plan):
    delta = tuple(b - a for a, b in zip(start, plan.steps))
    ticks = max(map(abs, delta))
    if not ticks:
        return
    position = list(start)
    accum = [0, 0, 0]
    for _ in range(ticks):
        for a in range(3):
            accum[a] += abs(delta[a])
            if accum[a] >= ticks:
                accum[a] -= ticks
                position[a] += 1 if delta[a] > 0 else -1
        yield model.fk(model.from_steps(position))


@dataclass(frozen=True)
class DrawingOperation:
    point: Setpoint
    kind: str
    stroke: int
    part: str = ""


@dataclass(frozen=True)
class DrawingProgram:
    settings: BKSettings
    dry: bool
    start_steps: tuple
    operations: tuple
    strokes: tuple
    max_model_error_mm: float
    runs: tuple = ()

    @property
    def points(self):
        return tuple(op.point for op in self.operations)


def compile_bk(model, start_steps, settings, dry=False, cancelled=lambda: False):
    """Read-only full preflight, including each emitted DDA state on the ink."""
    s = settings.validate(paper=True)
    strokes = pattern_strokes(s)
    up = s.paper_z + s.lift_mm
    lo, hi = model.z_limits_mm
    if not lo <= s.paper_z < up <= hi:
        raise ValueError("Z giấy hoặc Z nhấc bút nằm ngoài hành trình đã home.")
    if s.lift_mm * model.factors[0] < 1:
        raise ValueError("Độ nhấc bút nhỏ hơn một xung Z; kiểm tra thông số Z.")
    for stroke in strokes:
        for v in stroke:
            model.ik((*v.xy, s.paper_z))
    position = start_steps
    operations = []
    max_error = 0.0
    plans = []
    starts = []

    def append(xyz, kind, stroke, part=""):
        nonlocal position, max_error
        if cancelled():
            raise ValueError("Tác vụ vẽ đã hủy.")
        if len(operations) >= 2000:
            raise ValueError("Mẫu vượt2000đoạn; tăng bước nét hoặc giảm cỡ chữ.")
        point = Setpoint(
            f"{s.pattern}{stroke+1}_{kind}_{len(operations)+1}",
            *xyz,
            s.draw_speed if kind in ("ink", "dry") else s.travel_speed,
            s.settle_s if kind == "lower" else 0,
        ).validate()
        try:
            plan = model.plan(position, point.xyz, point.speed_mm_s)
        except ValueError as e:
            raise ValueError(f"{point.name}: {e}") from e
        if kind in ("ink", "dry"):
            error = max(
                (
                    ink_distance(p[:2], part, s)
                    for p in (*plan.path, *tuple(dda_xyz(model, position, plan)))
                ),
                default=0,
            )
            max_error = max(max_error, error)
            if error > s.tolerance_mm:
                raise ValueError(
                    f"Sai lệch mô hình{error:.3f}mm vượt{s.tolerance_mm:g}mm tại{part}; kiểm tra độ phân giải/bước nét."
                )
            # A finer curve can produce repeated rounded motor coordinates.
            # Skip the stationary vertex only after checking its ink error.
            if plan.steps == position:
                return
        plans.append(plan)
        starts.append(position)
        operations.append(DrawingOperation(point, kind, stroke, part))
        position = plan.steps

    start = model.fk(model.from_steps(start_steps))
    # The first move has identical XY and only goes to pen-clear Z. Never
    # combine the initial lift with a lateral transfer from an arbitrary pose.
    append((*start[:2], up), "lift", 0)
    for index, stroke in enumerate(strokes):
        append((*stroke[0].xy, up), "travel", index)
        if not dry:
            append((*stroke[0].xy, s.paper_z), "lower", index)
        for vertex in stroke[1:]:
            append(
                (*vertex.xy, up if dry else s.paper_z),
                "dry" if dry else "ink",
                index,
                vertex.part,
            )
        if not dry:
            append((*stroke[-1].xy, up), "lift", index)
    runs = []
    i = 0
    while i < len(operations):
        if operations[i].kind not in ("ink", "dry"):
            i += 1
            continue
        first = i
        stroke = operations[i].stroke
        while (
            i < len(operations)
            and operations[i].kind in ("ink", "dry")
            and operations[i].stroke == stroke
        ):
            i += 1
        if i - first > 500:
            raise ValueError("Một nét vượt500đoạn; tăng bước nét hoặc giảm cỡ chữ.")
        segments = lookahead(
            plans[first:i],
            starts[first:i],
            [
                (operations[j - 1].point.xyz[:2], operations[j].point.xyz[:2])
                for j in range(first, i)
            ],
        )
        runs.append(
            StrokeRun(
                first,
                i,
                tuple(starts[first]),
                segments,
                sum(seg.seconds() for seg in segments),
            )
        )
    return DrawingProgram(
        s, dry, tuple(start_steps), tuple(operations), strokes, max_error, tuple(runs)
    )


def fit_drawing(model, start_steps, settings, cancelled=lambda: False):
    """Largest geometry at the chosen center, then full dry preflight.
    Leave 4% size clearance. Search is read-only and never confirms paper Z.
    """
    settings.validate()

    def geometry_ok(width):
        if cancelled():
            raise ValueError("Tìm kích thước đã hủy.")
        candidate = replace(settings, size_mm=width)
        try:
            for stroke in pattern_strokes(candidate):
                for vertex in stroke:
                    model.ik((*vertex.xy, model.z_limits_mm[0]))
            return True
        except ValueError:
            return False

    if not geometry_ok(4):
        raise ValueError(
            "Tâm hiện tại không chứa được mẫu; chỉnh tâm XY vào vùng làm việc."
        )
    low, high = 4.0, 180.0
    for _ in range(18):
        mid = (low + high) / 2
        if geometry_ok(mid):
            low = mid
        else:
            high = mid
    width = max(4, math.floor(low * 0.96 * 100) / 100)
    zlo, zhi = model.z_limits_mm
    if settings.lift_mm >= zhi - zlo:
        raise ValueError("Độ nhấc bút vượt hành trình Z.")
    trial_z = (zlo + zhi - settings.lift_mm) / 2
    for _ in range(12):
        candidate = replace(settings, size_mm=width)
        try:
            compile_bk(
                model, start_steps, replace(candidate, paper_z=trial_z), True, cancelled
            )
            return candidate
        except ValueError:
            if cancelled():
                raise ValueError("Tìm kích thước đã hủy.")
            if width <= 4:
                break
            width = max(4, math.floor(width * 0.9 * 100) / 100)
    raise ValueError(
        "Chưa tìm được cỡ chữ đạt dung sai; chỉnh tâm, bước nét hoặc kiểm tra độ phân giải."
    )
