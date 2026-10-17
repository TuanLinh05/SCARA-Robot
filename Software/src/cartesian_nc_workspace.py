"""
================================================================================
FILE: cartesian_nc_workspace.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Provides 2D canvas coordinate transforms (`Viewport`), interactive pose
     stepping (`bounded_pose`, `pose_ranges`), portable CSV setpoint persistence
     (`Setpoint`, `save_points`, `load_points`), and whole-program DDA path
     pre-validation (`validate_program`).

[VI] MỤC ĐÍCH:
     Cung cấp phép biến đổi tọa độ khung vẽ 2D (`Viewport`), hàm chia bước tư thế
     tương tác (`bounded_pose`, `pose_ranges`), lưu/đọc danh sách điểm CSV
     (`Setpoint`, `save_points`, `load_points`), và kiểm duyệt trước toàn bộ
     chương trình đường chạy DDA (`validate_program`).

[DEBUG TIP / MẸO GỠ LỖI]:
     - In `Viewport`, world `Y` points UP, whereas Tkinter Canvas `Y` points DOWN:
       notice the minus sign in `point()` (`self.height / 2 + self.pan_y - y * self.scale`).
     - Trong `Viewport`, trục `Y` thực tế hướng LÊN, còn trục `Y` của Tkinter Canvas
       hướng XUỐNG: hãy chú ý dấu trừ trong hàm `point()` (`self.height / 2 + self.pan_y - y * self.scale`).
================================================================================
"""

from dataclasses import dataclass
from pathlib import Path
import csv
import math

# [EN] Column headers for portable setpoint CSV files.
# [VI] Tên các cột tiêu chuẩn trong file CSV lưu chuỗi điểm setpoint.
POINT_FIELDS = ("name", "x", "y", "z", "speed_mm_s", "dwell_s")


# ==============================================================================
# 1. SETPOINT DATA MODEL / MÔ HÌNH DỮ LIỆU ĐIỂM ĐẶT (SETPOINT)
# ==============================================================================


@dataclass(frozen=True)
class Setpoint:
    """
    [EN] A named 3D target waypoint `(x, y, z)` with travel speed (`speed_mm_s`)
         and post-arrival dwell time (`dwell_s`).
    [VI] Một điểm đích 3D có tên `(x, y, z)` kèm theo tốc độ di chuyển (`speed_mm_s`)
         và thời gian dừng nghỉ sau khi đến đích (`dwell_s`).
    """

    name: str
    x: float
    y: float
    z: float
    speed_mm_s: float = 6.0
    dwell_s: float = 0.3

    @property
    def xyz(self):
        """
        [EN] Return the `(x, y, z)` Cartesian coordinates as a 3-tuple.
        [VI] Trả về bộ 3 tọa độ Đề-các `(x, y, z)`.
        """
        return (self.x, self.y, self.z)

    def validate(self):
        """
        [EN] Validate name length, finite numerical values, and safe ranges for
             coordinates, speed, and dwell time. Returns `self`.
        [VI] Kiểm tra độ dài tên, giá trị số hữu hạn và phạm vi hợp lệ của tọa độ,
             tốc độ và thời gian nghỉ. Trả về chính đối tượng `self`.
        """
        # [EN] 1. Check that `name` is a non-empty string of at most 64 characters.
        # [VI] 1. Kiểm tra `name` là chuỗi khác rỗng và dài tối đa 64 ký tự.
        if (
            not isinstance(self.name, str)
            or not self.name.strip()
            or len(self.name) > 64
        ):
            raise ValueError("Tên điểm cần 1–64 ký tự.")

        # [EN] 2. Check that all numeric fields are finite `int` or `float` (not `bool`).
        #         Note: In Python `bool` is a subclass of `int`, so `type(v) not in (int, float)`
        #         is used instead of `isinstance(v, (int, float))` to reject `True`/`False`.
        # [VI] 2. Kiểm tra mọi trường số đều là `int` hoặc `float` hữu hạn (không nhận `bool`).
        #         Lưu ý: Trong Python `bool` kế thừa từ `int`, nên dùng `type(v) not in (int, float)`
        #         để loại trừ `True`/`False`.
        numeric_values = (self.x, self.y, self.z, self.speed_mm_s, self.dwell_s)
        for val in numeric_values:
            if type(val) not in (int, float) or not math.isfinite(val):
                raise ValueError("Điểm cần các số hữu hạn.")

        # [EN] 3. Verify coordinates are within +/-2000 mm, speed in [0.05, 50] mm/s,
        #         and dwell time in [0, 60] seconds.
        # [VI] 3. Kiểm tra tọa độ trong khoảng +/-2000 mm, tốc độ [0.05, 50] mm/s,
        #         và thời gian nghỉ [0, 60] giây.
        coord_out_of_range = False
        for coord in self.xyz:
            if abs(coord) > 2000:
                coord_out_of_range = True
                break

        speed_valid = 0.05 <= self.speed_mm_s <= 50
        dwell_valid = 0 <= self.dwell_s <= 60
        if coord_out_of_range or not speed_valid or not dwell_valid:
            raise ValueError(
                "XYZ ngoài phạm vi hoặc tốc độ/thời gian nghỉ không hợp lệ."
            )

        return self


# ==============================================================================
# 2. 2D CANVAS VIEWPORT TRANSFORM / PHÉP BIẾN ĐỔI KHUNG NHÌN CANVAS 2D
# ==============================================================================


@dataclass(frozen=True)
class Viewport:
    """
    [EN] Immutable 2D camera transform mapping between robot world coordinates
         `(x, y)` in millimeters and Tkinter Canvas pixel coordinates `(px, py)`.
    [VI] Phép biến đổi khung nhìn 2D bất biến ánh xạ giữa tọa độ thực của robot
         `(x, y)` tính bằng milimét và tọa độ điểm ảnh `(px, py)` trên Tkinter Canvas.
    """

    width: float
    height: float
    reach: float
    zoom: float = 1.0
    pan_x: float = 0.0
    pan_y: float = 0.0

    @property
    def scale(self):
        """
        [EN] Effective pixels-per-millimeter scale factor including user zoom.
             Fits the full workspace diameter (`2.12 * reach`) inside the canvas
             with padding for rulers and labels.
        [VI] Hệ số tỷ lệ pixel/milimét thực tế đã bao gồm độ phóng đại `zoom`.
             Vừa khít đường kính vùng làm việc (`2.12 * reach`) vào khung vẽ
             sau khi chừa lề cho thước đo và nhãn.
        """
        usable_pixel_span = min(self.width - 100, self.height - 90)
        world_diameter_mm = 2.12 * self.reach
        base_scale = max(0.05, usable_pixel_span / world_diameter_mm)
        clamped_zoom = max(0.25, min(10.0, self.zoom))
        return max(0.05, base_scale * clamped_zoom)

    def point(self, x, y):
        """
        [EN] Convert world coordinates `(x, y)` in mm to Canvas pixel `(px, py)`.
             Note: Canvas Y grows downward, so `+y` (mm) subtracts `y * scale`.
        [VI] Chuyển đổi tọa độ thực `(x, y)` (mm) sang tọa độ điểm ảnh Canvas `(px, py)`.
             Lưu ý: Trục Y của Canvas hướng xuống dưới, nên `+y` (mm) phải trừ `y * scale`.
        """
        center_px = self.width / 2 + self.pan_x
        center_py = self.height / 2 + self.pan_y
        px = center_px + x * self.scale
        py = center_py - y * self.scale
        return (px, py)

    def world(self, x, y):
        """
        [EN] Convert Canvas pixel `(x, y)` back to world coordinates `(wx, wy)` in mm.
             Exact inverse of `point(wx, wy)`.
        [VI] Chuyển đổi ngược từ điểm ảnh Canvas `(x, y)` về tọa độ thực `(wx, wy)` (mm).
             Là phép biến đổi ngược chính xác của hàm `point(wx, wy)`.
        """
        wx = (x - self.width / 2 - self.pan_x) / self.scale
        wy = (self.height / 2 + self.pan_y - y) / self.scale
        return (wx, wy)

    def zoomed(self, factor, cx=None, cy=None):
        """
        [EN] Return a new `Viewport` zoomed by `factor`, keeping the world point
             under cursor `(cx, cy)` stationary on screen.
        [VI] Trả về `Viewport` mới được phóng đại theo `factor`, giữ cố định điểm
             thực nằm dưới con trỏ chuột `(cx, cy)` trên màn hình.
        """
        new_zoom = max(0.5, min(6.0, self.zoom * factor))
        if cx is None or cy is None:
            return Viewport(
                self.width, self.height, self.reach, new_zoom, self.pan_x, self.pan_y
            )

        # [EN] Find which world point `(wx, wy)` is currently under pixel `(cx, cy)`,
        #      then compute the new pan offset so `(wx, wy)` stays at `(cx, cy)`.
        # [VI] Tìm tọa độ thực `(wx, wy)` đang nằm dưới điểm ảnh `(cx, cy)`,
        #      sau đó tính độ lệch `pan` mới để `(wx, wy)` vẫn đứng yên tại `(cx, cy)`.
        wx, wy = self.world(cx, cy)
        base_view = Viewport(self.width, self.height, self.reach, new_zoom, 0.0, 0.0)
        px0, py0 = base_view.point(wx, wy)
        new_pan_x = cx - px0
        new_pan_y = cy - py0
        return Viewport(
            self.width,
            self.height,
            self.reach,
            new_zoom,
            new_pan_x,
            new_pan_y,
        )

    def panned(self, dx, dy):
        """
        [EN] Return a new `Viewport` shifted by `(dx, dy)` canvas pixels.
        [VI] Trả về `Viewport` mới đã dịch chuyển thêm `(dx, dy)` điểm ảnh.
        """
        return Viewport(
            self.width,
            self.height,
            self.reach,
            self.zoom,
            self.pan_x + dx,
            self.pan_y + dy,
        )


# ==============================================================================
# 3. INTERACTIVE POSE HELPERS & CSV I/O / HỖ TRỢ TƯ THẾ & ĐỌC/GHI FILE CSV
# ==============================================================================


def bounded_pose(current, target, angle_step=2.0, z_step=0.5):
    """
    [EN] Compute one affine joint segment from `current` `(j1, j2, z)` toward
         `target` `(j1, j2, z)` bounded by `angle_step` (deg) and `z_step` (mm).
         Preserves the current elbow branch during interactive follow mode.
    [VI] Tính một bước tiến tuyến tính trong không gian khớp từ `current` `(j1, j2, z)`
         hướng tới `target` `(j1, j2, z)` bị giới hạn bởi `angle_step` (độ) và `z_step` (mm).
         Giúp giữ nguyên nhánh khuỷu tay trong chế độ bám mô hình.
    """
    axis_limits = (angle_step, angle_step, z_step)
    allowed_fractions = [1.0]

    for curr_val, target_val, limit in zip(current, target, axis_limits):
        if target_val != curr_val:
            allowed_fractions.append(limit / abs(target_val - curr_val))

    fraction = min(allowed_fractions)

    next_pose = []
    for curr_val, target_val in zip(current, target):
        next_pose.append(curr_val + (target_val - curr_val) * fraction)
    return tuple(next_pose)


def pose_ranges(model):
    """
    [EN] Return the valid slider intervals `((j1_min, j1_max), (j2_min, j2_max),
         (z_min, z_max))` for the configured elbow branch, excluding singularities.
    [VI] Trả về khoảng giới hạn thanh trượt hợp lệ `((j1_min, j1_max), (j2_min, j2_max),
         (z_min, z_max))` cho nhánh khuỷu tay đang chọn, đã trừ vùng suy biến.
    """
    j1_range = model.joint_limits_deg[0]
    j2_lo, j2_hi = model.joint_limits_deg[1]

    # [EN] Minimum |J2| angle (deg) away from 0 deg (straight) and 180 deg (folded).
    # [VI] Góc |J2| tối thiểu (độ) cách xa điểm suy biến 0 độ (duỗi thẳng) và 180 độ (gập sát).
    minimum = math.degrees(math.asin(model.singularity_margin))

    if model.branch > 0:
        j2_range = (max(j2_lo, minimum), min(j2_hi, 180 - minimum))
    else:
        j2_range = (max(j2_lo, -180 + minimum), min(j2_hi, -minimum))

    if j2_range[0] >= j2_range[1]:
        raise ValueError("Nhánh khuỷu không có khoảng chỉnh hợp lệ.")

    return (j1_range, j2_range, model.z_limits_mm)


def test_points(model, z, speed=6):
    """
    [EN] Generate a 3x3 interior XY test grid at height `z`, filtered against the
         calibrated inverse kinematics limits.
    [VI] Tạo lưới 3x3 điểm kiểm thử XY bên trong vùng làm việc tại độ cao `z`,
         đã lọc qua giới hạn động học ngược thực tế sau hiệu chuẩn.
    """
    ratio = (model.l1_mm + model.l2_mm) / 196
    points = []
    for y in (170, 180, 155):
        for x in (0, -25, 25):
            xyz = (x * ratio, y * ratio, z)
            try:
                model.ik(xyz)
            except ValueError:
                continue
            point_name = f"P{len(points)+1:02d}"
            point = Setpoint(
                point_name, xyz[0], xyz[1], xyz[2], speed, 0.3
            ).validate()
            points.append(point)
    return points


def save_points(path, points):
    """
    [EN] Validate and save up to 500 `Setpoint` rows to a UTF-8-SIG CSV file.
    [VI] Kiểm tra và lưu tối đa 500 điểm `Setpoint` ra file CSV mã hóa UTF-8-SIG.
    """
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
    """
    [EN] Load and validate up to 500 `Setpoint` rows from a CSV file.
    [VI] Đọc và kiểm tra tính hợp lệ của tối đa 500 điểm `Setpoint` từ file CSV.
    """
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
                    row["name"],
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                    float(row["speed_mm_s"]),
                    float(row["dwell_s"]),
                ).validate()
            except (TypeError, ValueError, KeyError) as e:
                raise ValueError(f"Dòng{reader.line_num}: {e}") from e
            points.append(point)
    return points


def validate_program(model, start_steps, points, cancelled=lambda: False):
    """
    [EN] Validate every leg of a multi-point setpoint sequence before the first
         `MOVE` is sent; the controller also rechecks each leg at runtime.
    [VI] Kiểm duyệt toàn bộ từng chặng của chuỗi điểm setpoint trước khi phát lệnh
         `MOVE` đầu tiên; bộ điều khiển cũng sẽ kiểm tra lại từng chặng khi chạy thật.
    """
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
