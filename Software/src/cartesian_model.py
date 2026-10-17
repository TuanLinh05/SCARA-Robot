"""
============================================================================
FILE: cartesian_model.py
============================================================================
[EN] SCARA Forward/Inverse Kinematics, Stepper Pulse Quantization & Point-to-Point Planner
     - Units: millimeters (`mm`), degrees (`deg`), and signed integer stepper pulses.
     - Coordinate convention (matches `Simulation/matlab` and `Simulation/webots`):
       * Zero shoulder angle (`q1 = 0 deg`) points along the `+Y` axis.
       * Positive joint rotation turns counter-clockwise toward `-X`:
         `x = -L1 * sin(q1) + tx * cos(q1 + q2) - (L2 + ty) * sin(q1 + q2)`
         `y =  L1 * cos(q1) + tx * sin(q1 + q2) + (L2 + ty) * cos(q1 + q2)`
       * Wire axis order is `(Z, J1, J2)`, where `q2` is the RELATIVE elbow angle.

[VI] Mô hình Động học Thuận/Ngược SCARA, Lượng tử hóa Xung & Quy hoạch Điểm-tới-Điểm
     - Đơn vị: mi-li-mét (`mm`), độ (`deg`), và số nguyên xung động cơ bước có dấu.
     - Quy ước hệ tọa độ (đồng nhất với MATLAB và Webots):
       * Khi góc khớp vai `q1 = 0 độ`, cánh tay hướng dọc theo trục `+Y`.
       * Chiều quay dương hướng ngược chiều kim đồng hồ về phía `-X`.
       * Thứ tự 3 trục truyền xuống mạch luôn là `(Z, J1, J2)`, trong đó `q2` là
         góc khuỷu TƯƠNG ĐỐI so với khâu 1.

[DEBUG TIP / MẸO GỠ LỖI]:
- `round_nearest(x)` implements symmetric half-away-from-zero rounding (`2.5 -> 3`,
  `-2.5 -> -3`) so Python pulse counts match C firmware `rounded()` and MATLAB `round()`
  to the exact single microstep!
- Hàm `round_nearest(x)` làm tròn đối xứng qua điểm 0 (`2.5 -> 3`, `-2.5 -> -3`)
  giúp số xung tính bằng Python khớp tuyệt đối từng vi bước với hàm `rounded()`
  trong Firmware C và `round()` trong MATLAB!
============================================================================
"""

from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math


def round_nearest(x: float) -> int:
    """
    [EN] Symmetric half-away-from-zero rounding matching Firmware C `rounded()` and MATLAB `round()`.
         Note: Python's built-in `round(2.5)` returns `2` (round-to-even), whereas hardware
         step quantization requires `2.5 -> 3` and `-2.5 -> -3`.
    [VI] Làm tròn số nguyên đối xứng qua 0, đồng bộ hoàn toàn với Firmware C và MATLAB.
         Lưu ý: Hàm `round(2.5)` mặc định của Python trả về `2` (làm tròn về số chẵn),
         trong khi lượng tử hóa xung phần cứng yêu cầu `2.5 -> 3` và `-2.5 -> -3`.
    """
    if x >= 0:
        return int(math.floor(x + 0.5))
    else:
        return -int(math.floor(-x + 0.5))


@dataclass(frozen=True)
class RobotConfig:
    """
    [EN] Geometric, transmission, and velocity parameters of the 3-axis SCARA robot.
    [VI] Các thông số hình học, tỷ số truyền cơ khí và giới hạn vận tốc của robot SCARA 3 trục.
    """

    l1_mm: float = 98.0
    l2_mm: float = 98.0
    tool_offset_mm: tuple = (0.0, 0.0)
    home_deg: tuple = (0.0, 90.0)
    home_z_mm: float = 0.0
    joint_limits_deg: tuple = ((-100.0, 100.0), (10.0, 170.0))
    z_limits_mm: tuple = (-5.0, 120.0)
    microsteps: tuple = (16, 16, 8)
    ratios: tuple = (2.0, 3.0, 9.0)  # [EN] Z lead mm/rev; J1/J2 motor:joint / [VI] Bước vít me Z; tỷ số truyền J1/J2
    positive_high: tuple = (True, True, True)
    coupling: float = 0.0  # [EN] delta_q2 = delta_B - coupling * delta_A / [VI] Hệ số truyền đai chéo
    singularity_margin: float = 0.02
    max_master_rate: int = 800
    max_joint_speed_deg_s: tuple = (15.0, 10.0)
    max_z_speed_mm_s: float = 0.5

    def validate(self):
        """
        [EN] Validate all configuration fields and verify that `home_deg` / `home_z_mm`
             is a valid, non-singular pose.
        [VI] Kiểm tra tính hợp lệ của toàn bộ thông số cấu hình và đảm bảo vị trí Home
             nằm trong vùng làm việc an toàn, cách xa điểm kỳ dị.
        """
        # 1. Check tuple/list lengths for vector configuration parameters
        # 1. Kiểm tra số lượng phần tử của các thông số dạng mảng/tuple
        expected_lengths = (
            ("tool_offset_mm", 2),
            ("home_deg", 2),
            ("joint_limits_deg", 2),
            ("z_limits_mm", 2),
            ("microsteps", 3),
            ("ratios", 3),
            ("positive_high", 3),
            ("max_joint_speed_deg_s", 2),
        )
        for name, length in expected_lengths:
            value = getattr(self, name)
            if not isinstance(value, (tuple, list)) or len(value) != length:
                raise ValueError(f"{name}: sai số phần tử.")

        for interval in self.joint_limits_deg:
            if not isinstance(interval, (tuple, list)) or len(interval) != 2:
                raise ValueError("Mỗi giới hạn khớp cần hai số.")

        # 2. Verify all numeric fields are finite int/float numbers (not NaN, Inf, or bool)
        # 2. Kiểm tra tất cả các trường số đều là số thực/nguyên hữu hạn (không phải NaN, Inf hay bool)
        j1_limit_pair = self.joint_limits_deg[0]
        j2_limit_pair = self.joint_limits_deg[1]
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
            *j1_limit_pair,
            *j2_limit_pair,
        ]
        for x in numbers:
            if type(x) not in (int, float) or not math.isfinite(x):
                raise ValueError("Cấu hình cần các số hữu hạn.")

        # 3. Validate geometric link lengths, tool offset, and belt coupling bounds
        # 3. Kiểm tra chiều dài các khâu cơ khí, độ lệch đầu bút và hệ số truyền đai
        if not (10 <= self.l1_mm <= 500 and 10 <= self.l2_mm <= 500):
            raise ValueError("Chiều dài khâu cần trong 10–500 mm.")

        tx_mm, ty_mm = self.tool_offset_mm
        effective_l2_mm = math.hypot(tx_mm, self.l2_mm + ty_mm)
        if effective_l2_mm < 1:
            raise ValueError("Độ dài hiệu dụng của khâu 2 quá nhỏ.")

        if abs(tx_mm) > 200 or abs(ty_mm) > 200 or abs(self.coupling) > 2:
            raise ValueError("Tool offset hoặc hệ số liên động vượt phạm vi.")

        # 4. Validate stepper driver microsteps, transmission ratios, and DIR polarities
        # 4. Kiểm tra vi bước driver, tỷ số truyền cơ khí và cực tính chân DIR
        valid_microsteps = (1, 2, 4, 8, 16, 32)
        for m in self.microsteps:
            if type(m) is not int or m not in valid_microsteps:
                raise ValueError("Vi bước phải khớp DIP: 1, 2, 4, 8, 16 hoặc 32.")

        z_lead_mm = self.ratios[0]
        j1_ratio = self.ratios[1]
        j2_ratio = self.ratios[2]
        if not (0.5 <= z_lead_mm <= 20) or not (1 <= j1_ratio <= 100 and 1 <= j2_ratio <= 100):
            raise ValueError("Sai hành trình vít me hoặc tỷ số truyền.")

        for p in self.positive_high:
            if type(p) is not bool:
                raise ValueError("Z+ dùng DIR HIGH theo phần cứng đã xác nhận.")
        if not self.positive_high[0]:
            raise ValueError("Z+ dùng DIR HIGH theo phần cứng đã xác nhận.")

        # 5. Validate pulse rate, singularity margin, speed limits, and travel intervals
        # 5. Kiểm tra tốc độ phát xung, ngưỡng kỳ dị, vận tốc tối đa và giới hạn hành trình
        if type(self.max_master_rate) is not int or not (50 <= self.max_master_rate <= 1600):
            raise ValueError("Tốc độ master cần 50–1600 xung/s.")

        if not (0 < self.singularity_margin < 0.5) or not (0.01 <= self.max_z_speed_mm_s <= 4):
            raise ValueError("Sai ngưỡng kỳ dị hoặc tốc độ Z.")

        for joint_speed in self.max_joint_speed_deg_s:
            if not (0.1 <= joint_speed <= 60):
                raise ValueError("Tốc độ khớp cần 0,1–60 độ/s.")

        all_limit_intervals = (*self.joint_limits_deg, self.z_limits_mm)
        for lo, hi in all_limit_intervals:
            if lo >= hi or abs(lo) > 1000 or abs(hi) > 1000:
                raise ValueError("Sai giới hạn chuyển động.")

        # 6. Verify that the configured Home pose itself is valid and non-singular
        # 6. Đảm bảo bản thân tư thế Home đã cấu hình cũng hợp lệ và không rơi vào điểm kỳ dị
        self.check_pose((*self.home_deg, self.home_z_mm))
        return self

    @classmethod
    def load(cls, path):
        """
        [EN] Load and validate JSON config from `path`.
        [VI] Đọc và kiểm tra tính hợp lệ của file cấu hình JSON từ đường dẫn `path`.
        """
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or set(data) != set(cls.__dataclass_fields__):
            raise ValueError("File cấu hình thiếu hoặc thừa trường.")
        return cls(**data).validate()

    def save(self, path):
        """
        [EN] Validate and save JSON config to `path`.
        [VI] Kiểm tra tính hợp lệ và lưu cấu hình ra file JSON tại đường dẫn `path`.
        """
        self.validate()
        json_text = json.dumps(asdict(self), indent=2, ensure_ascii=False) + "\n"
        Path(path).write_text(json_text, encoding="utf-8")

    @property
    def factors(self):
        """
        [EN] Nominal pulse scale factors `(pulses_per_mm_Z, pulses_per_deg_J1, pulses_per_deg_J2)`.
             A standard 1.8-degree stepper motor has `360 / 1.8 = 200` full steps per revolution:
             - Z axis (leadscrew): `200 * microsteps_Z / lead_mm_per_rev` (pulses/mm)
             - J1/J2 (belt drive): `200 * microsteps_J * gear_ratio / 360` (pulses/deg)
        [VI] Hệ số quy đổi xung danh định `(xung/mm_Z, xung/độ_J1, xung/độ_J2)`.
             Động cơ bước 1.8 độ tiêu chuẩn có `360 / 1.8 = 200` bước đủ trên 1 vòng quay:
             - Trục Z (vít me): `200 * vi_bước_Z / bước_vít_me_mm` (xung/mm)
             - Trục J1/J2 (đai): `200 * vi_bước_J * tỷ_số_truyền / 360` (xung/độ)
        """
        z_pulses_per_mm = 200 * self.microsteps[0] / self.ratios[0]
        j1_pulses_per_deg = 200 * self.microsteps[1] * self.ratios[1] / 360
        j2_pulses_per_deg = 200 * self.microsteps[2] * self.ratios[2] / 360
        return (z_pulses_per_mm, j1_pulses_per_deg, j2_pulses_per_deg)

    def check_pose(self, q):
        """
        [EN] Verify that joint pose `q = (q1_deg, q2_deg, z_mm)` lies within joint/Z limits
             and stays away from the straight/folded elbow singularity (`|sin(q2 + delta)| >= margin`).
        [VI] Kiểm tra tư thế khớp `q = (q1_deg, q2_deg, z_mm)` nằm trong giới hạn góc/Z
             và không vi phạm vùng kỳ dị khuỷu tay duỗi thẳng (`0°`) hoặc gập sát (`180°`).
        """
        if len(q) != 3:
            raise ValueError("Vị trí cần ba số hữu hạn.")
        q1_deg, q2_deg, z_mm = q
        if not (
            math.isfinite(q1_deg) and math.isfinite(q2_deg) and math.isfinite(z_mm)
        ):
            raise ValueError("Vị trí cần ba số hữu hạn.")

        # Check each coordinate against its configured (lo, hi) interval (with 1e-8 float tolerance)
        # Kiểm tra từng tọa độ khớp với giới hạn dưới/trên (có dung sai số thực 1e-8)
        (j1_lo, j1_hi), (j2_lo, j2_hi) = self.joint_limits_deg
        z_lo, z_hi = self.z_limits_mm
        if (
            not (j1_lo - 1e-8 <= q1_deg <= j1_hi + 1e-8)
            or not (j2_lo - 1e-8 <= q2_deg <= j2_hi + 1e-8)
            or not (z_lo - 1e-8 <= z_mm <= z_hi + 1e-8)
        ):
            raise ValueError("Đích hoặc đường đi vượt giới hạn khớp/Z đã cấu hình.")

        # Effective elbow angle includes the 2D tool offset angle `delta`
        # Góc khuỷu hiệu dụng bao gồm cả góc lệch hình học `delta` của đầu bút
        tx_mm, ty_mm = self.tool_offset_mm
        delta_rad = math.atan2(-tx_mm, self.l2_mm + ty_mm)
        effective_elbow_rad = math.radians(q2_deg) + delta_rad
        if abs(math.sin(effective_elbow_rad)) < self.singularity_margin:
            raise ValueError("Đích hoặc đường đi quá gần tư thế kỳ dị.")

    def fk(self, q):
        """
        [EN] Forward Kinematics (FK): convert joint coordinates `q = (q1_deg, q2_deg, z_mm)`
             into Cartesian pen-tip coordinates `(x_mm, y_mm, z_mm)`.
             Why `-sin` for X and `+cos` for Y?
             Because `q1 = 0 deg` points along the `+Y` axis `(0, L1)`, and positive rotation
             turns counter-clockwise toward `-X`: `(-L1 * sin(q1), L1 * cos(q1))`.
        [VI] Động học Thuận (FK): tính tọa độ Đề-các của đầu bút `(x_mm, y_mm, z_mm)`
             từ góc khớp vai `q[0]`, góc khuỷu tương đối `q[1]` và cao độ `q[2]`.
             Tại sao X dùng `-sin` còn Y dùng `+cos`?
             Vì gốc `q1 = 0 độ` hướng dọc theo trục `+Y` `(0, L1)`, và chiều quay dương
             ngược chiều kim đồng hồ lệch sang phía `-X`: `(-L1 * sin(q1), L1 * cos(q1))`.
        """
        q1_deg, q2_deg, z_mm = q
        shoulder_rad = math.radians(q1_deg)
        forearm_abs_rad = math.radians(q1_deg + q2_deg)
        tx_mm, ty_mm = self.tool_offset_mm

        x_mm = (
            -self.l1_mm * math.sin(shoulder_rad)
            + tx_mm * math.cos(forearm_abs_rad)
            - (self.l2_mm + ty_mm) * math.sin(forearm_abs_rad)
        )
        y_mm = (
            self.l1_mm * math.cos(shoulder_rad)
            + tx_mm * math.sin(forearm_abs_rad)
            + (self.l2_mm + ty_mm) * math.cos(forearm_abs_rad)
        )
        return (x_mm, y_mm, z_mm)

    def elbow(self, q):
        """
        [EN] Return the 2D Cartesian position `(x_mm, y_mm)` of the elbow joint axis (`J2`).
        [VI] Tính tọa độ 2D `(x_mm, y_mm)` của tâm khớp khuỷu (`J2`) để vẽ cánh tay robot trên Canvas.
        """
        shoulder_rad = math.radians(q[0])
        elbow_x_mm = -self.l1_mm * math.sin(shoulder_rad)
        elbow_y_mm = self.l1_mm * math.cos(shoulder_rad)
        return (elbow_x_mm, elbow_y_mm)

    def _elbow_branch(self, delta):
        """
        [EN] Return `+1` (left-hand elbow / positive `q2`) or `-1` (right-hand elbow / negative `q2`)
             matching the configured `home_deg[1]` posture.
        [VI] Trả về `+1` (nhánh khuỷu dương) hoặc `-1` (nhánh khuỷu âm) khớp với tư thế `home_deg[1]`.
        """
        effective_home_elbow_rad = math.radians(self.home_deg[1]) + delta
        if math.sin(effective_home_elbow_rad) >= 0:
            return 1
        return -1

    def _default_near(self, z):
        """
        [EN] Default reference joint posture used when selecting the closest 360-deg IK wrap.
        [VI] Tư thế góc khớp tham chiếu mặc định dùng để chọn nghiệm IK gần nhất.
        """
        return (*self.home_deg, self.home_z_mm)

    def ik(self, xyz, near=None):
        """
        [EN] Inverse Kinematics (IK): solve joint pose `(q1_deg, q2_deg, z_mm)` for target
             Cartesian point `xyz = (x_mm, y_mm, z_mm)` on the robot's home elbow branch:
             1. Effective forearm length `L2_eff = hypot(tx, L2 + ty)` and angular offset `delta`.
             2. Law of Cosines in the triangle (Shoulder -> Elbow -> Pen Tip):
                Since `e = q2 + delta` is the exterior bend angle (`e = 0` when fully extended),
                `r^2 = x^2 + y^2 = L1^2 + L2_eff^2 + 2 * L1 * L2_eff * cos(e)`
                `=> cos(e) = (x^2 + y^2 - L1^2 - L2_eff^2) / (2 * L1 * L2_eff)`.
             3. Select the elbow sign matching `_elbow_branch(delta)` (never cross singularity!).
             4. Solve shoulder angle `q1`:
                The target's azimuth angle measured CCW from the `+Y` axis is `atan2(-x, y)`
                (because at `(0, +Y)`, `-x = 0` and `y > 0`, giving `0 rad`).
                Subtract the internal triangle angle `atan2(L2_eff * sin(e), L1 + L2_eff * cos(e))`.

        [VI] Động học Ngược (IK): giải góc khớp `(q1_deg, q2_deg, z_mm)` từ tọa độ điểm đích
             `xyz = (x_mm, y_mm, z_mm)` trên cùng nhánh khuỷu tay với tư thế Home:
             1. Tính chiều dài khâu 2 hiệu dụng `length` và góc lệch đầu bút `delta`.
             2. Áp dụng Định lý Hàm số Cosin trong tam giác (Vai -> Khuỷu -> Đầu bút):
                Vì `e = q2 + delta` là góc lệch ngoài của khâu 2 (`e = 0` khi duỗi thẳng),
                `r^2 = x^2 + y^2 = L1^2 + L2_eff^2 + 2 * L1 * L2_eff * cos(e)`
                `=> cos(e) = (x^2 + y^2 - L1^2 - L2_eff^2) / (2 * L1 * L2_eff)`.
             3. Giữ cố định nhánh nghiệm khuỷu tay theo `_elbow_branch(delta)` (không cắt qua kỳ dị).
             4. Giải góc vai `q1`:
                Góc phương vị của điểm `(x, y)` tính ngược chiều kim đồng hồ từ trục `+Y` là
                `atan2(-x, y)` (vì tại trục `+Y`, `-x = 0, y > 0` cho góc `0 rad`).
                Trừ đi góc trong tam giác `atan2(L2_eff * sin(e), L1 + L2_eff * cos(e))`.
        """
        if len(xyz) != 3:
            raise ValueError("XYZ cần ba số hữu hạn, đơn vị mm.")
        for coord in xyz:
            if not math.isfinite(coord):
                raise ValueError("XYZ cần ba số hữu hạn, đơn vị mm.")

        x, y, z = xyz
        tx, ty = self.tool_offset_mm

        # Step 1: Effective forearm length and tool angular offset
        # Bước 1: Tính chiều dài khâu 2 hiệu dụng và góc lệch đầu bút
        length = math.hypot(tx, self.l2_mm + ty)
        delta = math.atan2(-tx, self.l2_mm + ty)

        # Step 2: Law of Cosines for the exterior elbow angle `e`
        # Bước 2: Định lý Cosin tìm cosin của góc khuỷu hiệu dụng `e`
        radial_dist_sq = x * x + y * y
        cosine = (radial_dist_sq - self.l1_mm**2 - length**2) / (2 * self.l1_mm * length)
        if abs(cosine) > 1 + 1e-10 or math.hypot(x, y) < 1e-8:
            raise ValueError("Điểm ngoài vùng với tới của robot.")

        # Step 3: Retain the configured elbow branch; never flip across the straight-arm singularity
        # Bước 3: Giữ nguyên nhánh khuỷu tay đã cấu hình; không bao giờ đảo nhánh cắt qua điểm kỳ dị
        clamped_cos = max(-1, min(1, cosine))
        e = self._elbow_branch(delta) * math.acos(clamped_cos)

        # Step 4: Solve shoulder angle `a` relative to the +Y axis (`atan2(-x, y)`)
        # Bước 4: Giải góc vai `a` tính từ trục chuẩn +Y (`atan2(-x, y)`)
        target_angle_from_plus_y = math.atan2(-x, y)
        triangle_offset_angle = math.atan2(
            length * math.sin(e), self.l1_mm + length * math.cos(e)
        )
        a_rad = target_angle_from_plus_y - triangle_offset_angle
        a_deg = math.degrees(math.atan2(math.sin(a_rad), math.cos(a_rad)))
        q2_deg = math.degrees(e - delta)

        # Step 5: Test 360-degree wrap candidates and pick the valid one closest to `near[0]`
        # Bước 5: Kiểm tra các nghiệm chu kỳ 360 độ và chọn nghiệm hợp lệ gần góc `near[0]` nhất
        candidates = []
        for k in (-1, 0, 1):
            candidates.append((a_deg + 360 * k, q2_deg, z))

        valid = []
        for q in candidates:
            try:
                self.check_pose(q)
            except ValueError:
                continue
            valid.append(q)

        if not valid:
            raise ValueError("Điểm vượt giới hạn hoặc gần kỳ dị trên nhánh home.")

        if near is None:
            near = self._default_near(z)
        return min(valid, key=lambda q: abs(q[0] - near[0]))

    def to_steps(self, q):
        """
        [EN] Convert joint coordinates `q = (q1_deg, q2_deg, z_mm)` into integer motor
             step counts `(s_z, s_j1, s_j2)` including belt coupling `coupling * delta_q1`.
             Belt coupling formula:
             Because the J2 timing belt routes over a passive idler pulley on the shoulder,
             rotating shoulder J1 by `delta_q1` while Motor B is stationary causes the forearm
             to change its relative angle by `-coupling * delta_q1`. Therefore, to achieve a
             desired relative elbow change `delta_q2`, Motor B must turn by:
             `delta_B = delta_q2 + coupling * delta_q1`.
        [VI] Quy đổi tọa độ khớp `q = (q1_deg, q2_deg, z_mm)` sang số nguyên xung động cơ
             `(s_z, s_j1, s_j2)` có tính bù truyền động dây đai `coupling * delta_q1`.
             Công thức bù dây đai liên động:
             Do dây đai trục khuỷu J2 vắt qua puly trung gian trên khớp vai, khi khớp vai J1
             quay một góc `delta_q1` sẽ kéo khâu 2 lệch đi `-coupling * delta_q1`. Vì vậy, để
             khâu 2 đạt góc tương đối `delta_q2`, động cơ B phải quay tổng cộng:
             `delta_B = delta_q2 + coupling * delta_q1`.
        """
        self.check_pose(q)
        delta_z_mm = q[2] - self.home_z_mm
        delta_q1_deg = q[0] - self.home_deg[0]
        delta_q2_deg = q[1] - self.home_deg[1]
        delta_motor_b_deg = delta_q2_deg + self.coupling * delta_q1_deg

        axes = (delta_z_mm, delta_q1_deg, delta_motor_b_deg)
        result = []
        for axis_displacement, factor in zip(axes, self.factors):
            step_count = round_nearest(axis_displacement * factor)
            if abs(step_count) > 1_000_000:
                raise ValueError("Tọa độ motor vượt phạm vi firmware.")
            result.append(step_count)
        return tuple(result)

    def from_steps(self, steps):
        """
        [EN] Convert motor step counts `(s_z, s_j1, s_j2)` back into physical joint
             coordinates `(q1_deg, q2_deg, z_mm)`, inverting the belt coupling:
             `delta_q2 = delta_B - coupling * delta_A`.
        [VI] Chuyển đổi ngược từ số xung động cơ `(s_z, s_j1, s_j2)` ra tọa độ khớp
             vật lý `(q1_deg, q2_deg, z_mm)`, khử lượng bù dây đai liên động:
             `delta_q2 = delta_B - coupling * delta_A`.
        """
        f_z, f_j1, f_j2 = self.factors
        delta_z_mm = steps[0] / f_z
        delta_a_deg = steps[1] / f_j1
        delta_b_deg = steps[2] / f_j2

        q1_deg = self.home_deg[0] + delta_a_deg
        q2_deg = self.home_deg[1] + delta_b_deg - self.coupling * delta_a_deg
        z_mm = self.home_z_mm + delta_z_mm
        return (q1_deg, q2_deg, z_mm)

    def plan(self, start_steps, xyz, speed_mm_s=5.0):
        """
        [EN] Plan and preflight a point-to-point `MOVE` from `start_steps` to `xyz`:
             - Solves IK, quantizes to integer steps, and checks every single emitted
               DDA pulse (`for _ in range(ticks)`) against soft limits and singularities!
        [VI] Quy hoạch và kiểm tra an toàn trước khi chạy (preflight) cho lệnh `MOVE`
             từ `start_steps` tới `xyz`:
             - Giải IK, lượng tử hóa ra số bước nguyên, và mô phỏng kiểm tra TỪNG XUNG
               DDA (`for _ in range(ticks)`) để đảm bảo toàn bộ đường đi đều an toàn!
        """
        if not math.isfinite(speed_mm_s) or not (0.05 <= speed_mm_s <= 50):
            raise ValueError("Tốc độ đầu công tác cần 0,05–50 mm/s.")

        # 1. Resolve start pose and quantized target pose
        # 1. Tính tư thế khớp xuất phát và tư thế đích sau khi làm tròn thành số nguyên xung
        q0 = self.from_steps(start_steps)
        self.check_pose(q0)
        q1_ideal = self.ik(xyz, q0)
        steps = self.to_steps(q1_ideal)
        q1 = self.from_steps(steps)
        self.check_pose(q1)

        delta = (
            steps[0] - start_steps[0],
            steps[1] - start_steps[1],
            steps[2] - start_steps[2],
        )
        ticks = max(abs(delta[0]), abs(delta[1]), abs(delta[2]))
        if ticks > 1_000_000:
            raise ValueError("Đoạn vượt 1.000.000 xung/trục; chọn đích gần hơn.")
        if not ticks:
            return MovePlan(steps, 50, (self.fk(q0),), q1, 0.0)

        # 2. Verify that the joint interval [q0, q1] does not cross a 0/180 deg singularity
        # 2. Kiểm tra khoảng biến thiên góc khuỷu từ q0 đến q1 không cắt qua điểm kỳ dị 0/180 độ
        tx_mm, ty_mm = self.tool_offset_mm
        effective_delta = math.degrees(math.atan2(-tx_mm, self.l2_mm + ty_mm))
        elo, ehi = sorted((q0[1] + effective_delta, q1[1] + effective_delta))
        if math.floor(elo / 180) != math.floor(ehi / 180):
            raise ValueError("Đường đi cắt tư thế kỳ dị.")

        # 3. Compute upper bound on Cartesian tip travel to bound pulse rate `rate`
        # 3. Tính cận trên quãng đường di chuyển đầu bút để giới hạn tần số xung `rate`
        length = math.hypot(tx_mm, self.l2_mm + ty_mm)
        v1 = self.l1_mm * math.radians(q1[0] - q0[0])
        v2 = length * math.radians(q1[0] + q1[1] - q0[0] - q0[1])

        bound_lo = math.sqrt(
            max(0.0, v1 * v1 + v2 * v2 + 2.0 * v1 * v2 * math.cos(math.radians(elo)))
        )
        bound_hi = math.sqrt(
            max(0.0, v1 * v1 + v2 * v2 + 2.0 * v1 * v2 * math.cos(math.radians(ehi)))
        )
        bound = max(bound_lo, bound_hi)

        # Required move duration is the maximum of tip speed time and individual axis speed limits
        # Thời gian di chuyển tối thiểu là giá trị lớn nhất giữa thời gian theo vận tốc đầu bút và giới hạn từng trục
        dz_mm = q1[2] - q0[2]
        time_tip_s = math.hypot(bound, dz_mm) / speed_mm_s
        time_z_s = abs(dz_mm) / self.max_z_speed_mm_s
        time_j1_s = abs(q1[0] - q0[0]) / self.max_joint_speed_deg_s[0]
        time_j2_s = abs(q1[1] - q0[1]) / self.max_joint_speed_deg_s[1]
        duration = max(time_tip_s, time_z_s, time_j1_s, time_j2_s)

        # Pure Z moves can run up to the Z speed limit (capped at 6400 pps); arm moves use `max_master_rate`
        # Lệnh chỉ chạy trục Z được phép nâng tần số tới giới hạn trục Z (tối đa 6400 xung/s)
        is_pure_z_move = delta[1] == 0 and delta[2] == 0
        if is_pure_z_move:
            z_max_pps = round(self.factors[0] * self.max_z_speed_mm_s)
            ceiling = min(6400, max(self.max_master_rate, z_max_pps))
        else:
            ceiling = self.max_master_rate

        rate = min(ceiling, math.floor(ticks / max(duration, 1e-9)))
        if rate < 50:
            raise ValueError(
                "Tốc độ yêu cầu thấp hơn nhịp tối thiểu 50 xung/s; chỉnh tốc độ/vi bước."
            )
        if ticks > 590 * rate:
            raise ValueError("Đoạn quá lâu (giới hạn 600 s); chọn đích gần hơn.")

        # 4. Simulate every emitted Bresenham DDA step to verify rounding never steps out of bounds
        # 4. Mô phỏng từng bước xung DDA (thuật toán Bresenham) để chắc chắn sai số làm tròn không vượt giới hạn mềm
        position = list(start_steps)
        accum = [0, 0, 0]
        for _ in range(ticks):
            for axis_idx in range(3):
                accum[axis_idx] += abs(delta[axis_idx])
                if accum[axis_idx] >= ticks:
                    accum[axis_idx] -= ticks
                    if delta[axis_idx] > 0:
                        position[axis_idx] += 1
                    else:
                        position[axis_idx] -= 1
            self.check_pose(self.from_steps(position))

        # 5. Sample 101 points along the joint-space trajectory for the 2D/3D GUI preview
        # 5. Lấy mẫu 101 điểm dọc theo quỹ đạo khớp để vẽ đường xem trước trên giao diện
        path = []
        for i in range(101):
            fraction = i / 100
            q_interp = (
                q0[0] + (q1[0] - q0[0]) * fraction,
                q0[1] + (q1[1] - q0[1]) * fraction,
                q0[2] + (q1[2] - q0[2]) * fraction,
            )
            self.check_pose(q_interp)
            path.append(self.fk(q_interp))

        return MovePlan(steps, rate, tuple(path), q1, ticks / rate)


@dataclass(frozen=True)
class MovePlan:
    """
    [EN] Result of `RobotConfig.plan()` containing target steps, pulse rate, and 3D preview path.
    [VI] Kết quả quy hoạch một lệnh di chuyển gồm đích xung, tốc độ xung và đường xem trước.
    """

    steps: tuple
    rate: int
    path: tuple
    joints: tuple
    minimum_seconds: float


def measured_coupling(shoulder_delta, relative_elbow_delta):
    """
    [EN] Calculate belt coupling coefficient `k = -relative_elbow_delta / shoulder_delta`
         when Motor B is held fixed and shoulder J1 is rotated by `shoulder_delta`.
    [VI] Tính hệ số liên động dây đai `k = -relative_elbow_delta / shoulder_delta`
         khi khóa cứng Động cơ B và quay khớp vai J1 một góc `shoulder_delta`.
    """
    if not (math.isfinite(shoulder_delta) and math.isfinite(relative_elbow_delta)):
        raise ValueError("Cần đo DOF1 ít nhất 1° và nhập hai góc có dấu.")
    if abs(shoulder_delta) < 1:
        raise ValueError("Cần đo DOF1 ít nhất 1° và nhập hai góc có dấu.")

    result = -relative_elbow_delta / shoulder_delta
    if abs(result) > 2:
        raise ValueError("Hệ số đo vượt phạm vi cấu hình.")
    return result
