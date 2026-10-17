"""
============================================================================
FILE: cartesian_nc_model.py
============================================================================
[EN] NC Limit-Switch Calibration Configuration & Measured Robot Model Builder
     - `NCConfig`: Stores all physical switch angles, homing speeds, scan/backoff
       pulse budgets, and serializes them into Protocol-V5 configuration commands
       (`GEOM`, `SPAN`, `PARK`, `TUNE`, `POL`, `COUPLE`).
     - `MeasuredModel`: Subclasses `RobotConfig` using the exact pulse-per-mm/deg
       scale factors (`status.factor`) and travel spans (`status.range`, `status.span_md`)
       measured by the STM32 firmware during `INIT`.

[VI] Cấu hình Dò Gốc Công tắc Thường Đóng (NC) & Mô hình Robot Thực đo
     - `NCConfig`: Lưu trữ các góc đặt công tắc hành trình, vận tốc dò gốc, giới hạn
       quét/lùi xung, và xuất ra danh sách lệnh cấu hình Giao thức V5 gửi xuống mạch
       (`GEOM`, `SPAN`, `PARK`, `TUNE`, `POL`, `COUPLE`).
     - `MeasuredModel`: Kế thừa `RobotConfig` nhưng sử dụng trực tiếp hệ số xung
       (`status.factor`) và hành trình thực tế (`status.range`, `status.span_md`)
       do vi điều khiển STM32 đo được sau khi chạy xong `INIT`.
============================================================================
"""

from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math
from cartesian_model import RobotConfig, round_nearest


@dataclass(frozen=True)
class NCConfig:
    """
    [EN] Complete configuration for the 3-axis NC-switch SCARA robot.
    [VI] Cấu hình toàn diện cho robot SCARA 3 trục dùng công tắc hành trình thường đóng (NC).
    """

    l1_mm: float = 98.0
    l2_mm: float = 98.0
    tool_offset_mm: tuple = (0.0, 0.0)  # [EN] Local +X: right, +Y: along arm 2 / [VI] Độ lệch đầu bút (mm)
    lead_mm_rev: float = 2.0
    z_span_mm: float = 0.0  # [EN] 0: unmeasured, infer from lead_mm_rev / [VI] 0: tính theo bước vít me
    joint_limits_deg: tuple = ((-90.0, 90.0), (-90.0, 90.0))
    park_deg: tuple = (0.0, 45.0)
    z_clear_mm: float = 3.0
    microsteps: tuple = (16, 16, 8)
    ratios: tuple = (3.0, 9.0)
    positive_high: tuple = (
        True,
        True,
        False,
    )  # [EN] Z, J1, J2; opposite physical arm motor signs / [VI] Cực tính chân DIR của Z, J1, J2
    coupling: float = 0.333333  # [EN] Passive pulley on shoulder: relative elbow = B - A/3 / [VI] Hệ số truyền đai chéo
    auto_coupling: bool = True
    coupling_probe_pulses: int = 128
    arm_home_pps: int = 200
    j2_home_pps: int = 800
    j2_latch_pps: int = 200
    z_home_speed_mm_s: float = 2.0
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
    backlash_pulses: tuple = (0, 0)

    @property
    def z_home_pps(self):
        """
        [EN] Convert physical Z homing speed (`z_home_speed_mm_s`) into pulses/second.
             Formula: `(200 full_steps/rev * microsteps_Z / lead_mm_rev) * z_home_speed_mm_s`.
        [VI] Quy đổi vận tốc dò gốc Z (`mm/s`) sang tần số xung (`xung/giây`).
             Công thức: `(200 bước/vòng * vi_bước_Z / bước_vít_me_mm) * tốc_độ_mm_s`.
        """
        z_pulses_per_mm = 200 * self.microsteps[0] / self.lead_mm_rev
        return round_nearest(z_pulses_per_mm * self.z_home_speed_mm_s)

    @property
    def reach_mm(self):
        """
        [EN] Maximum radial reach of the pen tip in the XY plane (in mm).
        [VI] Bán kính vươn tối đa của đầu bút trong mặt phẳng XY (đơn vị mm).
        """
        tx_mm, ty_mm = self.tool_offset_mm
        effective_l2_mm = math.hypot(tx_mm, self.l2_mm + ty_mm)
        return self.l1_mm + effective_l2_mm

    def validate(self):
        """
        [EN] Validate all geometry, homing speeds, and scan/backoff pulse budgets.
        [VI] Kiểm tra tính hợp lệ của toàn bộ thông số hình học, tốc độ dò gốc và giới hạn xung.
        """
        # 1. Reuse `RobotConfig.validate()` to verify link lengths, ratios, microsteps, and park pose
        # 1. Tái sử dụng `RobotConfig.validate()` để kiểm tra chiều dài khâu, tỷ số truyền, vi bước và tư thế đỗ
        RobotConfig(
            l1_mm=self.l1_mm,
            l2_mm=self.l2_mm,
            tool_offset_mm=self.tool_offset_mm,
            home_deg=self.park_deg,
            joint_limits_deg=self.joint_limits_deg,
            home_z_mm=0,
            z_limits_mm=(0, 1000),
            ratios=(self.lead_mm_rev, *self.ratios),
            microsteps=self.microsteps,
            positive_high=self.positive_high,
            coupling=self.coupling,
            max_master_rate=self.max_master_rate,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_z_speed_mm_s=self.max_z_speed_mm_s,
        ).validate()

        # 2. Validate belt coupling and automatic probe parameters
        # 2. Kiểm tra hệ số liên động dây đai và thông số dò tự động
        if abs(self.coupling) > 2:
            raise ValueError("Hệ số liên động cần trong -2 đến 2.")

        if (
            type(self.auto_coupling) is not bool
            or type(self.coupling_probe_pulses) is not int
            or not (16 <= self.coupling_probe_pulses <= 256)
        ):
            raise ValueError(
                "Đo bù tự động cần true/false và bước thử J1 trong 16–256 xung."
            )

        # 3. Validate J1/J2 belt backlash compensation pulse counts
        # 3. Kiểm tra số xung bù độ rơ dây đai cho 2 khớp J1 và J2
        if not isinstance(self.backlash_pulses, (tuple, list)) or len(self.backlash_pulses) != 2:
            raise ValueError(
                "Bù độ rơ dây đai J1/J2 cần 2 số nguyên trong khoảng 0–256 xung."
            )
        for b in self.backlash_pulses:
            if type(b) is not int or not (0 <= b <= 256):
                raise ValueError(
                    "Bù độ rơ dây đai J1/J2 cần 2 số nguyên trong khoảng 0–256 xung."
                )
        object.__setattr__(
            self,
            "backlash_pulses",
            (int(self.backlash_pulses[0]), int(self.backlash_pulses[1])),
        )

        # 4. Validate physical switch angles and park posture margins
        # 4. Kiểm tra góc đặt công tắc hành trình và khoảng cách an toàn của tư thế đỗ (Park)
        for lo, hi in self.joint_limits_deg:
            span_deg = hi - lo
            if not (-180 <= lo < hi <= 180) or not (10 <= span_deg <= 360):
                raise ValueError(
                    "Giới hạn góc mỗi khớp cần đúng vị trí công tắc; khoảng 10–360°."
                )
        for p, (lo, hi) in zip(self.park_deg, self.joint_limits_deg):
            if not (lo + 2 < p < hi - 2):
                raise ValueError("Tư thế đỗ phải cách biên hơn 2°.")
        if not (10 <= abs(self.park_deg[1]) <= 170):
            raise ValueError("Góc đỗ J2 cần cách tư thế thẳng/gập.")

        # 5. Validate Z-axis span, clearance, and homing/latching speeds
        # 5. Kiểm tra hành trình trục Z, khoảng lùi an toàn và tốc độ dò/chốt công tắc
        for z_val in (self.z_span_mm, self.z_clear_mm):
            if type(z_val) not in (int, float) or not math.isfinite(z_val):
                raise ValueError("Thông số Z cần hữu hạn.")

        if not (0 <= self.z_span_mm <= 2000) or not (0.1 <= self.z_clear_mm <= 100):
            raise ValueError("Sai hành trình Z hoặc khoảng lùi khỏi biên trên.")

        if (
            type(self.z_home_speed_mm_s) not in (int, float)
            or not math.isfinite(self.z_home_speed_mm_s)
            or not (0.1 <= self.z_home_speed_mm_s <= 10)
        ):
            raise ValueError("Tốc độ calib Z cần 0,1–10 mm/s.")

        if self.z_home_pps > 6400:
            raise ValueError(
                f"Calib Z cần {self.z_home_pps} xung/s, vượt giới hạn 6400; giảm tốc độ hoặc kiểm tra lead/vi bước."
            )

        homing_rate_limits = (
            (self.arm_home_pps, 800),
            (self.z_home_pps, 6400),
            (self.j2_home_pps, 1600),
        )
        for rate, hi in homing_rate_limits:
            if type(rate) is not int or not (100 <= rate <= hi):
                raise ValueError("Tốc độ home ngoài phạm vi.")

        if type(self.z_latch_pps) is not int or not (50 <= self.z_latch_pps <= self.z_home_pps):
            raise ValueError("Tốc độ chốt biên Z cần 50 xung/s đến tốc độ quét Z.")

        if type(self.j2_latch_pps) is not int or not (50 <= self.j2_latch_pps <= self.j2_home_pps):
            raise ValueError("Tốc độ chốt biên J2 cần 50 xung/s đến tốc độ quét J2.")

        # 6. Validate background execution policy and scan/backoff pulse budgets
        # 6. Kiểm tra cờ chạy ngầm và ngân sách xung tìm biên / lùi nhả công tắc
        if type(self.home_in_background) is not bool:
            raise ValueError("home_in_background cần true/false.")
        if type(self.motion_in_background) is not bool:
            raise ValueError("motion_in_background cần true/false.")

        scan_backoff_pairs = (
            (self.arm_scan_pulses, self.arm_backoff_pulses),
            (self.z_scan_pulses, self.z_backoff_pulses),
        )
        for scan, back in scan_backoff_pairs:
            if (
                type(scan) is not int
                or type(back) is not int
                or not (256 <= scan <= 1000000)
                or not (32 <= back <= 8192)
                or back >= scan
            ):
                raise ValueError("Sai ngân sách tìm biên hoặc lùi công tắc.")

        # 7. Verify arm pulse factors and coupled step ratio stay within firmware integer limits
        # 7. Kiểm tra hệ số xung 2 khâu tay máy và tỷ số bù liên động nằm trong giới hạn của Firmware
        f_j1 = 200 * self.microsteps[1] * self.ratios[0] / 360
        f_j2 = 200 * self.microsteps[2] * self.ratios[1] / 360
        arm_factors = [f_j1, f_j2]
        if (
            not (1 <= f_j1 <= 976)
            or not (1 <= f_j2 <= 976)
            or abs(self.coupling * arm_factors[1] / arm_factors[0]) > 64
        ):
            raise ValueError("Tỷ số xung liên động vượt phạm vi firmware.")
        return self

    @classmethod
    def load(cls, path):
        """
        [EN] Load and validate an `NCConfig` JSON file from `path`.
        [VI] Đọc và kiểm tra file cấu hình JSON `NCConfig` từ đường dẫn `path`.
        """
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data):
        """
        [EN] Construct and validate an `NCConfig` from a dictionary, filling in backward-compatible
             defaults for fields added in newer software revisions.
        [VI] Khởi tạo và kiểm tra `NCConfig` từ dictionary, tự động điền giá trị mặc định tương thích
             ngược cho các trường mới bổ sung ở các phiên bản phần mềm sau.
        """
        if isinstance(data, dict):
            data = dict(data)
            legacy_z_home_pps = data.pop("z_home_pps", None)
            data.setdefault("z_home_speed_mm_s", 2.0)
            data.setdefault("tool_offset_mm", [0.0, 0.0])
            default_z_latch = min(400, legacy_z_home_pps if legacy_z_home_pps is not None else 1600)
            data.setdefault("z_latch_pps", default_z_latch)
            data.setdefault("home_in_background", True)
            data.setdefault("motion_in_background", True)
            data.setdefault("j2_home_pps", 800)
            data.setdefault("j2_latch_pps", 200)
            data.setdefault("auto_coupling", True)
            data.setdefault("coupling_probe_pulses", 128)
            data.setdefault("backlash_pulses", (0, 0))
        if not isinstance(data, dict) or set(data) != set(cls.__dataclass_fields__):
            raise ValueError("File cấu hình NC thiếu hoặc thừa trường.")
        return cls(**data).validate()

    def save(self, path):
        """
        [EN] Validate and save this `NCConfig` to a formatted UTF-8 JSON file.
        [VI] Kiểm tra tính hợp lệ và lưu `NCConfig` ra file JSON định dạng UTF-8.
        """
        self.validate()
        json_text = json.dumps(asdict(self), indent=2, ensure_ascii=False) + "\n"
        Path(path).write_text(json_text, encoding="utf-8")

    def commands(self, session):
        """
        [EN] Generate the sequence of ASCII setup commands (`GEOM`, `SPAN`, `PARK`,
             `TUNE`, `POL`, `COUPLE`, and optional `BACKLASH`) required by the
             firmware before `INIT`.
             Fixed-point scaling used on the wire (so the STM32 needs no float math):
             - `coupling_ppm`: `round_nearest(coupling * 1_000_000)` (parts per million)
             - `factor_q10`  : `round_nearest(factor * 1024)` (Q10 fixed-point, `1024 = 2^10`)
             - `angle_mdeg`  : `round_nearest(deg * 1000)` (millidegrees)
             - `length_um`   : `round_nearest(mm * 1000)` (micrometers)
        [VI] Sinh danh sách các lệnh cài đặt ASCII (`GEOM`, `SPAN`, `PARK`, `TUNE`,
             `POL`, `COUPLE` và `BACKLASH` nếu có) để gửi xuống vi điều khiển
             trước khi chạy lệnh `INIT`.
             Quy ước số nguyên dấu phẩy tĩnh gửi xuống STM32:
             - `coupling_ppm`: `round_nearest(coupling * 1_000_000)` (phần triệu)
             - `factor_q10`  : `round_nearest(factor * 1024)` (dấu phẩy tĩnh Q10, `1024 = 2^10`)
             - `angle_mdeg`  : `round_nearest(deg * 1000)` (mili-độ)
             - `length_um`   : `round_nearest(mm * 1000)` (micrô-mét)
        """
        self.validate()
        s = session

        # 1. `GEOM`: coupling (ppm), nominal pulse factors (Q10 = *1024), and microsteps
        # 1. Lệnh `GEOM`: hệ số liên động (ppm), hệ số xung danh định (*1024), và vi bước
        f_z = 200 * self.microsteps[0] / self.lead_mm_rev
        f_j1 = 200 * self.microsteps[1] * self.ratios[0] / 360
        f_j2 = 200 * self.microsteps[2] * self.ratios[1] / 360
        factors = (f_z, f_j1, f_j2)

        coupling_ppm = round_nearest(self.coupling * 1000000)
        factors_q10_str = " ".join(str(round_nearest(f * 1024)) for f in factors)
        microsteps_str = " ".join(str(m) for m in self.microsteps)
        geom_cmd = f"GEOM {s} {coupling_ppm} {factors_q10_str} {microsteps_str}"

        # 2. `SPAN`: switch angles (millidegrees) and nominal Z travel (micrometers)
        # 2. Lệnh `SPAN`: góc đặt công tắc (mili-độ) và hành trình trục Z (micrô-mét)
        j1_lo_md = round_nearest(self.joint_limits_deg[0][0] * 1000)
        j1_hi_md = round_nearest(self.joint_limits_deg[0][1] * 1000)
        j2_lo_md = round_nearest(self.joint_limits_deg[1][0] * 1000)
        j2_hi_md = round_nearest(self.joint_limits_deg[1][1] * 1000)
        z_span_um = round_nearest(self.z_span_mm * 1000)
        span_cmd = f"SPAN {s} {j1_lo_md} {j1_hi_md} {j2_lo_md} {j2_hi_md} {z_span_um}"

        # 3. `PARK`: post-homing park angles (millidegrees) and Z top clearance (micrometers)
        # 3. Lệnh `PARK`: góc đỗ sau khi về gốc (mili-độ) và khoảng cách lùi Z (micrô-mét)
        park_j1_md = round_nearest(self.park_deg[0] * 1000)
        park_j2_md = round_nearest(self.park_deg[1] * 1000)
        z_clear_um = round_nearest(self.z_clear_mm * 1000)
        park_cmd = f"PARK {s} {park_j1_md} {park_j2_md} {z_clear_um}"

        # 4. `TUNE`: homing/latching pulse rates and scan/backoff pulse budgets
        # 4. Lệnh `TUNE`: tần số xung dò/chốt công tắc và giới hạn xung quét/lùi
        tune_cmd = (
            f"TUNE {s} {self.arm_home_pps} {self.z_home_pps} "
            f"{self.arm_scan_pulses} {self.z_scan_pulses} "
            f"{self.arm_backoff_pulses} {self.z_backoff_pulses} "
            f"{self.z_latch_pps} {self.j2_home_pps} {self.j2_latch_pps}"
        )

        # 5. `POL`: stepper DIR pin polarity for each axis (`Z`, `J1`, `J2`)
        # 5. Lệnh `POL`: cực tính chân DIR cho từng trục (`Z`, `J1`, `J2`)
        pol_cmds = []
        for axis_name, is_pos_high in zip(("Z", "J1", "J2"), self.positive_high):
            pol_cmds.append(f"POL {s} {axis_name} {int(is_pos_high)}")

        # 6. `COUPLE`: automatic belt coupling probe enable flag and pulse count
        # 6. Lệnh `COUPLE`: bật/tắt tự động nhích đo tỷ số truyền đai và số xung thử
        couple_cmd = f"COUPLE {s} {int(self.auto_coupling)} {self.coupling_probe_pulses}"

        cmds = [geom_cmd, span_cmd, park_cmd, tune_cmd, *pol_cmds, couple_cmd]

        # 7. Optional `BACKLASH`: belt reversal compensation pulses for J1 and J2
        # 7. Lệnh `BACKLASH` (tùy chọn): số xung bù độ rơ đảo chiều cho J1 và J2
        if self.backlash_pulses[0] != 0 or self.backlash_pulses[1] != 0:
            b1, b2 = self.backlash_pulses
            cmds.append(f"BACKLASH {s} {b1} {b2}")

        return cmds

    def model(self, status):
        """
        [EN] Construct a calibrated `MeasuredModel` from a referenced firmware `status` packet.
             Unpacks Q10 fixed-point `status.factor` (`f / 1024`) and applies a 2-degree
             soft safety margin inside the physical limit switches.
        [VI] Khởi tạo đối tượng `MeasuredModel` từ gói tin trạng thái `status` sau khi đã dò gốc.
             Giải mã hệ số xung Q10 (`f / 1024`) và đặt biên mềm an toàn cách công tắc 2 độ.
        """
        self.validate()
        if not status.referenced:
            raise ValueError("Chưa hoàn thành HOME + CALIB.")
        for f in status.factor:
            if f <= 0:
                raise ValueError("Chưa hoàn thành HOME + CALIB.")
        for n in status.range:
            if n < 64:
                raise ValueError("Chưa hoàn thành HOME + CALIB.")

        # Convert Q10 fixed-point factors back to floating-point pulses per mm/deg
        # Chuyển đổi hệ số xung từ định dạng Q10 (nhân 1024) về số thực xung/mm và xung/độ
        factors = tuple(f / 1024 for f in status.factor)
        upper = status.range[0] / factors[0]
        margin = max(0.1, 2 / factors[0])
        if upper <= 2 * margin:
            raise ValueError("Hành trình Z đo được quá ngắn.")

        # Determine elbow branch (+1 or -1) from the configured park posture
        # Xác định nhánh khuỷu tay (+1 hoặc -1) dựa theo góc đỗ `park_deg[1]`
        tx_mm, ty_mm = self.tool_offset_mm
        delta = math.atan2(-tx_mm, self.l2_mm + ty_mm)
        if math.sin(math.radians(self.park_deg[1]) + delta) > 0:
            branch = 1
        else:
            branch = -1

        # Compute soft joint limits inset by 2 degrees from the physical switches
        # Tính giới hạn mềm của khớp (lùi vào 2 độ so với vị trí công tắc hành trình vật lý)
        span_md = getattr(status, "span_md", (0, 0))
        joint_limits = []
        for j, (lo, hi) in enumerate(self.joint_limits_deg):
            if len(span_md) > j and span_md[j] > 0:
                hi_eff = lo + span_md[j] / 1000
            else:
                hi_eff = hi
            joint_limits.append((lo + 2, hi_eff - 2))

        return MeasuredModel(
            l1_mm=self.l1_mm,
            l2_mm=self.l2_mm,
            tool_offset_mm=tuple(self.tool_offset_mm),
            home_deg=(0.0, 0.0),
            home_z_mm=0,
            joint_limits_deg=tuple(joint_limits),
            z_limits_mm=(margin, upper - margin),
            ratios=(self.lead_mm_rev, *self.ratios),
            microsteps=self.microsteps,
            positive_high=self.positive_high,
            coupling=status.coupling_ppm / 1000000,
            max_master_rate=self.max_master_rate,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_z_speed_mm_s=self.max_z_speed_mm_s,
            measured_factors=factors,
            branch=branch,
        )


@dataclass(frozen=True)
class MeasuredModel(RobotConfig):
    """
    [EN] Calibrated robot kinematic model using firmware-measured scale factors and spans.
    [VI] Mô hình động học robot đã hiệu chuẩn theo hệ số xung và hành trình thực đo từ Firmware.
    """

    measured_factors: tuple = (1.0, 1.0, 1.0)
    branch: int = 1

    @property
    def factors(self):
        """
        [EN] Return the exact pulse scale factors measured by the STM32 during `INIT`.
        [VI] Trả về hệ số quy đổi xung thực tế do STM32 đo được trong quá trình `INIT`.
        """
        return self.measured_factors

    def _elbow_branch(self, delta):
        """
        [EN] Return the calibrated elbow branch sign (`+1` or `-1`).
        [VI] Trả về dấu nhánh khuỷu tay đã hiệu chuẩn (`+1` hoặc `-1`).
        """
        return self.branch

    def _default_near(self, z):
        """
        [EN] Default reference joint posture `(0, 45 * branch, z)` for IK branch selection.
        [VI] Tư thế góc khớp tham chiếu mặc định `(0, 45 * branch, z)` khi giải động học ngược.
        """
        return (0, 45 * self.branch, z)


def estimate_backlash_from_status(status):
    """
    [EN] Estimate J1 and J2 belt backlash compensation pulses from the forward (`n1`)
         and reverse (`n2`) full-sweep calibration counts reported by firmware.
         Uses `|n1 - n2| // 2` (clamped to `[0..256]`) to isolate one-way mechanical
         reversal take-up from round-trip switch hysteresis.
    [VI] Ước lượng số xung bù độ rơ dây đai J1 và J2 từ chênh lệch giữa số xung quét
         thuận (`n1`) và quét ngược (`n2`) sau khi hiệu chuẩn 2 chiều.
         Lấy `|n1 - n2| // 2` (giới hạn trong `[0..256]`) để tách phần độ rơ đảo chiều
         một phía khỏi độ trễ cơ học hai chiều của công tắc.
    """
    if status is None:
        return (0, 0)

    n1_tuple = getattr(status, "n1", ())
    n2_tuple = getattr(status, "n2", ())
    if len(n1_tuple) < 3 or len(n2_tuple) < 3:
        return (0, 0)

    out = []
    # Axis index 1 is Shoulder (J1), Axis index 2 is Elbow (J2)
    # Chỉ số trục 1 là Khớp vai (J1), chỉ số trục 2 là Khớp khuỷu (J2)
    for axis_idx in (1, 2):
        n1_a = int(n1_tuple[axis_idx])
        n2_a = int(n2_tuple[axis_idx])
        if n1_a < 64 or n2_a < 64:
            out.append(0)
        else:
            diff = abs(n1_a - n2_a)
            if diff >= 2:
                est = diff // 2
            else:
                est = diff
            clamped_est = max(0, min(256, est))
            out.append(clamped_est)

    return (out[0], out[1])
