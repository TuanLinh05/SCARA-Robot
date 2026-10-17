"""
================================================================================
FILE: cartesian_nc_calibration.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Advanced Physical & Geometric Calibration Algorithms + Interactive Wizard:
     1. Auto-Backlash Estimation (`estimate_backlash_from_status`): estimates
        belt reversal take-up pulses from bidirectional homing spans `(n1, n2)`.
     2. 2-Point Geometric Switch-Angle Calibration (`solve_switch_angle_offsets`):
        computes true negative/positive limit-switch angles `joint_limits_deg`
        from 1 or more user-probed reference coordinates `(X_true, Y_true)` on
        millimeter grid paper.
     3. 3-Point Paper Bed Leveling (`fit_paper_plane`): fits a tilted paper
        plane `Z(x, y) = z_ref + kx*(x - x_ref) + ky*(y - y_ref)` from 3 probed
        contact points `(x, y, z)` on the drawing table.

[VI] MỤC ĐÍCH:
     Thuật toán Hiệu chuẩn Cơ khí & Hình học Nâng cao + Trình hướng dẫn Trực quan:
     1. Tự động Ước lượng Độ rơ Dây đai (`estimate_backlash_from_status`): tính
        số xung bù độ rơ đảo chiều từ chênh lệch quét thuận/ngược `(n1, n2)`.
     2. Hiệu chuẩn Góc Công tắc Hành trình 2-Điểm (`solve_switch_angle_offsets`):
        giải động học ngược từ 1 hoặc 2 điểm chuẩn `(X_true, Y_true)` trên giấy
        kẻ ô ly để tìm chính xác góc đặt thực tế của công tắc hành trình.
     3. San phẳng Mặt bàn Giấy 3-Điểm (`fit_paper_plane`): khớp phương trình mặt
        phẳng giấy nghiêng `Z(x, y) = z_ref + kx*(x - x_ref) + ky*(y - y_ref)`
        từ 3 điểm chạm đầu bút trên mặt bàn.
================================================================================
"""

from dataclasses import dataclass, replace
import math
import tkinter as tk
from tkinter import ttk

from cartesian_nc_model import estimate_backlash_from_status
from cartesian_nc_theme import BG, CARD, LINE, TEXT, MUTED, ACCENT, GREEN, RED, BLUE


# ==============================================================================
# 1. GEOMETRIC SWITCH-ANGLE CALIBRATION / HIỆU CHUẨN GÓC CÔNG TẮC 2-ĐIỂM
# ==============================================================================


@dataclass(frozen=True)
class SwitchCalibrationResult:
    """
    [EN] Output of 1-point or 2-point geometric switch-angle calibration.
    [VI] Kết quả tính hiệu chuẩn góc công tắc hành trình từ các điểm chuẩn.
    """

    delta_deg: tuple
    new_joint_limits_deg: tuple
    rms_error_before_mm: float
    rms_error_after_mm: float
    point_errors_before_mm: tuple
    point_errors_after_mm: tuple


def solve_switch_angle_offsets(model, config, samples):
    """
    [EN] Solve the true physical switch angle offsets `(d_theta1, d_theta2)` in
         degrees from 1 or more reference samples `(steps, (x_true_mm, y_true_mm))`.
         Each sample pairs the robot's recorded step tuple `(sz, s1, s2)` with
         the true physical pen-tip coordinates `(x_true_mm, y_true_mm)` measured
         on millimeter grid paper.
         Algorithm:
         1. Convert recorded `steps` -> reported joint angles `q_rep = (q1_rep, q2_rep, z)`.
         2. Solve Inverse Kinematics for `(x_true, y_true)` -> true angles `q_true = (q1_true, q2_true, z)`.
         3. Average the angular offsets `d_theta1 = mean(q1_true - q1_rep)` and
            `d_theta2 = mean(q2_true - q2_rep)`, then shift `joint_limits_deg` by `(d_theta1, d_theta2)`.

    [VI] Giải tìm độ lệch góc đặt công tắc thực tế `(d_theta1, d_theta2)` (đơn vị độ)
         từ 1 hoặc nhiều mẫu điểm chuẩn `(steps, (x_true_mm, y_true_mm))`.
         Mỗi mẫu gồm bộ xung bước `(sz, s1, s2)` lúc đầu bút chạm điểm chuẩn và
         tọa độ thực `(x_true_mm, y_true_mm)` đọc trên giấy kẻ ly.
         Thuật toán:
         1. Đổi bộ xung `steps` -> góc khớp báo cáo hiện tại `q_rep = (q1_rep, q2_rep, z)`.
         2. Giải Động học Ngược từ `(x_true, y_true)` -> góc khớp thực `q_true = (q1_true, q2_true, z)`.
         3. Lấy trung bình độ lệch góc `d_theta1 = mean(q1_true - q1_rep)` và
            `d_theta2 = mean(q2_true - q2_rep)`, sau đó cộng bù vào `joint_limits_deg`.
    """
    if not samples:
        raise ValueError("Cần ít nhất 1 điểm mẫu để hiệu chuẩn góc công tắc.")

    d1_list = []
    d2_list = []
    q_rep_list = []
    target_xy_list = []
    errors_before = []

    for idx, (steps, true_xy) in enumerate(samples):
        if len(steps) != 3 or len(true_xy) < 2:
            raise ValueError(f"Mẫu #{idx + 1} không đúng định dạng (steps, (x, y)).")
        tx, ty = float(true_xy[0]), float(true_xy[1])
        if not (math.isfinite(tx) and math.isfinite(ty)):
            raise ValueError(f"Tọa độ mẫu #{idx + 1} phải là số hữu hạn.")

        # 1. Reported joint angles and Cartesian tip position under current switch limits
        # 1. Góc khớp và tọa độ đầu bút báo cáo theo giới hạn công tắc hiện tại
        int_steps = (int(steps[0]), int(steps[1]), int(steps[2]))
        q_rep = model.from_steps(int_steps)
        xyz_rep = model.fk(q_rep)
        err_before = math.hypot(xyz_rep[0] - tx, xyz_rep[1] - ty)

        # 2. Solve true joint angles via Inverse Kinematics near the reported posture
        # 2. Giải động học ngược tìm góc khớp thực tế gần tư thế hiện tại nhất
        q_true = model.ik((tx, ty, q_rep[2]), near=q_rep)
        d1_list.append(q_true[0] - q_rep[0])
        d2_list.append(q_true[1] - q_rep[1])
        q_rep_list.append(q_rep)
        target_xy_list.append((tx, ty))
        errors_before.append(err_before)

    # 3. Compute mean angular offset for J1 and J2 and verify sanity bounds (<= 15 deg)
    # 3. Tính độ lệch góc trung bình cho J1 và J2, kiểm tra ngưỡng an toàn (<= 15 độ)
    d_theta1 = sum(d1_list) / len(d1_list)
    d_theta2 = sum(d2_list) / len(d2_list)
    if abs(d_theta1) > 15.0 or abs(d_theta2) > 15.0:
        raise ValueError(
            f"Độ lệch góc tính được quá lớn (ΔJ1={d_theta1:+.2f}°, ΔJ2={d_theta2:+.2f}° > ±15°); "
            "hãy kiểm tra lại tọa độ nhập hoặc nhánh khuỷu tay."
        )

    # 4. Evaluate residual Cartesian error after applying `(d_theta1, d_theta2)`
    # 4. Tính sai số vị trí còn lại sau khi cộng bù `(d_theta1, d_theta2)`
    errors_after = []
    for q_rep, (tx, ty) in zip(q_rep_list, target_xy_list):
        q_corr = (q_rep[0] + d_theta1, q_rep[1] + d_theta2, q_rep[2])
        xyz_corr = model.fk(q_corr)
        err_after = math.hypot(xyz_corr[0] - tx, xyz_corr[1] - ty)
        errors_after.append(err_after)

    mean_sq_before = sum(e * e for e in errors_before) / len(errors_before)
    mean_sq_after = sum(e * e for e in errors_after) / len(errors_after)
    rms_before = math.sqrt(mean_sq_before)
    rms_after = math.sqrt(mean_sq_after)

    (lo1, hi1), (lo2, hi2) = config.joint_limits_deg
    new_limits = (
        (round(lo1 + d_theta1, 3), round(hi1 + d_theta1, 3)),
        (round(lo2 + d_theta2, 3), round(hi2 + d_theta2, 3)),
    )

    return SwitchCalibrationResult(
        delta_deg=(round(d_theta1, 4), round(d_theta2, 4)),
        new_joint_limits_deg=new_limits,
        rms_error_before_mm=round(rms_before, 4),
        rms_error_after_mm=round(rms_after, 4),
        point_errors_before_mm=tuple(round(e, 4) for e in errors_before),
        point_errors_after_mm=tuple(round(e, 4) for e in errors_after),
    )


# ==============================================================================
# 2. 3-POINT PAPER BED LEVELING / SAN PHẲNG MẶT PHẲNG GIẤY 3-ĐIỂM
# ==============================================================================


@dataclass(frozen=True)
class PaperPlaneFit:
    """
    [EN] Fitted paper plane `Z(x, y) = z_ref + kx*(x - x_ref) + ky*(y - y_ref)`.
    [VI] Kết quả nội suy mặt phẳng giấy nghiêng qua 3 điểm chạm.
    """

    z_ref_mm: float
    paper_tilt: tuple
    ref_xy: tuple
    tilt_deg: float
    max_residual_mm: float

    def z_at(self, x_mm, y_mm):
        """
        [EN] Evaluate the tilted paper Z height (in mm) at `(x_mm, y_mm)`.
        [VI] Tính độ cao mặt giấy Z (mm) tại tọa độ `(x_mm, y_mm)`.
        """
        kx, ky = self.paper_tilt
        rx, ry = self.ref_xy
        dx = float(x_mm) - rx
        dy = float(y_mm) - ry
        return self.z_ref_mm + kx * dx + ky * dy


def fit_paper_plane(points_xyz, ref_xy=(-69.296, 167.296)):
    """
    [EN] Fit a plane `Z(x, y) = z_ref + kx * (x - x_ref) + ky * (y - y_ref)` through
         3 or more non-collinear contact points `(x_mm, y_mm, z_mm)` using ordinary
         least-squares (OLS) linear regression:
         1. Center coordinates around centroid `(cx, cy, cz)`:
            `dx_i = x_i - cx`, `dy_i = y_i - cy`, `dz_i = z_i - cz`.
         2. Normal equations for minimizing `sum((kx * dx_i + ky * dy_i - dz_i)^2)`:
            `[ sxx  sxy ] [ kx ] = [ sxz ]`
            `[ sxy  syy ] [ ky ]   [ syz ]`
         3. Solve the 2x2 system via Cramer's Rule with determinant `det = sxx * syy - sxy^2`:
            `kx = (syy * sxz - sxy * syz) / det`
            `ky = (sxx * syz - sxy * sxz) / det`

    [VI] Khớp mặt phẳng `Z(x, y) = z_ref + kx * (x - x_ref) + ky * (y - y_ref)` đi qua
         tối thiểu 3 điểm chạm không thẳng hàng `(x_mm, y_mm, z_mm)` bằng phương pháp
         bình phương tối thiểu (Least-Squares):
         1. Dịch gốc tọa độ về trọng tâm `(cx, cy, cz)`:
            `dx_i = x_i - cx`, `dy_i = y_i - cy`, `dz_i = z_i - cz`.
         2. Hệ phương trình chuẩn (Normal Equations) cực tiểu hóa tổng bình phương sai số:
            `[ sxx  sxy ] [ kx ] = [ sxz ]`
            `[ sxy  syy ] [ ky ]   [ syz ]`
         3. Giải hệ 2x2 bằng quy tắc Cramer với định thức `det = sxx * syy - sxy^2`:
            `kx = (syy * sxz - sxy * syz) / det`
            `ky = (sxx * syz - sxy * sxz) / det`
    """
    if len(points_xyz) < 3:
        raise ValueError("Cần ít nhất 3 điểm chạm (x, y, z) để xác định mặt phẳng giấy.")

    pts = []
    for idx, p in enumerate(points_xyz):
        if len(p) != 3:
            raise ValueError(f"Điểm #{idx + 1} cần đủ 3 tọa độ (x, y, z).")
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"Điểm #{idx + 1} chứa giá trị không hữu hạn.")
        pts.append((x, y, z))

    # Step 1: Compute centroid (cx, cy, cz) of the sampled points
    # Bước 1: Tính tọa độ trọng tâm (cx, cy, cz) của tập điểm mẫu
    num_pts = len(pts)
    cx = sum(p[0] for p in pts) / num_pts
    cy = sum(p[1] for p in pts) / num_pts
    cz = sum(p[2] for p in pts) / num_pts

    # Step 2: Accumulate second-moment sums (sxx, syy, sxy, sxz, syz)
    # Bước 2: Tính các tổng tích vô hướng quanh trọng tâm (sxx, syy, sxy, sxz, syz)
    sxx = 0.0
    syy = 0.0
    sxy = 0.0
    sxz = 0.0
    syz = 0.0
    for x, y, z in pts:
        dx = x - cx
        dy = y - cy
        dz = z - cz
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
        sxz += dx * dz
        syz += dy * dz

    # Step 3: Solve 2x2 normal equations via Cramer's Rule (det = sxx * syy - sxy^2)
    # Bước 3: Giải hệ phương trình 2 ẩn bằng định thức Cramer (det = sxx * syy - sxy^2)
    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-6:
        raise ValueError(
            "Ba điểm lấy mẫu trên mặt giấy quá gần nhau hoặc thẳng hàng; hãy chọn 3 góc tạo thành tam giác rộng."
        )

    kx = (syy * sxz - sxy * syz) / det
    ky = (sxx * syz - sxy * sxz) / det
    slope_mag = math.hypot(kx, ky)
    tilt_deg = math.degrees(math.atan(slope_mag))
    if slope_mag > 0.25:
        raise ValueError(
            f"Độ dốc mặt giấy quá lớn ({tilt_deg:.1f}° > 14°); "
            "hãy kiểm tra lại tọa độ Z của 3 điểm."
        )

    # Step 4: Evaluate reference height `z_ref` at `ref_xy` and maximum fit residual
    # Bước 4: Tính độ cao chuẩn `z_ref` tại tâm `ref_xy` và sai số khớp lớn nhất
    rx, ry = float(ref_xy[0]), float(ref_xy[1])
    z_ref = cz + kx * (rx - cx) + ky * (ry - cy)

    residuals = []
    for x, y, z in pts:
        z_pred = z_ref + kx * (x - rx) + ky * (y - ry)
        residuals.append(abs(z_pred - z))

    return PaperPlaneFit(
        z_ref_mm=round(z_ref, 4),
        paper_tilt=(round(kx, 6), round(ky, 6)),
        ref_xy=(round(rx, 3), round(ry, 3)),
        tilt_deg=round(tilt_deg, 3),
        max_residual_mm=round(max(residuals), 5),
    )


# ==============================================================================
# 3. GUI CALIBRATION WIZARD MIXIN / HỘP THOẠI TRÌNH HƯỚNG DẪN HIỆU CHUẨN
# ==============================================================================


class CalibrationWizardMixin:
    """
    [EN] Mixin for `NCApp` providing an interactive 3-tab Calibration Wizard:
         - Tab 1: Auto-Backlash Estimation & Compensation (`BACKLASH`).
         - Tab 2: 2-Point Geometric Switch-Angle Calibration (`joint_limits_deg`).
         - Tab 3: 3-Point Paper Bed Leveling (`paper_tilt` + `paper_z_mm`).
    [VI] Lớp Mixin cho `NCApp` cung cấp Hộp thoại Hiệu chuẩn Nâng cao 3 tab:
         - Tab 1: Tự động đo & bù độ rơ dây đai (`BACKLASH`).
         - Tab 2: Hiệu chuẩn góc đặt công tắc hành trình 2-điểm (`joint_limits_deg`).
         - Tab 3: San phẳng mặt bàn giấy nghiêng 3-điểm (`paper_tilt` + `paper_z_mm`).
    """

    def open_calibration_wizard(self):
        """
        [EN] Open or focus the 3-tab Advanced Calibration Wizard window.
        [VI] Mở hoặc đưa cửa sổ Trình hướng dẫn Hiệu chuẩn Nâng cao lên phía trước.
        """
        win = getattr(self, "calibration_window", None)
        if win is not None and win.winfo_exists():
            win.deiconify()
            win.lift()
            win.focus_force()
            self._refresh_calibration_wizard()
            return

        win = tk.Toplevel(self)
        win.title("SCARA · TRÌNH HƯỚNG DẪN HIỆU CHUẨN NÂNG CAO (CALIBRATION WIZARD)")
        win.geometry("760x620")
        win.configure(bg=BG)
        self.calibration_window = win

        outer = tk.Frame(win, bg=BG, padx=16, pady=12)
        outer.pack(fill="both", expand=True)

        self.label(
            outer,
            "🎯 TRÌNH HƯỚNG DẪN HIỆU CHUẨN CƠ KHÍ & MẶT GIẤY NÂNG CAO",
            14,
            TEXT,
        ).pack(anchor="w")
        self.label(
            outer,
            "Bù độ rơ dây đai từ |n1 − n2|  ·  Chuẩn hóa góc công tắc 2-điểm trên giấy ô ly  ·  San phẳng mặt bàn giấy 3-điểm",
            9,
            MUTED,
        ).pack(anchor="w", pady=(2, 10))

        nb = ttk.Notebook(outer, style="Scara.TNotebook")
        nb.pack(fill="both", expand=True)

        tab_bl = tk.Frame(nb, bg=CARD, padx=14, pady=12)
        tab_sw = tk.Frame(nb, bg=CARD, padx=14, pady=12)
        tab_bed = tk.Frame(nb, bg=CARD, padx=14, pady=12)
        nb.add(tab_bl, text="1 · Bù Độ Rơ Đai (Backlash)")
        nb.add(tab_sw, text="2 · Góc Công Tắc 2-Điểm")
        nb.add(tab_bed, text="3 · Mặt Phẳng Giấy 3-Điểm")

        self._build_backlash_tab(tab_bl)
        self._build_switch_offset_tab(tab_sw)
        self._build_bed_leveling_tab(tab_bed)
        self._refresh_calibration_wizard()

    def _build_backlash_tab(self, parent):
        """
        [EN] Build Tab 1 UI: Auto-Backlash Estimation & Compensation (`BACKLASH`).
        [VI] Xây dựng giao diện Tab 1: Tự động ước lượng và bù độ rơ dây đai (`BACKLASH`).
        """
        self.label(
            parent,
            "TỰ ĐỘNG ƯỚC LƯỢNG & BÙ ĐỘ RƠ DÂY ĐAI (AUTO-BACKLASH)",
            11,
            ACCENT,
        ).pack(anchor="w")
        self.label(
            parent,
            "Khi robot chạy HOME / CALIB toàn diện, vi điều khiển đo số xung quét thuận (n1) và quét ngược (n2).\n"
            "Chênh lệch |n1 − n2| phản ánh độ rơ dây đai + độ trễ công tắc. Hệ thống lấy |n1 − n2| // 2 để đề xuất số xung bù đảo chiều.",
            9,
            MUTED,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(4, 10))

        self.cal_bl_telemetry_var = tk.StringVar(
            value="Chưa có dữ liệu n1/n2 từ lần HOME gần nhất."
        )
        tel_lbl = self.label(
            parent,
            textvariable=self.cal_bl_telemetry_var,
            size=10,
            color="#efc25a",
            wraplength=680,
            justify="left",
        )
        tel_lbl.pack(anchor="w", pady=(0, 10))

        form = tk.Frame(parent, bg=CARD)
        form.pack(fill="x", pady=6)
        self.cal_bl_j1_var = tk.StringVar(value=str(self.config.backlash_pulses[0]))
        self.cal_bl_j2_var = tk.StringVar(value=str(self.config.backlash_pulses[1]))

        r1 = tk.Frame(form, bg=CARD)
        r1.pack(fill="x", pady=3)
        self.label(r1, "Xung bù độ rơ trục Vai J1 (0–256 xung):", 10, TEXT).pack(
            side="left"
        )
        self.entry(r1, self.cal_bl_j1_var).pack(side="right")

        r2 = tk.Frame(form, bg=CARD)
        r2.pack(fill="x", pady=3)
        self.label(r2, "Xung bù độ rơ trục Khuỷu J2 (0–256 xung):", 10, TEXT).pack(
            side="left"
        )
        self.entry(r2, self.cal_bl_j2_var).pack(side="right")

        btn_row = tk.Frame(parent, bg=CARD)
        btn_row.pack(fill="x", pady=(12, 6))
        self.button(
            btn_row,
            "⚡ Lấy tự động từ |n1 − n2| vừa đo",
            self._fill_estimated_backlash,
            "#1f5a73",
        ).pack(side="left", padx=(0, 8))
        self.button(
            btn_row,
            "💾 Lưu & Gửi lệnh BACKLASH xuống STM32",
            self._apply_backlash_calibration,
            "#206c61",
        ).pack(side="left")

        b1_init, b2_init = self.config.backlash_pulses
        self.cal_bl_status_var = tk.StringVar(
            value=f"Đang cấu hình: BACKLASH J1={b1_init} xung, J2={b2_init} xung."
        )
        self.label(
            parent,
            textvariable=self.cal_bl_status_var,
            size=9,
            color=GREEN,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(8, 0))

    def _build_switch_offset_tab(self, parent):
        """
        [EN] Build Tab 2 UI: 2-Point Geometric Switch-Angle Calibration.
        [VI] Xây dựng giao diện Tab 2: Hiệu chuẩn góc công tắc hành trình 2-điểm.
        """
        self.label(
            parent,
            "HIỆU CHUẨN GÓC ĐẶT CÔNG TẮC HÀNH TRÌNH 2-ĐIỂM (SWITCH OFFSET)",
            11,
            ACCENT,
        ).pack(anchor="w")
        self.label(
            parent,
            "Hướng dẫn: Đặt tờ giấy kẻ ô ly trên bàn. Dùng tab Jog đưa đầu bút chạm Điểm chuẩn 1 rồi bấm 'Lấy xung hiện tại', "
            "nhập tọa độ thực (X1, Y1) đọc trên thước/giấy. Lặp lại với Điểm chuẩn 2 (tùy chọn) rồi bấm 'Tính & Cập nhật'.",
            9,
            MUTED,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(4, 8))

        self.cal_sw_samples = [None, None]
        self.cal_sw_step_vars = [
            tk.StringVar(value="Chưa chốt xung"),
            tk.StringVar(value="Chưa chốt xung"),
        ]
        self.cal_sw_xy_vars = [
            (tk.StringVar(value="-60.0"), tk.StringVar(value="150.0")),
            (tk.StringVar(value="40.0"), tk.StringVar(value="150.0")),
        ]

        for idx in range(2):
            box = tk.Frame(
                parent,
                bg="#142338",
                padx=10,
                pady=8,
                highlightbackground=LINE,
                highlightthickness=1,
            )
            box.pack(fill="x", pady=4)
            top = tk.Frame(box, bg="#142338")
            top.pack(fill="x")
            tag_text = "(Bắt buộc)" if idx == 0 else "(Khuyến nghị)"
            self.label(
                top,
                f"Điểm chuẩn #{idx + 1} {tag_text}",
                10,
                TEXT,
            ).pack(side="left")
            self.button(
                top,
                f"📍 Lấy xung từ vị trí Robot hiện tại (#{idx + 1})",
                lambda i=idx: self._capture_switch_sample(i),
                "#24597a",
            ).pack(side="right")
            self.label(
                box, textvariable=self.cal_sw_step_vars[idx], size=9, color="#efc25a"
            ).pack(anchor="w", pady=(4, 4))

            xy_row = tk.Frame(box, bg="#142338")
            xy_row.pack(fill="x")
            self.label(xy_row, "X thực tế (mm):", 9, MUTED).pack(side="left")
            self.entry(xy_row, self.cal_sw_xy_vars[idx][0]).pack(
                side="left", padx=(6, 18)
            )
            self.label(xy_row, "Y thực tế (mm):", 9, MUTED).pack(side="left")
            self.entry(xy_row, self.cal_sw_xy_vars[idx][1]).pack(side="left", padx=6)

        act = tk.Frame(parent, bg=CARD)
        act.pack(fill="x", pady=(8, 4))
        self.button(
            act,
            "🔍 Tính Độ Lệch Góc & Lưu Giới Hạn Công Tắc Mới",
            self._solve_and_apply_switch_offsets,
            "#206c61",
        ).pack(side="left")

        j1_lim, j2_lim = self.config.joint_limits_deg
        self.cal_sw_result_var = tk.StringVar(
            value=f"Giới hạn góc hiện tại: J1={j1_lim}, J2={j2_lim}"
        )
        self.label(
            parent,
            textvariable=self.cal_sw_result_var,
            size=9,
            color=GREEN,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(6, 0))

    def _build_bed_leveling_tab(self, parent):
        """
        [EN] Build Tab 3 UI: 3-Point Paper Bed Leveling (`paper_tilt` + `paper_z_mm`).
        [VI] Xây dựng giao diện Tab 3: San phẳng mặt bàn giấy nghiêng 3-điểm.
        """
        self.label(
            parent,
            "SAN PHẲNG MẶT BÀN GIẤY NGHIÊNG 3-ĐIỂM (3-POINT PAPER BED LEVELING)",
            11,
            ACCENT,
        ).pack(anchor="w")
        self.label(
            parent,
            "Hướng dẫn: Hạ ngòi bút vừa chạm mặt giấy tại 3 góc tạo thành tam giác trên vùng vẽ. "
            "Hệ thống sẽ khớp mặt phẳng Z(x, y) = Z0 + kx·(x − cx) + ky·(y − cy) và tự động bù cao độ Z cho từng nét vẽ!",
            9,
            MUTED,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(4, 8))

        defaults = (
            ("-95.0", "145.0", "12.0"),
            ("-40.0", "145.0", "12.0"),
            ("-69.3", "185.0", "12.0"),
        )
        self.cal_bed_vars = []
        for idx, (dx, dy, dz) in enumerate(defaults):
            row = tk.Frame(
                parent,
                bg="#142338",
                padx=10,
                pady=6,
                highlightbackground=LINE,
                highlightthickness=1,
            )
            row.pack(fill="x", pady=3)
            self.label(row, f"Điểm P{idx + 1} (X, Y, Z mm):", 9, TEXT).pack(side="left")
            vx = tk.StringVar(value=dx)
            vy = tk.StringVar(value=dy)
            vz = tk.StringVar(value=dz)
            self.cal_bed_vars.append((vx, vy, vz))
            self.button(
                row,
                f"📍 Lấy XYZ hiện tại (P{idx + 1})",
                lambda i=idx: self._capture_bed_point(i),
                "#24597a",
            ).pack(side="right")
            for lbl, var in (("Z:", vz), ("Y:", vy), ("X:", vx)):
                self.entry(row, var).pack(side="right", padx=(2, 8))
                self.label(row, lbl, 9, MUTED).pack(side="right")

        act = tk.Frame(parent, bg=CARD)
        act.pack(fill="x", pady=(10, 4))
        self.button(
            act,
            "🔍 Nội suy Mặt Phẳng Giấy & Áp dụng Bù Nghiêng Z",
            self._solve_and_apply_bed_leveling,
            "#206c61",
        ).pack(side="left", padx=(0, 8))
        self.button(
            act,
            "↺ Đặt lại Mặt Giấy Phẳng (kx=0, ky=0)",
            self._reset_bed_leveling,
            "#3b477a",
        ).pack(side="left")

        tilt = getattr(self, "paper_tilt", (0.0, 0.0))
        self.cal_bed_result_var = tk.StringVar(
            value=f"Độ nghiêng mặt giấy hiện tại: kx={tilt[0]:+.5f}, ky={tilt[1]:+.5f} mm/mm."
        )
        self.label(
            parent,
            textvariable=self.cal_bed_result_var,
            size=9,
            color=GREEN,
            wraplength=680,
            justify="left",
        ).pack(anchor="w", pady=(6, 0))

    def _refresh_calibration_wizard(self):
        """
        [EN] Refresh the live `n1`/`n2` telemetry summary in Tab 1.
        [VI] Cập nhật thông tin số xung quét thuận/ngược `n1`/`n2` trên Tab 1.
        """
        if not hasattr(self, "cal_bl_telemetry_var"):
            return
        if self.state and self.state.referenced:
            est = estimate_backlash_from_status(self.state)
            n1, n2 = self.state.n1, self.state.n2
            diff_j1 = abs(n1[1] - n2[1])
            diff_j2 = abs(n1[2] - n2[2])
            self.cal_bl_telemetry_var.set(
                f"Số xung CALIB gần nhất:  J1 (n1={n1[1]}, n2={n2[1]}, |Δ|={diff_j1})  ·  "
                f"J2 (n1={n1[2]}, n2={n2[2]}, |Δ|={diff_j2})\n"
                f"➔ Đề xuất bù độ rơ đảo chiều:  J1 = {est[0]} xung  ·  J2 = {est[1]} xung."
            )
        else:
            self.cal_bl_telemetry_var.set(
                "Robot chưa có dữ liệu HOME / CALIB 2 chiều. Hãy chạy HOME / CALIB trước để tự động đo |n1 − n2|."
            )

    def _fill_estimated_backlash(self):
        """
        [EN] Populate the J1/J2 backlash entry boxes from `estimate_backlash_from_status()`.
        [VI] Tự động điền số xung bù độ rơ J1/J2 ước lượng từ `|n1 - n2|` vào ô nhập.
        """
        self._refresh_calibration_wizard()
        if not self.state or not self.state.referenced:
            self.cal_bl_status_var.set(
                "Cần hoàn thành HOME / CALIB trước khi lấy tự động từ |n1 − n2|."
            )
            return
        est = estimate_backlash_from_status(self.state)
        self.cal_bl_j1_var.set(str(est[0]))
        self.cal_bl_j2_var.set(str(est[1]))
        self.cal_bl_status_var.set(
            f"Đã nạp giá trị ước lượng từ |n1 − n2|: J1={est[0]} xung, J2={est[1]} xung. Bấm Lưu để áp dụng."
        )

    def _apply_backlash_calibration(self):
        """
        [EN] Validate, save, and transmit the `BACKLASH <session> <b1> <b2>` command to STM32.
        [VI] Kiểm tra, lưu file cấu hình và gửi lệnh `BACKLASH <session> <b1> <b2>` xuống STM32.
        """
        try:
            b1 = int(self.cal_bl_j1_var.get())
            b2 = int(self.cal_bl_j2_var.get())
            new_cfg = replace(self.config, backlash_pulses=(b1, b2)).validate()
        except Exception as e:
            self.cal_bl_status_var.set(f"Lỗi: {e}")
            return

        self.config = new_cfg
        self.config.save(self.config_path)
        if self.link and self.idle():
            self.sequence([f"BACKLASH {self.link.session} {b1} {b2}"])
        self.cal_bl_status_var.set(
            f"Đã lưu & áp dụng bù độ rơ dây đai: J1={b1} xung, J2={b2} xung."
        )
        self.render()

    def _capture_switch_sample(self, idx):
        """
        [EN] Record the robot's current step tuple `(sz, s1, s2)` for switch calibration point `idx`.
        [VI] Chốt bộ xung `(sz, s1, s2)` tại vị trí hiện tại của robot cho điểm chuẩn `idx`.
        """
        if not self.editor_ready():
            self.cal_sw_result_var.set(
                "Cần hoàn thành HOME / CALIB trước khi chốt mẫu xung."
            )
            return
        pos = tuple(self.state.pos)
        q = self.model.from_steps(pos)
        xyz = self.model.fk(q)
        self.cal_sw_samples[idx] = pos
        self.cal_sw_step_vars[idx].set(
            f"Đã chốt xung={pos} · Góc báo cáo=({q[0]:+.2f}°, {q[1]:+.2f}°) · XYZ báo cáo=({xyz[0]:+.2f}, {xyz[1]:+.2f}) mm"
        )
        self.cal_sw_xy_vars[idx][0].set(f"{xyz[0]:.2f}")
        self.cal_sw_xy_vars[idx][1].set(f"{xyz[1]:.2f}")

    def _solve_and_apply_switch_offsets(self):
        """
        [EN] Run `solve_switch_angle_offsets()` on the captured samples and update `joint_limits_deg`.
        [VI] Giải độ lệch góc công tắc từ các điểm mẫu đã chốt và cập nhật `joint_limits_deg`.
        """
        if not self.editor_ready():
            self.cal_sw_result_var.set(
                "Cần hoàn thành HOME / CALIB trước khi giải hiệu chuẩn góc công tắc."
            )
            return
        active_samples = []
        try:
            for idx in range(2):
                if self.cal_sw_samples[idx] is not None:
                    tx = float(self.cal_sw_xy_vars[idx][0].get())
                    ty = float(self.cal_sw_xy_vars[idx][1].get())
                    active_samples.append((self.cal_sw_samples[idx], (tx, ty)))
            res = solve_switch_angle_offsets(self.model, self.config, active_samples)
            new_cfg = replace(
                self.config, joint_limits_deg=res.new_joint_limits_deg
            ).validate()
        except Exception as e:
            self.cal_sw_result_var.set(f"Lỗi hiệu chuẩn góc công tắc: {e}")
            return

        self.config = new_cfg
        self.config.save(self.config_path)
        if self.state and self.state.referenced:
            self.model = self.config.model(self.state)
        self.cal_sw_result_var.set(
            f"Thành công! ΔJ1={res.delta_deg[0]:+.3f}°, ΔJ2={res.delta_deg[1]:+.3f}° · "
            f"Sai số RMS: {res.rms_error_before_mm:.3f} mm ➔ {res.rms_error_after_mm:.3f} mm · "
            f"Giới hạn mới: {res.new_joint_limits_deg}"
        )
        self.render()

    def _capture_bed_point(self, idx):
        """
        [EN] Copy the robot's current `(X, Y, Z)` pen-tip position into bed-leveling point `idx`.
        [VI] Lấy tọa độ đầu bút `(X, Y, Z)` hiện tại điền vào điểm chạm mặt giấy `P{idx + 1}`.
        """
        if not self.editor_ready():
            self.cal_bed_result_var.set(
                "Cần hoàn thành HOME / CALIB trước khi lấy tọa độ XYZ điểm chạm."
            )
            return
        q = self.model.from_steps(self.state.pos)
        xyz = self.model.fk(q)
        vx, vy, vz = self.cal_bed_vars[idx]
        vx.set(f"{xyz[0]:.3f}")
        vy.set(f"{xyz[1]:.3f}")
        vz.set(f"{xyz[2]:.3f}")

    def _solve_and_apply_bed_leveling(self):
        """
        [EN] Fit the 3-point paper plane via `fit_paper_plane()` and update `self.paper_tilt` + `paper_z`.
        [VI] Nội suy mặt phẳng giấy qua 3 điểm bằng `fit_paper_plane()` và cập nhật `paper_tilt` + `paper_z`.
        """
        try:
            pts = []
            for vx, vy, vz in self.cal_bed_vars:
                pts.append((float(vx.get()), float(vy.get()), float(vz.get())))

            if hasattr(self, "bk_vars") and "center_x" in self.bk_vars:
                cx = float(self.bk_vars["center_x"].get())
            else:
                cx = -69.296

            if hasattr(self, "bk_vars") and "center_y" in self.bk_vars:
                cy = float(self.bk_vars["center_y"].get())
            else:
                cy = 167.296

            fit = fit_paper_plane(pts, ref_xy=(cx, cy))
        except Exception as e:
            self.cal_bed_result_var.set(f"Lỗi nội suy mặt phẳng giấy: {e}")
            return

        self.paper_tilt = fit.paper_tilt
        if hasattr(self, "bk_vars") and "paper_z" in self.bk_vars:
            self.bk_vars["paper_z"].set(f"{fit.z_ref_mm:.2f}")
        if hasattr(self, "bk_changed"):
            self.bk_changed()
        self.cal_bed_result_var.set(
            f"Đã áp dụng mặt phẳng giấy! Z_tâm={fit.z_ref_mm:.3f} mm · "
            f"kx={fit.paper_tilt[0]:+.5f}, ky={fit.paper_tilt[1]:+.5f} (nghiêng {fit.tilt_deg:.2f}°, dư={fit.max_residual_mm:.4f} mm)."
        )

    def _reset_bed_leveling(self):
        """
        [EN] Reset paper tilt slopes to flat horizontal `(kx = 0.0, ky = 0.0)`.
        [VI] Đặt lại độ nghiêng mặt giấy về phẳng ngang `(kx = 0.0, ky = 0.0)`.
        """
        self.paper_tilt = (0.0, 0.0)
        if hasattr(self, "bk_changed"):
            self.bk_changed()
        self.cal_bed_result_var.set("Đã đặt lại mặt giấy phẳng (kx = 0.0, ky = 0.0).")
