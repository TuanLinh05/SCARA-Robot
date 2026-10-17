r"""
================================================================================
FILE: cartesian_nc_analytics.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Real-time telemetry recorder, multi-channel scientific oscilloscope plotter,
     offline `.jsonl` hardware log analyzer, and automated Markdown/LaTeX/CSV/SVG
     report generator (`AnalyticsMixin` for `NCApp`).
     Designed specifically to help students and researchers collect experimental
     data, visualize kinematics/dynamics, and generate publication-ready tables
     and figures for robotics lab reports and theses.

     HOW THE OSCILLOSCOPE & TELEMETRY RECORDER WORK (For 1st/2nd-Year Students):
     1. Ring Buffer (`collections.deque(maxlen=3600)`): Stores up to 3600 time
        samples (`TelemetrySample`) in RAM. When the 3601st sample arrives, the
        oldest sample is automatically discarded so memory never overflows.
     2. Numerical Velocity Differentiation: Since stepper motors have no velocity
        sensor, we estimate tip speed $v$ between consecutive samples $(t_1, x_1, y_1, z_1)$
        and $(t_2, x_2, y_2, z_2)$ using the 3D Euclidean distance divided by $\Delta t$:
        $$v = \frac{\sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2 + (z_2 - z_1)^2}}{t_2 - t_1}$$
     3. Auto-Scaling Time-Series Plotter (`_draw_time_series_chart`): Maps any
        time interval $[t_{\min}, t_{\max}]$ and signal range $[v_{\min}, v_{\max}]$
        into a rectangular pixel box `(px0, py0, px1, py1)` on the Tkinter Canvas.

[VI] MỤC ĐÍCH:
     Bộ ghi dữ liệu telemetry thời gian thực, đồ thị dao động ký đa kênh, công cụ
     đọc lại file log phần cứng `.jsonl`, và trình xuất Báo cáo Kỹ thuật tự động
     ra định dạng Markdown/LaTeX, CSV, SVG (`AnalyticsMixin` cho `NCApp`).
     Được thiết kế chuyên biệt để hỗ trợ sinh viên và người nghiên cứu thu thập
     số liệu thực nghiệm, trực quan hóa động học và xuất bảng biểu / đồ thị
     chất lượng cao phục vụ viết báo cáo thí nghiệm và đồ án môn học.

     NGUYÊN LÝ BỘ GHI & ĐỒ THỊ DAO ĐỘNG KÝ (Dành cho Sinh viên năm 1–2):
     1. Bộ đệm vòng (`collections.deque(maxlen=3600)`): Lưu tối đa 3600 mẫu thời
        gian (`TelemetrySample`) trong RAM. Khi có mẫu thứ 3601, mẫu cũ nhất sẽ
        tự động bị đẩy ra để phần mềm chạy cả ngày cũng không bao giờ tràn bộ nhớ.
     2. Đạo hàm số tính vận tốc đầu bút: Từ 2 mẫu liên tiếp $(t_1, x_1, y_1, z_1)$
        và $(t_2, x_2, y_2, z_2)$, vận tốc tức thời $v$ được tính bằng quãng đường
        Euclid 3 chiều chia cho khoảng thời gian $\Delta t = t_2 - t_1$.
     3. Đồ thị tự động căn thang đo (`_draw_time_series_chart`): Ánh xạ tuyến tính
        miền thời gian $[t_{\min}, t_{\max}]$ và miền giá trị $[v_{\min}, v_{\max}]$
        vào khung hình chữ nhật điểm ảnh `(px0, py0, px1, py1)` trên Canvas.
================================================================================
"""

from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, filedialog
from cartesian_nc_model import MeasuredModel
from cartesian_nc_3d import export_canvas_to_svg
from cartesian_nc_theme import BG, CARD, LINE, MUTED, TEXT, ACCENT, BLUE, GREEN, RED


@dataclass(frozen=True)
class TelemetrySample:
    """
    [EN] A single timestamped telemetry snapshot for plotting and CSV/report export.
    [VI] Một mẫu dữ liệu telemetry gắn nhãn thời gian dùng để vẽ đồ thị và xuất báo cáo.
    """

    t: float                # [EN] Relative time (s) / [VI] Thời gian tương đối (giây)
    x_mm: float             # [EN] Pen tip X coordinate (mm) / [VI] Tọa độ X đầu bút (mm)
    y_mm: float             # [EN] Pen tip Y coordinate (mm) / [VI] Tọa độ Y đầu bút (mm)
    z_mm: float             # [EN] Pen tip Z height (mm) / [VI] Độ cao Z đầu bút (mm)
    j1_deg: float           # [EN] Shoulder angle J1 (deg) / [VI] Góc khớp vai J1 (độ)
    j2_deg: float           # [EN] Relative elbow angle J2 (deg) / [VI] Góc khuỷu tương đối J2 (độ)
    motor_b_deg: float      # [EN] Coupled motor B angle J2 + k*J1 (deg) / [VI] Góc động cơ B đã bù đai (độ)
    z_steps: int            # [EN] Z step count from reference / [VI] Số xung bước trục Z
    a_steps: int            # [EN] Motor A (J1) step count / [VI] Số xung bước động cơ A (J1)
    b_steps: int            # [EN] Motor B (J2) step count / [VI] Số xung bước động cơ B (J2)
    pps: int                # [EN] Instantaneous step rate (pulses/s) / [VI] Tần số phát xung (xung/s)
    vel_mm_s: float         # [EN] Estimated 3D tip speed (mm/s) / [VI] Vận tốc dài đầu bút (mm/s)
    fifo_used: int          # [EN] Occupied STM32 path FIFO slots (0..32) / [VI] Số ô FIFO đang dùng (0..32)
    manipulability: float   # [EN] Kinematic manipulability |sin(J2)| / [VI] Chỉ số khả thao tác |sin(J2)|
    stage: int              # [EN] Firmware stage enum (0..12) / [VI] Giai đoạn hoạt động (0..12)
    busy: int               # [EN] 1 if moving, 0 if idle / [VI] 1 nếu đang chạy, 0 nếu đứng yên
    switches: int           # [EN] 6-bit limit switch bitmask / [VI] Mặt nạ 6 bit công tắc hành trình


class AnalyticsMixin:
    """
    [EN] Mixin class providing live telemetry acquisition, 4-tab scientific
         plotting, offline `.jsonl` session log replay, and CSV / Markdown / SVG
         academic report generation.
    [VI] Lớp Mixin cung cấp thu thập dữ liệu thời gian thực, vẽ đồ thị khoa học
         4 kênh, đọc lại file log `.jsonl` thực nghiệm và xuất báo cáo CSV / Markdown / SVG.
    """

    def init_analytics(self):
        """
        [EN] Initialize the telemetry ring buffer and analytics window state.
        [VI] Khởi tạo bộ đệm vòng lưu trữ dữ liệu telemetry và trạng thái cửa sổ phân tích.
        """
        self.telemetry_samples = deque(maxlen=3600)
        self.telemetry_t0 = None
        self._last_tel_pos = None
        self._last_tel_snapshot = None
        self.analytics_window = None
        self.analytics_canvas = None
        self.analytics_tree = None
        self.analytics_tab_var = 0
        self.analytics_source_var = None
        self._analytics_draw_key = None
        self.recording_paused = False

    def record_telemetry_sample(self, status):
        """
        [EN] Record a new `TelemetrySample` when a fresh `NCStatus` arrives.
             Deduplicates identical idle frames so the buffer captures meaningful
             motion and state transitions without wasting memory.
        [VI] Ghi nhận một mẫu `TelemetrySample` mới khi nhận gói `NCStatus`.
             Tự động lọc bỏ các gói đứng yên trùng lặp để bộ đệm tập trung lưu trữ
             quá trình chuyển động và thay đổi trạng thái.
        """
        if self.recording_paused or status is None:
            return

        # Step 1: Convert firmware uptime (`up_ms`) to relative time `t_rel` in seconds.
        # Bước 1: Đổi thời gian hoạt động của firmware (`up_ms`) sang thời gian tương đối `t_rel` (giây).
        now = status.up_ms / 1000.0 if status.up_ms > 0 else time.monotonic()
        if self.telemetry_t0 is None or (now < self.telemetry_t0):
            self.telemetry_t0 = now
            self._last_tel_pos = None
        t_rel = max(0.0, now - self.telemetry_t0)

        # Step 2: Skip duplicate idle packets arriving within 0.25s of the last sample.
        # Bước 2: Bỏ qua các gói tin đứng yên trùng hệt nhau nếu chưa quá 0.25 giây.
        snap_sig = (
            status.pos,
            status.pps,
            status.stage,
            status.busy,
            status.switches,
            status.path,
        )
        if (
            self._last_tel_snapshot == snap_sig
            and self.telemetry_samples
            and (t_rel - self.telemetry_samples[-1].t) < 0.25
        ):
            return
        self._last_tel_snapshot = snap_sig

        # Step 3: Obtain the kinematic model to convert step counts into mm and degrees.
        # Bước 3: Lấy mô hình động học để quy đổi số xung bước sang milimét và độ.
        model = self.model
        if model is None and status.referenced:
            try:
                model = self.config.model(status)
            except Exception:
                model = None

        if model is not None:
            q = model.from_steps(status.pos)
            xyz = model.fk(q)
            j1_deg, j2_deg, z_mm = q
            x_mm, y_mm, _ = xyz
            # Coupled belt drive formula: theta_B = J2 + k * J1
            # Công thức truyền động đai chéo: theta_B = J2 + k * J1
            motor_b_deg = j2_deg + model.coupling * j1_deg
            manip = abs(math.sin(math.radians(j2_deg)))
        else:
            x_mm = y_mm = z_mm = j1_deg = j2_deg = motor_b_deg = manip = 0.0

        # Step 4: Estimate 3D Cartesian tip velocity `vel` (mm/s) by finite difference.
        # Bước 4: Tính vận tốc dài 3D của đầu bút `vel` (mm/s) bằng sai phân hữu hạn.
        vel = 0.0
        if self._last_tel_pos is not None and model is not None:
            pt_t, pt_x, pt_y, pt_z = self._last_tel_pos
            dt = t_rel - pt_t
            if dt > 1e-4:
                dist_3d = math.sqrt(
                    (x_mm - pt_x) ** 2 + (y_mm - pt_y) ** 2 + (z_mm - pt_z) ** 2
                )
                vel = min(dist_3d / dt, 250.0)
        if model is not None:
            self._last_tel_pos = (t_rel, x_mm, y_mm, z_mm)

        fifo_free = status.path[3] if len(status.path) >= 4 else 32
        fifo_used = max(0, min(32, 32 - fifo_free))

        self.telemetry_samples.append(
            TelemetrySample(
                t=round(t_rel, 3),
                x_mm=round(x_mm, 3),
                y_mm=round(y_mm, 3),
                z_mm=round(z_mm, 3),
                j1_deg=round(j1_deg, 3),
                j2_deg=round(j2_deg, 3),
                motor_b_deg=round(motor_b_deg, 3),
                z_steps=status.pos[0],
                a_steps=status.pos[1],
                b_steps=status.pos[2],
                pps=status.pps,
                vel_mm_s=round(vel, 2),
                fifo_used=fifo_used,
                manipulability=round(manip, 4),
                stage=status.stage,
                busy=status.busy,
                switches=status.switches,
            )
        )

    def compute_telemetry_summary(self):
        """
        [EN] Compute comprehensive engineering statistics from `self.telemetry_samples`
             for the Analytics table and Markdown/LaTeX report export.
        [VI] Tính toán bảng chỉ số thống kê kỹ thuật toàn diện từ `self.telemetry_samples`
             phục vụ hiển thị bảng số liệu và xuất Báo cáo Markdown/LaTeX.
        """
        samples = list(self.telemetry_samples)
        if not samples:
            return [
                ("Số mẫu thu thập (Samples)", "0 mẫu"),
                (
                    "Trạng thái dữ liệu",
                    "Chưa có mẫu — hãy kết nối hoặc nạp file .jsonl",
                ),
            ]

        n = len(samples)
        duration = max(0.0, samples[-1].t - samples[0].t)

        # Accumulate 2D XY distance and 3D XYZ path length across consecutive samples.
        # Cộng dồn quãng đường mặt phẳng XY và quãng đường không gian 3D giữa các mẫu liên tiếp.
        total_dist = 0.0
        xy_dist = 0.0
        for a, b in zip(samples, samples[1:]):
            dxy = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
            dz = abs(b.z_mm - a.z_mm)
            xy_dist += dxy
            total_dist += math.hypot(dxy, dz)

        # Filter active motion samples to compute meaningful average speed & FIFO load.
        # Lọc các mẫu đang chuyển động để tính vận tốc trung bình và mức chiếm dụng FIFO.
        moving = []
        for s in samples:
            if s.busy or s.pps > 0 or s.vel_mm_s > 0.1:
                moving.append(s)

        v_max = max((s.vel_mm_s for s in samples), default=0.0)
        if moving:
            v_avg = sum(s.vel_mm_s for s in moving) / len(moving)
            fifo_avg = sum(s.fifo_used for s in moving) / len(moving)
        else:
            v_avg = 0.0
            fifo_avg = 0.0

        pps_max = max((s.pps for s in samples), default=0)
        fifo_max = max((s.fifo_used for s in samples), default=0)

        # Find min/max coordinate and joint angle bounds over the recorded session.
        # Tìm giá trị cực tiểu / cực đại của tọa độ XYZ và góc khớp J1, J2.
        j1_min = min(s.j1_deg for s in samples)
        j1_max = max(s.j1_deg for s in samples)
        j2_min = min(s.j2_deg for s in samples)
        j2_max = max(s.j2_deg for s in samples)
        z_min = min(s.z_mm for s in samples)
        z_max = max(s.z_mm for s in samples)
        x_min = min(s.x_mm for s in samples)
        x_max = max(s.x_mm for s in samples)
        y_min = min(s.y_mm for s in samples)
        y_max = max(s.y_mm for s in samples)

        manips = [s.manipulability for s in samples if s.manipulability > 0]
        manip_min = min(manips) if manips else 0.0
        manip_max = max(manips) if manips else 0.0

        rows = [
            ("Số mẫu thu thập (Samples)", f"{n:,} mẫu"),
            ("Tổng thời gian khảo sát (Duration)", f"{duration:.2f} s"),
            ("Quãng đường đầu bút 3D (Total Path)", f"{total_dist:.2f} mm"),
            ("Quãng đường mặt phẳng XY (XY Path)", f"{xy_dist:.2f} mm"),
            ("Vận tốc đầu bút TB / Cực đại", f"{v_avg:.2f} / {v_max:.2f} mm/s"),
            ("Tần số xung cực đại (Max Step Rate)", f"{pps_max:,} xung/s (pps)"),
            (
                "Mức chiếm dụng FIFO TB / Cực đại",
                f"{fifo_avg:.1f} / {fifo_max} (trên 32 đoạn)",
            ),
            ("Phạm vi tọa độ X (X Range)", f"[{x_min:+.2f}, {x_max:+.2f}] mm"),
            ("Phạm vi tọa độ Y (Y Range)", f"[{y_min:+.2f}, {y_max:+.2f}] mm"),
            ("Phạm vi độ cao Z (Z Range)", f"[{z_min:.2f}, {z_max:.2f}] mm"),
            (
                "Biên độ góc khớp vai J1",
                f"[{j1_min:+.2f}°, {j1_max:+.2f}°] (Δ={j1_max - j1_min:.2f}°)",
            ),
            (
                "Biên độ góc khớp khuỷu J2",
                f"[{j2_min:+.2f}°, {j2_max:+.2f}°] (Δ={j2_max - j2_min:.2f}°)",
            ),
            (
                "Chỉ số khả thao tác |sin(J2)|",
                f"min={manip_min:.4f} · max={manip_max:.4f}",
            ),
        ]

        if self.model is not None:
            f_z, f_j1, f_j2 = self.model.factors
            rows.extend(
                [
                    (
                        "Hệ số xung/đơn vị [Z, J1, J2]",
                        f"{f_z:.2f} xung/mm · {f_j1:.3f} / {f_j2:.3f} xung/°",
                    ),
                    (
                        "Tỷ số bù truyền đai (k_cfg / k_model)",
                        f"k_cfg={self.config.coupling:.5f} · k_model={self.model.coupling:.5f}",
                    ),
                ]
            )
        if getattr(self, "bk_active_plan", None) is not None:
            plan = self.bk_active_plan
            rows.append(
                (
                    "Sai số mô hình bản vẽ (Max Model Error)",
                    f"≤ {plan.max_model_error_mm:.4f} mm ({len(plan.runs)} nét liên tục)",
                )
            )
        return rows

    # ==========================================================================
    # 2. ANALYTICS & REPORT STUDIO WINDOW / CỬA SỔ ĐỒ THỊ & XUẤT BÁO CÁO
    # ==========================================================================

    def open_analytics(self):
        """
        [EN] Open the Telemetry, Oscilloscope & Academic Report Studio window.
        [VI] Mở cửa sổ Phân tích Dữ liệu Động học, Đồ thị & Xuất Báo cáo Kỹ thuật.
        """
        if self.analytics_window and self.analytics_window.winfo_exists():
            self.analytics_window.lift()
            self.render_analytics(force=True)
            return

        win = tk.Toplevel(self)
        self.analytics_window = win
        win.transient(self)
        win.title("SCARA · Phân tích Động học, Đồ thị Telemetry & Xuất Báo cáo")
        win.geometry("1180x780")
        win.minsize(1020, 680)
        win.configure(bg=BG)

        # Header & Action Toolbar
        # Thanh tiêu đề và các nút công cụ Xuất dữ liệu / Báo cáo
        top = tk.Frame(
            win,
            bg=CARD,
            padx=16,
            pady=10,
            highlightbackground=LINE,
            highlightthickness=1,
        )
        top.pack(fill="x", padx=16, pady=(14, 10))
        title_box = tk.Frame(top, bg=CARD)
        title_box.pack(side="left")
        self.label(
            title_box, "TRUNG TÂM DỮ LIỆU & BÁO CÁO THỰC NGHIỆM", 14, TEXT
        ).pack(anchor="w")
        self.analytics_source_var = tk.StringVar(
            value="Nguồn: Telemetry trực tiếp (Real-time)"
        )
        self.label(
            title_box, textvariable=self.analytics_source_var, size=9, color=ACCENT
        ).pack(anchor="w")

        btn_bar = tk.Frame(top, bg=CARD)
        btn_bar.pack(side="right")
        for text, cmd, col in (
            ("📂 Nạp Log .jsonl", self.load_jsonl_session, "#2b4c73"),
            ("🗑 Xóa bộ đệm", self.clear_telemetry_samples, "#3a3f58"),
            ("📊 Xuất CSV", self.export_telemetry_csv, "#24597a"),
            ("🖼 Xuất Đồ thị SVG", self.export_analytics_svg, "#266b73"),
            ("📝 Xuất Báo cáo .md (LaTeX)", self.export_markdown_report, "#1f6e5c"),
        ):
            b = self.button(btn_bar, text, cmd, col)
            b.configure(font=("Segoe UI", 9, "bold"), padx=10, pady=4)
            b.pack(side="left", padx=3)

        # Main Split Body: Left = Multi-Channel Plots, Right = Engineering Statistics Table
        # Thân cửa sổ chia 2 cột: Trái = Đồ thị 4 kênh, Phải = Bảng số liệu thống kê kỹ thuật
        body = tk.Frame(win, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=(0, 14))
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        # Left Panel: 4-Channel Scientific Plotter
        # Khung bên trái: Đồ thị khoa học 4 tab
        left = tk.Frame(
            body,
            bg=CARD,
            padx=14,
            pady=12,
            highlightbackground=LINE,
            highlightthickness=1,
        )
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        tab_bar = tk.Frame(left, bg=CARD)
        tab_bar.pack(fill="x", pady=(0, 8))
        self.label(tab_bar, "ĐỒ THỊ THỜI GIAN THỰC:", 10, MUTED).pack(
            side="left", padx=(0, 8)
        )
        self.analytics_tab_btns = []
        for idx, name in enumerate(
            (
                "1. Tọa độ XYZ(t)",
                "2. Góc khớp J1, J2, B(t)",
                "3. Vận tốc & Bộ đệm FIFO",
                "4. Khả thao tác |sin J2| & Quỹ đạo XY",
            )
        ):
            btn_bg = "#24597a" if idx == self.analytics_tab_var else "#1c3049"
            b = self.button(
                tab_bar,
                name,
                lambda i=idx: self.select_analytics_tab(i),
                btn_bg,
            )
            b.configure(font=("Segoe UI", 9), padx=9, pady=3)
            b.pack(side="left", padx=2)
            self.analytics_tab_btns.append(b)

        c = tk.Canvas(
            left, bg="#0d1828", highlightthickness=1, highlightbackground=LINE
        )
        c.pack(fill="both", expand=True)
        self.analytics_canvas = c
        c.bind("<Configure>", lambda e: self.render_analytics(force=True))

        # Right Panel: Live Engineering Summary Table
        # Khung bên phải: Bảng tổng hợp số liệu kỹ thuật phục vụ viết báo cáo
        right = tk.Frame(
            body,
            bg=CARD,
            padx=14,
            pady=12,
            highlightbackground=LINE,
            highlightthickness=1,
        )
        right.grid(row=0, column=1, sticky="nsew")
        self.label(right, "BẢNG SỐ LIỆU KỸ THUẬT (DÙNG CHO BÁO CÁO)", 11, TEXT).pack(
            anchor="w", pady=(0, 6)
        )
        tree_frame = tk.Frame(right, bg=CARD)
        tree_frame.pack(fill="both", expand=True)
        tree = ttk.Treeview(
            tree_frame,
            style="Scara.Treeview",
            columns=("metric", "value"),
            show="headings",
            selectmode="browse",
        )
        tree.heading("metric", text="Chỉ tiêu / Thông số")
        tree.heading("value", text="Giá trị đo / Tính toán")
        tree.column("metric", width=215, anchor="w")
        tree.column("value", width=215, anchor="w")
        sb = ttk.Scrollbar(tree_frame, orient="vertical", command=tree.yview)
        sb.pack(side="right", fill="y")
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side="left", fill="both", expand=True)
        self.analytics_tree = tree

        self.render_analytics(force=True)

    def select_analytics_tab(self, idx):
        """
        [EN] Switch between the 4 scientific plot views in the Analytics window.
        [VI] Chuyển đổi giữa 4 chế độ đồ thị khoa học trong cửa sổ Phân tích.
        """
        self.analytics_tab_var = idx
        for i, b in enumerate(getattr(self, "analytics_tab_btns", ())):
            b.configure(bg="#24597a" if i == idx else "#1c3049")
        self.render_analytics(force=True)

    def clear_telemetry_samples(self):
        """
        [EN] Clear recorded telemetry samples and reset the time origin `t0`.
        [VI] Xóa bộ đệm mẫu dữ liệu và đặt lại mốc thời gian `t0 = 0`.
        """
        self.telemetry_samples.clear()
        self.telemetry_t0 = None
        self._last_tel_pos = None
        self._last_tel_snapshot = None
        if self.analytics_source_var:
            self.analytics_source_var.set(
                "Nguồn: Đã xóa bộ đệm — đang chờ dữ liệu mới"
            )
        self.render_analytics(force=True)

    def load_jsonl_session(self, path=None):
        """
        [EN] Load a recorded `.jsonl` hardware log (from `Doc/logs/` or `logs/`)
             so students can inspect real hardware runs and generate charts/reports offline.
        [VI] Nạp một file log phần cứng `.jsonl` đã ghi (từ `Doc/logs/` hoặc `logs/`)
             để phân tích lại quỹ đạo thực nghiệm và xuất đồ thị / báo cáo ngoại tuyến.
        """
        if not path:
            repo_logs = Path(__file__).resolve().parents[2] / "Doc" / "logs"
            init_dir = (
                repo_logs
                if repo_logs.is_dir()
                else Path(__file__).resolve().parents[1]
            )
            path = filedialog.askopenfilename(
                parent=self.analytics_window or self,
                title="Chọn file Log phiên làm việc (.jsonl)",
                initialdir=str(init_dir),
                filetypes=[("JSON Lines Log", "*.jsonl"), ("All Files", "*.*")],
            )
        if not path:
            return
        loaded = []
        t0 = None
        last_pos = None
        cfg = self.config
        temp_model = self.model

        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                if rec.get("type") != "status" or "status" not in rec:
                    continue
                st = rec["status"]
                up_ms = st.get("up_ms", 0)
                pos = tuple(st.get("pos", (0, 0, 0)))
                rng = tuple(st.get("range", (0, 0, 0)))
                pps = int(st.get("pps", 0))
                stage = int(st.get("stage", 0))
                busy = int(st.get("busy", 0))
                switches = int(st.get("switches", 0))
                path_info = tuple(st.get("path", (0, 0, 0, 32)))
                coupling_ppm = int(st.get("coupling_ppm", 500000))

                # Reconstruct a `MeasuredModel` from the logged Q10 fixed-point factors if needed.
                # Khôi phục mô hình `MeasuredModel` từ hệ số xung Q10 lưu trong file log nếu cần.
                if temp_model is None:
                    try:
                        raw_factors = tuple(
                            st.get("factor", (1638400, 24576, 24576))
                        )
                        factors = tuple(max(1.0, f / 1024.0) for f in raw_factors)
                        z_hi = (
                            max(10.0, rng[0] / factors[0] - 0.1)
                            if rng[0] > 0
                            else 50.0
                        )
                        temp_model = MeasuredModel(
                            l1_mm=cfg.l1_mm,
                            l2_mm=cfg.l2_mm,
                            tool_offset_mm=tuple(cfg.tool_offset_mm),
                            home_deg=(0.0, 0.0),
                            home_z_mm=0.0,
                            joint_limits_deg=cfg.joint_limits_deg,
                            z_limits_mm=(0.1, z_hi),
                            ratios=(cfg.lead_mm_rev, *cfg.ratios),
                            microsteps=cfg.microsteps,
                            positive_high=cfg.positive_high,
                            coupling=coupling_ppm / 1_000_000.0,
                            max_master_rate=cfg.max_master_rate,
                            max_joint_speed_deg_s=cfg.max_joint_speed_deg_s,
                            max_z_speed_mm_s=cfg.max_z_speed_mm_s,
                            measured_factors=factors,
                            branch=1,
                        )
                    except Exception:
                        pass

                t_sec = up_ms / 1000.0 if up_ms > 0 else len(loaded) * 0.04
                if t0 is None or t_sec < t0:
                    t0 = t_sec
                t_rel = max(0.0, t_sec - t0)

                if temp_model is not None:
                    q = temp_model.from_steps(pos)
                    xyz = temp_model.fk(q)
                    j1_d, j2_d, z_m = q
                    x_m, y_m, _ = xyz
                    mb_d = j2_d + temp_model.coupling * j1_d
                    manip = abs(math.sin(math.radians(j2_d)))
                else:
                    x_m = y_m = z_m = j1_d = j2_d = mb_d = manip = 0.0

                vel = 0.0
                if last_pos is not None and temp_model is not None:
                    dt = t_rel - last_pos[0]
                    if dt > 1e-4:
                        dist_3d = math.sqrt(
                            (x_m - last_pos[1]) ** 2
                            + (y_m - last_pos[2]) ** 2
                            + (z_m - last_pos[3]) ** 2
                        )
                        vel = min(250.0, dist_3d / dt)
                if temp_model is not None:
                    last_pos = (t_rel, x_m, y_m, z_m)

                free_slots = path_info[3] if len(path_info) >= 4 else 32
                fifo_used = max(0, min(32, 32 - free_slots))
                loaded.append(
                    TelemetrySample(
                        t=round(t_rel, 3),
                        x_mm=round(x_m, 3),
                        y_mm=round(y_m, 3),
                        z_mm=round(z_m, 3),
                        j1_deg=round(j1_d, 3),
                        j2_deg=round(j2_d, 3),
                        motor_b_deg=round(mb_d, 3),
                        z_steps=pos[0],
                        a_steps=pos[1],
                        b_steps=pos[2],
                        pps=pps,
                        vel_mm_s=round(vel, 2),
                        fifo_used=fifo_used,
                        manipulability=round(manip, 4),
                        stage=stage,
                        busy=busy,
                        switches=switches,
                    )
                )

        if loaded:
            self.telemetry_samples.clear()
            step = max(1, len(loaded) // 3600)
            self.telemetry_samples.extend(loaded[::step])
            if self.analytics_source_var:
                self.analytics_source_var.set(
                    f"Nguồn: File Log '{Path(path).name}' ({len(self.telemetry_samples)} mẫu)"
                )
            self.render_analytics(force=True)

    # ==========================================================================
    # 3. SCIENTIFIC PLOT RENDERING / VẼ ĐỒ THỊ ĐỘNG HỌC & TELEMETRY
    # ==========================================================================

    def _draw_time_series_chart(self, canvas, box, title, series_list, samples):
        """
        [EN] Render a multi-curve time-series chart inside `box = (x0, y0, x1, y1)`
             with grid lines, Y-axis auto-scaling, legend, and current value badges.
        [VI] Vẽ đồ thị đa đường theo thời gian trong khung `box = (x0, y0, x1, y1)`
             kèm lưới tọa độ, tự động căn thang trục Y, chú giải và giá trị hiện tại.
        """
        x0, y0, x1, y1 = box
        pad_l, pad_r, pad_t, pad_b = 56, 18, 32, 28
        px0, py0 = x0 + pad_l, y0 + pad_t
        px1, py1 = x1 - pad_r, y1 - pad_b
        if px1 <= px0 + 20 or py1 <= py0 + 20:
            return

        canvas.create_rectangle(x0, y0, x1, y1, fill="#101e31", outline="#243a56")
        canvas.create_text(
            x0 + 12,
            y0 + 8,
            text=title,
            anchor="nw",
            fill=TEXT,
            font=("Segoe UI", 9, "bold"),
        )

        if len(samples) < 2:
            canvas.create_text(
                (px0 + px1) * 0.5,
                (py0 + py1) * 0.5,
                text="Đang chờ dữ liệu chuyển động hoặc nạp file Log .jsonl…",
                fill=MUTED,
                font=("Segoe UI", 9),
            )
            return

        # Step 1: Compute time domain [t_min, t_max] and value range [v_min, v_max].
        # Bước 1: Tìm khoảng thời gian [t_min, t_max] và khoảng giá trị trục tung [v_min, v_max].
        t_min = samples[0].t
        t_max = max(t_min + 0.1, samples[-1].t)

        vals = []
        for _label, _color, fn in series_list:
            for s in samples:
                vals.append(fn(s))
        v_min = min(vals)
        v_max = max(vals)
        if abs(v_max - v_min) < 1e-3:
            v_min -= 1.0
            v_max += 1.0
        else:
            margin = (v_max - v_min) * 0.08
            v_min -= margin
            v_max += margin

        # Step 2: Draw 5 horizontal Y-grid lines and 6 vertical time-grid lines.
        # Bước 2: Vẽ 5 đường lưới ngang (trục Y) và 6 đường lưới dọc (trục thời gian t).
        for i in range(5):
            u = i / 4.0
            gy = py1 - u * (py1 - py0)
            gv = v_min + u * (v_max - v_min)
            canvas.create_line(px0, gy, px1, gy, fill="#1c3048")
            canvas.create_text(
                px0 - 6,
                gy,
                text=f"{gv:.1f}",
                anchor="e",
                fill=MUTED,
                font=("Segoe UI", 8),
            )
        for i in range(6):
            u = i / 5.0
            gx = px0 + u * (px1 - px0)
            gt = t_min + u * (t_max - t_min)
            canvas.create_line(gx, py0, gx, py1, fill="#1c3048")
            canvas.create_text(
                gx,
                py1 + 11,
                text=f"{gt:.1f}s",
                fill=MUTED,
                font=("Segoe UI", 8),
            )

        # Step 3: Decimate points to <= 400 vertices per polyline for instant rendering.
        # Bước 3: Lấy mẫu tối đa 400 điểm trên mỗi đường cong để vẽ cực nhanh.
        stride = max(1, len(samples) // 400)
        pts_samples = samples[::stride]
        if pts_samples[-1] is not samples[-1]:
            pts_samples.append(samples[-1])

        # Step 4: Plot each signal curve and draw its top-right legend badge.
        # Bước 4: Vẽ từng đường tín hiệu và nhãn chú giải giá trị ở góc trên phải.
        lx = x1 - 14
        for label, color, fn in reversed(series_list):
            coords = []
            for s in pts_samples:
                norm_t = (s.t - t_min) / (t_max - t_min)
                norm_v = (fn(s) - v_min) / (v_max - v_min)
                sx = px0 + norm_t * (px1 - px0)
                sy = py1 - norm_v * (py1 - py0)
                coords.extend((sx, sy))
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=color, width=2)
            cur_val = fn(samples[-1])
            badge_txt = f"● {label}: {cur_val:.2f}"
            canvas.create_text(
                lx,
                y0 + 8,
                text=badge_txt,
                anchor="ne",
                fill=color,
                font=("Segoe UI", 8, "bold"),
            )
            lx -= len(badge_txt) * 6.5 + 14

    def render_analytics(self, force=False):
        """
        [EN] Refresh the Analytics charts and statistics Treeview if the window is open.
        [VI] Cập nhật đồ thị và bảng thống kê kỹ thuật nếu cửa sổ Phân tích đang mở.
        """
        if not self.analytics_window or not self.analytics_window.winfo_exists():
            return
        c = self.analytics_canvas
        w = max(c.winfo_width(), 100)
        h = max(c.winfo_height(), 100)
        samples = list(self.telemetry_samples)
        last_t = samples[-1].t if samples else -1.0
        key = (w, h, self.analytics_tab_var, len(samples), last_t)
        if not force and self._analytics_draw_key == key:
            return
        self._analytics_draw_key = key

        # 1. Update Statistics Treeview
        # 1. Cập nhật bảng số liệu thống kê kỹ thuật bên phải
        if self.analytics_tree is not None:
            self.analytics_tree.delete(*self.analytics_tree.get_children())
            for i, (metric, val) in enumerate(self.compute_telemetry_summary()):
                self.analytics_tree.insert(
                    "", "end", iid=str(i), values=(metric, val)
                )

        # 2. Draw Selected Plot Tab
        # 2. Vẽ đồ thị tương ứng với tab đang chọn
        c.delete("all")
        tab = self.analytics_tab_var
        mid_y = h // 2
        if tab == 0:
            self._draw_time_series_chart(
                c,
                (8, 8, w - 8, mid_y - 4),
                "TỌA ĐỘ MẶT PHẲNG ĐẦU BÚT X(t) & Y(t) [mm]",
                (
                    ("X (mm)", BLUE, lambda s: s.x_mm),
                    ("Y (mm)", GREEN, lambda s: s.y_mm),
                ),
                samples,
            )
            self._draw_time_series_chart(
                c,
                (8, mid_y + 4, w - 8, h - 8),
                "ĐỘ CAO TRỤC Z(t) [mm]",
                (("Z (mm)", "#c4a2f4", lambda s: s.z_mm),),
                samples,
            )
        elif tab == 1:
            self._draw_time_series_chart(
                c,
                (8, 8, w - 8, mid_y - 4),
                "GÓC KHỚP VAI J1(t) & KHUỶU TƯƠNG ĐỐI J2(t) [độ °]",
                (
                    ("J1 (°)", BLUE, lambda s: s.j1_deg),
                    ("J2 (°)", GREEN, lambda s: s.j2_deg),
                ),
                samples,
            )
            self._draw_time_series_chart(
                c,
                (8, mid_y + 4, w - 8, h - 8),
                "GÓC ĐỘNG CƠ B ĐÃ BÙ ĐAI: θ_B(t) = J2 + k·J1 [độ °]",
                (("Motor B (°)", "#efc25a", lambda s: s.motor_b_deg),),
                samples,
            )
        elif tab == 2:
            self._draw_time_series_chart(
                c,
                (8, 8, w - 8, mid_y - 4),
                "VẬN TỐC ĐẦU BÚT [mm/s] & TẦN SỐ XUNG [×100 pps]",
                (
                    ("Vận tốc (mm/s)", ACCENT, lambda s: s.vel_mm_s),
                    ("Xung (×100 pps)", "#efc25a", lambda s: s.pps / 100.0),
                ),
                samples,
            )
            self._draw_time_series_chart(
                c,
                (8, mid_y + 4, w - 8, h - 8),
                "MỨC CHIẾM DỤNG BỘ ĐỆM ĐOẠN LỆNH TRÊN STM32 (FIFO 0..32)",
                (("FIFO đang dùng", RED, lambda s: float(s.fifo_used)),),
                samples,
            )
        else:
            self._draw_time_series_chart(
                c,
                (8, 8, w - 8, mid_y - 4),
                "CHỈ SỐ KHẢ THAO TÁC ĐỘNG HỌC |sin(J2)| (Khoảng cách tới điểm suy biến)",
                (("|sin(J2)|", "#efc25a", lambda s: s.manipulability),),
                samples,
            )
            # Bottom half: 2D XY Path Map from recorded samples
            # Nửa dưới Tab 4: Bản đồ quỹ đạo mặt phẳng XY đã ghi nhận
            x0, y0, x1, y1 = 8, mid_y + 4, w - 8, h - 8
            c.create_rectangle(x0, y0, x1, y1, fill="#101e31", outline="#243a56")
            c.create_text(
                x0 + 12,
                y0 + 8,
                text="BẢN ĐỒ QUỸ ĐẠO ĐẦU BÚT XY ĐÃ GHI NHẬN [mm]",
                anchor="nw",
                fill=TEXT,
                font=("Segoe UI", 9, "bold"),
            )
            if len(samples) >= 2:
                xs = [s.x_mm for s in samples]
                ys = [s.y_mm for s in samples]
                xmin, xmax = min(xs) - 5.0, max(xs) + 5.0
                ymin, ymax = min(ys) - 5.0, max(ys) + 5.0
                span_x = max(10.0, xmax - xmin)
                span_y = max(10.0, ymax - ymin)
                scale = min((x1 - x0 - 80) / span_x, (y1 - y0 - 50) / span_y)
                cx_s = (x0 + x1) * 0.5
                cy_s = (y0 + y1 + 16) * 0.5
                xm = (xmin + xmax) * 0.5
                ym = (ymin + ymax) * 0.5
                stride = max(1, len(samples) // 500)
                pts = []
                for s in samples[::stride]:
                    sx = cx_s + (s.x_mm - xm) * scale
                    sy = cy_s - (s.y_mm - ym) * scale
                    pts.extend((sx, sy))
                if len(pts) >= 4:
                    c.create_line(*pts, fill=GREEN, width=2)

    # ==========================================================================
    # 4. EXPORT TOOLS FOR REPORTS / CÁC HÀM XUẤT DỮ LIỆU & BÁO CÁO KỸ THUẬT
    # ==========================================================================

    def export_telemetry_csv(self, path=None):
        """
        [EN] Export all recorded `TelemetrySample` rows to a UTF-8 `.csv` file.
        [VI] Xuất toàn bộ chuỗi thời gian `TelemetrySample` ra file bảng tính `.csv`.
        """
        if not path:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = filedialog.asksaveasfilename(
                parent=self.analytics_window or self,
                title="Xuất dữ liệu Telemetry ra CSV (Excel / MATLAB / Python)",
                defaultextension=".csv",
                initialfile=f"scara_telemetry_{timestamp_str}.csv",
                filetypes=[("CSV Spreadsheet", "*.csv")],
            )
        if not path:
            return
        fields = (
            "t",
            "x_mm",
            "y_mm",
            "z_mm",
            "j1_deg",
            "j2_deg",
            "motor_b_deg",
            "z_steps",
            "a_steps",
            "b_steps",
            "pps",
            "vel_mm_s",
            "fifo_used",
            "manipulability",
            "stage",
            "busy",
            "switches",
        )
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(fields)
            for s in self.telemetry_samples:
                row_values = []
                for field_name in fields:
                    row_values.append(getattr(s, field_name))
                writer.writerow(row_values)
        self.message.set(
            f"Đã xuất {len(self.telemetry_samples)} mẫu dữ liệu ra CSV: {path}"
        )

    def export_analytics_svg(self, path=None):
        """
        [EN] Save the current scientific plot canvas as a vector `.svg` graphic.
        [VI] Xuất đồ thị khoa học hiện tại ra file ảnh vector `.svg` cho Báo cáo.
        """
        if not self.analytics_canvas:
            return
        if not path:
            path = filedialog.asksaveasfilename(
                parent=self.analytics_window or self,
                title="Xuất Đồ thị Telemetry ra Vector SVG",
                defaultextension=".svg",
                initialfile=f"scara_chart_tab{self.analytics_tab_var + 1}.svg",
                filetypes=[("SVG Vector Graphic", "*.svg")],
            )
        if path:
            export_canvas_to_svg(self.analytics_canvas, path)
            self.message.set(f"Đã xuất đồ thị SVG: {path}")

    def export_markdown_report(self, path=None):
        """
        [EN] Generate a comprehensive Markdown + LaTeX engineering report (`.md`)
             containing robot configuration, kinematic formulas, calibration
             parameters, and experimental telemetry statistics.
        [VI] Tự động tổng hợp và xuất Báo cáo Kỹ thuật định dạng Markdown + LaTeX (`.md`)
             bao gồm thông số hình học, phương trình động học, kết quả hiệu chuẩn HOME
             và bảng thống kê thực nghiệm.
        """
        if not path:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = filedialog.asksaveasfilename(
                parent=self.analytics_window or self,
                title="Xuất Báo cáo Kỹ thuật SCARA (.md / LaTeX)",
                defaultextension=".md",
                initialfile=f"Bao_cao_SCARA_{timestamp_str}.md",
                filetypes=[("Markdown Report", "*.md")],
            )
        if not path:
            return

        cfg = self.config
        summary_rows = self.compute_telemetry_summary()
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        fw_build_name = getattr(self.state, "build", "SCARA_CART_NC_HOME_V5_R9")
        lines = [
            "# BÁO CÁO KỸ THUẬT & SỐ LIỆU THỰC NGHIỆM ROBOT SCARA 3 BẬC TỰ DO",
            f"- **Thời điểm trích xuất báo cáo**: `{now_str}`",
            f"- **Phiên bản phần mềm / Firmware**: `SCARA Motion & Drawing Studio R15` / `{fw_build_name}`",
            "",
            "---",
            "",
            "## 1. Thông số Hình học & Truyền động Cơ khí (Mechanical & Kinematic Parameters)",
            "",
            "| Thông số (Parameter) | Ký hiệu | Giá trị cấu hình | Đơn vị |",
            "| :--- | :---: | :---: | :---: |",
            f"| Chiều dài khâu 1 (Shoulder Link 1) | $L_1$ | `{cfg.l1_mm:.2f}` | mm |",
            f"| Chiều dài khâu 2 (Elbow Link 2) | $L_2$ | `{cfg.l2_mm:.2f}` | mm |",
            f"| Độ lệch đầu bút TCP (Tool Offset) | $(t_x, t_y)$ | `({cfg.tool_offset_mm[0]:.2f}, {cfg.tool_offset_mm[1]:.2f})` | mm |",
            f"| Bước vít me trục nâng hạ Z | $p_Z$ | `{cfg.lead_mm_rev:.2f}` | mm/vòng |",
            f"| Hành trình trục nâng hạ Z | $Z_{{span}}$ | `{cfg.z_span_mm:.2f}` | mm |",
            f"| Tỷ số truyền đai khớp J1 / J2 | $(R_1, R_2)$ | `({cfg.ratios[0]:.2f}, {cfg.ratios[1]:.2f})` | — |",
            f"| Phân độ vi bước [Z, J1, J2] | $M$ | `{cfg.microsteps}` | microsteps |",
            f"| Góc nghỉ chuẩn sau HOME (Park Pose) | $(\\theta_{{1,park}}, \\theta_{{2,park}})$ | `({cfg.park_deg[0]:.1f}°, {cfg.park_deg[1]:.1f}°)` | độ (°) |",
            f"| Giới hạn an toàn góc khớp J1 | $[\\theta_{{1,\\min}}, \\theta_{{1,\\max}}]$ | `[{cfg.joint_limits_deg[0][0]:.1f}°, {cfg.joint_limits_deg[0][1]:.1f}°]` | độ (°) |",
            f"| Giới hạn an toàn góc khớp J2 | $[\\theta_{{2,\\min}}, \\theta_{{2,\\max}}]$ | `[{cfg.joint_limits_deg[1][0]:.1f}°, {cfg.joint_limits_deg[1][1]:.1f}°]` | độ (°) |",
            f"| Hệ số truyền đai chéo (Coupling) | $k$ | `{cfg.coupling:.6f}` | rad/rad |",
            f"| Tự động hiệu chuẩn bù đai (Auto Coupling) | `auto_coupling` | `{cfg.auto_coupling}` | — |",
            "",
            "## 2. Mô hình Toán học Động học & Bù Đai (Kinematic Equations)",
            "",
            "- **Động học thuận (Forward Kinematics — Gốc tọa độ tại tâm khớp vai J1, trục $+Y$ hướng thẳng ra trước)**:",
            "  $$x = -L_1 \\sin\\theta_1 - (L_2 + t_y)\\sin(\\theta_1 + \\theta_2) + t_x \\cos(\\theta_1 + \\theta_2)$$",
            "  $$y = L_1 \\cos\\theta_1 + (L_2 + t_y)\\cos(\\theta_1 + \\theta_2) + t_x \\sin(\\theta_1 + \\theta_2)$$",
            "- **Quan hệ truyền động bù dây đai hai cấp trục khuỷu J2 (Coupled Belt Drive)**:",
            "  $$\\theta_B = \\theta_2 + k \\cdot \\theta_1$$",
            "",
            "## 3. Bảng Thống kê Dữ liệu Thực nghiệm (Experimental Telemetry Summary)",
            "",
            "| Chỉ tiêu đánh giá (Metric) | Giá trị thu thập / Tính toán (Measured Value) |",
            "| :--- | :--- |",
        ]
        for metric, val in summary_rows:
            lines.append(f"| {metric} | `{val}` |")

        lines.extend(
            [
                "",
                "---",
                "*Báo cáo được sinh tự động bởi phân hệ Trung tâm Dữ liệu & Báo cáo Thực nghiệm (SCARA Motion Studio).*",
            ]
        )
        Path(path).write_text("\n".join(lines), encoding="utf-8")
        self.message.set(f"Đã xuất Báo cáo Kỹ thuật Markdown/LaTeX: {path}")
