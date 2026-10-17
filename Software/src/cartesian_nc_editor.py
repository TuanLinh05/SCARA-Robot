"""
================================================================================
FILE: cartesian_nc_editor.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Interactive 2D/3D workspace editor, step-jog pad, and setpoint table manager
     (`WorkspaceEditor` mixin for `NCApp`):
     - Renders the reachable workspace polygon, paper sheet zone, coordinate grid,
       live two-link SCARA arm (`#5aaeff` / `#60d5ae`), and yellow target pose
       (`#efc25a`) in 2D mode, or the shaded 3D X-SCARA STL assembly in 3D mode.
     - Supports mouse dragging of the end-effector (XY inverse kinematics) or
       the elbow joint (`J1` angle) in 2D mode, and 3D orbital camera rotation
       in 3D mode.
     - Provides a dedicated `"Jog & Tùy biến"` tab with step-increment buttons
       (`±X`, `±Y`, `±Z`, `±J1°`, `±J2°`) and quick pose presets (`Tâm giấy`, `Park`).
     - Provides the multi-point setpoint manager window (`open_points`) with
       CSV import/export and 3x3 test-grid generation.

     3-LAYER 2D CANVAS ARCHITECTURE (For 1st/2nd-Year Students):
     To keep the 2D canvas running at 60 FPS without lag, `draw_workspace()`
     divides all graphics into 3 tagged layers, each with its own cache key:
     - Layer 1 (`tags="grid"`, cached by `self.grid_key`): Static coordinate grid,
       paper rectangle, and reachable workspace polygon. Only redrawn when the
       window is resized or zoomed/panned!
     - Layer 2 (`tags="waypoints"`, cached by `self.point_key`): Purple setpoint
       dots (`P01`, `P02`, ...). Only redrawn when the setpoint list changes!
     - Layer 3 (`tags="dynamic"`, cached by `self.dynamic_key`): The moving
       2-link robot arm, yellow target ghost arm, and dashed trajectory preview.

[VI] MỤC ĐÍCH:
     Trình biên tập vùng làm việc 2D/3D tương tác, bảng điều khiển Jog theo bước
     và quản lý bảng chuỗi điểm (`WorkspaceEditor` mixin cho `NCApp`):
     - Vẽ miền làm việc khả dụng, khung tờ giấy vẽ, lưới tọa độ, tay máy SCARA 2 khâu
       trực tiếp (`#5aaeff` / `#60d5ae`) và tư thế đích màu vàng (`#efc25a`) ở chế độ
       2D, hoặc mô hình 3D X-SCARA lắp ráp từ STL ở chế độ 3D.
     - Hỗ trợ kéo chuột tại đầu công tác (giải động học ngược XY) hoặc tại khớp khuỷu
       (chỉnh góc `J1`) ở chế độ 2D, và xoay góc nhìn 3D ở chế độ 3D.
     - Cung cấp tab `"Jog & Tùy biến"` với bàn phím di chuyển từng bước (`±X`, `±Y`,
       `±Z`, `±J1°`, `±J2°`) và các nút vị trí nhanh (`Tâm giấy`, `Vị trí Park`).
     - Cung cấp cửa sổ quản lý danh sách điểm đặt (`open_points`) hỗ trợ nhập/xuất
       file CSV và tự động sinh lưới 9 điểm kiểm thử.

     KIẾN TRÚC VẼ 3 LỚP TRÊN CANVAS 2D (Dành cho Sinh viên năm 1–2):
     Để đồ thị 2D đạt tốc độ 60 khung hình/giây mượt mà, hàm `draw_workspace()`
     chia các đối tượng vẽ thành 3 lớp (layer) gắn nhãn `tags` riêng biệt:
     - Lớp 1 (`tags="grid"`, khóa `self.grid_key`): Lưới tọa độ tĩnh, khung giấy
       và đa giác miền làm việc. Chỉ vẽ lại khi đổi cỡ cửa sổ hoặc phóng to/dịch chuyển!
     - Lớp 2 (`tags="waypoints"`, khóa `self.point_key`): Các điểm mốc màu tím
       (`P01`, `P02`,...). Chỉ vẽ lại khi thêm/xóa điểm trong bảng!
     - Lớp 3 (`tags="dynamic"`, khóa `self.dynamic_key`): Hai khâu tay máy đang
       chuyển động, tay máy đích màu vàng và đường dự kiến nét đứt.
================================================================================
"""

import math
import time
import tkinter as tk
from tkinter import ttk, filedialog
from cartesian_nc_3d import Camera3D, render_scara_3d_on_canvas
from cartesian_nc_workspace import (
    Viewport,
    Setpoint,
    pose_ranges,
    test_points,
    load_points,
    save_points,
)


class WorkspaceEditor:
    """
    [EN] Mixin class providing the interactive 2D/3D robot canvas, joint sliders,
         step-jog pad, and the setpoint sequence table window.
    [VI] Lớp Mixin cung cấp đồ thị tương tác 2D/3D, thanh trượt góc khớp, bàn phím
         Jog theo bước và cửa sổ quản lý bảng chuỗi điểm setpoint.
    """

    # ==========================================================================
    # 1. INITIALIZATION & CONTROL TABS / KHỞI TẠO & CÁC TAB ĐIỀU KHIỂN
    # ==========================================================================

    def init_editor(self):
        """
        [EN] Initialize workspace viewport, joint pose variables, jog step, and setpoint list.
        [VI] Khởi tạo khung nhìn vùng làm việc, các biến tư thế khớp, bước Jog và danh sách điểm.
        """
        self.target_pose = None
        self.view = None
        self.view_zoom = 1.0
        self.view_pan = (0.0, 0.0)
        self.pan_origin = None
        self.grid_key = None
        self.point_key = None
        self.dynamic_key = None
        self.editor_state_key = None
        self.drag_kind = None
        self.editor_sync = False
        self.pose_values = [tk.DoubleVar(value=0) for _ in range(3)]
        self.pose_entries = [tk.StringVar(value="0") for _ in range(3)]
        self.jog_step = tk.StringVar(value="5")
        self.pose_note = tk.StringVar(value="HOME + CALIB để chỉnh mô hình")
        self.cursor_note = tk.StringVar(
            value="Kéo đầu công tác chọn XY · kéo khuỷu chọn góc J1 · Lăn chuột Zoom"
        )
        self.follow_enabled = tk.BooleanVar(value=False)
        self.points = []
        self.points_window = None
        self.show_points = tk.BooleanVar(value=False)
        self.pose_scales = []
        self.pose_widgets = []
        self.point_buttons = []

    def build_workspace_tabs(self, right):
        """
        [EN] Build the right-hand control notebook (`Nhập XYZ`, `Chỉnh mô hình`,
             and `Jog & Tùy biến` tabs) plus the `Điểm thử / Chuỗi setpoint` button.
        [VI] Dựng khung tab điều khiển bên phải (gồm tab `Nhập XYZ`, `Chỉnh mô hình`
             và `Jog & Tùy biến`) cùng nút mở cửa sổ `Điểm thử / Chuỗi setpoint`.
        """
        self.points_btn = self.button(
            right, "Điểm thử / Chuỗi setpoint", self.open_points, "#394771"
        )
        self.points_btn.pack(side="bottom", fill="x", pady=(4, 0))
        tabs = ttk.Notebook(right, style="Scara.TNotebook", width=310)
        tabs.pack(fill="both", expand=True, pady=(4, 0))
        xyz = tk.Frame(tabs, bg=right.cget("bg"))
        pose = tk.Frame(tabs, bg=right.cget("bg"))
        jog = tk.Frame(tabs, bg=right.cget("bg"))
        tabs.add(xyz, text="Nhập XYZ")
        tabs.add(pose, text="Chỉnh mô hình")
        tabs.add(jog, text="Jog & Tùy biến")

        # ----------------------------------------------------------------------
        # Tab 1: Joint Pose Sliders ("Chỉnh mô hình")
        # Tab 1: Thanh trượt chỉnh góc khớp J1, J2 và độ cao Z ("Chỉnh mô hình")
        # ----------------------------------------------------------------------
        self.label(
            pose,
            "J2 là góc giữa hai arm; góc dương về −X.",
            9,
            "#9cb0cb",
            wraplength=300,
        ).pack(anchor="w", pady=(6, 4))
        for i, title in enumerate(("J1 (°)", "Khuỷu J2 (°)", "Z (mm)")):
            row = tk.Frame(pose, bg=pose.cget("bg"))
            row.pack(fill="x")
            self.label(row, title, size=10).pack(side="left")
            entry = self.entry(row, self.pose_entries[i])
            entry.pack(side="right")
            entry.bind("<Return>", lambda event: self.commit_pose())
            scale = tk.Scale(
                pose,
                from_=-88,
                to=88,
                orient="horizontal",
                resolution=0.01,
                showvalue=False,
                variable=self.pose_values[i],
                command=lambda value: self.slider_pose(),
                bg=pose.cget("bg"),
                fg="#edf3fc",
                troughcolor="#243651",
                highlightthickness=0,
                sliderlength=22,
            )
            scale.pack(fill="x", pady=(0, 6))
            scale.bind("<ButtonRelease-1>", lambda event: self.release_pose())
            self.pose_scales.append(scale)
            self.pose_widgets.extend((entry, scale))

        row = tk.Frame(pose, bg=pose.cget("bg"))
        row.pack(fill="x", pady=2)
        self.label(row, "Tốc độ tối đa (mm/s)", size=10).pack(side="left")
        entry = self.entry(row, self.speed)
        entry.pack(side="right")
        self.pose_widgets.append(entry)

        row = tk.Frame(pose, bg=pose.cget("bg"))
        row.pack(fill="x", pady=3)
        b_apply = self.button(row, "Áp dụng", self.commit_pose)
        b_apply.pack(side="left")
        b_send = self.button(row, "Gửi tư thế", self.move, "#245d9c")
        b_send.pack(side="right")
        self.pose_widgets.extend((b_apply, b_send))

        self.follow_checkbox = tk.Checkbutton(
            pose,
            text="Robot bám theo mô hình",
            variable=self.follow_enabled,
            command=self.toggle_follow,
            bg=pose.cget("bg"),
            fg="#edf3fc",
            selectcolor="#243651",
            activebackground=pose.cget("bg"),
        )
        self.follow_checkbox.pack(anchor="w", pady=4)

        # ----------------------------------------------------------------------
        # Tab 2: Interactive Step-Jog Pad & Quick Presets ("Jog & Tùy biến")
        # Tab 2: Bàn phím dịch chuyển từng bước & Vị trí nhanh ("Jog & Tùy biến")
        # ----------------------------------------------------------------------
        self._build_jog_tab(jog)

        self.workspace_tabs = tabs
        return xyz

    def _build_jog_tab(self, jog):
        """
        [EN] Populate the `"Jog & Tùy biến"` tab with step size buttons, directional
             Cartesian/Joint jog buttons, and quick preset targets.
        [VI] Dựng giao diện tab `"Jog & Tùy biến"` gồm chọn bước nhảy, bàn phím dịch
             chuyển từng bước theo trục Đề-các / góc khớp và các điểm đến nhanh.
        """
        bg = jog.cget("bg")
        step_row = tk.Frame(jog, bg=bg)
        step_row.pack(fill="x", pady=(6, 4))
        self.label(step_row, "Bước nhảy (mm / °):", 9, "#9cb0cb").pack(side="left")
        step_entry = self.entry(step_row, self.jog_step)
        step_entry.configure(width=6, font=("Segoe UI", 10))
        step_entry.pack(side="right")
        self.pose_widgets.append(step_entry)

        quick_steps = tk.Frame(jog, bg=bg)
        quick_steps.pack(fill="x", pady=(0, 6))
        for val in ("0.5", "1", "5", "10", "20"):
            b = self.button(
                quick_steps, val, lambda v=val: self.jog_step.set(v), "#1f324d"
            )
            b.configure(font=("Segoe UI", 8, "bold"), padx=6, pady=2)
            b.pack(side="left", expand=True, fill="x", padx=1)
            self.pose_widgets.append(b)

        # Directional XY + Z Pad (3x3 Button Grid)
        # Bàn phím điều hướng XY + Z (Lưới nút bấm 3x3)
        pad = tk.Frame(jog, bg=bg)
        pad.pack(fill="x", pady=2)
        for c in range(3):
            pad.columnconfigure(c, weight=1)
        jog_specs = (
            (0, 1, "▲ +Y", lambda: self.jog_by(dy=1), "#24597a"),
            (1, 0, "◀ −X", lambda: self.jog_by(dx=-1), "#24597a"),
            (1, 1, "● Tâm giấy", self.preset_paper_center, "#1f5c4b"),
            (1, 2, "▶ +X", lambda: self.jog_by(dx=1), "#24597a"),
            (2, 1, "▼ −Y", lambda: self.jog_by(dy=-1), "#24597a"),
            (0, 2, "▲ Z Lên", lambda: self.jog_by(dz=1), "#3d356b"),
            (2, 2, "▼ Z Xuống", lambda: self.jog_by(dz=-1), "#3d356b"),
            (0, 0, "⌂ Park", self.preset_park_pose, "#344c79"),
            (2, 0, "✍ Z Giấy", self.preset_paper_z, "#344c79"),
        )
        for r, c, label, cmd, col in jog_specs:
            b = self.button(pad, label, cmd, col)
            b.configure(font=("Segoe UI", 9, "bold"), padx=4, pady=3)
            b.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)
            self.pose_widgets.append(b)

        # Joint J1 / J2 Jog Row
        # Hàng nút dịch chuyển trực tiếp góc khớp J1 / J2
        j_row = tk.Frame(jog, bg=bg)
        j_row.pack(fill="x", pady=(6, 2))
        for label, dj1, dj2 in (
            ("↺ J1 −", -1, 0),
            ("↻ J1 +", 1, 0),
            ("↺ J2 −", 0, -1),
            ("↻ J2 +", 0, 1),
        ):
            b = self.button(
                j_row,
                label,
                lambda a=dj1, b_=dj2: self.jog_by(dj1=a, dj2=b_),
                "#283f5e",
            )
            b.configure(font=("Segoe UI", 9), padx=4, pady=2)
            b.pack(side="left", expand=True, fill="x", padx=1)
            self.pose_widgets.append(b)

    def jog_by(self, dx=0.0, dy=0.0, dz=0.0, dj1=0.0, dj2=0.0):
        """
        [EN] Increment the current target XYZ or Joint angles by `jog_step` and
             immediately execute the bounded move on the robot.
        [VI] Dịch chuyển tọa độ đích XYZ hoặc góc khớp một lượng `jog_step` và
             gửi lệnh di chuyển ngay tới robot.
        """
        if not self.editor_ready() or self.program_points:
            return
        try:
            raw_step = float(self.jog_step.get())
            step = max(0.05, min(50.0, raw_step))
        except ValueError:
            step = 5.0

        base_q = self.target_pose or self.model.from_steps(self.state.pos)
        try:
            if dj1 or dj2:
                # Joint-space jog: directly add `dj1 * step` and `dj2 * step` (degrees).
                # Jog trong không gian khớp: cộng trực tiếp góc `dj1 * step` và `dj2 * step` (độ).
                new_j1 = base_q[0] + dj1 * step
                new_j2 = base_q[1] + dj2 * step
                q = (new_j1, new_j2, base_q[2])
            else:
                # Cartesian-space jog: compute current FK `(x, y, z)`, add `(dx, dy, dz) * step`,
                # then solve Inverse Kinematics (`self.model.ik`) for the new joint angles.
                # Jog trong không gian Đề-các: tính FK `(x, y, z)` hiện tại, cộng độ dịch chuyển,
                # sau đó giải Động học ngược (`self.model.ik`) để tìm góc khớp mới.
                x, y, z = self.model.fk(base_q)
                target_xyz = (x + dx * step, y + dy * step, z + dz * step)
                q = self.model.ik(target_xyz, base_q)
            if self.set_target_pose(q, False):
                self.move()
        except ValueError as e:
            self.pose_note.set(str(e))
            self.message.set(str(e))

    def preset_paper_center(self):
        """
        [EN] Move the target XY to the configured drawing center `(center_x, center_y)`
             keeping the current safe Z height.
        [VI] Đưa tọa độ đích XY về tâm vùng giấy vẽ `(center_x, center_y)` giữ nguyên Z hiện tại.
        """
        if not self.editor_ready() or self.program_points:
            return
        q0 = self.target_pose or self.model.from_steps(self.state.pos)
        try:
            cx = float(self.bk_vars["center_x"].get() or 0.0)
            cy = float(self.bk_vars["center_y"].get() or 165.0)
            q = self.model.ik((cx, cy, q0[2]), q0)
            if self.set_target_pose(q, False):
                self.move()
        except ValueError as e:
            self.message.set(str(e))

    def preset_park_pose(self):
        """
        [EN] Move the robot to the configured safe park pose (`park_deg` and safe top Z).
        [VI] Đưa robot về tư thế nghỉ an toàn (`park_deg` và độ cao Z nâng an toàn).
        """
        if not self.editor_ready() or self.program_points:
            return
        q0 = self.model.from_steps(self.state.pos)
        branch_sign = 1 if self.model.branch >= 0 else -1
        park_j2 = abs(self.config.park_deg[1]) * branch_sign
        safe_z = min(
            self.model.z_limits_mm[1],
            max(q0[2], self.config.z_span_mm - self.config.z_clear_mm),
        )
        if self.set_target_pose((self.config.park_deg[0], park_j2, safe_z), False):
            self.move()

    def preset_paper_z(self):
        """
        [EN] Lower or raise Z to the confirmed `paper_z` height at the current XY position.
        [VI] Đưa trục Z tới đúng cao độ mặt giấy `paper_z` tại vị trí XY hiện tại.
        """
        if not self.editor_ready() or self.program_points:
            return
        try:
            pz_str = self.bk_vars["paper_z"].get().strip()
            if not pz_str:
                raise ValueError("Chưa nhập Z bút chạm giấy trong Thư viện hình & chữ.")
            q0 = self.target_pose or self.model.from_steps(self.state.pos)
            if self.set_target_pose((q0[0], q0[1], float(pz_str)), False):
                self.move()
        except ValueError as e:
            self.message.set(str(e))

    # ==========================================================================
    # 2. INTERACTIVE POSE & CANVAS MOUSE HANDLERS / XỬ LÝ TƯ THẾ & CHUỘT CANVAS
    # ==========================================================================

    def bind_workspace(self):
        """
        [EN] Bind mouse hover, left-drag IK/elbow (or 3D orbit), scroll-wheel zoom,
             and middle/right-click pan events on `self.canvas`.
        [VI] Gán các sự kiện di chuột, kéo chuột trái chỉnh đầu bút/khuỷu tay (hoặc xoay 3D),
             lăn chuột phóng đại và kéo chuột giữa/phải dịch chuyển trên `self.canvas`.
        """
        self.canvas.bind("<Motion>", self.canvas_hover)
        self.canvas.bind("<ButtonPress-1>", self.canvas_press)
        self.canvas.bind("<B1-Motion>", self.canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.canvas_release)
        self.canvas.bind("<MouseWheel>", self.canvas_wheel)
        self.canvas.bind("<ButtonPress-2>", self.canvas_pan_start)
        self.canvas.bind("<B2-Motion>", self.canvas_pan_drag)
        self.canvas.bind("<ButtonPress-3>", self.canvas_pan_start)
        self.canvas.bind("<B3-Motion>", self.canvas_pan_drag)
        self.canvas.bind("<Double-Button-3>", self.reset_view)

    def editor_ready(self):
        """
        [EN] Return True if the robot has a fresh connection and calibrated model.
        [VI] Trả về True nếu kết nối đang sống và đã có mô hình động học sau hiệu chuẩn.
        """
        return self.fresh() and self.local_reference and self.model is not None

    def set_target_pose(self, q, interactive=False):
        """
        [EN] Validate joint pose `q = (j1_deg, j2_deg, z_mm)`, synchronize the
             joint sliders and XYZ entries, and schedule a follow-mode update if
             `interactive` and `self.follow_enabled` are active.
        [VI] Kiểm tra tư thế khớp `q = (j1_deg, j2_deg, z_mm)`, đồng bộ thanh trượt
             khớp cùng các ô XYZ, và đặt lịch cập nhật bám mô hình nếu đang bật.
        """
        if not self.editor_ready():
            return False
        try:
            self.model.check_pose(q)
            if q[1] * self.model.branch <= 0:
                raise ValueError("Giữ nhánh khuỷu của lượt home hiện tại.")
            self.target_pose = tuple(q)
            xyz = self.model.fk(q)
            self.editor_sync = True
            for v, e, x in zip(self.pose_values, self.pose_entries, q):
                v.set(x)
                e.set(f"{x:.3f}")
            self.editor_sync = False
            for v, x in zip(self.xyz, xyz):
                v.set(f"{x:.3f}")
            self.pose_note.set(
                f"Đích X={xyz[0]:+.2f} · Y={xyz[1]:+.2f} · Z={xyz[2]:.3f} mm   |   J1={q[0]:+.2f}° · J2={q[1]:+.2f}°"
            )
            if interactive and self.follow_enabled.get():
                self.follow_target = xyz
                self.follow_revision += 1
                self.follow_due = time.monotonic() + 0.2
                self.trace_event(
                    "model_target", xyz=xyz, joints=q, revision=self.follow_revision
                )
            self.draw()
            return True
        except ValueError as e:
            self.editor_sync = False
            self.pose_note.set(str(e))
            self.message.set(str(e))
            if interactive:
                self.follow_target = None
                self.follow_revision += 1
            return False

    def sync_editor(self):
        """
        [EN] Update slider ranges from `pose_ranges(self.model)` and snap the
             virtual target arm to the robot's current step position.
        [VI] Cập nhật giới hạn thanh trượt theo `pose_ranges(self.model)` và đồng bộ
             tay máy đích ảo trùng với vị trí xung hiện tại của robot.
        """
        if self.editor_ready():
            self.editor_sync = True
            for scale, (lo, hi) in zip(self.pose_scales, pose_ranges(self.model)):
                scale.configure(from_=lo, to=hi)
            self.editor_sync = False
            self.set_target_pose(self.model.from_steps(self.state.pos))
            self.grid_key = None

    def slider_pose(self):
        """
        [EN] Callback invoked when the user drags any of the J1, J2, or Z sliders.
        [VI] Hàm gọi lại khi người dùng kéo thanh trượt góc khớp J1, J2 hoặc độ cao Z.
        """
        if self.editor_sync or not self.editor_ready() or self.program_points:
            return
        current_slider_q = tuple(v.get() for v in self.pose_values)
        self.set_target_pose(current_slider_q, True)

    def commit_pose(self):
        """
        [EN] Apply typed joint angle / Z values from the text entry boxes.
        [VI] Áp dụng giá trị góc khớp J1, J2 và độ cao Z nhập từ bàn phím.
        """
        if self.program_points:
            return
        try:
            q = tuple(float(v.get()) for v in self.pose_entries)
        except ValueError:
            self.pose_note.set("Góc và Z cần là số.")
            return
        self.set_target_pose(q, True)

    def release_pose(self):
        """
        [EN] Trigger immediate follow-mode dispatch when user releases mouse/slider.
        [VI] Kích hoạt gửi lệnh bám mô hình ngay khi người dùng nhả chuột hoặc thanh trượt.
        """
        if self.follow_enabled.get() and self.follow_target:
            self.follow_due = time.monotonic()

    def canvas_hover(self, event):
        """
        [EN] Update the live world-coordinate readout under the mouse cursor.
        [VI] Cập nhật tọa độ thực tế (mm) tại vị trí con trỏ chuột dưới chân đồ thị.
        """
        if getattr(self, "workspace_mode", "2D") == "3D":
            return
        if self.view:
            x, y = self.view.world(event.x, event.y)
            self.cursor_note.set(
                f"Chuột X={x:+.1f} · Y={y:+.1f} mm   |   Kéo đầu công tác / khuỷu   ·   Chuyển sang '3D Mô hình X-SCARA' ở góc trên phải"
            )

    def canvas_press(self, event):
        """
        [EN] In 3D mode, start orbital rotation; in 2D mode, start dragging either
             the elbow joint (`"j1"`) or the end-effector (`"xy"`).
        [VI] Ở chế độ 3D, bắt đầu xoay góc nhìn; ở chế độ 2D, bắt đầu kéo khớp khuỷu
             (`"j1"`) hoặc kéo đầu công tác (`"xy"`).
        """
        if getattr(self, "workspace_mode", "2D") == "3D":
            self._cam3d_drag = (event.x, event.y, self.cam3d_az, self.cam3d_el)
            return
        if not self.editor_ready() or self.program_points:
            self.message.set("HOME + CALIB để chỉnh mô hình.")
            return

        # Check if the user clicked within 16 pixels of either the target elbow or live elbow.
        # Kiểm tra xem người dùng có nhấp chuột trong bán kính 16 pixel quanh khớp khuỷu hay không.
        q = self.target_pose or self.model.from_steps(self.state.pos)
        live_q = self.model.from_steps(self.state.pos)
        clicked_near_elbow = False
        for pose_candidate in (q, live_q):
            elbow_world_xy = self.model.elbow(pose_candidate)
            ex, ey = self.view.point(*elbow_world_xy)
            if math.hypot(event.x - ex, event.y - ey) < 16:
                clicked_near_elbow = True
                break

        self.drag_kind = "j1" if clicked_near_elbow else "xy"
        self.canvas_drag(event)

    def canvas_drag(self, event):
        """
        [EN] Update 3D camera orbit (in 3D mode) or target pose via IK / J1 (in 2D mode).
        [VI] Cập nhật góc xoay camera 3D (ở chế độ 3D) hoặc tư thế đích qua IK / J1 (ở chế độ 2D).
        """
        if getattr(self, "workspace_mode", "2D") == "3D":
            if getattr(self, "_cam3d_drag", None):
                ox, oy, az, el = self._cam3d_drag
                self.cam3d_az = (az - (event.x - ox) * 0.45) % 360.0
                self.cam3d_el = max(-80.0, min(85.0, el + (event.y - oy) * 0.35))
                self.draw_workspace()
            return
        self.canvas_hover(event)
        if not self.drag_kind or not self.editor_ready():
            return
        x, y = self.view.world(event.x, event.y)
        old = self.target_pose or self.model.from_steps(self.state.pos)
        try:
            if self.drag_kind == "j1":
                # Dragging the elbow directly sets J1 angle (`atan2(-x, y)` because 0° is along +Y).
                # Kéo khớp khuỷu sẽ chỉnh trực tiếp góc J1 (`atan2(-x, y)` vì mốc 0° nằm dọc trục +Y).
                if math.hypot(x, y) < 1:
                    return
                lo, hi = self.model.joint_limits_deg[0]
                raw_j1_deg = math.degrees(math.atan2(-x, y))
                clamped_j1_deg = max(lo, min(hi, raw_j1_deg))
                q = (clamped_j1_deg, old[1], old[2])
            else:
                # Dragging the end-effector solves Inverse Kinematics for `(x, y, old_z)`.
                # Kéo đầu công tác sẽ giải bài toán Động học ngược (IK) cho tọa độ `(x, y, old_z)`.
                q = self.model.ik((x, y, old[2]), old)
            self.set_target_pose(q, True)
        except ValueError as e:
            self.pose_note.set("Đích không hợp lệ: " + str(e))
            self.follow_target = None
            self.follow_revision += 1

    def canvas_release(self, event):
        """
        [EN] End canvas drag interaction and flush pending follow-mode move.
        [VI] Kết thúc thao tác kéo chuột trên đồ thị và phát lệnh bám mô hình nếu có.
        """
        self._cam3d_drag = None
        self.drag_kind = None
        self.release_pose()

    def canvas_wheel(self, event):
        """
        [EN] Zoom the 2D or 3D workspace viewport in/out.
        [VI] Phóng to / thu nhỏ đồ thị 2D hoặc mô hình 3D.
        """
        up = getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0
        if getattr(self, "workspace_mode", "2D") == "3D":
            factor = 1.12 if up else (1.0 / 1.12)
            self.cam3d_zoom = max(0.4, min(3.5, self.cam3d_zoom * factor))
            self.draw_workspace()
            return
        if not hasattr(self, "view") or self.view is None:
            return
        factor = 1.15 if up else (1.0 / 1.15)
        zoomed = self.view.zoomed(factor, event.x, event.y)
        self.view_zoom = zoomed.zoom
        self.view_pan = (zoomed.pan_x, zoomed.pan_y)
        self.draw_workspace()

    def canvas_pan_start(self, event):
        """
        [EN] Record initial mouse and pan offsets when starting a middle/right drag.
        [VI] Ghi lại vị trí chuột ban đầu khi bắt đầu kéo dịch chuyển khung nhìn.
        """
        if getattr(self, "workspace_mode", "2D") == "3D":
            self.pan_origin = (event.x, event.y, *self.cam3d_pan)
        else:
            self.pan_origin = (event.x, event.y, *self.view_pan)

    def canvas_pan_drag(self, event):
        """
        [EN] Pan the 2D or 3D workspace viewport by the mouse drag delta.
        [VI] Dịch chuyển khung nhìn đồ thị 2D hoặc 3D theo độ lệch kéo chuột.
        """
        if not self.pan_origin:
            return
        ox, oy, px, py = self.pan_origin
        if getattr(self, "workspace_mode", "2D") == "3D":
            self.cam3d_pan = (px + event.x - ox, py + event.y - oy)
        else:
            self.view_pan = (px + event.x - ox, py + event.y - oy)
        self.draw_workspace()

    def reset_view(self, _event=None):
        """
        [EN] Reset zoom and pan on double-right-click.
        [VI] Đặt lại độ phóng đại và góc nhìn khi nhấp đúp chuột phải.
        """
        if getattr(self, "workspace_mode", "2D") == "3D":
            self.cam3d_az = 142.0
            self.cam3d_el = 28.0
            self.cam3d_zoom = 1.0
            self.cam3d_pan = (0.0, 0.0)
        else:
            self.view_zoom = 1.0
            self.view_pan = (0.0, 0.0)
        self.pan_origin = None
        self.draw_workspace()

    # ==========================================================================
    # 3. LAYERED 2D / 3D WORKSPACE RENDERING / VẼ ĐỒ THỊ VÙNG LÀM VIỆC 2D & 3D
    # ==========================================================================

    def render_editor(self):
        """
        [EN] Enable or disable pose sliders, jog pad, and setpoint buttons based on robot state.
        [VI] Bật hoặc tắt các thanh trượt tư thế, bàn phím Jog và nút chuỗi điểm tùy theo trạng thái.
        """
        enabled = self.editor_ready() and not self.program_points
        if not self.editor_ready():
            self.target_pose = None
        win_open = bool(self.points_window and self.points_window.winfo_exists())
        editable = (
            not self.program_points
            and not self.follow_enabled.get()
            and self.idle()
        )
        state_key = (enabled, win_open, editable, len(self.point_buttons))
        if state_key == self.editor_state_key:
            return
        self.editor_state_key = state_key
        for widget in self.pose_widgets:
            widget.configure(state="normal" if enabled else "disabled")
        self.follow_checkbox.configure(state="normal" if enabled else "disabled")
        if win_open:
            for b in self.point_buttons:
                b.configure(state="normal" if editable else "disabled")

    def draw_workspace(self):
        """
        [EN] Render either the 3D X-SCARA Digital Twin (when `workspace_mode == "3D"`)
             or the 3-layer 2D XY workspace (`"grid"`, `"waypoints"`, `"dynamic"`).
             Also updates the floating 3D window if open.
        [VI] Vẽ mô hình 3D X-SCARA Digital Twin (khi `workspace_mode == "3D"`) hoặc
             đồ thị vùng làm việc 2D theo 3 lớp (`"grid"`, `"waypoints"`, `"dynamic"`).
             Đồng thời cập nhật cửa sổ 3D rời nếu đang mở.
        """
        if hasattr(self, "draw_3d_window"):
            self.draw_3d_window()
        if not hasattr(self, "canvas"):
            return
        c = self.canvas
        w = max(c.winfo_width(), 100)
        h = max(c.winfo_height(), 100)
        reach = self.config.reach_mm
        self.view = Viewport(w, h, reach, self.view_zoom, *self.view_pan)

        # If in 3D mode, delegate rendering to the 3D Digital Twin engine.
        # Nếu đang ở chế độ 3D, chuyển giao việc vẽ cho bộ dựng 3D Digital Twin.
        if getattr(self, "workspace_mode", "2D") == "3D":
            cam = Camera3D(
                w, h, self.cam3d_az, self.cam3d_el, self.cam3d_zoom, *self.cam3d_pan
            )
            render_scara_3d_on_canvas(self, c, cam)
            return

        p = self.view.point
        bounds = pose_ranges(self.model)[:2] if self.editor_ready() else None

        # ----------------------------------------------------------------------
        # Layer 1: Static Grid, Paper Sheet Zone & Reachable Workspace Envelope
        # Lớp 1: Lưới tọa độ tĩnh, Khung tờ giấy vẽ & Đa giác miền làm việc khả dụng
        # ----------------------------------------------------------------------
        key = (
            w,
            h,
            self.config.l1_mm,
            self.config.l2_mm,
            tuple(self.config.tool_offset_mm),
            bounds,
            round(self.view_zoom, 4),
            round(self.view_pan[0], 1),
            round(self.view_pan[1], 1),
        )
        if key != self.grid_key:
            c.delete("all")
            self.grid_key = key
            self.point_key = None
            self.dynamic_key = None

            # Trace the 4 edges of the (J1, J2) joint limit rectangle using Forward Kinematics
            # to draw the exact reachable workspace polygon (`#142d40`).
            # Quét dọc 4 cạnh của hình chữ nhật giới hạn góc khớp (J1, J2) qua Động học thuận (FK)
            # để vẽ chính xác đa giác miền làm việc khả dụng (`#142d40`).
            if bounds:
                (alo, ahi), (blo, bhi) = bounds
                outline = []
                four_joint_edges = (
                    (alo, blo, ahi - alo, 0),
                    (ahi, blo, 0, bhi - blo),
                    (ahi, bhi, alo - ahi, 0),
                    (alo, bhi, 0, blo - bhi),
                )
                for a, b, da, db in four_joint_edges:
                    for i in range(25):
                        j1_sample = a + da * i / 24
                        j2_sample = b + db * i / 24
                        wx, wy, _wz = self.model.fk((j1_sample, j2_sample, 1))
                        sx, sy = p(wx, wy)
                        outline.append(sx)
                        outline.append(sy)
                c.create_polygon(
                    *outline, fill="#142d40", outline="#29526b", tags="grid"
                )

            # Subtle 110x90 mm standard paper zone around (0, 165).
            # Khung nét đứt biểu diễn vùng đặt giấy tiêu chuẩn 110x90 mm quanh tâm (0, 165).
            c.create_rectangle(
                *p(-55, 210),
                *p(55, 120),
                outline="#2c4d6e",
                dash=(4, 4),
                tags="grid",
            )
            step = 25 if self.view.scale >= 0.9 else 50
            limit = int(reach // step) * step
            for mm in range(-limit, limit + 1, step):
                c.create_line(
                    *p(mm, -reach), *p(mm, reach), fill="#26374d", tags="grid"
                )
                c.create_line(
                    *p(-reach, mm), *p(reach, mm), fill="#26374d", tags="grid"
                )
                if mm:
                    x, y = p(mm, 0)
                    c.create_text(
                        x,
                        y + 12,
                        text=str(mm),
                        fill="#9cb0cb",
                        font=("Segoe UI", 8),
                        tags="grid",
                    )
                    x, y = p(0, mm)
                    c.create_text(
                        x - 9,
                        y,
                        text=str(mm),
                        fill="#9cb0cb",
                        anchor="e",
                        font=("Segoe UI", 8),
                        tags="grid",
                    )
            c.create_oval(
                *p(-reach, reach),
                *p(reach, -reach),
                outline="#44617c",
                dash=(3, 4),
                tags="grid",
            )
            c.create_line(
                *p(-reach, 0),
                *p(reach, 0),
                fill="#819ab7",
                arrow="last",
                width=2,
                tags="grid",
            )
            c.create_line(
                *p(0, -reach),
                *p(0, reach),
                fill="#819ab7",
                arrow="last",
                width=2,
                tags="grid",
            )
            x, y = p(reach, 0)
            c.create_text(
                x, y - 15, text="+X (mm)", fill="#c6d8ec", anchor="e", tags="grid"
            )
            x, y = p(0, reach)
            c.create_text(
                x + 9, y, text="+Y (mm)", fill="#c6d8ec", anchor="w", tags="grid"
            )
            x, y = p(0, 0)
            c.create_text(x + 8, y + 14, text="0", fill="#edf3fc", tags="grid")

        # ----------------------------------------------------------------------
        # Layer 2: Setpoint Waypoint Markers (`tags="waypoints"`)
        # Lớp 2: Các điểm mốc Setpoint màu tím (`tags="waypoints"`)
        # ----------------------------------------------------------------------
        point_key = (
            key,
            self.show_points.get(),
            tuple((point.x, point.y) for point in self.points),
        )
        if point_key != self.point_key:
            c.delete("waypoints")
            self.point_key = point_key
            if self.show_points.get():
                for i, point in enumerate(self.points, 1):
                    x, y = p(point.x, point.y)
                    c.create_oval(
                        x - 3,
                        y - 3,
                        x + 3,
                        y + 3,
                        fill="#ad92ee",
                        outline="",
                        tags="waypoints",
                    )
                    if self.view.scale >= 1:
                        c.create_text(
                            x + 5,
                            y - 8,
                            text=str(i),
                            anchor="w",
                            fill="#ad92ee",
                            font=("Segoe UI", 8),
                            tags="waypoints",
                        )

        # ----------------------------------------------------------------------
        # Layer 3: Dynamic Robot Arm, Target Pose & Trajectory Preview (`tags="dynamic"`)
        # Lớp 3: Tay máy 2 khâu thực tế, Tay máy đích & Đường dự kiến (`tags="dynamic"`)
        # ----------------------------------------------------------------------
        total_trail_pts = sum(len(t) for t in getattr(self, "bk_trails", ()))
        dyn_key = (
            key,
            self.editor_ready(),
            self.state.pos if self.state else None,
            self.target_pose,
            id(self.preview),
            getattr(self, "bk_overlay", ()),
            total_trail_pts,
        )
        if dyn_key == self.dynamic_key:
            return
        self.dynamic_key = dyn_key
        c.delete("dynamic")
        c.create_text(
            12,
            12,
            text="● Robot: xanh   ◇ Đích: vàng   Vùng với tới: nền xanh   ▭ Khung giấy chuẩn: nét đứt",
            anchor="nw",
            fill="#9cb0cb",
            font=("Segoe UI", 9),
            tags="dynamic",
        )

        # Draw planned trajectory preview curve if available.
        # Vẽ đường quỹ đạo dự kiến nét đứt màu vàng nếu có.
        if self.preview:
            coords = []
            for x, y, _z in self.preview.path:
                sx, sy = p(x, y)
                coords.append(sx)
                coords.append(sy)
            if len(coords) > 3:
                c.create_line(
                    *coords, fill="#efc25a", width=2, dash=(3, 3), tags="dynamic"
                )

        # Draw yellow pattern preview overlay on the 2D workspace.
        # Vẽ lớp phủ mẫu hình/chữ màu vàng lên mặt phẳng làm việc 2D.
        self.draw_bk_overlay(c, p)

        # Draw the 2-link SCARA arm:
        # 1) `self.target_pose` (dashed yellow `#efc25a`)
        # 2) `q` live robot pose (Link 1 in blue `#5aaeff`, Link 2 in green `#60d5ae`)
        if self.editor_ready():
            q = self.model.from_steps(self.state.pos)
            arm_poses_to_draw = (
                (self.target_pose, "#efc25a", 3, (4, 3)),
                (q, "#5aaeff", 6, None),
            )
            for pose, color, width, dash in arm_poses_to_draw:
                if pose is None:
                    continue
                p0 = p(0, 0)
                p1 = p(*self.model.elbow(pose))
                p2 = p(*self.model.fk(pose)[:2])
                args = {"fill": color, "width": width, "tags": "dynamic"}
                if dash:
                    args["dash"] = dash
                c.create_line(*p0, *p1, **args)
                args["fill"] = "#60d5ae" if pose is q else color
                c.create_line(*p1, *p2, **args)
                for pos in (p1, p2):
                    c.create_oval(
                        pos[0] - 5,
                        pos[1] - 5,
                        pos[0] + 5,
                        pos[1] + 5,
                        fill=args["fill"],
                        outline="",
                        tags="dynamic",
                    )
        else:
            c.create_text(
                w / 2,
                h / 2 - 24,
                text="HOME + CALIB để xác lập vị trí",
                fill="#9cb0cb",
                tags="dynamic",
            )
        x, y = p(0, 0)
        c.create_oval(
            x - 4, y - 4, x + 4, y + 4, fill="#edf3fc", outline="", tags="dynamic"
        )

    # ==========================================================================
    # 4. SETPOINT SEQUENCE TABLE WINDOW / CỬA SỔ QUẢN LÝ CHUỖI ĐIỂM SETPOINT
    # ==========================================================================

    def open_points(self):
        """
        [EN] Open the Setpoint Sequence Manager window (`self.points_window`).
        [VI] Mở cửa sổ quản lý Điểm thử / Chuỗi setpoint (`self.points_window`).
        """
        if self.points_window and self.points_window.winfo_exists():
            self.points_window.lift()
            return
        win = tk.Toplevel(self)
        self.points_window = win
        win.transient(self)
        win.title("Điểm thử / Chuỗi setpoint · mm")
        win.geometry("850x590")
        win.configure(bg="#172338")
        self.point_buttons = []
        self.label(
            win,
            "Chọn dòng để nhập đích. Chạy chuỗi sẽ kiểm tra tất cả đoạn trước khi bắt đầu.",
            10,
        ).pack(anchor="w", padx=12, pady=8)
        pane = tk.Frame(win, bg="#172338")
        pane.pack(fill="both", expand=True, padx=12)
        tree = ttk.Treeview(
            pane,
            style="Scara.Treeview",
            columns=("name", "x", "y", "z", "speed", "dwell"),
            show="headings",
            selectmode="browse",
        )
        for column, title, width in zip(
            tree["columns"],
            ("Tên", "X (mm)", "Y (mm)", "Z (mm)", "mm/s", "Nghỉ (s)"),
            (130, 100, 100, 100, 90, 85),
        ):
            tree.heading(column, text=title)
            tree.column(column, width=width, anchor="center")
        scrollbar = ttk.Scrollbar(pane, orient="vertical", command=tree.yview)
        scrollbar.pack(side="right", fill="y")
        tree.configure(yscrollcommand=scrollbar.set)
        tree.pack(side="left", fill="both", expand=True)
        self.points_tree = tree
        tree.bind("<<TreeviewSelect>>", lambda event: self.select_point())
        row = tk.Frame(win, bg="#172338")
        row.pack(fill="x", padx=12, pady=8)
        for title, command in (
            ("Tạo điểm thử", self.generate_points),
            ("Thêm đích", self.add_point),
            ("Sửa dòng", self.edit_point),
            ("Xóa", self.delete_point),
            ("Nạp CSV", self.import_points),
            ("Lưu CSV", self.export_points),
        ):
            b = self.button(row, title, command)
            b.pack(side="left", padx=2)
            self.point_buttons.append(b)
        row = tk.Frame(win, bg="#172338")
        row.pack(fill="x", padx=12, pady=(0, 8))
        for title, command in (
            ("Chạy điểm chọn", self.run_selected_point),
            ("Chạy toàn bộ chuỗi", self.start_program),
        ):
            b = self.button(row, title, command, "#245d9c")
            b.pack(side="left", padx=2)
            self.point_buttons.append(b)
        self.button(row, "■ DỪNG", self.stop, "#90364c").pack(side="right")
        tk.Checkbutton(
            win,
            text="Hiện các điểm trên đồ thị",
            variable=self.show_points,
            command=self.draw,
            bg="#172338",
            fg="#edf3fc",
            selectcolor="#243651",
        ).pack(anchor="w", padx=12)
        self.label(
            win,
            "Điểm thử giữ Z hiện tại. Đường đi là nội suy khớp; quỹ đạo XY có thể cong.",
            9,
            "#9cb0cb",
        ).pack(anchor="w", padx=12, pady=(4, 10))
        if not self.points and self.editor_ready():
            self.generate_points()
        self.refresh_points()
        self.render_editor()

    def refresh_points(self):
        """
        [EN] Repopulate the Treeview table from `self.points` and redraw waypoints.
        [VI] Cập nhật lại bảng Treeview từ danh sách `self.points` và vẽ lại các điểm.
        """
        if not self.points_window or not self.points_window.winfo_exists():
            return
        self.points_tree.delete(*self.points_tree.get_children())
        for i, p in enumerate(self.points):
            px, py, pz = p.xyz
            row_values = (
                p.name,
                f"{px:.3f}",
                f"{py:.3f}",
                f"{pz:.3f}",
                f"{p.speed_mm_s:g}",
                f"{p.dwell_s:g}",
            )
            self.points_tree.insert("", "end", iid=str(i), values=row_values)
        self.draw()

    def selected_index(self):
        """
        [EN] Return the integer index of the currently selected row in `points_tree`.
        [VI] Trả về chỉ số dòng đang được chọn trong bảng `points_tree`.
        """
        selection = (
            self.points_tree.selection() if hasattr(self, "points_tree") else ()
        )
        return int(selection[0]) if selection else None

    def select_point(self):
        """
        [EN] Copy the selected table row's XYZ and speed into the editor and preview IK.
        [VI] Sao chép tọa độ XYZ và tốc độ của dòng đang chọn vào ô nhập và hiển thị tư thế.
        """
        i = self.selected_index()
        if i is None or self.program_points or self.planning or not self.editor_ready():
            return
        self.follow_enabled.set(False)
        self.follow_target = None
        p = self.points[i]
        for v, x in zip(self.xyz, p.xyz):
            v.set(f"{x:.3f}")
        self.speed.set(str(p.speed_mm_s))
        try:
            current_q = self.model.from_steps(self.state.pos)
            self.set_target_pose(self.model.ik(p.xyz, current_q))
        except ValueError as e:
            self.message.set(str(e))

    def generate_points(self):
        """
        [EN] Populate `self.points` with a 3x3 reachable test grid at the current Z height.
        [VI] Tạo danh sách điểm thử lưới 3x3 nằm trong vùng làm việc tại độ cao Z hiện tại.
        """
        if not self.operable() or not self.editor_ready() or self.program_points:
            return
        try:
            current_z = self.model.from_steps(self.state.pos)[2]
            current_speed = float(self.speed.get())
            self.points = test_points(self.model, current_z, current_speed)
        except (ValueError, AttributeError) as e:
            self.message.set(str(e))
            return
        self.show_points.set(True)
        self.refresh_points()
        self.trace_event("test_points", points=[vars(p) for p in self.points])

    def add_point(self):
        """
        [EN] Append the current XYZ entry values as a new `Setpoint` in `self.points`.
        [VI] Thêm tọa độ XYZ hiện tại vào cuối danh sách `self.points`.
        """
        if not self.operable() or self.program_points:
            return
        if len(self.points) >= 500:
            self.message.set("Tối đa500 điểm.")
            return
        try:
            point_name = f"P{len(self.points) + 1:02d}"
            x_val = float(self.xyz[0].get())
            y_val = float(self.xyz[1].get())
            z_val = float(self.xyz[2].get())
            spd_val = float(self.speed.get())
            new_setpoint = Setpoint(
                point_name, x_val, y_val, z_val, spd_val
            ).validate()
            self.points.append(new_setpoint)
        except ValueError as e:
            self.message.set(str(e))
            return
        self.refresh_points()

    def edit_point(self):
        """
        [EN] Open a modal dialog to edit the selected `Setpoint` row.
        [VI] Mở hộp thoại chỉnh sửa thông số của dòng `Setpoint` đang chọn.
        """
        i = self.selected_index()
        if i is None or self.program_points:
            return
        p = self.points[i]
        win = tk.Toplevel(self)
        win.title("Sửa setpoint")
        win.configure(bg="#172338")
        win.transient(self.points_window)
        win.grab_set()
        fields = []
        for name, value in zip(
            ("Tên", "X", "Y", "Z", "mm/s", "Nghỉ (s)"),
            (p.name, *p.xyz, p.speed_mm_s, p.dwell_s),
        ):
            row = tk.Frame(win, bg="#172338")
            row.pack(fill="x", padx=12, pady=4)
            self.label(row, name).pack(side="left")
            v = tk.StringVar(value=str(value))
            self.entry(row, v).pack(side="right")
            fields.append(v)
        note = tk.StringVar()
        self.label(win, textvariable=note, color="#ff8795", wraplength=300).pack()

        def save():
            try:
                if self.program_points:
                    raise ValueError("Dừng chuỗi trước khi sửa.")
                numeric_vals = [float(v.get()) for v in fields[1:]]
                self.points[i] = Setpoint(
                    fields[0].get(), *numeric_vals
                ).validate()
                self.refresh_points()
                win.destroy()
            except ValueError as e:
                note.set(str(e))

        self.button(win, "Lưu điểm", save).pack(pady=8)

    def delete_point(self):
        """
        [EN] Delete the currently selected row from `self.points`.
        [VI] Xóa điểm đang chọn khỏi danh sách `self.points`.
        """
        i = self.selected_index()
        if i is not None and not self.program_points:
            self.points.pop(i)
            self.refresh_points()

    def import_points(self):
        """
        [EN] Load a setpoint list from a UTF-8 CSV file.
        [VI] Nạp danh sách điểm setpoint từ file CSV.
        """
        if self.program_points:
            return
        path = filedialog.askopenfilename(
            parent=self.points_window,
            initialdir=self.config_path.parent,
            filetypes=[("Setpoints CSV", "*.csv")],
        )
        if path:
            try:
                self.points = load_points(path)
                self.refresh_points()
            except (OSError, ValueError) as e:
                self.message.set(str(e))

    def export_points(self):
        """
        [EN] Save `self.points` to a UTF-8 CSV file.
        [VI] Lưu danh sách `self.points` ra file CSV.
        """
        path = filedialog.asksaveasfilename(
            parent=self.points_window,
            initialdir=self.config_path.parent,
            defaultextension=".csv",
            initialfile="scara_setpoints.csv",
            filetypes=[("Setpoints CSV", "*.csv")],
        )
        if path:
            try:
                save_points(path, self.points)
            except (OSError, ValueError) as e:
                self.message.set(str(e))

    def run_selected_point(self):
        """
        [EN] Validate and execute a single move to the currently selected table row.
        [VI] Kiểm duyệt và di chuyển tới điểm đang được chọn trong bảng.
        """
        i = self.selected_index()
        if i is not None:
            self.start_plan(
                True, target=self.points[i].xyz, speed=self.points[i].speed_mm_s
            )
