"""
================================================================================
FILE: cartesian_nc_layout.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Pure UI layout construction for the upgraded SCARA Motion Studio dashboard
     (`build_dashboard`, `update_dashboard`) and the Drawing Studio window
     (`build_drawing_studio`, `draw_thumbnail`):
     - 6 live telemetry metric cards (`X`, `Y`, `Z`, `J1 · J2`, `XUNG & FIFO`, `MỐC`).
     - Interactive 2D XY / 3D X-SCARA Digital Twin switcher bar & SVG snapshot export.
     - 6 visual LED limit-switch badges (`Z+`, `Z-`, `J1+`, `J1-`, `J2+`, `J2-`).
     - Custom Text (A-Z, 0-9) and Parametric Pattern Generator tab in Drawing Studio.

     TKINTER GEOMETRY MANAGERS EXPLAINED (For 1st/2nd-Year Students):
     Tkinter uses two main layout managers in this file:
     1. `.grid(row=..., column=...)`: Arranges widgets in a 2D table. Calling
        `parent.columnconfigure(c, weight=1)` or `rowconfigure(r, weight=1)` tells
        Tkinter which row/column should stretch when the window is resized.
     2. `.pack(side=..., fill=..., expand=...)`: Stacks widgets along one edge
        (`"left"`, `"right"`, `"top"`, `"bottom"`) inside a container frame.

[VI] MỤC ĐÍCH:
     Xây dựng bố cục giao diện nâng cấp toàn diện cho bảng điều khiển chính SCARA
     Motion Studio (`build_dashboard`, `update_dashboard`) và cửa sổ Drawing Studio
     (`build_drawing_studio`, `draw_thumbnail`):
     - 6 thẻ hiển thị thông số trực tiếp (`X`, `Y`, `Z`, `J1 · J2`, `XUNG & FIFO`, `MỐC`).
     - Thanh chuyển đổi đồ thị 2D XY / Mô hình 3D X-SCARA trực tiếp & nút xuất ảnh SVG.
     - 6 đèn LED trực quan hiển thị trạng thái công tắc hành trình (`Z±`, `J1±`, `J2±`).
     - Tab tạo Chữ tùy ý (A-Z, 0-9) và Mẫu hình học tham số trong cửa sổ Drawing Studio.

     GIẢI THÍCH CÁCH BỐ TRÍ GIAO DIỆN TKINTER (Dành cho Sinh viên năm 1–2):
     File này sử dụng 2 cơ chế sắp xếp vị trí chính của Tkinter:
     1. `.grid(row=..., column=...)`: Đặt phần tử vào ô hàng/cột của bảng 2 chiều.
        Lệnh `columnconfigure(c, weight=1)` hoặc `rowconfigure(r, weight=1)` chỉ định
        cột/hàng nào sẽ tự động giãn rộng khi người dùng phóng to cửa sổ.
     2. `.pack(side=..., fill=..., expand=...)`: Xếp nối tiếp các phần tử dọc theo
        một cạnh (`"left"`, `"right"`, `"top"`, `"bottom"`) bên trong khung chứa.
================================================================================
"""

import math
import tkinter as tk
from tkinter import ttk
from cartesian_nc_drawing import BKSettings, pattern_strokes, pattern_dimensions
from cartesian_nc_shapes import PATTERNS

from cartesian_nc_theme import (
    BG,
    CARD,
    LINE,
    MUTED,
    TEXT,
    ACCENT,
    GREEN,
    RED,
    DASHBOARD_BLUE as BLUE,
)


# ==============================================================================
# 1. REUSABLE PANEL, HEADING & TTK STYLING / KHUNG THẺ, TIÊU ĐỀ & STYLE TTK
# ==============================================================================


def panel(parent, padx=16, pady=12):
    """
    [EN] Create a bordered card frame (`tk.Frame`) with `CARD` fill and `LINE` border.
    [VI] Tạo khung thẻ có viền mảnh (`tk.Frame`) với màu nền `CARD` và viền `LINE`.
    """
    return tk.Frame(
        parent,
        bg=CARD,
        padx=padx,
        pady=pady,
        highlightbackground=LINE,
        highlightthickness=1,
    )


def heading(app, parent, title, subtitle=None):
    """
    [EN] Render a section title and optional muted subtitle inside `parent`.
    [VI] Hiển thị tiêu đề phân mục và dòng mô tả phụ (nếu có) bên trong `parent`.
    """
    app.label(parent, title, 12, TEXT).pack(anchor="w")
    if subtitle:
        app.label(parent, subtitle, 9, MUTED, wraplength=300, justify="left").pack(
            anchor="w", pady=(3, 8)
        )


def configure_style(app):
    """
    [EN] Configure dark-navy `ttk.Style` rules for Combobox, Notebook tabs, and Treeview.
    [VI] Thiết lập quy tắc giao diện tối `ttk.Style` cho Combobox, tab Notebook và bảng Treeview.
    """
    style = ttk.Style(app)
    style.theme_use("clam")

    # Style for dropdown menus (COM port selector & pattern selector).
    # Định dạng màu sắc cho danh sách thả xuống (chọn cổng COM và chọn mẫu vẽ).
    style.configure(
        "TCombobox",
        fieldbackground="#21344e",
        background="#21344e",
        foreground=TEXT,
        arrowcolor=MUTED,
        bordercolor=LINE,
        padding=5,
    )
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", "#21344e"), ("disabled", "#19273b")],
        foreground=[("readonly", TEXT), ("disabled", "#72849c")],
    )

    # Style for tabbed notebooks (`Scara.TNotebook`).
    # Định dạng màu sắc cho các khung chuyển tab (`Scara.TNotebook`).
    style.configure("Scara.TNotebook", background=CARD, borderwidth=0)
    style.configure(
        "Scara.TNotebook.Tab",
        background="#1c3049",
        foreground=MUTED,
        padding=(11, 7),
        font=("Segoe UI", 9, "bold"),
    )
    style.map(
        "Scara.TNotebook.Tab",
        background=[("selected", "#24597a")],
        foreground=[("selected", TEXT)],
    )

    # Style for data tables (`Scara.Treeview` in Setpoints and Analytics windows).
    # Định dạng bảng dữ liệu (`Scara.Treeview` trong cửa sổ Setpoint và Phân tích).
    style.configure(
        "Scara.Treeview",
        background=CARD,
        fieldbackground=CARD,
        foreground=TEXT,
        rowheight=28,
        font=("Segoe UI", 9),
    )
    style.configure(
        "Scara.Treeview.Heading",
        background="#21344e",
        foreground=TEXT,
        font=("Segoe UI", 9, "bold"),
    )


# ==============================================================================
# 2. MAIN DASHBOARD LAYOUT / BỐ CỤC BẢNG ĐIỀU KHIỂN CHÍNH
# ==============================================================================


def build_dashboard(app):
    """
    [EN] Build the upgraded SCARA Motion Studio window:
         - Row 0: Header banner, connection status badge, 3D Viewer, Analytics & Drawing Studio launchers.
         - Row 1: Top USB/COM toolbar (`Kết nối`, `Cơ khí`, `HOME / CALIB`, `Nhả/Giữ arm`, `DỪNG`).
         - Row 2: 6 live metric cards (`X`, `Y`, `Z`, `GÓC KHỚP J1·J2`, `XUNG & FIFO`, `MỐC & TRẠNG THÁI`).
         - Row 3: Interactive 2D/3D workspace canvas (left) + XYZ / Chỉnh mô hình / Jog & Tùy biến / Home tabs (right).
         - Row 4: Status, 6 visual LED switch badges & Firmware diagnostic footer notebook + log button.
    [VI] Xây dựng bố cục cửa sổ chính SCARA Motion Studio nâng cấp:
         - Hàng 0: Tiêu đề ứng dụng, huy hiệu kết nối, nút mở Mô hình 3D, Phân tích & Báo cáo, Thư viện hình & chữ.
         - Hàng 1: Thanh công cụ USB/COM (`Kết nối`, `Cơ khí`, `HOME / CALIB`, `Nhả/Giữ arm`, `DỪNG`).
         - Hàng 2: 6 thẻ thông số trực tiếp (`X`, `Y`, `Z`, `GÓC KHỚP J1·J2`, `XUNG & FIFO`, `MỐC & TRẠNG THÁI`).
         - Hàng 3: Đồ thị vùng làm việc 2D/3D (trái) + khung tab điều khiển XYZ / Mô hình / Jog / Home (phải).
         - Hàng 4: Khung trạng thái kèm 6 đèn LED công tắc & chẩn đoán firmware dưới cùng + nút Mở log.
    """
    configure_style(app)
    outer = tk.Frame(app, bg=BG, padx=20, pady=14)
    outer.pack(fill="both", expand=True)
    outer.columnconfigure(0, weight=1)
    outer.rowconfigure(3, weight=1)

    # --------------------------------------------------------------------------
    # Row 0: Header Banner & Studio Window Launchers
    # Hàng 0: Tiêu đề ứng dụng & các nút mở cửa sổ chuyên dụng
    # --------------------------------------------------------------------------
    head = tk.Frame(outer, bg=BG)
    head.grid(row=0, column=0, sticky="ew", pady=(0, 12))
    brand = tk.Frame(head, bg=BG)
    brand.pack(side="left")
    app.label(brand, "SCARA  /  MOTION & DIGITAL TWIN STUDIO", 20).pack(anchor="w")
    app.label(
        brand,
        "ĐIỀU KHIỂN ĐỘNG HỌC XYZ  ·  MÔ HÌNH 3D STL  ·  PHÂN TÍCH DỮ LIỆU & BÁO CÁO",
        9,
        MUTED,
    ).pack(anchor="w", pady=(2, 0))

    app.badge = app.label(head, "● CHƯA KẾT NỐI", 10, MUTED)
    app.badge.pack(side="right", padx=(12, 0))
    app.studio_btn = app.button(head, "🎨 Thư viện hình & chữ", app.open_bk, "#266b73")
    app.studio_btn.pack(side="right", padx=4)

    if hasattr(app, "open_analytics"):
        app.analytics_btn = app.button(
            head, "📊 Phân tích & Báo cáo", app.open_analytics, "#2a528a"
        )
        app.analytics_btn.pack(side="right", padx=4)
    if hasattr(app, "open_3d_window"):
        app.win3d_btn = app.button(
            head, "🧊 Cửa sổ 3D rời", app.open_3d_window, "#3b477a"
        )
        app.win3d_btn.pack(side="right", padx=4)
    if hasattr(app, "open_calibration_wizard"):
        app.calib_wiz_btn = app.button(
            head, "🎯 Hiệu chuẩn Nâng cao", app.open_calibration_wizard, "#285a48"
        )
        app.calib_wiz_btn.pack(side="right", padx=4)

    # --------------------------------------------------------------------------
    # Row 1: Connection & Hardware Action Bar
    # Hàng 1: Thanh công cụ Kết nối USB/COM & Điều khiển phần cứng
    # --------------------------------------------------------------------------
    bar = panel(outer, pady=8)
    bar.grid(row=1, column=0, sticky="ew", pady=(0, 10))
    app.label(bar, "USB / COM", 9, MUTED).pack(side="left", padx=(0, 10))
    app.ports = ttk.Combobox(bar, textvariable=app.port, width=12, state="readonly")
    app.ports.pack(side="left")
    app.button(bar, "↻", app.refresh_ports).pack(side="left", padx=6)
    app.connect_btn = app.button(bar, "Kết nối", app.toggle_connection, "#2866a2")
    app.connect_btn.pack(side="left")
    app.settings_btn = app.button(bar, "⚙ Cơ khí", app.settings)
    app.settings_btn.pack(side="left", padx=8)
    app.home_btn = app.button(bar, "⌂ HOME / CALIB", app.initialize, "#206c61")
    app.home_btn.pack(side="left", padx=3)
    if hasattr(app, "quick_initialize"):
        app.quick_home_btn = app.button(
            bar, "⚡ HOME Nhanh", app.quick_initialize, "#1f5a73"
        )
        app.quick_home_btn.pack(side="left", padx=3)
    app.release_btn = app.button(bar, "Nhả arm", lambda: app.hold(False))
    app.release_btn.pack(side="left", padx=3)
    app.hold_btn = app.button(bar, "Giữ arm", lambda: app.hold(True))
    app.hold_btn.pack(side="left", padx=3)
    app.stop_btn = app.button(bar, "■ DỪNG TẤT CẢ  /  Esc", app.stop, "#a53b55")
    app.stop_btn.pack(side="right")

    # --------------------------------------------------------------------------
    # Row 2: 6 Live Telemetry Readout Cards (X, Y, Z, Joints J1/J2, Speed/FIFO, Reference)
    # Hàng 2: 6 thẻ hiển thị thông số trực tiếp (X, Y, Z, Góc khớp, Tốc độ/FIFO, Mốc)
    # --------------------------------------------------------------------------
    metrics = tk.Frame(outer, bg=BG)
    metrics.grid(row=2, column=0, sticky="ew", pady=(0, 10))
    app.axis_readouts = [tk.StringVar(value="—") for _ in range(3)]
    app.joints_readout = tk.StringVar(value="— / —")
    app.joints_sub_readout = tk.StringVar(value="J1 · J2 (°) · |sin J2|")
    app.pps_fifo_readout = tk.StringVar(value="0 pps")
    app.pps_fifo_sub = tk.StringVar(value="FIFO: 0/32 đoạn · Nhàn rỗi")
    app.reference_readout = tk.StringVar(value="CHƯA LẤY MỐC")

    cards_spec = (
        ("X · NGANG", app.axis_readouts[0], BLUE, "mm · vị trí theo xung", 17),
        ("Y · TRUNG TUYẾN", app.axis_readouts[1], ACCENT, "mm · vị trí theo xung", 17),
        ("Z · ĐỘ CAO", app.axis_readouts[2], "#c4a2f4", "mm · vị trí theo xung", 17),
        ("GÓC KHỚP J1 · J2", app.joints_readout, "#efc25a", app.joints_sub_readout, 14),
        ("TỐC ĐỘ & BỘ ĐỆM", app.pps_fifo_readout, GREEN, app.pps_fifo_sub, 14),
        ("MỐC & CHUYỂN ĐỘNG", app.reference_readout, MUTED, "HOME + CALIB xác lập mốc", 12),
    )
    for i, (title, var, color, sub, fsize) in enumerate(cards_spec):
        metrics.columnconfigure(i, weight=1, uniform="metrics")
        card = panel(metrics, padx=12, pady=6)
        right_pad = (0, 8) if i < 5 else 0
        card.grid(row=0, column=i, sticky="nsew", padx=right_pad)
        app.label(card, title, 8, MUTED).pack(anchor="w")
        app.label(card, textvariable=var, size=fsize, color=color, anchor="w").pack(
            fill="x", pady=(2, 0)
        )
        if isinstance(sub, tk.StringVar):
            app.label(card, textvariable=sub, size=8, color=MUTED, anchor="w").pack(
                fill="x"
            )
        else:
            app.label(card, sub, 8, MUTED).pack(anchor="w")

    # --------------------------------------------------------------------------
    # Row 3: Interactive 2D/3D Workspace (Left) & Multi-Tab Control Panel (Right)
    # Hàng 3: Đồ thị vùng làm việc 2D/3D (Trái) & Khung tab điều khiển (Phải)
    # --------------------------------------------------------------------------
    body = tk.Frame(outer, bg=BG)
    body.grid(row=3, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    body.rowconfigure(0, weight=1)

    workspace = panel(body, padx=14, pady=10)
    workspace.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
    row = tk.Frame(workspace, bg=CARD)
    row.pack(fill="x", pady=(0, 6))
    app.label(row, "KHÔNG GIAN LÀM VIỆC & MÔ HÌNH SỐ", 11).pack(side="left")

    # 2D / 3D Mode Switcher & SVG Export Bar
    # Thanh chuyển đổi chế độ 2D / 3D và nút xuất ảnh vector SVG
    mode_bar = tk.Frame(row, bg=CARD)
    mode_bar.pack(side="right")
    if hasattr(app, "set_workspace_mode"):
        app.mode_2d_btn = app.button(
            mode_bar, "2D Mặt phẳng XY", lambda: app.set_workspace_mode("2D"), "#24597a"
        )
        app.mode_2d_btn.configure(font=("Segoe UI", 8, "bold"), padx=8, pady=2)
        app.mode_2d_btn.pack(side="left", padx=2)
        app.mode_3d_btn = app.button(
            mode_bar,
            "3D Mô hình X-SCARA",
            lambda: app.set_workspace_mode("3D"),
            "#1c3049",
        )
        app.mode_3d_btn.configure(font=("Segoe UI", 8, "bold"), padx=8, pady=2)
        app.mode_3d_btn.pack(side="left", padx=2)
        svg_btn = app.button(
            mode_bar, "📷 Xuất SVG", app.export_workspace_svg, "#1f5c4b"
        )
        svg_btn.configure(font=("Segoe UI", 8), padx=7, pady=2)
        svg_btn.pack(side="left", padx=(6, 0))
    else:
        app.label(row, "Gốc tại J1 · đơn vị mm", 9, MUTED).pack(side="right")

    app.canvas = tk.Canvas(workspace, bg="#101d2f", highlightthickness=0)
    app.canvas.pack(fill="both", expand=True)
    app.canvas.bind("<Configure>", lambda e: app.draw())
    app.bind_workspace()
    app.label(
        workspace, textvariable=app.cursor_note, size=9, color=MUTED, anchor="w"
    ).pack(fill="x", pady=(5, 0))
    app.label(
        workspace,
        textvariable=app.pose_note,
        size=9,
        color="#efc25a",
        anchor="w",
        wraplength=780,
        justify="left",
    ).pack(fill="x")
    app.label(
        workspace,
        textvariable=app.details,
        size=9,
        color=MUTED,
        anchor="w",
        justify="left",
        wraplength=780,
    ).pack(fill="x", pady=(2, 0))

    # Right Control Panel with Notebook Tabs
    # Bảng điều khiển bên phải chứa các tab chức năng
    control = panel(body, padx=14, pady=10)
    control.grid(row=0, column=1, sticky="nsew")
    heading(app, control, "TRUNG TÂM ĐIỀU KHIỂN")
    app.progress_label = app.label(
        control,
        textvariable=app.progress,
        size=9,
        color=ACCENT,
        wraplength=320,
        justify="left",
        height=1,
        anchor="nw",
    )
    app.progress_label.pack(fill="x", pady=(2, 4))
    xyz = app.build_workspace_tabs(control)
    home = tk.Frame(app.workspace_tabs, bg=CARD)
    app.workspace_tabs.add(home, text="Home")
    heading(app, home, "HOME & CALIB", "Đo hai biên, đo tỷ lệ bù rồi lấy mốc tọa độ.")
    app.home_hint = app.label(home, "", 9, MUTED, justify="left", wraplength=300)
    app.home_hint.pack(anchor="w", pady=(8, 8))
    if hasattr(app, "open_calibration_wizard"):
        app.button(
            home,
            "🎯 Mở Trình hướng dẫn Hiệu chuẩn Nâng cao",
            app.open_calibration_wizard,
            "#285a48",
        ).pack(fill="x", pady=(2, 6))
    app.label(
        home,
        "Nhấn HOME / CALIB (hoặc ⚡ HOME Nhanh sau lần calib đầu) trên thanh kết nối.\nCông tắc NC và mất USB luôn được bảo vệ.\nNhả arm làm mất mốc; Z luôn giữ lực.",
        9,
        MUTED,
        wraplength=300,
        justify="left",
    ).pack(anchor="w", pady=8)

    for name, var in zip(("X · mm", "Y · mm", "Z · mm"), app.xyz):
        row = tk.Frame(xyz, bg=CARD)
        row.pack(fill="x", pady=3)
        app.label(row, name, 10, MUTED).pack(side="left")
        app.entry(row, var).pack(side="right")
    row = tk.Frame(xyz, bg=CARD)
    row.pack(fill="x", pady=(6, 6))
    app.label(row, "Tốc độ · mm/s", 10, MUTED).pack(side="left")
    app.entry(row, app.speed).pack(side="right")
    app.preview_btn = app.button(xyz, "Xem trước đường đi", app.preview_target)
    app.preview_btn.pack(fill="x", pady=2)
    app.move_btn = app.button(xyz, "DI CHUYỂN ĐẾN XYZ", app.move, "#2866a2")
    app.move_btn.pack(fill="x", pady=2)
    app.current_btn = app.button(xyz, "Lấy vị trí hiện tại", app.current_target)
    app.current_btn.pack(fill="x", pady=2)

    # --------------------------------------------------------------------------
    # Row 4: Footer Status, Visual Switch LED Badges & Firmware Diagnostics Notebook
    # Hàng 4: Khung trạng thái dưới cùng, 6 đèn LED công tắc & Chẩn đoán Firmware
    # --------------------------------------------------------------------------
    foot = panel(outer, pady=7)
    foot.grid(row=4, column=0, sticky="ew", pady=(10, 0))
    app.status_tabs = ttk.Notebook(foot, style="Scara.TNotebook", height=45)
    app.status_tabs.pack(fill="x")
    status = tk.Frame(app.status_tabs, bg=CARD)
    diagnostic = tk.Frame(app.status_tabs, bg=CARD)
    app.status_tabs.add(status, text="Trạng thái & công tắc")
    app.status_tabs.add(diagnostic, text="Chẩn đoán firmware")
    app.status_message = app.label(
        status,
        textvariable=app.message,
        size=10,
        wraplength=1200,
        anchor="w",
        justify="left",
    )
    app.status_message.pack(fill="x", pady=(4, 2))
    app.switch_label = app.label(
        status,
        textvariable=app.switch_text,
        size=9,
        color=ACCENT,
        anchor="w",
        wraplength=1200,
    )
    app.switch_label.pack(fill="x")
    app.fw_label = app.label(
        diagnostic,
        "Chờ firmware",
        9,
        MUTED,
        anchor="w",
        wraplength=1200,
        justify="left",
    )
    app.fw_label.pack(fill="x", pady=(3, 0))
    app.input_label = app.label(
        diagnostic,
        textvariable=app.input_details,
        size=9,
        color=MUTED,
        anchor="w",
        wraplength=1200,
        justify="left",
    )
    app.input_label.pack(fill="x", pady=(3, 0))

    # Bottom LED Bar + Log Launcher
    # Thanh 6 đèn LED báo công tắc NC và nút mở thư mục Log
    row = tk.Frame(foot, bg=CARD)
    row.pack(fill="x", pady=(4, 0))
    app.switch_leds = []
    led_bar = tk.Frame(row, bg=CARD)
    led_bar.pack(side="left", padx=(0, 12))
    app.label(led_bar, "NC LED:", 8, MUTED).pack(side="left", padx=(0, 4))
    for sw_name in ("Z+", "Z−", "J1+", "J1−", "J2+", "J2−"):
        badge = tk.Label(
            led_bar,
            text=f"● {sw_name}",
            bg="#1c2d42",
            fg=MUTED,
            font=("Segoe UI", 8, "bold"),
            padx=5,
            pady=1,
        )
        badge.pack(side="left", padx=2)
        app.switch_leds.append((sw_name, badge))

    app.label(row, textvariable=app.log_text, size=8, color=MUTED, anchor="w").pack(
        side="left", fill="x", expand=True
    )
    log = app.button(row, "Mở log", app.open_logs)
    log.configure(font=("Segoe UI", 9), pady=1)
    log.pack(side="right")

    def wrap_status(event):
        """
        [EN] Dynamically adjust text wrap-length when the footer is resized.
        [VI] Tự động điều chỉnh độ rộng ngắt dòng khi người dùng thay đổi cỡ cửa sổ.
        """
        width = max(500, event.width - 40)
        for widget in (
            app.status_message,
            app.switch_label,
            app.fw_label,
            app.input_label,
        ):
            if int(widget.cget("wraplength")) != width:
                widget.configure(wraplength=width)

    foot.bind("<Configure>", wrap_status)


def update_dashboard(app):
    """
    [EN] Refresh all 6 top telemetry cards, the 6 visual LED limit-switch badges,
         and dynamically adjust footer tab height.
    [VI] Cập nhật cả 6 thẻ số đo trực tiếp phía trên, 6 đèn LED công tắc hành trình
         và tự động co giãn chiều cao khung trạng thái dưới cùng.
    """
    if not hasattr(app, "axis_readouts"):
        return

    def update(var, value):
        """
        [EN] Only call `var.set(value)` if the string value actually changed.
        [VI] Chỉ gán `var.set(value)` khi chuỗi hiển thị thực sự thay đổi.
        """
        if var.get() != value:
            var.set(value)

    # 1. Update X, Y, Z, Joint Angles (J1/J2), and Reference Status Cards
    # 1. Cập nhật các thẻ X, Y, Z, Góc khớp (J1/J2) và Trạng thái mốc
    if app.editor_ready():
        q = app.model.from_steps(app.state.pos)
        xyz = app.model.fk(q)
        for var, value in zip(app.axis_readouts, xyz):
            update(var, f"{value:+.3f}")

        ref_status_str = "ĐANG CHẠY" if app.state.busy else "SẴN SÀNG"
        update(app.reference_readout, ref_status_str)

        if hasattr(app, "joints_readout"):
            # Manipulability index |sin(J2)| measures how far the elbow is from singularity (0° or 180°).
            # Chỉ số khả thao tác |sin(J2)| đo khoảng cách tới điểm suy biến duỗi thẳng/gập sát (0°/180°).
            manip = abs(math.sin(math.radians(q[1])))
            branch_label = "+ (Trái)" if q[1] >= 0 else "− (Phải)"
            update(app.joints_readout, f"{q[0]:+.1f}° / {q[1]:+.1f}°")
            update(
                app.joints_sub_readout,
                f"|sin J2|={manip:.3f} · Nhánh {branch_label}",
            )
    else:
        for var in app.axis_readouts:
            update(var, "—")
        is_calibrating = bool(app.state and app.state.busy)
        update(
            app.reference_readout,
            "ĐANG CALIB" if is_calibrating else "CHƯA LẤY MỐC",
        )
        if hasattr(app, "joints_readout"):
            update(app.joints_readout, "— / —")
            update(app.joints_sub_readout, "J1 · J2 (°) · |sin J2|")

    # 2. Update Step Rate (pps) and STM32 Segment FIFO Occupancy Card
    # 2. Cập nhật thẻ Tốc độ phát xung (pps) và mức đầy bộ đệm FIFO 32 đoạn trên STM32
    if app.state and hasattr(app, "pps_fifo_readout"):
        s = app.state
        fifo_free = s.path[3] if len(s.path) >= 4 else 32
        fifo_used = max(0, min(32, 32 - fifo_free))
        busy_label = "Đang bận" if s.busy else "Nhàn rỗi"
        update(app.pps_fifo_readout, f"{s.pps:,} pps")
        update(
            app.pps_fifo_sub,
            f"FIFO: {fifo_used}/32 đoạn · {busy_label}",
        )

    # 3. Update 6 Visual NC Switch LED Badges (`Z+`, `Z−`, `J1+`, `J1−`, `J2+`, `J2−`)
    # 3. Cập nhật 6 đèn LED trực quan báo trạng thái công tắc hành trình thường đóng (NC)
    if app.state and hasattr(app, "switch_leds"):
        sw_mask = app.state.switches
        for bit_idx, (sw_name, badge) in enumerate(app.switch_leds):
            # Extract bit `bit_idx` from the 6-bit `sw_mask` integer.
            # Trích xuất bit thứ `bit_idx` từ số nguyên 6-bit `sw_mask`.
            is_open = bool((sw_mask >> bit_idx) & 1)
            bg_col = "#6e2236" if is_open else "#173d38"
            fg_col = RED if is_open else GREEN
            txt = f"● {sw_name}:HỞ" if is_open else f"● {sw_name}:OK"
            if badge.cget("text") != txt:
                badge.configure(text=txt, bg=bg_col, fg=fg_col)

    # 4. Auto-switch to Firmware Diagnostics tab on hardware fault and adjust height
    # 4. Tự động chuyển sang tab Chẩn đoán Firmware nếu có lỗi phần cứng và căn chiều cao
    if app.state and (app.state.fault or app.state.conflicts or app.state.errors):
        app.status_tabs.select(1)
    if app.status_tabs.index("current") == 1:
        height = max(
            75, app.fw_label.winfo_reqheight() + app.input_label.winfo_reqheight() + 10
        )
    else:
        height = max(
            45,
            app.status_message.winfo_reqheight()
            + app.switch_label.winfo_reqheight()
            + 9,
        )
    if int(app.status_tabs.cget("height")) != height:
        app.status_tabs.configure(height=height)


# ==============================================================================
# 3. DRAWING STUDIO WINDOW LAYOUT / BỐ CỤC CỬA SỔ DRAWING STUDIO
# ==============================================================================


def draw_thumbnail(canvas, pattern):
    """
    [EN] Render a miniature vector preview of `pattern` onto a gallery tile `canvas`.
    [VI] Vẽ hình thu nhỏ (thumbnail) của mẫu `pattern` lên ô `canvas` trong thư viện.
    """
    # Create temporary preview settings centered at (0, 0) with width = 20 mm.
    # Tạo thông số mẫu tạm thời đặt tâm tại (0, 0) với chiều rộng 20 mm.
    settings = BKSettings(
        pattern=pattern, size_mm=20, center_x=0, center_y=0, step_mm=2
    )
    width, height = pattern_dimensions(settings)
    w = int(canvas.cget("width"))
    h = int(canvas.cget("height"))

    # Compute uniform pixel-per-mm scale so the pattern fits inside the tile with padding.
    # Tính hệ số phóng đại (pixel/mm) sao cho hình vẽ vừa khít bên trong ô có chừa lề.
    scale_x = (w - 20) / width
    scale_y = (h - 12) / height
    scale = min(scale_x, scale_y)

    # Convert each 2D stroke vertex `(vx, vy)` into canvas pixel coordinates `(sx, sy)`.
    # Note: Canvas Y increases downward, so we subtract `vy * scale` from `h / 2`.
    # Đổi từng đỉnh `(vx, vy)` sang tọa độ điểm ảnh `(sx, sy)` trên Canvas.
    # Lưu ý: Trục Y của Canvas hướng xuống dưới nên ta lấy `h / 2` trừ đi `vy * scale`.
    for stroke in pattern_strokes(settings):
        flat_coords = []
        for vertex in stroke:
            vx, vy = vertex.xy
            screen_x = w / 2 + vx * scale
            screen_y = h / 2 - vy * scale
            flat_coords.append(screen_x)
            flat_coords.append(screen_y)

        canvas.create_line(
            *flat_coords,
            fill=ACCENT,
            width=2,
        )


def build_drawing_studio(app, win):
    """
    [EN] Build the 3-column Drawing Studio modal window (`win`):
         - Column 0: Pattern gallery (`Hình vẽ` / `Chữ` tabs + SVG/DXF/G-code import).
         - Column 1: Interactive 2D stroke preview canvas with live ink trail overlay.
         - Column 2: Size, center XY, paper-Z confirmation, speed, tolerance, and
           the new `"Tùy biến"` tab for Custom Text (A-Z, 0-9) and Parametric Patterns.
         - Bottom Dock: `Xem trước`, `Chạy khô`, `BẮT ĐẦU VẼ`, and `DỪNG / Esc` buttons.
    [VI] Xây dựng bố cục 3 cột cho cửa sổ Drawing Studio (`win`):
         - Cột 0: Thư viện mẫu (`Hình vẽ` / `Chữ` + nút Nhập SVG / DXF / G-code).
         - Cột 1: Khung Canvas xem trước đường đi các nét vẽ và vết bút thực tế.
         - Cột 2: Các ô thiết lập kích thước, tâm XY, xác nhận Z giấy, tốc độ, dung sai
           và tab `"Tùy biến"` tạo Chữ tùy ý (A-Z, 0-9) hoặc Mẫu tham số.
         - Thanh dưới cùng: Các nút `Xem trước`, `Chạy khô`, `BẮT ĐẦU VẼ` và `DỪNG / Esc`.
    """
    win.configure(bg=BG)
    win.columnconfigure(1, weight=1)
    win.rowconfigure(1, weight=1)

    head = tk.Frame(win, bg=BG, padx=18, pady=12)
    head.grid(row=0, column=0, columnspan=3, sticky="ew")
    app.label(head, "DRAWING & CAD/CAM STUDIO", 18).pack(side="left")
    app.bk_status = tk.StringVar(value="CHƯA XÁC NHẬN Z GIẤY")
    app.label(head, textvariable=app.bk_status, size=9, color=ACCENT).pack(side="right")

    # --------------------------------------------------------------------------
    # Column 0: Pattern Gallery & Vector File Importer
    # Cột 0: Thư viện mẫu có sẵn & Nút nạp file Vector ngoài
    # --------------------------------------------------------------------------
    gallery = panel(win, padx=10)
    gallery.grid(row=1, column=0, sticky="ns", padx=(16, 10))
    heading(app, gallery, "01  /  THƯ VIỆN MẪU")
    tabs = ttk.Notebook(gallery, style="Scara.TNotebook")
    tabs.pack(fill="both", expand=True, pady=(8, 0))
    app.bk_tiles = {}
    app.bk_gallery_tabs = tabs

    for group in ("Hình vẽ", "Chữ"):
        page = tk.Frame(tabs, bg=CARD)
        tabs.add(page, text=group)

        # Collect all patterns belonging to this tab group ("Hình vẽ" or "Chữ").
        # Lọc danh sách các mẫu thuộc nhóm tab hiện tại ("Hình vẽ" hoặc "Chữ").
        entries = []
        for key, data in PATTERNS.items():
            if data[1] == group:
                entries.append((key, data))

        # Arrange gallery tiles in a 2-column grid (`row = i // 2`, `col = i % 2`).
        # Xếp các ô mẫu thành bảng 2 cột (`row = i // 2`, `col = i % 2`).
        for i, (key, data) in enumerate(entries):
            tile = tk.Frame(
                page,
                bg="#1a2c43",
                highlightbackground=LINE,
                highlightthickness=2,
                padx=3,
                pady=4,
            )
            tile.grid(row=i // 2, column=i % 2, padx=3, pady=4, sticky="ew")
            c = tk.Canvas(tile, bg="#1a2c43", width=96, height=57, highlightthickness=0)
            c.pack()
            draw_thumbnail(c, key)
            b = app.button(
                tile, data[0], lambda k=key: app.choose_pattern(k), "#1a2c43"
            )
            b.configure(font=("Segoe UI", 9), padx=2, pady=3)
            b.pack(fill="x")
            c.bind("<Button-1>", lambda e, k=key: app.choose_pattern(k))
            app.bk_tiles[key] = (tile, b)

        current_pat = app.bk_vars["pattern"].get()
        if current_pat in PATTERNS and PATTERNS[current_pat][1] == group:
            tabs.select(page)

    if hasattr(app, "open_vector_file"):
        imp = app.button(
            gallery, "📂 Nhập SVG / DXF / G-code", app.open_vector_file, "#213f5f"
        )
        imp.configure(font=("Segoe UI", 9), pady=3)
        imp.pack(fill="x", pady=(8, 0))
    app.label(
        gallery,
        "Chọn mẫu → chỉnh cỡ/tâm\n→ xác nhận giấy → chạy khô",
        9,
        MUTED,
        justify="left",
    ).pack(anchor="w", pady=(8, 0))

    # --------------------------------------------------------------------------
    # Column 1: Stroke Preview Canvas
    # Cột 1: Khung Canvas xem trước các nét vẽ và vết mực thực tế
    # --------------------------------------------------------------------------
    preview = panel(win, padx=12)
    preview.grid(row=1, column=1, sticky="nsew")
    row = tk.Frame(preview, bg=CARD)
    row.pack(fill="x", pady=(0, 8))
    app.bk_title = tk.StringVar()
    app.bk_dimensions = tk.StringVar()
    app.label(row, textvariable=app.bk_title, size=13).pack(side="left")
    app.label(row, textvariable=app.bk_dimensions, size=9, color=MUTED).pack(
        side="right"
    )
    app.bk_canvas = tk.Canvas(
        preview, bg="#101d2f", highlightthickness=0, width=420, height=420
    )
    app.bk_canvas.pack(fill="both", expand=True)
    app.bk_canvas.bind("<Configure>", lambda e: app.draw_bk_window())
    app.label(
        preview,
        "Vàng: đường dự kiến  ·  Hồng: vị trí theo xung  ·  Số: thứ tự nét  ·  Kéo chuột để dời tâm XY",
        8,
        MUTED,
    ).pack(anchor="w", pady=(7, 0))
    app.label(
        preview,
        textvariable=app.bk_note,
        size=10,
        color="#e8c979",
        wraplength=440,
        justify="left",
        anchor="w",
    ).pack(fill="x", pady=(7, 0))

    # --------------------------------------------------------------------------
    # Column 2: Drawing Parameters, Paper-Z Confirmation & Custom Generator Tab
    # Cột 2: Thông số bản vẽ, Xác nhận Z mặt giấy & Tab tạo Chữ/Mẫu tùy ý
    # --------------------------------------------------------------------------
    controls = panel(win, padx=14)
    controls.grid(row=1, column=2, sticky="ns", padx=(10, 16))
    heading(app, controls, "02  /  THIẾT LẬP")
    app.bk_fields = []
    app.bk_buttons = []

    initial_pattern_code = app.bk_vars["pattern"].get()
    initial_pattern_title = PATTERNS.get(initial_pattern_code, ("BK",))[0]
    app.bk_choice = tk.StringVar(value=initial_pattern_title)

    pattern_display_titles = tuple(v[0] for v in PATTERNS.values())
    choices = ttk.Combobox(
        controls,
        textvariable=app.bk_choice,
        values=pattern_display_titles,
        width=23,
        state="readonly",
    )
    choices.pack(fill="x", pady=(6, 8))

    def on_pattern_combobox_selected(_event):
        """
        [EN] Look up the pattern code matching the selected display title and activate it.
        [VI] Tìm mã mẫu ứng với tên hiển thị được chọn trong Combobox và kích hoạt mẫu đó.
        """
        chosen_title = app.bk_choice.get()
        for pat_code, pat_info in PATTERNS.items():
            if pat_info[0] == chosen_title:
                app.choose_pattern(pat_code)
                break

    choices.bind("<<ComboboxSelected>>", on_pattern_combobox_selected)
    app.bk_fields.append(choices)

    def field(parent, key, title):
        """
        [EN] Create a labeled numeric entry row bound to `app.bk_vars[key]`.
        [VI] Tạo một dòng gồm nhãn tên thông số và ô nhập gắn với `app.bk_vars[key]`.
        """
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=3)
        app.label(row, title, 10, MUTED).pack(side="left", padx=(0, 8))
        entry = app.entry(row, app.bk_vars[key])
        entry.configure(width=8)
        entry.pack(side="right")
        app.bk_fields.append(entry)

    app.label(controls, "BỐ CỤC · mm", 9, BLUE).pack(anchor="w")
    for key, title in (
        ("size_mm", "Chiều rộng"),
        ("center_x", "Tâm X"),
        ("center_y", "Tâm Y"),
    ):
        field(controls, key, title)
    app.bk_fit_button = app.button(
        controls, "Phóng lớn vừa vùng", app.fit_bk, "#344c79"
    )
    app.bk_fit_button.pack(fill="x", pady=(4, 8))
    app.label(controls, "BÚT & TỐC ĐỘ", 9, BLUE).pack(anchor="w")
    for key, title in (
        ("paper_z", "Z bút chạm giấy · mm"),
        ("lift_mm", "Nâng bút · mm"),
        ("draw_speed", "Vẽ · mm/s"),
    ):
        field(controls, key, title)
    capture = app.button(
        controls, "Lấy Z hiện tại làm mặt giấy", app.capture_paper, "#344c79"
    )
    capture.configure(font=("Segoe UI", 10))
    capture.pack(fill="x", pady=3)
    confirm = app.button(controls, "Xác nhận Z đã nhập", app.confirm_paper)
    confirm.configure(font=("Segoe UI", 10))
    confirm.pack(fill="x", pady=2)
    app.bk_buttons.extend((capture, confirm))

    details = ttk.Notebook(controls, style="Scara.TNotebook", height=150)
    details.pack(fill="x", pady=(8, 0))
    advanced = tk.Frame(details, bg=CARD)
    details.add(advanced, text="Nâng cao")
    for key, title in (
        ("travel_speed", "Chuyển bút · mm/s"),
        ("step_mm", "Bước chia nét · mm"),
        ("tolerance_mm", "Sai lệch ≤ · mm"),
        ("settle_s", "Chờ bút · s"),
    ):
        field(advanced, key, title)

    # New Tab: Custom Text (A-Z, 0-9) & Parametric Generator
    # Tab mới: Tạo Chữ tùy ý (A-Z, 0-9) & Mẫu hình học tham số
    if hasattr(app, "build_custom_drawing_tab"):
        custom_tab = tk.Frame(details, bg=CARD, padx=4, pady=4)
        details.add(custom_tab, text="Chữ & Mẫu tùy ý")
        app.build_custom_drawing_tab(custom_tab)

    # --------------------------------------------------------------------------
    # Row 2: Bottom Action Dock
    # Hàng 2: Thanh nút bấm điều khiển chạy dưới cùng
    # --------------------------------------------------------------------------
    dock = panel(win, padx=14, pady=9)
    dock.grid(row=2, column=0, columnspan=3, sticky="ew", padx=16, pady=(10, 12))
    app.label(
        dock,
        "Bézier bậc 3 · profile bậc 5 · Tối ưu đường đi 2-Opt TSP\nBiên NC và mất kết nối luôn được bảo vệ",
        9,
        MUTED,
        justify="left",
    ).pack(side="left")
    stop = app.button(dock, "■ DỪNG / Esc", app.stop, "#a53b55")
    stop.pack(side="right", padx=(8, 0))
    app.bk_stop_button = stop
    run = app.button(dock, "BẮT ĐẦU VẼ", lambda: app.start_bk(False), "#206c61")
    run.pack(side="right", padx=4)
    dry = app.button(dock, "Chạy khô", lambda: app.start_bk(True), "#2866a2")
    dry.pack(side="right", padx=4)
    check = app.button(dock, "Xem trước", app.preview_bk)
    check.pack(side="right", padx=4)
    app.bk_buttons.extend((check, dry, run))
