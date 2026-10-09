"""Dashboard and drawing studio layouts; motion decisions stay in NCApp."""

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
    DASHBOARD_BLUE as BLUE,
)


def panel(parent, padx=16, pady=12):
    return tk.Frame(
        parent,
        bg=CARD,
        padx=padx,
        pady=pady,
        highlightbackground=LINE,
        highlightthickness=1,
    )


def heading(app, parent, title, subtitle=None):
    app.label(parent, title, 12, TEXT).pack(anchor="w")
    if subtitle:
        app.label(parent, subtitle, 9, MUTED, wraplength=300, justify="left").pack(
            anchor="w", pady=(3, 8)
        )


def configure_style(app):
    style = ttk.Style(app)
    style.theme_use("clam")
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
    style.configure("Scara.TNotebook", background=CARD, borderwidth=0)
    style.configure(
        "Scara.TNotebook.Tab",
        background="#1c3049",
        foreground=MUTED,
        padding=(13, 8),
        font=("Segoe UI", 10),
    )
    style.map(
        "Scara.TNotebook.Tab",
        background=[("selected", "#24597a")],
        foreground=[("selected", TEXT)],
    )
    style.configure(
        "Scara.Treeview",
        background=CARD,
        fieldbackground=CARD,
        foreground=TEXT,
        rowheight=30,
        font=("Segoe UI", 10),
    )
    style.configure(
        "Scara.Treeview.Heading",
        background="#21344e",
        foreground=TEXT,
        font=("Segoe UI", 10),
    )


def build_dashboard(app):
    configure_style(app)
    outer = tk.Frame(app, bg=BG, padx=20, pady=16)
    outer.pack(fill="both", expand=True)
    outer.columnconfigure(0, weight=1)
    outer.rowconfigure(3, weight=1)
    head = tk.Frame(outer, bg=BG)
    head.grid(row=0, column=0, sticky="ew", pady=(0, 14))
    brand = tk.Frame(head, bg=BG)
    brand.pack(side="left")
    app.label(brand, "SCARA  /  MOTION STUDIO", 22).pack(anchor="w")
    app.label(brand, "ĐIỀU KHIỂN XYZ  ·  MÔ HÌNH  ·  THƯ VIỆN HÌNH VẼ", 9, MUTED).pack(
        anchor="w", pady=(2, 0)
    )
    app.badge = app.label(head, "● CHƯA KẾT NỐI", 10, MUTED)
    app.badge.pack(side="right", padx=14)
    app.studio_btn = app.button(head, "Thư viện hình & chữ", app.open_bk, "#266b73")
    app.studio_btn.pack(side="right", padx=5)

    bar = panel(outer, pady=9)
    bar.grid(row=1, column=0, sticky="ew", pady=(0, 12))
    app.label(bar, "USB / COM", 9, MUTED).pack(side="left", padx=(0, 10))
    app.ports = ttk.Combobox(bar, textvariable=app.port, width=12, state="readonly")
    app.ports.pack(side="left")
    app.button(bar, "↻", app.refresh_ports).pack(side="left", padx=6)
    app.connect_btn = app.button(bar, "Kết nối", app.toggle_connection, "#2866a2")
    app.connect_btn.pack(side="left")
    app.settings_btn = app.button(bar, "Cơ khí", app.settings)
    app.settings_btn.pack(side="left", padx=8)
    app.home_btn = app.button(bar, "HOME / CALIB", app.initialize, "#206c61")
    app.home_btn.pack(side="left", padx=3)
    app.release_btn = app.button(bar, "Nhả arm", lambda: app.hold(False))
    app.release_btn.pack(side="left", padx=3)
    app.hold_btn = app.button(bar, "Giữ arm", lambda: app.hold(True))
    app.hold_btn.pack(side="left", padx=3)
    app.stop_btn = app.button(bar, "■ DỪNG TẤT CẢ  /  Esc", app.stop, "#a53b55")
    app.stop_btn.pack(side="right")

    metrics = tk.Frame(outer, bg=BG)
    metrics.grid(row=2, column=0, sticky="ew", pady=(0, 12))
    app.axis_readouts = [tk.StringVar(value="—") for _ in range(3)]
    app.reference_readout = tk.StringVar(value="CHƯA LẤY MỐC")
    for i, (title, var, color) in enumerate(
        zip(
            ("X · NGANG", "Y · TRUNG TUYẾN", "Z · ĐỘ CAO", "MỐC & CHUYỂN ĐỘNG"),
            (*app.axis_readouts, app.reference_readout),
            (BLUE, ACCENT, "#c4a2f4", MUTED),
        )
    ):
        metrics.columnconfigure(i, weight=1, uniform="metrics")
        card = panel(metrics, pady=7)
        card.grid(row=0, column=i, sticky="nsew", padx=(0, 9) if i < 3 else 0)
        app.label(card, title, 9, MUTED).pack(anchor="w")
        app.label(
            card, textvariable=var, size=18 if i < 3 else 12, color=color, anchor="w"
        ).pack(fill="x", pady=(3, 0))
        app.label(
            card,
            "mm · vị trí theo xung" if i < 3 else "HOME + CALIB để xác lập tọa độ",
            8,
            MUTED,
        ).pack(anchor="w")

    body = tk.Frame(outer, bg=BG)
    body.grid(row=3, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    body.rowconfigure(0, weight=1)
    workspace = panel(body)
    workspace.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
    row = tk.Frame(workspace, bg=CARD)
    row.pack(fill="x", pady=(0, 8))
    app.label(row, "VÙNG LÀM VIỆC XY", 12).pack(side="left")
    app.label(row, "Gốc tại J1 · đơn vị mm", 9, MUTED).pack(side="right")
    app.canvas = tk.Canvas(workspace, bg="#101d2f", highlightthickness=0)
    app.canvas.pack(fill="both", expand=True)
    app.canvas.bind("<Configure>", lambda e: app.draw())
    app.bind_workspace()
    app.label(
        workspace, textvariable=app.cursor_note, size=9, color=MUTED, anchor="w"
    ).pack(fill="x", pady=(6, 0))
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
    ).pack(fill="x", pady=(4, 0))

    control = panel(body)
    control.grid(row=0, column=1, sticky="nsew")
    heading(app, control, "ĐIỀU KHIỂN")
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
    app.progress_label.pack(fill="x", pady=(4, 4))
    xyz = app.build_workspace_tabs(control)
    home = tk.Frame(app.workspace_tabs, bg=CARD)
    app.workspace_tabs.add(home, text="Home")
    heading(app, home, "HOME & CALIB", "Đo hai biên, đo tỷ lệ bù rồi lấy mốc tọa độ.")
    app.home_hint = app.label(home, "", 9, MUTED, justify="left", wraplength=300)
    app.home_hint.pack(anchor="w", pady=(8, 8))
    app.label(
        home,
        "Nhấn HOME / CALIB trên thanh kết nối.\nCông tắc NC và mất USB luôn được bảo vệ.\nNhả arm làm mất mốc; Z luôn giữ lực.",
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
    row.pack(fill="x", pady=(7, 8))
    app.label(row, "Tốc độ · mm/s", 10, MUTED).pack(side="left")
    app.entry(row, app.speed).pack(side="right")
    app.preview_btn = app.button(xyz, "Xem trước đường đi", app.preview_target)
    app.preview_btn.pack(fill="x", pady=3)
    app.move_btn = app.button(xyz, "DI CHUYỂN ĐẾN XYZ", app.move, "#2866a2")
    app.move_btn.pack(fill="x", pady=3)
    app.current_btn = app.button(xyz, "Lấy vị trí hiện tại", app.current_target)
    app.current_btn.pack(fill="x", pady=3)

    foot = panel(outer, pady=8)
    foot.grid(row=4, column=0, sticky="ew", pady=(12, 0))
    app.status_tabs = ttk.Notebook(foot, style="Scara.TNotebook", height=45)
    app.status_tabs.pack(fill="x")
    status = tk.Frame(app.status_tabs, bg=CARD)
    diagnostic = tk.Frame(app.status_tabs, bg=CARD)
    app.status_tabs.add(status, text="Trạng thái & công tắc")
    app.status_tabs.add(diagnostic, text="Chẩn đoán firmware")
    app.status_message = app.label(
        status,
        textvariable=app.message,
        size=11,
        wraplength=1200,
        anchor="w",
        justify="left",
    )
    app.status_message.pack(fill="x", pady=(6, 3))
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
    row = tk.Frame(foot, bg=CARD)
    row.pack(fill="x", pady=(4, 0))
    app.label(row, textvariable=app.log_text, size=8, color=MUTED, anchor="w").pack(
        side="left", fill="x", expand=True
    )
    log = app.button(row, "Mở log", app.open_logs)
    log.configure(font=("Segoe UI", 9), pady=1)
    log.pack(side="right")

    def wrap_status(event):
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
    if not hasattr(app, "axis_readouts"):
        return

    def update(var, value):
        if var.get() != value:
            var.set(value)

    if app.editor_ready():
        xyz = app.model.fk(app.model.from_steps(app.state.pos))
        for var, value in zip(app.axis_readouts, xyz):
            update(var, f"{value:+.3f}")
        update(app.reference_readout, "ĐANG CHẠY" if app.state.busy else "SẴN SÀNG")
    else:
        for var in app.axis_readouts:
            update(var, "—")
        update(
            app.reference_readout,
            "ĐANG CALIB" if app.state and app.state.busy else "CHƯA LẤY MỐC",
        )
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


def draw_thumbnail(canvas, pattern):
    settings = BKSettings(
        pattern=pattern, size_mm=20, center_x=0, center_y=0, step_mm=2
    )
    width, height = pattern_dimensions(settings)
    w = int(canvas.cget("width"))
    h = int(canvas.cget("height"))
    scale = min((w - 20) / width, (h - 12) / height)
    for stroke in pattern_strokes(settings):
        canvas.create_line(
            *(
                v
                for vertex in stroke
                for v in (w / 2 + vertex.xy[0] * scale, h / 2 - vertex.xy[1] * scale)
            ),
            fill=ACCENT,
            width=2,
        )


def build_drawing_studio(app, win):
    win.configure(bg=BG)
    win.columnconfigure(1, weight=1)
    win.rowconfigure(1, weight=1)
    head = tk.Frame(win, bg=BG, padx=18, pady=14)
    head.grid(row=0, column=0, columnspan=3, sticky="ew")
    app.label(head, "DRAWING STUDIO", 19).pack(side="left")
    app.bk_status = tk.StringVar(value="CHƯA XÁC NHẬN Z GIẤY")
    app.label(head, textvariable=app.bk_status, size=9, color=ACCENT).pack(side="right")

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
        entries = [(key, data) for key, data in PATTERNS.items() if data[1] == group]
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
        if PATTERNS[app.bk_vars["pattern"].get()][1] == group:
            tabs.select(page)
    app.label(
        gallery,
        "Chọn mẫu → chỉnh cỡ/tâm\n→ xác nhận giấy → chạy khô",
        9,
        MUTED,
        justify="left",
    ).pack(anchor="w", pady=(14, 0))

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
        "Vàng: đường dự kiến  ·  Hồng: vị trí theo xung  ·  Số: thứ tự nét",
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

    controls = panel(win, padx=14)
    controls.grid(row=1, column=2, sticky="ns", padx=(10, 16))
    heading(app, controls, "02  /  THIẾT LẬP")
    app.bk_fields = []
    app.bk_buttons = []
    app.bk_choice = tk.StringVar(value=PATTERNS[app.bk_vars["pattern"].get()][0])
    choices = ttk.Combobox(
        controls,
        textvariable=app.bk_choice,
        values=tuple(v[0] for v in PATTERNS.values()),
        width=23,
        state="readonly",
    )
    choices.pack(fill="x", pady=(8, 10))
    choices.bind(
        "<<ComboboxSelected>>",
        lambda e: app.choose_pattern(
            next(k for k, v in PATTERNS.items() if v[0] == app.bk_choice.get())
        ),
    )
    app.bk_fields.append(choices)

    def field(parent, key, title):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=4)
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
    app.bk_fit_button.pack(fill="x", pady=(5, 12))
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
    capture.pack(fill="x", pady=4)
    confirm = app.button(controls, "Xác nhận Z đã nhập", app.confirm_paper)
    confirm.configure(font=("Segoe UI", 10))
    confirm.pack(fill="x", pady=3)
    app.bk_buttons.extend((capture, confirm))
    details = ttk.Notebook(controls, style="Scara.TNotebook", height=150)
    details.pack(fill="x", pady=(12, 0))
    advanced = tk.Frame(details, bg=CARD)
    details.add(advanced, text="Nâng cao")
    for key, title in (
        ("travel_speed", "Chuyển bút · mm/s"),
        ("step_mm", "Bước chia nét · mm"),
        ("tolerance_mm", "Sai lệch ≤ · mm"),
        ("settle_s", "Chờ bút · s"),
    ):
        field(advanced, key, title)

    dock = panel(win, padx=14, pady=10)
    dock.grid(row=2, column=0, columnspan=3, sticky="ew", padx=16, pady=(12, 14))
    app.label(
        dock,
        "Bézier bậc 3 · profile bậc 5\nBiên NC và mất kết nối luôn được bảo vệ",
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
