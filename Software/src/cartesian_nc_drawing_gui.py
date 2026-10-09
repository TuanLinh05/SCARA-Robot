"""BK controls: explicit paper-Z confirmation, preview, dry run and ink run."""

from dataclasses import asdict
import math
import time
import tkinter as tk
from tkinter import ttk
from cartesian_nc_drawing import (
    BKSettings,
    pattern_strokes,
    pattern_dimensions,
    compile_bk,
    fit_drawing,
)
from cartesian_nc_shapes import PATTERNS


class DrawingUI:
    def init_drawing(self):
        self.bk_vars = {
            name: tk.StringVar(value="" if value is None else str(value))
            for name, value in asdict(BKSettings()).items()
        }
        self.bk_window = None
        self.bk_fields = []
        self.bk_buttons = []
        self.bk_note = tk.StringVar(value="Xác định Z giấy trước khi chạy.")
        self.bk_confirmed_epoch = None
        self.bk_confirmed_z = None
        self.bk_pending = False
        self.bk_overlay = ()
        self.bk_overlay_settings = None
        self.bk_overlay_key = None
        self.bk_plan = None
        self.bk_active = False
        self.bk_active_plan = None
        self.bk_trails = [[], [], []]
        self.bk_canvas_key = None
        self.bk_selected_pattern = "BK"
        self.bk_control_key = None
        for variable in self.bk_vars.values():
            variable.trace_add("write", self.bk_changed)

    def bk_changed(self, *args):
        self.bk_plan = None
        self.bk_canvas_key = None

    def bk_settings(self):
        try:
            data = {
                name: (
                    variable.get().strip().upper()
                    if name == "pattern"
                    else (
                        None
                        if name == "paper_z" and not variable.get().strip()
                        else float(variable.get())
                    )
                )
                for name, variable in self.bk_vars.items()
            }
        except ValueError:
            raise ValueError("Thông số kích thước/Z cần là số.")
        return BKSettings(**data).validate()

    def bk_confirmed(self):
        try:
            settings = self.bk_settings()
        except ValueError:
            return False
        return (
            self.editor_ready()
            and self.bk_confirmed_epoch == self.state.epoch
            and settings.paper_z == self.bk_confirmed_z
            and settings.paper_z is not None
        )

    def open_bk(self):
        if self.bk_window and self.bk_window.winfo_exists():
            self.bk_window.lift()
            return
        from cartesian_nc_layout import build_drawing_studio

        win = tk.Toplevel(self)
        self.bk_window = win
        win.transient(self)
        win.title("SCARA · Drawing Studio · GUI R13 / firmware R9")
        win.geometry("1200x920")
        win.minsize(1080, 880)
        build_drawing_studio(self, win)
        self.bk_control_key = None
        self.preview_bk()
        self.render_drawing()

    def select_drawing_pattern(self):
        # These are preset widths, not an automatic motion or a paper-Z change.
        choice = self.bk_vars["pattern"].get()
        try:
            width = float(self.bk_vars["size_mm"].get())
        except ValueError:
            width = 0
        previous = PATTERNS.get(self.bk_selected_pattern, PATTERNS["BK"])
        if choice in PATTERNS and (width == previous[2] or not width):
            self.bk_vars["size_mm"].set(str(PATTERNS[choice][2]))
        self.bk_selected_pattern = choice
        if self.bk_window and self.bk_window.winfo_exists() and choice in PATTERNS:
            self.bk_gallery_tabs.select(0 if PATTERNS[choice][1] == "Hình vẽ" else 1)
        self.preview_bk()

    def choose_pattern(self, pattern):
        if self._active_task() or pattern not in PATTERNS:
            return
        self.bk_vars["pattern"].set(pattern)
        self.select_drawing_pattern()

    def capture_paper(self):
        if (
            not self.operable()
            or not self.editor_ready()
            or self.program_points
            or self.follow_enabled.get()
        ):
            return
        z = self.model.from_steps(self.state.pos)[2]
        self.bk_vars["paper_z"].set(f"{z:.6f}")
        self.confirm_paper()

    def fit_bk(self):
        if not self.operable() or not self.editor_ready() or self._active_task():
            return
        try:
            settings = self.bk_settings()
        except ValueError as e:
            self.bk_note.set(str(e))
            return
        self.submit_planning(
            lambda model, start, cancelled: fit_drawing(
                model, start, settings, cancelled
            ),
            purpose="drawing_fit",
            name="SCARA drawing fit",
        )
        self.bk_note.set(
            "Đang tìm cỡ lớn tại tâm đã chọn và kiểm tra toàn bộ đường đi; không di chuyển robot…"
        )

    def accept_fit(self, settings):
        self.bk_vars["size_mm"].set(f"{settings.size_mm:.2f}")
        self.preview_bk()
        width, height = pattern_dimensions(settings)
        self.bk_note.set(
            f"Cỡ đã kiểm tra: {width:g}×{height:.2f}mm, tâm({settings.center_x:g},{settings.center_y:g}).\nCó chừa 4% theo kích thước; chạy khô trước khi vẽ."
        )
        self.trace_event("drawing_fit", settings=asdict(settings))

    def confirm_paper(self):
        if (
            not self.operable()
            or not self.editor_ready()
            or self.program_points
            or self.follow_enabled.get()
        ):
            return
        try:
            settings = self.bk_settings().validate(paper=True)
            lo, hi = self.model.z_limits_mm
            if not lo <= settings.paper_z < settings.paper_z + settings.lift_mm <= hi:
                raise ValueError(
                    "Z giấy hoặc Z nâng nằm ngoài hành trình. Chỉnh Z/độ nhấc bút."
                )
            self.bk_confirmed_epoch = self.state.epoch
            self.bk_confirmed_z = settings.paper_z
            self.bk_note.set(
                f"Đã xác nhận Z giấy={settings.paper_z:.3f}; Z nâng={settings.paper_z+settings.lift_mm:.3f}mm."
            )
            self.trace_event(
                "paper_confirmed", epoch=self.state.epoch, settings=asdict(settings)
            )
        except ValueError as e:
            self.bk_note.set(str(e))

    def preview_bk(self):
        try:
            settings = self.bk_settings()
            strokes = pattern_strokes(settings)
            if self.editor_ready():
                z = self.model.from_steps(self.state.pos)[2]
                for stroke in strokes:
                    for v in stroke:
                        self.model.ik((*v.xy, z))
            if not self.bk_active and self.bk_overlay_settings != settings:
                self.bk_trails = [[] for _ in strokes]
            self.bk_overlay = strokes
            self.bk_overlay_settings = settings
            self.bk_overlay_key = None
            self.bk_canvas_key = None
            width, height = pattern_dimensions(settings)
            self.bk_note.set(
                f"{PATTERNS[settings.pattern][0]} {width:g}×{height:.2f}mm; tâm({settings.center_x:g}, {settings.center_y:g}); {len(strokes)}nét.\n"
                + (
                    "Z giấy đã xác nhận; chạy khô trước để kiểm tra."
                    if self.bk_confirmed()
                    else "Cần HOME và xác nhận Z bút chạm giấy trước khi chạy."
                )
            )
            self.trace_event("bk_preview", settings=asdict(settings))
            self.draw()
            self.draw_bk_window()
        except ValueError as e:
            self.bk_note.set(str(e))
            self.message.set(str(e))
            if not self.bk_active:
                self.bk_overlay = ()
                self.bk_overlay_settings = None
                self.bk_overlay_key = None
                if self.bk_window and self.bk_window.winfo_exists():
                    self.bk_canvas.delete("all")
                    self.bk_canvas.create_text(
                        self.bk_canvas.winfo_width() / 2,
                        self.bk_canvas.winfo_height() / 2,
                        text="Mẫu chưa hợp lệ\n" + str(e),
                        fill="#ff8795",
                        width=350,
                    )
                self.draw()

    def start_bk(self, dry=False):
        if (
            not self.operable()
            or not self.editor_ready()
            or self.program_points
            or self.follow_enabled.get()
        ):
            return
        if not self.bk_confirmed():
            self.bk_note.set("Cần xác nhận Z mặt giấy trong lượt home hiện tại.")
            return
        try:
            settings = self.bk_settings().validate(paper=True)
        except ValueError as e:
            self.bk_note.set(str(e))
            return
        self.clear_automation()
        self.bk_pending = True
        self.submit_planning(
            lambda model, start, cancelled: compile_bk(
                model, start, settings, dry, cancelled
            ),
            purpose="drawing_check",
            name="BK path validation",
        )
        self.bk_note.set(
            "Đang kiểm tra toàn bộ nét, chuyển bút, xung và sai lệch mô hình…"
        )

    def accept_bk(self, program):
        if not self.bk_confirmed() or program.settings != self.bk_settings():
            self.bk_note.set("Thông số hoặc Z giấy đã đổi; hãy kiểm tra và chạy lại.")
            return
        if not self._foreground() and not self.background_allowed():
            self.stop("FOCUS")
            return
        self.bk_plan = program
        self.bk_active_plan = program
        self.bk_active = True
        self.bk_trails = [[] for _ in program.strokes]
        self.bk_overlay = program.strokes
        self.bk_overlay_settings = program.settings
        self.bk_overlay_key = None
        self.bk_canvas_key = None
        self.program_points = program.points
        self.program_index = 0
        self.program_wait = 0
        self.bk_started = time.monotonic()
        self.bk_note.set(
            f"{'Chạy khô' if program.dry else 'Vẽ'} {PATTERNS[program.settings.pattern][0]}: {len(program.runs)} nét liên tục · thời gian trên nét≈{sum(r.seconds for r in program.runs):.1f}s.\nSai lệch mô hình tối đa {program.max_model_error_mm:.3f}mm; thời gian chưa gồm nâng/chuyển bút."
        )
        self.trace_event(
            "drawing_start",
            dry=program.dry,
            settings=asdict(program.settings),
            operations=len(program.points),
            max_model_error_mm=program.max_model_error_mm,
        )

    def record_bk(self, status, complete=False):
        if (
            not self.bk_active
            or self.bk_active_plan.dry
            or not status.referenced
            or not self.model
        ):
            return
        if not complete and (
            status.job != self.active_job
            or self.active_purpose not in ("program", "stroke")
        ):
            return
        operation = self.bk_active_plan.operations[self.program_index]
        if operation.kind != "ink" and not (complete and operation.kind == "lower"):
            return
        xyz = self.model.fk(self.model.from_steps(status.pos))
        if abs(xyz[2] - self.bk_active_plan.settings.paper_z) > max(
            0.01, 1.5 / self.model.factors[0]
        ):
            return
        xy = xyz[:2]
        trail = self.bk_trails[operation.stroke]
        if not trail or math.dist(xy, trail[-1]) > 0.0001:
            trail.append(xy)

    def render_drawing(self):
        if not self.editor_ready():
            self.bk_confirmed_epoch = None
        if self.bk_window and self.bk_window.winfo_exists():
            editable = not self._active_task()
            pattern = self.bk_vars["pattern"].get()
            name = PATTERNS.get(pattern, (pattern,))[0]
            confirmed = self.bk_confirmed()
            ready = self.editor_ready()

            def update(var, value):
                if var.get() != value:
                    var.set(value)

            update(self.bk_choice, name)
            update(self.bk_title, name)
            update(
                self.bk_status,
                "● Z GIẤY ĐÃ XÁC NHẬN" if confirmed else "○ CHƯA XÁC NHẬN Z GIẤY",
            )
            try:
                width, height = pattern_dimensions(self.bk_settings())
                update(self.bk_dimensions, f"{width:g} × {height:.2f} mm")
            except ValueError:
                update(self.bk_dimensions, "—")
            key = (editable, pattern, confirmed, ready)
            if key != self.bk_control_key:
                self.bk_control_key = key
                for code, (tile, button) in self.bk_tiles.items():
                    tile.configure(
                        highlightbackground="#61d9c4" if code == pattern else "#263952"
                    )
                    button.configure(state="normal" if editable else "disabled")
                for field in self.bk_fields:
                    field.configure(
                        state=(
                            (
                                "readonly"
                                if isinstance(field, ttk.Combobox)
                                else "normal"
                            )
                            if editable
                            else "disabled"
                        )
                    )
                for i, button in enumerate(self.bk_buttons):
                    enabled = editable and (i == 2 or ready) and (i < 3 or confirmed)
                    button.configure(state="normal" if enabled else "disabled")
                self.bk_fit_button.configure(
                    state="normal" if editable and ready else "disabled"
                )
            self.draw_bk_window()

    def draw_bk_window(self):
        if (
            not self.bk_window
            or not self.bk_window.winfo_exists()
            or not self.bk_overlay_settings
        ):
            return
        canvas = self.bk_canvas
        s = self.bk_overlay_settings
        w = max(canvas.winfo_width(), 100)
        h = max(canvas.winfo_height(), 100)
        width, height = pattern_dimensions(s)
        factor = min((w - 80) / (width * 1.15), (h - 80) / (height * 1.15))

        def point(x, y):
            return (
                w / 2 + (x - s.center_x) * factor,
                h / 2 - (y - s.center_y) * factor,
            )

        key = (w, h, s, self.bk_overlay)
        if key != self.bk_canvas_key:
            canvas.delete("all")
            self.bk_canvas_key = key
            half_w = width / 2
            half_h = height / 2
            for v in (-half_w, 0, half_w):
                x, y = point(s.center_x + v, s.center_y - half_h)
                canvas.create_text(x, y + 15, text=f"{s.center_x+v:g}", fill="#9cb0cb")
            for v in (-half_h, 0, half_h):
                x, y = point(s.center_x - half_w, s.center_y + v)
                canvas.create_text(
                    x - 8, y, text=f"{s.center_y+v:.2f}", anchor="e", fill="#9cb0cb"
                )
            canvas.create_rectangle(
                *point(s.center_x - half_w, s.center_y + half_h),
                *point(s.center_x + half_w, s.center_y - half_h),
                outline="#35506b",
                dash=(3, 4),
            )
            # A local metric grid makes scale and placement clear on the preview.
            step = next(
                (v for v in (1, 2, 5, 10, 20, 25, 50) if v >= max(width, height) / 6),
                50,
            )
            for x in range(
                math.ceil((s.center_x - half_w) / step),
                math.floor((s.center_x + half_w) / step) + 1,
            ):
                canvas.create_line(
                    *point(x * step, s.center_y - half_h),
                    *point(x * step, s.center_y + half_h),
                    fill="#203249",
                )
            for y in range(
                math.ceil((s.center_y - half_h) / step),
                math.floor((s.center_y + half_h) / step) + 1,
            ):
                canvas.create_line(
                    *point(s.center_x - half_w, y * step),
                    *point(s.center_x + half_w, y * step),
                    fill="#203249",
                )
            for i, stroke in enumerate(self.bk_overlay, 1):
                coords = [v for vertex in stroke for v in point(*vertex.xy)]
                canvas.create_line(*coords, fill="#efc25a", width=3)
                x, y = point(*stroke[0].xy)
                canvas.create_oval(
                    x - 4, y - 4, x + 4, y + 4, fill="#60d5ae", outline=""
                )
                canvas.create_text(x + 7, y - 10, text=str(i), fill="#60d5ae")
        canvas.delete("ink")
        for trail in self.bk_trails:
            if len(trail) > 1:
                canvas.create_line(
                    *(v for xy in trail for v in point(*xy)),
                    fill="#ff8795",
                    width=2,
                    tags="ink",
                )

    def draw_bk_overlay(self, canvas, point):
        if not self.bk_overlay:
            canvas.delete("bk")
            self.bk_overlay_key = None
            return
        key = (self.grid_key, self.bk_overlay)
        if key != self.bk_overlay_key or not canvas.find_withtag("bk"):
            canvas.delete("bk")
            self.bk_overlay_key = key
            for stroke in self.bk_overlay:
                canvas.create_line(
                    *(v for vertex in stroke for v in point(*vertex.xy)),
                    fill="#efc25a",
                    width=2,
                    tags="bk",
                )
