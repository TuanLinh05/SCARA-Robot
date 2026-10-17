"""
================================================================================
FILE: cartesian_nc_drawing_gui.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Interactive Drawing Studio controller (`DrawingUI` mixin for `NCApp`):
     - Enforces explicit per-epoch paper-Z confirmation (`bk_confirmed`,
       `capture_paper`, `confirm_paper`) so the pen never plunges into the table
       without operator verification after homing.
     - Handles pattern selection, external SVG/DXF/G-code file loading,
       drag-to-reposition on the preview canvas, background size fitting (`fit_bk`),
       dry-run (`start_bk(True)`), and real ink execution (`start_bk(False)`).
     - Incrementally renders live pink ink trails (`record_bk`, `_bk_trail_drawn`)
       on the preview canvas as telemetry arrives.

     INCREMENTAL INK RENDERING EXPLAINED (For 1st/2nd-Year Students):
     When the robot is drawing a complex pattern, its pen tip may pass through
     thousands of `(X, Y)` points. If we deleted and redrew all thousand lines
     on every 40 ms telemetry packet, the GUI would lag. Instead, we store a
     counter `self._bk_trail_drawn[stroke_idx]` that remembers how many points
     of each stroke have already been drawn on the canvas, and only draw the
     newly arrived line segment!

[VI] MỤC ĐÍCH:
     Bộ điều khiển tương tác cho cửa sổ Drawing Studio (`DrawingUI` mixin cho `NCApp`):
     - Bắt buộc xác nhận cao độ mặt giấy Z theo từng lượt lấy gốc (`bk_confirmed`,
       `capture_paper`, `confirm_paper`) để tránh trường hợp bút lao xuống mặt bàn
       khi chưa kiểm tra độ cao.
     - Xử lý chọn mẫu hình/chữ, nạp file vector SVG/DXF/G-code ngoài, kéo chuột
       đổi tâm bản vẽ, tự động phóng lớn vừa vùng làm việc (`fit_bk`), chạy khô
       (`start_bk(True)`) và vẽ thật (`start_bk(False)`).
     - Vẽ bồi dần vết mực màu hồng theo thời gian thực (`record_bk`, `_bk_trail_drawn`)
       lên khung xem trước khi nhận dữ liệu telemetry từ STM32.

     GIẢI THÍCH KỸ THUẬT VẼ BỒI VẾT MỰC (Dành cho Sinh viên năm 1–2):
     Khi robot vẽ một hình phức tạp, đầu bút đi qua hàng nghìn điểm `(X, Y)`. Nếu
     cứ mỗi 40 ms lại xóa toàn bộ Canvas và vẽ lại từ đầu hàng nghìn đoạn thẳng,
     giao diện sẽ bị chậm. Thay vào đó, mảng `self._bk_trail_drawn[idx]` ghi nhớ
     số điểm đã vẽ của từng nét và chỉ vẽ nối thêm đúng đoạn thẳng mới nhất!

[DEBUG TIP / MẸO GỠ LỖI]:
     - If the "Chạy khô" (Dry Run) and "BẮT ĐẦU VẼ" (Start Drawing) buttons are
       disabled (grayed out), check `self.bk_confirmed()`: both `self.editor_ready()`
       (HOMEd) AND `self.bk_confirmed_epoch == self.state.epoch` must be True!
     - Nếu hai nút "Chạy khô" và "BẮT ĐẦU VẼ" bị mờ (khóa), hãy kiểm tra hàm
       `self.bk_confirmed()`: robot phải vừa hoàn tất `HOME + CALIB` (`self.editor_ready()`)
       VÀ người dùng phải bấm xác nhận Z mặt giấy trong đúng lượt Home hiện tại!
================================================================================
"""

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
    CUSTOM_PATTERNS,
    load_vector_strokes,
    register_custom_pattern,
)
from cartesian_nc_font import custom_text_strokes, parametric_generator_strokes
from cartesian_nc_shapes import PATTERNS
from pathlib import Path


class DrawingUI:
    """
    [EN] Mixin class managing the Drawing Studio window state, paper-Z safety
         confirmation, interactive preview canvas, custom text/parametric generators,
         and live ink trail rendering.
    [VI] Lớp Mixin quản lý trạng thái cửa sổ Drawing Studio, xác nhận an toàn
         cao độ mặt giấy Z, khung vẽ xem trước tương tác, bộ tạo chữ/mẫu tham số
         tùy ý và hiển thị vết bút trực tiếp.
    """

    # ==========================================================================
    # 1. INITIALIZATION & SETTINGS PARSING / KHỞI TẠO & ĐỌC THÔNG SỐ BẢN VẼ
    # ==========================================================================

    def init_drawing(self):
        """
        [EN] Initialize observable Tkinter variables and state for Drawing Studio.
        [VI] Khởi tạo các biến quan sát Tkinter và trạng thái cho phân hệ Drawing Studio.
        """
        self.paper_tilt = (0.0, 0.0)

        # Create a `tk.StringVar` for each field in `BKSettings` (except `paper_tilt`).
        # Tạo biến chuỗi `tk.StringVar` cho từng trường trong `BKSettings` (trừ `paper_tilt`).
        default_settings_dict = asdict(BKSettings())
        self.bk_vars = {}
        for name, value in default_settings_dict.items():
            if name != "paper_tilt":
                initial_text = "" if value is None else str(value)
                self.bk_vars[name] = tk.StringVar(value=initial_text)

        self.bk_window = None
        self.bk_fields = []
        self.bk_buttons = []
        self.bk_custom_widgets = []
        self.custom_text_var = tk.StringVar(value="BACH KHOA")
        self.custom_rot_var = tk.StringVar(value="0")
        self.custom_param_var = tk.StringVar(value="6")
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
        self._bk_trail_drawn = [0, 0, 0]
        self._bk_drag_origin = None
        self.bk_canvas_key = None
        self.bk_selected_pattern = "BK"
        self.bk_control_key = None

        # Attach a write-observer so changing any entry invalidates the cached plan.
        # Gắn hàm theo dõi để khi người dùng sửa bất kỳ ô nào thì xóa bản kế hoạch cũ.
        for variable in self.bk_vars.values():
            variable.trace_add("write", self.bk_changed)

    def bk_changed(self, *args):
        """
        [EN] Invalidate cached drawing plan and canvas key when any setting changes.
        [VI] Xóa kế hoạch vẽ và khóa bộ đệm canvas khi người dùng sửa bất kỳ thông số nào.
        """
        self.bk_plan = None
        self.bk_canvas_key = None

    def bk_settings(self):
        """
        [EN] Parse the current UI entry variables into a validated `BKSettings` object.
        [VI] Đọc các ô nhập liệu trên giao diện và tạo đối tượng `BKSettings` đã kiểm tra.
        """
        try:
            data = {}
            for name, variable in self.bk_vars.items():
                raw_text = variable.get().strip()
                if name == "pattern":
                    data[name] = raw_text.upper()
                elif name == "paper_z" and not raw_text:
                    # `paper_z` is allowed to be empty (`None`) during preview before confirmation.
                    # `paper_z` có thể để trống (`None`) khi chỉ mới xem trước, chưa xác nhận giấy.
                    data[name] = None
                else:
                    data[name] = float(raw_text)
        except ValueError:
            raise ValueError("Thông số kích thước/Z cần là số.")

        data["paper_tilt"] = getattr(self, "paper_tilt", (0.0, 0.0))
        return BKSettings(**data).validate()

    def bk_confirmed(self):
        """
        [EN] Return True if the robot is referenced and `paper_z` has been
             explicitly confirmed for the current calibration `epoch`.
        [VI] Trả về True nếu robot đã lấy mốc và cao độ `paper_z` đã được xác nhận
             cho đúng phiên hiệu chuẩn (`epoch`) hiện tại.
        """
        try:
            settings = self.bk_settings()
        except ValueError:
            return False

        is_ready = self.editor_ready()
        is_same_epoch = is_ready and (self.bk_confirmed_epoch == self.state.epoch)
        is_same_z = (
            (settings.paper_z == self.bk_confirmed_z)
            and (settings.paper_z is not None)
        )
        return is_ready and is_same_epoch and is_same_z

    # ==========================================================================
    # 2. WINDOW LIFECYCLE & CANVAS DRAGGING / MỞ CỬA SỔ & KÉO TÂM BẢN VẼ
    # ==========================================================================

    def open_bk(self):
        """
        [EN] Open or focus the Drawing Studio window and render the initial preview.
        [VI] Mở hoặc đưa cửa sổ Drawing Studio lên trên cùng và vẽ hình xem trước.
        """
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
        if hasattr(self, "bk_canvas"):
            self.bk_canvas.bind("<ButtonPress-1>", self.bk_canvas_press)
            self.bk_canvas.bind("<B1-Motion>", self.bk_canvas_drag)
            self.bk_canvas.bind("<ButtonRelease-1>", self.bk_canvas_release)
        self.bk_control_key = None
        self.preview_bk()
        self.render_drawing()

    def bk_canvas_press(self, event):
        """
        [EN] Record starting mouse coordinates and pattern center for drag-to-move.
        [VI] Lưu tọa độ chuột ban đầu và tâm mẫu vẽ khi bắt đầu nhấn giữ để kéo tâm.
        """
        if self._active_task() or not self.bk_overlay_settings:
            self._bk_drag_origin = None
            return
        s = self.bk_overlay_settings
        w = max(self.bk_canvas.winfo_width(), 100)
        h = max(self.bk_canvas.winfo_height(), 100)
        width, height = pattern_dimensions(s)

        # Compute pixel-per-mm scale factor on the preview canvas.
        # Tính hệ số tỷ lệ số điểm ảnh trên mỗi mm (pixel/mm) trên khung xem trước.
        scale_x = (w - 80) / (width * 1.15)
        scale_y = (h - 80) / (height * 1.15)
        factor = max(min(scale_x, scale_y), 0.1)
        self._bk_drag_origin = (event.x, event.y, s.center_x, s.center_y, factor)

    def bk_canvas_drag(self, event):
        """
        [EN] Update `center_x` and `center_y` as the user drags on the preview canvas.
        [VI] Cập nhật tâm `center_x` và `center_y` khi người dùng kéo chuột trên khung xem trước.
        """
        if not self._bk_drag_origin or self._active_task():
            return
        ox, oy, cx, cy, factor = self._bk_drag_origin

        # Convert mouse pixel delta `(dx_px, dy_px)` to millimeter shift `(dx_mm, dy_mm)`.
        # Đổi độ lệch kéo chuột từ pixel sang milimét (trục Y ngược dấu vì Y màn hình hướng xuống).
        dx_mm = (event.x - ox) / factor
        dy_mm = -(event.y - oy) / factor
        nx = round(cx + dx_mm, 1)
        ny = round(cy + dy_mm, 1)
        self.bk_vars["center_x"].set(f"{nx:g}")
        self.bk_vars["center_y"].set(f"{ny:g}")

    def bk_canvas_release(self, _event):
        """
        [EN] Re-validate and redraw the pattern preview when mouse drag ends.
        [VI] Kiểm tra lại và vẽ lại hình xem trước khi người dùng nhả chuột.
        """
        if not self._bk_drag_origin:
            return
        ox, oy, *_ = self._bk_drag_origin
        self._bk_drag_origin = None
        if abs(_event.x - ox) > 1 or abs(_event.y - oy) > 1:
            self.preview_bk()

    # ==========================================================================
    # 3. PATTERN SELECTION & VECTOR IMPORT / CHỌN MẪU & NHẬP FILE VECTOR NGOÀI
    # ==========================================================================

    def _pattern_title(self, code):
        """
        [EN] Return the display title for a built-in or custom pattern `code`.
        [VI] Trả về tên hiển thị của mẫu có sẵn hoặc mẫu tùy chỉnh theo mã `code`.
        """
        if code in PATTERNS:
            return PATTERNS[code][0]
        if code in CUSTOM_PATTERNS:
            return CUSTOM_PATTERNS[code][0]
        return code

    def load_vector_file(self, path):
        """
        [EN] Parse an external `.svg`, `.dxf`, or `.gcode` file, register it as
             `"CUSTOM"`, and preview it immediately.
        [VI] Đọc file `.svg`, `.dxf`, hoặc `.gcode` bên ngoài, đăng ký vào mã `"CUSTOM"`
             và hiển thị xem trước ngay lập tức.
        """
        if self._active_task():
            return
        strokes = load_vector_strokes(path)
        title = Path(path).stem[:18] or "Vector"
        register_custom_pattern("CUSTOM", title, strokes, default_width=30.0)
        self.bk_selected_pattern = "CUSTOM"
        self.bk_vars["pattern"].set("CUSTOM")
        self.preview_bk()
        self.render_drawing()

    def open_vector_file(self):
        """
        [EN] Open a file dialog to choose an SVG, DXF, or G-code file for import.
        [VI] Mở hộp thoại chọn file SVG, DXF hoặc G-code để nạp vào chương trình.
        """
        if self._active_task():
            return
        from tkinter import filedialog

        path = filedialog.askopenfilename(
            parent=self.bk_window or self,
            title="Chọn file Vector (SVG / DXF / G-code)",
            filetypes=[
                ("Vector & G-code", "*.svg *.dxf *.gcode *.nc *.ngc *.tap"),
                ("SVG", "*.svg"),
                ("DXF", "*.dxf"),
                ("G-code", "*.gcode *.nc *.ngc *.tap"),
            ],
        )
        if path:
            try:
                self.load_vector_file(path)
            except Exception as e:
                self.bk_note.set(str(e))

    def build_custom_drawing_tab(self, parent):
        """
        [EN] Build the `"Chữ & Mẫu tùy ý"` tab inside Drawing Studio's right panel,
             allowing operators to generate arbitrary A-Z/0-9 text strings, rotated
             motifs, spirals, calibration grids, and N-petal flowers.
        [VI] Dựng giao diện tab `"Chữ & Mẫu tùy ý"` trong cửa sổ Drawing Studio,
             cho phép tạo văn bản bất kỳ (A-Z, 0-9), góc xoay, đường xoắn ốc,
             lưới hiệu chuẩn và hoa N cánh.
        """
        self.bk_custom_widgets = []
        bg = parent.cget("bg")

        # Row 1: Custom Text input (A-Z, 0-9) and "Tạo" button.
        # Hàng 1: Ô nhập chuỗi chữ tùy ý (A-Z, 0-9) và nút "Tạo".
        r1 = tk.Frame(parent, bg=bg)
        r1.pack(fill="x", pady=2)
        self.label(r1, "Chữ (A-Z,0-9):", 9, "#9fb1ca").pack(side="left")
        txt_entry = self.entry(r1, self.custom_text_var)
        txt_entry.configure(width=11, font=("Segoe UI", 9))
        txt_entry.pack(side="right")
        b_txt = self.button(r1, "Tạo", self.apply_custom_text, "#24597a")
        b_txt.configure(font=("Segoe UI", 8, "bold"), padx=6, pady=1)
        b_txt.pack(side="right", padx=3)

        # Row 2: Rotation angle (deg) and order N parameter entries.
        # Hàng 2: Ô nhập góc xoay (độ) và tham số bậc N (số vòng / số cánh / số cạnh).
        r2 = tk.Frame(parent, bg=bg)
        r2.pack(fill="x", pady=2)
        self.label(r2, "Xoay (°) / Bậc N:", 9, "#9fb1ca").pack(side="left")
        n_entry = self.entry(r2, self.custom_param_var)
        n_entry.configure(width=4, font=("Segoe UI", 9))
        n_entry.pack(side="right")
        rot_entry = self.entry(r2, self.custom_rot_var)
        rot_entry.configure(width=5, font=("Segoe UI", 9))
        rot_entry.pack(side="right", padx=3)

        # Row 3: Quick Parametric Pattern Generator buttons.
        # Hàng 3: Các nút tạo nhanh mẫu hình học tham số.
        r3 = tk.Frame(parent, bg=bg)
        r3.pack(fill="x", pady=(4, 2))
        for label, kind in (
            ("Xoắn ốc", "SPIRAL"),
            ("Lưới chuẩn", "GRID"),
            ("Hoa N cánh", "ROSE"),
            ("Đa giác N", "POLYGON"),
        ):
            b = self.button(
                r3,
                label,
                lambda k=kind: self.apply_parametric_pattern(k),
                "#213f5f",
            )
            b.configure(font=("Segoe UI", 8), padx=4, pady=2)
            b.pack(side="left", expand=True, fill="x", padx=1)
            self.bk_custom_widgets.append(b)
        self.bk_custom_widgets.extend((txt_entry, b_txt, rot_entry, n_entry))

    def apply_custom_text(self):
        """
        [EN] Compile the user-typed string in `self.custom_text_var` with rotation
             `self.custom_rot_var` and register it as `"CUSTOM"`.
        [VI] Biên dịch chuỗi chữ tùy ý trong `self.custom_text_var` kèm góc xoay
             `self.custom_rot_var` và đăng ký vào mẫu `"CUSTOM"`.
        """
        if self._active_task():
            return
        try:
            width = float(self.bk_vars["size_mm"].get() or 40.0)
            rot = float(self.custom_rot_var.get() or 0.0)
            text = self.custom_text_var.get().strip().upper()
            strokes = custom_text_strokes(text, width_mm=width, rotation_deg=rot)
            register_custom_pattern(
                "CUSTOM", f"Chữ: {text[:12]}", strokes, default_width=width
            )
            self.bk_selected_pattern = "CUSTOM"
            self.bk_vars["pattern"].set("CUSTOM")
            self.preview_bk()
            self.render_drawing()
        except ValueError as e:
            self.bk_note.set(str(e))

    def apply_parametric_pattern(self, kind):
        """
        [EN] Generate a custom parametric test pattern (`SPIRAL`, `GRID`, `ROSE`, `POLYGON`)
             and preview it immediately in Drawing Studio.
        [VI] Sinh mẫu hình học tham số tùy biến (`SPIRAL`, `GRID`, `ROSE`, `POLYGON`)
             và hiển thị xem trước ngay trong cửa sổ Drawing Studio.
        """
        if self._active_task():
            return
        try:
            width = float(self.bk_vars["size_mm"].get() or 30.0)
            rot = float(self.custom_rot_var.get() or 0.0)
            n_val = int(float(self.custom_param_var.get() or 6))
            strokes = parametric_generator_strokes(
                kind, param=n_val, width_mm=width, rotation_deg=rot
            )
            titles = {
                "SPIRAL": f"Xoắn ốc {n_val} vòng",
                "GRID": f"Lưới chuẩn {n_val} bậc",
                "ROSE": f"Hoa {n_val} cánh",
                "POLYGON": f"Đa giác {n_val} cạnh",
            }
            register_custom_pattern(
                "CUSTOM", titles.get(kind, kind), strokes, default_width=width
            )
            self.bk_selected_pattern = "CUSTOM"
            self.bk_vars["pattern"].set("CUSTOM")
            self.preview_bk()
            self.render_drawing()
        except ValueError as e:
            self.bk_note.set(str(e))

    def select_drawing_pattern(self):
        """
        [EN] Apply preset default width when switching between built-in patterns
             and refresh the preview canvas.
        [VI] Áp dụng chiều rộng mặc định khi chuyển đổi giữa các mẫu có sẵn và
             cập nhật lại lại khung xem trước.
        """
        # These are preset widths, not an automatic motion or a paper-Z change.
        # Đây chỉ là kích thước gợi ý mặc định, không tự chạy máy hay đổi Z giấy.
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
            tab_idx = 0 if PATTERNS[choice][1] == "Hình vẽ" else 1
            self.bk_gallery_tabs.select(tab_idx)
        self.preview_bk()

    def choose_pattern(self, pattern):
        """
        [EN] Select `pattern` by code when clicking a gallery tile or combobox item.
        [VI] Chọn mẫu `pattern` theo mã khi bấm vào ô thư viện hoặc danh sách thả xuống.
        """
        is_known_pattern = (pattern in PATTERNS) or (pattern in CUSTOM_PATTERNS)
        if self._active_task() or not is_known_pattern:
            return
        self.bk_vars["pattern"].set(pattern)
        self.select_drawing_pattern()

    # ==========================================================================
    # 4. PAPER-Z CONFIRMATION, AUTO-FIT & EXECUTION / XÁC NHẬN Z, CĂN CỠ & VẼ
    # ==========================================================================

    def capture_paper(self):
        """
        [EN] Copy the robot's current calibrated Z coordinate into `paper_z` and
             confirm it for the current homing epoch.
        [VI] Lấy độ cao Z hiện tại của robot gán vào ô `paper_z` và xác nhận làm
             mặt giấy cho lượt lấy gốc hiện tại.
        """
        if (
            not self.operable()
            or not self.editor_ready()
            or self.program_points
            or self.follow_enabled.get()
        ):
            return
        current_joint_pose = self.model.from_steps(self.state.pos)
        z = current_joint_pose[2]
        self.bk_vars["paper_z"].set(f"{z:.6f}")
        self.confirm_paper()

    def fit_bk(self):
        """
        [EN] Launch background binary search (`fit_drawing`) to find the largest
             safe pattern width at the chosen XY center.
        [VI] Khởi chạy tìm kiếm nhị phân ở luồng ngầm (`fit_drawing`) để tìm kích thước
             mẫu lớn nhất vừa khít vùng làm việc tại tâm XY đã chọn.
        """
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
        """
        [EN] Accept the fitted `settings` on the main thread and update `size_mm`.
        [VI] Nhận kết quả căn cỡ `settings` trên luồng chính và cập nhật ô `size_mm`.
        """
        self.bk_vars["size_mm"].set(f"{settings.size_mm:.2f}")
        self.preview_bk()
        width, height = pattern_dimensions(settings)
        self.bk_note.set(
            f"Cỡ đã kiểm tra: {width:g}×{height:.2f}mm, tâm({settings.center_x:g},{settings.center_y:g}).\n"
            "Có chừa 4% theo kích thước; chạy khô trước khi vẽ."
        )
        self.trace_event("drawing_fit", settings=asdict(settings))

    def confirm_paper(self):
        """
        [EN] Validate that `paper_z` and `paper_z + lift_mm` lie within calibrated
             Z travel and lock in confirmation for `self.state.epoch`.
        [VI] Kiểm tra `paper_z` và `paper_z + lift_mm` nằm trong hành trình trục Z
             đã hiệu chuẩn, sau đó chốt xác nhận cho phiên `self.state.epoch`.
        """
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
            lifted_z = settings.paper_z + settings.lift_mm
            if not (lo <= settings.paper_z < lifted_z <= hi):
                raise ValueError(
                    "Z giấy hoặc Z nâng nằm ngoài hành trình. Chỉnh Z/độ nhấc bút."
                )
            self.bk_confirmed_epoch = self.state.epoch
            self.bk_confirmed_z = settings.paper_z
            self.bk_note.set(
                f"Đã xác nhận Z giấy={settings.paper_z:.3f}; Z nâng={lifted_z:.3f}mm."
            )
            self.trace_event(
                "paper_confirmed", epoch=self.state.epoch, settings=asdict(settings)
            )
        except ValueError as e:
            self.bk_note.set(str(e))

    def preview_bk(self):
        """
        [EN] Generate pattern strokes, check IK reachability if calibrated, and
             update both the Drawing Studio preview canvas and workspace overlay.
        [VI] Sinh các nét vẽ của mẫu, kiểm tra động học ngược nếu đã lấy mốc, đồng thời
             cập nhật hình xem trước trên cả cửa sổ Drawing Studio và đồ thị chính.
        """
        try:
            settings = self.bk_settings()
            strokes = pattern_strokes(settings)

            # If the robot is calibrated, verify that every vertex is reachable via IK!
            # Nếu robot đã lấy mốc, kiểm tra động học ngược (IK) cho mọi đỉnh của nét vẽ!
            if self.editor_ready():
                z = self.model.from_steps(self.state.pos)[2]
                for stroke in strokes:
                    for v in stroke:
                        vx, vy = v.xy
                        self.model.ik((vx, vy, z))

            # Reset live ink trail buffers if not currently executing a drawing job.
            # Xóa bộ đệm vết mực thực tế nếu hiện không trong quá trình chạy vẽ.
            if not self.bk_active and self.bk_overlay_settings != settings:
                self.bk_trails = [[] for _ in strokes]
                self._bk_trail_drawn = [0 for _ in strokes]

            self.bk_overlay = strokes
            self.bk_overlay_settings = settings
            self.bk_overlay_key = None
            self.bk_canvas_key = None
            width, height = pattern_dimensions(settings)
            if self.bk_confirmed():
                confirm_hint = "Z giấy đã xác nhận; chạy khô trước để kiểm tra."
            else:
                confirm_hint = "Cần HOME và xác nhận Z bút chạm giấy trước khi chạy."

            self.bk_note.set(
                f"{self._pattern_title(settings.pattern)} {width:g}×{height:.2f}mm; "
                f"tâm({settings.center_x:g}, {settings.center_y:g}); {len(strokes)}nét.\n"
                + confirm_hint
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
        """
        [EN] Submit full DDA preflight compilation (`compile_bk`) on a background
             thread for either a dry run (`dry=True`, pen stays lifted) or ink run.
        [VI] Gửi tác vụ biên dịch và kiểm duyệt toàn bộ xung DDA (`compile_bk`) sang
             luồng ngầm cho chế độ chạy khô (`dry=True`, luôn nhấc bút) hoặc vẽ thật.
        """
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
        """
        [EN] Accept a validated `DrawingProgram` on the main thread and start
             automated execution via `self.program_points`.
        [VI] Nghiệm thu chương trình vẽ `DrawingProgram` đã kiểm duyệt trên luồng chính
             và bắt đầu chuỗi chạy tự động thông qua `self.program_points`.
        """
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
        self._bk_trail_drawn = [0 for _ in program.strokes]
        self.bk_overlay = program.strokes
        self.bk_overlay_settings = program.settings
        self.bk_overlay_key = None
        self.bk_canvas_key = None
        self.program_points = program.points
        self.program_index = 0
        self.program_wait = 0
        self.bk_started = time.monotonic()

        mode_str = "Chạy khô" if program.dry else "Vẽ"
        title_str = self._pattern_title(program.settings.pattern)
        total_ink_seconds = sum(r.seconds for r in program.runs)
        self.bk_note.set(
            f"{mode_str} {title_str}: {len(program.runs)} nét liên tục · "
            f"thời gian trên nét≈{total_ink_seconds:.1f}s.\n"
            f"Sai lệch mô hình tối đa {program.max_model_error_mm:.3f}mm; "
            "thời gian chưa gồm nâng/chuyển bút."
        )
        self.trace_event(
            "drawing_start",
            dry=program.dry,
            settings=asdict(program.settings),
            operations=len(program.points),
            max_model_error_mm=program.max_model_error_mm,
        )

    # ==========================================================================
    # 5. LIVE INK TRAIL RECORDING & CANVAS RENDERING / GHI VẾT MỰC & VẼ CANVAS
    # ==========================================================================

    def record_bk(self, status, complete=False):
        """
        [EN] Record live XY pen-down coordinates from incoming `NCStatus` frames
             into `self.bk_trails` when the pen is at `paper_z`.
        [VI] Ghi lại tọa độ XY thực tế khi hạ bút từ các gói `NCStatus` gửi về
             vào `self.bk_trails` khi đầu bút đang chạm mặt giấy `paper_z`.
        """
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

        # Compute live Forward Kinematics `(x, y, z)` from stepper pulse counts `status.pos`.
        # Tính động học thuận `(x, y, z)` từ bộ đếm xung bước hiện tại `status.pos`.
        joint_pose = self.model.from_steps(status.pos)
        xyz = self.model.fk(joint_pose)

        # Verify that the pen Z height is within 1.5 Z-steps of `paper_z`.
        # Kiểm tra độ cao Z thực tế đang chạm mặt giấy (sai lệch trong vòng 1.5 xung Z).
        z_tolerance_mm = max(0.01, 1.5 / self.model.factors[0])
        if abs(xyz[2] - self.bk_active_plan.settings.paper_z) > z_tolerance_mm:
            return

        xy = xyz[:2]
        trail = self.bk_trails[operation.stroke]
        if not trail or math.dist(xy, trail[-1]) > 0.0001:
            trail.append(xy)

    def render_drawing(self):
        """
        [EN] Update Drawing Studio widget states, tile highlights, and preview canvas.
        [VI] Cập nhật trạng thái các nút bấm, viền sáng ô mẫu và khung vẽ của Drawing Studio.
        """
        if not self.editor_ready():
            self.bk_confirmed_epoch = None
        if self.bk_window and self.bk_window.winfo_exists():
            editable = not self._active_task()
            pattern = self.bk_vars["pattern"].get()
            name = self._pattern_title(pattern)
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

                # Highlight the selected pattern tile in cyan-mint (`#61d9c4`).
                # Tô sáng viền ô mẫu đang chọn bằng màu xanh ngọc (`#61d9c4`).
                for code, (tile, button) in self.bk_tiles.items():
                    border_col = "#61d9c4" if code == pattern else "#263952"
                    tile.configure(highlightbackground=border_col)
                    button.configure(state="normal" if editable else "disabled")

                # Enable/disable input fields (Combobox uses `"readonly"` when enabled).
                # Bật/tắt các ô nhập thông số (Combobox dùng `"readonly"` khi bật).
                for field in self.bk_fields:
                    if editable:
                        field_state = (
                            "readonly" if isinstance(field, ttk.Combobox) else "normal"
                        )
                    else:
                        field_state = "disabled"
                    field.configure(state=field_state)

                # Enable/disable action buttons in `self.bk_buttons`:
                # Index 0 (`capture_paper`) & 1 (`confirm_paper`): require `editable` and `ready`
                # Index 2 (`preview_bk`): only requires `editable`
                # Index 3 (`dry`) & 4 (`run`): require `editable`, `ready`, and `confirmed`!
                for i, button in enumerate(self.bk_buttons):
                    can_use_without_homing = (i == 2)
                    does_not_need_z_confirm = (i < 3)
                    enabled = (
                        editable
                        and (can_use_without_homing or ready)
                        and (does_not_need_z_confirm or confirmed)
                    )
                    button.configure(state="normal" if enabled else "disabled")

                self.bk_fit_button.configure(
                    state="normal" if (editable and ready) else "disabled"
                )
                for w in getattr(self, "bk_custom_widgets", ()):
                    w.configure(state="normal" if editable else "disabled")
            self.draw_bk_window()

    def draw_bk_window(self):
        """
        [EN] Render the Drawing Studio preview canvas:
             - Redraws the static grid, bounding box, and yellow planned strokes
               only when `self.bk_canvas_key` changes.
             - Incrementally appends new pink live-ink segments (`tags="ink"`)
               using `self._bk_trail_drawn` to avoid full-canvas redraws.
        [VI] Vẽ khung xem trước trong cửa sổ Drawing Studio:
             - Chỉ vẽ lại lưới tọa độ, khung bao và đường dự kiến màu vàng khi
               `self.bk_canvas_key` thay đổi.
             - Vẽ nối tiếp từng đoạn vết mực thực tế màu hồng (`tags="ink"`)
               nhờ bộ đếm `self._bk_trail_drawn` để tránh phải xóa vẽ lại toàn bộ.
        """
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
            """
            [EN] Convert world coordinates `(x, y)` in mm to preview canvas pixels.
            [VI] Đổi tọa độ thực `(x, y)` (mm) sang tọa độ điểm ảnh trên khung xem trước.
            """
            screen_x = w / 2 + (x - s.center_x) * factor
            screen_y = h / 2 - (y - s.center_y) * factor
            return (screen_x, screen_y)

        # ----------------------------------------------------------------------
        # Layer 1: Static Grid, Bounding Box & Yellow Planned Strokes
        # Lớp 1: Lưới tọa độ tĩnh, khung bao chữ nhật và nét vẽ dự kiến màu vàng
        # ----------------------------------------------------------------------
        key = (w, h, s, self.bk_overlay)
        if key != self.bk_canvas_key:
            canvas.delete("all")
            self.bk_canvas_key = key
            self._bk_trail_drawn = [0 for _ in self.bk_trails]
            half_w = width / 2
            half_h = height / 2

            # X and Y axis tick labels around the pattern bounding box.
            # Nhãn tọa độ X và Y xung quanh khung bao của hình vẽ.
            for v in (-half_w, 0, half_w):
                x, y = point(s.center_x + v, s.center_y - half_h)
                canvas.create_text(
                    x, y + 15, text=f"{s.center_x + v:g}", fill="#9cb0cb"
                )
            for v in (-half_h, 0, half_h):
                x, y = point(s.center_x - half_w, s.center_y + v)
                canvas.create_text(
                    x - 8, y, text=f"{s.center_y + v:.2f}", anchor="e", fill="#9cb0cb"
                )

            canvas.create_rectangle(
                *point(s.center_x - half_w, s.center_y + half_h),
                *point(s.center_x + half_w, s.center_y - half_h),
                outline="#35506b",
                dash=(3, 4),
            )

            # Choose a clean metric grid spacing (`step` in mm) so ~6 grid cells span the pattern.
            # Chọn bước lưới tọa độ (`step` tính bằng mm) sao cho hình vẽ có khoảng 6 ô lưới.
            step = 50
            min_desired_step = max(width, height) / 6
            for candidate_step in (1, 2, 5, 10, 20, 25, 50):
                if candidate_step >= min_desired_step:
                    step = candidate_step
                    break

            x_grid_start = math.ceil((s.center_x - half_w) / step)
            x_grid_end = math.floor((s.center_x + half_w) / step)
            for x in range(x_grid_start, x_grid_end + 1):
                canvas.create_line(
                    *point(x * step, s.center_y - half_h),
                    *point(x * step, s.center_y + half_h),
                    fill="#203249",
                )

            y_grid_start = math.ceil((s.center_y - half_h) / step)
            y_grid_end = math.floor((s.center_y + half_h) / step)
            for y in range(y_grid_start, y_grid_end + 1):
                canvas.create_line(
                    *point(s.center_x - half_w, y * step),
                    *point(s.center_x + half_w, y * step),
                    fill="#203249",
                )

            # Draw each planned stroke in yellow (`#efc25a`) with a green start dot and stroke number.
            # Vẽ từng nét dự kiến bằng màu vàng (`#efc25a`), đánh dấu điểm bắt đầu màu xanh lá kèm số thứ tự nét.
            for i, stroke in enumerate(self.bk_overlay, 1):
                coords = []
                for vertex in stroke:
                    sx, sy = point(*vertex.xy)
                    coords.append(sx)
                    coords.append(sy)
                canvas.create_line(*coords, fill="#efc25a", width=3)

                start_x, start_y = point(*stroke[0].xy)
                canvas.create_oval(
                    start_x - 4,
                    start_y - 4,
                    start_x + 4,
                    start_y + 4,
                    fill="#60d5ae",
                    outline="",
                )
                canvas.create_text(
                    start_x + 7, start_y - 10, text=str(i), fill="#60d5ae"
                )

        # ----------------------------------------------------------------------
        # Layer 2: Incremental Pink Live-Ink Trail (`tags="ink"`)
        # Lớp 2: Vẽ bồi từng đoạn vết mực thực tế màu hồng (`tags="ink"`)
        # ----------------------------------------------------------------------
        need_trail_reset = len(self._bk_trail_drawn) != len(self.bk_trails)
        if not need_trail_reset:
            for trail_pts, drawn_count in zip(self.bk_trails, self._bk_trail_drawn):
                if len(trail_pts) < drawn_count:
                    need_trail_reset = True
                    break

        if need_trail_reset:
            canvas.delete("ink")
            self._bk_trail_drawn = [0 for _ in self.bk_trails]

        for idx, trail in enumerate(self.bk_trails):
            drawn = self._bk_trail_drawn[idx]
            if len(trail) >= 2 and drawn < len(trail):
                # Slice from `drawn - 1` so the new segment connects seamlessly to the last point.
                # Lấy từ điểm `drawn - 1` để đoạn thẳng mới nối liền mạch với điểm cuối trước đó.
                segment = trail[max(0, drawn - 1) :]
                if len(segment) >= 2:
                    segment_coords = []
                    for xy in segment:
                        sx, sy = point(*xy)
                        segment_coords.append(sx)
                        segment_coords.append(sy)
                    canvas.create_line(
                        *segment_coords,
                        fill="#ff8795",
                        width=2,
                        tags="ink",
                    )
                self._bk_trail_drawn[idx] = len(trail)

    def draw_bk_overlay(self, canvas, point):
        """
        [EN] Render the yellow drawing preview overlay onto the main 2D workspace canvas.
        [VI] Vẽ lớp phủ hình xem trước màu vàng lên đồ thị vùng làm việc 2D chính.
        """
        if not self.bk_overlay:
            canvas.delete("bk")
            self.bk_overlay_key = None
            return
        key = (self.grid_key, self.bk_overlay)
        if key != self.bk_overlay_key or not canvas.find_withtag("bk"):
            canvas.delete("bk")
            self.bk_overlay_key = key
            for stroke in self.bk_overlay:
                flat_coords = []
                for vertex in stroke:
                    sx, sy = point(*vertex.xy)
                    flat_coords.append(sx)
                    flat_coords.append(sy)
                canvas.create_line(
                    *flat_coords,
                    fill="#efc25a",
                    width=2,
                    tags="bk",
                )
