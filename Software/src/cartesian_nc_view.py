"""
================================================================================
FILE: cartesian_nc_view.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Tkinter widget factories, dirty-keyed telemetry/button state rendering,
     and the mechanical configuration dialog (`StudioView` mixin for `NCApp`).
     Uses state-tuple caching (`_view_btn_key`, `_view_tel_key`) so Tkinter
     widgets are only reconfigured when relevant state actually changes,
     eliminating UI flicker and CPU overhead during 25Hz–66Hz polling.

     HOW DIRTY-KEY CACHING WORKS (For 1st/2nd-Year Students):
     If we call `button.configure(...)` on 15 buttons 60 times every second,
     Tkinter will spend huge CPU time redrawing unchanged buttons, causing the
     window to flicker. Instead, we pack the variables that affect button states
     into a tuple `btn_key`. We only call `.configure(...)` when `btn_key` is
     different from the previous frame's `self._view_btn_key`!

[VI] MỤC ĐÍCH:
     Các hàm tạo widget Tkinter, cập nhật hiển thị telemetry/trạng thái nút bấm
     có bộ đệm kiểm tra thay đổi (`_view_btn_key`, `_view_tel_key`), và hộp thoại
     cấu hình thông số cơ khí (`StudioView` mixin cho `NCApp`).
     Nhờ cơ chế so sánh khóa trạng thái, các widget chỉ cập nhật khi dữ liệu thực sự
     thay đổi, giúp giao diện mượt mà và không bị nhấp nháy ở tần số quét 25Hz–66Hz.

     NGUYÊN LÝ BỘ ĐỆM KHÓA TRẠNG THÁI (Dành cho Sinh viên năm 1–2):
     Nếu gọi lệnh `button.configure(...)` liên tục 60 lần/giây cho hàng chục nút
     bấm dù trạng thái không đổi, giao diện Tkinter sẽ bị giật và nhấp nháy.
     Để khắc phục, ta gom các biến điều kiện vào một bộ (tuple) `btn_key`. Chỉ
     khi `btn_key` khác với giá trị ở chu kỳ trước (`_view_btn_key`), chương trình
     mới thực sự cập nhật lại các nút bấm!

[DEBUG TIP / MẸO GỠ LỖI]:
     - In `settings()`, the top quick-edit form fields (`l1_mm`, `l2_mm`, `beta`, etc.)
       automatically synchronize with the raw JSON editor below via `sync_form_to_json`.
       Saving new mechanical parameters clears `self.local_reference` so the user
       must run `HOME + CALIB` again with the updated geometry.
     - Trong hàm `settings()`, biểu mẫu chỉnh nhanh phía trên (`l1_mm`, `l2_mm`, `beta`,...)
       tự động đồng bộ với khung soạn thảo JSON bên dưới qua `sync_form_to_json`.
       Khi lưu thông số mới, `self.local_reference` sẽ được đặt lại về `False` để bắt buộc
       chạy lại `HOME + CALIB` với thông số hình học mới.
================================================================================
"""

from dataclasses import asdict
import json
import time
import tkinter as tk
from cartesian_nc_model import NCConfig
from cartesian_nc_messages import STAGES, PHASES, Z_PHASES, ERRORS, REASONS
from cartesian_nc_theme import BG, CARD, TEXT, MUTED, GREEN, RED
from cartesian_nc_shapes import PATTERNS
from cartesian_nc_layout import build_dashboard, update_dashboard


class StudioView:
    """
    [EN] Mixin class providing reusable UI widget builders, live telemetry
         formatting, and the mechanical settings editor window.
    [VI] Lớp Mixin cung cấp các hàm tạo phần tử giao diện, định dạng thông số
         telemetry trực tiếp và cửa sổ chỉnh sửa cấu hình cơ khí.
    """

    # ==========================================================================
    # 1. WIDGET FACTORY HELPERS / CÁC HÀM TẠO PHẦN TỬ GIAO DIỆN CƠ BẢN
    # ==========================================================================

    def label(self, parent, text=None, size=11, color=TEXT, **kw):
        """
        [EN] Create a themed `tk.Label` inheriting the parent's background color.
        [VI] Tạo nhãn `tk.Label` theo tông màu giao diện, kế thừa màu nền của khung chứa.
        """
        parent_bg = parent.cget("bg")
        return tk.Label(
            parent,
            text=text,
            bg=parent_bg,
            fg=color,
            font=("Segoe UI", size),
            **kw,
        )

    def button(self, parent, text, command, color="#2b3c55", **kw):
        """
        [EN] Create a flat themed `tk.Button` bound to `command`.
        [VI] Tạo nút bấm phẳng `tk.Button` theo chủ đề màu, gắn với hàm xử lý `command`.
        """
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=color,
            fg=TEXT,
            activebackground="#3a5b7e",
            activeforeground=TEXT,
            relief="flat",
            font=("Segoe UI", 11),
            padx=12,
            pady=4,
            **kw,
        )

    def card(self, parent):
        """
        [EN] Create a padded card container (`tk.Frame`) with background `CARD`.
        [VI] Tạo khung chứa dạng thẻ (`tk.Frame`) có đệm lề và màu nền `CARD`.
        """
        return tk.Frame(parent, bg=CARD, padx=16, pady=12)

    def entry(self, parent, var):
        """
        [EN] Create a flat numeric/text input box (`tk.Entry`) bound to `var`.
        [VI] Tạo ô nhập liệu phẳng (`tk.Entry`) gắn với biến quan sát `var`.
        """
        return tk.Entry(
            parent,
            textvariable=var,
            width=10,
            bg="#243651",
            fg=TEXT,
            insertbackground=TEXT,
            relief="flat",
            font=("Segoe UI", 12),
        )

    def _build(self):
        """
        [EN] Construct the main dashboard layout inside `self`.
        [VI] Dựng bố cục bảng điều khiển chính bên trong cửa sổ `self`.
        """
        build_dashboard(self)

    # ==========================================================================
    # 2. TELEMETRY & UI STATE RENDERING / CẬP NHẬT TRẠNG THÁI NÚT & TELEMETRY
    # ==========================================================================

    def render(self):
        """
        [EN] Synchronize button enable/disable states, switch bitmasks, homing
             progress, firmware diagnostics, and live XYZ readouts with `self.state`.
        [VI] Đồng bộ trạng thái bật/tắt của các nút bấm, trạng thái 6 công tắc,
             tiến độ dò gốc, chẩn đoán firmware và tọa độ XYZ trực tiếp từ `self.state`.
        """
        if not hasattr(self, "home_btn"):
            return

        # Step 1: Query current high-level operational flags.
        # Bước 1: Kiểm tra các cờ trạng thái vận hành tổng quát hiện tại.
        fresh = self.fresh()
        idle = self.idle()
        operate = self.operable()
        has_program_running = bool(self.program_points)
        is_follow_active = self.follow_enabled.get()
        autonomous = has_program_running or is_follow_active

        is_firmware_busy = bool(self.state and self.state.busy)
        holding_mask = self.state.holding if self.state else None
        fault_flag = self.state.fault if self.state else None

        # ----------------------------------------------------------------------
        # PART 1: Update button states only when the control state tuple changes
        # PHẦN 1: Chỉ cập nhật trạng thái các nút bấm khi bộ khóa điều khiển đổi
        # ----------------------------------------------------------------------
        btn_key = (
            bool(self.link),
            fresh,
            idle,
            operate,
            autonomous,
            self.active_job,
            is_firmware_busy,
            bool(self.pending),
            bool(self.planning),
            holding_mask,
            self.local_reference,
            fault_flag,
            self.config,
        )
        if btn_key != getattr(self, "_view_btn_key", None):
            self._view_btn_key = btn_key

            # 1a. Update Connect / Disconnect button label.
            # 1a. Cập nhật nhãn nút Kết nối / Ngắt kết nối.
            connect_text = "Ngắt kết nối" if self.link else "Kết nối"
            self.connect_btn.configure(text=connect_text)

            # 1b. Update Homing speed hint text in the Home tab.
            # 1b. Cập nhật dòng gợi ý tốc độ quét/chốt công tắc trong tab Home.
            if hasattr(self, "home_hint"):
                hint_line_1 = (
                    f"Z quét {self.config.z_home_speed_mm_s:g} mm/s "
                    f"≈{self.config.z_home_pps:,} xung/s · chốt {self.config.z_latch_pps:,}\n"
                )
                hint_line_2 = (
                    f"J2 quét {self.config.j2_home_pps:,} · "
                    f"chốt {self.config.j2_latch_pps:,} xung/s"
                )
                self.home_hint.configure(text=hint_line_1 + hint_line_2)

            # 1c. Update top-right Connection status badge.
            # 1c. Cập nhật huy hiệu báo trạng thái kết nối ở góc trên phải.
            badge_text = "● ĐÃ KẾT NỐI" if fresh else "● CHƯA KẾT NỐI"
            badge_color = GREEN if fresh else MUTED
            self.badge.configure(text=badge_text, fg=badge_color)

            # 1d. Enable Mechanical Settings button only when completely idle.
            # 1d. Chỉ cho phép bấm nút "Cơ khí" khi máy hoàn toàn rảnh.
            can_open_settings = (
                (not autonomous)
                and (self.active_job is None)
                and (not is_firmware_busy)
                and (not self.pending)
                and (not self.planning)
            )
            self.settings_btn.configure(
                state="normal" if can_open_settings else "disabled"
            )

            # 1e. Enable HOME buttons only when operable and all 3 axes hold torque (holding == 7 = 0b111).
            # 1e. Chỉ bật nút HOME khi sẵn sàng vận hành và cả 3 trục đang giữ lực (holding == 7 = 0b111).
            can_home = operate and (not autonomous) and (self.state.holding == 7)
            home_state = "normal" if can_home else "disabled"
            self.home_btn.configure(state=home_state)
            if hasattr(self, "quick_home_btn"):
                self.quick_home_btn.configure(state=home_state)

            # 1f. Enable XYZ Preview / Move / Current buttons only after homing (`local_reference`).
            # 1f. Chỉ bật các nút Xem trước / Di chuyển XYZ sau khi đã lấy mốc (`local_reference`).
            can_move_xyz = (
                operate and self.local_reference and (not self.program_points)
            )
            xyz_btn_state = "normal" if can_move_xyz else "disabled"
            for b in (self.preview_btn, self.move_btn, self.current_btn):
                b.configure(state=xyz_btn_state)

            # 1g. Enable Release / Hold arm buttons when idle and not faulted.
            # 1g. Bật nút Nhả / Giữ arm khi máy đang đứng yên và không có lỗi.
            can_toggle_hold = idle and (not autonomous) and (not self.state.fault)
            hold_btn_state = "normal" if can_toggle_hold else "disabled"
            for b in (self.release_btn, self.hold_btn):
                b.configure(state=hold_btn_state)

        # Render workspace editor controls and Drawing Studio controls.
        # Cập nhật trạng thái bảng điều khiển vùng làm việc và cửa sổ Drawing Studio.
        self.render_editor()
        self.render_drawing()

        # ----------------------------------------------------------------------
        # PART 2: Format live firmware telemetry strings when `tel_key` changes
        # PHẦN 2: Định dạng chuỗi thông tin telemetry trực tiếp khi `tel_key` đổi
        # ----------------------------------------------------------------------
        if self.state:
            s = self.state

            # Bucket elapsed drawing time to 0.1s intervals so the UI updates smoothly.
            # Làm tròn thời gian vẽ theo từng bước 0.1 giây để giao diện cập nhật mượt.
            is_uploading_current_job = bool(
                self.path_upload and s.job == self.path_upload.job
            )
            if is_uploading_current_job:
                started_t = getattr(self, "bk_started", 0.0)
                elapsed_bucket = round(time.monotonic() - started_t, 1)
            else:
                elapsed_bucket = None

            tel_key = (
                s,
                self.local_reference,
                id(self.model),
                self.program_points,
                self.program_index,
                elapsed_bucket,
                self.config,
            )
            if tel_key != getattr(self, "_view_tel_key", None):
                self._view_tel_key = tel_key

                # --------------------------------------------------------------
                # 2a. Format 6-switch status bar (Z, J1, J2 positive/negative limits)
                #     Bitmask layout in `s.switches`:
                #     - Axis 0 (Z):  bit 0 = Z+,  bit 1 = Z−
                #     - Axis 1 (J1): bit 2 = J1+, bit 3 = J1−
                #     - Axis 2 (J2): bit 4 = J2+, bit 5 = J2−
                # 2a. Định dạng thanh trạng thái 6 công tắc hành trình (Z, J1, J2):
                #     Cấu trúc bit trong `s.switches`:
                #     - Trục 0 (Z):  bit 0 = Z+,  bit 1 = Z−
                #     - Trục 1 (J1): bit 2 = J1+, bit 3 = J1−
                #     - Trục 2 (J2): bit 4 = J2+, bit 5 = J2−
                # --------------------------------------------------------------
                pins = ("Z PA3/PA4", "J1 PB0/PB1", "J2 PB10/PB11")
                text = []
                for axis_idx, pin_label in enumerate(pins):
                    # Shift right by `2 * axis_idx` and mask the 2 lowest bits (`& 3`).
                    # Dịch phải `2 * axis_idx` bit và lấy 2 bit cuối (`& 3`) của trục đó.
                    bits = (s.switches >> (2 * axis_idx)) & 3
                    pos_state_str = "CHẠM/HỞ" if (bits & 1) else "đóng"
                    neg_state_str = "CHẠM/HỞ" if (bits & 2) else "đóng"

                    axis_summary = f"{pin_label}: +{pos_state_str} / −{neg_state_str}"
                    if s.conflicts & (1 << axis_idx):
                        axis_summary += " (lỗi đã giữ)"
                    if s.errors & (1 << axis_idx):
                        axis_summary += " (GPIO lỗi)"
                    text.append(axis_summary)

                self.switch_text.set("   ·   ".join(text))

                # --------------------------------------------------------------
                # 2b. Format stage & motion progress banner
                # 2b. Định dạng nhãn tiến độ giai đoạn & chuyển động
                # --------------------------------------------------------------
                progress = STAGES[s.stage]
                self.progress_label.configure(height=1 if s.stage == 8 else 3)

                if s.stage == 8 and s.busy:
                    progress = "Đang di chuyển"

                if self.program_points:
                    current_pt = self.program_points[self.program_index]
                    total_pts = len(self.program_points)
                    progress = (
                        f"Chuỗi {self.program_index + 1}/{total_pts}: {current_pt.name}"
                    )

                if self.path_upload and s.job == self.path_upload.job:
                    total, received, done, free = s.path
                    code = self.bk_active_plan.settings.pattern
                    if hasattr(self, "_pattern_title"):
                        ptitle = self._pattern_title(code)
                    else:
                        ptitle = PATTERNS.get(code, (code,))[0]

                    stroke_num = (
                        self.bk_active_plan.operations[self.program_index].stroke + 1
                    )
                    progress = f"Nét {stroke_num}: {done}/{total} đoạn · {s.pps} xung/s"

                    elapsed_s = time.monotonic() - self.bk_started
                    fifo_occupied = 32 - free
                    max_err = self.bk_active_plan.max_model_error_mm
                    self.bk_note.set(
                        f"{ptitle} · {elapsed_s:.1f}s\n"
                        f"Nét {done}/{total} đoạn; bộ đệm {fifo_occupied}/32 · nạp {received}/{total}\n"
                        f"Sai lệch mô hình ≤{max_err:.3f}mm; vị trí theo xung."
                    )

                if s.stage == 8 and not s.referenced:
                    progress = "Mất mốc · cần HOME + CALIB"

                # If currently scanning limit switches during homing, append phase & step counts.
                # Nếu đang trong giai đoạn dò công tắc hành trình, hiển thị thêm pha và số xung đo đi/về.
                if s.busy and s.stage in (1, 2, 4, 6, 11):
                    if s.stage == 1:
                        active_axis = 0  # Z axis
                    elif s.stage == 4:
                        active_axis = 1  # J1 shoulder axis
                    else:
                        active_axis = 2  # J2 elbow axis

                    phase_table = Z_PHASES if active_axis == 0 else PHASES
                    phase_name = phase_table[s.phase[active_axis]]
                    progress += "\n" + phase_name
                    progress += (
                        f"\nĐo đi/về: {s.n1[active_axis]} / {s.n2[active_axis]} xung"
                    )

                # If homing failed (stage 9), list per-axis error reasons and failed phases.
                # Nếu quá trình lấy gốc thất bại (stage 9), liệt kê lỗi cụ thể của từng trục.
                if s.stage == 9:
                    errors = []
                    axis_names = ("Z", "J1", "J2")
                    for i, (axis_name, err_code) in enumerate(
                        zip(axis_names, s.cal_error)
                    ):
                        if err_code:
                            err_desc = (
                                ERRORS[err_code]
                                if err_code < len(ERRORS)
                                else err_code
                            )
                            phase_table = Z_PHASES if i == 0 else PHASES
                            failed_phase_name = phase_table[s.failed_phase[i]]
                            errors.append(
                                f"{axis_name}: {err_desc} (pha {failed_phase_name})"
                            )
                    reason_text = REASONS.get(s.reason, s.reason)
                    progress += "\n" + reason_text + "\n" + "; ".join(errors)

                if s.input_wait:
                    progress = (
                        "TẠM DỪNG XUNG\nĐang xác nhận công tắc\n" + STAGES[s.stage]
                    )
                if s.gate_wait:
                    gate_axis_name = ("Z", "J1", "J2")[s.gate_axis]
                    progress = (
                        f"Đợi chân {gate_axis_name} ổn định\n"
                        "Trước khi chạy trục tiếp theo"
                    )
                self.progress.set(progress)

                # --------------------------------------------------------------
                # 2c. Format low-level firmware diagnostics tab
                #     Note: `beta_q` is Q20 fixed-point (divided by 2^20 = 1,048,576),
                #     and `coupling_ppm` is parts-per-million (divided by 1,000,000).
                # 2c. Định dạng tab chẩn đoán phần cứng & firmware chi tiết
                #     Lưu ý: `beta_q` lưu dạng dấu phẩy tĩnh Q20 (chia cho 2^20 = 1.048.576),
                #     còn `coupling_ppm` tính theo phần triệu (chia cho 1.000.000).
                # --------------------------------------------------------------
                beta_ratio = s.beta_q / 1048576
                coupling_status = "đã đo" if s.coupling_ready else "chưa xác nhận"
                k_val = s.coupling_ppm / 1000000
                dir_z = s.dir_levels & 1
                dir_j1 = (s.dir_levels >> 1) & 1
                dir_j2 = (s.dir_levels >> 2) & 1

                fw_line_1 = (
                    f"{s.build} · N[Z,J1,J2]={s.range} · "
                    f"timer trễ={s.timer_late} · GPIO lỗi={s.motor_error}\n"
                )
                fw_line_2 = (
                    f"Bù B/A={beta_ratio:+.5f} xung/xung ({coupling_status}) · "
                    f"thử ΔA/ΔB={s.probe_da}/{s.probe_db} · "
                    f"k={k_val:.6f} · DIR[Z,J1,J2]={dir_z}/{dir_j1}/{dir_j2}"
                )
                self.fw_label.configure(text=fw_line_1 + fw_line_2)

                if s.input_axis >= 0 and s.input_kind:
                    pairs = (
                        ("Z", "PA3", "PA4"),
                        ("J1", "PB0", "PB1"),
                        ("J2", "PB10", "PB11"),
                    )
                    axis, p, n = pairs[s.input_axis]
                    bits = (s.input_bits >> (2 * s.input_axis)) & 3
                    event_descriptions = {
                        1: "đã từng đọc HIGH/HIGH",
                        2: "HIGH/HIGH đã giữ lỗi",
                        3: "tín hiệu dao động quá lâu",
                        4: "lỗi đọc GPIO",
                        5: "một biên mở nhưng không xác nhận ổn định",
                        6: "xung thoáng qua ở trục đứng yên đã lọc",
                        7: "tín hiệu chưa an toàn để chạy trục",
                    }
                    event = event_descriptions[s.input_kind]
                    current = (s.raw >> (2 * s.input_axis)) & 3

                    event_pos_bit = bits & 1
                    event_neg_bit = (bits >> 1) & 1
                    cur_pos_bit = current & 1
                    cur_neg_bit = (current >> 1) & 1

                    self.input_details.set(
                        f"Nguồn sự kiện: {axis} · {p}/{n}={event_pos_bit}/{event_neg_bit} "
                        f"lúc {s.input_at} ms; hiện tại {cur_pos_bit}/{cur_neg_bit}. "
                        f"{event} · tạm dừng phục hồi: {s.recovered} · "
                        f"đã lọc khi đứng yên: {s.idle_ignored} · mở giả [Z,J1,J2]: {s.false_hits}"
                    )
                    fault_stop = s.reason in (
                        "both_open",
                        "both_latched",
                        "switch_unstable",
                        "calibration_failed",
                        "switch_io",
                        "j2_guard",
                        "edge_limit",
                    )
                    is_error_highlight = (
                        bool(s.errors)
                        or bool(s.conflicts)
                        or (s.stage == 9 and fault_stop)
                        or (s.input_kind in (2, 3, 4, 5, 7))
                    )
                    self.input_label.configure(
                        fg=RED if is_error_highlight else GREEN
                    )
                else:
                    self.input_details.set("")

                # --------------------------------------------------------------
                # 2d. Format live Cartesian XYZ and Joint angles readout
                # 2d. Định dạng thông số tọa độ Đề-các XYZ và góc khớp trực tiếp
                # --------------------------------------------------------------
                if self.local_reference and self.model:
                    q = self.model.from_steps(s.pos)
                    xyz = self.model.fk(q)
                    self.metric.set(
                        "X {:+.2f}    Y {:+.2f}    Z {:.2f} mm".format(*xyz)
                    )
                    z_max_mm = s.range[0] / self.model.factors[0]
                    if self.config.z_span_mm:
                        z_span_note = "đã nhập hành trình thực"
                    else:
                        z_span_note = "ước tính; hãy đo và nhập hành trình thực"

                    self.details.set(
                        f"J1 {q[0]:+.2f}° · J2 tương đối {q[1]:+.2f}° · Xung theo mốc: {s.pos}\n"
                        f"Tổng xung phát [Z,J1,J2]: {s.total}\n"
                        f"Z: 0–{z_max_mm:.2f} mm {z_span_note} · Vị trí theo xung, chưa có encoder."
                    )
                else:
                    is_currently_homing = s.busy and (s.stage not in (8, 9))
                    self.metric.set(
                        "ĐANG LẤY MỐC" if is_currently_homing else "CHƯA LẤY MỐC"
                    )
                    reason_desc = REASONS.get(s.reason, s.reason)
                    self.details.set(
                        "Tọa độ XYZ chỉ có hiệu lực sau khi cả ba trục hoàn thành khởi tạo.\n"
                        f"Tổng xung phát [Z,J1,J2]: {s.total}\n"
                        + reason_desc
                    )

        # Step 3: Record live telemetry sample and refresh Analytics, Dashboard, and Canvas.
        # Bước 3: Lưu mẫu telemetry và cập nhật cửa sổ Phân tích, Bảng điều khiển, Đồ thị.
        if self.state and hasattr(self, "record_telemetry_sample"):
            self.record_telemetry_sample(self.state)
        if hasattr(self, "render_analytics"):
            self.render_analytics()
        update_dashboard(self)
        self.draw()

    def draw(self):
        """
        [EN] Redraw the 2D/3D interactive workspace canvas.
        [VI] Vẽ lại mặt phẳng làm việc tương tác 2D/3D trên Canvas.
        """
        self.draw_workspace()

    # ==========================================================================
    # 3. MECHANICAL SETTINGS DIALOG / HỘP THOẠI CẤU HÌNH THÔNG SỐ CƠ KHÍ
    # ==========================================================================

    def settings(self):
        """
        [EN] Open the mechanical & homing configuration dialog (`NCConfig`),
             offering both a quick-entry form for common parameters and a full
             JSON editor for advanced tuning.
        [VI] Mở cửa sổ thiết lập thông số cơ khí & dò gốc (`NCConfig`), bao gồm
             biểu mẫu nhập nhanh các thông số chính và khung soạn thảo JSON đầy đủ.
        """
        if self.settings_window and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return

        win = tk.Toplevel(self)
        self.settings_window = win
        win.title("Thông số SCARA / NC")
        win.geometry("820x740")
        win.configure(bg=BG)

        self.label(win, "THÔNG SỐ CƠ KHÍ & HOMING", 17).pack(
            anchor="w", padx=18, pady=(12, 4)
        )
        self.label(
            win,
            "l1/l2 phải đo giữa các tâm khớp/điểm gá; tool_offset_mm là độ lệch đầu bút tại gá arm 2.\n"
            "Chỉnh nhanh qua biểu mẫu bên dưới hoặc chỉnh trực tiếp JSON nâng cao.",
            10,
            MUTED,
            justify="left",
        ).pack(anchor="w", padx=18, pady=(0, 8))

        # Top quick-entry card containing 8 primary mechanical parameters in a 2x4 grid.
        # Khung biểu mẫu nhập nhanh phía trên gồm 8 thông số cơ khí chính xếp lưới 2x4.
        form = self.card(win)
        form.pack(fill="x", padx=18, pady=(0, 8))
        form_fields = (
            ("l1_mm", "Dài L1 (mm)", float),
            ("l2_mm", "Dài L2 (mm)", float),
            ("z_span_mm", "Hành trình Z (mm)", float),
            ("j1_span_deg", "Góc J1 (°)", float),
            ("j2_span_deg", "Góc J2 (°)", float),
            ("z_home_speed_mm_s", "Tốc độ home Z (mm/s)", float),
            ("j2_home_pps", "Xung home J2 (pps)", int),
            ("beta", "Hệ số bù dây đai β", float),
        )
        form_vars = {}
        syncing = False

        # Raw JSON editor widget below the quick form.
        # Khung soạn thảo văn bản JSON chi tiết nằm bên dưới biểu mẫu.
        text = tk.Text(
            win,
            bg=CARD,
            fg=TEXT,
            insertbackground=TEXT,
            font=("Consolas", 10),
            wrap="none",
            undo=True,
            height=14,
        )

        def sync_form_to_json(*_args):
            """
            [EN] Whenever a quick-form box is edited, update the corresponding key
                 in the JSON text box below (`syncing` flag prevents re-entry).
            [VI] Mỗi khi người dùng sửa một ô trên biểu mẫu nhanh, tự động cập nhật
                 khóa tương ứng trong khung JSON bên dưới (cờ `syncing` tránh lặp vô tận).
            """
            nonlocal syncing
            if syncing:
                return
            try:
                raw = json.loads(text.get("1.0", "end"))
                for key, _label, caster in form_fields:
                    raw[key] = caster(form_vars[key].get())
                syncing = True
                text.delete("1.0", "end")
                text.insert("1.0", json.dumps(raw, indent=2, ensure_ascii=False))
            except Exception:
                pass
            finally:
                syncing = False

        # Create the 8 quick-edit cells (4 columns per row).
        # Tạo 8 ô nhập liệu nhanh (mỗi hàng 4 cột: `row = idx // 4`, `col = idx % 4`).
        cfg_dict = asdict(self.config)
        for idx, (key, title, _caster) in enumerate(form_fields):
            row_idx = idx // 4
            col_idx = idx % 4
            cell = tk.Frame(form, bg=CARD)
            cell.grid(row=row_idx, column=col_idx, sticky="ew", padx=6, pady=4)
            form.columnconfigure(col_idx, weight=1)
            self.label(cell, title, 9, MUTED).pack(anchor="w")

            initial_str = str(cfg_dict.get(key, ""))
            var = tk.StringVar(value=initial_str)
            var.trace_add("write", sync_form_to_json)
            form_vars[key] = var
            self.entry(cell, var).pack(fill="x")

        text.pack(fill="both", expand=True, padx=18, pady=4)
        text.insert("1.0", json.dumps(cfg_dict, indent=2, ensure_ascii=False))

        note = tk.StringVar()
        self.label(win, textvariable=note, color=RED, wraplength=750).pack(
            fill="x", padx=18
        )

        def save():
            """
            [EN] Validate and save the JSON configuration to disk, then invalidate
                 the current reference so the user re-homes with the new geometry.
            [VI] Kiểm tra và lưu cấu hình JSON xuống đĩa, đồng thời xóa mốc tọa độ
                 cũ để yêu cầu người dùng chạy lại HOME + CALIB với thông số mới.
            """
            try:
                if not self.idle() and self.link:
                    raise ValueError("Đang xử lý tác vụ; dừng trước khi đổi thông số.")
                parsed_json = json.loads(text.get("1.0", "end"))
                cfg = NCConfig.from_dict(parsed_json)
                cfg.save(self.config_path)
                self.config = cfg
                self.config_error = None
                self.local_reference = False
                self.model = None
                self.preview = None
                self.message.set(
                    "Đã lưu thông số. Bấm HOME + CALIB để áp dụng và lấy lại mốc."
                )
                win.destroy()
                self.settings_window = None
                self.render()
            except Exception as e:
                note.set(str(e))

        self.button(win, "Lưu thông số", save, "#1c6b58").pack(pady=(4, 14))
