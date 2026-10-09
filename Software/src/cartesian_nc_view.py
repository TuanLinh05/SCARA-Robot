"""Tk widgets, telemetry rendering and mechanical settings for NCApp."""

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
    def label(self, parent, text=None, size=11, color=TEXT, **kw):
        return tk.Label(
            parent,
            text=text,
            bg=parent.cget("bg"),
            fg=color,
            font=("Segoe UI", size),
            **kw,
        )

    def button(self, parent, text, command, color="#2b3c55", **kw):
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
        return tk.Frame(parent, bg=CARD, padx=16, pady=12)

    def entry(self, parent, var):
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
        build_dashboard(self)

    def render(self):
        if not hasattr(self, "home_btn"):
            return
        fresh = self.fresh()
        idle = self.idle()
        operate = self.operable()
        self.connect_btn.configure(text="Ngắt kết nối" if self.link else "Kết nối")
        if hasattr(self, "home_hint"):
            self.home_hint.configure(
                text=f"Z quét {self.config.z_home_speed_mm_s:g} mm/s ≈{self.config.z_home_pps:,} xung/s · chốt {self.config.z_latch_pps:,}\n"
                + f"J2 quét {self.config.j2_home_pps:,} · chốt {self.config.j2_latch_pps:,} xung/s"
            )
        self.badge.configure(
            text="● ĐÃ KẾT NỐI" if fresh else "● CHƯA KẾT NỐI",
            fg=GREEN if fresh else MUTED,
        )
        autonomous = bool(self.program_points) or self.follow_enabled.get()
        self.settings_btn.configure(
            state=(
                "normal"
                if not autonomous
                and self.active_job is None
                and not (self.state and self.state.busy)
                and not self.pending
                and not self.planning
                else "disabled"
            )
        )
        self.home_btn.configure(
            state=(
                "normal"
                if operate and not autonomous and self.state.holding == 7
                else "disabled"
            )
        )
        for b in (self.preview_btn, self.move_btn, self.current_btn):
            b.configure(
                state=(
                    "normal"
                    if operate and self.local_reference and not self.program_points
                    else "disabled"
                )
            )
        for b in (self.release_btn, self.hold_btn):
            b.configure(
                state=(
                    "normal"
                    if idle and not autonomous and not self.state.fault
                    else "disabled"
                )
            )
        self.render_editor()
        self.render_drawing()
        if self.state:
            s = self.state
            pins = ("Z PA3/PA4", "J1 PB0/PB1", "J2 PB10/PB11")
            text = []
            for a, p in enumerate(pins):
                bits = (s.switches >> (2 * a)) & 3
                text.append(
                    f"{p}: +{'CHẠM/HỞ' if bits&1 else 'đóng'} / −{'CHẠM/HỞ' if bits&2 else 'đóng'}"
                )
                if s.conflicts & (1 << a):
                    text[-1] += " (lỗi đã giữ)"
                if s.errors & (1 << a):
                    text[-1] += " (GPIO lỗi)"
            self.switch_text.set("   ·   ".join(text))
            progress = STAGES[s.stage]
            self.progress_label.configure(height=1 if s.stage == 8 else 3)
            if s.stage == 8 and s.busy:
                progress = "Đang di chuyển"
            if self.program_points:
                progress = f"Chuỗi {self.program_index+1}/{len(self.program_points)}: {self.program_points[self.program_index].name}"
            if self.path_upload and s.job == self.path_upload.job:
                total, received, done, free = s.path
                progress = f"Nét {self.bk_active_plan.operations[self.program_index].stroke+1}: {done}/{total} đoạn · {s.pps} xung/s"
                self.bk_note.set(
                    f"{PATTERNS[self.bk_active_plan.settings.pattern][0]} · {time.monotonic()-self.bk_started:.1f}s\nNét {done}/{total} đoạn; bộ đệm {32-free}/32 · nạp {received}/{total}\nSai lệch mô hình ≤{self.bk_active_plan.max_model_error_mm:.3f}mm; vị trí theo xung."
                )
            if s.stage == 8 and not s.referenced:
                progress = "Mất mốc · cần HOME + CALIB"
            if s.busy and s.stage in (1, 2, 4, 6, 11):
                a = 0 if s.stage == 1 else 1 if s.stage == 4 else 2
                progress += "\n" + (Z_PHASES if a == 0 else PHASES)[s.phase[a]]
                progress += f"\nĐo đi/về: {s.n1[a]} / {s.n2[a]} xung"
            if s.stage == 9:
                errors = [
                    f"{a}: {ERRORS[e] if e<len(ERRORS) else e} (pha {(Z_PHASES if i==0 else PHASES)[s.failed_phase[i]]})"
                    for i, (a, e) in enumerate(zip(("Z", "J1", "J2"), s.cal_error))
                    if e
                ]
                progress += (
                    "\n" + REASONS.get(s.reason, s.reason) + "\n" + "; ".join(errors)
                )
            if s.input_wait:
                progress = "TẠM DỪNG XUNG\nĐang xác nhận công tắc\n" + STAGES[s.stage]
            if s.gate_wait:
                progress = f"Đợi chân {('Z','J1','J2')[s.gate_axis]} ổn định\nTrước khi chạy trục tiếp theo"
            self.progress.set(progress)
            self.fw_label.configure(
                text=f"{s.build} · N[Z,J1,J2]={s.range} · timer trễ={s.timer_late} · GPIO lỗi={s.motor_error}\n"
                + f"Bù B/A={s.beta_q/1048576:+.5f} xung/xung ({'đã đo' if s.coupling_ready else 'chưa xác nhận'}) · thử ΔA/ΔB={s.probe_da}/{s.probe_db} · k={s.coupling_ppm/1000000:.6f} · DIR[Z,J1,J2]={s.dir_levels&1}/{(s.dir_levels>>1)&1}/{(s.dir_levels>>2)&1}"
            )
            if s.input_axis >= 0 and s.input_kind:
                pairs = (
                    ("Z", "PA3", "PA4"),
                    ("J1", "PB0", "PB1"),
                    ("J2", "PB10", "PB11"),
                )
                axis, p, n = pairs[s.input_axis]
                bits = (s.input_bits >> (2 * s.input_axis)) & 3
                event = {
                    1: "đã từng đọc HIGH/HIGH",
                    2: "HIGH/HIGH đã giữ lỗi",
                    3: "tín hiệu dao động quá lâu",
                    4: "lỗi đọc GPIO",
                    5: "một biên mở nhưng không xác nhận ổn định",
                    6: "xung thoáng qua ở trục đứng yên đã lọc",
                    7: "tín hiệu chưa an toàn để chạy trục",
                }[s.input_kind]
                current = (s.raw >> (2 * s.input_axis)) & 3
                self.input_details.set(
                    f"Nguồn sự kiện: {axis} · {p}/{n}={bits&1}/{(bits>>1)&1} lúc {s.input_at} ms; hiện tại {current&1}/{(current>>1)&1}. "
                    + f"{event} · tạm dừng phục hồi: {s.recovered} · đã lọc khi đứng yên: {s.idle_ignored} · mở giả [Z,J1,J2]: {s.false_hits}"
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
                self.input_label.configure(
                    fg=(
                        RED
                        if s.errors
                        or s.conflicts
                        or (s.stage == 9 and fault_stop)
                        or s.input_kind in (2, 3, 4, 5, 7)
                        else GREEN
                    )
                )
            else:
                self.input_details.set("")
            if self.local_reference and self.model:
                q = self.model.from_steps(s.pos)
                xyz = self.model.fk(q)
                self.metric.set("X {:+.2f}    Y {:+.2f}    Z {:.2f} mm".format(*xyz))
                self.details.set(
                    f"J1 {q[0]:+.2f}° · J2 tương đối {q[1]:+.2f}° · Xung theo mốc: {s.pos}\nTổng xung phát [Z,J1,J2]: {s.total}\n"
                    + f"Z: 0–{s.range[0]/self.model.factors[0]:.2f} mm "
                    + (
                        "đã nhập hành trình thực"
                        if self.config.z_span_mm
                        else "ước tính; hãy đo và nhập hành trình thực"
                    )
                    + " · Vị trí theo xung, chưa có encoder."
                )
            else:
                self.metric.set(
                    "ĐANG LẤY MỐC"
                    if s.busy and s.stage not in (8, 9)
                    else "CHƯA LẤY MỐC"
                )
                self.details.set(
                    "Tọa độ XYZ chỉ có hiệu lực sau khi cả ba trục hoàn thành khởi tạo.\n"
                    + f"Tổng xung phát [Z,J1,J2]: {s.total}\n"
                    + REASONS.get(s.reason, s.reason)
                )
        update_dashboard(self)
        self.draw()

    def draw(self):
        self.draw_workspace()

    def settings(self):
        if self.settings_window and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return
        win = tk.Toplevel(self)
        self.settings_window = win
        win.title("Thông số SCARA / NC")
        win.geometry("790x650")
        win.configure(bg=BG)
        self.label(win, "THÔNG SỐ CƠ KHÍ", 17).pack(anchor="w", padx=18, pady=(12, 4))
        self.label(
            win,
            "l1/l2 phải đo giữa các tâm khớp/điểm gá; tool_offset_mm là độ lệch đầu bút tại gá arm 2.\nOffset [+X sang phải, +Y dọc arm 2] khi hai arm thẳng hướng +Y; chưa đo giữ [0,0].\nGóc giữa công tắc phải đo thật; 180° là giả định. Z chưa đo giữ z_span_mm=0.",
            10,
            MUTED,
            justify="left",
        ).pack(anchor="w", padx=18, pady=(0, 8))
        text = tk.Text(
            win,
            bg=CARD,
            fg=TEXT,
            insertbackground=TEXT,
            font=("Consolas", 11),
            wrap="none",
            undo=True,
        )
        text.pack(fill="both", expand=True, padx=18, pady=4)
        text.insert(
            "1.0", json.dumps(asdict(self.config), indent=2, ensure_ascii=False)
        )
        note = tk.StringVar()
        self.label(win, textvariable=note, color=RED, wraplength=750).pack(
            fill="x", padx=18
        )

        def save():
            try:
                if not self.idle() and self.link:
                    raise ValueError("Đang xử lý tác vụ; dừng trước khi đổi thông số.")
                cfg = NCConfig.from_dict(json.loads(text.get("1.0", "end")))
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
