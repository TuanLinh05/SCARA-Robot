"""
============================================================================
FILE: cartesian_nc_safety.py
============================================================================
[EN] Emergency Stop, Reference Invalidation & Window Focus Safety Policy
     - `stop(source)`: Cancels background planning, clears queued commands,
       transmits `"STOP <source>"` immediately over USB, and invalidates local
       reference if the robot was moving when stopped.
     - `focus_lost(source)`: Automatically disarms interactive cursor-follow
       (or stops the robot if background motion is disabled in `NCConfig`)
       whenever the user Alt-Tabs away or minimizes the window.

[VI] Điều khiển Dừng Khẩn cấp, Hủy Gốc Tọa độ & An toàn Tiêu điểm Cửa sổ
     - `stop(source)`: Hủy ngay luồng tính toán quỹ đạo, xóa hàng đợi lệnh,
       gửi tức thì lệnh `"STOP <source>"` xuống mạch qua USB, và yêu cầu Home
       lại nếu robot bị dừng đột ngột khi đang chạy.
     - `focus_lost(source)`: Tự động tắt chế độ bám chuột trực tiếp (hoặc dừng
       robot nếu cấu hình không cho phép chạy ngầm) khi người dùng chuyển sang
       cửa sổ khác hoặc thu nhỏ phần mềm.
============================================================================
"""

from dataclasses import asdict
import ctypes
import json
import sys
import threading
import time
import tkinter as tk
from cartesian_nc_messages import REASONS, STOP_REASONS
from cartesian_nc_policy import background_allowed


class SafetyController:
    """
    [EN] Mixin providing emergency stop and window focus/minimize safety handlers.
    [VI] Lớp Mixin cung cấp cơ chế dừng khẩn cấp và giám sát an toàn cửa sổ giao diện.
    """

    def stop(self, source="USER"):
        """
        [EN] Halt all software and firmware motion immediately and log the stop context.
             If the robot was actively moving (`was_busy`), the step-counter reference
             is invalidated so the operator must re-home before the next move.
        [VI] Dừng ngay lập tức mọi chuyển động trên phần mềm và dưới mạch, đồng thời ghi log.
             Nếu robot đang di chuyển khi bị dừng (`was_busy`), gốc tọa độ sẽ bị hủy hiệu lực
             để bắt buộc người vận hành phải chạy lại HOME + CALIB trước khi chạy tiếp.
        """
        self.cancel_planning()
        self.clear_automation()
        self.active_purpose = None

        was_busy = self.active_job is not None or (
            self.state is not None and self.state.busy
        )

        # Gather diagnostic snapshot for the stop event log
        # Thu thập thông tin chẩn đoán tại thời điểm nhấn DỪNG để ghi vào nhật ký
        now = time.monotonic()
        elapsed_s = round(now - self.opened, 3) if self.opened else 0
        data_age_s = round(now - self.rx, 3) if self.rx else None
        if self.state is not None:
            firmware_reason = self.state.reason
            input_axis = self.state.input_axis
            input_bits = self.state.input_bits
            status_dict = asdict(self.state)
        else:
            firmware_reason = None
            input_axis = None
            input_bits = None
            status_dict = None

        context = {
            "type": "stop_sent",
            "source": source,
            "demo": self.demo,
            "elapsed_s": elapsed_s,
            "task": self.task_kind,
            "job": self.active_job,
            "firmware_reason": firmware_reason,
            "input_axis": input_axis,
            "input_bits": input_bits,
            "data_age_s": data_age_s,
        }

        self.commands = []
        self.pending = None
        self.active_job = None
        self.expect_epoch = None
        self.task_kind = None

        if self.link:
            self.trace_event(
                "stop_sent",
                context=context,
                status=status_dict,
            )
            self.trace_event("tx", command="STOP " + source)
            try:
                self.link.send("STOP " + source)
            except Exception:
                pass

            if not self.demo:
                path = self.event_log_path
                payload = json.dumps(context, ensure_ascii=False) + "\n"

                def _write_stop():
                    try:
                        with path.open("a", encoding="utf-8") as f:
                            f.write(payload)
                    except OSError:
                        pass

                threading.Thread(
                    target=_write_stop, daemon=True, name="SCARA stop log"
                ).start()

        if was_busy:
            self.local_reference = False
            self.model = None

        reason_code = STOP_REASONS.get(source, "stopped")
        stop_msg = REASONS.get(reason_code, "Đã gửi DỪNG")
        suffix = "; cần HOME + CALIB lại." if was_busy else "."
        self.message.set(stop_msg + suffix)
        self.render()

    def _foreground(self):
        """
        [EN] Check whether this application window (or one of its child dialogs)
             currently owns the OS foreground focus.
             On Windows, `GetAncestor(..., 3)` (`GA_ROOTOWNER`) resolves child
             Toplevel windows (such as the Calibration Wizard) back to the main
             root window so opening a dialog does not trigger a false focus-loss stop.
        [VI] Kiểm tra xem cửa sổ phần mềm (hoặc hộp thoại con của nó) có đang được
             người dùng chọn hiển thị trên cùng (foreground) hay không.
             Trên Windows, hàm `GetAncestor(..., 3)` (`GA_ROOTOWNER`) giúp nhận diện
             cửa sổ con (như Hộp thoại Hiệu chuẩn) thuộc cùng ứng dụng chính để không
             gây dừng nhầm khi mở hộp thoại.
        """
        if sys.platform == "win32":
            try:
                user32 = ctypes.windll.user32
                ancestor = user32.GetAncestor
                ancestor.argtypes = (ctypes.c_void_p, ctypes.c_uint)
                ancestor.restype = ctypes.c_void_p
                foreground = user32.GetForegroundWindow
                foreground.restype = ctypes.c_void_p

                # 2 = GA_ROOT, 3 = GA_ROOTOWNER
                main = ancestor(self.winfo_id(), 2)
                active = foreground()
                return bool(
                    main and active and ancestor(main, 3) == ancestor(active, 3)
                )
            except (OSError, AttributeError):
                pass
        return self.focus_displayof() is not None

    def _active_task(self):
        """
        [EN] Return True if any motion job, command queue, planner, or follow mode is active.
        [VI] Trả về True nếu đang có lệnh chạy, đang tính quỹ đạo hoặc đang bật chế độ bám.
        """
        firmware_busy = bool(self.state and self.state.busy)
        has_queued_cmds = bool(self.pending or self.commands)
        has_program = bool(self.program_points)
        return (
            self.active_job is not None
            or firmware_busy
            or has_queued_cmds
            or self.planning
            or has_program
            or self.follow_enabled.get()
        )

    def _home_background(self):
        """
        [EN] Return True if the active task is `INIT` (homing) and background homing is enabled.
        [VI] Trả về True nếu đang chạy lệnh dò gốc `INIT` và cấu hình cho phép chạy Home ngầm.
        """
        return self.task_kind == "INIT" and self.config.home_in_background

    def background_allowed(self):
        """
        [EN] Return True if the current task is permitted to continue when the window loses focus.
        [VI] Trả về True nếu tác vụ hiện tại được phép tiếp tục chạy khi cửa sổ mất tiêu điểm.
        """
        return background_allowed(self.task_kind, self.config)

    def disarm_follow(self):
        """
        [EN] Disarm interactive cursor-follow mode when focus is lost, allowing the
             currently executing short move to finish cleanly without losing calibration.
        [VI] Tự động tắt chế độ bám con trỏ khi chuyển cửa sổ, vẫn cho phép đoạn ngắn
             đang chạy hoàn tất êm ái mà không làm mất gốc tọa độ đã hiệu chuẩn.
        """
        if self.follow_enabled.get() or self.follow_target is not None:
            self.follow_enabled.set(False)
            self.follow_target = None
            self.follow_revision += 1
            self.trace_event(
                "follow_disarmed", reason="focus_lost", job=self.active_job
            )
            self.message.set(
                "Đã tắt bám khi chuyển app; đoạn đang chạy sẽ hoàn thành, giữ mốc calib."
            )

    def focus_lost(self, source):
        """
        [EN] Handle window focus loss or minimization according to `background_allowed()`.
        [VI] Xử lý sự kiện mất tiêu điểm hoặc thu nhỏ cửa sổ theo quy tắc `background_allowed()`.
        """
        if not self._active_task():
            return
        if not self.background_allowed():
            self.stop(source)
            return
        self.disarm_follow()

    def _focus_check(self):
        """
        [EN] Deferred Tkinter `<FocusOut>` callback verifying whether the app lost OS foreground focus.
        [VI] Hàm kiểm tra sự kiện `<FocusOut>` xem người dùng có thực sự chuyển sang ứng dụng khác không.
        """
        try:
            if not self._foreground():
                self.focus_lost("FOCUS")
        except tk.TclError:
            pass

    def _unmap_check(self):
        """
        [EN] Tkinter `<Unmap>` callback invoked when the main window is minimized.
        [VI] Hàm xử lý sự kiện `<Unmap>` khi cửa sổ chính bị thu nhỏ xuống thanh tác vụ.
        """
        self.focus_lost("MINIMIZE")
