"""Stop, reference invalidation and foreground/background policy for NCApp."""

from dataclasses import asdict
import ctypes
import json
import sys
import time
import tkinter as tk
from cartesian_nc_messages import REASONS, STOP_REASONS
from cartesian_nc_policy import background_allowed


class SafetyController:
    def stop(self, source="USER"):
        self.cancel_planning()
        self.clear_automation()
        self.active_purpose = None
        was_busy = self.active_job is not None or (
            self.state is not None and self.state.busy
        )
        context = {
            "type": "stop_sent",
            "source": source,
            "demo": self.demo,
            "elapsed_s": round(time.monotonic() - self.opened, 3) if self.opened else 0,
            "task": self.task_kind,
            "job": self.active_job,
            "firmware_reason": self.state.reason if self.state else None,
            "input_axis": self.state.input_axis if self.state else None,
            "input_bits": self.state.input_bits if self.state else None,
            "data_age_s": round(time.monotonic() - self.rx, 3) if self.rx else None,
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
                status=asdict(self.state) if self.state else None,
            )
            self.trace_event("tx", command="STOP " + source)
            try:
                self.link.send("STOP " + source)
            except Exception:
                pass
            if not self.demo:
                try:
                    with self.event_log_path.open("a", encoding="utf-8") as f:
                        f.write(json.dumps(context, ensure_ascii=False) + "\n")
                except OSError:
                    pass
        if was_busy:
            self.local_reference = False
            self.model = None
        self.message.set(
            REASONS.get(STOP_REASONS.get(source, "stopped"), "Đã gửi DỪNG")
            + ("; cần HOME + CALIB lại." if was_busy else ".")
        )
        self.render()

    def _foreground(self):
        if sys.platform == "win32":
            try:
                user32 = ctypes.windll.user32
                ancestor = user32.GetAncestor
                ancestor.argtypes = (ctypes.c_void_p, ctypes.c_uint)
                ancestor.restype = ctypes.c_void_p
                foreground = user32.GetForegroundWindow
                foreground.restype = ctypes.c_void_p
                main = ancestor(self.winfo_id(), 2)
                active = foreground()
                return bool(
                    main and active and ancestor(main, 3) == ancestor(active, 3)
                )
            except (OSError, AttributeError):
                pass
        return self.focus_displayof() is not None

    def _active_task(self):
        return (
            self.active_job is not None
            or bool(self.state and self.state.busy)
            or bool(self.pending or self.commands)
            or self.planning
            or bool(self.program_points)
            or self.follow_enabled.get()
        )

    def _home_background(self):
        return self.task_kind == "INIT" and self.config.home_in_background

    def background_allowed(self):
        return background_allowed(self.task_kind, self.config)

    def disarm_follow(self):
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
        if not self._active_task():
            return
        if not self.background_allowed():
            self.stop(source)
            return
        self.disarm_follow()

    def _focus_check(self):
        try:
            if not self._foreground():
                self.focus_lost("FOCUS")
        except tk.TclError:
            pass

    def _unmap_check(self):
        self.focus_lost("MINIMIZE")
