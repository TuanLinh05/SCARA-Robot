"""SCARA XYZ application composition. No boot motion; --demo is offline."""

import argparse
import ctypes
from pathlib import Path
import queue
import sys
import time
import tkinter as tk
from cartesian_nc_model import NCConfig
from cartesian_nc_protocol import NCLink, Ack, BUILD
from cartesian_nc_editor import WorkspaceEditor
from cartesian_nc_drawing_gui import DrawingUI
from cartesian_nc_connection import ConnectionController
from cartesian_nc_motion import MotionController
from cartesian_nc_planning import PlanningController
from cartesian_nc_safety import SafetyController
from cartesian_nc_view import StudioView
from cartesian_nc_theme import BG, CARD, TEXT, MUTED, BLUE, GREEN, RED
from cartesian_nc_messages import (
    STAGES,
    PHASES,
    Z_PHASES,
    ERRORS,
    REASONS,
    STOP_REASONS,
    GUI_VERSION,
)
from cartesian_nc_policy import GUI_POLL_MS, PATH_POLL_MS

CONFIG_PATH = (
    Path(sys.executable).parent
    if getattr(sys, "frozen", False)
    else Path(__file__).resolve().parents[1]
) / "cartesian_nc_config.json"


class NCApp(
    ConnectionController,
    MotionController,
    PlanningController,
    SafetyController,
    StudioView,
    WorkspaceEditor,
    DrawingUI,
    tk.Tk,
):
    def __init__(self, factory=NCLink, config_path=CONFIG_PATH, demo=False):
        if sys.platform == "win32":
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except (OSError, AttributeError):
                pass
        super().__init__()
        self.tk.call("tk", "scaling", 96 / 72)
        self.title(
            "SCARA · MOTION & DRAWING STUDIO · GUI R14 · FW R9"
            + (" · MÔ PHỎNG" if demo else "")
        )
        self.geometry("1360x940")
        self.minsize(1160, 900)
        self.configure(bg=BG)
        self.factory, self.config_path, self.demo = factory, Path(config_path), demo
        self.config_error = None
        try:
            self.config = NCConfig.load(self.config_path)
        except FileNotFoundError:
            self.config = NCConfig().validate()
        except Exception as e:
            self.config = NCConfig().validate()
            self.config_error = str(e)
        self.link = None
        self.state = None
        self.rx = self.opened = 0
        self.job = 0
        self.active_job = None
        self.pending = None
        self.commands = []
        self.last_keep = self.started = 0
        self.expect_epoch = None
        self.task_kind = None
        self.event_log_path = self.config_path.with_name("cartesian_events.jsonl")
        self.trace = None
        self.last_log_path = None
        self.log_text = tk.StringVar(value="Log tự động: tạo file mỗi lần kết nối")
        self.local_reference = False
        self.model = None
        self.preview = None
        self.settings_window = None
        self.planning = False
        self.plan_results = queue.Queue()
        self.plan_token = 0
        self.active_purpose = None
        self.follow_target = None
        self.follow_revision = 0
        self.follow_due = 0
        self.program_points = ()
        self.program_index = 0
        self.program_wait = 0
        self.path_upload = None
        self.path_run = None
        self.path_since = 0
        self.port = tk.StringVar(value="DEMO" if demo else "")
        self.xyz = [tk.StringVar(value=v) for v in ("-69.296", "167.296", "20.0")]
        self.speed = tk.StringVar(value="5")
        self.message = tk.StringVar(
            value="Kiểm tra thông số cơ khí và sáu công tắc, rồi bấm HOME + CALIB."
        )
        self.metric = tk.StringVar(value="CHƯA LẤY MỐC")
        self.details = tk.StringVar(value="")
        self.progress = tk.StringVar(value=STAGES[0])
        self.switch_text = tk.StringVar(value="Chưa có dữ liệu công tắc")
        self.input_details = tk.StringVar(value="")
        self.init_editor()
        self.init_drawing()
        self._build()
        self.refresh_ports()
        self.render()
        self.bind_all("<Escape>", lambda e: self.stop("ESC"))
        self.bind("<FocusOut>", lambda e: self.after(10, self._focus_check))
        self.bind(
            "<Unmap>", lambda e: self._unmap_check() if e.widget is self else None
        )
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(GUI_POLL_MS, self.poll)
        if self.config_error:
            self.message.set("Cấu hình lỗi; khóa chuyển động: " + self.config_error)

    def poll(self):
        """Keep USB liveness, preflight acceptance and automation on the Tk thread."""
        now = time.monotonic()
        if self.link:
            self._drain_events(now)
        self._maintain_connection(now)
        self._poll_planning(now)
        self.automation_tick(now)
        self.path_tick(now)
        self.render()
        self.after(PATH_POLL_MS if self.path_upload else GUI_POLL_MS, self.poll)
        if self.trace and self.trace.error:
            self.log_text.set("Lỗi ghi log: " + self.trace.error)

    def close(self):
        self.disconnect("Đã đóng.", "CLOSE")
        self.destroy()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    p.add_argument("--smoke-test", action="store_true")
    args = p.parse_args()
    if args.demo or args.smoke_test:
        from cartesian_nc_demo import DemoLink

        app = NCApp(factory=DemoLink, demo=True)
    else:
        app = NCApp()
    if args.smoke_test:
        app.after(100, app.toggle_connection)
        app.after(400, app.initialize)
        app.after(3000, app.close)
    app.mainloop()


if __name__ == "__main__":
    main()
