"""
================================================================================
FILE: cartesian_nc_control.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Top-level application entry point and class composition (`NCApp`).
     Combines all specialized controller and UI mixins (`ConnectionController`,
     `MotionController`, `PlanningController`, `SafetyController`, `StudioView`,
     `WorkspaceEditor`, `DrawingUI`) into a single Tkinter `tk.Tk` root window.
     No motion occurs on boot; `--demo` launches an offline simulated robot.

[VI] MỤC ĐÍCH:
     Điểm khởi chạy ứng dụng cấp cao nhất và hợp thành lớp điều khiển chính (`NCApp`).
     Kết hợp toàn bộ các lớp Mixin chuyên biệt (`ConnectionController`,
     `MotionController`, `PlanningController`, `SafetyController`, `StudioView`,
     `WorkspaceEditor`, `DrawingUI`) vào cửa sổ gốc Tkinter `tk.Tk`.
     Không tự động chạy động cơ khi khởi động; cờ `--demo` chạy chế độ mô phỏng.

[DEBUG TIP / MẸO GỠ LỖI]:
     - Run `python Software/src/cartesian_nc_control.py --demo` to test the full
       GUI, kinematics, and drawing compiler without physical STM32 hardware!
     - Chạy lệnh `python Software/src/cartesian_nc_control.py --demo` để kiểm thử
       toàn bộ giao diện, động học và bộ biên dịch nét vẽ mà không cần mạch STM32!
================================================================================
"""

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
from cartesian_nc_3d import Viewer3DMixin
from cartesian_nc_analytics import AnalyticsMixin
from cartesian_nc_calibration import CalibrationWizardMixin
from cartesian_nc_connection import ConnectionController
from cartesian_nc_motion import MotionController
from cartesian_nc_policy import GUI_POLL_MS, PATH_POLL_MS
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

# [EN] Default path to `cartesian_nc_config.json` (supports PyInstaller frozen builds).
#      When packaged as a `.exe` (`sys.frozen`), look next to the executable;
#      otherwise look in the `Software/` parent folder above `src/`.
# [VI] Đường dẫn mặc định tới file cấu hình `cartesian_nc_config.json` (hỗ trợ cả bản `.exe`).
#      Khi đóng gói `.exe` (`sys.frozen`), tìm cạnh file thực thi;
#      ngược lại tìm trong thư mục cha `Software/` phía trên `src/`.
if getattr(sys, "frozen", False):
    _BASE_DIR = Path(sys.executable).parent
else:
    _BASE_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = _BASE_DIR / "cartesian_nc_config.json"


# ==============================================================================
# 1. MAIN APPLICATION CLASS / LỚP ỨNG DỤNG CHÍNH (NCApp)
# ==============================================================================


class NCApp(
    ConnectionController,
    MotionController,
    PlanningController,
    SafetyController,
    StudioView,
    WorkspaceEditor,
    DrawingUI,
    Viewer3DMixin,
    AnalyticsMixin,
    CalibrationWizardMixin,
    tk.Tk,
):
    """
    [EN] Main Tkinter application window integrating serial communication,
         kinematics, safety gates, interactive 2D/3D workspace editor,
         real-time telemetry analytics, and drawing studio.
         Why multiple inheritance (Mixin architecture)?
         - Splitting a large application into focused Mixin classes keeps each
           module short and easy to study while sharing `self` on the main
           Tkinter thread without complex callback wiring.
    [VI] Cửa sổ ứng dụng Tkinter chính tích hợp truyền thông serial, động học,
         cổng an toàn, trình biên tập không gian 2D/3D, phân tích dữ liệu
         thời gian thực và studio vẽ hình.
         Tại sao dùng đa kế thừa (kiến trúc Mixin)?
         - Chia nhỏ ứng dụng lớn thành các lớp Mixin theo chức năng giúp từng file
           ngắn gọn, dễ đọc cho sinh viên mà vẫn dùng chung đối tượng `self` trên
           luồng chính Tkinter.
    """

    def __init__(self, factory=NCLink, config_path=CONFIG_PATH, demo=False):
        # ----------------------------------------------------------------------
        # [EN] STEP 1: Enable high-DPI crisp rendering on Windows and init Tk.
        # [VI] BƯỚC 1: Bật hiển thị sắc nét độ phân giải cao (High-DPI) trên Windows.
        # ----------------------------------------------------------------------
        if sys.platform == "win32":
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except (OSError, AttributeError):
                pass

        super().__init__()
        self.tk.call("tk", "scaling", 96 / 72)

        demo_suffix = " · MÔ PHỎNG" if demo else ""
        self.title(
            "SCARA · MOTION & DIGITAL TWIN STUDIO · GUI R15 · FW R9" + demo_suffix
        )
        self.geometry("1360x940")
        self.minsize(1160, 900)
        self.configure(bg=BG)

        # ----------------------------------------------------------------------
        # [EN] STEP 2: Load and validate mechanical configuration from JSON.
        # [VI] BƯỚC 2: Tải và kiểm tra thông số cấu hình cơ khí từ file JSON.
        # ----------------------------------------------------------------------
        self.factory = factory
        self.config_path = Path(config_path)
        self.demo = demo
        self.config_error = None
        try:
            self.config = NCConfig.load(self.config_path)
        except FileNotFoundError:
            self.config = NCConfig().validate()
        except Exception as e:
            self.config = NCConfig().validate()
            self.config_error = str(e)

        # [EN] Initialize connection/ACK queues, safety state, and JSONL event logger.
        # [VI] Khởi tạo hàng đợi kết nối/ACK, trạng thái an toàn và bộ ghi log JSONL.
        event_log_path = self.config_path.with_name("cartesian_events.jsonl")
        self.init_runtime_state(self.config, self.config_path, event_log_path)

        # ----------------------------------------------------------------------
        # [EN] STEP 3: Initialize primary Tkinter observable variables (`StringVar`).
        # [VI] BƯỚC 3: Khởi tạo các biến trạng thái giao diện Tkinter (`StringVar`).
        # ----------------------------------------------------------------------
        self.log_text = tk.StringVar(value="Log tự động: tạo file mỗi lần kết nối")
        self.settings_window = None
        self.port = tk.StringVar(value="DEMO" if demo else "")

        # [EN] Default Cartesian target entries: X = -69.296 mm, Y = 167.296 mm, Z = 20.0 mm.
        # [VI] Tọa độ đích Đề-các mặc định trên ô nhập: X = -69.296 mm, Y = 167.296 mm, Z = 20.0 mm.
        self.xyz = [
            tk.StringVar(value="-69.296"),
            tk.StringVar(value="167.296"),
            tk.StringVar(value="20.0"),
        ]
        self.speed = tk.StringVar(value="8")
        self.message = tk.StringVar(
            value="Kiểm tra thông số cơ khí và sáu công tắc, rồi bấm HOME + CALIB."
        )
        self.metric = tk.StringVar(value="CHƯA LẤY MỐC")
        self.details = tk.StringVar(value="")
        self.progress = tk.StringVar(value=STAGES[0])
        self.switch_text = tk.StringVar(value="Chưa có dữ liệu công tắc")
        self.input_details = tk.StringVar(value="")

        # ----------------------------------------------------------------------
        # [EN] STEP 4: Initialize sub-modules and build UI layout.
        # [VI] BƯỚC 4: Khởi tạo các phân hệ biên tập, vẽ hình, 3D, phân tích và dựng UI.
        # ----------------------------------------------------------------------
        self.init_editor()
        self.init_drawing()
        self.init_3d_viewer()
        self.init_analytics()
        self._build()
        self.refresh_ports()
        self.render()

        # ----------------------------------------------------------------------
        # [EN] STEP 5: Bind global safety shortcuts and window lifecycle events.
        #      - Pressing `Escape` anywhere triggers immediate emergency stop.
        #      - Losing window focus or minimizing stops motion unless background
        #        execution is explicitly allowed.
        # [VI] BƯỚC 5: Gán phím tắt dừng khẩn cấp (`Esc`) và sự kiện vòng đời cửa sổ.
        #      - Bấm phím `Escape` ở bất kỳ đâu sẽ kích hoạt dừng khẩn cấp ngay lập tức.
        #      - Mất tiêu điểm hoặc thu nhỏ cửa sổ sẽ dừng máy trừ khi bật cho phép chạy nền.
        # ----------------------------------------------------------------------
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
        """
        [EN] Periodic main-thread event loop (`GUI_POLL_MS` = 40ms normally, or
             `PATH_POLL_MS` = 15ms while streaming continuous stroke segments).
             Keeps USB liveness, preflight acceptance, and automation on the Tk thread.
        [VI] Vòng lặp quét sự kiện định kỳ trên luồng chính (`GUI_POLL_MS` = 40ms
             thông thường, hoặc `PATH_POLL_MS` = 15ms khi đang nạp nét vẽ liên tục).
             Giữ toàn bộ xử lý USB, nghiệm thu quỹ đạo và tự động hóa trên luồng Tk.
        """
        now = time.monotonic()

        # [EN] 1. Drain incoming `status` and `ack` events from the USB reader thread.
        # [VI] 1. Xử lý hết các gói `status` và `ack` nhận về từ luồng đọc USB.
        if self.link:
            self._drain_events(now)

        # [EN] 2. Check status/ACK timeouts and send `KEEP` heartbeat if needed.
        # [VI] 2. Kiểm tra quá hạn status/ACK và gửi gói giữ kết nối `KEEP` nếu cần.
        self._maintain_connection(now)

        # [EN] 3. Accept completed background trajectory calculations.
        # [VI] 3. Nghiệm thu kết quả tính toán quỹ đạo đã hoàn tất từ luồng ngầm.
        self._poll_planning(now)

        # [EN] 4. Step multi-point programs, drawing strokes, or follow-mode targets.
        # [VI] 4. Cập nhật tiến trình chạy chuỗi điểm, nét vẽ hoặc chế độ bám mô hình.
        self.automation_tick(now)

        # [EN] 5. Stream `SEG` and `GO` commands if a continuous stroke is active.
        # [VI] 5. Bơm tiếp các lệnh `SEG` và `GO` nếu đang truyền nét vẽ liên tục.
        self.path_tick(now)

        # [EN] 6. Redraw the UI and schedule the next poll tick.
        # [VI] 6. Vẽ lại giao diện và hẹn giờ cho chu kỳ quét tiếp theo.
        self.render()
        next_delay_ms = PATH_POLL_MS if self.path_upload else GUI_POLL_MS
        self.after(next_delay_ms, self.poll)

        if self.trace and self.trace.error:
            self.log_text.set("Lỗi ghi log: " + self.trace.error)

    def close(self):
        """
        [EN] Cleanly disconnect from USB (sending `STOP CLOSE`) and destroy window.
        [VI] Ngắt kết nối USB an toàn (gửi lệnh `STOP CLOSE`) và đóng cửa sổ.
        """
        self.disconnect("Đã đóng.", "CLOSE")
        self.destroy()


# ==============================================================================
# 2. CLI ENTRY POINT / HÀM KHỞI CHẠY DÒNG LỆNH
# ==============================================================================


def main():
    """
    [EN] Parse command-line flags (`--demo`, `--smoke-test`) and launch `NCApp`.
    [VI] Phân tích tham số dòng lệnh (`--demo`, `--smoke-test`) và khởi chạy `NCApp`.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    args = parser.parse_args()

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
