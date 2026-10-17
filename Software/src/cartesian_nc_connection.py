"""
================================================================================
FILE: cartesian_nc_connection.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Manages the USB Serial lifecycle, JSONL session logging, and acknowledged
     command sequencing (`ConnectionController` mixin for `NCApp`).
     Processes incoming `ack` and `status` events from the serial thread on the
     Tkinter main thread and enforces watchdog liveness (`KEEP` heartbeats and
     ACK/status timeouts).

[VI] MỤC ĐÍCH:
     Quản lý vòng đời kết nối USB Serial, ghi nhật ký phiên định dạng JSONL và
     điều phối chuỗi lệnh có xác nhận (`ConnectionController` mixin cho `NCApp`).
     Xử lý các sự kiện `ack` và `status` từ luồng serial trên luồng chính Tkinter,
     đồng thời duy trì cơ chế giám sát kết nối sống (`KEEP` heartbeat và kiểm tra
     quá hạn ACK/status).

[DEBUG TIP / MẸO GỠ LỖI]:
     - Every sent command (`tx`), received ACK (`ack`), status snapshot (`status`),
       and rejected raw line (`rx_rejected`) is recorded by `SessionLog` in the
       `Software/logs/` folder. Open this folder via the "Mở thư mục log" button
       when debugging communication dropouts.
     - Mọi lệnh gửi đi (`tx`), phản hồi (`ack`), trạng thái (`status`) và dòng dữ liệu
       lỗi (`rx_rejected`) đều được `SessionLog` ghi lại trong thư mục `Software/logs/`.
       Hãy mở thư mục này bằng nút "Mở thư mục log" khi cần gỡ lỗi mất kết nối.
================================================================================
"""

from dataclasses import asdict
import queue
import sys
import time
from cartesian_nc_protocol import BUILD
from cartesian_nc_log import SessionLog
from cartesian_nc_messages import GUI_VERSION, REASONS
from cartesian_nc_policy import (
    status_fresh,
    status_expired,
    hardware_ready,
    ACK_TIMEOUT_S,
    KEEP_INTERVAL_S,
    MAX_EVENTS_PER_POLL,
)


class ConnectionController:
    """
    [EN] Mixin class providing USB connection management, command queue sequencing,
         event draining, and watchdog maintenance for `NCApp`.
    [VI] Lớp Mixin cung cấp chức năng quản lý kết nối USB, xếp hàng chuỗi lệnh,
         xử lý sự kiện nhận về và duy trì watchdog an toàn cho `NCApp`.
    """

    def init_runtime_state(self, config=None, config_path=None, event_log_path=None):
        """
        [EN] Initialize or reset all runtime connection, job, and planning attributes.
        [VI] Khởi tạo hoặc đặt lại toàn bộ các thuộc tính kết nối, công việc và quy hoạch.
        """
        if config is not None:
            self.config = config
        if config_path is not None:
            self.config_path = config_path
        if event_log_path is not None:
            self.event_log_path = event_log_path

        # Session logging and serial link state
        # Trạng thái ghi nhật ký phiên và kết nối serial
        self.trace = None
        self.last_log_path = None
        self.link = None
        self.state = None
        self.rx = 0
        self.opened = 0

        # Command sequencing and active job tracking
        # Theo dõi chuỗi lệnh đang chờ và công việc (job) đang chạy
        self.job = 0
        self.active_job = None
        self.pending = None
        self.commands = []
        self.last_keep = 0
        self.started = 0
        self.expect_epoch = None
        self.task_kind = None

        # Kinematic calibration reference and background preflight planning state
        # Trạng thái mốc hiệu chuẩn động học và quy hoạch đường đi nền
        self.local_reference = False
        self.model = None
        self.preview = None
        self.planning = False
        self.plan_results = queue.Queue()
        self.plan_token = 0
        self.active_purpose = None

        # Interactive follow-mode and automated setpoint/drawing state
        # Trạng thái chế độ bám mô hình (follow) và chạy chuỗi điểm/nét vẽ tự động
        self.follow_target = None
        self.follow_revision = 0
        self.follow_due = 0
        self.program_points = ()
        self.program_index = 0
        self.program_wait = 0
        self.path_upload = None
        self.path_run = None
        self.path_since = 0

    def trace_event(self, kind, **fields):
        """
        [EN] Append a structured diagnostic event to the active JSONL session log.
        [VI] Ghi thêm một sự kiện chẩn đoán có cấu trúc vào file log JSONL hiện tại.
        """
        if self.trace:
            self.trace.record(kind, **fields)

    def open_logs(self):
        """
        [EN] Open the `logs/` directory in Windows Explorer for easy student inspection.
        [VI] Mở thư mục `logs/` trong Windows Explorer để sinh viên dễ dàng kiểm tra.
        """
        root = self.config_path.parent / "logs"
        root.mkdir(exist_ok=True)
        if sys.platform == "win32":
            import os

            os.startfile(root)

    # ==========================================================================
    # READINESS QUERIES / KIỂM TRA TRẠNG THÁI SẴN SÀNG
    # ==========================================================================

    def fresh(self):
        """
        [EN] Return True if we have an active link and a non-expired status packet.
        [VI] Trả về True nếu đang có kết nối và gói tin trạng thái chưa bị quá hạn.
        """
        return status_fresh(self.link, self.state, self.rx, time.monotonic())

    def idle(self):
        """
        [EN] Return True if the robot is connected, fresh, not busy, and has no
             pending commands or background planning tasks.
        [VI] Trả về True nếu robot đang kết nối, trạng thái mới, không bận, và không
             có lệnh chờ hay tác vụ tính toán ngầm nào.
        """
        return (
            self.fresh()
            and not self.state.busy
            and self.active_job is None
            and self.pending is None
            and not self.commands
            and not self.planning
        )

    def operable(self):
        """
        [EN] Return True if the robot is idle AND all hardware safety checks pass.
        [VI] Trả về True nếu robot đang rảnh VÀ thỏa mãn mọi điều kiện an toàn phần cứng.
        """
        return self.idle() and hardware_ready(self.state, self.config_error)

    # ==========================================================================
    # USB PORT & CONNECTION LIFECYCLE / QUẢN LÝ CỔNG COM & VÒNG ĐỜI KẾT NỐI
    # ==========================================================================

    def refresh_ports(self):
        """
        [EN] Enumerate available serial COM ports (or `["DEMO"]` in simulation mode).
        [VI] Liệt kê các cổng COM hiện có trên máy (hoặc `["DEMO"]` ở chế độ mô phỏng).
        """
        if self.demo:
            ports = ["DEMO"]
        else:
            try:
                from serial.tools import list_ports

                ports = sorted(p.device for p in list_ports.comports())
            except Exception as e:
                self.message.set(str(e))
                ports = []
        self.ports.configure(values=ports)
        if self.port.get() not in ports:
            if ports:
                self.port.set(ports[0])
            else:
                self.port.set("")

    def toggle_connection(self):
        """
        [EN] Connect to the selected COM port (creating a new JSONL session log)
             or disconnect if already connected.
        [VI] Kết nối tới cổng COM đã chọn (tạo file log JSONL mới) hoặc ngắt kết nối
             nếu đang kết nối.
        """
        if self.link:
            self.disconnect("Đã ngắt USB.")
            return
        try:
            selected_port = self.port.get()
            if not selected_port:
                raise ValueError("Chọn cổng COM.")
            self.link = self.factory(selected_port)
            self.opened = time.monotonic()
            try:
                self.trace = SessionLog(self.config_path.parent / "logs")
                self.last_log_path = self.trace.path
                self.log_text.set("Log: " + self.trace.path.name)
                self.trace_event(
                    "connection",
                    port=selected_port,
                    session=self.link.session,
                    build=BUILD,
                    gui_version=GUI_VERSION,
                    config=asdict(self.config),
                    demo=self.demo,
                    axis_order=["Z", "J1", "J2"],
                )
                self.trace_event(
                    "startup", commands=["STOP", f"HELLO {self.link.session}"]
                )
            except OSError as e:
                self.trace = None
                self.log_text.set("Không tạo được log: " + str(e))

            self.state = None
            self.rx = 0
            self.job = 0
            self.local_reference = False
            self.model = None
            self.expect_epoch = None
            self.message.set("Đang xác nhận firmware NC v5…")
        except Exception as e:
            self.message.set(str(e))
        self.render()

    def send(self, line):
        """
        [EN] Log and transmit a single ASCII command line over USB Serial.
             Automatically disconnects on USB write failure.
        [VI] Ghi log và truyền một dòng lệnh ASCII qua USB Serial.
             Tự động ngắt kết nối nếu xảy ra lỗi ghi USB.
        """
        self.trace_event("tx", command=line)
        try:
            self.link.send(line)
            return True
        except Exception as e:
            self.disconnect("Lỗi USB: " + str(e))
            return False

    def disconnect(self, message, source="DISCONNECT"):
        """
        [EN] Safely stop motion, close the serial link and session log, clear
             calibration state, and update the UI message.
        [VI] Dừng chuyển động an toàn, đóng kết nối serial và file log, xóa trạng thái
             mốc hiệu chuẩn và cập nhật thông báo trên giao diện.
        """
        self.stop(source)

        link = self.link
        self.link = None
        if link:
            link.close()

        self.trace_event("disconnect", source=source, message=message)

        trace = self.trace
        self.trace = None
        if trace:
            trace.close()
            if trace.error:
                self.log_text.set("Lỗi ghi log: " + trace.error)

        self.state = None
        self.local_reference = False
        self.model = None
        self.message.set(message)
        self.render()

    # ==========================================================================
    # COMMAND SEQUENCING & EVENT PROCESSING / ĐIỀU PHỐI LỆNH & XỬ LÝ SỰ KIỆN
    # ==========================================================================

    def sequence(self, lines):
        """
        [EN] Queue a list of ASCII commands to be sent one-by-one as each command
             is acknowledged (`Ack.ok == 1`) by the STM32.
        [VI] Xếp hàng danh sách các lệnh ASCII để gửi tuần tự từng lệnh một sau khi
             nhận được xác nhận (`Ack.ok == 1`) từ STM32.
        """
        self.commands = list(lines)
        self.next_command()

    def next_command(self):
        """
        [EN] Pop and transmit the next command in `self.commands`, recording its
             opcode and timestamp in `self.pending` for ACK timeout tracking.
        [VI] Lấy và gửi lệnh tiếp theo trong `self.commands`, lưu mã lệnh và thời điểm
             gửi vào `self.pending` để theo dõi thời gian chờ phản hồi ACK.
        """
        if not self.commands or not self.link:
            return
        line = self.commands.pop(0)
        opcode = line.split()[0]
        now = time.monotonic()
        self.pending = (opcode, now)

        # Motion/homing commands start a tracked job requiring periodic KEEP heartbeats
        # Các lệnh chuyển động/dò gốc bắt đầu một công việc cần gửi nhịp KEEP định kỳ
        if opcode in ("INIT", "MOVE", "PATH"):
            self.task_kind = opcode
            self.active_job = self.job
            self.started = now
            self.last_keep = now
        self.send(line)

    def _drain_events(self, now):
        """
        [EN] Drain up to `MAX_EVENTS_PER_POLL` events from the serial background
             thread queue without blocking the Tkinter UI loop.
        [VI] Rút tối đa `MAX_EVENTS_PER_POLL` sự kiện từ hàng đợi của luồng đọc serial
             mà không làm treo vòng lặp giao diện Tkinter.
        """
        for _ in range(MAX_EVENTS_PER_POLL):
            if self.link is None:
                break
            try:
                event = self.link.events.get_nowait()
            except queue.Empty:
                break

            event_kind = event[0]
            if event_kind == "error":
                self.disconnect("Lỗi USB: " + event[1])
                break
            if event_kind == "rx_rejected":
                self.trace_event(
                    "rx_rejected", received_monotonic=event[1], raw=event[2]
                )
                continue
            if event_kind == "ack":
                self._handle_ack(event[2], event[1])
                continue
            self._handle_status(event[2], event[1], now)

    def _handle_ack(self, ack, received):
        """
        [EN] Process a command acknowledgment (`Ack`) from STM32:
             - Route `PATH`/`SEG`/`GO` ACKs to `self.path_upload` during continuous drawing.
             - Advance `self.next_command()` on `ok == 1`, or abort and display the
               translated rejection reason on `ok == 0`.
        [VI] Xử lý gói xác nhận lệnh (`Ack`) từ STM32:
             - Chuyển các ACK của `PATH`/`SEG`/`GO` tới `self.path_upload` khi đang vẽ nét.
             - Gửi lệnh kế tiếp qua `self.next_command()` nếu `ok == 1`, hoặc hủy chuỗi lệnh
               và hiển thị lý do từ chối nếu `ok == 0`.
        """
        self.trace_event("ack", received_monotonic=received, ack=asdict(ack))
        if ack.session != self.link.session:
            return

        # 1. Special handling for streaming path segments (`PATH` -> `SEG` -> `GO`)
        # 1. Xử lý riêng cho luồng nạp quỹ đạo liên tục (`PATH` -> `SEG` -> `GO`)
        if self.path_upload and ack.op in ("PATH", "SEG", "GO"):
            try:
                if self.path_upload.ack(ack):
                    if self.path_upload.pending:
                        self.pending = (self.path_upload.pending, received)
                    else:
                        self.pending = None
                    if hasattr(self, "path_tick"):
                        self.path_tick(received)
            except ValueError as e:
                self.stop("MODEL")
                self.message.set("Đường vẽ bị từ chối: " + str(e))
            return

        # 2. Standard sequential command ACK handling
        # 2. Xử lý xác nhận cho chuỗi lệnh tuần tự thông thường
        if self.pending and ack.op == self.pending[0]:
            if (
                ack.op in ("INIT", "MOVE", "PATH")
                and self.active_job is not None
                and ack.job not in (0, self.active_job)
            ):
                return
            self.pending = None
            if not ack.ok:
                self.commands = []
                self.active_job = None
                self.local_reference = False
                self.expect_epoch = None
                self.task_kind = None
                reason_vi = REASONS.get(ack.reason, ack.reason)
                self.message.set("STM32 từ chối: " + reason_vi)
            else:
                self.next_command()
        return

    def _handle_status(self, s, received, now):
        """
        [EN] Process a validated `NCStatus` telemetry snapshot from STM32:
             - Updates `self.state`, path FIFO credits, and live drawing traces.
             - Builds the calibrated `MeasuredModel` when `INIT` completes (`epoch` matches).
             - Detects job completion or faults and advances multi-point/stroke automation.
        [VI] Xử lý bản chụp telemetry `NCStatus` hợp lệ từ STM32:
             - Cập nhật `self.state`, số ô trống FIFO đường chạy và vết vẽ trực tiếp.
             - Khởi tạo mô hình động học `MeasuredModel` khi `INIT` hoàn tất (khớp `epoch`).
             - Phát hiện khi công việc hoàn thành hoặc gặp lỗi để chuyển sang bước tự động kế tiếp.
        """
        self.trace_event("status", received_monotonic=received, status=asdict(s))
        if s.session != self.link.session:
            return
        self.state = s
        self.rx = received
        self.job = max(self.job, s.job)

        if self.path_upload:
            self.path_upload.status(s)
            if not self.pending and hasattr(self, "path_tick"):
                self.path_tick(now)

        self.record_bk(s)

        if s.fault or s.conflicts or not s.referenced:
            self.local_reference = False

        # Check if HOME + CALIB (`INIT`) has just finished for the expected epoch
        # Kiểm tra xem quá trình HOME + CALIB (`INIT`) đã hoàn tất cho epoch mong đợi chưa
        just_finished_init = (
            self.expect_epoch is not None
            and s.referenced
            and s.epoch == self.expect_epoch
            and not s.busy
        )
        if just_finished_init:
            try:
                self.model = self.config.model(s)
                self.local_reference = True
                self.expect_epoch = None
                self.current_target()
                self.message.set("HOME + CALIB xong; có thể nhập XYZ.")
            except Exception as e:
                self.stop("MODEL")
                self.message.set(str(e))

        if self.local_reference and (not s.referenced or not self.model):
            self.local_reference = False

        # Check if the currently active motion job has finished (`not s.busy`)
        # Kiểm tra xem công việc chuyển động hiện tại đã kết thúc (`not s.busy`) hay chưa
        job_finished = (
            self.active_job is not None and s.job == self.active_job and not s.busy
        )
        if job_finished:
            purpose = self.active_purpose
            self.active_purpose = None
            self.active_job = None
            self.task_kind = None
            if s.stage == 9 or s.fault:
                self.clear_automation()
                self.expect_epoch = None
                self.message.set(REASONS.get(s.reason, s.reason))
            elif s.reason == "complete":
                if purpose == "stroke" and s.referenced and self.path_run:
                    self.finish_stroke(now)
                elif purpose == "program" and s.referenced and self.program_points:
                    self.finish_program_point(now)
                else:
                    self.message.set(
                        "Đã đến đích theo số xung; vị trí thực chưa có encoder xác nhận."
                    )
            else:
                if self.bk_active:
                    self.bk_note.set(REASONS.get(s.reason, s.reason))
                self.clear_automation()
                self.message.set(REASONS.get(s.reason, s.reason))

    def _maintain_connection(self, now):
        """
        [EN] Periodic liveness watchdog called every GUI poll:
             1. Disconnects if status telemetry has expired.
             2. Sends `STOP ACK` if a pending command exceeds `ACK_TIMEOUT_S`.
             3. Sends periodic `KEEP <session> <job>` heartbeats while a job is active.
        [VI] Bộ giám sát kết nối định kỳ được gọi ở mỗi chu kỳ quét GUI:
             1. Ngắt kết nối nếu gói tin trạng thái bị quá hạn.
             2. Gửi lệnh dừng `STOP ACK` nếu lệnh đang chờ phản hồi vượt quá `ACK_TIMEOUT_S`.
             3. Gửi xung nhịp `KEEP <session> <job>` định kỳ khi đang có lệnh chạy.
        """
        if self.link:
            if status_expired(self.state, self.opened, self.rx, now):
                self.disconnect(
                    "USB/status quá hạn; đã gửi DỪNG. Kiểm tra đúng firmware NC v5 R9.",
                    "STALE",
                )
            elif self.pending and now - self.pending[1] > ACK_TIMEOUT_S:
                self.stop("ACK")
                self.message.set("Lệnh không được STM32 xác nhận; đã dừng.")
            elif (
                self.active_job is not None and now - self.last_keep >= KEEP_INTERVAL_S
            ):
                self.last_keep = now
                self.send(f"KEEP {self.link.session} {self.active_job}")
