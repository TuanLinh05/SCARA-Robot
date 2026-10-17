"""
================================================================================
FILE: cartesian_nc_demo.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Offline simulated serial transport (`DemoLink`) for running the GUI without
     physical STM32 hardware (`--demo` and `--smoke-test` modes).
     Emulates the Protocol v5 state machine (`HELLO`, `STOP`, `INIT`, `MOVE`,
     `PATH`, `SEG`, `GO`, `KEEP`, `HOLD`, `POL`, etc.) and emits realistic
     `NCStatus` and `Ack` events every 80ms on a background daemon thread.
     (Note: Real calibration and ISR timing correctness are tested by C harnesses.)

[VI] MỤC ĐÍCH:
     Lớp giả lập kết nối serial ngoại tuyến (`DemoLink`) giúp chạy thử giao diện GUI
     mà không cần cắm mạch STM32 thật (chế độ `--demo` và `--smoke-test`).
     Mô phỏng máy trạng thái Giao thức v5 (`HELLO`, `STOP`, `INIT`, `MOVE`,
     `PATH`, `SEG`, `GO`, `KEEP`, `HOLD`, `POL`, v.v.) và phát các sự kiện
     `NCStatus` cùng `Ack` mỗi 80ms từ một luồng chạy ngầm.
     (Lưu ý: Độ chính xác hiệu chuẩn và ngắt ISR thực tế được kiểm thử bằng bộ test C.)

[DEBUG TIP / MẸO GỠ LỖI]:
     - In `DemoLink`, `INIT` takes 1.3 seconds and `MOVE` takes 0.5 seconds.
       When testing new GUI features offline, you can inspect how `DemoLink.send()`
       parses each ASCII command and transitions `self.stage` and `self.reason`.
     - Trong `DemoLink`, lệnh `INIT` mất 1.3 giây và `MOVE` mất 0.5 giây.
       Khi thử nghiệm tính năng giao diện mới ở chế độ ngoại tuyến, bạn có thể xem
       hàm `DemoLink.send()` để hiểu cách từng lệnh ASCII làm thay đổi `self.stage`
       và `self.reason`.
================================================================================
"""

from dataclasses import replace
import queue
import threading
import time
from cartesian_nc_protocol import NCStatus, Ack, BUILD


class DemoLink:
    """
    [EN] Drop-in mock replacement for `NCLink` that simulates an STM32 running
         `SCARA_CARTESIAN_NC_V5` (`BUILD = "SCARA_CART_NC_HOME_V5_R9"`).
    [VI] Lớp giả lập thay thế tương thích hoàn toàn cho `NCLink`, mô phỏng vi điều khiển
         STM32 đang chạy firmware `SCARA_CARTESIAN_NC_V5` (`BUILD = "SCARA_CART_NC_HOME_V5_R9"`).
    """

    def __init__(self, port):
        # [EN] Simulated protocol session, event queue, and motion state variables.
        # [VI] Các biến mô phỏng phiên giao thức, hàng đợi sự kiện và trạng thái chuyển động.
        self.session = 7
        self.events = queue.Queue()
        self.closed = False
        self.job = 0
        self.epoch = 0
        self.stage = 0
        self.busy = False
        self.referenced = False

        # [EN] Bitmasks: `holding = 7` (0b111: Z, J1, J2 holding torque enabled),
        #      `pol = 3` (0b011: Z and J1 limit switch polarity active-high).
        # [VI] Mặt nạ bit: `holding = 7` (0b111: cả 3 trục Z, J1, J2 đang bật lực giữ),
        #      `pol = 3` (0b011: cực tính công tắc trục Z và J1 ở mức cao).
        self.holding = 7
        self.pol = 3
        self.pos = (20000, 0, 0)
        self.goal = self.pos
        self.phase = (0, 0, 0)
        self.since = time.monotonic()
        self.ends = 0
        self.initializing = False
        self.poll_due = 0

        # [EN] Default simulated calibration factors (Q10 fixed-point) and pulse spans.
        # [VI] Hệ số hiệu chuẩn giả lập mặc định (dấu phẩy tĩnh Q10) và dải xung toàn hành trình.
        self.factor = (1638400, 27307, 40960)
        self.range = (40000, 4800, 7200)
        self.n1_span = (40000, 4804, 7206)
        self.n2_span = (40000, 4796, 7194)
        self.backlash = (0, 0)
        self.comp_pulses = (0, 0)
        self.reason = "unreferenced"

        # [EN] Simulated 32-slot hardware path FIFO state.
        # [VI] Trạng thái bộ đệm đường chạy FIFO 32 ô giả lập.
        self.path_active = False
        self.path_started = False
        self.path_queue = []
        self.path_total = 0
        self.path_received = 0
        self.path_done = 0
        self._lock = threading.RLock()

        self.emit()
        self.closed_event = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        """
        [EN] Background loop advancing simulated motion and emitting status every 80ms.
        [VI] Vòng lặp chạy ngầm cập nhật chuyển động mô phỏng và phát gói status mỗi 80ms.
        """
        while not self.closed_event.wait(0.08):
            self.advance()

    def emit(self):
        """
        [EN] Construct and enqueue a simulated `NCStatus` snapshot into `self.events`.
        [VI] Tạo và đẩy một bản chụp trạng thái `NCStatus` mô phỏng vào hàng đợi `self.events`.
        """
        with self._lock:
            active_or_ref = self.referenced or self.busy
            if self.initializing:
                mode_code = 2
            elif self.busy:
                mode_code = 1
            else:
                mode_code = 0

            up_ms_val = int((time.monotonic() - self.since) * 1000)
            free_fifo_slots = 32 - len(self.path_queue)

            v = NCStatus(
                session=self.session,
                job=self.job,
                up_ms=up_ms_val,
                busy=int(self.busy),
                referenced=int(self.referenced),
                epoch=self.epoch,
                stage=self.stage,
                pos=self.pos,
                goal=self.goal,
                total=(80000, 7200, 28800),
                range=self.range if active_or_ref else (0, 0, 0),
                factor=self.factor if active_or_ref else (0, 0, 0),
                n1=self.n1_span if self.referenced else (0, 0, 0),
                n2=self.n2_span if self.referenced else (0, 0, 0),
                phase=self.phase,
                cal_error=(0, 0, 0),
                failed_phase=(0, 0, 0),
                switches=0,
                ready=7,
                holding=self.holding,
                pol=self.pol,
                conflicts=0,
                tick=0,
                ticks=0,
                motor_error=0,
                fault=0,
                timer_late=0,
                timer_errno=0,
                raw=0,
                errors=0,
                input_wait=0,
                recovered=0,
                input_axis=-1,
                input_kind=0,
                input_bits=0,
                input_at=0,
                brief=(0, 0, 0),
                idle_ignored=0,
                gate_wait=0,
                gate_axis=0,
                false_hits=(0, 0, 0),
                mode=mode_code,
                selected=0,
                pps=800 if self.busy else 0,
                direction=(0, 0, 0),
                dir_levels=0,
                coupling_ppm=333333,
                beta_q=524282,
                beta_measured=524288 if self.referenced else 0,
                probe_da=128 if self.referenced else 0,
                probe_db=64 if self.referenced else 0,
                coupling_ready=int(self.referenced),
                span_md=(180000, 180000) if active_or_ref else (0, 0),
                comp_pulses=self.comp_pulses,
                reason=self.reason,
                build=BUILD,
            )
            v = replace(
                v,
                path=(
                    self.path_total,
                    self.path_received,
                    self.path_done,
                    free_fifo_slots,
                ),
            )
            self.events.put(("status", time.monotonic(), v))

    def send(self, line):
        """
        [EN] Parse an incoming ASCII command from the GUI, update simulated state,
             and enqueue an immediate `Ack(..., ok=1, reason="ok")`.
        [VI] Phân tích dòng lệnh ASCII gửi từ GUI, cập nhật trạng thái mô phỏng
             và đẩy ngay gói xác nhận `Ack(..., ok=1, reason="ok")` vào hàng đợi.
        """
        with self._lock:
            if self.closed:
                raise OSError("Demo đóng.")

            w = line.split()
            op = w[0]

            # [EN] 1. Handshake: `HELLO <session>` sets the active session ID.
            # [VI] 1. Bắt tay: `HELLO <session>` thiết lập mã phiên làm việc.
            if op == "HELLO":
                self.session = int(w[1])

            # [EN] 2. Emergency/Controlled Stop: `STOP [REASON]` halts motion immediately.
            # [VI] 2. Dừng khẩn cấp/chủ động: `STOP [REASON]` dừng mọi chuyển động ngay lập tức.
            if op == "STOP":
                if self.busy:
                    self.referenced = False
                    if self.initializing:
                        self.stage = 9
                self.busy = False
                self.initializing = False
                self.path_active = False
                self.path_started = False
                self.path_queue = []
                names = {
                    "USER": "stop_user",
                    "ESC": "stop_escape",
                    "FOCUS": "stop_focus",
                    "MINIMIZE": "stop_minimize",
                    "DISCONNECT": "stop_disconnect",
                    "CLOSE": "stop_close",
                    "STALE": "stop_stale",
                    "ACK": "stop_ack_timeout",
                    "MODEL": "stop_model",
                }
                if len(w) > 1:
                    self.reason = names[w[1]]
                elif not self.reason.startswith("stop_"):
                    self.reason = "stopped"

            # [EN] 3. Mechanical parameter updates invalidate the current reference.
            # [VI] 3. Thay đổi thông số cơ khí sẽ làm mất hiệu lực mốc chuẩn hiện tại.
            if op in ("GEOM", "SPAN", "PARK", "TUNE", "POL", "COUPLE"):
                self.referenced = False

            if op == "BACKLASH" and len(w) == 4:
                self.backlash = (int(w[2]), int(w[3]))

            # [EN] 4. Bitwise polarity update: set (`| bit`) or clear (`& ~bit`) axis bit.
            # [VI] 4. Cập nhật bit cực tính công tắc: bật (`| bit`) hoặc tắt (`& ~bit`) bit trục.
            if op == "POL":
                axis_index = ("Z", "J1", "J2").index(w[2])
                bit = 1 << axis_index
                if int(w[3]):
                    self.pol = self.pol | bit
                else:
                    self.pol = self.pol & ~bit

            # [EN] 5. Bitwise holding torque update on J1 (bit 1) or J2 (bit 2).
            # [VI] 5. Cập nhật bit lực giữ động cơ bước trên trục J1 (bit 1) hoặc J2 (bit 2).
            if op == "HOLD":
                axis_bit_index = 1 if w[2] == "J1" else 2
                bit = 1 << axis_bit_index
                if int(w[3]):
                    self.holding = self.holding | bit
                else:
                    self.holding = self.holding & ~bit
                self.referenced = False

            # [EN] 6. Homing & Calibration (`INIT <session> <job> [QUICK]`):
            #         Full homing takes 1.3s; Quick Re-Home takes 0.45s.
            # [VI] 6. Lấy gốc & Hiệu chuẩn (`INIT <session> <job> [QUICK]`):
            #         Lấy gốc đầy đủ mất 1.3 giây; Lấy gốc nhanh mất 0.45 giây.
            if op == "INIT":
                self.job = int(w[2])
                self.initializing = True
                self.busy = True
                self.referenced = False
                self.stage = 1
                self.reason = "initializing"
                is_quick = len(w) >= 4 and w[3] == "QUICK"
                duration_s = 0.45 if is_quick else 1.3
                self.ends = time.monotonic() + duration_s

            # [EN] 7. Single-point DDA move (`MOVE <session> <job> <z> <j1> <j2> <rate>`).
            # [VI] 7. Di chuyển DDA đơn điểm (`MOVE <session> <job> <z> <j1> <j2> <rate>`).
            if op == "MOVE":
                self.job = int(w[2])
                self.goal = (int(w[3]), int(w[4]), int(w[5]))
                self.comp_pulses = (
                    self.comp_pulses[0] + self.backlash[0],
                    self.comp_pulses[1] + self.backlash[1],
                )
                self.busy = True
                self.initializing = False
                self.ends = time.monotonic() + 0.5
                self.reason = "moving"

            # [EN] 8. Continuous stroke FIFO commands (`PATH`, `SEG`, `GO`).
            # [VI] 8. Nhóm lệnh truyền nét vẽ liên tục qua FIFO (`PATH`, `SEG`, `GO`).
            if op == "PATH":
                self.job = int(w[2])
                self.path_total = int(w[3])
                self.path_received = 0
                self.path_done = 0
                self.path_queue = []
                self.path_active = True
                self.busy = True
                self.path_started = False
                self.reason = "path_loading"

            if op == "SEG":
                assert (
                    self.path_active
                    and int(w[3]) == self.path_received
                    and len(self.path_queue) < 32
                )
                segment_steps = (int(w[4]), int(w[5]), int(w[6]))
                self.path_queue.append(segment_steps)
                self.path_received += 1

            if op == "GO":
                self.path_started = True
                self.reason = "path_running"

            # [EN] 9. Heartbeat (`KEEP <session>`): advance simulation without extra ACK.
            # [VI] 9. Xung giữ kết nối (`KEEP <session>`): cập nhật mô phỏng, không cần ACK.
            if op == "KEEP":
                self.advance()
                return

            self.events.put(
                ("ack", time.monotonic(), Ack(self.session, self.job, op, 1, "ok"))
            )
            self.emit()

    def advance(self):
        """
        [EN] Step the simulated robot state forward in time (draining one `PATH`
             segment per tick or completing `INIT`/`MOVE` when `now >= self.ends`).
        [VI] Cập nhật tiến trình mô phỏng theo thời gian (rút một đoạn `PATH` mỗi nhịp
             hoặc hoàn tất lệnh `INIT`/`MOVE` khi `now >= self.ends`).
        """
        with self._lock:
            now = time.monotonic()
            if self.path_active:
                if self.path_started and self.path_queue:
                    next_pos = self.path_queue.pop(0)
                    self.pos = next_pos
                    self.goal = next_pos
                    self.path_done += 1
                    if self.path_done == self.path_total:
                        self.busy = False
                        self.path_active = False
                        self.path_started = False
                        self.reason = "complete"
                self.emit()
                return

            if self.busy and now >= self.ends:
                if self.initializing:
                    self.epoch += 1
                    self.stage = 8
                    self.referenced = True
                    self.pos = (35200, 0, 1800)
                    self.reason = "initialized"
                    self.phase = (10, 10, 10)
                else:
                    self.pos = self.goal
                    self.reason = "complete"
                self.busy = False
                self.initializing = False
            elif self.initializing:
                elapsed_init = 1.3 - (self.ends - now)
                self.stage = min(7, 1 + int(elapsed_init * 5))
                self.phase = (4, 4, 4)
            self.emit()

    def close(self):
        """
        [EN] Stop the background simulation thread and close the mock link.
        [VI] Dừng luồng mô phỏng chạy ngầm và đóng kết nối giả lập.
        """
        self.closed = True
        self.closed_event.set()
        self.thread.join(timeout=0.2)
