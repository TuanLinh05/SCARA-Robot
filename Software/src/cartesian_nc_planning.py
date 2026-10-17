"""
================================================================================
FILE: cartesian_nc_planning.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Background preflight trajectory calculation and main-thread acceptance of
     planning results (`PlanningController` mixin for `NCApp`).
     - Offloads CPU-intensive DDA step simulation (`model.plan`, `validate_program`,
       `compile_bk`, `auto_fit_settings`) to daemon worker threads so the Tkinter
       GUI remains responsive.
     - Enforces strict thread safety: worker threads NEVER touch Tkinter widgets
       or the USB serial link, and results are accepted on the main thread ONLY
       if the calibration epoch, model, and starting step position have not changed.

[VI] MỤC ĐÍCH:
     Tính toán quy hoạch quỹ đạo kiểm tra trước ở luồng chạy ngầm và nghiệm thu
     kết quả trên luồng chính (`PlanningController` mixin cho `NCApp`).
     - Chuyển các phép mô phỏng từng xung DDA nặng về tính toán (`model.plan`,
       `validate_program`, `compile_bk`, `auto_fit_settings`) sang luồng phụ để
       giao diện Tkinter luôn mượt mà.
     - Đảm bảo an toàn đa luồng nghiêm ngặt: luồng phụ KHÔNG BAO GIỜ chạm vào
       widget Tkinter hay cổng USB, và kết quả chỉ được chấp nhận trên luồng chính
       khi phiên hiệu chuẩn (`epoch`), mô hình và vị trí xuất phát không bị thay đổi.

[DEBUG TIP / MẸO GỠ LỖI]:
     - If a planned move is silently ignored or shows "Mốc hoặc vị trí đã đổi...",
       check `self.plan_token` and `self.state.pos` in `_poll_planning()`: any new
       planning request or robot movement invalidates older background calculations.
     - Nếu lệnh di chuyển bị bỏ qua hoặc báo "Mốc hoặc vị trí đã đổi...", hãy kiểm tra
       `self.plan_token` và `self.state.pos` trong `_poll_planning()`: mọi yêu cầu
       tính toán mới hoặc sự thay đổi vị trí robot đều vô hiệu hóa kết quả cũ.
================================================================================
"""

from dataclasses import asdict
import queue
import threading
from typing import NamedTuple


# ==============================================================================
# 1. BACKGROUND PLANNING RESULT CONTAINER / CẤU TRÚC KẾT QUẢ QUY HOẠCH NGẦM
# ==============================================================================


class PlanningResult(NamedTuple):
    """
    [EN] Immutable message passed from the background planning thread back to
         the Tkinter main thread via `self.plan_results`.
         - `token`: Monotonic request ID used to discard superseded calculations.
         - `epoch`: Calibration epoch when the calculation started.
         - `start`: Starting step tuple `(z, j1, j2)` when the calculation started.
         - `model`: Kinematic model snapshot used during calculation.
         - `moving`: `True` to transmit `MOVE` immediately; `False` for preview only.
         - `result`: Computed `Plan` / `BKProgram` / tuple of `Setpoint`, or `Exception`.
         - `purpose`: Task tag (`"manual"`, `"follow"`, `"program"`, `"program_check"`,
                      `"drawing_check"`, `"drawing_fit"`).
         - `revision`: Follow-mode target revision counter.
    [VI] Gói kết quả bất biến truyền từ luồng tính toán ngầm về luồng chính Tkinter
         thông qua hàng đợi `self.plan_results`.
         - `token`: Mã thứ tự yêu cầu dùng để loại bỏ các kết quả tính toán cũ.
         - `epoch`: Phiên hiệu chuẩn tại thời điểm bắt đầu tính toán.
         - `start`: Tọa độ xung xuất phát `(z, j1, j2)` lúc bắt đầu tính.
         - `model`: Bản chụp mô hình động học dùng để tính quỹ đạo.
         - `moving`: `True` nếu cần gửi lệnh `MOVE` chạy thật; `False` nếu chỉ xem trước.
         - `result`: Kết quả tính (`Plan`, `BKProgram`, danh sách điểm) hoặc `Exception`.
         - `purpose`: Nhãn mục đích (`"manual"`, `"follow"`, `"program"`, `"program_check"`,
                      `"drawing_check"`, `"drawing_fit"`).
         - `revision`: Bộ đếm phiên bản mục tiêu của chế độ bám mô hình (`follow`).
    """

    token: int
    epoch: int
    start: tuple
    model: object
    moving: bool
    result: object
    purpose: str
    revision: int


# ==============================================================================
# 2. PLANNING CONTROLLER MIXIN / BỘ ĐIỀU KHIỂN QUY HOẠCH ĐƯỜNG ĐI
# ==============================================================================


class PlanningController:
    """
    [EN] Mixin class managing background trajectory validation threads and
         safe main-thread dispatch of validated `MOVE` commands.
    [VI] Lớp Mixin quản lý các luồng kiểm duyệt quỹ đạo chạy ngầm và phát lệnh
         `MOVE` an toàn trên luồng chính sau khi đã kiểm tra giới hạn.
    """

    def submit_planning(
        self, calculate, *, purpose, moving=False, revision=0, name="SCARA preflight"
    ):
        """
        [EN] Capture the current calibration snapshot (`model`, `start`, `epoch`)
             and launch a daemon thread to run `calculate(model, start, cancelled)`.
             Workers never touch Tk widgets or send USB commands.
        [VI] Chụp lại trạng thái hiệu chuẩn hiện tại (`model`, `start`, `epoch`)
             và khởi chạy luồng ngầm thực thi hàm `calculate(model, start, cancelled)`.
             Luồng phụ tuyệt đối không truy cập Tkinter hay gửi lệnh USB.
        """
        # [EN] Increment `plan_token` so any older background thread is marked stale.
        # [VI] Tăng `plan_token` để đánh dấu hủy hiệu lực mọi luồng tính toán cũ hơn.
        self.planning = True
        self.plan_token += 1
        token = self.plan_token

        # [EN] Snapshot robot state on the main thread before spawning the worker.
        # [VI] Chụp lại trạng thái robot trên luồng chính trước khi tạo luồng phụ.
        model = self.model
        start = self.state.pos
        epoch = self.state.epoch

        def is_cancelled():
            """
            [EN] Return True if a newer planning request or cancellation occurred.
            [VI] Trả về True nếu đã có yêu cầu tính toán mới hơn hoặc lệnh hủy.
            """
            return token != self.plan_token

        def worker():
            """
            [EN] Background worker function: runs pure math validation and pushes
                 the result (or caught Exception) into the thread-safe queue.
            [VI] Hàm chạy trên luồng phụ: thực hiện tính toán thuần túy và đẩy kết quả
                 (hoặc ngoại lệ bắt được) vào hàng đợi an toàn đa luồng.
            """
            try:
                result = calculate(model, start, is_cancelled)
            except Exception as error:
                result = error

            package = PlanningResult(
                token, epoch, start, model, moving, result, purpose, revision
            )
            self.plan_results.put(package)

        thread = threading.Thread(target=worker, daemon=True, name=name)
        thread.start()
        return thread

    def cancel_planning(self):
        """
        [EN] Invalidate any in-flight background planning task by incrementing `plan_token`.
        [VI] Hủy hiệu lực mọi tác vụ quy hoạch đang chạy ngầm bằng cách tăng `plan_token`.
        """
        self.plan_token += 1
        self.planning = False

    def _poll_planning(self, now):
        """
        [EN] Drain completed `PlanningResult` items on the Tkinter main thread,
             verify that the robot state still matches the snapshot when planning
             started, and either update the preview or transmit the `MOVE` command.
        [VI] Lấy các kết quả `PlanningResult` đã hoàn thành trên luồng chính Tkinter,
             kiểm tra xem trạng thái robot có còn khớp với thời điểm bắt đầu tính toán
             hay không, sau đó cập nhật hình xem trước hoặc phát lệnh `MOVE`.
        """
        while True:
            try:
                item = self.plan_results.get_nowait()
            except queue.Empty:
                break

            token = item.token
            epoch = item.epoch
            start = item.start
            model = item.model
            moving = item.moving
            result = item.result
            purpose = item.purpose
            revision = item.revision

            # ------------------------------------------------------------------
            # [EN] 1. Ignore stale results superseded by a newer planning request.
            # [VI] 1. Bỏ qua kết quả cũ đã bị thay thế bởi yêu cầu tính toán mới hơn.
            # ------------------------------------------------------------------
            if token != self.plan_token:
                continue

            self.planning = False
            if purpose == "drawing_check":
                self.bk_pending = False

            # ------------------------------------------------------------------
            # [EN] 2. Reject if robot lost reference, recalibrated, or moved
            #         while the background worker was computing.
            # [VI] 2. Từ chối nếu robot mất mốc, hiệu chuẩn lại hoặc đã dịch chuyển
            #         trong lúc luồng phụ đang tính toán.
            # ------------------------------------------------------------------
            snapshot_changed = (
                not self.operable()
                or not self.local_reference
                or self.model != model
                or self.state.epoch != epoch
                or self.state.pos != start
            )
            if snapshot_changed:
                self.message.set(
                    "Mốc hoặc vị trí đã đổi trong khi tính đường đi; hãy thử lại."
                )
                continue

            # [EN] Ignore follow-mode or program moves if those modes were turned off.
            # [VI] Bỏ qua lệnh bám mô hình hoặc chuỗi điểm nếu chế độ đó đã bị tắt.
            if purpose == "follow" and (
                not self.follow_enabled.get() or revision != self.follow_revision
            ):
                continue
            if purpose == "program" and not self.program_points:
                continue

            # ------------------------------------------------------------------
            # [EN] 3. Handle validation/kinematic exceptions returned by worker.
            # [VI] 3. Xử lý các ngoại lệ vi phạm giới hạn/động học từ luồng phụ.
            # ------------------------------------------------------------------
            if isinstance(result, Exception):
                self.preview = None
                error_text = str(result)
                self.message.set(error_text)
                if purpose in ("follow", "program", "program_check", "drawing_check"):
                    self.clear_automation()
                if purpose in ("drawing_check", "drawing_fit"):
                    self.bk_note.set(error_text)
                continue

            # ------------------------------------------------------------------
            # [EN] 4. Dispatch specialized drawing/program preflight completions.
            # [VI] 4. Điều phối kết quả kiểm tra cho chế độ vẽ hình hoặc chuỗi điểm.
            # ------------------------------------------------------------------
            if purpose == "drawing_check":
                self.accept_bk(result)
                continue
            if purpose == "drawing_fit":
                self.accept_fit(result)
                continue
            if purpose == "program_check":
                self.program_points = result
                self.program_index = 0
                self.program_wait = now
                serialized_points = []
                for point_item in result:
                    serialized_points.append(asdict(point_item))
                self.trace_event("program_start", points=serialized_points)
                continue

            # ------------------------------------------------------------------
            # [EN] 5. Standard single-segment `Plan` result (preview or `MOVE`).
            # [VI] 5. Kết quả quy hoạch 1 đoạn `Plan` thông thường (xem trước hoặc `MOVE`).
            # ------------------------------------------------------------------
            self.preview = result
            if purpose == "manual":
                self.set_target_pose(result.joints)

            if moving:
                # [EN] Enforce foreground window safety check before starting motion.
                # [VI] Kiểm tra an toàn cửa sổ tiêu điểm (foreground) trước khi phát lệnh chạy.
                if not self._foreground():
                    if purpose == "follow":
                        self.focus_lost("FOCUS")
                        continue
                    if not self.background_allowed():
                        self.stop("FOCUS")
                        continue

                # [EN] If target step coordinates equal current position, no pulse needed.
                # [VI] Nếu tọa độ xung đích trùng với vị trí hiện tại thì không cần phát xung.
                if result.steps == self.state.pos:
                    if purpose == "program":
                        self.finish_program_point(now)
                    elif purpose == "follow":
                        self.follow_target = None
                    else:
                        self.message.set("Đích đã trùng vị trí hiện tại.")
                    continue

                # [EN] Allocate a new job ID and format the `MOVE` command.
                # [VI] Cấp mã công việc (`job`) mới và định dạng lệnh `MOVE`.
                self.active_purpose = purpose
                self.job = max(self.job, self.state.job) + 1

                step_strings = []
                for step_val in result.steps:
                    step_strings.append(str(step_val))
                steps_text = " ".join(step_strings)

                move_cmd = (
                    f"MOVE {self.link.session} {self.job} "
                    + steps_text
                    + f" {result.rate}"
                )
                self.sequence([move_cmd])

                if purpose == "program":
                    point = self.program_points[self.program_index]
                    total_points = len(self.program_points)
                    self.message.set(
                        f"Đang tới {point.name} ({self.program_index+1}/{total_points})…"
                    )
                    row_id = str(self.program_index)
                    can_select_row = (
                        not self.bk_active
                        and self.points_window
                        and self.points_window.winfo_exists()
                        and self.points_tree.exists(row_id)
                    )
                    if can_select_row:
                        self.points_tree.selection_set(row_id)
                        self.points_tree.see(row_id)
                else:
                    self.message.set("Đang gửi chuyển động đã kiểm tra giới hạn.")
            else:
                j1_deg = result.joints[0]
                j2_deg = result.joints[1]
                self.message.set(
                    f"Đích khớp: J1={j1_deg:.2f}°, J2={j2_deg:.2f}°; nhịp ≤{result.rate} xung/s. Đường đi có thể cong."
                )
