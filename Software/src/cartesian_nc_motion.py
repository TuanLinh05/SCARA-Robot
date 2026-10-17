"""
================================================================================
FILE: cartesian_nc_motion.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     High-level motion orchestration (`MotionController` mixin for `NCApp`):
     - `initialize()`: sends mechanical configuration + `INIT` to home and
       calibrate all 3 axes against the 6 limit switches.
     - `preview_target()` / `move()`: triggers background DDA path validation
       and executes single-point Cartesian moves.
     - `start_program()` / `automation_tick()`: runs multi-point setpoint tables,
       interactive model-follow mode, and continuous `PATH`/`SEG`/`GO` strokes.
     - `hold()`: toggles stepper motor holding torque on J1/J2 (invalidating home).

[VI] MỤC ĐÍCH:
     Điều phối chuyển động cấp cao (`MotionController` mixin cho `NCApp`):
     - `initialize()`: gửi cấu hình cơ khí + lệnh `INIT` để dò gốc (Home) và
       hiệu chuẩn cả 3 trục bằng 6 công tắc hành trình.
     - `preview_target()` / `move()`: kích hoạt kiểm tra đường đi DDA ngầm và
       thực thi di chuyển tới điểm XYZ đơn lẻ.
     - `start_program()` / `automation_tick()`: chạy bảng chuỗi điểm (setpoint),
       chế độ bám mô hình trực tiếp, và truyền nét vẽ liên tục `PATH`/`SEG`/`GO`.
     - `hold()`: bật/tắt lực giữ động cơ bước J1/J2 (yêu cầu lấy mốc lại sau khi đổi).

[DEBUG TIP / MẸO GỠ LỖI]:
     - Notice that `start_stroke()` checks `self.state.pos != run.start_steps`
       before uploading `PATH`: if the robot drifted or missed a step before the
       stroke starts, the software aborts immediately rather than drawing offset!
     - Lưu ý hàm `start_stroke()` luôn kiểm tra `self.state.pos != run.start_steps`
       trước khi nạp `PATH`: nếu tọa độ xung bị lệch trước khi bắt đầu nét vẽ,
       phần mềm sẽ hủy ngay lập tức thay vì vẽ lệch vị trí!
================================================================================
"""

from dataclasses import asdict
import time
from cartesian_nc_workspace import bounded_pose, validate_program
from cartesian_nc_path import PathUpload
from cartesian_nc_shapes import PATTERNS


class MotionController:
    """
    [EN] Mixin class providing homing initialization, point-to-point motion,
         interactive follow mode, setpoint program execution, and stroke uploads.
    [VI] Lớp Mixin cung cấp chức năng lấy gốc hiệu chuẩn, di chuyển điểm-tới-điểm,
         chế độ bám mô hình, chạy chương trình chuỗi điểm và tải nét vẽ liên tục.
    """

    # ==========================================================================
    # 1. HOMING & CALIBRATION / LẤY GỐC & HIỆU CHUẨN (HOME + CALIB)
    # ==========================================================================

    def initialize(self, quick=False):
        """
        [EN] Send mechanical parameters (`GEOM`, `SPAN`, `PARK`, `TUNE`, `POL`,
             `COUPLE`) followed by `INIT <session> <job> [QUICK]` to start the
             homing sequence on the STM32. When `quick=True` and the robot has
             already completed a full calibration in this session, each axis only
             latches its negative switch (`SEEK_N -> CLEAR_N -> LATCH_N`) using
             the saved span, cutting homing time by ~75%.
        [VI] Gửi các thông số cơ khí (`GEOM`, `SPAN`, `PARK`, `TUNE`, `POL`,
             `COUPLE`) kèm lệnh `INIT <session> <job> [QUICK]` để bắt đầu quy trình
             dò gốc trên STM32. Khi `quick=True` và robot đã từng hiệu chuẩn toàn
             diện, mỗi trục chỉ cần chốt công tắc âm bằng dải xung đã lưu, rút ngắn
             ~75% thời gian lấy mốc.
        """
        # [EN] Require an idle, fault-free connection with all 3 axes holding (`0b111 = 7`).
        # [VI] Yêu cầu kết nối rảnh, không lỗi và cả 3 trục đang bật lực giữ (`0b111 = 7`).
        if not self.operable() or self.state.holding != 7:
            return

        # [EN] Clear any running automation and invalidate the previous calibration model.
        # [VI] Xóa mọi chương trình tự động đang chạy và hủy mô hình hiệu chuẩn cũ.
        self.clear_automation()
        self.local_reference = False
        self.model = None
        self.preview = None

        # [EN] Expect the firmware calibration epoch to increment by 1 when homing finishes.
        # [VI] Kỳ vọng số phiên hiệu chuẩn (`epoch`) trên firmware tăng thêm 1 khi lấy gốc xong.
        self.expect_epoch = self.state.epoch + 1
        self.job = max(self.job, self.state.job) + 1
        self.task_kind = "INIT"

        # [EN] Format full `INIT` or `INIT ... QUICK` command and queue behind config lines.
        # [VI] Định dạng lệnh `INIT` đầy đủ hoặc `INIT ... QUICK` và xếp sau chuỗi lệnh cấu hình.
        if quick:
            init_cmd = f"INIT {self.link.session} {self.job} QUICK"
        else:
            init_cmd = f"INIT {self.link.session} {self.job}"

        self.trace_event(
            "home_start",
            config=asdict(self.config),
            job=self.job,
            quick=bool(quick),
        )

        config_commands = self.config.commands(self.link.session)
        self.sequence(config_commands + [init_cmd])

        if quick:
            mode_note = "Đang HOME Nhanh (chỉ chốt biên âm dựa trên dải xung đã đo); Esc/Dừng hủy ngay. "
        else:
            mode_note = "Robot sẽ chạy đến các công tắc để đo; Esc/Dừng hủy ngay. "

        if self.config.home_in_background:
            bg_note = "Home tiếp tục khi đổi cửa sổ; biên và mất kết nối vẫn được bảo vệ."
        else:
            bg_note = "Giữ cửa sổ này ở phía trước."

        self.message.set(mode_note + bg_note)
        self.render()

    def quick_initialize(self):
        """
        [EN] Convenience trigger for Quick Re-Home (`INIT <session> <job> QUICK`).
        [VI] Hàm gọi nhanh chế độ Lấy gốc Nhanh (`INIT <session> <job> QUICK`).
        """
        self.initialize(quick=True)

    # ==========================================================================
    # 2. SINGLE-POINT PREVIEW & MOVE / XEM TRƯỚC & DI CHUYỂN ĐIỂM ĐƠN
    # ==========================================================================

    def get_plan(self):
        """
        [EN] Synchronously compute and validate a DDA `Plan` from current position
             to the XYZ entry fields at the requested speed.
        [VI] Tính toán và kiểm duyệt đồng bộ một `Plan` DDA từ vị trí hiện tại tới
             tọa độ trong ô nhập XYZ với tốc độ yêu cầu.
        """
        if not self.fresh() or not self.local_reference or not self.model:
            raise ValueError("Cần HOME + CALIB trước.")

        target_coords = []
        for coord_var in self.xyz:
            target_coords.append(float(coord_var.get()))
        target_xyz = tuple(target_coords)
        requested_speed = float(self.speed.get())

        return self.model.plan(self.state.pos, target_xyz, requested_speed)

    def preview_target(self):
        """
        [EN] Validate and preview the path to the XYZ entry fields without moving.
        [VI] Kiểm tra và hiển thị xem trước đường đi tới tọa độ XYZ mà không chạy thật.
        """
        self.start_plan(False)

    def move(self):
        """
        [EN] Validate the path to the XYZ entry fields and execute `MOVE` if safe.
        [VI] Kiểm tra đường đi tới tọa độ XYZ và phát lệnh chạy `MOVE` nếu an toàn.
        """
        self.start_plan(True)

    def start_plan(self, moving, target=None, speed=None, purpose="manual", revision=0):
        """
        [EN] Submit a single-segment path validation request to the background
             planning worker thread.
        [VI] Gửi yêu cầu kiểm duyệt đường đi một đoạn sang luồng tính toán ngầm.
        """
        if not self.operable() or not self.local_reference or not self.model:
            return
        try:
            if target is None:
                parsed_coords = []
                for coord_var in self.xyz:
                    parsed_coords.append(float(coord_var.get()))
                target = tuple(parsed_coords)
            else:
                target = tuple(target)

            if speed is None:
                speed = float(self.speed.get())
        except ValueError as e:
            self.message.set(str(e))
            return

        if purpose == "manual":
            self.clear_automation()

        def calculate_single_plan(model, start, cancelled):
            return model.plan(start, target, speed)

        self.submit_planning(
            calculate_single_plan,
            purpose=purpose,
            moving=moving,
            revision=revision,
            name="SCARA path validation",
        )
        self.message.set("Đang kiểm tra đường đi và từng bước DDA…")
        self.render()

    # ==========================================================================
    # 3. AUTOMATION, FOLLOW MODE & SETPOINT PROGRAMS / TỰ ĐỘNG HÓA & CHUỖI ĐIỂM
    # ==========================================================================

    def clear_automation(self):
        """
        [EN] Reset all active drawing uploads, follow-mode targets, and multi-point
             program state.
        [VI] Xóa toàn bộ trạng thái tải nét vẽ, mục tiêu bám mô hình và chuỗi điểm tự động.
        """
        self.path_upload = None
        self.path_run = None
        self.bk_active = False
        self.bk_pending = False
        self.follow_enabled.set(False)
        self.follow_target = None
        self.follow_revision += 1
        self.program_points = ()
        self.program_index = 0
        self.program_wait = 0

    def toggle_follow(self):
        """
        [EN] Enable or disable interactive "follow model" mode (where dragging the
             virtual robot arm or joint sliders streams bounded step increments).
        [VI] Bật hoặc tắt chế độ "bám mô hình" trực tiếp (kéo tay máy ảo trên màn hình
             hoặc thanh trượt khớp sẽ điều khiển robot thật đi theo từng bước nhỏ).
        """
        if not self.follow_enabled.get():
            self.follow_target = None
            self.follow_revision += 1
            if self.active_purpose == "follow":
                self.stop()
            return
        if not self.operable() or not self.local_reference or self.program_points:
            self.follow_enabled.set(False)
            self.message.set("Cần lấy mốc và dừng trước khi bật bám mô hình.")
            return
        self.message.set(
            "Bám mô hình đã bật. Kéo đầu công tác, khuỷu hoặc thanh trượt để điều khiển."
        )
        self.trace_event("follow_enabled")

    def start_program(self):
        """
        [EN] Validate the entire user setpoint table (`self.points`) in the
             background before executing the first point.
        [VI] Kiểm duyệt toàn bộ bảng chuỗi điểm (`self.points`) ở luồng ngầm trước
             khi bắt đầu chạy điểm đầu tiên.
        """
        if (
            not self.operable()
            or not self.local_reference
            or not self.points
            or self.program_points
        ):
            return
        self.clear_automation()
        points = tuple(self.points)

        def calculate(model, start, cancelled):
            validate_program(model, start, points, cancelled)
            return points

        self.submit_planning(
            calculate, purpose="program_check", name="SCARA setpoint validation"
        )
        self.message.set(f"Đang kiểm tra toàn bộ{len(points)} điểm và đường đi…")

    def finish_program_point(self, now):
        """
        [EN] Called when a setpoint move or continuous stroke completes:
             records telemetry, schedules the dwell timer (`point.dwell_s`), or
             finalizes the program/drawing when the last point is reached.
        [VI] Được gọi khi hoàn thành một điểm trong chuỗi hoặc một nét vẽ liên tục:
             ghi nhận telemetry, đặt bộ đếm thời gian nghỉ (`point.dwell_s`), hoặc
             hoàn tất chương trình/bản vẽ khi đã đi hết điểm cuối cùng.
        """
        point = self.program_points[self.program_index]
        drawing = self.bk_active
        is_dry_run = self.bk_active_plan.dry if drawing else False

        self.record_bk(self.state, True)
        self.trace_event(
            "setpoint_complete",
            index=self.program_index,
            point=asdict(point),
            status=asdict(self.state),
        )

        # [EN] Advance to the next waypoint and set the post-move dwell timer.
        # [VI] Tăng chỉ số sang điểm tiếp theo và đặt hẹn giờ dừng nghỉ (`dwell_s`).
        self.program_index += 1
        self.program_wait = now + point.dwell_s

        if self.program_index == len(self.program_points):
            self.program_points = ()
            self.message.set("Đã hoàn thành chuỗi setpoint.")
            if drawing:
                self.bk_active = False
                code = self.bk_active_plan.settings.pattern
                if hasattr(self, "_pattern_title"):
                    pattern = self._pattern_title(code)
                else:
                    pattern = PATTERNS.get(code, (code,))[0]

                if is_dry_run:
                    message = f"Đã chạy khô {pattern}; bút ở Z nâng."
                else:
                    message = f"Đã vẽ {pattern} theo xung; bút đã nâng."

                self.message.set(message)
                self.bk_note.set(message)
                elapsed = now - self.bk_started
                self.bk_note.set(message + f" Tổng thời gian {elapsed:.1f}s.")
                self.trace_event(
                    "drawing_complete",
                    dry=is_dry_run,
                    elapsed_s=elapsed,
                    status=asdict(self.state),
                )
        else:
            self.message.set(
                f"Đã đến{point.name}; nghỉ{point.dwell_s:g}s trước điểm tiếp theo."
            )

    def automation_tick(self, now):
        """
        [EN] Periodic state-machine tick for multi-point programs, continuous
             drawing strokes, and interactive model-follow mode.
        [VI] Nhịp cập nhật máy trạng thái định kỳ cho chương trình chuỗi điểm,
             nét vẽ liên tục và chế độ bám mô hình tương tác.
        """
        if not self.editor_ready():
            if self.follow_enabled.get() or self.program_points:
                self.clear_automation()
            return
        if not self.operable():
            return

        # ----------------------------------------------------------------------
        # [EN] Branch A: Advance multi-point program or drawing after dwell time.
        # [VI] Nhánh A: Chuyển sang điểm tiếp theo hoặc nét vẽ sau khi hết thời gian nghỉ.
        # ----------------------------------------------------------------------
        if self.program_points:
            if now >= self.program_wait:
                if self.bk_active:
                    # [EN] Check if a continuous pen-down stroke starts at `program_index`.
                    # [VI] Kiểm tra xem có nét vẽ liên tục nào bắt đầu tại `program_index` không.
                    matching_run = None
                    for candidate_run in self.bk_active_plan.runs:
                        if candidate_run.first == self.program_index:
                            matching_run = candidate_run
                            break
                    if matching_run is not None:
                        self.start_stroke(matching_run, now)
                        return

                point = self.program_points[self.program_index]
                current_joint_pose = self.model.from_steps(self.state.pos)
                target_joint_pose = self.model.ik(point.xyz, current_joint_pose)
                self.set_target_pose(target_joint_pose)
                self.start_plan(
                    True, target=point.xyz, speed=point.speed_mm_s, purpose="program"
                )
            return

        # ----------------------------------------------------------------------
        # [EN] Branch B: Advance interactive follow mode by one bounded step.
        # [VI] Nhánh B: Tiến thêm một bước khớp giới hạn trong chế độ bám mô hình.
        # ----------------------------------------------------------------------
        if (
            self.follow_enabled.get()
            and self.follow_target is not None
            and now >= self.follow_due
        ):
            if not self._foreground():
                self.focus_lost("FOCUS")
                return
            try:
                q0 = self.model.from_steps(self.state.pos)
                q1 = self.model.ik(self.follow_target, q0)
                if self.model.to_steps(q1) == self.state.pos:
                    self.follow_target = None
                    self.message.set("Đã bám đến đích mô hình.")
                    return
                segment = bounded_pose(q0, q1)
                segment_xyz = self.model.fk(segment)
                self.start_plan(
                    True,
                    target=segment_xyz,
                    purpose="follow",
                    revision=self.follow_revision,
                )
            except ValueError as e:
                self.follow_target = None
                self.message.set(str(e))

    # ==========================================================================
    # 4. CONTINUOUS STROKE FIFO STREAMING / TRUYỀN NÉT VẼ LIÊN TỤC QUA FIFO
    # ==========================================================================

    def start_stroke(self, run, now):
        """
        [EN] Initiate a continuous `PATH` upload for a multi-segment pen-down stroke.
        [VI] Khởi tạo quá trình tải đường chạy `PATH` liên tục cho một nét vẽ nhiều đoạn.
        """
        # [EN] Verify the robot is at the exact expected starting step coordinates.
        # [VI] Kiểm tra robot đang đứng đúng tại tọa độ xung đầu nét vẽ đã tính toán.
        if self.state.pos != run.start_steps:
            self.clear_automation()
            self.message.set("Vị trí đầu nét đã đổi; đã hủy đường vẽ.")
            return

        self.job = max(self.job, self.state.job) + 1
        self.path_run = run
        self.path_since = now
        self.path_upload = PathUpload(self.link.session, self.job, run.segments)
        self.path_upload.pending = "PATH"
        self.active_purpose = "stroke"

        path_cmd = f"PATH {self.link.session} {self.job} {len(run.segments)}"
        self.sequence([path_cmd])
        self.message.set(
            f"Vẽ nét liên tục: {len(run.segments)} đoạn; dự kiến {run.seconds:.1f}s theo xung."
        )

        serialized_profiles = []
        for seg in run.segments:
            serialized_profiles.append(asdict(seg))

        self.trace_event(
            "stroke_start",
            job=self.job,
            first=run.first,
            end=run.end,
            estimated_seconds=run.seconds,
            profiles=serialized_profiles,
        )

    def path_tick(self, now):
        """
        [EN] Pump `SEG` and `GO` commands from `self.path_upload` into the serial
             link whenever the command window and STM32 FIFO credits permit.
        [VI] Bơm các lệnh `SEG` và `GO` từ `self.path_upload` xuống đường truyền
             serial mỗi khi cửa sổ lệnh và số ô trống FIFO trên STM32 cho phép.
        """
        if not self.path_upload or not self.link:
            return
        if self.pending and self.pending[0] not in ("SEG", "GO"):
            return
        while self.path_upload and self.link:
            line = self.path_upload.next()
            if not line:
                break
            self.pending = (self.path_upload.pending, now)
            if not self.send(line):
                break

    def finish_stroke(self, now):
        """
        [EN] Finalize a completed `PATH` stroke, log timing/starvation diagnostics,
             and advance the setpoint program index to the end of the stroke.
        [VI] Hoàn tất một nét vẽ liên tục `PATH`, ghi log thời gian/chẩn đoán bộ đệm,
             và chuyển chỉ số chương trình tới điểm cuối của nét vẽ.
        """
        run = self.path_run
        self.trace_event(
            "stroke_complete",
            job=self.state.job,
            elapsed_s=now - self.path_since,
            estimated_s=run.seconds,
            segments=len(run.segments),
            timer_late=self.state.timer_late,
            starved=self.state.starved,
            pos=self.state.pos,
        )
        self.path_run = None
        self.path_upload = None
        self.pending = None

        # [EN] Jump `program_index` to `run.end - 1` so `finish_program_point` advances to `run.end`.
        # [VI] Đặt `program_index = run.end - 1` để `finish_program_point` tăng lên `run.end`.
        self.program_index = run.end - 1
        self.finish_program_point(now)

    # ==========================================================================
    # 5. POSE SYNC & MOTOR HOLDING TORQUE / ĐỒNG BỘ TƯ THẾ & LỰC GIỮ ĐỘNG CƠ
    # ==========================================================================

    def current_target(self):
        """
        [EN] Snap the interactive workspace target to the robot's current live pose.
        [VI] Đồng bộ mục tiêu trên vùng làm việc trùng với tư thế hiện tại của robot.
        """
        if self.fresh() and self.local_reference and self.model:
            self.follow_target = None
            self.follow_revision += 1
            self.sync_editor()

    def hold(self, value):
        """
        [EN] Enable (`value=1`) or release (`value=0`) holding torque on J1 and J2.
             Releasing or changing holding torque invalidates the calibration reference.
        [VI] Bật (`value=1`) hoặc nhả (`value=0`) lực giữ động cơ trên trục J1 và J2.
             Thay đổi lực giữ sẽ xóa mốc chuẩn (cần bấm HOME + CALIB lại).
        """
        if not self.idle() or self.state.fault:
            return
        self.clear_automation()
        self.local_reference = False
        self.model = None
        self.expect_epoch = None

        hold_flag = int(value)
        self.sequence(
            [
                f"HOLD {self.link.session} J1 {hold_flag}",
                f"HOLD {self.link.session} J2 {hold_flag}",
            ]
        )
        self.message.set("Cần HOME + CALIB lại sau khi thay đổi lực giữ.")
        self.render()
