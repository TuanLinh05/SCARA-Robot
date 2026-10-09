"""Validated moves, setpoint automation and continuous stroke uploads for NCApp."""

from dataclasses import asdict
import time
from cartesian_nc_workspace import bounded_pose, validate_program
from cartesian_nc_path import PathUpload
from cartesian_nc_shapes import PATTERNS


class MotionController:
    def initialize(self):
        if not self.operable() or self.state.holding != 7:
            return
        self.clear_automation()
        self.local_reference = False
        self.model = None
        self.preview = None
        self.expect_epoch = self.state.epoch + 1
        self.job = max(self.job, self.state.job) + 1
        self.task_kind = "INIT"
        self.trace_event("home_start", config=asdict(self.config), job=self.job)
        self.sequence(
            self.config.commands(self.link.session)
            + [f"INIT {self.link.session} {self.job}"]
        )
        self.message.set(
            "Robot sẽ chạy đến các công tắc để đo; Esc/Dừng hủy ngay. "
            + (
                "Home tiếp tục khi đổi cửa sổ; biên và mất kết nối vẫn được bảo vệ."
                if self.config.home_in_background
                else "Giữ cửa sổ này ở phía trước."
            )
        )
        self.render()

    def get_plan(self):
        if not self.fresh() or not self.local_reference or not self.model:
            raise ValueError("Cần HOME + CALIB trước.")
        return self.model.plan(
            self.state.pos,
            tuple(float(v.get()) for v in self.xyz),
            float(self.speed.get()),
        )

    def preview_target(self):
        self.start_plan(False)

    def move(self):
        self.start_plan(True)

    def start_plan(self, moving, target=None, speed=None, purpose="manual", revision=0):
        if not self.operable() or not self.local_reference or not self.model:
            return
        try:
            target = (
                tuple(float(v.get()) for v in self.xyz)
                if target is None
                else tuple(target)
            )
            speed = float(self.speed.get()) if speed is None else speed
        except ValueError as e:
            self.message.set(str(e))
            return
        if purpose == "manual":
            self.clear_automation()
        self.submit_planning(
            lambda model, start, cancelled: model.plan(start, target, speed),
            purpose=purpose,
            moving=moving,
            revision=revision,
            name="SCARA path validation",
        )
        self.message.set("Đang kiểm tra đường đi và từng bước DDA…")
        self.render()

    def clear_automation(self):
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
        point = self.program_points[self.program_index]
        drawing = self.bk_active
        mode = self.bk_active_plan.dry if drawing else False
        self.record_bk(self.state, True)
        self.trace_event(
            "setpoint_complete",
            index=self.program_index,
            point=asdict(point),
            status=asdict(self.state),
        )
        self.program_index += 1
        self.program_wait = now + point.dwell_s
        if self.program_index == len(self.program_points):
            self.program_points = ()
            self.message.set("Đã hoàn thành chuỗi setpoint.")
            if drawing:
                self.bk_active = False
                pattern = PATTERNS[self.bk_active_plan.settings.pattern][0]
                message = (
                    f"Đã chạy khô {pattern}; bút ở Z nâng."
                    if mode
                    else f"Đã vẽ {pattern} theo xung; bút đã nâng."
                )
                self.message.set(message)
                self.bk_note.set(message)
                elapsed = now - self.bk_started
                self.bk_note.set(message + f" Tổng thời gian {elapsed:.1f}s.")
                self.trace_event(
                    "drawing_complete",
                    dry=mode,
                    elapsed_s=elapsed,
                    status=asdict(self.state),
                )
        else:
            self.message.set(
                f"Đã đến{point.name}; nghỉ{point.dwell_s:g}s trước điểm tiếp theo."
            )

    def automation_tick(self, now):
        if not self.editor_ready():
            if self.follow_enabled.get() or self.program_points:
                self.clear_automation()
            return
        if not self.operable():
            return
        if self.program_points:
            if now >= self.program_wait:
                if self.bk_active:
                    run = next(
                        (
                            r
                            for r in self.bk_active_plan.runs
                            if r.first == self.program_index
                        ),
                        None,
                    )
                    if run is not None:
                        self.start_stroke(run, now)
                        return
                point = self.program_points[self.program_index]
                self.set_target_pose(
                    self.model.ik(point.xyz, self.model.from_steps(self.state.pos))
                )
                self.start_plan(
                    True, target=point.xyz, speed=point.speed_mm_s, purpose="program"
                )
            return
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
                self.start_plan(
                    True,
                    target=self.model.fk(segment),
                    purpose="follow",
                    revision=self.follow_revision,
                )
            except ValueError as e:
                self.follow_target = None
                self.message.set(str(e))

    def start_stroke(self, run, now):
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
        self.sequence([f"PATH {self.link.session} {self.job} {len(run.segments)}"])
        self.message.set(
            f"Vẽ nét liên tục: {len(run.segments)} đoạn; dự kiến {run.seconds:.1f}s theo xung."
        )
        self.trace_event(
            "stroke_start",
            job=self.job,
            first=run.first,
            end=run.end,
            estimated_seconds=run.seconds,
            profiles=[asdict(s) for s in run.segments],
        )

    def path_tick(self, now):
        if not self.path_upload or self.pending or not self.link:
            return
        line = self.path_upload.next()
        if line:
            self.pending = (line.split()[0], now)
            self.send(line)

    def finish_stroke(self, now):
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
        self.path_run = self.path_upload = None
        self.pending = None
        self.program_index = run.end - 1
        self.finish_program_point(now)

    def current_target(self):
        if self.fresh() and self.local_reference and self.model:
            self.follow_target = None
            self.follow_revision += 1
            self.sync_editor()

    def hold(self, value):
        if not self.idle() or self.state.fault:
            return
        self.clear_automation()
        self.local_reference = False
        self.model = None
        self.expect_epoch = None
        self.sequence(
            [
                f"HOLD {self.link.session} J1 {int(value)}",
                f"HOLD {self.link.session} J2 {int(value)}",
            ]
        )
        self.message.set("Cần HOME + CALIB lại sau khi thay đổi lực giữ.")
        self.render()
