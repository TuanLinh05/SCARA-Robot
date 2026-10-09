"""Background preflight calculations and main-thread acceptance of their results."""

from dataclasses import asdict
import queue
import threading
from typing import NamedTuple


class PlanningResult(NamedTuple):
    token: int
    epoch: int
    start: tuple
    model: object
    moving: bool
    result: object
    purpose: str
    revision: int


class PlanningController:
    def submit_planning(
        self, calculate, *, purpose, moving=False, revision=0, name="SCARA preflight"
    ):
        """Capture the reference before starting; workers never touch Tk or send USB."""
        self.planning = True
        self.plan_token += 1
        token = self.plan_token
        model = self.model
        start = self.state.pos
        epoch = self.state.epoch

        def worker():
            try:
                result = calculate(model, start, lambda: token != self.plan_token)
            except Exception as error:
                result = error
            self.plan_results.put(
                PlanningResult(
                    token, epoch, start, model, moving, result, purpose, revision
                )
            )

        thread = threading.Thread(target=worker, daemon=True, name=name)
        thread.start()
        return thread

    def cancel_planning(self):
        self.plan_token += 1
        self.planning = False

    def _poll_planning(self, now):
        while True:
            try:
                token, epoch, start, model, moving, result, purpose, revision = (
                    self.plan_results.get_nowait()
                )
            except queue.Empty:
                break
            if token != self.plan_token:
                continue
            self.planning = False
            if purpose == "drawing_check":
                self.bk_pending = False
            if (
                not self.operable()
                or not self.local_reference
                or self.model != model
                or self.state.epoch != epoch
                or self.state.pos != start
            ):
                self.message.set(
                    "Mốc hoặc vị trí đã đổi trong khi tính đường đi; hãy thử lại."
                )
                continue
            if purpose == "follow" and (
                not self.follow_enabled.get() or revision != self.follow_revision
            ):
                continue
            if purpose == "program" and not self.program_points:
                continue
            if isinstance(result, Exception):
                self.preview = None
                self.message.set(str(result))
                if purpose in ("follow", "program", "program_check", "drawing_check"):
                    self.clear_automation()
                if purpose in ("drawing_check", "drawing_fit"):
                    self.bk_note.set(str(result))
                continue
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
                self.trace_event("program_start", points=[asdict(p) for p in result])
                continue
            self.preview = result
            if purpose == "manual":
                self.set_target_pose(result.joints)
            if moving:
                if not self._foreground():
                    if purpose == "follow":
                        self.focus_lost("FOCUS")
                        continue
                    if not self.background_allowed():
                        self.stop("FOCUS")
                        continue
                if result.steps == self.state.pos:
                    if purpose == "program":
                        self.finish_program_point(now)
                    elif purpose == "follow":
                        self.follow_target = None
                    else:
                        self.message.set("Đích đã trùng vị trí hiện tại.")
                    continue
                self.active_purpose = purpose
                self.job = max(self.job, self.state.job) + 1
                self.sequence(
                    [
                        f"MOVE {self.link.session} {self.job} "
                        + " ".join(map(str, result.steps))
                        + f" {result.rate}"
                    ]
                )
                if purpose == "program":
                    point = self.program_points[self.program_index]
                    self.message.set(
                        f"Đang tới {point.name} ({self.program_index+1}/{len(self.program_points)})…"
                    )
                    if (
                        not self.bk_active
                        and self.points_window
                        and self.points_window.winfo_exists()
                        and self.points_tree.exists(str(self.program_index))
                    ):
                        self.points_tree.selection_set(str(self.program_index))
                        self.points_tree.see(str(self.program_index))
                else:
                    self.message.set("Đang gửi chuyển động đã kiểm tra giới hạn.")
            else:
                self.message.set(
                    f"Đích khớp: J1={result.joints[0]:.2f}°, J2={result.joints[1]:.2f}°; nhịp ≤{result.rate} xung/s. Đường đi có thể cong."
                )
