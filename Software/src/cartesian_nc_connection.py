"""USB connection, logging and acknowledged command sequencing for NCApp."""

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
    def trace_event(self, kind, **fields):
        if self.trace:
            self.trace.record(kind, **fields)

    def open_logs(self):
        root = self.config_path.parent / "logs"
        root.mkdir(exist_ok=True)
        if sys.platform == "win32":
            import os

            os.startfile(root)

    def fresh(self):
        return status_fresh(self.link, self.state, self.rx, time.monotonic())

    def idle(self):
        return (
            self.fresh()
            and not self.state.busy
            and self.active_job is None
            and self.pending is None
            and not self.commands
            and not self.planning
        )

    def operable(self):
        return self.idle() and hardware_ready(self.state, self.config_error)

    def refresh_ports(self):
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
            self.port.set(ports[0] if ports else "")

    def toggle_connection(self):
        if self.link:
            self.disconnect("Đã ngắt USB.")
            return
        try:
            if not self.port.get():
                raise ValueError("Chọn cổng COM.")
            self.link = self.factory(self.port.get())
            self.opened = time.monotonic()
            try:
                self.trace = SessionLog(self.config_path.parent / "logs")
                self.last_log_path = self.trace.path
                self.log_text.set("Log: " + self.trace.path.name)
                self.trace_event(
                    "connection",
                    port=self.port.get(),
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
        self.trace_event("tx", command=line)
        try:
            self.link.send(line)
            return True
        except Exception as e:
            self.disconnect("Lỗi USB: " + str(e))
            return False

    def disconnect(self, message, source="DISCONNECT"):
        self.stop(source)
        link, self.link = self.link, None
        if link:
            link.close()
        self.trace_event("disconnect", source=source, message=message)
        trace, self.trace = self.trace, None
        if trace:
            trace.close()
            if trace.error:
                self.log_text.set("Lỗi ghi log: " + trace.error)
        self.state = None
        self.local_reference = False
        self.model = None
        self.message.set(message)
        self.render()

    def sequence(self, lines):
        self.commands = list(lines)
        self.next_command()

    def next_command(self):
        if not self.commands or not self.link:
            return
        line = self.commands.pop(0)
        self.pending = (line.split()[0], time.monotonic())
        if self.pending[0] in ("INIT", "MOVE", "PATH"):
            self.task_kind = self.pending[0]
            self.active_job = self.job
            self.started = time.monotonic()
            self.last_keep = self.started
        self.send(line)

    def _drain_events(self, now):
        for _ in range(MAX_EVENTS_PER_POLL):
            if self.link is None:
                break
            try:
                event = self.link.events.get_nowait()
            except queue.Empty:
                break
            if event[0] == "error":
                self.disconnect("Lỗi USB: " + event[1])
                break
            if event[0] == "rx_rejected":
                self.trace_event(
                    "rx_rejected", received_monotonic=event[1], raw=event[2]
                )
                continue
            if event[0] == "ack":
                self._handle_ack(event[2], event[1])
                continue
            self._handle_status(event[2], event[1], now)

    def _handle_ack(self, ack, received):
        self.trace_event("ack", received_monotonic=received, ack=asdict(ack))
        if ack.session != self.link.session:
            return
        if self.path_upload and ack.op in ("PATH", "SEG", "GO"):
            try:
                if self.path_upload.ack(ack):
                    self.pending = None
            except ValueError as e:
                self.stop("MODEL")
                self.message.set("Đường vẽ bị từ chối: " + str(e))
            return
        if self.pending and ack.op == self.pending[0]:
            self.pending = None
            if not ack.ok:
                self.commands = []
                self.active_job = None
                self.local_reference = False
                self.expect_epoch = None
                self.task_kind = None
                self.message.set(
                    "STM32 từ chối: " + REASONS.get(ack.reason, ack.reason)
                )
            else:
                self.next_command()
        return

    def _handle_status(self, s, received, now):
        self.trace_event("status", received_monotonic=received, status=asdict(s))
        if s.session != self.link.session:
            return
        self.state = s
        self.rx = received
        self.job = max(self.job, s.job)
        if self.path_upload:
            self.path_upload.status(s)
        self.record_bk(s)
        if s.fault or s.conflicts or not s.referenced:
            self.local_reference = False
        if (
            self.expect_epoch is not None
            and s.referenced
            and s.epoch == self.expect_epoch
            and not s.busy
        ):
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
        if self.active_job is not None and s.job == self.active_job and not s.busy:
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
