"""Offline GUI fixture. Real calibration/ISR correctness is tested by C harnesses."""

from dataclasses import replace
import queue
import threading
import time
from cartesian_nc_protocol import NCStatus, Ack, BUILD


class DemoLink:
    def __init__(self, port):
        self.session = 7
        self.events = queue.Queue()
        self.closed = False
        self.job = 0
        self.epoch = 0
        self.stage = 0
        self.busy = False
        self.referenced = False
        self.holding = 7
        self.pol = 3
        self.pos = (20000, 0, 0)
        self.goal = self.pos
        self.phase = (0, 0, 0)
        self.since = time.monotonic()
        self.ends = 0
        self.initializing = False
        self.poll_due = 0
        self.factor = (1638400, 13653, 40960)
        self.range = (40000, 2400, 7200)
        self.reason = "unreferenced"
        self.path_active = False
        self.path_started = False
        self.path_queue = []
        self.path_total = self.path_received = self.path_done = 0
        self.emit()
        self.closed_event = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while not self.closed_event.wait(0.08):
            self.advance()

    def emit(self):
        v = NCStatus(
            session=self.session,
            job=self.job,
            up_ms=int((time.monotonic() - self.since) * 1000),
            busy=int(self.busy),
            referenced=int(self.referenced),
            epoch=self.epoch,
            stage=self.stage,
            pos=self.pos,
            goal=self.goal,
            total=(80000, 7200, 28800),
            range=self.range if self.referenced or self.busy else (0, 0, 0),
            factor=self.factor if self.referenced or self.busy else (0, 0, 0),
            n1=self.range if self.referenced else (0, 0, 0),
            n2=self.range if self.referenced else (0, 0, 0),
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
            mode=2 if self.initializing else 1 if self.busy else 0,
            selected=0,
            pps=800 if self.busy else 0,
            direction=(0, 0, 0),
            dir_levels=0,
            coupling_ppm=1333333,
            beta_q=4194304,
            probe_da=64 if self.referenced else 0,
            probe_db=256 if self.referenced else 0,
            coupling_ready=int(self.referenced),
            reason=self.reason,
            build=BUILD,
        )
        v = replace(
            v,
            path=(
                self.path_total,
                self.path_received,
                self.path_done,
                32 - len(self.path_queue),
            ),
        )
        self.events.put(("status", time.monotonic(), v))

    def send(self, line):
        if self.closed:
            raise OSError("Demo đóng.")
        w = line.split()
        op = w[0]
        if op == "HELLO":
            self.session = int(w[1])
        if op == "STOP":
            if self.busy:
                self.referenced = False
                self.stage = 9 if self.initializing else self.stage
            self.busy = self.initializing = False
            self.path_active = self.path_started = False
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
        if op in ("GEOM", "SPAN", "PARK", "TUNE", "POL", "COUPLE"):
            self.referenced = False
        if op == "POL":
            bit = 1 << ("Z", "J1", "J2").index(w[2])
            self.pol = (self.pol | bit) if int(w[3]) else (self.pol & ~bit)
        if op == "HOLD":
            bit = 1 << (1 if w[2] == "J1" else 2)
            self.holding = (self.holding | bit) if int(w[3]) else (self.holding & ~bit)
            self.referenced = False
        if op == "INIT":
            self.job = int(w[2])
            self.initializing = self.busy = True
            self.referenced = False
            self.stage = 1
            self.reason = "initializing"
            self.ends = time.monotonic() + 1.3
        if op == "MOVE":
            self.job = int(w[2])
            self.goal = tuple(map(int, w[3:6]))
            self.busy = True
            self.initializing = False
            self.ends = time.monotonic() + 0.5
            self.reason = "moving"
        if op == "PATH":
            self.job = int(w[2])
            self.path_total = int(w[3])
            self.path_received = self.path_done = 0
            self.path_queue = []
            self.path_active = self.busy = True
            self.path_started = False
            self.reason = "path_loading"
        if op == "SEG":
            assert (
                self.path_active
                and int(w[3]) == self.path_received
                and len(self.path_queue) < 32
            )
            self.path_queue.append(tuple(map(int, w[4:7])))
            self.path_received += 1
        if op == "GO":
            self.path_started = True
            self.reason = "path_running"
        if op == "KEEP":
            self.advance()
            return
        self.events.put(
            ("ack", time.monotonic(), Ack(self.session, self.job, op, 1, "ok"))
        )
        self.emit()

    def advance(self):
        now = time.monotonic()
        if self.path_active:
            if self.path_started and self.path_queue:
                self.pos = self.goal = self.path_queue.pop(0)
                self.path_done += 1
                if self.path_done == self.path_total:
                    self.busy = self.path_active = self.path_started = False
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
            self.busy = self.initializing = False
        elif self.initializing:
            self.stage = min(7, 1 + int((1.3 - (self.ends - now)) * 5))
            self.phase = (4, 4, 4)
        self.emit()

    def close(self):
        self.closed = True
        self.closed_event.set()
        self.thread.join(timeout=0.2)
