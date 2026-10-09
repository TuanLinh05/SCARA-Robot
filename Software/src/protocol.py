"""SCARA Z Jog v2: both physical limits are required in telemetry."""

from dataclasses import dataclass, MISSING
import json
import math
import queue
import secrets
import threading
import time

STEPS_PER_REV = 3200
MIN_RATE, MAX_RATE = 100, 6400


def pulse_rate(speed_mm_s: float, lead_mm: float) -> int:
    if not all(math.isfinite(x) for x in (speed_mm_s, lead_mm)):
        raise ValueError("Thông số phải là số hữu hạn.")
    if not 0.5 <= lead_mm <= 20 or not 0.05 <= speed_mm_s <= 10:
        raise ValueError("Hành trình: 0,5–20 mm/vòng; tốc độ: 0,05–10 mm/s.")
    rate = round(speed_mm_s * STEPS_PER_REV / lead_mm)
    if not MIN_RATE <= rate <= MAX_RATE:
        raise ValueError(
            f"Tốc độ cần {rate:,} xung/s. Chọn tốc độ trong khoảng "
            f"{MIN_RATE * lead_mm / STEPS_PER_REV:.3f}–"
            f"{MAX_RATE * lead_mm / STEPS_PER_REV:.3f} mm/s."
        )
    return rate


@dataclass(frozen=True)
class Status:
    session: int
    job: int
    up_ms: int
    dir: int
    pos: int
    total: int
    move: int
    target: int
    rate: int
    adc: int
    top: int
    adc_bottom: int
    bottom: int
    ready_top: int
    ready_bottom: int
    error_top: int
    error_bottom: int
    conflict: int
    ready: int
    error: int
    reason: str
    # Optional metadata; retain compatibility with the original v2 firmware.
    build: str = ""
    pulse_us: int = 0
    motor_error: int = -1
    pul_pin: int = -2
    dir_pin: int = -2
    ena_pin: int = -2

    @classmethod
    def parse(cls, line: bytes):
        try:
            value = json.loads(line)
            if (
                value.get("type") != "status"
                or value.get("protocol") != 2
                or value.get("fw") != "SCARA_Z_JOG_V2"
                or value.get("spr") != STEPS_PER_REV
            ):
                return None
            fields = cls.__dataclass_fields__
            required = [k for k, field in fields.items() if field.default is MISSING]
            if any(type(value.get(k)) is not int for k in required if k != "reason"):
                return None
            if not isinstance(value.get("reason"), str) or len(value["reason"]) > 40:
                return None
            if (
                value["dir"] not in (-1, 0, 1)
                or any(
                    value[k] not in (0, 1)
                    for k in (
                        "top",
                        "bottom",
                        "ready",
                        "ready_top",
                        "ready_bottom",
                        "conflict",
                    )
                )
                or value["ready"] != int(value["ready_top"] and value["ready_bottom"])
                or not 0 <= value["adc"] <= 4095
                or not 0 <= value["adc_bottom"] <= 4095
                or any(
                    not 0 <= value[k] <= 0xFFFFFFFF
                    for k in ("session", "job", "up_ms", "move", "target", "rate")
                )
                or not 0 <= value["total"] <= 0xFFFFFFFFFFFFFFFF
                or not -(1 << 63) <= value["pos"] < (1 << 63)
            ):
                return None
            if "build" in value and (
                not isinstance(value["build"], str)
                or len(value["build"]) > 64
                or any(
                    not (c.isascii() and (c.isalnum() or c in "_-"))
                    for c in value["build"]
                )
            ):
                return None
            for name, low, high in (
                ("pulse_us", 0, 10000),
                ("motor_error", -1, 4),
                ("pul_pin", -4095, 1),
                ("dir_pin", -4095, 1),
                ("ena_pin", -4095, 1),
            ):
                if name in value and (
                    type(value[name]) is not int or not low <= value[name] <= high
                ):
                    return None
            return cls(**{k: value[k] for k in fields if k in value})
        except (ValueError, TypeError, AttributeError, UnicodeError):
            return None


class LineDecoder:
    def __init__(
        self, parser=None, max_frame_bytes=1024, on_rejected=None, overflow_marker=None
    ):
        self.buffer = bytearray()
        self.discard = False
        self.parser = parser or Status.parse
        self.max_frame_bytes = max_frame_bytes
        self.on_rejected = on_rejected
        self.overflow_marker = overflow_marker

    def feed(self, data):
        messages = []
        for byte in data:
            if byte == 10:
                if not self.discard:
                    raw = bytes(self.buffer)
                    message = self.parser(raw)
                    if message is not None:
                        messages.append(message)
                    elif self.on_rejected:
                        self.on_rejected(raw)
                elif self.on_rejected and self.overflow_marker is not None:
                    self.on_rejected(self.overflow_marker)
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) >= self.max_frame_bytes:
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(byte)
        return messages


class SerialLink:
    def __init__(self, port, serial_factory=None):
        if serial_factory is None:
            import serial

            serial_factory = serial.Serial
        self.events = queue.Queue(maxsize=128)
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.session = secrets.randbelow(0xFFFFFFFE) + 1
        self.serial = serial_factory(
            port=port, baudrate=115200, timeout=0.02, write_timeout=0.1
        )
        try:
            self.serial.dtr = True
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.send("STOP")
            self.send(f"HELLO {self.session}")
        except Exception:
            self.serial.close()
            raise
        self.thread = threading.Thread(
            target=self._read, daemon=True, name="USB serial"
        )
        self.thread.start()

    def _event(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.events.put_nowait(event)

    def _make_decoder(self):
        return LineDecoder()

    def _message_kind(self, message):
        return "status"

    def _read(self):
        decoder = self._make_decoder()
        try:
            while not self.closed.is_set():
                data = self.serial.read(256)
                for message in decoder.feed(data):
                    self._event(
                        (self._message_kind(message), time.monotonic(), message)
                    )
        except Exception as error:
            if not self.closed.is_set():
                self._event(("error", str(error)))

    def send(self, line):
        payload = (line + "\n").encode("ascii")
        with self.lock:
            if self.closed.is_set():
                raise OSError("Cổng COM đã đóng.")
            if self.serial.write(payload) != len(payload):
                raise OSError("Không gửi được đầy đủ lệnh USB.")

    def close(self):
        if self.closed.is_set():
            return
        try:
            self.send("STOP")
        except Exception:
            pass
        with self.lock:
            self.closed.set()
            self.serial.close()
        self.thread.join(timeout=0.2)
