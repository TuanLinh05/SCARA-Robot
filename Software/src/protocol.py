"""
============================================================================
FILE: protocol.py
============================================================================
[EN] Framed USB Serial Transport & Priority Event Queue (`LineDecoder`, `SerialLink`)
     - `LineDecoder`: Reassembles arbitrary USB byte chunks into newline-delimited
       frames with overflow protection (`max_frame_bytes`).
     - `SerialLink`: Manages the background USB reader thread, thread-safe command
       writes, and a priority event queue that NEVER drops `"ack"` or `"error"`
       events even if flooded with 10 Hz `"status"` telemetry!

[VI] Giao vận Cổng Nối tiếp USB & Hàng đợi Sự kiện Ưu tiên (`LineDecoder`, `SerialLink`)
     - `LineDecoder`: Ghép các mảnh byte nhận từ USB thành từng dòng hoàn chỉnh
       phân cách bởi ký tự xuống dòng `\n` (`byte == 10`), có chống tràn bộ đệm.
     - `SerialLink`: Quản lý luồng đọc USB chạy ngầm, khóa an toàn khi gửi lệnh,
       và cơ chế hàng đợi ưu tiên: không bao giờ làm mất gói tin xác nhận `"ack"`
       hoặc báo lỗi `"error"` ngay cả khi hàng đợi đầy các gói `"status"`!

[DEBUG TIP / MẸO GỠ LỖI]:
- Look at `SerialLink._event()`: when `self.events` reaches 128 items, it scans
  for an older `"status"` item and evicts it so a newly arrived `"ack"` or `"error"`
  is ALWAYS delivered to the GUI state machine!
- Xem hàm `SerialLink._event()`: khi hàng đợi đủ 128 phần tử, hàm sẽ tìm và xóa bớt
  một gói `"status"` cũ để nhường chỗ cho gói `"ack"` hoặc `"error"`, đảm bảo máy
  trạng thái điều khiển trên PC không bao giờ bị treo chờ ACK!
============================================================================
"""

import queue
import secrets
import threading
import time


class LineDecoder:
    """
    [EN] Stateful byte-stream framer that splits incoming bytes on `\n` (ASCII 10).
    [VI] Bộ tách dòng từ luồng byte thô dựa trên ký tự xuống dòng `\n` (mã ASCII 10).
    """

    def __init__(
        self, parser=None, max_frame_bytes=1024, on_rejected=None, overflow_marker=None
    ):
        """
        [EN] Initialize the line decoder buffer and frame-size safety limits.
        [VI] Khởi tạo bộ đệm tách dòng và giới hạn kích thước khung tin an toàn.
        """
        self.buffer = bytearray()
        self.discard = False
        if parser is not None:
            self.parser = parser
        else:
            self.parser = lambda raw: raw
        self.max_frame_bytes = max_frame_bytes
        self.on_rejected = on_rejected
        self.overflow_marker = overflow_marker

    def feed(self, data):
        """
        [EN] Feed a `bytes`/`bytearray` chunk and return a list of parsed messages.
             - Bytes are accumulated in `self.buffer` until `\n` (`byte == 10`).
             - If a line exceeds `max_frame_bytes` before `\n`, `self.discard` is set
               to `True` so the corrupted oversized line is safely dropped.
        [VI] Nạp một mảng byte nhận từ cổng COM và trả về danh sách các bản tin hoàn chỉnh.
             - Các byte được gom vào `self.buffer` cho đến khi gặp ký tự `\n` (`byte == 10`).
             - Nếu một dòng dài quá `max_frame_bytes` trước khi gặp `\n`, cờ `self.discard`
               được bật lên `True` để hủy bỏ an toàn toàn bộ dòng bị tràn đó.
        """
        messages = []
        for byte in data:
            if byte == 10:  # [EN] '\n' newline character / [VI] Ký tự xuống dòng '\n'
                if not self.discard:
                    raw = bytes(self.buffer)
                    message = self.parser(raw)
                    if message is not None:
                        messages.append(message)
                    elif self.on_rejected:
                        self.on_rejected(raw)
                elif self.on_rejected and self.overflow_marker is not None:
                    self.on_rejected(self.overflow_marker)

                # Reset state for the next incoming line
                # Đặt lại bộ đệm để đón dòng dữ liệu tiếp theo
                self.buffer.clear()
                self.discard = False

            elif not self.discard:
                if len(self.buffer) >= self.max_frame_bytes:
                    # Line exceeded maximum allowed length -> enter discard mode until next '\n'
                    # Dòng vượt quá độ dài tối đa -> xóa bộ đệm và bỏ qua cho tới dấu '\n' kế tiếp
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(byte)

        return messages


class SerialLink:
    """
    [EN] Thread-safe USB serial connection wrapper with automatic `STOP` + `HELLO`
         handshake on open and `STOP` on close.
    [VI] Lớp quản lý kết nối cổng COM/USB an toàn đa luồng, tự động gửi `STOP` +
         `HELLO <session>` khi mở cổng và gửi `STOP` an toàn trước khi đóng cổng.
    """

    def __init__(self, port, serial_factory=None):
        """
        [EN] Open the serial port at 115200 baud, generate a random 32-bit session ID,
             transmit the initial `STOP` + `HELLO <session>` handshake, and start the
             background reader thread.
        [VI] Mở cổng COM ở tốc độ 115200 baud, sinh mã phiên 32-bit ngẫu nhiên,
             gửi chuỗi bắt tay `STOP` + `HELLO <session>`, và khởi chạy luồng đọc nền.
        """
        if serial_factory is None:
            import serial

            serial_factory = serial.Serial

        self.events = queue.Queue(maxsize=128)
        self.lock = threading.Lock()
        self.closed = threading.Event()
        # Generate a random non-zero 32-bit session ID (1 .. 0xFFFFFFFE)
        # Sinh mã phiên (session ID) 32-bit ngẫu nhiên khác 0 (1 .. 0xFFFFFFFE)
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
        """
        [EN] Enqueue an incoming event, evicting an older `"status"` frame if needed
             so critical `"ack"` and `"error"` events are never lost.
        [VI] Đưa sự kiện vào hàng đợi; nếu đầy sẽ tự động loại bỏ bớt một gói `"status"`
             cũ để không bao giờ làm rơi gói `"ack"` hay `"error"`.
        """
        with self.events.mutex:
            queue_is_full = (
                self.events.maxsize > 0
                and len(self.events.queue) >= self.events.maxsize
            )
            if queue_is_full:
                # Try to find and remove the oldest "status" packet to free one slot
                # Tìm và xóa gói "status" cũ nhất trong hàng đợi để nhường 1 chỗ trống
                dropped = False
                for idx, item in enumerate(self.events.queue):
                    is_status_item = (
                        isinstance(item, tuple) and len(item) > 0 and item[0] == "status"
                    )
                    if is_status_item:
                        del self.events.queue[idx]
                        dropped = True
                        break

                # If no old "status" could be evicted and the new event itself is "status", drop it
                # Nếu không có gói "status" cũ nào để xóa và gói mới cũng là "status" thì bỏ qua gói mới
                incoming_is_status = (
                    isinstance(event, tuple) and len(event) > 0 and event[0] == "status"
                )
                if not dropped and incoming_is_status:
                    return

            self.events.queue.append(event)
            self.events.unfinished_tasks += 1
            self.events.not_empty.notify()

    def _make_decoder(self):
        """
        [EN] Factory method returning the line decoder instance for `_read()`.
        [VI] Hàm khởi tạo bộ tách dòng `LineDecoder` dùng cho luồng đọc `_read()`.
        """
        return LineDecoder()

    def _message_kind(self, message):
        """
        [EN] Classify a decoded message for priority queueing (`"status"` by default).
        [VI] Phân loại bản tin đã giải mã để xếp hàng đợi ưu tiên (mặc định là `"status"`).
        """
        return "status"

    def _read(self):
        """
        [EN] Background thread loop polling up to 256 bytes at a time from USB serial.
        [VI] Vòng lặp luồng nền đọc liên tục tối đa 256 byte mỗi lần từ cổng USB.
        """
        decoder = self._make_decoder()
        try:
            while not self.closed.is_set():
                data = self.serial.read(256)
                for message in decoder.feed(data):
                    kind = self._message_kind(message)
                    timestamp = time.monotonic()
                    self._event((kind, timestamp, message))
        except Exception as error:
            if not self.closed.is_set():
                self._event(("error", str(error)))

    def send(self, line):
        """
        [EN] Thread-safely transmit a single ASCII command line terminated by `\n`.
        [VI] Gửi một dòng lệnh ASCII kết thúc bằng `\n` xuống mạch (có khóa `self.lock`).
        """
        payload = (line + "\n").encode("ascii")
        with self.lock:
            if self.closed.is_set():
                raise OSError("Cổng COM đã đóng.")
            bytes_written = self.serial.write(payload)
            if bytes_written != len(payload):
                raise OSError("Không gửi được đầy đủ lệnh USB.")

    def close(self):
        """
        [EN] Send a best-effort `STOP`, close the serial port, and join the reader thread.
        [VI] Gửi lệnh `STOP` bảo vệ trước khi đóng cổng COM và kết thúc luồng đọc.
        """
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
