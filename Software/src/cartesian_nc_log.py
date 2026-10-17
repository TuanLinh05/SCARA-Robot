"""
================================================================================
FILE: cartesian_nc_log.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Asynchronous Background JSONL Session Logger (`SessionLog`).
     Writes every transmitted command (`tx`), firmware ACK (`ack`), status
     frame (`status`), and safety event into a timestamped `.jsonl` file using
     a dedicated daemon thread so disk I/O never stalls the Tkinter GUI or
     the 80 ms USB heartbeat timer!

     HOW THIS WORKS (Producer-Consumer Pattern for 1st/2nd-Year Students):
     1. Main GUI Thread (Producer): Calls `log.record(...)`, which simply drops
        a Python dictionary into an in-memory `queue.Queue` in microseconds
        (`put_nowait`) and returns immediately without waiting for the hard drive.
     2. Background Thread (Consumer): Runs `_write()` in a loop, pulling items
        out of the queue, converting them to JSON text lines, and writing them
        to the `.jsonl` file on disk.

[VI] MỤC ĐÍCH:
     Bộ Ghi Nhật ký Phiên Làm việc Bất đồng bộ (`SessionLog`).
     Ghi lại toàn bộ lệnh gửi đi (`tx`), phản hồi (`ack`), trạng thái (`status`)
     và sự kiện an toàn vào file `.jsonl` thông qua một luồng nền (daemon thread)
     riêng biệt. Cách này giúp thao tác ghi ổ cứng không bao giờ làm khựng giao
     diện Tkinter hay làm trễ nhịp gửi lệnh KEEP 80 ms!

     NGUYÊN LÝ HOẠT ĐỘNG (Mô hình Producer-Consumer cho Sinh viên năm 1–2):
     1. Luồng Giao diện Chính (Producer - Nguồn phát): Gọi hàm `log.record(...)`
        để bỏ một từ điển dữ liệu vào hàng đợi trên RAM (`queue.Queue`) chỉ mất
        vài micro-giây (`put_nowait`), rồi quay lại điều khiển robot ngay.
     2. Luồng Chạy Ngầm (Consumer - Bộ ghi): Chạy vòng lặp `_write()`, lần lượt
        lấy dữ liệu từ hàng đợi ra, chuyển thành chuỗi JSON và ghi xuống ổ cứng.
================================================================================
"""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid


class SessionLog:
    """
    [EN] Non-blocking JSON Lines (`.jsonl`) logger backed by a bounded thread-safe queue.
    [VI] Lớp ghi log định dạng JSON Lines (`.jsonl`) không chặn (non-blocking) dùng hàng đợi an toàn luồng.
    """

    def __init__(self, directory):
        """
        [EN] Create the log directory, open a uniquely named `.jsonl` file, and
             start the background writer thread.
        [VI] Tạo thư mục chứa log, mở một file `.jsonl` có tên duy nhất theo thời gian
             và khởi chạy luồng ghi dữ liệu chạy ngầm.
        """
        # Step 1: Ensure the target log folder exists on disk.
        # Bước 1: Đảm bảo thư mục lưu file log đã tồn tại trên ổ cứng.
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)

        # Step 2: Build a unique filename using UTC timestamp, process ID, and random UUID.
        # Bước 2: Tạo tên file duy nhất gồm giờ UTC, mã tiến trình (PID) và chuỗi ngẫu nhiên UUID.
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        process_id = os.getpid()
        random_suffix = uuid.uuid4().hex[:8]
        filename = f"cartesian_{stamp}_{process_id}_{random_suffix}.jsonl"
        self.path = root / filename

        # Step 3: Open the file in exclusive creation mode ("x") with UTF-8 encoding.
        # Bước 3: Mở file ở chế độ tạo mới độc quyền ("x") với bảng mã tiếng Việt UTF-8.
        self.stream = self.path.open("x", encoding="utf-8", newline="\n")

        # Step 4: Initialize the thread-safe queue (holds up to 4096 events in RAM).
        # Bước 4: Khởi tạo hàng đợi an toàn luồng (chứa tối đa 4096 bản ghi trong RAM).
        self.queue = queue.Queue(maxsize=4096)
        self.error = None
        self.dropped = 0
        self.started = time.monotonic()
        self.closed = False

        # Step 5: Launch the background daemon thread that writes queued items to disk.
        # Bước 5: Khởi chạy luồng nền (daemon=True) chuyên lấy dữ liệu từ hàng đợi ghi ra đĩa.
        self.thread = threading.Thread(
            target=self._write,
            daemon=True,
            name="SCARA debug log",
        )
        self.thread.start()

    def record(self, kind, **fields):
        """
        [EN] Enqueue a log event without blocking the caller. If the 4096-item queue
             is momentarily full, increments `self.dropped` instead of stalling the GUI.
        [VI] Đưa một bản ghi vào hàng đợi ngay lập tức (không chờ). Nếu hàng đợi 4096
             phần tử bị đầy, chỉ tăng biến đếm `self.dropped` chứ không làm treo giao diện.
        """
        # Do not accept new records if the logger is already closed or encountered a disk error.
        # Không nhận thêm bản ghi mới nếu file log đã đóng hoặc ổ cứng đang báo lỗi.
        if self.closed or self.error:
            return

        # Compute current UTC timestamp string and elapsed time (seconds) since startup.
        # Tính chuỗi thời gian chuẩn UTC và thời gian đã trôi qua (giây) kể từ lúc mở app.
        utc_now_str = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        elapsed_seconds = round(time.monotonic() - self.started, 6)

        # Combine standard metadata with any extra keyword arguments (`**fields`).
        # Ghép thông tin thời gian chuẩn với các trường dữ liệu chi tiết (`**fields`).
        item = {
            "type": kind,
            "utc": utc_now_str,
            "elapsed_s": elapsed_seconds,
        }
        item.update(fields)

        # Push into queue without waiting (`put_nowait`).
        # Đẩy ngay vào hàng đợi không chờ đợi (`put_nowait`).
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        """
        [EN] Background consumer loop that serializes queued dicts to disk and flushes
             immediately on important events (or every 200 ms for routine telemetry).
        [VI] Vòng lặp chạy ngầm lấy dữ liệu từ hàng đợi ghi xuống đĩa và đẩy dữ liệu
             (flush) ngay khi có sự kiện quan trọng hoặc định kỳ mỗi 200 ms.
        """
        last_flush = time.monotonic()
        reported = 0
        try:
            while True:
                # Wait up to 0.2 seconds for the next log dictionary from the main thread.
                # Chờ tối đa 0.2 giây để lấy bản ghi tiếp theo từ luồng chính.
                try:
                    item = self.queue.get(timeout=0.2)
                except queue.Empty:
                    # Queue was idle for 200 ms -> flush buffered bytes to disk.
                    # Hàng đợi rảnh quá 200 ms -> đẩy dữ liệu đệm xuống ổ cứng.
                    self.stream.flush()
                    last_flush = time.monotonic()
                    continue

                # A `None` item is the "poison pill" signal telling this thread to stop.
                # Giá trị `None` là tín hiệu báo cho luồng ghi dừng vòng lặp và đóng file.
                if item is None:
                    break

                # If any events were dropped while the queue was full, log a notice first.
                # Nếu trước đó có bản ghi bị rớt do đầy hàng đợi, ghi một dòng thông báo.
                if self.dropped != reported:
                    newly_dropped_count = self.dropped - reported
                    dropped_notice = {
                        "type": "log_dropped",
                        "count": newly_dropped_count,
                    }
                    self.stream.write(json.dumps(dropped_notice) + "\n")
                    reported = self.dropped

                # Convert Python dictionary to a single compact JSON line and write it.
                # Chuyển từ điển Python thành một dòng chuỗi JSON gọn và ghi vào file.
                line = json.dumps(item, ensure_ascii=False, separators=(",", ":"))
                self.stream.write(line + "\n")

                # Flush immediately for critical events (anything except routine status/tx),
                # or at least once every 0.2 seconds during high-rate telemetry.
                # Đẩy xuống đĩa ngay nếu là sự kiện quan trọng (khác status/tx định kỳ),
                # hoặc định kỳ mỗi 0.2 giây khi đang truyền dữ liệu liên tục.
                is_important_event = item["type"] not in ("status", "tx")
                time_since_last_flush = time.monotonic() - last_flush
                if is_important_event or (time_since_last_flush > 0.2):
                    self.stream.flush()
                    last_flush = time.monotonic()

            self.stream.flush()
        except (OSError, TypeError, ValueError) as e:
            self.error = str(e)
        finally:
            try:
                self.stream.close()
            except OSError as e:
                self.error = str(e)

    def close(self):
        """
        [EN] Signal the writer thread to flush remaining items and close the file cleanly.
        [VI] Báo hiệu cho luồng ghi hoàn tất các bản ghi còn tồn đọng và đóng file an toàn.
        """
        if self.closed:
            return

        # Record final summary event before closing.
        # Ghi bản ghi tổng kết cuối phiên trước khi đóng file.
        self.record("log_end", dropped=self.dropped)
        self.closed = True

        # Send `None` sentinel into the queue so `_write` breaks out of its `while True` loop.
        # Đưa tín hiệu `None` vào hàng đợi để hàm `_write` thoát khỏi vòng lặp `while True`.
        try:
            self.queue.put(None, timeout=0.2)
        except queue.Full:
            self.error = "Hàng đợi log đầy khi đóng."
            return

        # Wait up to 1 second for the background thread to finish writing to disk.
        # Chờ tối đa 1 giây để luồng chạy ngầm ghi nốt dữ liệu và đóng file hoàn tất.
        self.thread.join(timeout=1)
        if self.thread.is_alive():
            self.error = "Ghi log chưa hoàn tất khi đóng."
