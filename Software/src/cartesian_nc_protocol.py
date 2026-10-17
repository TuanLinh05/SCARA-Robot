"""
================================================================================
FILE: cartesian_nc_protocol.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Protocol v5 parser and serial link specialization for the SCARA Cartesian
     NC Firmware (`SCARA_CARTESIAN_NC_V5`, build `SCARA_CART_NC_HOME_V5_R9`).
     Validates incoming JSON `status` and `ack` packets against strict range,
     type, and state-machine invariants so corrupted or mismatched firmware
     frames are rejected immediately before reaching the GUI or planner.

[VI] MỤC ĐÍCH:
     Bộ phân tích giao thức v5 và lớp kết nối nối tiếp chuyên biệt cho Firmware
     SCARA Cartesian NC (`SCARA_CARTESIAN_NC_V5`, bản dựng `SCARA_CART_NC_HOME_V5_R9`).
     Kiểm tra nghiêm ngặt các gói JSON `status` và `ack` nhận được về kiểu dữ liệu,
     giới hạn giá trị và tính nhất quán của máy trạng thái để loại bỏ ngay các
     khung tin lỗi hoặc sai phiên bản firmware trước khi đưa vào GUI.

[DEBUG TIP / MẸO GỠ LỖI]:
     - If the GUI shows "USB/status quá hạn" (status expired) even though the
       board is plugged in, check the `rx_rejected` events in the `.jsonl` log!
       Usually either `BUILD` does not match the flashed STM32 firmware or one
       of the range checks in `NCStatus.parse()` returned `None`.
     - Nếu giao diện báo "USB/status quá hạn" dù đã cắm mạch, hãy kiểm tra sự kiện
       `rx_rejected` trong file log `.jsonl`! Thường do hằng số `BUILD` không khớp
       với firmware nạp trên STM32 hoặc một điều kiện kiểm tra biên trong
       `NCStatus.parse()` trả về `None`.
================================================================================"""

from dataclasses import dataclass
import json
import time
from protocol import SerialLink, LineDecoder

# ==============================================================================
# 1. PROTOCOL VERSION CONSTANTS / HẰNG SỐ PHIÊN BẢN GIAO THỨC
# ==============================================================================

# [EN] Exact firmware build string required by this GUI version.
# [VI] Chuỗi định danh bản dựng firmware chính xác mà phiên bản GUI này yêu cầu.
BUILD = "SCARA_CART_NC_HOME_V5_R9"


def _is_valid_ascii_identifier(value, max_len=64):
    """
    [EN] Check that `value` is a non-empty ASCII string (up to `max_len` chars)
         containing only letters, digits, or underscores `_`.
    [VI] Kiểm tra `value` là chuỗi ASCII khác rỗng (tối đa `max_len` ký tự)
         chỉ gồm chữ cái, chữ số hoặc dấu gạch dưới `_`.
    """
    if not isinstance(value, str) or not value or len(value) > max_len:
        return False
    for ch in value:
        if not (ch.isascii() and (ch.isalnum() or ch == "_")):
            return False
    return True


# ==============================================================================
# 2. TELEMETRY & ACK DATA STRUCTURES / CẤU TRÚC DỮ LIỆU TELEMETRY & XÁC NHẬN
# ==============================================================================


@dataclass(frozen=True)
class NCStatus:
    """
    [EN] Immutable snapshot of a validated Protocol v5 `status` frame from STM32.
         Contains all 3-axis positions, calibration factors, 6-switch bitmasks,
         DDA path FIFO state, and hardware timer diagnostics.
    [VI] Bản chụp bất biến của khung tin `status` Giao thức v5 hợp lệ từ STM32.
         Chứa toàn bộ vị trí 3 trục, hệ số hiệu chuẩn, mặt nạ bit 6 công tắc,
         trạng thái bộ đệm đường chạy DDA và chẩn đoán bộ định thời phần cứng.
    """

    session: int
    job: int
    up_ms: int
    busy: int
    referenced: int
    epoch: int
    stage: int
    pos: tuple
    goal: tuple
    total: tuple
    range: tuple
    factor: tuple
    n1: tuple
    n2: tuple
    phase: tuple
    cal_error: tuple
    failed_phase: tuple
    switches: int
    ready: int
    holding: int
    pol: int
    conflicts: int
    tick: int
    ticks: int
    motor_error: int
    fault: int
    timer_late: int
    timer_errno: int
    raw: int
    errors: int
    input_wait: int
    recovered: int
    input_axis: int
    input_kind: int
    input_bits: int
    input_at: int
    brief: tuple
    idle_ignored: int
    gate_wait: int
    gate_axis: int
    false_hits: tuple
    mode: int
    selected: int
    pps: int
    direction: tuple
    dir_levels: int
    coupling_ppm: int
    beta_q: int
    probe_da: int
    probe_db: int
    coupling_ready: int
    reason: str
    build: str
    path: tuple = (0, 0, 0, 32)
    starved: int = 0
    beta_measured: int = 0
    span_md: tuple = (0, 0)
    comp_pulses: tuple = (0, 0)

    @classmethod
    def parse(cls, v):
        """
        [EN] Validate dictionary `v` decoded from JSON and construct an `NCStatus`
             instance, or return `None` if any field violates protocol invariants.
        [VI] Kiểm tra từ điển `v` giải mã từ JSON và khởi tạo đối tượng `NCStatus`,
             hoặc trả về `None` nếu bất kỳ trường nào vi phạm quy tắc giao thức.
        """
        # 1. Verify firmware identifier and build revision
        # 1. Kiểm tra định danh firmware và mã bản dựng
        if v.get("fw") != "SCARA_CARTESIAN_NC_V5" or v.get("build") != BUILD:
            return None

        # Provide backward-compatible defaults for optional telemetry fields
        # Cung cấp giá trị mặc định tương thích ngược cho các trường tùy chọn
        v = {
            "beta_measured": 0,
            "span_md": [0, 0],
            "comp_pulses": [0, 0],
            **v,
        }

        # 2. Validate field types and array lengths
        # 2. Kiểm tra kiểu dữ liệu và độ dài của các trường mảng
        arrays = (
            "pos",
            "goal",
            "total",
            "range",
            "factor",
            "n1",
            "n2",
            "phase",
            "cal_error",
            "failed_phase",
            "brief",
            "false_hits",
            "direction",
            "path",
            "span_md",
            "comp_pulses",
        )
        for k in cls.__dataclass_fields__:
            field_val = v.get(k)
            if k in arrays:
                if k == "path":
                    expected_len = 4
                elif k in ("span_md", "comp_pulses"):
                    expected_len = 2
                else:
                    expected_len = 3

                if not isinstance(field_val, list) or len(field_val) != expected_len:
                    return None
                for item in field_val:
                    if type(item) is not int:
                        return None
            elif k in ("reason", "build"):
                if not _is_valid_ascii_identifier(field_val, max_len=64):
                    return None
            elif type(field_val) is not int:
                return None

        # 3. Validate boolean flags (must be 0 or 1)
        # 3. Kiểm tra các cờ nhị phân (phải là 0 hoặc 1)
        for flag_key in ("busy", "referenced", "fault", "input_wait", "gate_wait"):
            if v[flag_key] not in (0, 1):
                return None

        # 4. Validate state machine stage, switch bitmask, mode, and pulse rate
        # 4. Kiểm tra giai đoạn máy trạng thái, mặt nạ công tắc, chế độ và tốc độ xung
        if not (0 <= v["stage"] <= 12) or not (0 <= v["switches"] <= 63):
            return None
        if (
            not (0 <= v["mode"] <= 3)
            or not (0 <= v["selected"] <= 2)
            or not (0 <= v["pps"] <= 6400)
        ):
            return None

        # High pulse rates (>1600 pps) are only allowed for Z axis during Z-homing or Z-only moves
        # Tốc độ xung cao (>1600 pps) chỉ được phép cho trục Z khi đang tìm gốc Z hoặc chỉ chạy Z
        if v["pps"] > 1600:
            is_z_homing = (
                v["mode"] == 2
                and v["selected"] == 0
                and v["stage"] == 1
                and v["busy"]
                and not v["referenced"]
            )
            is_z_only_move = (
                v["mode"] == 1
                and v["stage"] == 8
                and v["busy"]
                and v["referenced"]
                and v["direction"][1] == 0
                and v["direction"][2] == 0
            )
            if not (is_z_homing or is_z_only_move):
                return None

        # 5. Validate motor directions, belt coupling, and calibration probe metrics
        # 5. Kiểm tra chiều quay động cơ, truyền động dây đai và thông số dò hiệu chuẩn
        for dir_val in v["direction"]:
            if dir_val not in (-1, 0, 1):
                return None
        if not (0 <= v["dir_levels"] <= 7):
            return None
        if (
            not (-2000000 <= v["coupling_ppm"] <= 2000000)
            or abs(v["beta_q"]) > 67108864
            or abs(v["beta_measured"]) > 67108864
        ):
            return None
        for span_val in v["span_md"]:
            if not (0 <= span_val <= 360000):
                return None
        for comp_val in v["comp_pulses"]:
            if not (0 <= comp_val <= 0xFFFFFFFF):
                return None
        if (
            abs(v["probe_da"]) > 512
            or abs(v["probe_db"]) > 8000000
            or v["coupling_ready"] not in (0, 1)
        ):
            return None

        # 6. Validate axis bitmasks (Z holding must always be bit 0 = 1) and counters
        # 6. Kiểm tra mặt nạ bit các trục (trục Z luôn giữ lực ở bit 0 = 1) và bộ đếm
        for mask_key in ("ready", "holding", "pol", "conflicts", "errors"):
            if not (0 <= v[mask_key] <= 7):
                return None
        if not (v["holding"] & 1):
            return None

        u32_counter_keys = (
            "session",
            "job",
            "up_ms",
            "epoch",
            "timer_late",
            "recovered",
            "input_at",
        )
        for counter_key in u32_counter_keys:
            if not (0 <= v[counter_key] <= 0xFFFFFFFF):
                return None
        for brief_val in v["brief"]:
            if not (0 <= brief_val <= 0xFFFFFFFF):
                return None

        # 7. Validate switch input recovery and safety gate states
        # 7. Kiểm tra trạng thái khôi phục công tắc và cổng an toàn
        if (
            not (-1 <= v["input_axis"] <= 2)
            or not (0 <= v["input_kind"] <= 7)
            or not (0 <= v["raw"] <= 63)
            or not (0 <= v["input_bits"] <= 63)
        ):
            return None
        if not (0 <= v["gate_axis"] <= 2) or not (0 <= v["idle_ignored"] <= 0xFFFFFFFF):
            return None
        for hit_count in v["false_hits"]:
            if not (0 <= hit_count <= 0xFFFFFFFF):
                return None

        if v["gate_wait"] and (not v["busy"] or v["referenced"]):
            return None
        if v["input_wait"] and (
            not v["busy"] or v["referenced"] or v["stage"] in (0, 8, 9)
        ):
            return None

        # 8. Validate step coordinates, calibration ranges, and DDA path FIFO counters
        # 8. Kiểm tra tọa độ xung, dải hiệu chuẩn và bộ đếm FIFO đường chạy DDA
        for coord_key in ("pos", "goal"):
            for step_coord in v[coord_key]:
                if abs(step_coord) > 8000000:
                    return None
        for total_steps in v["total"]:
            if not (0 <= total_steps <= 0xFFFFFFFFFFFFFFFF):
                return None
        for sweep_key in ("range", "n1", "n2"):
            for sweep_count in v[sweep_key]:
                if not (0 <= sweep_count <= 1000000):
                    return None
        for factor_q10 in v["factor"]:
            if not (0 <= factor_q10 <= 100000000):
                return None
        for phase_key in ("phase", "failed_phase"):
            for phase_id in v[phase_key]:
                if not (0 <= phase_id <= 11):
                    return None
        for cal_err in v["cal_error"]:
            if not (0 <= cal_err <= 9):
                return None
        if (
            not (0 <= v["motor_error"] <= 5)
            or not (0 <= v["tick"] <= v["ticks"] <= 1000000)
        ):
            return None

        total, received, done, free = v["path"]
        if (
            not (0 <= done <= received <= total <= 500)
            or not (0 <= free <= 32)
            or not (0 <= v["starved"] <= 0xFFFFFFFF)
        ):
            return None

        # 9. Cross-check high-level state invariants (`referenced` and `busy`)
        # 9. Kiểm tra chéo tính nhất quán của trạng thái cấp cao (`referenced` và `busy`)
        if v["referenced"]:
            if (
                v["stage"] != 8
                or v["fault"]
                or v["conflicts"]
                or v["errors"]
                or v["holding"] != 7
            ):
                return None
            for rng in v["range"]:
                if rng < 64:
                    return None
            for fac in v["factor"]:
                if fac <= 0:
                    return None

        if v["busy"] and (not v["session"] or v["fault"] or v["holding"] != 7):
            return None

        # Convert list fields to immutable tuples before instantiating `NCStatus`
        # Chuyển đổi các trường danh sách sang tuple bất biến trước khi khởi tạo `NCStatus`
        parsed_fields = {}
        for k in cls.__dataclass_fields__:
            if k in arrays:
                parsed_fields[k] = tuple(v[k])
            else:
                parsed_fields[k] = v[k]
        return cls(**parsed_fields)


@dataclass(frozen=True)
class Ack:
    """
    [EN] Immutable command acknowledgment packet (`type == "ack"`) from STM32.
    [VI] Gói tin xác nhận lệnh bất biến (`type == "ack"`) gửi lên từ STM32.
    """

    session: int
    job: int
    op: str
    ok: int
    reason: str


# ==============================================================================
# 3. FRAME PARSER & SERIAL LINK / HÀM GIẢI MÃ KHUNG TIN & KẾT NỐI SERIAL
# ==============================================================================


def parse_message(raw):
    """
    [EN] Parse a single newline-delimited JSON byte string into either an
         `NCStatus` or `Ack` object. Returns `None` if malformed or invalid.
    [VI] Giải mã một dòng byte JSON thành đối tượng `NCStatus` hoặc `Ack`.
         Trả về `None` nếu khung tin bị hỏng hoặc không hợp lệ.
    """
    try:
        v = json.loads(raw)
        if (
            not isinstance(v, dict)
            or type(v.get("protocol")) is not int
            or v["protocol"] != 5
        ):
            return None

        msg_type = v.get("type")
        if msg_type == "status":
            return NCStatus.parse(v)

        if msg_type == "ack":
            for int_key in ("session", "job", "ok"):
                if type(v.get(int_key)) is not int:
                    return None
            if v["ok"] not in (0, 1):
                return None
            if not (0 <= v["session"] <= 0xFFFFFFFF) or not (0 <= v["job"] <= 0xFFFFFFFF):
                return None
            for str_key in ("op", "reason"):
                if not _is_valid_ascii_identifier(v.get(str_key), max_len=64):
                    return None

            return Ack(
                session=v["session"],
                job=v["job"],
                op=v["op"],
                ok=v["ok"],
                reason=v["reason"],
            )
    except (ValueError, TypeError, KeyError, UnicodeError):
        pass
    return None


class NCDecoder(LineDecoder):
    """
    [EN] LineDecoder specialized for Protocol v5 (maximum 1536 bytes per JSON line).
    [VI] Bộ giải mã từng dòng chuyên biệt cho Giao thức v5 (tối đa 1536 byte mỗi dòng JSON).
    """

    def __init__(self, on_rejected=None):
        """
        [EN] Configure `LineDecoder` with `parse_message` and 1536-byte frame limit.
        [VI] Cấu hình `LineDecoder` với hàm `parse_message` và giới hạn khung 1536 byte.
        """
        super().__init__(
            parser=parse_message,
            max_frame_bytes=1536,
            on_rejected=on_rejected,
            overflow_marker=b"frame_larger_than_1536_bytes",
        )


class NCLink(SerialLink):
    """
    [EN] SerialLink specialization that decodes `NCStatus` and `Ack` frames and
         routes rejected frames to the session event queue for diagnostic logging.
    [VI] Lớp kế thừa SerialLink giúp giải mã khung tin `NCStatus` và `Ack`, đồng thời
         đẩy các khung tin bị từ chối vào hàng đợi sự kiện để ghi log chẩn đoán.
    """

    def _make_decoder(self):
        """
        [EN] Create an `NCDecoder` that records any rejected line as an `"rx_rejected"` event.
        [VI] Tạo `NCDecoder` tự động ghi nhận mọi dòng lỗi thành sự kiện `"rx_rejected"`.
        """
        def _on_rejected_frame(raw):
            decoded_text = raw.decode("utf-8", errors="replace")
            self._event(("rx_rejected", time.monotonic(), decoded_text))

        return NCDecoder(_on_rejected_frame)

    def _message_kind(self, message):
        """
        [EN] Return `"status"` for `NCStatus` packets and `"ack"` for `Ack` packets.
        [VI] Trả về `"status"` nếu là gói `NCStatus`, ngược lại trả về `"ack"`.
        """
        if isinstance(message, NCStatus):
            return "status"
        return "ack"
