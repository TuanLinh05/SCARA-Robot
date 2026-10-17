"""
============================================================================
FILE: cartesian_nc_policy.py
============================================================================
[EN] Pure Timing & Hardware Readiness Policies (Independent of Tkinter & Serial)
     Defines watchdog intervals, timeout thresholds, and pure boolean predicates
     for checking whether firmware telemetry is fresh and hardware is ready.

[VI] Các Hằng số Thời gian & Quy tắc Kiểm tra Sẵn sàng Phần cứng
     Định nghĩa chu kỳ gửi gói tin giữ kết nối (heartbeat), ngưỡng quá hạn (timeout),
     và các hàm kiểm tra độ tươi của dữ liệu cũng như trạng thái sẵn sàng của phần cứng
     (tách biệt hoàn toàn khỏi giao diện Tkinter và cổng COM để dễ viết Unit Test).
============================================================================
"""

# [EN] Communication timing constants (seconds and milliseconds):
#      - STATUS_TIMEOUT_S   : Max age of firmware status packet (0.8 s; firmware sends at 10 Hz).
#      - HANDSHAKE_TIMEOUT_S: Max wait for first status packet after opening COM port (3.0 s).
#      - ACK_TIMEOUT_S      : Max wait for command ACK reply from firmware (1.0 s).
#      - KEEP_INTERVAL_S    : Host heartbeat transmit interval (0.08 s = 80 ms, well under 350 ms lease).
# [VI] Các hằng số thời gian giao tiếp (giây và mili-giây):
#      - STATUS_TIMEOUT_S   : Thời gian tối đa chờ gói status (0.8 s; mạch gửi mỗi 0.1 s).
#      - HANDSHAKE_TIMEOUT_S: Thời gian chờ bắt tay lần đầu sau khi mở cổng COM (3.0 s).
#      - ACK_TIMEOUT_S      : Thời gian chờ phản hồi ACK cho mỗi lệnh gửi đi (1.0 s).
#      - KEEP_INTERVAL_S    : Chu kỳ gửi lệnh KEEP duy trì kết nối (80 ms, nhỏ hơn nhiều mức 350 ms của mạch).
STATUS_TIMEOUT_S = 0.8
HANDSHAKE_TIMEOUT_S = 3.0
ACK_TIMEOUT_S = 1.0
KEEP_INTERVAL_S = 0.08
MAX_EVENTS_PER_POLL = 128
GUI_POLL_MS = 25
PATH_POLL_MS = 10


def status_fresh(link, state, received, now):
    """
    [EN] Return True if `link` is connected, `state` belongs to the current `link.session`,
         and the last status packet arrived less than `STATUS_TIMEOUT_S` seconds ago.
    [VI] Trả về True nếu đang kết nối, gói `state` khớp với mã phiên `link.session`,
         và dữ liệu mới nhận được cách đây chưa quá `STATUS_TIMEOUT_S` (0.8 giây).
    """
    if link is None or state is None:
        return False
    if state.session != link.session:
        return False
    age_s = now - received
    return age_s < STATUS_TIMEOUT_S


def status_expired(state, opened, received, now):
    """
    [EN] Return True if the serial connection has timed out waiting for the initial
         handshake (`state is None`) or waiting for periodic telemetry updates.
    [VI] Trả về True nếu quá thời gian chờ bắt tay ban đầu (`HANDSHAKE_TIMEOUT_S`)
         hoặc mất tín hiệu cập nhật trạng thái định kỳ quá `STATUS_TIMEOUT_S`.
    """
    if state is None:
        time_since_open_s = now - opened
        return time_since_open_s > HANDSHAKE_TIMEOUT_S

    time_since_last_rx_s = now - received
    return time_since_last_rx_s > STATUS_TIMEOUT_S


def hardware_ready(state, config_error=None):
    """
    [EN] Return True if the MCU reports all 3 axes' switch pairs debounced (`ready == 7`,
         binary `0b111`), no driver fault, no switch conflicts, and no GPIO read errors.
    [VI] Trả về True khi cả 3 trục đều đã đọc công tắc ổn định (`ready == 7`, tức `0b111`),
         không có lỗi cấu hình, không lỗi driver (`fault`), không chập/hở công tắc (`conflicts`).
    """
    if state is None or config_error:
        return False

    # Bitmask 7 = 0b111 means axis 0 (Z), axis 1 (J1), and axis 2 (J2) are all debounced & ready
    # Mặt nạ bit 7 = 0b111 nghĩa là cả 3 trục Z (bit 0), J1 (bit 1) và J2 (bit 2) đều đã sẵn sàng
    all_axes_debounced = state.ready == 7
    no_hardware_faults = (
        not state.fault and not state.conflicts and not state.errors
    )
    return all_axes_debounced and no_hardware_faults


def background_allowed(task_kind, config):
    """
    [EN] Check whether the active motion (`INIT` vs normal `MOVE`/`PATH`) is allowed
         to continue running when the user switches focus to another window.
    [VI] Kiểm tra xem tác vụ đang chạy (`INIT` hay di chuyển `MOVE`/`PATH`) có được phép
         tiếp tục chạy nền khi người dùng bấm sang cửa sổ phần mềm khác hay không.
    """
    if task_kind == "INIT":
        return config.home_in_background
    return config.motion_in_background
