"""
================================================================================
FILE: cartesian_nc_messages.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Human-Readable Vietnamese Operator Messages & Telemetry Translation Tables.
     Maps numeric firmware enums (`stage`, `phase`, `cal_error`) and status/ACK
     `reason` tokens into clear Vietnamese explanations for the GUI dashboard.

     HOW THIS WORKS (For 1st/2nd-Year Students):
     The STM32 microcontroller has limited RAM and USB bandwidth, so it sends
     compact integer codes (like `stage=8`, `phase=[10,10,10]`) and short ASCII
     tokens (like `reason=soft_limit`). This module acts as a dictionary that
     translates those compact machine codes into helpful Vietnamese sentences.

[VI] MỤC ĐÍCH:
     Bảng Tra cứu Thông báo Tiếng Việt cho Người Vận hành & Gỡ lỗi.
     Ánh xạ các mã số trạng thái từ Firmware (`stage`, `phase`, `cal_error`) và
     chuỗi nguyên nhân `reason` sang thông báo Tiếng Việt dễ hiểu trên giao diện.

     NGUYÊN LÝ HOẠT ĐỘNG (Dành cho Sinh viên năm 1–2):
     Vi điều khiển STM32 có bộ nhớ RAM và băng thông USB hạn chế, nên chỉ gửi
     các mã số nguyên ngắn gọn (như `stage=8`) hoặc từ khóa tiếng Anh ngắn (như
     `reason=soft_limit`). File này đóng vai trò như một cuốn "từ điển" dịch
     các mã máy đó sang câu thông báo Tiếng Việt rõ ràng cho người vận hành.
================================================================================
"""

# ==============================================================================
# 1. MULTI-STAGE HOMING & CALIBRATION STAGES / CÁC GIAI ĐOẠN DÒ GỐC 3 TRỤC
# ==============================================================================
# [EN] Matches `enum ct_stage` in firmware `cart_core.h` (indexed 0 to 12).
#      Usage: `STAGES[status.stage]` returns the Vietnamese stage name.
# [VI] Khớp với `enum ct_stage` trong file firmware `cart_core.h` (chỉ số 0..12).
#      Cách dùng: `STAGES[status.stage]` trả về tên giai đoạn bằng Tiếng Việt.
STAGES = (
    "Chưa lấy mốc",                     # 0: Unreferenced (initial power-on state)
    "Z: đo hai biên",                   # 1: Z-axis full-span limit switch scan
    "J2: đo sơ bộ",                     # 2: J2 elbow preliminary scan
    "J2: về giữa",                      # 3: Move J2 to safe center before moving J1
    "J1: đo và bù J2",                  # 4: J1 shoulder scan with belt-coupling compensation
    "J1: về mốc góc",                   # 5: Move J1 to reference angle
    "J2: lấy lại mốc",                  # 6: Re-home J2 accurately after J1 is referenced
    "Về tư thế đỗ",                     # 7: Move all axes to safe Park Pose
    "Sẵn sàng XYZ",                     # 8: Calibrated & ready for Cartesian XYZ commands
    "Khởi tạo thất bại",                # 9: Homing / calibration faulted
    "Đo bù: dịch J1 một đoạn nhỏ",      # 10: Auto-coupling probe: step J1 by small delta
    "Đo bù: đo lại biên J2",            # 11: Auto-coupling probe: re-measure J2 limit shift
    "Đo bù: trở về tư thế ban đầu",     # 12: Auto-coupling probe: return to starting pose
)

# ==============================================================================
# 2. PER-AXIS LIMIT SWITCH SCAN PHASES / CÁC PHA DÒ CÔNG TẮC HÀNH TRÌNH
# ==============================================================================
# [EN] Per-axis calibration phases for rotary joints J1/J2 (`enum hc_phase` in `home_worker.h`).
#      Each axis seeks the negative limit (−), backs off, latches slowly, scans to
#      the positive limit (+), and returns to verify repeatability.
# [VI] Các pha dò công tắc hành trình cho khớp xoay J1/J2 (`enum hc_phase` trong `home_worker.h`).
#      Mỗi trục sẽ tìm biên âm (−), lùi ra, tiến vào chậm để chốt mốc chính xác,
#      quét sang biên dương (+), rồi quay lại biên âm để kiểm tra độ lặp lại.
PHASES = (
    "Chờ",                  # 0: Idle / waiting
    "Tìm biên −",           # 1: Fast seek toward negative limit switch
    "Lùi biên −",           # 2: Back off until negative switch closes again
    "Tiếp cận chậm −",      # 3: Slow latch onto negative limit switch
    "Quét đến biên +",      # 4: Full travel scan toward positive limit switch
    "Lùi biên +",           # 5: Back off from positive switch
    "Tiếp cận chậm +",      # 6: Slow latch onto positive limit switch (records n1)
    "Quét lại biên −",      # 7: Return scan back toward negative limit switch
    "Lùi biên −",           # 8: Back off from negative switch
    "Tiếp cận chậm −",      # 9: Second slow latch on negative switch (records n2)
    "Hoàn tất",             # 10: Axis calibration complete
    "Lỗi",                  # 11: Axis calibration failed
)

# [EN] Per-axis calibration phases for vertical axis Z (`enum hc_phase`).
#      NOTE FOR STUDENTS: On this SCARA robot, the negative step direction of Z
#      is at the TOP switch ("biên trên"), and the positive limit is at the BOTTOM ("biên dưới").
# [VI] Các pha dò công tắc hành trình cho trục đứng Z (`enum hc_phase`).
#      LƯU Ý CHO SINH VIÊN: Trên cơ cấu trục Z này, chiều bước âm hướng LÊN TRÊN
#      ("biên trên") và chiều bước dương hướng XUỐNG DƯỚI ("biên dưới").
Z_PHASES = (
    "Chờ",                      # 0: Idle / waiting
    "Tìm biên trên",            # 1: Fast seek upward to top switch
    "Lùi khỏi biên trên",       # 2: Back off downward from top switch
    "Tiếp cận chậm biên trên",  # 3: Slow latch onto top switch
    "Quét xuống biên dưới",     # 4: Measure downward travel to bottom switch
    "Lùi khỏi biên dưới",       # 5: Back off upward from bottom switch
    "Tiếp cận chậm biên dưới",  # 6: Slow latch onto bottom switch (records n1)
    "Quét lên biên trên",       # 7: Return scan upward to top switch
    "Lùi khỏi biên trên",       # 8: Back off downward from top switch
    "Tiếp cận chậm biên trên",  # 9: Second slow latch onto top switch (records n2)
    "Hoàn tất",                 # 10: Z calibration complete
    "Lỗi",                      # 11: Z calibration failed
)

# ==============================================================================
# 3. CALIBRATION ERROR CODES / MÃ LỖI HIỆU CHUẨN TRỤC
# ==============================================================================
# [EN] Calibration error descriptions (`enum hc_error` in `home_worker.h`, indexed 0 to 8).
# [VI] Mô tả chi tiết mã lỗi hiệu chuẩn từng trục (`enum hc_error` trong `home_worker.h`).
ERRORS = (
    "",                                 # 0: No error (OK)
    "Đã hủy",                           # 1: Aborted by user or stop command
    "Mất heartbeat",                    # 2: USB heartbeat timeout during homing
    "GPIO/timer lỗi",                   # 3: Hardware timer or GPIO peripheral fault
    "Đầu vào công tắc lỗi",             # 4: Limit switch input conflict / wiring fault
    "Vượt ngân sách tìm biên",          # 5: Switch not found within max step budget
    "Chạm chưa được xác nhận ổn định",  # 6: Switch debounce failed to settle
    "Hai lần đo không lặp lại",         # 7: Forward vs backward step counts (n1 vs n2) differ too much
    "Lệnh calib không hợp lệ",          # 8: Invalid calibration command parameters
)

# ==============================================================================
# 4. FIRMWARE STATUS & ACK REASON DICTIONARY / TỪ ĐIỂN NGUYÊN NHÂN TRẠNG THÁI
# ==============================================================================
# [EN] Mapping from firmware `reason` token to operator-friendly Vietnamese guidance.
# [VI] Bảng tra từ mã `reason` của Firmware sang hướng dẫn xử lý bằng Tiếng Việt.
REASONS = {
    # --- Normal operational states / Trạng thái vận hành bình thường ---
    "unreferenced": "Cần HOME + CALIB",
    "initialized": "Đã lấy mốc, sẵn sàng XYZ",
    "initializing": "Đang khởi tạo",
    "moving": "Đang di chuyển",
    "complete": "Đã đến đích",
    "stopped": "Đã dừng; home lại nếu đang chạy",
    "released": "Arm đang nhả lực",

    # --- Safety, Limit Switch & Hardware Faults / Lỗi an toàn, công tắc & phần cứng ---
    "heartbeat_lost": "Mất heartbeat; đã dừng",
    "both_limits": "Hai biên cùng mở: kiểm tra NC/GND rồi reset nếu lỗi bị giữ",
    "both_open": "Đã đọc HIGH/HIGH; chuyển động đã bị hủy",
    "both_latched": "HIGH/HIGH đủ 20 ms: kiểm tra NC/GND, rồi reset STM32",
    "checking_inputs": "Đã dừng xung, đang xác nhận tín hiệu công tắc",
    "switch_unstable": "Tín hiệu không ổn định trở lại trong 200 ms; đã hủy",
    "calibration_failed": "Calib thất bại",
    "j2_guard": "J2 chạm biên khi J1 calib; kiểm tra hệ số bù/chiều",
    "j2_limit": "Đã chạm biên J2",
    "j1_limit": "Đã chạm biên J1",
    "z_limit": "Đã chạm biên Z",
    "switch_stale": "Dữ liệu công tắc quá hạn trong firmware",
    "switch_io": "Lỗi đọc GPIO công tắc",
    "switch_not_ready": "Đợi công tắc ổn định",
    "motor_gpio_error": "Đầu ra motor lỗi; cần reset",
    "waiting_switch": "Đợi tín hiệu ổn định trước khi chạy trục tiếp theo",
    "switch_transition_unstable": "Tín hiệu không ổn định khi bắt đầu chạy trục",
    "timer_error": "Timer lỗi; cần reset",

    # --- Belt Coupling Calibration Diagnostics / Chẩn đoán đo bù dây đai chéo ---
    "coupling_sign": "Dấu bù đo được ngược cấu hình: kiểm tra chiều J2; xem log",
    "coupling_measurement": "Phép đo bù quá nhỏ hoặc vượt giới hạn; xem log",
    "coupling_geometry": "Hệ số góc suy ra ngoài phạm vi; kiểm tra góc tại công tắc",
    "coupling_repeat": "Hành trình J2 thay đổi khi đo bù; kiểm tra cơ khí/công tắc",
    "coupling_probe_limit": "Không đủ khoảng cho bước thử J1; xem log",
    "coupling_probe_short": "Bước thử J1 quá ngắn để đo bù",

    # --- Workspace & Communication Limits / Giới hạn vùng làm việc & kết nối ---
    "soft_limit": "Vượt giới hạn góc/Z",
    "usb_lost": "USB mất kết nối",
    "z_clearance": "Khoảng lùi Z không phù hợp hành trình",
}

# [EN] Mapping from GUI-side stop trigger code to firmware reason key.
# [VI] Bảng ánh xạ từ mã sự kiện dừng phía giao diện GUI sang khóa nguyên nhân `reason`.
STOP_REASONS = {
    "USER": "stop_user",          # Operator clicked the red STOP button
    "ESC": "stop_escape",         # Operator pressed the Escape key
    "FOCUS": "stop_focus",        # GUI window lost focus while moving
    "MINIMIZE": "stop_minimize",  # GUI window was minimized
    "DISCONNECT": "stop_disconnect",  # User clicked Disconnect
    "CLOSE": "stop_close",        # User closed the application window
    "STALE": "stop_stale",        # USB status telemetry timed out
    "ACK": "stop_ack_timeout",    # Firmware did not acknowledge a command in time
    "MODEL": "stop_model",        # Kinematic model or reference became invalid
}

# [EN] Add human-readable Vietnamese descriptions for GUI-triggered stops.
# [VI] Bổ sung câu thông báo Tiếng Việt cho các nguyên nhân dừng từ phía giao diện.
REASONS.update(
    {
        "stop_user": "Đã dừng bằng nút DỪNG",
        "stop_escape": "Đã dừng bằng phím Esc",
        "stop_focus": "Đã dừng vì GUI mất focus",
        "stop_minimize": "Đã dừng vì GUI bị thu nhỏ",
        "stop_disconnect": "Đã dừng khi ngắt COM",
        "stop_close": "Đã dừng khi đóng GUI",
        "stop_stale": "Đã dừng vì dữ liệu USB quá hạn",
        "stop_ack_timeout": "Đã dừng vì lệnh không được xác nhận",
        "stop_model": "Đã dừng vì mốc/mô hình không hợp lệ",
    }
)

# [EN] Add human-readable Vietnamese descriptions for continuous path FIFO streaming.
# [VI] Bổ sung câu thông báo Tiếng Việt cho chế độ truyền quỹ đạo vẽ nét liên tục (FIFO).
REASONS.update(
    {
        "path_loading": "Đang nạp bộ đệm nét",
        "path_running": "Đang vẽ nét liên tục",
        "path_starved": "Hết bộ đệm đường vẽ; đã dừng. Gửi log để kiểm tra USB/GUI.",
        "path_sequence": "Sai thứ tự/bộ đệm đường vẽ",
        "path_profile": "Thông số tăng tốc đường vẽ không hợp lệ",
    }
)

# [EN] Protocol & GUI build identifier recorded in `.jsonl` session logs.
# [VI] Mã định danh phiên bản giao diện được ghi vào đầu mỗi file log `.jsonl`.
GUI_VERSION = "SCARA_GUI_V5_R14"
