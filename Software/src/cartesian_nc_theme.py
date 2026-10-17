"""
================================================================================
FILE: cartesian_nc_theme.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Dark-Mode Color Palette Constants for the SCARA Tkinter Studio GUI.
     Centralizes all hexadecimal color strings (`#RRGGBB`) used across control
     cards, canvas coordinate grids, interactive buttons, and LED status badges.

     WHY CENTRALIZE COLORS? (Software Engineering Tip for Students):
     Instead of hardcoding hex strings like `"#0b1220"` in dozens of files,
     storing them as named constants in one theme file makes the code easier
     to read and allows changing the entire GUI theme in one place.

[VI] MỤC ĐÍCH:
     Bảng Mã Màu Giao diện Tối (Dark Theme) cho Phần mềm Điều khiển Robot SCARA.
     Tập trung toàn bộ các chuỗi mã màu thập lục phân (`#RRGGBB`) dùng chung cho
     các khung thẻ điều khiển, lưới tọa độ Canvas, nút bấm và đèn LED trạng thái.

     TẠI SAO NÊN TẬP TRUNG MÃ MÀU? (Mẹo Lập trình cho Sinh viên):
     Thay vì viết trực tiếp các mã màu khó nhớ như `"#0b1220"` rải rác ở nhiều
     file, việc đặt tên hằng số tại một file duy nhất giúp code dễ đọc hơn và
     cho phép đổi màu toàn bộ phần mềm chỉ tại một nơi.
================================================================================
"""

# ------------------------------------------------------------------------------
# 1. BACKGROUND & CONTAINER COLORS / MÀU NỀN CỬA SỔ & KHUNG THẺ
# ------------------------------------------------------------------------------
# [EN] Deep navy background for the main application window.
# [VI] Màu xanh đen đậm làm nền cho cửa sổ chính của ứng dụng.
BG = "#0b1220"

# [EN] Slightly lighter navy background for grouped control panel cards.
# [VI] Màu xanh đậm vừa phải làm nền cho các khung thẻ bảng điều khiển.
CARD = "#142136"

# [EN] Subtle border and coordinate grid line color.
# [VI] Màu đường viền mảnh quanh khung thẻ và đường lưới tọa độ.
LINE = "#263952"

# ------------------------------------------------------------------------------
# 2. TYPOGRAPHY & TEXT COLORS / MÀU CHỮ HIỂN THỊ
# ------------------------------------------------------------------------------
# [EN] High-contrast off-white color for primary text and numerical readouts.
# [VI] Màu trắng sáng độ tương phản cao cho chữ chính và số đo tọa độ.
TEXT = "#ecf3fc"

# [EN] Muted slate-gray color for secondary labels, units, and hints.
# [VI] Màu xám xanh dịu mắt cho các nhãn phụ, đơn vị đo và dòng gợi ý.
MUTED = "#9fb1ca"

# ------------------------------------------------------------------------------
# 3. SEMANTIC STATUS & ACCENT COLORS / MÀU NHẤN & TRẠNG THÁI HỆ THỐNG
# ------------------------------------------------------------------------------
# [EN] Primary blue accent for active buttons, X-axis readouts, and Link 1 (L1).
# [VI] Màu xanh dương chủ đạo cho nút bấm chính, số đo trục X và Khâu 1 (L1).
BLUE = "#5aaeff"

# [EN] Emerald green for "Connected / Ready" badges, OK switches, and Link 2 (L2).
# [VI] Màu xanh lục bảo cho trạng thái "Đã kết nối / Sẵn sàng", công tắc OK và Khâu 2.
GREEN = "#60d5ae"

# [EN] Coral red for emergency STOP buttons, fault alerts, and triggered switches.
# [VI] Màu đỏ san hô cho nút DỪNG khẩn cấp, cảnh báo lỗi và công tắc chạm biên.
RED = "#ff8795"

# [EN] Cyan-mint highlight accent for Y-axis readouts, thumbnails, and progress text.
# [VI] Màu xanh ngọc sáng làm điểm nhấn cho trục Y, hình thu nhỏ và dòng tiến độ.
ACCENT = "#61d9c4"

# [EN] Bright dashboard blue used for section headers and metric highlights.
# [VI] Màu xanh dương sáng dùng cho tiêu đề phân mục trên bảng điều khiển.
DASHBOARD_BLUE = "#69adff"
