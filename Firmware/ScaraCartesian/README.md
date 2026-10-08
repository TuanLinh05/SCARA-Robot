# SCARA Cartesian — firmware R9

Nguồn Zephyr3.7 cho Blue Pill STM32F103C8 (64KiB flash,20KiB RAM).
Bộ hiện tại: `Hex/SCARA_Cartesian_NC_Home_v5_R9.hex` và `.bin`.
Nạp tại `0x08000000`, reset. Build tag `SCARA_CART_NC_HOME_V5_R9`, protocol5.
GUI hiện tại ở `../../Software/Run_Cartesian_GUI.bat`.

## Đấu dây

TB6600 kiểu chung âm: PUL−, DIR−, ENA− nối GND chung.

| Trục | PUL+ | DIR+ | ENA+ | Biên DIR HIGH | Biên DIR LOW |
|---|---|---|---|---|---|
| Z | PA0 | PA1 | PA2 | PA3 trên | PA4 dưới |
| J1 | PA8 | PB15 | PB14 | PB0 | PB1 |
| J2 | PA6 | PA7 | PB13 | PB10 | PB11 |

Công tắc **COM–NC**: COM→GND, NC→GPIO kéo lên3,3V; NO bỏ trống.
Bình thường LOW, chạm/hở dây HIGH. J2 DIR HIGH quay ngược dấu hình học J1;
GUI cấu hình `positive_high=[true,true,false]`. Không tự đảo dấu khi chưa đo.
Z giữ cấp dòng; arm chỉ nhả/giữ bằng lệnh thủ công lúc dừng.

## Home, di chuyển và bảo vệ

Không chạy khi boot. Home quét hai biên Z/J2, thử J1 một đoạn ngắn để đo
beta=dB/dA, dùng beta cho lượt quét dài J1 rồi suy ra hệ số góc theo xung/độ
đã đo. Không dùng tỷ số truyền danh định để ép bù khi J1 chưa được đo.
Các góc tại công tắc và chiều dài L1/L2 do người dùng nhập; chỉ đo xung
không chứng minh góc180° hoặc kích thước98mm là đúng.

Z home3mm/s quy đổi ra4800pps với lead2mm/vòng và vi bước16, trần6400pps
riêng calib Z; chốt/lùi400pps và có ramp. Arm/MOVE/đường vẽ trần1600pps.
PATH/SEG/GO dùng FIFO32 đoạn và profile v² bậc5 fixed-point, đổi DIR sau
khi hạ HIGH, HIGH/DIR setup≥50µs. GUI preflight toàn bộ nét trước khi phát.

Kiểm tra sáu biên ở từng cạnh xung, giới hạn mềm, heartbeat350ms, USB,
GPIO/readback và cạn bộ đệm. STOP/lỗi hủy mốc, không tự tiếp tục sau lỗi.
Số xung là số cạnh đã phát/ghi nhận trong firmware, không phải encoder.

## Build và xác minh

- `tools/build.ps1`: Zephyr SDK0.16.8, Ninja, Python ở `D:/zephyrproject/.venv`.
  Dùng junction đã kiểm tra trỏ về workspace để tránh đường dẫn có dấu cách.
- `tools/verify.ps1`: build harness C vào `build_host/`, chạy full home,
  GPIO/timer, DDA, giới hạn, USB/watchdog và kiểm thử mô hình/giao tiếp.
- `tools/verify_artifacts.py`: kiểm tra HEX/BIN, vector và RAM của ELF sau build;
  khi không còn ELF, dùng báo cáo build khớp checksum của bộ hiện tại.
- `build_mcu/`, `build_host/`: sản phẩm sinh lại; có thể xóa sau khi kiểm tra.

`../Ref/` giữ mã CubeMX gốc từng chạy phần cứng, chỉ để tham khảo.
Không còn các dự án jog, HEX hay archive nguồn của phiên bản cũ.
