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
  Dùng `-BuildDirectory build_refactor -SkipExport` để kiểm tra bản sửa trong
  thư mục riêng; bỏ `-SkipExport` khi cần xuất lại bộ HEX/BIN trong `Hex/`.
- `tools/verify.ps1`: build harness C vào `build_host/`, chạy full home,
  GPIO/timer, DDA, giới hạn, USB/watchdog, biên parser/JSON và kiểm thử
  mô hình/giao tiếp. `-CoreOnly` chỉ chạy phần firmware.
- `tools/verify_artifacts.py`: kiểm tra HEX/BIN, vector và RAM. Mặc định chỉ đọc
  bộ phát hành và báo cáo khớp checksum trong `Hex/`. Truyền `--elf` để kiểm tra
  ELF của bản mới; ảnh flash trong ELF phải khớp BIN trước khi dùng số RAM.
  Chỉ ghi báo cáo khi có `--output`, không gắn mã nguồn mới vào báo cáo cũ.
- `build_mcu/`, `build_refactor/`, `build_host/`: sản phẩm sinh lại.

Từ thư mục gốc, chạy `./Verify_SCARA.ps1 -Gui` để kiểm tra firmware trước rồi
toàn bộ software, gồm năm GUI smoke test bằng cổng giả lập. Có thể truyền
`-Python` và `-Gcc`; mặc định tìm công cụ trên PATH rồi thử đường dẫn cài cũ.

Kiểm tra ảnh vừa build mà không thay bộ phát hành:

```powershell
./Firmware/ScaraCartesian/tools/build.ps1 -BuildDirectory build_refactor -SkipExport
D:/zephyrproject/.venv/Scripts/python.exe Firmware/ScaraCartesian/tools/verify_artifacts.py --hex Firmware/ScaraCartesian/build_refactor/zephyr/zephyr.hex --bin Firmware/ScaraCartesian/build_refactor/zephyr/zephyr.bin --elf Firmware/ScaraCartesian/build_refactor/zephyr/zephyr.elf --output Firmware/ScaraCartesian/build_refactor/verification.json
```

Kiểm tra ELF cần `pyelftools`, đã có trong môi trường Python của Zephyr.

`../Ref/` giữ mã CubeMX gốc từng chạy phần cứng, chỉ để tham khảo.
Không còn các dự án jog, HEX hay archive nguồn của phiên bản cũ.

## Cấu trúc mã nguồn

- `src/main.c`: nối phần cứng Zephyr với worker, GPIO/readback, timer ISR,
  callback USB và vòng lặp chính. Mọi thay đổi trạng thái dùng cùng khóa IRQ;
  HIGH được xả hết trước khi đổi DIR hoặc bắt đầu lượt chạy tiếp theo.
- `src/cart_core.c`: bảo vệ công tắc/heartbeat, DDA, FIFO đường vẽ, profile
  tốc độ và các bước home/coupling. `home_worker.c` thực hiện từng lượt đo;
  `jog_core.c` xử lý trạng thái công tắc và bộ phát xung một trục.
- `src/cart_commands.c`: xác nhận session/job, cấu hình và lệnh chuyển trạng
  thái; `cart_core_internal.h` chỉ chia sẻ những chuyển trạng thái cần thiết
  với worker. Giao diện dùng bởi lớp phần cứng vẫn ở `cart_core.h`.
- `src/command_text.c`: token và số nguyên dùng bộ nhớ của caller, không dùng
  con trỏ toàn cục của `strtok`. Grammar riêng của Cartesian và jog được giữ.
- `src/command_stream.c`: nhận lệnh dạng dòng ASCII, giới hạn 191
  byte nội dung, bỏ CR, loại cả dòng lỗi và xóa phần dòng cũ khi USB đổi epoch.
- `src/cart_telemetry.c`: chụp các trường trạng thái trong khóa, định dạng
  status ngoài khóa và tạo JSON ACK. Schema và thứ tự trường protocol5 không
  thay đổi.

`tests/test_protocol.c` kiểm tra chiều dài dòng, byte lỗi, reset session,
giới hạn số nguyên, parser độc lập, ACK chính xác và snapshot không bị thay đổi
sau khi state tiếp tục chạy. `tests/host_firmware.c` vẫn chạy trực tiếp mã
GPIO/timer thực với ngoại vi giả lập, gồm full home và chuyển đoạn đường vẽ.
