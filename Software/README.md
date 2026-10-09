# SCARA — GUI R14 / firmware R9

Chỉ giữ bộ hiện tại. Mở **Run_Cartesian_GUI.bat**; EXE nằm trong
`dist/SCARA_Cartesian_NC_v5_R14/`. Firmware tương ứng:
`../Firmware/ScaraCartesian/Hex/SCARA_Cartesian_NC_Home_v5_R9.hex`, nạp tại
`0x08000000`. Dòng firmware trong GUI phải là `SCARA_CART_NC_HOME_V5_R9`.

## Dùng robot

1. Kết nối COM, kiểm tra **Cơ khí**, rồi **HOME / CALIB**.
2. Điều khiển bằng XYZ hoặc tab **Chỉnh mô hình**; chuỗi điểm có trong
   **Điểm thử / Chuỗi setpoint**. Tọa độ là ước tính từ xung, chưa có encoder.
3. **Thư viện hình & chữ** có BK/LINH/THOA, hoa6 cánh, sao, trái tim,
   tròn, elip, vuông, tam giác, hình thoi. Chọn cỡ/tâm hoặc phóng vừa vùng.
4. Chỉnh bút vừa chạm giấy, lấy/xác nhận Z giấy; chạy khô trước khi vẽ.
   Các nét dùng chuyển động liên tục, nâng bút khi đổi nét và khi hoàn thành.
5. Dừng/Esc, mất USB/heartbeat/status, lỗi biên/GPIO, reset hoặc nhả arm
   đang dùng làm mất mốc. Khi đó HOME lại. Chuyển app giữ chạy nền theo
   cấu hình; chế độ bám chuột tắt bám và hoàn thành đoạn ngắn đã gửi.

## Cơ khí và độ chính xác

Mặc định L1/L2=98mm, hai khớp có góc công tắc±90°, Zlead2mm/vòng, vi bước
Z16/J1–J2 8. Phải đo kích thước tới đầu bút và góc thực tại công tắc.
Calib đo số xung giữa biên và đo bù dB/dA; không tự biết góc thực hoặc vị trí
bút. Z calib3mm/s theo lead cấu hình, chốt400pps; Z thường1mm/s.

`tool_offset_mm=[x,y]`: lệch ngang phải và dọc theo arm2, tính từ điểm
chuẩn cuối L2 đến đầu bút. Khi hai arm thẳng theo+Y, các hướng này là+X/+Y.
Mặc định `[0,0]`; chưa đo thì giữ nguyên. Không cộng cùng độ lệch vào cảL2
và offset. FK/IK/vùng với tới dùng cùng thông số. Lưu thay đổi rồi HOME lại.

Bộ đếm đúng không chứng minh motor/bút đã đến vị trí vật lý. Với nét méo,
so cùng mẫu ở4mm/s và12mm/s, tì bút nhẹ, giữ chắc gá/giấy, kiểm tra độ rơ
và các khoảng cách/góc đã đo. Báo cáo log thật: [báo cáo](reports/drawing_audit_20261008.md).

## Cấu trúc

- `src/`: mã GUI, mô hình, giao tiếp, hình vẽ, log và phân tích.
- `tests/`: kiểm thử hiện tại, chỉ dùng cổng giả lập.
- `packaging/`: một spec PyInstaller cho GUI R14.
- `dist/SCARA_Cartesian_NC_v5_R14/`: bộ chạy; **giữ nguyên `_internal`**.
  Cấu hình và log phiên nằm cạnh EXE; log cũ có ích đã được gom vào `logs/`
  tại đây. Dữ liệu phiên mô phỏng và các bản phát hành cũ đã được bỏ.
- `reports/`: báo cáo kiểm thử hiện tại và kiểm tra nét vẽ.

## Phát triển và kiểm thử

Python3.12, `pip install -r requirements.txt`. Chạy nguồn:
`python src/cartesian_nc_control.py`; `--demo` không mở COM thật.
Build EXE: `./Build_Cartesian_NC_GUI.ps1`. Script cài bộ đóng gói theo
`requirements-build.txt` vào `.packaging/` nếu chưa có. Đây là cache có thể xóa.

Kiểm thử: `./Verify_Cartesian_NC_GUI.ps1 -Gui`. Script luôn build/kiểm thử
harness firmware trước để tránh dùng mẫu C cũ, rồi tự tìm mọi `test_*.py`.
Từ thư mục gốc, có thể chạy `./Verify_SCARA.ps1 -Gui`. Bỏ `-Gui` để chỉ chạy
các unit; `-Python` và `-Gcc` nhận đường dẫn công cụ nếu chưa có trên PATH.
Build/checks tạo `build/`, `__pycache__/`, `Firmware/ScaraCartesian/build_host/`;
đó là sản phẩm sinh lại, có thể xóa sau khi dùng. Các kiểm thử không chạy robot.

GUI được chia theo trách nhiệm: `cartesian_nc_control.py` khởi tạo và điều phối
vòng Tk; `cartesian_nc_connection.py` xử lý USB/session/ACK/status;
`cartesian_nc_motion.py` quản lý lệnh, chuỗi điểm và upload nét;
`cartesian_nc_planning.py` quản lý worker tính đường đi và loại kết quả khi mốc
hoặc vị trí đã đổi; `cartesian_nc_safety.py` quản lý STOP/focus;
`cartesian_nc_view.py` dựng dashboard/cài đặt. Timing và điều kiện cho phép chạy
nằm trong `cartesian_nc_policy.py`, màu/thông báo trong các module riêng.
`NCApp`, CLI, cấu hình JSON và giao thức firmware vẫn giữ giao diện hiện tại.

Định dạng Python bằng Black 26.1.0 (`pip install -r requirements-dev.txt`, rồi
`python -m black src tests packaging/SCARA_Cartesian_NC_v5_R14.spec`).

Phân tích log chỉ đọc, không mở COM:
`python src/cartesian_nc_audit.py path/to/session.jsonl --out reports/drawing_audit`.
Log `stroke_start`, `stroke_complete`, `drawing_complete` lưu profile, xung,
thời gian và lỗi bộ đệm. Báo cáo không đo được trượt bước, độ rơ hoặc bút võng.
