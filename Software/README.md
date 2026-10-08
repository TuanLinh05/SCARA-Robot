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

Kiểm thử: `./Verify_Cartesian_NC_GUI.ps1 -Gui`. Nếu chưa có harness firmware,
script tự build/kiểm thử nó trước. Có thể bỏ `-Gui` để chạy các unit.
Build/checks tạo `build/`, `__pycache__/`, `Firmware/ScaraCartesian/build_host/`;
đó là sản phẩm sinh lại, có thể xóa sau khi dùng. Các kiểm thử không chạy robot.

Phân tích log chỉ đọc, không mở COM:
`python src/cartesian_nc_audit.py path/to/session.jsonl --out reports/drawing_audit`.
Log `stroke_start`, `stroke_complete`, `drawing_complete` lưu profile, xung,
thời gian và lỗi bộ đệm. Báo cáo không đo được trượt bước, độ rơ hoặc bút võng.
