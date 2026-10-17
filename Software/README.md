# SCARA Motion & Drawing Studio — GUI R15 / Firmware R9

Khởi chạy trực tiếp từ mã nguồn Python qua **[`Run_Cartesian_GUI.bat`](file:///d:/HK261/Robot/HPMR/Scara%20Robot/Software/Run_Cartesian_GUI.bat)** (tự động ưu tiên chạy `python src/cartesian_nc_control.py` để luôn áp dụng bản nâng cấp mới nhất). Firmware tương ứng trên STM32F103C8: `../Firmware/ScaraCartesian/Hex/SCARA_Cartesian_NC_Home_v5_R9.hex` (`SCARA_CART_NC_HOME_V5_R9`).

---

## 1. Các Phân hệ Nâng cấp Toàn diện trong GUI R15

1. **Mô hình 3D X-SCARA Digital Twin (`cartesian_nc_3d.py`)**:
   - Nạp trực tiếp các lưới 3D nhị phân thực tế từ `Hardware/arm/STL/` (`Base.STL`, `Arm1.STL`, `Arm2.STL`, động cơ NEMA17, puly GT2, cụm nâng hạ Z `Tool.STL`) theo ma trận lắp ráp chuẩn của dự án `madl3x/x-scara`.
   - Áp dụng thuật toán rút gọn lưới theo diện tích tam giác (~1.000 đa giác có tô bóng Phong/Lambertian + sắp xếp độ sâu Painter's Algorithm), đảm bảo tốc độ hiển thị **>60 FPS** ngay trên Tkinter Canvas chuẩn mà không cần thư viện OpenGL ngoài.
   - Cho phép chuyển đổi tức thời giữa **`Mặt phẳng 2D (CAD)`** và **`Mô hình 3D (X-SCARA STL)`** ngay trên khung làm việc chính, hoặc mở **Cửa sổ 3D Độc lập (`🧊 Mở Cửa sổ 3D`)** để quan sát song song 2D + 3D.
   - Tương tác chuột trong 3D: **Kéo chuột trái** để xoay quỹ đạo camera (Orbit), **Kéo chuột phải/giữa** để dịch chuyển tâm nhìn (Pan), **Lăn con lăn** để phóng to/thu nhỏ (Zoom), kèm các nút góc nhìn nhanh (`Isometric`, `Mặt đứng`, `Mặt cạnh`, `Mặt bằng`) và nút **Xuất Vector `.svg`** độ phân giải cao để chèn vào báo cáo/luận văn.

2. **Bảng Điều khiển Trực quan 6 Thẻ Thông số & 6 Đèn LED Công tắc NC (`cartesian_nc_layout.py`, `cartesian_nc_view.py`)**:
   - **6 Thẻ Số liệu Thời gian thực**: Tọa độ `X`, `Y`, `Z` (hiện đồng thời xung bước thực đo), `Góc khớp J1 · J2` (kèm góc động cơ B đã bù đai $\theta_B = \theta_2 + k\theta_1$ và chỉ số khả thao tác $|\sin\theta_2|$), `Tốc độ & Bộ đệm` (vận tốc đầu bút $\text{mm/s}$, tần số xung $\text{pps}$, mức chiếm dụng bộ đệm `FIFO x/32` trên STM32), và `Mốc & Chuyển động`.
   - **6 Đèn LED Công tắc Hành trình NC (`Z+`, `Z-`, `J1+`, `J1-`, `J2+`, `J2-`)**: Hiển thị trực tiếp trạng thái đóng/mở từng kênh công tắc từ thanh ghi `switches` của vi điều khiển (xanh lá `OK` khi kín mạch, đỏ cảnh báo `CHẠM` khi hở mạch/chạm cữ).

3. **Tab `Jog & Tùy biến` — Điều khiển Bước Tinh chỉnh & Tư thế Nhanh (`cartesian_nc_layout.py`, `cartesian_nc_editor.py`)**:
   - **Bàn phím Step-Jog**: Chọn bước nhảy chính xác (`0.5`, `1`, `2`, `5`, `10`, `20 mm`) và dịch chuyển từng bước theo `±X`, `±Y`, `±Z` để rà mặt giấy hoặc căn chỉnh gá kẹp.
   - **Tư thế Nhanh (Quick Poses)**: Đi tới nhanh **Tâm vùng vẽ** `(0, 165 mm, Z an toàn)`, **Tư thế nghỉ chuẩn (Park Pose)** sau HOME, hoặc **Hạ xuống cao độ mặt giấy (`Z giấy`)**.
   - **Thanh trượt tốc độ toàn cục**: Điều chỉnh hệ số tốc độ từ `25%` đến `150%` ngay trên giao diện.

4. **Xưởng Vẽ & Khắc Mở rộng — Chữ Tùy ý (`A–Z`, `0–9`) & Bộ Sinh Mẫu Tham số (`cartesian_nc_font.py`, `cartesian_nc_drawing_gui.py`)**:
   - Mở rộng bộ font nét đơn (single-stroke vector font) đầy đủ bảng chữ cái in hoa **`A–Z`**, chữ số **`0–9`** và ký hiệu (`-`, `+`, `.`, `/`), đồng thời giữ nguyên tuyệt đối tọa độ mẫu chuẩn `BK`, `LINH`, `THOA`.
   - Tab **`Chữ & Mẫu tùy ý`** trong cửa sổ *Thư viện hình & chữ* cho phép:
     - Nhập bất kỳ chuỗi văn bản nào (ví dụ: `SCARA 2026`, `BACH KHOA`, mã số sinh viên) kèm góc xoay tùy ý.
     - Tạo tự động 4 họ đường cong tham số kỹ thuật: **Xoắn ốc Archimedes (`SPIRAL`)**, **Lưới hiệu chuẩn độ vuông góc (`GRID`)**, **Hoa Rhodonea toán học (`ROSE`)**, và **Đa giác đồng tâm (`POLYGON`)**.
     - Nạp trực tiếp file vector ngoại vi (`.svg`, `.dxf`, `.gcode`, `.nc`, `.ngc`).

5. **Trung tâm Dữ liệu, Đồ thị Động học & Xuất Báo cáo Học thuật (`cartesian_nc_analytics.py`)**:
   - Tự động thu thập chuỗi thời gian `TelemetrySample` (vị trí $X, Y, Z$, góc khớp $J_1, J_2, \theta_B$, vận tốc đầu bút, tần số xung $\text{pps}$, mức chiếm dụng bộ đệm `FIFO`, chỉ số $|\sin J_2|$).
   - **4 Kênh Đồ thị Khoa học (Scientific Oscilloscope)**:
     1. *Đồ thị Tọa độ $X(t), Y(t), Z(t)$*
     2. *Đồ thị Góc khớp $J_1(t), J_2(t)$ & Góc động cơ $\theta_B(t)$*
     3. *Đồ thị Vận tốc đầu bút ($\text{mm/s}$), Xung/s ($\text{pps}$) & Bộ đệm `FIFO`*
     4. *Chỉ số Khả thao tác $|\sin J_2|$ & Bản đồ Quỹ đạo XY thực tế*
   - **Nạp lại Log `.jsonl` ngoại tuyến**: Đọc trực tiếp các file log phần cứng trong `Doc/logs/` để vẽ lại đồ thị và trích xuất số liệu mà không cần cắm mạch.
   - **3 Công cụ Xuất Báo cáo 1-Click**:
     - **Xuất CSV (`export_telemetry_csv`)**: File bảng tính chuẩn UTF-8 mở trực tiếp bằng Excel / MATLAB / Python Pandas.
     - **Xuất Đồ thị Vector SVG (`export_analytics_svg`)**: Ảnh vector sắc nét để chèn vào Word / LaTeX.
     - **Xuất Báo cáo Kỹ thuật Markdown/LaTeX (`export_markdown_report`)**: Tự động tổng hợp bảng thông số hình học, phương trình động học thuận/bù đai LaTeX, và bảng thống kê thực nghiệm đầy đủ.

---

## 2. Quy trình Vận hành Chuẩn

1. Kết nối cổng COM (hoặc chạy với `--demo`), kiểm tra thông số trong **Cài đặt & Cơ khí**, sau đó bấm **`HOME / CALIB`**.
2. Điều khiển điểm đích qua **Điểm đích XYZ**, kéo thả trực tiếp trong tab **Chỉnh mô hình**, hoặc dùng bàn phím bước nhỏ trong tab **Jog & Tùy biến**.
3. Mở **Thư viện hình & chữ** để chọn mẫu chuẩn (`BK`, `LINH`, `THOA`), hình học cơ bản, nhập chữ/mẫu tham số tùy ý hoặc nạp file `.svg/.dxf/.gcode`.
4. Dùng tab **Jog & Tùy biến** để hạ bút chạm nhẹ mặt giấy, bấm **Lấy Z hiện tại** và **Xác nhận Z giấy**, chạy **Chạy thử trên không (Dry Run)** trước khi bấm **Bắt đầu vẽ thật**.
5. Mở **Đồ thị & Báo cáo (`📈 Đồ thị & Báo cáo`)** để theo dõi đáp ứng động học thời gian thực hoặc xuất bảng số liệu `.csv`, hình `.svg` và báo cáo `.md`.

---

## 3. Phát triển & Kiểm thử Tự động

- Chạy GUI trực tiếp từ mã nguồn:
  ```powershell
  python src/cartesian_nc_control.py
  # Hoặc chạy chế độ mô phỏng không cần mạch thật:
  python src/cartesian_nc_control.py --demo
  ```
- Chạy toàn bộ bộ kiểm thử tự động (Firmware C host tests + 74 Python unit tests + 6 kịch bản GUI smoke tests + Webots tests):
  ```powershell
  powershell -ExecutionPolicy Bypass -File ../Verify_SCARA.ps1 -Gui
  ```
