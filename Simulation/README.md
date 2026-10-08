# Mô phỏng X-SCARA vẽ bằng bút lông

Hai môi trường dùng chung hình học, tham số SI và CSV quỹ đạo. MATLAB/Simulink
dùng mô hình giải tích để nghiên cứu thuật toán; Webots dùng vật lý vật rắn,
tiếp xúc đầu bút và giấy, cùng thiết bị Pen để hiển thị nét.

## Tìm file nhanh

```text
Simulation/
├── common/
│   ├── config/scara.json              Thông số robot/bút dùng chung
│   ├── trajectories/                 CSV đường vẽ
│   └── assets/                       Mesh theo khâu, giấy, nguồn/giấy phép
├── matlab/
│   ├── run_scara.m                    Mở giao diện MATLAB
│   ├── startup_scara.m                Thêm đường dẫn + đặt cache
│   ├── config/                       Đọc tham số chung
│   ├── src/                          kinematics, dynamics, control,
│   │                                 trajectory, simulation, io, visualization
│   ├── simulink/models/              scara_brush.slx
│   ├── examples/                     So sánh PID, đọc log Webots
│   ├── tests/                        Kiểm chứng MATLAB/Simulink
│   └── output/                       figures, data, logs, validation, cache
└── webots/
    ├── worlds/scara_draw.wbt          Mở file này trong Webots
    ├── protos/                       Robot SCARA và mặt giấy
    ├── controllers/scara_draw/       Entry point + core/ thuật toán
    ├── config/demo.json              Chọn hình, chế độ điều khiển, màu mực
    ├── tools/                        Tạo lại dự án, kiểm tra offline
    ├── tests/                        Động học, điều khiển, cấu trúc, I/O
    └── output/                       runs/<thời điểm>/, validation/
```

## Chạy Webots

1. Cài Webots và Python 3. Không cần NumPy hay ROS cho controller này.
2. Mở [webots/worlds/scara_draw.wbt](webots/worlds/scara_draw.wbt) bằng **File → Open World**.
3. Nhấn **Run**. Robot di chuyển đến điểm đầu, hạ bút, vẽ hoa rồi nhấc bút.
4. Sửa [webots/config/demo.json](webots/config/demo.json), sau đó **Reset/Reload**
   để thử `circle`, `square`, `csv` hoặc `control_mode: "pid"`.

Xem [hướng dẫn Webots](webots/README.md) để chỉnh thuật toán, đọc log và debug.
World/PROTO dùng header `R2025a`, bản ổn định chính thức được kiểm tra ở thời điểm
tạo dự án. Không cần đổi header chỉ vì cài một bản mới hơn. Máy hiện chưa có
Webots nên chưa xác nhận khởi chạy hoặc vật lý trên bản 2026 bạn định cài.

## Chạy MATLAB/Simulink

Trong MATLAB, chuyển Current Folder đến `Simulation/matlab` rồi chạy:

```matlab
run_scara
```

Hoặc `startup_scara; out=scara_simulink('flower');`. File SLX nằm trong
`matlab/simulink/models/scara_brush.slx`, tự thêm đường dẫn khi mở/chạy trực tiếp.
Xem [hướng dẫn MATLAB](matlab/README.md).

## Các điểm cần biết trước khi dùng kết quả trong bài tập

- Hình dáng tay lấy từ 11 instance STL trong `Hardware/arm/STL`, gom thành
  5 mesh có đơn vị mét. Chiều dài hai khâu mặc định 98 mm. Tầng lắp ráp Z,
  thành bên, khung, motor, bút và giá kẹp còn là xấp xỉ.
- Mô hình có hai khớp quay và trục Z nâng cả cụm tay. Webots có thêm khớp
  đàn hồi thụ động ở đầu bút, GPS đầu bút, cảm biến lực và encoder.
- Khối lượng, quán tính, ma sát, giới hạn khớp/lực và độ đàn hồi cần hiệu chỉnh
  bằng đo đạc. STL chỉ cung cấp bề mặt; không tự chứa các thông số này.
- Bút dùng tiếp xúc cầu nhỏ + lò xo để nghiên cứu lực tì. Nét Pen có độ rộng
  cố định; chưa mô phỏng từng sợi bút, mực loang hoặc nét thư pháp.
- Collision dùng hình đơn giản và `selfCollision FALSE`; chưa kiểm chứng tránh
  va chạm chi tiết. Đai/crosstalk chỉ được quy đổi tọa độ A/B trong log, chưa
  được mô phỏng như truyền động đàn hồi hay mô hình điện động cơ bước.

## Trạng thái kiểm chứng

MATLAB/Simulink đã chạy thực tế trên R2026a Update 2 sau khi sắp xếp lại thư mục.
Kiểm thử Webots offline kiểm tra thuật toán, đối chiếu MATLAB, I/O controller,
trường node theo nguồn R2025a, đường dẫn mesh và tính nhất quán khớp/quán tính.
Kiểm thử mock không mô phỏng vật lý Webots. Kết quả nằm trong `output/validation`
của từng môi trường; phần chạy Webots thật còn chờ cài phần mềm.

Nguồn thiết kế: [madl3x/x-scara](https://github.com/madl3x/x-scara), GPL-3.0.
Mesh chuyển đổi giữ manifest nguồn, mã băm và giấy phép trong `common/assets`.
Tài liệu [Pen](https://cyberbotics.com/doc/reference/pen),
[khớp](https://cyberbotics.com/doc/reference/jointparameters),
[TouchSensor](https://cyberbotics.com/doc/reference/touchsensor) và
[bản phát hành Webots](https://github.com/cyberbotics/webots/releases).
