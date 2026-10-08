# Webots: X-SCARA + bút lông

## Mở và chạy

Mở `worlds/scara_draw.wbt` bằng **File → Open World** rồi nhấn **Run**.
Không cần tạo project hay import STL thủ công. Tất cả asset nằm trong `Simulation`
và dùng đường dẫn tương đối; controller Python chỉ dùng thư viện chuẩn.

World dùng định dạng R2025a. Có thể thử trên bản mới hơn bạn cài, nhưng hiện
chưa chạy kiểm chứng bằng Webots thật, kể cả bản 2026. Nguồn phiên bản:
[Cyberbotics releases](https://github.com/cyberbotics/webots/releases).

Nếu Webots không tìm được Python, chọn executable Python 3 trong phần
**Preferences → Python command**, ví dụ
`C:/Users/<tên_user>/AppData/Local/Programs/Python/Python312/python.exe`;
trên máy khác chọn đường dẫn cài Python tương ứng.

Robot có khoảng 2 s đến điểm đầu + 0.5 s ổn định, sau đó vẽ và nhấc bút.
Kết thúc controller giữ vị trí, không tự lặp. Muốn chạy lại hoặc xóa nét,
dùng **Reset/Reload** world. Console hiển thị giai đoạn, sai số XY và lực tì.

## Chọn đường vẽ và thuật toán

Sửa `config/demo.json`, rồi Reset/Reload:

```json
{
  "trajectory": "flower",
  "control_mode": "position",
  "feedforward": true,
  "time_scale": 1.0
}
```

Đây là trích đoạn; giữ các trường khác của file. `trajectory` nhận `flower`,
`circle`, `square`, `csv`. Với `csv`, `csv` là đường dẫn tính từ `Simulation`,
mặc định `common/trajectories/example_drawing.csv`. `time_scale=2` vẽ chậm gấp
đôi. IK kiểm tra toàn bộ quỹ đạo trước khi ra lệnh động cơ.

| Chế độ | Hành vi | File để chỉnh |
| --- | --- | --- |
| position | Gửi góc S/E và vị trí Z qua servo Webots; thích hợp mở lần đầu, kiểm tra IK/quỹ đạo | `core/model.py`, `core/trajectory.py` |
| pid | PID riêng xuất Nm tại hai khớp, N tại Z; setTorque/setForce tắt servo vị trí bên trong Webots | `core/control.py` |
| feedforward | Trong chế độ pid, bật/tắt bù M·qdd + C + ma sát + tiếp xúc ước lượng; trọng lực Z luôn được bù | `core/model.py`, `common/config/scara.json` |

Chế độ position vẫn dùng vật lý/collision. Chế độ pid dùng encoder làm phản hồi,
đạo hàm lọc thấp 30 Hz, bão hòa và anti-windup. Gains được kế thừa từ mô hình
MATLAB; cần thử/tinh chỉnh khi chạy vật lý Webots thật. Mô hình vật lý Webots
và plant giải tích MATLAB khác nhau, nên không kỳ vọng sai số giống hệt.

## Cấu trúc để dễ debug

| File/thư mục | Vai trò |
| --- | --- |
| `worlds/scara_draw.wbt` | Gravity, timestep, camera, bàn, ma sát brush-paper |
| `protos/ScaraBrush.proto` | Cây khớp, motor, encoder, khối lượng/quán tính, boundingObject, bút |
| `protos/DrawingPaper.proto` | Mặt giấy Z=0, UV texture, collision |
| `controllers/scara_draw/scara_draw.py` | Kết nối thiết bị, vòng step, warmup/hold, ghi log |
| `controllers/scara_draw/core/model.py` | FK/IK, crosstalk, feedforward giải tích |
| `controllers/scara_draw/core/trajectory.py` | Hoa/tròn/vuông/CSV, quintic, nội suy tham chiếu |
| `controllers/scara_draw/core/control.py` | PID mô-men/lực và điều kiện cho phép ghi nét |
| `controllers/scara_draw/core/diagnostics.py` | CSV và summary của từng lần chạy |
| `config/demo.json` | Lựa chọn thực nghiệm; không sửa mã để đổi hình/màu/chế độ |
| `tools/build_project.py` | Nguồn tạo world/PROTO/mesh; chỉnh ở đây nếu muốn thay cấu trúc |
| `tools/validate_project.py`, `tests/` | Kiểm thử offline; không cần cài Webots |
| `common/config/scara.json` | Tham số dùng chung với MATLAB; nằm ngoài thư mục webots |

World, PROTO và mesh là file tạo từ builder. Chỉnh trực tiếp PROTO để thử nhanh
được, nhưng tạo lại sẽ ghi đè; nên đưa thay đổi lâu dài vào builder hoặc JSON.
Khi đổi tham số vật lý, chạy `python Simulation/webots/tools/build_project.py`
từ thư mục gốc, rồi Reload world. Controller báo lỗi rõ nếu JSON và world lệch.

## Khớp và thiết bị

```text
SCARA (đế cố định, gốc vai [0,-0.110,0])
└── Z_JOINT / z_motor / z_sensor       Nâng cả carriage + tay + bút
    └── CARRIAGE
        └── SHOULDER_JOINT / shoulder_motor / shoulder_sensor
            └── SHOULDER (L1)
                └── ELBOW_JOINT / elbow_motor / elbow_sensor
                    └── ELBOW (L2)
                        └── BRUSH_BODY
                            └── BRUSH_SPRING / brush_deflection_sensor
                                └── BRUSH_TIP / brush_contact
                                    ├── tool_gps
                                    └── drawing_pen
```

Tọa độ ENU: X/Y nằm trên giấy, Z hướng lên. Encoder quay đo rad, encoder Z đo m.
`brush_contact` là force-3d, `lookupTable []` để đọc N thật thay vì hệ số mặc định
×10. Thành phần |Fz| được ghi là lực tì. GPS nằm ở đáy đầu bút, cho tọa độ thực tế.

Lò xo đầu bút K=800 N/m, C=4 Ns/m, đầu tiếp xúc là cầu bán kính 1 mm với khối lượng
10 g. Phần thân bút 30 g còn lại gắn cứng vào tay. Tổng khối lượng nâng là 1.8 kg.
Webots dùng 2 ms, ma sát brush-paper 0.18. Tất cả là giá trị khởi đầu ước lượng.

Pen hướng -Z, range 0.5 mm. Chỉ bật mực khi quỹ đạo yêu cầu, cảm biến có lực ≥0.02 N,
đầu bút gần mặt giấy và nằm trong giấy 100 × 100 mm. Nét mặc định rộng 0.5 mm.
Pen vẽ vào lớp texture trong phiên mô phỏng; file paper.png vẫn là giấy trắng.

Khớp khuỷu có stop cơ học [-π,π]; giới hạn làm việc 10–170° đặt ở motor và bộ
kiểm tra quỹ đạo. Webots yêu cầu hard stop hinge bao quanh góc zero, vì vậy
không dùng minStop dương. Chế độ raw effort có kiểm tra lệch giới hạn và chuyển
sang giữ vị trí nếu có lỗi. `selfCollision FALSE` và collision đơn giản phù hợp
thử thuật toán, chưa đại diện mọi khoảng hở của cụm lắp STL.

## Log và dùng với MATLAB

Mỗi lần chạy tạo `output/runs/YYYYMMDD_HHMMSS_xxxxxx/`:

- `parameters.json`: chụp lại tham số robot và lựa chọn demo.
- `tracking.csv`: tham chiếu/encoder, XYZ GPS, lực tì, độ nén, trạng thái mực,
  sai số XY, lệnh effort, feedback motor và tọa độ A/B/steps.
- `summary.json`: completed/interrupted/error, RMS/max XY khi yêu cầu vẽ,
  lực lớn nhất. Chỉ có khi controller kết thúc hoặc hoàn tất hình.

Tên cột có đơn vị. Trong position mode, cột `*_command_Nm/N` để trống vì lệnh
là vị trí; feedback vẫn được ghi. Motor feedback không thay thế lực đầu bút.
RMS tính trên mẫu log có pen_requested trong pha draw; không tự loại mẫu mất
tiếp xúc để che sai số. Mặc định log mỗi 5 step = 10 ms.

Trong MATLAB, ở thư mục `Simulation/matlab`:

```matlab
startup_scara
analyze_webots_run % chọn tracking.csv bằng hộp thoại
```

Đây là trao đổi CSV sau chạy; chưa có kết nối thời gian thực MATLAB–Webots hoặc
Simulink đồng mô phỏng.

## Kiểm chứng và xử lý lỗi

```powershell
# Từ Scara Robot
python Simulation/webots/tools/build_project.py
python Simulation/webots/tools/validate_project.py
```

Kết quả: `output/validation/offline_validation.json`. Bộ kiểm tra dùng định nghĩa
trường node từ nguồn Webots R2025a, kiểm tra FK/IK, CSV, PID/anti-windup, I/O
bằng mock, mesh/nguồn/đơn vị, cấu hình và quán tính. Fixture MATLAB đối chiếu
công thức độc lập. Việc này không xác nhận runtime, render hay ODE physics.

`output/validation/geometry_preview.png` là ảnh hình học offline đọc từ PROTO/STL,
có đường vẽ mục tiêu; không là ảnh chụp Webots đang chạy. Có thể tạo lại bằng
`tools/preview_geometry.py` nếu Python có NumPy và Matplotlib; controller và các
kiểm thử chính không cần hai thư viện này.

Khi test thật: mở position mode trước, kiểm tra tay/bút, giấy và Fn trong Console;
sau đó đổi pid, so sánh CSV. Nếu rung, giảm tốc bằng `time_scale=2` trước khi chỉnh
gains/timestep. Nếu có lỗi, xem dòng đầu `[SCARA ERROR]` trong Console.

- Missing device: mở đúng world, giữ nguyên tên device hoặc sửa I/O theo tên mới.
- Physical configuration changed: tạo lại bằng builder, Reload world.
- Không thấy mực: kiểm tra pen_requested/ink_enabled/Fn/Z_tip trong CSV và giấy có texture.
- Rung/mất tiếp xúc: kiểm tra K/C, gain Z, dt và lực khả dụng; chưa hiệu chỉnh trên Webots thật.
- Không tìm được mesh: giữ nguyên cây `Simulation/common` và `Simulation/webots`.

Nguồn: [X-SCARA](https://github.com/madl3x/x-scara),
[Pen](https://cyberbotics.com/doc/reference/pen),
[Motor](https://cyberbotics.com/doc/reference/motor),
[JointParameters](https://cyberbotics.com/doc/reference/jointparameters),
[TouchSensor](https://cyberbotics.com/doc/reference/touchsensor),
[Physics](https://cyberbotics.com/doc/reference/physics).
