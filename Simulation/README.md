# Mô phỏng SCARA

Hai môi trường phục vụ hai mục đích:

- **MATLAB/Simulink hiện hành** bám robot stepper và firmware R9: số xung DDA, calib đo trong log, bù đai J1/J2, NC/watchdog và quỹ đạo của GUI. Giao diện STL và các kịch bản độ rơ/đáp ứng giúp nghiên cứu sai lệch nét vẽ.
- **Webots** giữ mô hình vật lý nghiên cứu ban đầu: PID, ma sát, khối lượng, tiếp xúc bút/giấy là ước lượng. Chưa đồng bộ các giá trị calib R9; không dùng mặc định Webots thay số đo của robot thực.

## MATLAB/Simulink

Đặt Current Folder ở `Simulation/matlab`, chạy:

```matlab
run_scara
```

Hoặc:

```matlab
startup_scara;
out=scara_simulink('flower');
validate_scara;
```

Model duy nhất hiện hành: `matlab/simulink/models/scara_stepper.slx`, dùng khối Simulink chuẩn, không có MATLAB Function/S-Function tự viết. Tham số thực nghiệm ở `matlab/config/robot_calibration.json`; hướng dẫn và giới hạn tại [matlab/README.md](matlab/README.md).

Góc q2 là góc tương đối giữa hai arm; B=q2+0.338806*q1 theo log đã calib. Z=0 tại công tắc dưới, +Z lên. L1=L2=98 mm và bước vít me 2 mm/vòng vẫn là giả định người dùng. Robot không có encoder, nên số xung không xác nhận vị trí vật lý hay độ chính xác nét trên giấy.

## Webots

1. Mở `webots/worlds/scara_draw.wbt` bằng Webots.
2. Run để chạy demo.
3. Sửa `webots/config/demo.json`, Reset/Reload để đổi hình hoặc chế độ điều khiển.

Xem [webots/README.md](webots/README.md). `common/config/scara.json`, CSV và assets vẫn phục vụ mô hình Webots cũ. Không thay cấu hình vật lý của nó trong lần cập nhật MATLAB này. Fixture `webots/tests/fixtures/matlab_oracle.json` giữ bộ đối chiếu cũ; không tái sinh nó bằng planner R9 vì tham số/quy ước Z đã khác.

## Cấu trúc

```text
Simulation/
  matlab/       Một model stepper hiện hành, giao diện, source và tests
  webots/       World/PROTO/controller vật lý nghiên cứu
  common/       Cấu hình Webots, CSV và mesh/giấy phép dùng lại
```

STL nguồn trong `Hardware/arm/STL`, 11 instance ghép theo khâu. Khung, vị trí tầng lắp, motor và bút vẫn là xấp xỉ. Nguồn CAD: [madl3x/x-scara](https://github.com/madl3x/x-scara), GPL-3.0; manifest và giấy phép nằm trong `common/assets`.