# Dữ liệu chung MATLAB và Webots

- `config/scara.json`: nguồn tham số số duy nhất, đơn vị m, rad, s, kg, N, Nm.
- `trajectories/example_drawing.csv`: cột `x_mm,y_mm,pen`; trạng thái pen trên
  hàng i áp dụng cho đoạn từ hàng i-1 đến i. `0` nhấc, `1` hạ bút để vẽ.
- `assets/meshes`: STL đã chuyển từ mm sang m, đặt về hệ tọa độ từng khâu.
- `assets/manifest.json`: 11 instance STL gốc, dấu trục, dịch chuyển, nguồn,
  mã băm SHA256, giấy phép và mức xấp xỉ lắp ráp.
- `assets/textures/paper.png`: giấy trắng cho Webots Pen, không lưu nét chạy
  mô phỏng vào file PNG gốc.

## Tham số để debug

| Nhóm | Trường trong scara.json | Ý nghĩa |
| --- | --- | --- |
| Hình học | L1, L2, base, toolOffset, paperHalfSize | Chiều dài, tâm vai, lệch đầu bút, khổ giấy |
| Giới hạn | jointLimits, zLimits, singularityMargin | Góc S/E và chiều cao đầu bút tự do; giới hạn làm việc giả định |
| Truyền động | elbowCrosstalkRatio, stepsPerDegree, stepsPerMeterZ | Quy đổi A=S, B=E+S/3 và tọa độ vi bước theo firmware |
| Động lực học | mass1, mass2, massTool, zMass, rotorInertia | Khối lượng/quán tính ước lượng; zMass bao gồm mọi phần được nâng |
| Ma sát | viscous, coulomb, brushMu | Cản nhớt, ma sát khớp và tiếp xúc bút/giấy |
| Bút | brushK, brushC, brushCompression, penLift | Lò xo/giảm chấn, độ nén đích và độ nhấc bút |
| Điều khiển | Kp, Kd, Ki, actuatorLimits, feedforward | PID; giới hạn là mô-men KHỚP và lực Z, chưa là thông số rotor |
| Thời gian | dt, gravity | Bước mô phỏng mặc định 2 ms, gia tốc trọng trường |
| Hiển thị | lineWidth0, lineWidthPerN | Webots dùng độ rộng cố định lineWidth0; quan hệ theo lực ở MATLAB là minh họa |

S/E dương quay quanh +Z; tay ở góc 0 hướng +Y. Z=0 là đầu bút **chưa biến dạng**
đúng mặt giấy. Z âm biểu diễn nén bút; Webots spring làm đầu bút thực tế nằm gần
mặt giấy, thay vì cố xuyên đầu bút vào giấy.

Sau khi đổi hình học/thông số vật lý, chạy từ thư mục gốc `Scara Robot`:

```powershell
python Simulation/webots/tools/build_project.py
python Simulation/webots/tools/validate_project.py
```

Reload world Webots. Controller kiểm tra mã băm cấu hình vật lý và từ chối
chạy khi world cũ không khớp JSON. Đổi gains PID, độ nén bút hoặc chọn đường vẽ
không cần dựng lại world. MATLAB đọc lại JSON khi gọi `scara_params`; nếu đã
đặt biến `scara_p` trong workspace, gọi `scara_setup('flower',scara_params())`
để áp dụng mặc định mới.

Đổi L1/L2 làm scale đồ họa theo Y, phục vụ thử thuật toán; chưa là thiết kế CAD
cho một robot với chiều dài mới. Khi đổi cấu hình, các fixture đối chiếu MATLAB
trong Webots tests cần tạo lại bằng `startup_scara; validate_webots_interop`.
