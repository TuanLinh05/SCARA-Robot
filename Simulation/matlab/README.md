# MATLAB/Simulink: X-SCARA gắn bút lông

Bộ mô phỏng ban đầu cho bài tập: hiển thị 3D từ STL có sẵn, động học/quỹ đạo vẽ,
động lực học và PID, cùng một mô hình Simulink chỉnh sửa được. Các file gốc trong
`Hardware` và `Firmware` không bị sửa.

## Chạy nhanh trong MATLAB

Mở thư mục `Simulation/matlab` trong MATLAB rồi chạy:

```matlab
run_scara
```

Cửa sổ có mô hình 3D xoay được bằng chuột, thanh chỉnh góc vai/khuỷu/Z, bốn hình
vẽ mẫu và nhập CSV. Chọn `Kinematics (IK)` để xem chuyển động lý tưởng hoặc
`PID + dynamics` để chạy vòng điều khiển. `Model feedforward` bật/tắt bù theo
mô hình; `Plot / export last run` xuất MAT và CSV vào `output/data/matlab`.
`run_scara` tự gọi `startup_scara` để thêm đường dẫn các thư mục nguồn.

Chạy thuật toán trực tiếp, không cần giao diện:

```matlab
startup_scara
p = scara_params();
ref = scara_reference('flower',p); % circle, square, bk hoặc đường dẫn CSV
out = scara_simulate(ref,p);
scara_plot(out,p);
scara_export(out,ref,p);
```

Chạy mô hình Simulink đã lưu:

```matlab
startup_scara
p = scara_params();
scara_setup('flower',p);
open_system(fullfile(p.modelsRoot,'scara_brush.slx'));
% Nhấn Run, hoặc:
out = scara_simulink('flower');
```

Để tạo lại file SLX từ mã nguồn: `build_scara_simulink`. Hàm này từ chối tạo lại
khi mô hình đang mở có thay đổi chưa lưu. MATLAB cơ bản đủ cho mô phỏng và 3D;
Simulink cần cho SLX. Đã kiểm tra trên MATLAB/Simulink R2026a Update 2. Bộ này
không cần Robotics System Toolbox hoặc Simscape Multibody.

## Vẽ chữ BK theo hình tham khảo

Chạy `run_scara`, chọn **BK** ở ô Trajectory, rồi **Run + animate**. Hoặc:

```matlab
startup_scara
[out,ref] = draw_bk(); % Mô phỏng PID, xem hình và lưu dữ liệu BK riêng
% Với Simulink:
out_sl = scara_simulink('bk');
```

Nút **Open Simulink model** dùng quỹ đạo đang chọn trong giao diện, kể cả BK.

Hình giữ tọa độ nguồn X/Y từ 0 đến 0.5: B có thân X=0 và hai nửa đường tròn
bán kính 0.125; K có thân X=0.25, giao điểm (0.25,0.25) và hai đầu chéo ở
(0.5,0.5)/(0.5,0). Các trục tọa độ trong ảnh tham khảo không là nét vẽ.
Mẫu thu nhỏ đồng dạng, đặt giữa giấy: mặc định **70 × 70 mm**, trong giấy
100 × 100 mm. Không dùng trực tiếp kích thước 0.5 m vì vượt tầm với robot.

Robot vẽ ba nét liên tục: thân + hai cung B, thân K, hai nét chéo K. Bút nhấc
6 mm giữa các nét; các đoạn dùng quintic, dừng êm ở chỗ đổi hướng. Cung B được
tính bằng sin/cos, không ghép nhiều đoạn CSV ngắn có dừng ở từng điểm.
Hình học và tốc độ nằm trong `src/trajectory/scara_bk_path.m`.

`draw_bk` lưu `output/figures/bk_drawing.png`, `bk_tracking.png` và
`output/data/matlab/bk/simulation.csv/.mat`, để giữ riêng dữ liệu mẫu này.
`validate_bk(true)` kiểm tra hình học, nhấc bút, giới hạn và đối chiếu Simulink;
báo cáo nằm trong `output/validation/bk_validation.json`.

## Thử thuật toán của bài tập

| Mục tiêu | File/hàm cần chỉnh |
| --- | --- |
| Chiều dài tay, góc giới hạn, khối lượng, ma sát, bút, gains PID | `../common/config/scara.json`, đọc bởi `config/scara_params.m` |
| Động học thuận và Jacobian | `scara_fk.m` |
| Động học ngược, nhánh nghiệm, giới hạn và kỳ dị | `scara_ik.m` |
| Quỹ đạo và nhấc/hạ bút | `scara_reference.m` |
| PID hoặc thuật toán điều khiển khác | `scara_controller.m` |
| Ma trận quán tính, Coriolis, lực tiếp xúc | `scara_dynamics.m`, `scara_accel.m` |
| Vòng điều khiển dạng sơ đồ khối | `simulink/models/scara_brush.slx` |
| So sánh PID và PID có feedforward | `compare_scara_controllers.m` |
| Vị trí các mesh trong cụm lắp ráp | `scara_meshes.m`, `scara_update.m` |

Khối MATLAB Function trong SLX gọi cùng các hàm điều khiển và động lực học như
bản MATLAB. Đổi thuật toán trong `scara_controller.m` sẽ tác động cả hai môi
trường. Đây là mô hình động lực học giải tích với khớp lý tưởng; chưa có mô hình
điện động cơ bước, bộ phát xung STM32 hay đai đàn hồi.

Ví dụ thay gain và thử ảnh hưởng tiếp xúc:

```matlab
p = scara_params();
p.Kp(1:2) = [1; 0.7];
p.brushK = 500;
p.feedforward = false;
ref = scara_reference('circle',p);
out = scara_simulink('circle',p);
```

`../common/trajectories/example_drawing.csv` có ba cột `x_mm,y_mm,pen`. Giá trị `pen` trên hàng i áp
dụng cho đoạn từ hàng i-1 đến hàng i. `pen=0` di chuyển với bút nhấc; `pen=1`
vẽ với bút hạ. Mỗi đoạn dùng thời gian quintic để dừng êm ở các nút. Khổ giấy
mẫu là 100 × 100 mm; chưa có bộ đọc SVG, xử lý ảnh hoặc tối ưu thứ tự nét.

## Hệ tọa độ và phương trình

Trong các hàm tính toán dùng mét, radian, giây, N và Nm. Giao diện/CSV dùng
mm và độ với tên cột chỉ rõ đơn vị. `q = [S; E; z]`, trong đó E là góc khuỷu
tương đối với đoạn tay thứ nhất. Gốc giấy ở giữa tờ giấy; tâm vai ở
`[0; -0.110]` m. Góc 0 hướng theo +Y; góc dương quay về -X. Quy ước dấu này
theo `x_scara_fwdk` trong mã nguồn repo.

Với đầu bút không có lệch XY:

```text
x = base_x - L1 sin(S) - L2 sin(S+E)
y = base_y + L1 cos(S) + L2 cos(S+E)
z = free_tip_height
```

`scara_ik` cũng xử lý `toolOffset`, kiểm tra bán kính với tới, vùng kỳ dị và
giới hạn khớp. Chiều dài L1/L2 mặc định 98 mm theo repo. Góc giới hạn trong bộ
mô phỏng là giá trị làm việc giả định, cần đo trên robot thật. Kiểm tra động
học không xác nhận tránh va chạm giữa các chi tiết STL.

Tọa độ điều khiển tương đương firmware:

```text
A = S
B = E + S/3
```

Tỷ số 3 và steps/degree lấy từ Configuration.h của X-SCARA, với motor 200
steps/rev và vi bước 16. A/B không phải góc rotor. CSV steps là tọa độ tuyệt
đối đã làm tròn để phân tích; chưa là luồng xung có homing/giới hạn tốc độ
để nạp vào STM32. Các mô-men trong mô phỏng là mô-men tại khớp; muốn quy đổi
về trục động cơ phải xét cả tỷ số truyền và hiệu suất.

Động lực học:

```text
M(q) qdd + C(q,qd) + friction(qd) + gravity = actuator + J' F_contact
```

Hai đoạn tay được xấp xỉ bằng thanh đồng nhất, bút bằng khối lượng tập trung.
Ma trận M có liên kết giữa hai khớp quay. Z dùng khối lượng của toàn cụm nâng;
trọng lực tác động vào Z. PID có bù trọng lực, bù mô hình tùy chọn, giới hạn
mô-men/lực và chống tích lũy sai số khi bão hòa.

Mô hình bút dùng chiều cao của đầu bút khi chưa biến dạng: z < 0 là độ nén
đầu bút/lò xo, không phải độ xuyên vào giấy. Khi z <= 0:

```text
Fn = max(0, -k*z - c*zd)
Fxy = -mu*Fn*vxy / sqrt(norm(vxy)^2 + 0.001^2)
```

Nét chỉ được ghi khi có lực tì và vị trí nằm trên giấy. Cột
`illustrative_line_width_mm` dùng quan hệ tuyến tính giả định giữa lực và
độ rộng nét. Hình vẽ hiển thị bằng đường tâm có độ dày cố định: chưa mô phỏng
biến dạng sợi bút, mực, nhòe hoặc nét thư pháp. Nếu dùng bút lông marker,
độ đàn hồi này có thể đại diện cho giá kẹp có lò xo; cần đo lại k và c.

## Độ chính xác và việc cần hiệu chỉnh

- Bốn tấm vai/khuỷu, hai bản đế, bốn thành bên và một rod hub được đọc trực
  tiếp từ STL: tổng 11 instance. Tâm lỗ XY suy ra từ mesh. Khoảng cách các
  tầng theo Z và vị trí thành bên là lắp ráp xấp xỉ.
- Thanh dẫn hướng, nhôm định hình, motor và trục dùng hình học đơn giản.
  Bút và giá kẹp là hình khái niệm, chưa là thiết kế giá kẹp để in 3D.
- Khối lượng, quán tính motor, ma sát, lực cực đại, độ cứng/giảm chấn và
  quan hệ lực–độ rộng nét là thông số ước lượng, tập trung trong một file.
- Chưa mô phỏng độ rơ, mất bước, đàn hồi đai, độ võng tay, tiếp xúc mesh
  hay va chạm. Bộ mô phỏng phù hợp để thử thuật toán ban đầu; chưa thay thế
  mô hình cơ khí được hiệu chỉnh hoặc phép đo trên robot.
- Feedforward mặc định sử dụng cùng mô hình với plant, nên sai số số học
  rất nhỏ. Không suy ra độ chính xác robot thật từ các giá trị đó. Tắt
  feedforward hoặc thay thông số/gains để phân tích độ nhạy.

Với robot vẽ, nên đặt bút thẳng đứng, có giá kẹp đàn hồi nhẹ, hiệu chỉnh tọa
độ đầu bút và mặt giấy, rồi thử nét cơ bản trước khi thử tranh. Với hình có
nét cần hướng nghiêng cố định, cần bổ sung bậc tự do đầu bút hoặc ràng buộc
quỹ đạo; cấu hình 2R+Z hiện tại không điều khiển hướng đầu bút độc lập.

## Kiểm chứng và kết quả mẫu

Chạy `validate_scara(true)` để kiểm tra:

- 100 trường hợp FK/IK, Jacobian bằng sai phân, M xác định dương và quan hệ
  năng lượng Coriolis; thêm trường hợp đầu bút lệch XY.
- Từ chối điểm ngoài tầm với, điểm kỳ dị và tâm gập không xác định.
- Lực tì bằng 0 khi nhấc, lực đàn hồi khi nén, bão hòa và chống windup PID.
- Vẽ circle/flower/square và CSV, kiểm tra sai số, giới hạn và trạng thái bút.
- Đọc STL, render mô hình và so sánh nghiệm RK4 với Simulink ode4.

`output/validation/validation.json` chứa số liệu kiểm tra. `output/figures/scara_3d.png`,
`arm_detail.png`, `tracking.png` và `simulink_model.png` là các ảnh mẫu.
`output/data/matlab/simulation.mat` giữ dữ liệu SI và tham số; CSV có cột góc, vị trí,
lực tì, sai số, mô-men và tọa độ bước để dùng trong báo cáo bài tập.

Với quỹ đạo hoa và bộ tham số mặc định, bản MATLAB và Simulink khác nhau tối đa
2.45e-15 ở các trạng thái. PID bù trọng lực có sai số XY RMS khoảng 1.1406 mm;
PID có bù mô hình khoảng 0.00015 mm trong mô hình lý tưởng này. Những giá trị
này dùng để kiểm chứng và so sánh thuật toán, không là thông số độ chính xác
của phần cứng. `output/figures/controller_comparison.png` chứa đồ thị so sánh và
`output/figures/app_ui.png` là ảnh giao diện đang chạy.

## Cấu trúc và log Webots

`src/kinematics`, `dynamics`, `control`, `trajectory`, `simulation`, `io`,
`visualization` chia hàm theo vai trò. `simulink` chứa hàm dựng/chạy SLX,
`tests` chứa kiểm chứng, `examples` chứa so sánh PID và `analyze_webots_run`.
`startup_scara` chỉ thêm thư mục nguồn và đặt cache/codegen ở `output`.

`startup_scara; analyze_webots_run` đọc CSV cảm biến do Webots xuất và vẽ
quỹ đạo, sai số, lực tì và Z. `validate_webots_interop` tạo fixture đối chiếu
FK/PID/quỹ đạo cho Python tests. Đây là trao đổi file sau chạy, chưa là đồng
mô phỏng thời gian thực. Các tham số dùng chung nằm ở `../common/config/scara.json`.

## Nguồn tham khảo

- X-SCARA, Alex Mircescu: https://github.com/madl3x/x-scara
- Cấu hình hình học và truyền động: https://github.com/madl3x/x-scara/blob/master/firmware/CONFIGURE.md
- Quy ước động học và crosstalk: https://github.com/madl3x/x-scara/blob/master/firmware/Marlin-2.0.x/Marlin/src/module/scara.cpp
- Tỷ số crosstalk: https://github.com/madl3x/x-scara/blob/master/firmware/Marlin-2.0.x/Marlin/Configuration.h
- Hướng dẫn lắp ráp và CAD: https://github.com/madl3x/x-scara/blob/master/hardware/arm/README.md
- MATLAB Function: https://www.mathworks.com/help/simulink/slref/matlabfunction.html

STL và thiết kế gốc thuộc repo X-SCARA, công bố theo GPL-3.0. Giữ nguồn và
tuân thủ giấy phép của repo khi phân phối lại thiết kế hoặc dữ liệu mesh.
