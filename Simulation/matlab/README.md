# MATLAB / Simulink — SCARA thực nghiệm

Mô phỏng hiện dùng firmware **SCARA_CART_NC_HOME_V5_R9** và bộ lập quỹ đạo Python đang dùng trong GUI. Robot thực điều khiển stepper hở vòng; không có encoder hoặc PID mô-men. Vì vậy mô hình chính dùng **số xung**, thay mô hình PID/động lực học giả định trước đây.

## Chạy

Cần MATLAB + Simulink, Python 3 và thư mục `Software/src` trong cùng dự án. Không cần Simscape hoặc Robotics System Toolbox. Không mở COM, nạp firmware hay điều khiển robot.

Trong MATLAB, đặt Current Folder ở `Simulation/matlab`:

```matlab
run_scara                       % Giao diện 3D STL, 8 mẫu vẽ và slider khớp
startup_scara;
out = scara_simulink('flower');  % Chạy Simulink + xuất CSV/MAT và biểu đồ
build_scara_simulink(true);      % Dựng lại sơ đồ nếu đã sửa builder
validate_scara;                 % Kiểm thử số xung, động học, NC, watchdog, GUI
scara_home_log;                 % Xem telemetry HOME thật đã ghi
```

Mở `simulink/models/scara_stepper.slx` để xem các khối. Nếu muốn đổi mẫu khi chạy trực tiếp: `scara_setup('star'); sim('scara_stepper');`. Mẫu: flower, circle, square, star, heart, BK, LINH, THOA. Giao diện phát lại nhanh hơn thời gian thực; trục thời gian của log/biểu đồ vẫn là thời gian mô phỏng.

Sáu mẫu đã vẽ thật dùng lại kích thước/tâm/tốc độ trong log: HEART, CIRCLE, BK, LINH, STAR, FLOWER. Hai mẫu square/THOA dùng thông số demo. Chỉnh các giá trị `drawing_settings` trong snapshot để đổi kích cỡ hoặc mặt giấy rồi tái sinh reference.

Nếu MATLAB không tìm thấy Python: `p=scara_params(); p.python='đường dẫn/python.exe'; scara_simulink('BK',p);`.

## Thông số từ lần chạy thật

Nguồn được giữ trong `config/robot_calibration.json`: log `cartesian_20261008_104801_16264_86b4eff2.jsonl`, HOME epoch 1. Log ghi GUI R13; mô phỏng dùng bộ lập quỹ đạo hiện tại trong Software R14.

| Thông số | Mặc định |
|---|---:|
| J1 | 26.2939453125 xung/độ |
| J2 | 39.41015625 xung/độ |
| Z | 1600 xung/mm |
| Hệ số bù k | 0.338806 |
| Probe đai | ΔA=128, ΔB=65; β=0.5078125 |
| Vi bước theo cấu hình Z/J1/J2 | 16 / 8 / 8 |
| Quãng đo Z | 87277 xung ≈ 54.548 mm |
| Tốc độ quét HOME Z | **3 mm/s**, danh định 4800 xung/s |
| Quét HOME J2 | 800 xung/s |
| Timers PATH/motion tối đa | 1600 xung/s theo cấu hình |
| Watchdog | 350 ms |
| Chu kỳ quan sát mô phỏng | 2 ms |

Quan hệ dùng **góc khuỷu tương đối q2**:
- A = q1.
- B = q2 + k·q1.
- q2 = B − k·A.
- Hướng arm 2 so với bệ = q1 + q2.

A/B được đổi sang rad từ số xung đã đo. Số xung chuẩn hóa theo thứ tự firmware **[Z, A, B]**, đã bỏ offset HOME; đây không phải góc rotor motor. Trường `config.coupling=1.333333` là giá trị trước tự calib, **không dùng thay k đã đo**. β = k·F2/F1 nên không lấy β làm hệ số góc trực tiếp.

L1=L2=98 mm, lệch TCP ban đầu bằng 0; góc giữa công tắc giả định 180°. Z được đổi sang mm với bước vít me giả định 2 mm/vòng. Các giả định này chưa được đo độc lập. Z=0 tại công tắc dưới, +Z hướng lên. X/Y gốc tại tâm J1; góc 0 hướng +Y, góc dương hướng −X. Giấy mặc định tâm (0,165) mm, Z=20 mm, nhấc bút 2 mm.

## Sơ đồ Simulink

Mô hình sử dụng các khối có sẵn: **From Workspace, Gain, Sum, Trigonometric Function, Unit Delay, Compare To Constant, Logical Operator, Enabled Subsystem, Backlash, Discrete Transfer Fcn, Switch, Scope, To Workspace**. Không có MATLAB Function, S-Function hoặc vòng PID tự viết trong SLX.

1. Python gọi đúng IK, hình vẽ, lookahead và profile quintic v²(distance) của GUI; dựng lại các cạnh xung DDA nguyên và lấy mẫu số đếm mỗi 2 ms.
2. **From Workspace tắt Interpolate**: số đếm giữ nguyên giữa mẫu, không biến thành xung phân số.
3. Safety kiểm tra soft limits, hiệu lực dữ liệu, tuổi heartbeat và sáu NC theo chiều chuyển động. Cho lùi khỏi một biên; hai NC cùng mở hoặc lỗi khác sẽ chốt dừng đến lần chạy mô phỏng mới.
4. Enabled Subsystem giữ số đếm cuối khi dừng.
5. Gain/Sum quy đổi số đếm → q1/q2/Z với hệ số bù đo được.
6. Backlash và bộ lọc bậc một là lớp khảo sát độ rơ/đáp ứng, mặc định tắt. FK dùng các khối lượng giác rồi ghi XYZ, xung, lỗi mô hình, pen và trạng thái cho phép.

[From Workspace — MathWorks](https://www.mathworks.com/help/simulink/slref/fromworkspace.html), [Backlash — MathWorks](https://www.mathworks.com/help/simulink/slref/backlash.html).

Bộ tạo quỹ đạo chạy offline trên máy tính được giữ bằng Python để tránh viết lại thuật toán đã kiểm thử. SLX mô phỏng chuyển động từ số đếm; **không mô phỏng điện áp PUL, optocoupler TB6600, độ rộng HIGH 50 µs, USB/FIFO theo thời gian thực hoặc từng bước nội bộ HOME**. Thời gian dựng xung không gồm chờ upload và trễ chuyển segment, nên không dùng nó như dự báo thời gian vẽ chính xác. Biểu đồ HOME lấy mẫu telemetry USB thật; các lần đặt lại gốc số đếm không phải robot nhảy vị trí.

## Khảo sát vì sao nét có thể méo

```matlab
startup_scara;
p=scara_params();
p.backlash=[deg2rad(.2);deg2rad(.2);0]; % độ rộng vùng rơ J1/J2, Z
p.lagSeconds=[.01;.01;0];              % độ trễ đáp ứng giả định
p.actualL2=.099;                       % hình học thật khác bộ lập quỹ đạo
p.actualToolOffset=[.001;0];           % đầu bút lệch ngang 1 mm
out=scara_simulink('star',p);
```

Đây là **kịch bản độ nhạy**, không phải các thông số đã xác định từ ảnh. Không tự bù rơ hoặc sửa firmware bằng những giá trị này. Muốn hiệu chỉnh thật cần đo tâm khớp → đầu bút, góc công tắc và sai lệch khi đảo chiều.

```matlab
p=scara_params();
p.disconnectTime=1;                  % ngừng heartbeat từ giây 1
scara_simulink('circle',p);           % chốt dừng sau 350 ms
p=scara_params();p.forceOpenNc(3:4)=true; % J1 hai NC mở
scara_simulink('circle',p);
```

NC trong mô hình là tín hiệu thử nghiệm đã ổn định; không tái tạo nhiễu điện hoặc thuật toán xác nhận tiếp điểm của ISR. Bảo vệ robot thật vẫn do firmware đảm nhiệm.

## Thư mục gọn

- `config/`: snapshot calib và tham số SI.
- `src/`: động học, đọc log/xuất dữ liệu, sinh reference, hiển thị STL.
- `simulink/`: builder, setup/runner và **một** model SLX hiện hành.
- `tools/export_firmware_reference.py`: dùng lại planner hiện tại, không COM.
- `tests/validate_scara.m`: kiểm thử chính.
- `output/`: kết quả hiện tại; reference được cache nén MAT, tự tạo khi cần.

Sau khi đổi cấu hình calib, xóa `output/reference` để dựng lại xung. Các script/mô hình PID cũ, ví dụ trùng và cache build đã dọn; ZIP dự phòng ở `.cleanup_backup/matlab_before_firmware_alignment.zip` ngoài thư mục MATLAB.

Webots vẫn dùng `Simulation/common/config/scara.json` để nghiên cứu vật lý ước lượng. File đó không còn là mặc định MATLAB và chưa được đồng bộ với robot thực trong nhiệm vụ này.

## Giới hạn kết quả

Kết quả kiểm thử ở `output/validation/validation.json`. Sai số tính bằng phần mềm là giữa FK và đường đích lấy mẫu; không phải phép đo nét trên giấy. Log thật trước đó xác nhận các endpoint/số đếm của 21 nét, nhưng không xác nhận driver hoặc cơ khí thực thi đúng từng xung. Mô hình không biết trượt bước, độ rơ đã đo, độ võng bút, lực tì hoặc độ không phẳng của giấy. Độ rộng/đậm mực không được mô phỏng định lượng. STL giữ từ Hardware; vị trí khung, tầng lắp và bút vẫn là xấp xỉ.
