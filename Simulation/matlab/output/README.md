# Kết quả mô phỏng hiện tại

- `validation/validation.json`: kiểm thử MATLAB/Simulink hiện hành.
- `figures/`: sơ đồ khối, lắp ráp STL, quỹ đạo và telemetry HOME.
- `data/`: CSV/MAT của lần chạy gần nhất; HOME CSV.
- `reference/`: MAT nén chứa xung/đích do Python planner tạo, có thể xóa để tái sinh.
- `cache/`, `codegen/`: Simulink tự sinh, không phải mã nguồn.

Lỗi XY là lỗi mô hình, không phải phép đo nét vẽ thực. Không có encoder/đo lực để xác nhận độ chính xác cơ khí. Các trường hợp backlash/lag là giả định có thể chỉnh.