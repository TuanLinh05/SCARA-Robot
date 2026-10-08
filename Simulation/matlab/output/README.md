# Kết quả MATLAB/Simulink

- `figures/`: ảnh 3D, tracking, so sánh PID, sơ đồ Simulink.
- `data/matlab/`: simulation.mat/csv và controller_comparison.mat.
- `data/simulink/`: simulation.mat/csv chạy từ SLX.
- `logs/`: log chạy MATLAB; log cũ trước sắp xếp được giữ để đối chiếu.
- `validation/validation.json`: kiểm chứng đã chạy lại sau sắp xếp.
- `data/matlab/bk/`, `data/simulink/bk/`: CSV/MAT riêng của hình BK.
- `figures/bk_drawing.png`, `bk_tracking.png`: hình BK và đồ thị điều khiển.
- `validation/bk_validation.json`: hình học, nhấc bút và so sánh MATLAB/Simulink BK.
- `cache/`, `codegen/`: file tự sinh của Simulink, không thêm vào source path.

MAT có thể giữ đường dẫn trong biến p từ thời điểm tạo. Hàm mới đọc đường dẫn
theo vị trí project hiện tại; không dùng đường dẫn trong MAT cũ để tìm source.
