# Kiểm tra log nét vẽ

Nguồn: `D:\HK261\Robot\HPMR\Scara Robot\Software\dist\SCARA_Cartesian_NC_v5_R14\logs\cartesian_20261008_104801_16264_86b4eff2.jsonl`

Báo cáo đọc lệnh và telemetry. Không mở COM và không di chuyển robot.

| Mẫu | Cỡ ngang (mm) | Sai lệch mô hình tối đa (mm) | Hoàn tất | Trễ timer tăng lúc vẽ |
|---|---:|---:|---|---:|
| HEART | 46.32 | 0.1398 | Có | 0 |
| CIRCLE | 46.09 | 0.1387 | Có | 0 |
| BK | 46.09 | 0.1511 | Có | 0 |
| LINH | 46.09 | 0.1391 | Có | 0 |
| STAR | 46.09 | 0.1367 | Có | 0 |
| FLOWER | 41.07 | 0.1435 | Có | 0 |

Nét đóng có telemetry xác nhận đúng tọa độ xung cuối: 11/11; xem JSON để đối chiếu từng nét.

Đây không phải phép đo vị trí vật lý. Log không xác nhận motor/bút đã đi đúng các xung đó.

Điểm cần đo tiếp: góc thật của hai đầu hành trình, khoảng cách tâm J1–J2 và tâm J2–đầu bút thực; độ lệch/độ rơ khi đảo chiều và lực tì bút.
