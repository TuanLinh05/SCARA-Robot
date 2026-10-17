/*
 * ============================================================================
 * FILE: cart_core_internal.h
 * ============================================================================
 * [EN] Internal Helper Prototypes Shared Between `cart_core.c` & `cart_commands.c`
 *      Exposes internal validation, coupling update, raw move setup, and
 *      calibration launch helpers to the command parser without polluting the
 *      public `cart_core.h` header.
 *
 * [VI] Các Hàm Nội bộ Dùng Chung Giữa `cart_core.c` và `cart_commands.c`
 *      Cung cấp các hàm kiểm tra đầu vào, cập nhật hệ số bù đai `beta`, khởi tạo
 *      lệnh chạy thô và kích hoạt pha dò gốc cho bộ giải mã lệnh mà không làm
 *      phức tạp file giao diện công khai `cart_core.h`.
 * ============================================================================
 */

#ifndef SCARA_CART_CORE_INTERNAL_H
#define SCARA_CART_CORE_INTERNAL_H

#include "cart_core.h"

/*
 * [EN] Check all 3 axes' switches and driver states. Returns NULL if healthy,
 *      or a static error reason string ("motor_fault", "released", "switch_io", etc.).
 * [VI] Kiểm tra trạng thái công tắc và driver của cả 3 trục. Trả về NULL nếu an toàn,
 *      hoặc chuỗi mã lỗi tương ứng ("motor_fault", "released", "switch_io", ...) nếu có sự cố.
 */
const char *ct_inputs(struct ct_state *control, uint32_t now);

/*
 * [EN] Recompute fixed-point belt coupling ratio `control->beta` (scaled by `CT_Q = 2^20`)
 *      from `coupling_ppm` and axis scale factors `factor[1]`, `factor[2]`.
 * [VI] Tính toán lại hệ số bù đai dấu phẩy tĩnh `control->beta` (nhân với `CT_Q = 2^20`)
 *      từ `coupling_ppm` và hệ số quy đổi xung `factor[1]`, `factor[2]`.
 */
void ct_beta_update(struct ct_state *control);

/*
 * [EN] Verify whether motor step coordinates `position[CT_AXES]` lie within
 *      calibrated Z travel and J1/J2 angular soft limits (with `margin_md` margin).
 * [VI] Kiểm tra tọa độ bước `position[CT_AXES]` có nằm trong giới hạn mềm của
 *      trục Z và góc khớp vai J1 / khuỷu J2 (trừ biên an toàn `margin_md`) hay không.
 */
bool ct_pose_valid(const struct ct_state *control, const int32_t position[CT_AXES], int margin_md);

/*
 * [EN] Set up a coordinated multi-axis DDA move from `control->pos` to `target` at `rate` pps.
 *      When `park == true`, the move runs in internal `CT_PARK` mode (no backlash take-up).
 * [VI] Thiết lập bộ nội suy DDA đa trục từ vị trí hiện tại `control->pos` tới `target` ở tốc độ `rate`.
 *      Khi `park == true`, lệnh chạy ở chế độ nội bộ `CT_PARK` (không phát xung bù rơ).
 */
enum ct_result ct_raw_move(struct ct_state *control, const int32_t target[CT_AXES], uint32_t rate, bool park);

/*
 * [EN] Compute minimum number of ticks required to ramp between `end` and `peak`
 *      rates under quintic smootherstep profile with acceleration `accel`.
 * [VI] Tính số xung (ticks) tối thiểu cần thiết để tăng/giảm tốc giữa `end` và `peak`
 *      theo đường cong đa thức bậc 5 với gia tốc `accel`.
 */
uint32_t ct_path_ramp(uint32_t peak, uint32_t end, uint32_t accel);

/*
 * [EN] Start a single-axis calibration sequence on `axis` for initialization stage `stage`.
 * [VI] Bắt đầu chu trình dò công tắc hành trình trên trục `axis` ứng với bước `stage`.
 */
enum ct_result ct_begin_cal(struct ct_state *control, unsigned axis, enum ct_stage stage, uint32_t now);

#endif /* SCARA_CART_CORE_INTERNAL_H */
