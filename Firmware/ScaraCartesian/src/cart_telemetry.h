/*
 * ============================================================================
 * FILE: cart_telemetry.h
 * ============================================================================
 * [EN] Thread-Safe Telemetry Snapshot & JSON Serializer Interface
 *      Defines `struct ct_status` so the main loop can copy a lightweight
 *      snapshot of `struct ct_state` while holding `irq_lock()`, unlock IRQs
 *      immediately, and format the JSON status string outside the critical
 *      section without delaying stepper pulse interrupts!
 *
 * [VI] Giao diện Chụp Ảnh Trạng thái (Snapshot) & Đóng gói Bản tin JSON
 *      Định nghĩa `struct ct_status` giúp vòng lặp chính sao chép nhanh các
 *      thông số cần thiết trong khi khóa ngắt `irq_lock()`, sau đó mở khóa ngắt
 *      ngay lập tức rồi mới định dạng chuỗi JSON. Cách thiết kế này đảm bảo
 *      hàm `snprintf` không bao giờ làm trễ ngắt phát xung động cơ bước!
 * ============================================================================
 */

#ifndef SCARA_CART_TELEMETRY_H
#define SCARA_CART_TELEMETRY_H

#include "cart_core.h"
#include <stddef.h>

/*
 * [EN] Buffer capacities for JSON status broadcasts and command ACK replies.
 * [VI] Kích thước bộ đệm tối đa cho bản tin JSON trạng thái (`status`) và phản hồi (`ack`).
 */
#define CT_STATUS_CAPACITY 1536U
#define CT_ACK_CAPACITY    192U

/*
 * [EN] Compact telemetry snapshot copied under IRQ lock.
 *      Does NOT copy the 32-element `path_queue` FIFO to keep IRQ lock time minimal.
 * [VI] Bản chụp trạng thái gọn nhẹ được sao chép trong vùng khóa ngắt.
 *      Không sao chép mảng hàng đợi `path_queue` để thời gian khóa ngắt là ngắn nhất.
 */
struct ct_status {
    /*
     * [EN] Per-axis home-relative positions `p[3]`, goals `g[3]`, measured spans `range[3]`,
     *      Q10 scale factors `f[3]`, forward/reverse sweep counts `n1[3]`/`n2[3]`, and total pulses `total[3]`.
     * [VI] Tọa độ tương đối so với gốc `p[3]`, đích đến `g[3]`, hành trình đo được `range[3]`,
     *      hệ số tỷ lệ Q10 `f[3]`, số xung quét thuận/ngược `n1[3]`/`n2[3]`, và tổng xung `total[3]`.
     */
    int32_t p[CT_AXES], g[CT_AXES];
    uint32_t range[CT_AXES], f[CT_AXES], n1[CT_AXES], n2[CT_AXES];
    uint64_t total[CT_AXES];

    /*
     * [EN] Per-axis calibration phases/errors and packed status bitmasks (`mask`, `ready`, `hold`, `pol`, `conflicts`).
     * [VI] Pha dò gốc, mã lỗi hiệu chuẩn và các mặt nạ bit trạng thái đóng gói của 3 trục.
     */
    unsigned phases[CT_AXES], errors[CT_AXES], failed[CT_AXES];
    unsigned mask, ready, hold, pol, conflicts;

    /*
     * [EN] Session/job identifiers, reference epoch, DDA tick progress, and path FIFO counters (`path[0..3]`).
     * [VI] Mã phiên `session`, công việc `job`, số lần về gốc `epoch`, tiến độ `tick`/`ticks` và bộ đếm `path[0..3]`.
     */
    uint32_t session, job, epoch, tick, ticks, late;
    unsigned path[4];
    uint32_t starved, recovered, input_at, brief[CT_AXES], false_hits[CT_AXES], idle_ignored;

    /*
     * [EN] Switch diagnostic fields, motion directions, belt coupling telemetry, and hardware fault codes.
     * [VI] Thông tin chẩn đoán công tắc, chiều quay, thông số truyền động đai và mã lỗi phần cứng.
     */
    int input_axis;
    unsigned input_kind, input_bits, raw, io_errors;
    bool input_wait, gate_wait;
    unsigned gate_axis;
    int direction[CT_AXES];
    unsigned dir_levels, mode, selected, pps;
    int32_t coupling_ppm, beta_q, beta_measured, probe_da, probe_db, span_md[2];
    uint32_t comp_pulses[2];
    bool coupling_ready, busy, referenced, fault;
    enum ct_stage stage;
    const char *reason;
    int motor_error, timer_error;
};

/*
 * [EN] Capture a consistent snapshot of `control` into `status` (call with IRQs locked).
 * [VI] Sao chép trạng thái từ `control` sang `status` (gọi khi đã khóa ngắt).
 */
void ct_status_capture(
    struct ct_status *status,
    const struct ct_state *control,
    bool timer_running,
    uint32_t timer_late,
    uint32_t now
);

/*
 * [EN] Format captured `status` into a newline-terminated JSON string (call with IRQs unlocked).
 * [VI] Định dạng cấu trúc `status` thành chuỗi JSON kết thúc bằng '\n' (gọi sau khi đã mở khóa ngắt).
 */
int ct_status_format(
    char *output,
    size_t capacity,
    const struct ct_status *status,
    uint32_t now
);

/*
 * [EN] Format a command acknowledgment (`{"type":"ack",...}`) into a JSON line.
 * [VI] Định dạng bản tin phản hồi lệnh (`{"type":"ack",...}`) thành chuỗi JSON.
 */
int ct_ack_format(
    char *output,
    size_t capacity,
    uint32_t session,
    uint32_t job,
    const char *operation,
    bool ok,
    const char *reason
);

#endif /* SCARA_CART_TELEMETRY_H */
