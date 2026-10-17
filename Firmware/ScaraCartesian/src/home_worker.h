/*
 * ============================================================================
 * FILE: home_worker.h
 * ============================================================================
 * [EN] Single-Axis End-to-End Homing & Span Calibration State Machine
 *      Drives an axis across its full mechanical travel between negative and
 *      positive limit switches (`SEEK_N -> CLEAR_N -> LATCH_N -> SWEEP_P ->
 *      CLEAR_P -> LATCH_P -> SWEEP_N -> CLEAR_N2 -> LATCH_N2 -> DONE`) to
 *      measure the exact pulse span (`n1`, `n2`), verify repeatability within
 *      1%, and establish a repeatable home reference anchor.
 *
 * [VI] Máy Trạng thái Dò Gốc (Homing) & Đo Hành trình Tự động Đơn Trục
 *      Điều khiển trục chạy quét giữa hai công tắc hành trình âm và dương theo
 *      chu trình 9 bước (`SEEK_N -> CLEAR_N -> LATCH_N -> SWEEP_P -> CLEAR_P ->
 *      LATCH_P -> SWEEP_N -> CLEAR_N2 -> LATCH_N2 -> DONE`) để đo chính xác
 *      tổng số xung toàn hành trình (`n1` chiều đi, `n2` chiều về), kiểm tra
 *      độ lặp lại cơ khí (sai lệch <= 1%) và chốt điểm gốc tọa độ.
 * ============================================================================
 */

#ifndef SCARA_HOME_WORKER_H
#define SCARA_HOME_WORKER_H

#include "jog_core.h"

/*
 * [EN] Timeouts for a single calibration phase (10 min) and full calibration (30 min),
 *      plus switch contact confirmation window (200 ms).
 * [VI] Thời gian tối đa cho 1 pha dò (10 phút), toàn bộ quy trình (30 phút),
 *      và cửa sổ thời gian xác nhận chạm công tắc ổn định (200 ms).
 */
#define HMC_PHASE_MS   600000U
#define HMC_TOTAL_MS   1800000U
#define HMC_CONFIRM_MS 200U

/*
 * [EN] Calibration state machine phases:
 *      1. HMC_SEEK_N  : Fast seek toward negative switch.
 *      2. HMC_CLEAR_N : Back off until negative switch closes + 16 extra pulses.
 *      3. HMC_LATCH_N : Slow fine approach to latch negative anchor (`negative_anchor`).
 *      4. HMC_SWEEP_P : Fast sweep across full travel to positive switch.
 *      5. HMC_CLEAR_P : Back off until positive switch closes + 16 extra pulses.
 *      6. HMC_LATCH_P : Slow fine approach to latch positive anchor (`n1` measured).
 *      7. HMC_SWEEP_N : Fast sweep back to negative switch.
 *      8. HMC_CLEAR_N2: Back off from negative switch a second time.
 *      9. HMC_LATCH_N2: Slow fine approach to negative switch (`n2` measured & compared with `n1`).
 *
 * [VI] Các pha của máy trạng thái dò gốc và hiệu chuẩn hành trình:
 *      1. HMC_SEEK_N  : Chạy nhanh về phía công tắc âm.
 *      2. HMC_CLEAR_N : Lùi ra cho tới khi công tắc âm đóng lại + đi thêm 16 xung an toàn.
 *      3. HMC_LATCH_N : Chạy chậm (fine_rate) chạm lại công tắc âm để chốt mốc `negative_anchor`.
 *      4. HMC_SWEEP_P : Quét nhanh sang đầu bên kia tới công tắc dương.
 *      5. HMC_CLEAR_P : Lùi ra khỏi công tắc dương + đi thêm 16 xung.
 *      6. HMC_LATCH_P : Chạy chậm chạm lại công tắc dương để đo hành trình lượt đi `n1`.
 *      7. HMC_SWEEP_N : Quét nhanh quay về lại công tắc âm.
 *      8. HMC_CLEAR_N2: Lùi ra khỏi công tắc âm lần 2.
 *      9. HMC_LATCH_N2: Chạy chậm chạm công tắc âm lần 2 để đo `n2`, đối chiếu với `n1` và hoàn tất.
 */
enum hc_phase {
    HMC_IDLE,
    HMC_SEEK_N,
    HMC_CLEAR_N,
    HMC_LATCH_N,
    HMC_SWEEP_P,
    HMC_CLEAR_P,
    HMC_LATCH_P,
    HMC_SWEEP_N,
    HMC_CLEAR_N2,
    HMC_LATCH_N2,
    HMC_DONE,
    HMC_FAILED
};

/*
 * [EN] Diagnostic error codes if homing/calibration fails.
 * [VI] Mã lỗi chẩn đoán khi quá trình Homing/Calibration thất bại.
 */
enum hc_error {
    HMC_NONE,        /* [EN] No error / [VI] Không có lỗi */
    HMC_CANCELLED,   /* [EN] Aborted by STOP or external fault / [VI] Bị hủy bởi lệnh STOP hoặc lỗi khác */
    HMC_LINK,        /* [EN] Host heartbeat lease expired / [VI] Mất kết nối hoặc quá hạn gói tin KEEP */
    HMC_IO,          /* [EN] Motor/timer hardware I/O fault / [VI] Lỗi phần cứng chân GPIO hoặc Timer */
    HMC_INPUT,       /* [EN] Limit switch conflict or stale sample / [VI] Xung đột hoặc lỗi tín hiệu công tắc */
    HMC_BOUNDS,      /* [EN] Scan limit or time budget exceeded / [VI] Vượt quá số xung quét hoặc thời gian tối đa */
    HMC_UNSTABLE,    /* [EN] Switch contact failed to stabilize in 200 ms / [VI] Tiếp điểm công tắc chập chờn quá 200 ms */
    HMC_REPEAT,      /* [EN] Forward `n1` and reverse `n2` differ by > 1% / [VI] Sai lệch hành trình đi/về (`n1` vs `n2`) > 1% */
    HMC_BAD_COMMAND  /* [EN] Invalid state machine transition / [VI] Lỗi trạng thái lệnh nội bộ */
};

/*
 * [EN] Per-axis calibration worker state.
 * [VI] Cấu trúc dữ liệu lưu trạng thái dò gốc và kết quả đo hành trình của 1 trục.
 */
struct hc_state {
    enum hc_phase phase;
    enum hc_phase failed_phase;
    enum hc_error error;

    /*
     * [EN] Last successful calibrated model (retained in RAM if a later attempt fails).
     * [VI] Thông số hiệu chuẩn thành công gần nhất (được giữ lại trong RAM).
     */
    uint32_t pulses, angle_mdeg, micro, epoch;
    int64_t origin;
    bool reference;

    /*
     * [EN] Working variables for the active calibration attempt.
     * [VI] Các biến làm việc của lần hiệu chuẩn đang chạy.
     */
    uint32_t n1, n2, run_angle, run_micro, rate, fine_rate, max_scan, backoff;
    uint32_t false_hits;
    uint32_t start_ms, lease_ms, settle_ms, hit_ms;
    uint64_t start_total;
    int64_t negative_anchor, positive_anchor;
    bool settling, extra, arrived, quick;
};

/*
 * [EN] Return true while the calibration state machine is actively running phases 1..9.
 * [VI] Trả về `true` khi máy trạng thái đang chạy trong các pha từ `HMC_SEEK_N` đến `HMC_LATCH_N2`.
 */
bool hc_active(const struct hc_state *c);

/*
 * [EN] Cancel an active calibration and latch `failed_phase` and `error`.
 * [VI] Hủy quá trình dò gốc đang chạy và ghi lại pha bị lỗi `failed_phase` cùng mã lỗi `error`.
 */
void hc_cancel(struct hc_state *c, struct zj_state *s, enum zj_reason reason, enum hc_error error);

/*
 * [EN] Validate parameters and start the calibration sequence at `HMC_SEEK_N`.
 * [VI] Kiểm tra các tham số đầu vào và bắt đầu chu trình dò gốc từ pha `HMC_SEEK_N`.
 */
enum zj_command_result hc_start(
    struct hc_state *c,
    struct zj_state *s,
    uint32_t angle,
    uint32_t rate,
    uint32_t max_scan,
    uint32_t backoff,
    uint32_t micro,
    uint32_t now
);

/*
 * [EN] Advance the calibration state machine on each main-loop service tick.
 * [VI] Cập nhật tiến trình máy trạng thái dò gốc ở mỗi vòng lặp dịch vụ nền.
 */
enum zj_command_result hc_tick(struct hc_state *c, struct zj_state *s, uint32_t now);

#endif /* SCARA_HOME_WORKER_H */
