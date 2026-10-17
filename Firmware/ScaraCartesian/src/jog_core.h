/*
 * ============================================================================
 * FILE: jog_core.h
 * ============================================================================
 * [EN] Single-Axis Limit-Switch Debouncing & Guarded Jog State Machine
 *      Manages a pair of Normally-Closed (NC) limit switches per axis and
 *      provides a guarded pulse/velocity state machine used both for manual
 *      jogging and for individual axis calibration legs.
 *
 * [VI] Bộ Chống Dội Công tắc Hành trình & Máy Trạng thái Chạy Jog Đơn Trục
 *      Quản lý cặp công tắc hành trình thường đóng (NC) ở hai đầu mỗi trục và
 *      cung cấp máy trạng thái phát xung có bảo vệ an toàn, dùng chung cho cả
 *      chế độ chạy Jog thủ công và từng chặng dò công tắc khi Homing/Calibration.
 *
 * [HARDWARE NOTE / LƯU Ý PHẦN CỨNG]:
 * - Switches are wired NC (Normally Closed) to GND with internal pull-up:
 *   * GPIO LOW  (0): Switch is closed (normal, NOT triggered).
 *   * GPIO HIGH (1): Switch is pressed OR wire is broken (LIMIT TRIGGERED!).
 * - Công tắc hành trình đấu kiểu Thường Đóng (NC) nối GND kèm trở kéo lên:
 *   * Mức Thấp (0): Công tắc đóng bình thường (chưa chạm cữ).
 *   * Mức Cao  (1): Công tắc bị nhấn HOẶC đứt dây tín hiệu (KÍCH HOẠT DỪNG!).
 * ============================================================================
 */

#ifndef SCARA_JOG_CORE_H
#define SCARA_JOG_CORE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * [EN] Timing and rate constants:
 *      - ZJ_RELEASE_MS : Switch must stay LOW for 20 ms to clear triggered state.
 *      - ZJ_CONFLICT_MS: Both switches HIGH for >= 20 ms latches a permanent conflict.
 *      - ZJ_LEASE_MS   : Host heartbeat watchdog timeout (350 ms).
 *      - ZJ_SENSOR_MS  : Maximum allowed age of switch GPIO sample (30 ms).
 *      - ZJ_MAX_RATE   : Maximum pulse rate during calibration/jog (6400 pulses/s).
 *      - ZJ_MIN_RATE   : Minimum start/fine approach rate (50 pulses/s).
 *      - ZJ_SETUP_MS   : Direction-pin setup delay before ramping (50 ms).
 *      - ZJ_RAMP_MS    : Linear velocity ramp duration (2000 ms).
 *
 * [VI] Các hằng số thời gian và tốc độ xung:
 *      - ZJ_RELEASE_MS : Công tắc phải ổn định mức THẤP trong 20 ms mới được coi là đã nhả.
 *      - ZJ_CONFLICT_MS: Cả 2 công tắc cùng mức CAO >= 20 ms sẽ khóa lỗi xung đột phần cứng.
 *      - ZJ_LEASE_MS   : Thời gian chờ tối đa giữa 2 gói tin KEEP từ máy tính (350 ms).
 *      - ZJ_SENSOR_MS  : Tuổi mẫu đọc GPIO công tắc tối đa cho phép (30 ms).
 *      - ZJ_MAX_RATE   : Tốc độ phát xung tối đa (6400 xung/giây).
 *      - ZJ_MIN_RATE   : Tốc độ khởi đầu / tốc độ dò tinh vào công tắc (50 xung/giây).
 *      - ZJ_SETUP_MS   : Thời gian thiết lập chân DIR trước khi tăng tốc (50 ms).
 *      - ZJ_RAMP_MS    : Thời gian tăng tốc tuyến tính (2000 ms).
 */
#define ZJ_RELEASE_MS   20U
#define ZJ_CONFLICT_MS  20U
#define ZJ_LEASE_MS     350U
#define ZJ_SENSOR_MS    30U
#define ZJ_MAX_RATE     6400U
#define ZJ_MIN_RATE     50U
#define ZJ_SETUP_MS     50U
#define ZJ_RAMP_MS      2000U

/*
 * [EN] Stop / fault reason codes for a single-axis motion or switch monitor.
 * [VI] Mã nguyên nhân dừng hoặc lỗi của bộ giám sát công tắc và chuyển động trục.
 */
enum zj_reason {
    ZJ_IDLE,          /* [EN] Axis idle / [VI] Trục đang nghỉ */
    ZJ_STOP,          /* [EN] Stopped by host command / [VI] Dừng theo lệnh STOP */
    ZJ_TOP,           /* [EN] Hit positive/top limit switch / [VI] Chạm công tắc giới hạn trên/dương */
    ZJ_TIMEOUT,       /* [EN] Host heartbeat lease expired / [VI] Mất gói tin duy trì kết nối KEEP */
    ZJ_SENSOR,        /* [EN] GPIO read error on switch pin / [VI] Lỗi phần cứng khi đọc chân GPIO công tắc */
    ZJ_STALE_SENSOR,  /* [EN] Switch sample older than 30 ms / [VI] Quá 30 ms chưa cập nhật trạng thái công tắc */
    ZJ_NOT_READY,     /* [EN] Switch never observed stable since boot / [VI] Công tắc chưa sẵn sàng từ lúc bật máy */
    ZJ_INVALID,       /* [EN] Invalid command or parameter / [VI] Lệnh hoặc tham số không hợp lệ */
    ZJ_DONE,          /* [EN] Target pulse count completed / [VI] Đã phát đủ số xung mục tiêu */
    ZJ_TIMER,         /* [EN] Hardware timer fault / [VI] Lỗi bộ định thời phần cứng */
    ZJ_USB,           /* [EN] USB disconnected / [VI] Mất kết nối cáp USB */
    ZJ_BOTTOM,        /* [EN] Hit negative/bottom limit switch / [VI] Chạm công tắc giới hạn dưới/âm */
    ZJ_CONFLICT,      /* [EN] Both end-switches triggered simultaneously / [VI] Cả 2 công tắc cùng báo chạm (xung đột/đứt dây) */
    ZJ_GPIO,          /* [EN] Motor STEP/DIR/ENA pin fault / [VI] Lỗi chân điều khiển động cơ */
    ZJ_BUDGET,        /* [EN] Calibration leg exceeded time budget / [VI] Quá thời gian cho phép của một chặng dò */
    ZJ_RELEASED,      /* [EN] Motor driver disabled (HOLD=0) / [VI] Động cơ đang bị nhả phanh (HOLD=0) */
    ZJ_CAL            /* [EN] Calibration sequence error / [VI] Lỗi quy trình hiệu chuẩn (Calibration) */
};

/*
 * [EN] Result of a single-axis jog or calibration transition call.
 * [VI] Kết quả trả về khi gọi lệnh điều khiển trục hoặc chuyển pha hiệu chuẩn.
 */
enum zj_command_result {
    ZJ_OK,
    ZJ_STARTED,
    ZJ_REJECTED,
    ZJ_HOLD_CHANGE,
    ZJ_RESUMED
};

/*
 * [EN] Per-axis switch filter and motion state structure.
 *      Callers must serialize access (IRQ lock on MCU target, sequential in host tests).
 * [VI] Cấu trúc lưu trạng thái bộ lọc công tắc hành trình và bộ đếm xung của 1 trục.
 *      Khi truy cập từ vòng lặp chính và ngắt Timer cần khóa ngắt (`irq_lock`).
 */
struct zj_state {
    bool positive_high;                              /* [EN] Polarity mapping / [VI] Chiều dương ứng với công tắc top hay bottom */
    bool ready, top, ready_bottom, bottom, conflict; /* [EN] Debounced switch flags / [VI] Cờ trạng thái công tắc sau chống dội */
    bool seen, closing, closing_bottom, both_pending;
    int raw_pos, raw_neg;                            /* [EN] Latest raw GPIO levels (0 or 1) / [VI] Mức logic GPIO tức thời (0 hoặc 1) */
    int error, error_top, error_bottom;              /* [EN] Sticky GPIO driver error codes / [VI] Mã lỗi đọc phần cứng GPIO */
    int direction;                                   /* [EN] Current motion dir: -1, 0, +1 / [VI] Chiều quay hiện tại: -1, 0, +1 */
    uint32_t sample_ms, closed_ms, closed_bottom_ms, job_budget_ms;
    uint32_t opened_ms, opened_bottom_ms;
    uint32_t both_ms, brief_both;
    uint32_t session, last_id, job, target, move;
    uint32_t rate, start_ms, lease_ms;
    uint64_t total;                                  /* [EN] Cumulative pulses emitted / [VI] Tổng số xung đã phát từ lúc bật máy */
    int64_t position;                                /* [EN] Signed step coordinate / [VI] Tọa độ bước có dấu hiện tại */
    enum zj_reason reason;                           /* [EN] Latest stop/status reason / [VI] Nguyên nhân dừng gần nhất */
};

void zj_init(struct zj_state *s);
void zj_init_joint(struct zj_state *s);
void zj_stop(struct zj_state *s, enum zj_reason reason);

/*
 * [EN] Update switch debounce state from a simultaneous pair of GPIO reads.
 *      Triggering (HIGH) is immediate; releasing (LOW) requires 20 ms stability.
 * [VI] Cập nhật bộ lọc chống dội từ kết quả đọc đồng thời 2 công tắc đầu trục.
 *      Khi chạm công tắc (mức 1) sẽ dừng ngay lập tức; khi nhả (mức 0) cần ổn định đủ 20 ms.
 */
void zj_switches(struct zj_state *s, int positive, int negative, uint32_t now);

bool zj_ready(const struct zj_state *s);
bool zj_safe(struct zj_state *s, uint32_t now);
void zj_count_pulse(struct zj_state *s);
uint32_t zj_period_us(const struct zj_state *s, uint32_t now);

/*
 * [EN] Guarded motion start for manual jog or an internal calibration leg.
 * [VI] Khởi động phát xung có kiểm tra an toàn cho lệnh Jog hoặc chặng dò gốc.
 */
enum zj_command_result zj_begin(
    struct zj_state *s,
    int direction,
    uint32_t rate,
    uint32_t target,
    uint32_t now
);

enum zj_command_result zj_command(struct zj_state *s, const char *line, uint32_t now);
const char *zj_reason_name(enum zj_reason reason);

#endif /* SCARA_JOG_CORE_H */
