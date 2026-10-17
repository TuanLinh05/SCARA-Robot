/*
 * ============================================================================
 * FILE: jog_core.c
 * ============================================================================
 * [EN] Implementation of limit-switch debouncing, safety guards, and single-axis
 *      pulse ramping for SCARA axes.
 *
 * [VI] Triển khai thuật toán chống dội công tắc hành trình, khóa an toàn và
 *      điều khiển tăng tốc xung đơn trục cho robot SCARA.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - If an axis refuses to move with reason `ZJ_CONFLICT` ("both_latched" /
 *   "limits_conflict"), check whether the 2 limit switch connectors are unplugged!
 *   Because the switches are Normally-Closed (NC) with pull-up resistors, an
 *   unplugged connector reads `positive == 1 && negative == 1`.
 * - Nếu trục báo lỗi `ZJ_CONFLICT` ("both_latched") không chịu chạy, hãy kiểm tra
 *   xem giắc cắm công tắc hành trình có bị lỏng/tuột không! Vì công tắc đấu kiểu
 *   Thường Đóng (NC) có trở kéo lên, nếu tuột dây cả 2 chân đều đọc về mức 1.
 * ============================================================================
 */

#include "jog_core.h"
#include "command_text.h"
#include <limits.h>
#include <string.h>

/*
 * [EN] Initialize switch state conservatively: assume switches are triggered (HIGH)
 *      until real GPIO samples prove they are closed (LOW) for `ZJ_RELEASE_MS`.
 * [VI] Khởi tạo trạng thái ban đầu an toàn: giả định cả 2 công tắc đang bị kích hoạt (mức 1)
 *      cho tới khi đọc được tín hiệu mức 0 ổn định liên tục trong `ZJ_RELEASE_MS` (20 ms).
 */
void zj_init(struct zj_state *s)
{
    memset(s, 0, sizeof(*s));
    s->positive_high = true;
    s->top = true;
    s->bottom = true;
    s->raw_pos = 1;
    s->raw_neg = 1;
}

/*
 * [EN] Initialize joint switch monitor (alias of `zj_init` for Cartesian axes).
 * [VI] Khởi tạo bộ giám sát công tắc cho khớp (tương đương `zj_init`).
 */
void zj_init_joint(struct zj_state *s)
{
    zj_init(s);
}

/*
 * [EN] Immediately stop pulse motion and record the stop reason.
 * [VI] Dừng phát xung ngay lập tức và lưu lại mã nguyên nhân dừng.
 */
void zj_stop(struct zj_state *s, enum zj_reason reason)
{
    s->direction = 0;
    s->reason = reason;
}

/*
 * [EN] Return true only after both top and bottom switches have completed
 *      their initial power-on debounce check.
 * [VI] Trả về `true` khi cả 2 công tắc đầu trục đã được đọc và xác nhận sẵn sàng.
 */
bool zj_ready(const struct zj_state *s)
{
    return s->ready && s->ready_bottom;
}

/*
 * [EN] Debounce helper for a single switch input (`lower == false` -> positive/top,
 *      `lower == true` -> negative/bottom):
 *      - Any invalid GPIO driver return (`level != 0 && level != 1`) latches a sensor error.
 *      - `level == 1` (switch open/pressed): immediately sets `*active = true`.
 *      - `level == 0` (switch closed/normal): requires `ZJ_RELEASE_MS` (20 ms) of
 *        uninterrupted LOW before clearing `*active = false`.
 *
 * [VI] Hàm lọc chống dội cho từng công tắc (`lower == false` là đầu dương, `true` là đầu âm):
 *      - Nếu hàm đọc GPIO trả về mã lỗi (`level != 0 && level != 1`), khóa lỗi cảm biến ngay.
 *      - Nếu `level == 1` (chạm công tắc): bật cờ `*active = true` ngay lập tức để dừng khẩn.
 *      - Nếu `level == 0` (nhả công tắc): chờ đủ `ZJ_RELEASE_MS` (20 ms) liên tục ở mức 0
 *        mới xóa cờ `*active = false` để tránh nhiễu rung tiếp điểm cơ khí.
 */
static void sample_limit(struct zj_state *s, bool lower, int level, uint32_t now)
{
    bool *ready = lower ? &s->ready_bottom : &s->ready;
    bool *active = lower ? &s->bottom : &s->top;
    int *sensor_error = lower ? &s->error_bottom : &s->error_top;
    bool *closing = lower ? &s->closing_bottom : &s->closing;
    uint32_t *closed_ms = lower ? &s->closed_bottom_ms : &s->closed_ms;

    /*
     * [EN] Check for hardware GPIO read error (negative errno).
     * [VI] Kiểm tra lỗi phần cứng khi đọc thanh ghi GPIO.
     */
    if (level != 0 && level != 1) {
        *closing = false;
        *active = true;
        if (*sensor_error == 0) {
            *sensor_error = (level < 0) ? level : -1;
        }
        if (s->error == 0) {
            s->error = *sensor_error;
        }
        zj_stop(s, ZJ_SENSOR);
        return;
    }

    if (level != 0) {
        /*
         * [EN] Switch triggered (HIGH): latch active immediately.
         * [VI] Công tắc mở (mức CAO): báo chạm cữ ngay lập tức.
         */
        *active = true;
        *ready = true;
        *closing = false;
    } else {
        /*
         * [EN] Switch closed (LOW): start or continue the 20 ms release timer.
         * [VI] Công tắc đóng (mức THẤP): đếm thời gian ổn định 20 ms trước khi báo đã nhả.
         */
        if (!*closing) {
            *closing = true;
            *closed_ms = now;
        }
        if (now - *closed_ms >= ZJ_RELEASE_MS) {
            *active = false;
            *ready = true;
        }
    }
}

/*
 * [EN] Process a simultaneous reading of the positive and negative limit switches.
 * [VI] Xử lý mẫu đọc đồng thời của công tắc giới hạn dương (`positive`) và âm (`negative`).
 */
void zj_switches(struct zj_state *s, int positive, int negative, uint32_t now)
{
    bool continuous = s->seen && (now - s->sample_ms <= ZJ_SENSOR_MS);

    /*
     * [EN] Record timestamp when each switch transitions to HIGH (open).
     * [VI] Ghi lại thời điểm bắt đầu chuyển sang mức CAO của từng công tắc.
     */
    if (positive == 1 && (!continuous || s->raw_pos != 1)) {
        s->opened_ms = now;
    }
    if (negative == 1 && (!continuous || s->raw_neg != 1)) {
        s->opened_bottom_ms = now;
    }

    /*
     * [EN] A sampling gap (> 30 ms) resets pending release/conflict timers so
     *      missed samples never count as continuous stability.
     * [VI] Nếu khoảng cách giữa 2 lần đọc quá 30 ms, đặt lại bộ đếm chống dội để
     *      đảm bảo tính liên tục của tín hiệu.
     */
    if (s->seen && (now - s->sample_ms > ZJ_SENSOR_MS)) {
        s->closing = false;
        s->closing_bottom = false;
        s->both_pending = false;
    }

    s->seen = true;
    s->sample_ms = now;
    s->raw_pos = (positive == 0) ? 0 : 1;
    s->raw_neg = (negative == 0) ? 0 : 1;

    sample_limit(s, false, positive, now);
    sample_limit(s, true, negative, now);

    if (s->error != 0) {
        s->both_pending = false;
        zj_stop(s, ZJ_SENSOR);
        return;
    }

    /*
     * [EN] Physically, an axis cannot be at both ends at once. If both switches
     *      read HIGH, stop motion immediately; if sustained >= 20 ms, latch conflict.
     * [VI] Về mặt vật lý, 1 trục không thể chạm cả 2 đầu cùng lúc. Nếu cả 2 cùng mức 1,
     *      dừng trục ngay; nếu kéo dài >= 20 ms thì khóa lỗi `conflict` (thường do đứt dây).
     */
    if (positive == 1 && negative == 1) {
        if (!s->both_pending) {
            s->both_pending = true;
            s->both_ms = now;
        }
        if (now - s->both_ms >= ZJ_CONFLICT_MS) {
            s->conflict = true;
        }
        zj_stop(s, ZJ_CONFLICT);
        return;
    }

    if (s->both_pending && !s->conflict && s->brief_both < UINT32_MAX) {
        s->brief_both++;
    }
    s->both_pending = false;

    if (s->conflict) {
        zj_stop(s, ZJ_CONFLICT);
        return;
    }

    /*
     * [EN] Stop motion if moving into an active limit switch.
     * [VI] Dừng chuyển động nếu đang đi về phía công tắc hành trình đang báo chạm.
     */
    if (s->direction != 0) {
        bool moving_to_top = ((s->direction > 0) == s->positive_high);
        bool hit_limit = moving_to_top ? s->top : s->bottom;
        if (hit_limit) {
            zj_stop(s, moving_to_top ? ZJ_TOP : ZJ_BOTTOM);
        }
    }
}

/*
 * [EN] Check whether sensors allow motion in `direction` at timestamp `now`.
 * [VI] Kiểm tra toàn bộ điều kiện an toàn của công tắc trước và trong khi chạy theo chiều `direction`.
 */
static enum zj_reason sensor_guard(const struct zj_state *s, int direction, uint32_t now)
{
    if (s->error != 0) {
        return ZJ_SENSOR;
    }
    if (s->conflict) {
        return ZJ_CONFLICT;
    }
    if (!zj_ready(s)) {
        return ZJ_NOT_READY;
    }
    if (now - s->sample_ms > ZJ_SENSOR_MS) {
        return ZJ_STALE_SENSOR;
    }
    if (s->raw_pos && s->raw_neg) {
        return ZJ_CONFLICT;
    }

    /*
     * [EN] Endpoints are tied to physical DIR polarity, preventing motion further into a switch.
     * [VI] Chặn di chuyển tiếp vào phía công tắc đang bị nhấn (cho phép lùi ra theo chiều ngược lại).
     */
    bool physical_positive = ((direction > 0) == s->positive_high);
    if (physical_positive && (s->top || s->raw_pos)) {
        return ZJ_TOP;
    }
    if (!physical_positive && (s->bottom || s->raw_neg)) {
        return ZJ_BOTTOM;
    }
    return ZJ_IDLE;
}

/*
 * [EN] Verify that an active single-axis motion remains safe to emit the next pulse.
 * [VI] Kiểm tra an toàn trước mỗi xung: công tắc, thời gian chờ KEEP, ngân sách thời gian và tràn biến đếm.
 */
bool zj_safe(struct zj_state *s, uint32_t now)
{
    if (s->direction == 0) {
        return false;
    }

    enum zj_reason reason = sensor_guard(s, s->direction, now);
    if (reason != ZJ_IDLE) {
        zj_stop(s, reason);
        return false;
    }

    if (now - s->lease_ms >= ZJ_LEASE_MS) {
        zj_stop(s, ZJ_TIMEOUT);
        return false;
    }

    /*
     * [EN] Bounded internal calibration legs enforce `job_budget_ms`; manual jog leaves it 0.
     * [VI] Giới hạn thời gian tối đa cho một chặng dò công tắc khi hiệu chuẩn.
     */
    if (s->job_budget_ms != 0U && (now - s->start_ms >= s->job_budget_ms)) {
        zj_stop(s, ZJ_BUDGET);
        return false;
    }

    if (s->total == UINT64_MAX || s->move == UINT32_MAX ||
        (s->direction > 0 && s->position == INT64_MAX) ||
        (s->direction < 0 && s->position == INT64_MIN)) {
        zj_stop(s, ZJ_INVALID);
        return false;
    }

    return true;
}

/*
 * [EN] Advance position and pulse counters after emitting one rising STEP edge.
 * [VI] Cập nhật tọa độ bước và bộ đếm xung sau mỗi sườn lên của chân STEP.
 */
void zj_count_pulse(struct zj_state *s)
{
    s->position += s->direction;
    s->total++;
    s->move++;
    if (s->target != 0U && s->move >= s->target) {
        zj_stop(s, ZJ_DONE);
    }
}

/*
 * [EN] Compute pulse period in microseconds with a 50 ms setup pause followed by
 *      a 2000 ms linear ramp from `ZJ_MIN_RATE` (50 pps) to `s->rate`.
 * [VI] Tính chu kỳ xung (micro-giây): giữ tốc độ 50 pps trong 50 ms đầu, sau đó
 *      tăng tốc tuyến tính trong 2000 ms lên tới tốc độ mục tiêu `s->rate`.
 */
uint32_t zj_period_us(const struct zj_state *s, uint32_t now)
{
    uint32_t initial = ZJ_MIN_RATE;
    uint32_t elapsed = now - s->start_ms;
    elapsed = (elapsed > ZJ_SETUP_MS) ? (elapsed - ZJ_SETUP_MS) : 0U;
    if (elapsed > ZJ_RAMP_MS) {
        elapsed = ZJ_RAMP_MS;
    }

    uint32_t rate = initial + (s->rate - initial) * elapsed / ZJ_RAMP_MS;
    if (rate == 0U) {
        rate = ZJ_MIN_RATE;
    }
    return (1000000U + rate - 1U) / rate;
}

/*
 * [EN] Start a single-axis move of `target` pulses in `direction` (-1 or +1) at `rate` pps.
 * [VI] Bắt đầu phát `target` xung theo chiều `direction` (-1 hoặc +1) với tốc độ `rate` xung/giây.
 */
enum zj_command_result zj_begin(
    struct zj_state *s,
    int direction,
    uint32_t rate,
    uint32_t target,
    uint32_t now
)
{
    if (s->direction != 0 || (direction != 1 && direction != -1) ||
        rate < ZJ_MIN_RATE || rate > ZJ_MAX_RATE || target > 1000000U) {
        zj_stop(s, ZJ_INVALID);
        return ZJ_REJECTED;
    }

    s->target = target;
    s->rate = rate;
    s->move = 0;

    enum zj_reason reason = sensor_guard(s, direction, now);
    if (reason != ZJ_IDLE) {
        zj_stop(s, reason);
        return ZJ_REJECTED;
    }

    s->direction = direction;
    s->start_ms = now;
    s->lease_ms = now;
    s->reason = ZJ_IDLE;
    return ZJ_STARTED;
}

/*
 * [EN] Parse and execute standalone single-axis jog commands (STOP, STATUS, HELLO, KEEP, JOG).
 * [VI] Giải mã và thực thi các lệnh điều khiển Jog đơn trục (STOP, STATUS, HELLO, KEEP, JOG).
 */
enum zj_command_result zj_command(struct zj_state *s, const char *line, uint32_t now)
{
    char buffer[128];
    char *words[7];
    unsigned count = 0;

    if (!command_split(line, buffer, sizeof(buffer), words, 7, &count, true)) {
        goto invalid;
    }

    if (count == 1U && strcmp(words[0], "STOP") == 0) {
        zj_stop(s, ZJ_STOP);
        return ZJ_OK;
    }
    if (count == 1U && strcmp(words[0], "STATUS") == 0) {
        return ZJ_OK;
    }

    uint32_t session = 0;
    uint32_t id = 0;
    uint32_t rate = 0;
    uint32_t target = 0;

    if (count == 2U && strcmp(words[0], "HELLO") == 0 &&
        command_number_u32(words[1], &session) && session != 0U) {
        zj_stop(s, ZJ_STOP);
        s->session = session;
        s->last_id = 0;
        s->job = 0;
        return ZJ_OK;
    }

    if (count == 3U && strcmp(words[0], "KEEP") == 0) {
        bool valid_keep_args = command_number_u32(words[1], &session) &&
                               command_number_u32(words[2], &id);
        bool active_keep_ok = valid_keep_args &&
                              session == s->session && session != 0U && id == s->job &&
                              s->direction != 0 && (now - s->lease_ms < ZJ_LEASE_MS);
        if (active_keep_ok) {
            s->lease_ms = now;
            return ZJ_OK;
        }
        /*
         * [EN] Late KEEP cannot revive motion after lease timeout.
         * [VI] Bản tin KEEP đến trễ sau khi hết hạn lease sẽ bị từ chối, không tự chạy lại.
         */
        return ZJ_REJECTED;
    }

    if (count == 6U && strcmp(words[0], "JOG") == 0 &&
        command_number_u32(words[1], &session) &&
        command_number_u32(words[2], &id) &&
        command_number_u32(words[4], &rate) &&
        command_number_u32(words[5], &target) &&
        session != 0U && session == s->session && id > s->last_id && s->direction == 0 &&
        (strcmp(words[3], "U") == 0 || strcmp(words[3], "D") == 0) &&
        rate >= ZJ_MIN_RATE && rate <= ZJ_MAX_RATE && target <= 1000000U) {
        /*
         * [EN] Consume job ID even if `zj_begin` rejects due to active switch; no auto-retry.
         * [VI] Ghi nhận mã job ngay cả khi bị từ chối để tránh thực thi lặp lại ngoài ý muốn.
         */
        s->last_id = id;
        s->job = id;
        int direction = (words[3][0] == 'U') ? 1 : -1;
        return zj_begin(s, direction, rate, target, now);
    }

invalid:
    zj_stop(s, ZJ_INVALID);
    return ZJ_REJECTED;
}

/*
 * [EN] Convert `enum zj_reason` into human-readable telemetry string.
 * [VI] Chuyển đổi mã `enum zj_reason` sang chuỗi văn bản phục vụ bản tin trạng thái.
 */
const char *zj_reason_name(enum zj_reason reason)
{
    static const char *const names[] = {
        "idle",
        "stop",
        "top_limit",
        "link_timeout",
        "sensor_error",
        "sensor_stale",
        "sensor_not_ready",
        "invalid_command",
        "complete",
        "timer_error",
        "usb_lost",
        "bottom_limit",
        "limits_conflict",
        "motor_gpio_error",
        "jog_budget",
        "driver_released",
        "calibration_error"
    };
    unsigned idx = (unsigned)reason;
    unsigned total_names = (unsigned)(sizeof(names) / sizeof(names[0]));
    return (idx < total_names) ? names[idx] : "unknown";
}
