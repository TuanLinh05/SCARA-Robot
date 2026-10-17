/*
 * ============================================================================
 * FILE: home_worker.c
 * ============================================================================
 * [EN] Implementation of the 9-phase bidirectional homing and travel-span
 *      calibration state machine.
 *
 * [VI] Triển khai máy trạng thái dò gốc (Homing) và đo kiểm hành trình 2 chiều
 *      qua 9 pha tự động.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - If calibration fails with `HMC_REPEAT` (`cal_error == 7`), it means the
 *   forward pulse count `n1` and reverse pulse count `n2` differed by more than
 *   1%. Check belt tension, pulley set-screws, or stepper missed steps!
 * - Nếu quá trình Homing báo lỗi `HMC_REPEAT` (`cal_error == 7`), nghĩa là số xung
 *   đo được chiều đi (`n1`) và chiều về (`n2`) lệch nhau quá 1%. Sinh viên cần
 *   kiểm tra độ căng dây đai, ốc chí puly bị trượt hoặc động cơ bước bị mất bước!
 * ============================================================================
 */

#include "home_worker.h"
#include <limits.h>

/*
 * [EN] Return true while the calibration state machine is executing one of the 9 motion legs.
 * [VI] Trả về `true` nếu trục đang nằm trong 9 pha chạy dò gốc (`HMC_SEEK_N` .. `HMC_LATCH_N2`).
 */
bool hc_active(const struct hc_state *c)
{
    return (c->phase >= HMC_SEEK_N) && (c->phase <= HMC_LATCH_N2);
}

/*
 * [EN] Abort an active calibration, recording which phase failed and the error code.
 * [VI] Hủy quá trình dò gốc đang chạy, lưu lại pha bị lỗi (`failed_phase`) và nguyên nhân (`error`).
 */
void hc_cancel(struct hc_state *c, struct zj_state *s, enum zj_reason reason, enum hc_error error)
{
    if (!hc_active(c)) {
        return;
    }
    c->failed_phase = c->phase;
    c->phase = HMC_FAILED;
    c->error = error;
    c->reference = false;
    s->job_budget_ms = 0;
    zj_stop(s, reason);
}

/*
 * [EN] Read either the raw GPIO state (`raw == true`) or debounced state (`raw == false`)
 *      of the limit switch in the given `direction`.
 * [VI] Lấy trạng thái công tắc hành trình (mức thô `raw == true` hoặc sau chống dội `raw == false`)
 *      ứng với chiều chuyển động `direction`.
 */
static bool logical_limit(const struct zj_state *s, int direction, bool raw)
{
    bool positive = ((direction > 0) == s->positive_high);
    if (positive) {
        return raw ? (bool)s->raw_pos : s->top;
    }
    return raw ? (bool)s->raw_neg : s->bottom;
}

/*
 * [EN] Return the expected stop reason (`ZJ_TOP` or `ZJ_BOTTOM`) when reaching the switch in `direction`.
 * [VI] Trả về mã nguyên nhân dừng kỳ vọng (`ZJ_TOP` hoặc `ZJ_BOTTOM`) khi chạm công tắc ở chiều `direction`.
 */
static enum zj_reason endpoint(const struct zj_state *s, int direction)
{
    bool positive = ((direction > 0) == s->positive_high);
    return positive ? ZJ_TOP : ZJ_BOTTOM;
}

/*
 * [EN] Configure and start a single calibration motion leg:
 *      - Seek/Sweep legs (`SEEK_N`, `SWEEP_P`, `SWEEP_N`) use fast `c->rate` up to `c->max_scan` pulses.
 *      - Clear (backoff) legs (`CLEAR_N`, `CLEAR_P`, `CLEAR_N2`) move away from the switch at
 *        `c->fine_rate` up to `c->backoff` pulses.
 *      - Latch legs (`LATCH_N`, `LATCH_P`, `LATCH_N2`) approach the switch slowly at `c->fine_rate`
 *        up to `2 * c->backoff` pulses.
 *
 * [VI] Thiết lập và khởi chạy một chặng (leg) trong quy trình dò gốc:
 *      - Pha Tìm/Quét (`SEEK_N`, `SWEEP_P`, `SWEEP_N`): chạy tốc độ nhanh `c->rate`, tối đa `c->max_scan` xung.
 *      - Pha Lùi (`CLEAR_N`, `CLEAR_P`, `CLEAR_N2`): lùi khỏi công tắc với tốc độ chậm `c->fine_rate`,
 *        tối đa `c->backoff` xung.
 *      - Pha Chốt (`LATCH_N`, `LATCH_P`, `LATCH_N2`): tiến chậm `c->fine_rate` vào công tắc,
 *        tối đa `2 * c->backoff` xung.
 */
static enum zj_command_result leg(
    struct hc_state *c,
    struct zj_state *s,
    enum hc_phase phase,
    uint32_t now
)
{
    c->phase = phase;
    c->settling = false;
    c->extra = false;
    c->arrived = false;

    bool clearing = (phase == HMC_CLEAR_N || phase == HMC_CLEAR_N2 || phase == HMC_CLEAR_P);
    bool moves_positive = (phase == HMC_CLEAR_N || phase == HMC_CLEAR_N2 ||
                           phase == HMC_SWEEP_P || phase == HMC_LATCH_P);
    int direction = moves_positive ? 1 : -1;

    uint32_t limit = c->max_scan;
    if (clearing) {
        limit = c->backoff;
    } else if (phase == HMC_LATCH_N || phase == HMC_LATCH_P || phase == HMC_LATCH_N2) {
        limit = c->backoff * 2U;
    }

    bool fast_leg = (phase == HMC_SEEK_N || phase == HMC_SWEEP_P || phase == HMC_SWEEP_N);
    uint32_t rate = fast_leg ? c->rate : c->fine_rate;

    s->job_budget_ms = HMC_PHASE_MS;
    enum zj_command_result result = zj_begin(s, direction, rate, limit, now);

    /*
     * [EN] Also bound a settle at an already-active first stop, and preserve host lease.
     * [VI] Ghi nhận mốc thời gian bắt đầu pha và giữ nguyên thời hạn lease của máy tính.
     */
    s->start_ms = now;
    s->lease_ms = c->lease_ms;

    /*
     * [EN] In HMC_SEEK_N, if the axis is ALREADY resting on the negative switch,
     *      `zj_begin` returns ZJ_REJECTED with `endpoint(s, -1)`; that's normal!
     * [VI] Ở pha đầu tiên HMC_SEEK_N, nếu trục đang đè sẵn lên công tắc âm thì
     *      `zj_begin` từ chối chạy thêm là hoàn toàn bình thường, chuyển thẳng sang xác nhận.
     */
    bool already_at_seek_n = (phase == HMC_SEEK_N) && (s->reason == endpoint(s, -1));
    if (result == ZJ_REJECTED && !already_at_seek_n) {
        hc_cancel(c, s, s->reason, HMC_INPUT);
    }
    return (result == ZJ_STARTED) ? ZJ_STARTED : ZJ_OK;
}

/*
 * [EN] Validate parameters and start the calibration sequence at `HMC_SEEK_N`.
 * [VI] Kiểm tra tính hợp lệ của các thông số và bắt đầu chu trình dò gốc từ pha `HMC_SEEK_N`.
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
)
{
    if (c->fine_rate == 0U) {
        c->fine_rate = ZJ_MIN_RATE;
    }

    bool valid_micro = (micro == 1U || micro == 2U || micro == 4U ||
                        micro == 8U || micro == 16U || micro == 32U);
    bool valid_angle = (angle >= 1000U && angle <= 360000U);
    bool valid_rates = (rate >= 50U && rate <= ZJ_MAX_RATE &&
                        c->fine_rate >= ZJ_MIN_RATE && c->fine_rate <= rate);
    bool valid_scan_backoff = (max_scan >= 256U && max_scan <= 1000000U &&
                               backoff >= 32U && backoff <= 8192U && backoff < max_scan);

    if (hc_active(c) || s->direction != 0 ||
        !valid_angle || !valid_rates || !valid_scan_backoff ||
        !valid_micro || c->epoch == UINT32_MAX) {
        return ZJ_REJECTED;
    }

    c->reference = false;
    c->n1 = 0;
    c->n2 = 0;
    c->false_hits = 0;
    c->error = HMC_NONE;
    c->failed_phase = HMC_IDLE;
    c->run_angle = angle;
    c->run_micro = micro;
    c->rate = rate;
    c->max_scan = max_scan;
    c->backoff = backoff;
    c->start_ms = now;
    c->lease_ms = now;
    c->start_total = s->total;

    return leg(c, s, HMC_SEEK_N, now);
}

/*
 * [EN] Periodic service tick for the calibration state machine:
 *      - Step 1: Monitor host heartbeat lease (`ZJ_LEASE_MS`).
 *      - Step 2: Check limit switch health and sample freshness.
 *      - Step 3: Enforce total time, phase time, and total pulse budgets.
 *      - Step 4: During `CLEAR` legs, wait until the switch closes for 20 ms, then command
 *        16 extra clearance pulses before stopping.
 *      - Step 5: During `SEEK`/`SWEEP`/`LATCH` legs, wait while motor is stepping.
 *      - Step 6: Once stopped at the switch, confirm contact stability for 20 ms
 *        (or resume motion if a brief EMI glitch disappears).
 *      - Step 7: Advance to the next calibration phase and verify 1% repeatability on `LATCH_N2`.
 *
 * [VI] Hàm cập nhật trạng thái dò gốc theo chu kỳ:
 *      - Bước 1: Kiểm tra gói tin duy trì kết nối `KEEP` từ máy tính (`ZJ_LEASE_MS`).
 *      - Bước 2: Kiểm tra lỗi phần cứng hoặc xung đột công tắc hành trình.
 *      - Bước 3: Kiểm tra tổng thời gian, thời gian pha và ngân sách tổng số xung phát ra.
 *      - Bước 4: Trong các pha Lùi (`CLEAR`): ngay khi công tắc đóng ổn định 20 ms, cho chạy thêm
 *        16 xung nữa để tách hẳn khỏi vùng nhạy của tiếp điểm cơ khí.
 *      - Bước 5: Trong các pha Chạm (`SEEK`/`SWEEP`/`LATCH`), chờ khi trục vẫn đang quay.
 *      - Bước 6: Khi trục dừng, xác nhận công tắc mở ổn định 20 ms (nếu chỉ là xung nhiễu
 *        thoáng qua rồi đóng lại thì tự động chạy tiếp `ZJ_RESUMED`).
 *      - Bước 7: Chuyển sang pha tiếp theo và kiểm tra độ lặp lại <= 1% khi hoàn tất `LATCH_N2`.
 */
enum zj_command_result hc_tick(struct hc_state *c, struct zj_state *s, uint32_t now)
{
    if (!hc_active(c)) {
        return ZJ_OK;
    }

    /*
     * [EN] Step 1: Check host heartbeat lease.
     * [VI] Bước 1: Kiểm tra gói tin duy trì kết nối từ máy tính.
     */
    if (now - c->lease_ms >= ZJ_LEASE_MS) {
        hc_cancel(c, s, ZJ_TIMEOUT, HMC_LINK);
        return ZJ_OK;
    }

    /*
     * [EN] Step 2: Check limit switch health and sample freshness.
     * [VI] Bước 2: Kiểm tra lỗi phần cứng hoặc xung đột công tắc hành trình.
     */
    bool both_raw = (s->raw_pos != 0) && (s->raw_neg != 0);
    if (s->error != 0 || s->conflict || both_raw || !zj_ready(s) || (now - s->sample_ms > ZJ_SENSOR_MS)) {
        enum zj_reason stop_reason = ZJ_STALE_SENSOR;
        if (s->error != 0) {
            stop_reason = ZJ_SENSOR;
        } else if (s->conflict || both_raw) {
            stop_reason = ZJ_CONFLICT;
        } else if (!zj_ready(s)) {
            stop_reason = ZJ_NOT_READY;
        }
        hc_cancel(c, s, stop_reason, HMC_INPUT);
        return ZJ_OK;
    }

    /*
     * [EN] Step 3: Enforce total time, phase time, and total pulse budgets.
     * [VI] Bước 3: Kiểm tra tổng thời gian, thời gian pha và ngân sách tổng số xung phát ra.
     */
    uint64_t max_total_pulses = (uint64_t)c->max_scan * 3U + (uint64_t)c->backoff * 9U;
    if ((now - c->start_ms >= HMC_TOTAL_MS) ||
        (now - s->start_ms >= HMC_PHASE_MS) ||
        (s->total - c->start_total >= max_total_pulses)) {
        hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
        return ZJ_OK;
    }

    bool clearing = (c->phase == HMC_CLEAR_N || c->phase == HMC_CLEAR_N2 || c->phase == HMC_CLEAR_P);
    bool is_positive_end = (c->phase == HMC_SWEEP_P || c->phase == HMC_LATCH_P || c->phase == HMC_CLEAR_P);
    int toward = is_positive_end ? 1 : -1;

    /*
     * [EN] Step 4: Handle backoff/clearing phases (`HMC_CLEAR_*`).
     * [VI] Bước 4: Xử lý các pha lùi khỏi công tắc (`HMC_CLEAR_*`).
     */
    if (clearing) {
        if (s->direction != 0) {
            /*
             * [EN] Once the switch is completely released (both debounced and raw LOW),
             *      shorten target to `s->move + 16` pulses so we stop cleanly past the switch.
             * [VI] Khi công tắc đã nhả hoàn toàn, đặt đích dừng sau 16 xung nữa để tạo khoảng hở an toàn.
             */
            if (!c->extra && !logical_limit(s, toward, false) && !logical_limit(s, toward, true)) {
                if (s->move > c->backoff - 16U) {
                    hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
                    return ZJ_OK;
                }
                s->target = s->move + 16U;
                c->extra = true;
            }
            return ZJ_OK;
        }

        /*
         * [EN] Backoff move finished: verify it completed the 16 extra steps and switch is closed.
         * [VI] Khi đã dừng pha lùi: kiểm tra đã đi đủ 16 xung phụ và công tắc thực sự đã đóng.
         */
        if (s->reason != ZJ_DONE || !c->extra || logical_limit(s, toward, true)) {
            hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
            return ZJ_OK;
        }

        enum hc_phase next_latch = HMC_LATCH_N;
        if (c->phase == HMC_CLEAR_P) {
            next_latch = HMC_LATCH_P;
        } else if (c->phase == HMC_CLEAR_N2) {
            next_latch = HMC_LATCH_N2;
        }
        return leg(c, s, next_latch, now);
    }

    /*
     * [EN] Step 5: For seek/sweep/latch phases, wait while motor is still stepping.
     * [VI] Bước 5: Với các pha tiến vào công tắc, nếu trục vẫn đang chạy thì chờ tiếp.
     */
    if (s->direction != 0) {
        return ZJ_OK;
    }

    /*
     * [EN] Motor stopped: verify it stopped because it hit the expected endpoint switch!
     * [VI] Khi trục dừng: kiểm tra xem có đúng là dừng do chạm công tắc đích hay do lỗi/hết hành trình.
     */
    if (s->reason != endpoint(s, toward) || (s->move == 0U && c->phase != HMC_SEEK_N)) {
        enum hc_error error = HMC_BOUNDS;
        if (s->reason == ZJ_SENSOR || s->reason == ZJ_STALE_SENSOR ||
            s->reason == ZJ_NOT_READY || s->reason == ZJ_CONFLICT) {
            error = HMC_INPUT;
        } else if (s->reason == ZJ_TIMEOUT || s->reason == ZJ_USB) {
            error = HMC_LINK;
        } else if (s->reason == ZJ_GPIO || s->reason == ZJ_TIMER) {
            error = HMC_IO;
        }
        enum zj_reason cancel_reason = (s->reason == ZJ_TOP || s->reason == ZJ_BOTTOM) ? ZJ_CAL : s->reason;
        hc_cancel(c, s, cancel_reason, error);
        return ZJ_OK;
    }

    /*
     * [EN] Step 6: Endpoint contact confirmation:
     *      First HIGH stopped the ISR immediately. Now verify via GPIO history:
     *      - If it was a brief noise spike and the switch closed stably for 20 ms,
     *        resume the same leg (`ZJ_RESUMED`) without resetting pulse counts!
     *      - Once HIGH stays open for 20 ms (`ZJ_RELEASE_MS`), advance to next phase.
     *
     * [VI] Bước 6: Xác nhận chạm công tắc thực sự (chống nhiễu điện từ EMI):
     *      - Nếu chỉ là gai nhiễu làm dừng ngắt tạm thời rồi công tắc đóng lại đủ 20 ms,
     *        tăng biến đếm `false_hits` và cho chạy tiếp (`ZJ_RESUMED`).
     *      - Nếu công tắc giữ mức CAO đủ 20 ms, xác nhận đã chạm cữ thật và chuyển pha.
     */
    if (!c->arrived) {
        c->arrived = true;
        c->hit_ms = now;
    }

    bool positive = ((toward > 0) == s->positive_high);
    bool high = logical_limit(s, toward, true);
    uint32_t opened = positive ? s->opened_ms : s->opened_bottom_ms;
    uint32_t closed = positive ? s->closed_ms : s->closed_bottom_ms;
    bool closing = positive ? s->closing : s->closing_bottom;

    if (!high && closing && !logical_limit(s, toward, false) && (now - closed >= ZJ_RELEASE_MS)) {
        if (s->target != 0U && s->move >= s->target) {
            hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
            return ZJ_OK;
        }
        s->direction = toward;
        s->reason = ZJ_IDLE;
        c->arrived = false;
        c->settling = false;
        if (c->false_hits < UINT32_MAX) {
            c->false_hits++;
        }
        return ZJ_RESUMED;
    }

    if (!(high && (now - opened >= ZJ_RELEASE_MS))) {
        if (now - c->hit_ms >= HMC_CONFIRM_MS) {
            hc_cancel(c, s, ZJ_CAL, HMC_UNSTABLE);
        }
        return ZJ_OK;
    }

    /*
     * [EN] Step 7: Switch contact confirmed! Transition to the next calibration phase.
     * [VI] Bước 7: Đã xác nhận chạm công tắc ổn định! Chuyển sang pha tiếp theo của chu trình.
     */
    switch (c->phase) {
    case HMC_SEEK_N:
        return leg(c, s, HMC_CLEAR_N, now);

    case HMC_LATCH_N:
        c->negative_anchor = s->position;
        /*
         * [EN] In `QUICK` mode, if a valid `pulses` span was saved from a prior full calibration,
         *      finish immediately after latching the negative switch!
         * [VI] Ở chế độ `QUICK`, nếu đã có sẵn số xung hành trình `pulses` từ lần dò đầy đủ trước đó,
         *      hoàn tất ngay sau khi chốt công tắc âm mà không cần quét sang đầu dương!
         */
        if (c->quick && c->pulses >= 64U) {
            c->n1 = c->pulses;
            c->n2 = c->pulses;
            c->angle_mdeg = c->run_angle;
            c->micro = c->run_micro;
            c->origin = s->position;
            c->reference = true;
            c->epoch++;
            c->phase = HMC_DONE;
            c->error = HMC_NONE;
            s->job_budget_ms = 0;
            zj_stop(s, ZJ_DONE);
            return ZJ_OK;
        }
        return leg(c, s, HMC_SWEEP_P, now);

    case HMC_SWEEP_P:
        return leg(c, s, HMC_CLEAR_P, now);

    case HMC_LATCH_P: {
        if (s->position <= c->negative_anchor) {
            hc_cancel(c, s, ZJ_CAL, HMC_REPEAT);
            return ZJ_OK;
        }
        uint64_t distance = (uint64_t)s->position - (uint64_t)c->negative_anchor;
        if (distance < 64U || distance > c->max_scan) {
            hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
            return ZJ_OK;
        }
        c->n1 = (uint32_t)distance;
        c->positive_anchor = s->position;
        return leg(c, s, HMC_SWEEP_N, now);
    }

    case HMC_SWEEP_N:
        return leg(c, s, HMC_CLEAR_N2, now);

    case HMC_LATCH_N2: {
        if (c->positive_anchor <= s->position) {
            hc_cancel(c, s, ZJ_CAL, HMC_REPEAT);
            return ZJ_OK;
        }
        uint64_t distance = (uint64_t)c->positive_anchor - (uint64_t)s->position;
        if (distance < 64U || distance > c->max_scan) {
            hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS);
            return ZJ_OK;
        }
        c->n2 = (uint32_t)distance;

        /*
         * [EN] Verify forward (`n1`) and reverse (`n2`) spans agree within 1% (min 4 pulses).
         * [VI] Kiểm tra độ lệch giữa lượt đi `n1` và lượt về `n2` không vượt quá 1% (tối thiểu 4 xung).
         */
        uint32_t mean = (c->n1 + c->n2 + 1U) / 2U;
        uint32_t difference = (c->n1 > c->n2) ? (c->n1 - c->n2) : (c->n2 - c->n1);
        uint32_t tolerance = mean / 100U;
        if (tolerance < 4U) {
            tolerance = 4U;
        }
        if (difference > tolerance) {
            hc_cancel(c, s, ZJ_CAL, HMC_REPEAT);
            return ZJ_OK;
        }

        c->pulses = mean;
        c->angle_mdeg = c->run_angle;
        c->micro = c->run_micro;
        c->origin = s->position;
        c->reference = true;
        c->epoch++;
        c->phase = HMC_DONE;
        c->error = HMC_NONE;
        s->job_budget_ms = 0;
        zj_stop(s, ZJ_DONE);
        return ZJ_OK;
    }

    default:
        hc_cancel(c, s, ZJ_CAL, HMC_BAD_COMMAND);
        return ZJ_OK;
    }
}
