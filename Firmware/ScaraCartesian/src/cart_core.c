/*
 * ============================================================================
 * FILE: cart_core.c
 * ============================================================================
 * [EN] 3-Axis SCARA Core Motion Engine, Fixed-Point DDA, Belt Coupling & Homing
 *      Implements the real-time integer pulse interpolator (DDA), quintic
 *      smootherstep velocity profiler, backlash compensation, differential
 *      belt coupling (`beta`), and the multi-stage 3-axis initialization state
 *      machine (`HS_Z -> HS_J2_PRE -> HS_J2_MID -> HS_COUPLE_* -> HS_J1 ->
 *      HS_J1_MID -> HS_J2_FINAL -> HS_PARK -> HS_READY`).
 *
 * [VI] Động cơ Điều khiển Chuyển động Lõi 3 Trục SCARA, Nội suy DDA & Dò Gốc
 *      Triển khai bộ nội suy xung đa trục thời gian thực (DDA), quy hoạch vận tốc
 *      đa thức bậc 5 (quintic smootherstep), bù độ rơ cơ khí, bù truyền động đai
 *      vi sai (`beta`), và máy trạng thái tự động dò gốc 3 trục.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - Why does Motor B (`CT_J2`) step during `CT_J1` calibration?
 *   Because the forearm belt passes over the shoulder pulley! If Motor A (`CT_J1`)
 *   rotates without Motor B (`CT_J2`) compensating, the relative elbow angle
 *   would change and crash into the J2 limit switch. `cal_drive()` sets
 *   `c->cal_weight[2] = llabs(c->beta)` so Motor B tracks Motor A automatically!
 * - Tại sao Động cơ B (`CT_J2`) lại phát xung cùng lúc khi đang dò gốc Khớp 1 (`CT_J1`)?
 *   Vì dây đai kéo khâu 2 đi vòng qua puly khớp 1! Nếu Khớp 1 quay mà Động cơ B
 *   đứng yên, góc khuỷu tương đối sẽ bị thay đổi và va vào công tắc hành trình J2.
 *   Do đó hàm `cal_drive()` gán `c->cal_weight[2] = llabs(c->beta)` để Động cơ B
 *   tự động chạy bù giữ nguyên góc mở khuỷu tay!
 * ============================================================================
 */

#include "cart_core_internal.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

/*
 * [EN] Fixed-point unit conversion constants used throughout `cart_core.c`:
 *      - `CT_Q10_SCALE = 1024`: `factor[a]` stores `(pulses per mm or deg) * 1024`.
 *      - `CT_MDEG_PER_DEG = 1000`: Angles are stored in millidegrees (`1 deg = 1000 mdeg`),
 *        and Z distances in micrometers (`1 mm = 1000 um`).
 *      - `CT_FACTOR_MDEG_DIV = 1024 * 1000 = 1,024,000`: Combines the Q10 scale (`1024`)
 *        and the milli-unit scale (`1000`) into a single integer divisor!
 *      - `CT_PPM_UNIT = 1,000,000`: Parts-per-million (`1.0 = 1,000,000 ppm`).
 *
 * [VI] Các hằng số quy đổi đơn vị dấu phẩy tĩnh dùng xuyên suốt trong `cart_core.c`:
 *      - `CT_Q10_SCALE = 1024`: Hệ số `factor[a]` lưu `(số xung trên mỗi mm hoặc độ) * 1024`.
 *      - `CT_MDEG_PER_DEG = 1000`: Góc lưu bằng phần nghìn độ (`1 độ = 1000 mdeg`),
 *        và hành trình Z lưu bằng micromet (`1 mm = 1000 um`).
 *      - `CT_FACTOR_MDEG_DIV = 1024 * 1000 = 1.024.000`: Gộp cả hệ số Q10 (`1024`)
 *        lẫn hệ số phần nghìn (`1000`) vào chung một mẫu số nguyên!
 *      - `CT_PPM_UNIT = 1.000.000`: Đơn vị phần triệu (`1.0 = 1.000.000 ppm`).
 */
#define CT_FACTOR_MDEG_DIV 1024000LL
#define CT_PPM_UNIT        1000000LL

/*
 * [EN] Return the sign of a 64-bit integer: -1 if negative, 0 if zero, +1 if positive.
 * [VI] Trả về dấu của số nguyên 64-bit: -1 nếu âm, 0 nếu bằng 0, +1 nếu dương.
 */
static int sign64(int64_t x)
{
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

/*
 * [EN] Symmetric half-way integer division: rounds `x / d` to the nearest integer
 *      away from zero (matching `round_nearest` in Python and `round` in MATLAB).
 *      Why add `d / 2`? Because integer division in C truncates toward zero;
 *      adding half the divisor shifts the truncation threshold to 0.5!
 *
 * [VI] Phép chia làm tròn số nguyên đối xứng qua 0: làm tròn `x / d` tới số nguyên
 *      gần nhất (đồng bộ hoàn toàn với hàm `round_nearest` bên Python và `round` bên MATLAB).
 *      Tại sao cộng `d / 2`? Vì phép chia số nguyên trong C tự động cắt bỏ phần thập phân;
 *      việc cộng thêm một nửa mẫu số giúp làm tròn đúng tại ngưỡng 0.5!
 */
static int64_t rounded(int64_t x, int64_t d)
{
    if (x >= 0) {
        return (x + d / 2) / d;
    }
    return -((-x + d / 2) / d);
}

/*
 * [EN] Pack raw GPIO switch states of all 3 axes into a 6-bit diagnostic mask:
 *      - Bit 0: Z positive switch,  Bit 1: Z negative switch
 *      - Bit 2: J1 positive switch, Bit 3: J1 negative switch
 *      - Bit 4: J2 positive switch, Bit 5: J2 negative switch
 *
 * [VI] Đóng gói mức tín hiệu thô của 6 công tắc hành trình (3 trục x 2 đầu) vào
 *      một mặt nạ 6 bit để phục vụ chẩn đoán lỗi:
 *      - Bit 0: Công tắc dương Z,  Bit 1: Công tắc âm Z
 *      - Bit 2: Công tắc dương J1, Bit 3: Công tắc âm J1
 *      - Bit 4: Công tắc dương J2, Bit 5: Công tắc âm J2
 */
static unsigned raw_bits(const struct ct_state *c)
{
    unsigned bits = 0;
    for (unsigned a = 0; a < CT_AXES; a++) {
        unsigned pos_bit = (c->sw[a].raw_pos != 0) ? 1U : 0U;
        unsigned neg_bit = (c->sw[a].raw_neg != 0) ? 2U : 0U;
        unsigned axis_bits = pos_bit | neg_bit;
        bits |= (axis_bits << (2U * a));
    }
    return bits;
}

/*
 * [EN] Record a diagnostic switch input event (axis, raw bitmask, event kind, timestamp).
 * [VI] Ghi nhận sự kiện bất thường của công tắc hành trình vào bộ nhớ chẩn đoán.
 */
static void input_event(struct ct_state *c, unsigned axis, unsigned kind, uint32_t now)
{
    c->input_axis = (int)axis;
    c->input_bits = raw_bits(c);
    c->input_kind = kind;
    c->input_at_ms = now;
}

/*
 * [EN] Determine whether axis `a` is mechanically involved in the current motion:
 *      - Outside `initializing`, all axes are monitored.
 *      - During `INIT`, J1 motion also involves J2 through the mechanical belt coupling,
 *        whereas Z motion does not move either arm joint.
 *
 * [VI] Kiểm tra xem trục `a` có tham gia cơ học vào chuyển động hiện tại không:
 *      - Khi chạy bình thường, giám sát tất cả các trục.
 *      - Khi đang `INIT`, chuyển động của J1 kéo theo J2 qua dây đai nên J2 cũng được
 *        tính là có tham gia; ngược lại khi Z chạy thì J1 và J2 đứng yên.
 */
static bool involved(const struct ct_state *c, unsigned a)
{
    if (!c->initializing) {
        return true;
    }
    if (c->mode == CT_CAL) {
        bool is_selected_axis = (a == c->selected);
        bool is_coupled_j2 = (a == CT_J2 && c->selected == CT_J1);
        return is_selected_axis || is_coupled_j2;
    }
    if (c->mode == CT_PARK) {
        bool axis_has_steps = (c->delta[a] != 0U);
        bool j2_coupled_to_j1 = (a == CT_J2 && c->delta[CT_J1] != 0U);
        return axis_has_steps || j2_coupled_to_j1;
    }
    return true;
}

/*
 * [EN] Temporarily freeze pulse emission during `INIT` when a switch glitch or
 *      guard contact is detected, preserving DDA state until `stationary_check` finishes.
 * [VI] Tạm dừng phát xung trong quá trình `INIT` khi phát hiện nhiễu công tắc hoặc
 *      chạm bảo vệ J2, giữ nguyên bộ tích lũy DDA để kiểm tra tĩnh (`stationary_check`).
 */
static void pause_input(struct ct_state *c, unsigned a, uint32_t now, bool guard)
{
    if (!c->input_wait) {
        c->wait_running = c->running;
        c->wait_cm_dir = c->cm.direction;
        c->wait_cm_reason = c->cm.reason;
        c->wait_axes = 0;
        c->wait_bits = 0;
        c->wait_guard = false;
        c->wait_start_ms = now;
        c->wait_stable_ms = now;
        c->input_wait = true;
        c->running = false;
        input_event(c, a, 1, now);
        c->reason = "checking_inputs";
    }
    c->wait_axes |= (1U << a);
    c->wait_guard |= guard;
}

/*
 * [EN] Return true if the controller is actively stepping, initializing, or streaming a path.
 * [VI] Trả về `true` nếu bộ điều khiển đang phát xung, đang dò gốc `INIT`, hoặc đang chạy quỹ đạo `PATH`.
 */
bool ct_busy(const struct ct_state *c)
{
    return c->running || c->initializing || c->path_active;
}

/*
 * [EN] Initialize 3-axis controller state with default SCARA geometry and homing parameters.
 * [VI] Khởi tạo trạng thái mặc định cho bộ điều khiển SCARA 3 trục.
 */
void ct_init(struct ct_state *c)
{
    memset(c, 0, sizeof(*c));
    c->input_axis = -1;

    for (unsigned a = 0; a < CT_AXES; a++) {
        zj_init_joint(&c->sw[a]);
        c->holding[a] = true;
        /*
         * [EN] J2 DIR HIGH turns opposite to J1; internal step counts use a unified geometric sign.
         * [VI] Chân DIR mức CAO của J2 quay ngược chiều với J1; nội bộ dùng chung chiều hình học.
         */
        c->pol[a] = (a != CT_J2);
    }
    zj_init_joint(&c->cm);

    /*
     * [EN] Default nominal scale factors (`pulses per mm/deg * 1024`) and microstepping:
     *      - Z (`nominal[0]`) : 1600 steps/mm * 1024 = 1,638,400
     *      - J1 (`nominal[1]`): ~26.67 steps/deg * 1024 = 27,307
     *      - J2 (`nominal[2]`): 40 steps/deg * 1024 = 40,960
     * [VI] Hệ số quy đổi xung mặc định (`số xung trên mm/độ * 1024`) và vi bước:
     *      - Trục Z (`nominal[0]`) : 1600 xung/mm * 1024 = 1.638.400
     *      - Khớp J1 (`nominal[1]`): ~26,67 xung/độ * 1024 = 27.307
     *      - Khớp J2 (`nominal[2]`): 40 xung/độ * 1024 = 40.960
     */
    c->nominal[0] = 1638400;
    c->nominal[1] = 27307;
    c->nominal[2] = 40960;
    c->micro[0] = 16;
    c->micro[1] = 16;
    c->micro[2] = 8;

    for (unsigned j = 0; j < 2U; j++) {
        c->low_md[j] = -90000;
        c->high_md[j] = 90000;
    }

    /*
     * [EN] Default belt coupling ratio (333,333 ppm = 1/3) and park pose parameters.
     * [VI] Hệ số truyền động đai vi sai mặc định (333.333 ppm = 1/3) và thông số vị trí Park.
     */
    c->coupling_ppm = 333333;
    c->park_md[1] = 45000;
    c->auto_coupling = true;
    c->probe_pulses = 128;

    c->z_clear_um = 3000;
    c->arm_rate = 200;
    c->z_rate = 3200;
    c->z_fine_rate = 400;
    c->j2_rate = 800;
    c->j2_fine_rate = 200;
    c->arm_scan = 40000;
    c->z_scan = 300000;
    c->arm_back = 512;
    c->z_back = 1024;

    c->reason = "unreferenced";
    c->reply = "unreferenced";
}

/*
 * [EN] Emergency/controlled abort: stops all motion, cancels calibration, clears
 *      the path FIFO, invalidates `referenced`, and records the stop `reason`.
 * [VI] Hủy khẩn cấp mọi hoạt động: dừng phát xung, hủy quá trình dò gốc, xóa
 *      hàng đợi quỹ đạo, xóa cờ `referenced` và ghi lại nguyên nhân `reason`.
 */
void ct_abort(struct ct_state *c, const char *reason)
{
    if (c->initializing) {
        hc_cancel(&c->cal[c->selected], &c->cm, ZJ_STOP, HMC_CANCELLED);
        c->stage = HS_FAILED;
    }
    c->running = false;
    c->initializing = false;
    c->quick_init = false;
    c->referenced = false;
    c->input_wait = false;
    c->gate_wait = false;
    c->mode = CT_IDLE;
    c->reason = reason;
    c->path_active = false;
    c->path_started = false;
    c->path_size = 0;
    c->comp_step = false;
    memset(c->comp_left, 0, sizeof(c->comp_left));
    memset(c->direction, 0, sizeof(c->direction));
}

/*
 * [EN] Feed a simultaneous positive/negative limit switch GPIO reading for axis `a`.
 *      Updates both the axis monitor `c->sw[a]` and, during `CT_CAL`, the calibration
 *      monitor `c->cm`.
 * [VI] Nạp mẫu đọc cặp công tắc hành trình âm/dương của trục `a` vào bộ lọc chống dội
 *      `c->sw[a]` và bộ giám sát hiệu chuẩn `c->cm`.
 */
void ct_sample(struct ct_state *c, unsigned a, int positive, int negative, uint32_t now)
{
    if (a >= CT_AXES) {
        return;
    }

    bool had_conflict = c->sw[a].conflict;
    bool had_both = (c->sw[a].raw_pos != 0) && (c->sw[a].raw_neg != 0);
    int had_error = c->sw[a].error;

    zj_switches(&c->sw[a], positive, negative, now);

    /*
     * [EN] Record newly latched switch conflicts (`kind = 2`) or GPIO errors (`kind = 4`).
     * [VI] Ghi nhận sự kiện nếu vừa phát sinh xung đột 2 đầu công tắc (`kind = 2`) hoặc lỗi GPIO (`kind = 4`).
     */
    if ((!had_conflict && c->sw[a].conflict) || (!had_error && c->sw[a].error != 0)) {
        unsigned kind = (c->sw[a].error != 0) ? 4U : 2U;
        input_event(c, a, kind, now);
        if (!ct_busy(c) && !c->referenced) {
            c->reason = (c->sw[a].error != 0) ? "switch_io" : "both_latched";
        }
    }

    /*
     * [EN] Abort immediately if a switch conflict/error occurs while busy or referenced.
     * [VI] Hủy chuyển động và xóa gốc ngay nếu có lỗi công tắc khi đang chạy hoặc đã có gốc.
     */
    if ((c->sw[a].conflict || c->sw[a].error != 0) && (ct_busy(c) || c->referenced)) {
        const char *abort_reason = (c->sw[a].error != 0) ? "switch_io" : "both_latched";
        ct_abort(c, abort_reason);
    } else if (c->referenced && positive == 1 && negative == 1) {
        /*
         * [EN] Never silently retain a home reference if both switches open while idle.
         * [VI] Nếu đang đứng yên (đã có gốc) mà cả 2 công tắc cùng mở (đứt dây), xóa gốc ngay.
         */
        input_event(c, a, 1, now);
        ct_abort(c, "both_open");
    }

    /*
     * [EN] During `INIT`, handle transient switch glitches or J2 guard contacts via `pause_input`.
     * [VI] Trong khi `INIT`, xử lý chống nhiễu thoáng qua hoặc bảo vệ va chạm J2 qua `pause_input`.
     */
    if (c->initializing && positive == 1 && negative == 1 && !c->sw[a].conflict) {
        if (involved(c, a)) {
            bool is_j2_guard = (c->mode == CT_CAL && c->selected == CT_J1 && a == CT_J2);
            pause_input(c, a, now, is_j2_guard);
        } else if (!had_both) {
            if (c->idle_ignored < UINT32_MAX) {
                c->idle_ignored++;
            }
            input_event(c, a, 6, now);
        }
    } else if (c->initializing &&
               ((c->mode == CT_CAL && c->selected == CT_J1) || c->stage == HS_COUPLE_PROBE) &&
               a == CT_J2 && (positive == 1 || negative == 1)) {
        pause_input(c, a, now, true);
    }

    /*
     * [EN] Forward samples of the currently calibrating axis to the calibration monitor `c->cm`.
     * [VI] Chuyển tiếp mẫu đọc của trục đang dò gốc vào máy trạng thái `c->cm`.
     */
    if (c->initializing && c->mode == CT_CAL && a == c->selected) {
        int previous = c->cm.direction;
        zj_switches(&c->cm, positive, negative, now);
        if (previous != 0 && c->cm.direction == 0 && !c->input_wait &&
            (c->cm.reason == ZJ_TOP || c->cm.reason == ZJ_BOTTOM)) {
            c->last_hit_bits = raw_bits(c);
            c->last_hit_ms = now;
        }
    }
}

/*
 * [EN] Validate all 3 axes' switch states, motor hold states, and sample freshness.
 *      Returns `NULL` if everything is healthy, or an error token string otherwise.
 * [VI] Kiểm tra toàn diện lỗi động cơ, phanh từ (`holding`), lỗi công tắc và độ tươi
 *      của dữ liệu cảm biến trên cả 3 trục. Trả về `NULL` nếu an toàn.
 */
const char *ct_inputs(struct ct_state *c, uint32_t now)
{
    if (c->fault) {
        return "motor_fault";
    }
    for (unsigned a = 0; a < CT_AXES; a++) {
        const struct zj_state *s = &c->sw[a];
        if (!c->holding[a]) {
            return "released";
        }
        if (s->error != 0) {
            if (c->input_axis != (int)a || c->input_kind != 4U) {
                input_event(c, a, 4, now);
            }
            return "switch_io";
        }
        if (s->conflict) {
            if (c->input_axis != (int)a || c->input_kind != 2U) {
                input_event(c, a, 2, now);
            }
            return "both_latched";
        }
        if (s->raw_pos != 0 && s->raw_neg != 0 && !c->input_wait && involved(c, a)) {
            input_event(c, a, 1, now);
            return "both_open";
        }
        if (!zj_ready(s)) {
            return "switch_not_ready";
        }
        if (now - s->sample_ms > ZJ_SENSOR_MS) {
            return "switch_stale";
        }
    }
    return NULL;
}

/*
 * [EN] Return true if axis `a` is blocked by a limit switch in geometric direction `joint_dir`.
 * [VI] Trả về `true` nếu trục `a` đang bị chặn bởi công tắc hành trình ở phía `joint_dir`.
 */
static bool blocked(const struct ct_state *c, unsigned a, int joint_dir)
{
    if (joint_dir == 0) {
        return false;
    }
    bool positive = ((joint_dir > 0) == c->pol[a]);
    const struct zj_state *s = &c->sw[a];
    if (positive) {
        return s->top || (s->raw_pos != 0);
    }
    return s->bottom || (s->raw_neg != 0);
}

/*
 * [EN] Compute fixed-point belt coupling ratio `c->beta` (in `CT_Q` = 2^20 units):
 *      `beta = round(coupling_ppm * f2 * CT_Q / (1e6 * f1))`
 *      This represents how many Motor B (`CT_J2`) steps correspond to 1 Motor A (`CT_J1`)
 *      step when keeping the relative elbow angle constant.
 *
 * [VI] Tính hệ số bù truyền động đai dấu phẩy tĩnh `c->beta` (đơn vị `CT_Q` = 2^20):
 *      Biểu diễn số xung mà Động cơ B (`CT_J2`) cần quay bù trên mỗi xung của Động cơ A (`CT_J1`)
 *      để giữ nguyên góc mở tương đối của khuỷu tay.
 */
void ct_beta_update(struct ct_state *c)
{
    if (c->coupling_ready) {
        return; /* [EN] Verified pulse ratio already locked / [VI] Đã chốt tỷ số đo thực nghiệm */
    }
    uint32_t f1 = (c->factor[1] != 0U) ? c->factor[1] : c->nominal[1];
    uint32_t f2 = (c->factor[2] != 0U) ? c->factor[2] : c->nominal[2];

    int64_t num = (int64_t)c->coupling_ppm * (int64_t)f2 * CT_Q;
    int64_t den = CT_PPM_UNIT * (int64_t)f1;
    c->beta = rounded(num, den);
}

/*
 * [EN] Check whether motor step coordinate vector `p[CT_AXES]` is inside soft limits:
 *      - Step 1: Z step coordinate `z = p[0] - origin[0]` must be in `[0, range[0]]`.
 *      - Step 2: Convert J1 and J2 motor steps (`p[1] - origin[1]`, `p[2] - origin[2]`)
 *        into millidegrees (`a_md`, `b_md`) using `CT_FACTOR_MDEG_DIV = 1024 * 1000`.
 *      - Step 3: Subtract the belt coupling shift `(a_md * coupling_ppm) / 1e6` from
 *        Motor B angle `b_md` to obtain the true relative elbow angle `elbow_md`.
 *      - Step 4: Verify both `a_md` and `elbow_md` lie within `[low_md + margin_md, high_md - margin_md]`.
 *
 * [VI] Kiểm tra véc-tơ tọa độ xung `p[CT_AXES]` có nằm trong vùng làm việc cho phép không:
 *      - Bước 1: Tọa độ Z (`p[0] - origin[0]`) phải nằm trong đoạn `[0, range[0]]`.
 *      - Bước 2: Đổi số xung động cơ J1, J2 sang phần nghìn độ (`a_md`, `b_md`) nhờ nhân với
 *        `CT_FACTOR_MDEG_DIV = 1024 * 1000` rồi chia cho `factor`.
 *      - Bước 3: Trừ đi lượng kéo theo của dây đai `(a_md * coupling_ppm) / 1e6` khỏi góc
 *        Động cơ B (`b_md`) để thu được góc mở khuỷu tương đối thực sự `elbow_md`.
 *      - Bước 4: Kiểm tra cả `a_md` và `elbow_md` đều nằm trong `[low_md + margin_md, high_md - margin_md]`.
 */
bool ct_pose_valid(const struct ct_state *c, const int32_t p[CT_AXES], int margin_md)
{
    /* [EN] Step 1: Check Z axis step bounds. / [VI] Bước 1: Kiểm tra giới hạn xung trục Z. */
    int64_t z = (int64_t)p[0] - c->origin[0];
    if (z < 0 || z > (int64_t)c->range[0]) {
        return false;
    }

    /* [EN] Step 2: Convert J1 & J2 motor steps to millidegrees. / [VI] Bước 2: Đổi số xung J1, J2 sang mdeg. */
    int64_t j1_steps = (int64_t)p[1] - c->origin[1];
    int64_t j2_steps = (int64_t)p[2] - c->origin[2];
    int64_t a_md = rounded(j1_steps * CT_FACTOR_MDEG_DIV, c->factor[1]);
    int64_t b_md = rounded(j2_steps * CT_FACTOR_MDEG_DIV, c->factor[2]);

    /* [EN] Step 3: Compute true relative elbow angle `elbow_md`. / [VI] Bước 3: Tính góc khuỷu tương đối `elbow_md`. */
    int64_t coupled_md = rounded(a_md * c->coupling_ppm, CT_PPM_UNIT);
    int64_t elbow_md = b_md - coupled_md;

    /* [EN] Step 4: Compare against upper/lower soft limits. / [VI] Bước 4: So sánh với giới hạn góc mềm. */
    int32_t high_0 = (c->span_md[0] != 0) ? (c->low_md[0] + c->span_md[0]) : c->high_md[0];
    int32_t high_1 = (c->span_md[1] != 0) ? (c->low_md[1] + c->span_md[1]) : c->high_md[1];

    bool j1_ok = (a_md >= c->low_md[0] + margin_md) && (a_md <= high_0 - margin_md);
    bool j2_ok = (elbow_md >= c->low_md[1] + margin_md) && (elbow_md <= high_1 - margin_md);
    return j1_ok && j2_ok;
}

/*
 * [EN] Real-time safety guard checked on every pulse interrupt and main-loop tick:
 *      Verifies switch health, host heartbeat lease (`CT_LEASE`), total motion time
 *      budget, step position ceiling, limit-switch blocking, and soft limits.
 *
 * [VI] Hàm bảo vệ an toàn thời gian thực được gọi ở mỗi ngắt xung và mỗi vòng lặp chính:
 *      Kiểm tra công tắc, thời hạn gói tin KEEP (`CT_LEASE`), ngân sách thời gian,
 *      giới hạn đếm xung tuyệt đối, chặn chạm công tắc và giới hạn mềm.
 */
bool ct_safe(struct ct_state *c, uint32_t now)
{
    if (!c->running) {
        return false;
    }

    /*
     * [EN] 1. Check switches, watchdog lease, duration budget, and step position ceiling.
     * [VI] 1. Kiểm tra công tắc, thời gian chờ KEEP, tổng thời gian chạy và giới hạn đếm xung.
     */
    const char *why = ct_inputs(c, now);
    if (why == NULL && (now - c->lease_ms >= CT_LEASE)) {
        why = "heartbeat_lost";
    }
    uint32_t max_duration = c->initializing ? CT_INIT_MS : 600000U;
    if (why == NULL && (now - c->start_ms >= max_duration)) {
        why = "time_budget";
    }
    for (unsigned a = 0; a < CT_AXES && why == NULL; a++) {
        if (c->pos[a] <= -CT_MAX_POS || c->pos[a] >= CT_MAX_POS) {
            why = "position_budget";
        }
    }

    if (why != NULL) {
        ct_abort(c, why);
        return false;
    }

    /*
     * [EN] 2. Mode-specific limit checks:
     *         - In `CT_CAL`, stop pulses cleanly when the calibration monitor hits a switch.
     *         - In `CT_MOVE`/`CT_PARK`, abort if any axis hits a limit switch or exits soft limits.
     * [VI] 2. Kiểm tra giới hạn theo từng chế độ:
     *         - Khi đang dò gốc (`CT_CAL`): dừng phát xung khi chạm công tắc để worker xử lý.
     *         - Khi đang chạy (`CT_MOVE`/`CT_PARK`): hủy lệnh ngay nếu chạm công tắc hoặc vượt giới hạn mềm.
     */
    if (c->mode == CT_CAL) {
        if (c->cm.direction == 0 || !zj_safe(&c->cm, now)) {
            c->running = false;
            return false; /* [EN] Worker confirms/backs off at endpoint / [VI] Dừng xung để worker xác nhận điểm chạm */
        }
    } else {
        if (c->stage == HS_COUPLE_PROBE && blocked(c, CT_J1, c->direction[CT_J1])) {
            /*
             * [EN] Near a J1 endpoint during coupling probe: stop cleanly and retry away.
             * [VI] Nếu nhích thử J1 chạm công tắc, dừng nhẹ nhàng để đảo chiều nhích thử.
             */
            c->running = false;
            c->probe_contact = true;
            return false;
        }
        for (unsigned a = 0; a < CT_AXES; a++) {
            if (blocked(c, a, c->joint_dir[a])) {
                const char *lim_name = "j2_limit";
                if (a == CT_Z) {
                    lim_name = "z_limit";
                } else if (a == CT_J1) {
                    lim_name = "j1_limit";
                }
                ct_abort(c, lim_name);
                return false;
            }
        }
        if (c->mode == CT_MOVE && (!c->referenced || !ct_pose_valid(c, c->pos, 0))) {
            ct_abort(c, "soft_limit");
            return false;
        }
    }
    return true;
}

/*
 * [EN] Prepare a coordinated 3-axis DDA move from `c->pos` to `target` at `rate` pps:
 *      - Step 1: Compute per-axis step deltas `c->delta[a]` and dominant axis count `c->ticks`.
 *      - Step 2: Schedule backlash take-up pulses (`c->comp_left[a]`) if J1 or J2 reverses direction.
 *      - Step 3: Compute physical joint motion direction `c->joint_dir[a]` (accounting for belt coupling on J2).
 *
 * [VI] Chuẩn bị chuyển động nội suy DDA 3 trục từ `c->pos` tới `target` ở tốc độ `rate` xung/giây:
 *      - Bước 1: Tính khoảng cách xung từng trục `c->delta[a]` và tổng số nhịp `c->ticks` theo trục dài nhất.
 *      - Bước 2: Lên lịch phát xung bù độ rơ (`c->comp_left[a]`) nếu trục J1 hoặc J2 đảo chiều quay.
 *      - Bước 3: Tính chiều chuyển động thực tế của khớp `c->joint_dir[a]` (có trừ độ kéo theo dây đai ở J2).
 */
enum ct_result ct_raw_move(struct ct_state *c, const int32_t target[CT_AXES], uint32_t rate, bool park)
{
    c->tick = 0;
    c->ticks = 0;
    c->rate = rate;
    c->ramp_tick = 0;
    c->comp_step = false;

    /*
     * [EN] Step 1: Compute step distance `delta[a]` for each axis and find the longest axis (`ticks`).
     * [VI] Bước 1: Tính khoảng cách bước `delta[a]` của từng trục và tìm trục có số bước lớn nhất (`ticks`).
     */
    for (unsigned a = 0; a < CT_AXES; a++) {
        int64_t d = (int64_t)target[a] - c->pos[a];
        if (target[a] < -CT_MAX_POS || target[a] > CT_MAX_POS || llabs(d) > 1000000LL) {
            return CT_REJECTED;
        }
        c->goal[a] = target[a];
        c->direction[a] = sign64(d);
        c->delta[a] = (uint32_t)llabs(d);
        c->accum[a] = 0;
        if (c->delta[a] > c->ticks) {
            c->ticks = c->delta[a];
        }
    }

    if (c->ticks > 590U * rate) {
        return CT_REJECTED;
    }

    /*
     * [EN] Step 2: Arm backlash compensation when reversing motor direction during normal moves.
     * [VI] Bước 2: Nạp số xung bù rơ khi động cơ tay máy (J1, J2) đảo chiều trong chế độ chạy thường.
     */
    for (unsigned a = 0; a < CT_AXES; a++) {
        c->comp_left[a] = 0;
        if (a != CT_Z && c->direction[a] != 0) {
            if (!park && !c->initializing && c->referenced &&
                c->last_dir[a] != 0 && c->direction[a] != c->last_dir[a]) {
                c->comp_left[a] = c->backlash[a];
            }
            c->last_dir[a] = c->direction[a];
        }
    }

    /*
     * [EN] Step 3: Determine physical joint motion directions. For the elbow (`joint_dir[2]`),
     *      subtract the belt-coupled shoulder motion `beta * d_j1` from Motor B motion `d_j2 * CT_Q`.
     * [VI] Bước 3: Tính chiều chuyển động vật lý của từng khớp. Với khớp khuỷu (`joint_dir[2]`),
     *      lấy độ dịch chuyển Động cơ B (`d_j2 * CT_Q`) trừ đi lượng kéo theo do khớp vai (`beta * d_j1`).
     */
    c->joint_dir[0] = c->direction[0];
    c->joint_dir[1] = c->direction[1];
    int64_t d_j1 = (int64_t)target[1] - c->pos[1];
    int64_t d_j2 = (int64_t)target[2] - c->pos[2];
    int64_t net_elbow_q = d_j2 * CT_Q - c->beta * d_j1;
    c->joint_dir[2] = sign64(net_elbow_q);

    c->mode = park ? CT_PARK : CT_MOVE;
    c->running = (c->ticks != 0U);
    return c->running ? CT_START : CT_OK;
}

/*
 * [EN] Max derivative of quintic smootherstep `s(u) = 10u^3 - 15u^4 + 6u^5` is `15/8` at `u = 0.5`.
 *      Therefore ramping `v^2(s)` between `end^2` and `peak^2` with max acceleration `accel`
 *      requires `ceil(15 * (peak^2 - end^2) / (16 * accel))` ticks.
 *
 * [VI] Đạo hàm lớn nhất của hàm mượt bậc 5 `s(u) = 10u^3 - 15u^4 + 6u^5` là `15/8` tại `u = 0.5`.
 *      Do đó số xung cần để biến thiên vận tốc từ `end` lên `peak` với gia tốc cực đại `accel`
 *      là `ceil(15 * (peak^2 - end^2) / (16 * accel))`.
 */
uint32_t ct_path_ramp(uint32_t peak, uint32_t end, uint32_t accel)
{
    uint64_t diff_sq = (uint64_t)peak * peak - (uint64_t)end * end;
    uint64_t denom = 16ULL * accel;
    uint64_t num = 15ULL * diff_sq + denom - 1ULL;
    return (uint32_t)(num / denom);
}

/*
 * [EN] Dequeue and start the next segment from `c->path_queue` when the current
 *      segment finishes its final tick. Called directly inside the pulse ISR!
 * [VI] Lấy đoạn quỹ đạo tiếp theo từ hàng đợi `c->path_queue` ra chạy ngay khi
 *      đoạn hiện tại vừa phát xong xung cuối cùng. Hàm chạy trực tiếp trong ngắt!
 */
bool ct_path_advance(struct ct_state *c)
{
    if (!c->path_active) {
        return false;
    }
    if (c->path_started && c->tick < c->ticks) {
        return false;
    }
    if (c->path_started) {
        c->path_done++;
    }

    /*
     * [EN] If all segments of the `PATH` job have finished, mark motion complete.
     * [VI] Nếu đã chạy hết tổng số đoạn của lệnh `PATH`, kết thúc quỹ đạo.
     */
    if (c->path_done == c->path_total) {
        c->path_active = false;
        c->path_started = false;
        c->running = false;
        c->reason = "complete";
        return false;
    }

    /*
     * [EN] If the FIFO ring buffer ran dry before `path_total` was reached, abort with `path_starved`.
     * [VI] Nếu bộ đệm FIFO bị cạn trước khi chạy đủ `path_total` đoạn, báo lỗi `path_starved`.
     */
    if (c->path_size == 0U) {
        if (c->path_starved < UINT32_MAX) {
            c->path_starved++;
        }
        ct_abort(c, "path_starved");
        return false;
    }

    /*
     * [EN] Pop the head segment from the circular queue and configure its velocity ramps.
     * [VI] Lấy đoạn đầu tiên ra khỏi hàng đợi vòng và tính chiều dài đoạn tăng/giảm tốc.
     */
    const struct ct_segment *s = &c->path_queue[c->path_head];
    enum ct_result r = ct_raw_move(c, s->target, s->peak, false);

    c->path_accel = s->accel;
    c->path_entry = s->entry;
    c->path_exit = s->exit;
    c->path_up = ct_path_ramp(s->peak, s->entry, s->accel);
    c->path_down = ct_path_ramp(s->peak, s->exit, s->accel);
    c->path_head = (c->path_head + 1U) % CT_PATH_CAP;
    c->path_size--;
    c->path_started = true;

    if (r != CT_START) {
        ct_abort(c, "path_invalid");
        return false;
    }
    return true;
}

/*
 * [EN] Configure DDA weights and directions for an active calibration leg (`CT_CAL`).
 *      When calibrating `CT_J1`, `CT_J2` is driven simultaneously with weight `|beta|`
 *      to keep the relative elbow angle fixed!
 *
 * [VI] Thiết lập trọng số DDA và chiều quay cho pha dò công tắc (`CT_CAL`).
 *      Đặc biệt: khi dò trục vai `CT_J1`, trục `CT_J2` được cấp trọng số `|beta|` để
 *      chạy đồng bộ giúp giữ nguyên góc khuỷu tay!
 */
static enum ct_result cal_drive(struct ct_state *c)
{
    memset(c->direction, 0, sizeof(c->direction));
    memset(c->cal_weight, 0, sizeof(c->cal_weight));
    memset(c->accum, 0, sizeof(c->accum));

    unsigned a = c->selected;
    if (a == CT_Z) {
        c->direction[a] = -c->cm.direction;
    } else {
        c->direction[a] = c->cm.direction;
    }
    c->cal_weight[a] = CT_Q;
    c->cal_scale = CT_Q;

    /*
     * [EN] When sweeping shoulder J1, drive elbow motor J2 with weight `|beta|`
     *      so the forearm belt doesn't pull the elbow into its limit switch!
     * [VI] Khi quét khớp vai J1, phát xung đồng thời cho động cơ khuỷu J2 với
     *      trọng số `|beta|` để dây đai không kéo khâu 2 đập vào công tắc hành trình!
     */
    if (a == CT_J1) {
        c->direction[2] = c->direction[1] * sign64(c->beta);
        c->cal_weight[2] = llabs(c->beta);
        if (c->cal_weight[2] > c->cal_scale) {
            c->cal_scale = c->cal_weight[2];
        }
    }

    for (unsigned j = 1; j < CT_AXES; j++) {
        if (c->direction[j] != 0) {
            c->last_dir[j] = c->direction[j];
        }
    }

    c->running = (c->cm.direction != 0);
    c->ramp_ms = c->cm.start_ms;
    return c->running ? CT_START : CT_OK;
}

/*
 * [EN] Return true if a switch pair has no conflict and its debounced states match raw GPIO.
 * [VI] Trả về `true` nếu cặp công tắc không bị chập và trạng thái sau lọc khớp với mức GPIO thô.
 */
static bool pair_stable(const struct zj_state *s)
{
    bool both_open = (s->raw_pos != 0) && (s->raw_neg != 0);
    bool top_settled = (s->top == (bool)s->raw_pos);
    bool bottom_settled = (s->bottom == (bool)s->raw_neg);
    return (!both_open) && top_settled && bottom_settled;
}

/*
 * [EN] Pre-transition switch stability gate: ensures axis `a`'s switches are settled
 *      (and completely unpressed if `clear == true`) within `CT_INPUT_WAIT_MS`.
 * [VI] Cổng kiểm tra ổn định công tắc trước khi chuyển bước trong `INIT`: chờ tối đa
 *      `CT_INPUT_WAIT_MS` (200 ms) để công tắc ổn định hoàn toàn.
 */
static bool gate_ready(struct ct_state *c, unsigned a, bool clear, uint32_t now)
{
    const struct zj_state *s = &c->sw[a];
    bool unpressed = (s->raw_pos == 0) && (s->raw_neg == 0);
    if (pair_stable(s) && (!clear || unpressed)) {
        if (c->gate_wait && c->gate_axis == a) {
            c->gate_wait = false;
        }
        return true;
    }
    if (!c->gate_wait || c->gate_axis != a) {
        c->gate_start_ms = now;
        c->gate_axis = a;
        c->gate_wait = true;
    }
    c->reason = "waiting_switch";
    if (now - c->gate_start_ms >= CT_INPUT_WAIT_MS) {
        input_event(c, a, 7, now);
        ct_abort(c, clear ? "j2_guard" : "switch_transition_unstable");
    }
    return false;
}

/*
 * [EN] Verify all axes involved in an internal `CT_PARK` move have stable switches.
 * [VI] Kiểm tra mọi trục tham gia vào bước di chuyển nội bộ `CT_PARK` đều có công tắc ổn định.
 */
static bool park_ready(struct ct_state *c, const int32_t target[CT_AXES], uint32_t now)
{
    for (unsigned a = 0; a < CT_AXES; a++) {
        bool axis_moves = (target[a] != c->pos[a]) || (a == CT_J2 && target[CT_J1] != c->pos[CT_J1]);
        if (axis_moves && !gate_ready(c, a, false, now)) {
            return false;
        }
    }
    c->gate_wait = false;
    return true;
}

/*
 * [EN] Launch single-axis calibration on axis `a` for initialization stage `stage`.
 *      Configures axis-specific speed (`cal_rate`, `fine_rate`), scan limit, and backoff.
 * [VI] Khởi động quy trình dò công tắc hành trình cho trục `a` ở bước `stage`.
 *      Cài đặt tốc độ dò (`cal_rate`, `fine_rate`), giới hạn quét và khoảng lùi riêng cho từng trục.
 */
enum ct_result ct_begin_cal(struct ct_state *c, unsigned a, enum ct_stage stage, uint32_t now)
{
    if (!gate_ready(c, a, false, now) || (a == CT_J1 && !gate_ready(c, CT_J2, true, now))) {
        return c->initializing ? CT_OK : CT_REJECTED;
    }
    c->gate_wait = false;
    c->selected = a;
    c->stage = stage;
    c->mode = CT_CAL;

    c->cm = c->sw[a];
    c->cm.position = 0;
    c->cm.total = 0;
    c->cm.direction = 0;
    /*
     * [EN] Z worker negative endpoint is UP, so it finishes at the upper switch.
     * [VI] Điểm gốc âm của worker trục Z nằm ở phía TRÊN, nên kết thúc tại công tắc trên.
     */
    c->cm.positive_high = (a == CT_Z) ? (!c->pol[a]) : c->pol[a];

    c->cal[a].lease_ms = now;
    uint32_t span = 1000U;
    uint32_t cal_rate = c->arm_rate;
    uint32_t scan_limit = c->arm_scan;
    uint32_t backoff_dist = c->arm_back;

    if (a == CT_Z) {
        c->cal[a].fine_rate = c->z_fine_rate;
        span = 1000U;
        cal_rate = c->z_rate;
        scan_limit = c->z_scan;
        backoff_dist = c->z_back;
    } else if (a == CT_J2) {
        c->cal[a].fine_rate = c->j2_fine_rate;
        span = (uint32_t)(c->high_md[1] - c->low_md[1]);
        cal_rate = c->j2_rate;
    } else {
        c->cal[a].fine_rate = ZJ_MIN_RATE;
        span = (uint32_t)(c->high_md[0] - c->low_md[0]);
        cal_rate = c->arm_rate;
    }

    enum zj_command_result r = hc_start(
        &c->cal[a], &c->cm, span, cal_rate, scan_limit, backoff_dist, c->micro[a], now
    );
    c->cal[a].lease_ms = c->lease_ms;
    c->cm.lease_ms = c->lease_ms;

    if (r == ZJ_REJECTED) {
        ct_abort(c, "cal_start");
        return CT_REJECTED;
    }
    if (c->cm.direction == 0 && (c->cm.reason == ZJ_TOP || c->cm.reason == ZJ_BOTTOM)) {
        c->last_hit_bits = raw_bits(c);
        c->last_hit_ms = now;
    }
    c->reason = "initializing";
    return cal_drive(c);
}

/*
 * [EN] Helper to abort initialization and return `CT_REJECTED`.
 * [VI] Hàm tiện ích hủy quá trình khởi tạo `INIT` và trả về `CT_REJECTED`.
 */
static enum ct_result init_fail(struct ct_state *c, const char *reason)
{
    ct_abort(c, reason);
    return CT_REJECTED;
}

/*
 * [EN] Evaluate whether a paused switch glitch (`c->input_wait`) has settled
 *      cleanly within `CT_INPUT_WAIT_MS` (200 ms) so motion can resume:
 *      - Step 1: Track changes on the watched axes' switch bits (`wait_bits`).
 *      - Step 2: Check if any watched axis still has both switches open (`double_open`).
 *      - Step 3: Fail with `"switch_unstable"` if 200 ms expires without settling.
 *      - Step 4: Wait until switches remain unchanged and not double-open for `ZJ_RELEASE_MS` (20 ms).
 *      - Step 5: If J2 guard triggered and J2 is still pressed after 20 ms, fail with `"j2_guard"`.
 *      - Step 6: Otherwise, glitch has cleared! Restore motion state and resume stepping.
 *
 * [VI] Kiểm tra xem sự kiện nhiễu công tắc đang tạm dừng (`c->input_wait`) đã
 *      ổn định trở lại trong vòng 200 ms chưa để cho phép chạy tiếp:
 *      - Bước 1: Theo dõi các bit công tắc của trục đang chờ (`wait_bits`).
 *      - Bước 2: Kiểm tra xem còn trục nào bị mở đồng thời cả 2 công tắc (`double_open`) không.
 *      - Bước 3: Nếu quá 200 ms chưa ổn định, hủy `INIT` với lỗi `"switch_unstable"`.
 *      - Bước 4: Chờ tín hiệu giữ nguyên trạng thái và hết `double_open` đủ `ZJ_RELEASE_MS` (20 ms).
 *      - Bước 5: Nếu dừng do bảo vệ J2 mà sau 20 ms công tắc J2 vẫn bị nhấn thật, báo lỗi `"j2_guard"`.
 *      - Bước 6: Nếu công tắc đã đóng lại bình thường, khôi phục trạng thái và tiếp tục phát xung!
 */
static enum ct_result stationary_check(struct ct_state *c, uint32_t now)
{
    unsigned mask = 0;
    for (unsigned a = 0; a < CT_AXES; a++) {
        if (c->wait_axes & (1U << a)) {
            mask |= (3U << (2U * a));
        }
    }
    unsigned bits = raw_bits(c) & mask;
    if (bits != c->wait_bits) {
        c->wait_bits = bits;
        c->wait_stable_ms = now;
    }

    bool double_open = false;
    for (unsigned a = 0; a < CT_AXES; a++) {
        if (((bits >> (2U * a)) & 3U) == 3U) {
            double_open = true;
        }
    }

    if (now - c->wait_start_ms >= CT_INPUT_WAIT_MS) {
        c->input_kind = 3;
        return init_fail(c, "switch_unstable");
    }
    if (double_open || (now - c->wait_stable_ms < ZJ_RELEASE_MS)) {
        return CT_OK;
    }
    if (c->wait_guard && (c->sw[CT_J2].raw_pos != 0 || c->sw[CT_J2].raw_neg != 0)) {
        input_event(c, CT_J2, 7, now);
        return init_fail(c, "j2_guard");
    }

    c->input_wait = false;
    if (c->recovered < UINT32_MAX) {
        c->recovered++;
    }
    c->reason = "initializing";
    c->ramp_ms = now;
    c->ramp_tick = c->tick;
    c->running = c->wait_running;

    if (c->mode == CT_CAL) {
        c->cm.direction = c->wait_cm_dir;
        c->cm.reason = c->wait_cm_reason;
        /*
         * [EN] Never inherit a stale HIGH confirmation across the pause.
         * [VI] Không giữ lại trạng thái xác nhận chạm cũ trước khi tạm dừng.
         */
        c->cal[c->selected].settling = false;
        if (c->cm.direction != 0) {
            bool physical_positive = ((c->cm.direction > 0) == c->cm.positive_high);
            bool hit = physical_positive ? (c->cm.raw_pos != 0) : (c->cm.raw_neg != 0);
            if (hit) {
                c->cm.direction = 0;
                c->cm.reason = physical_positive ? ZJ_TOP : ZJ_BOTTOM;
                c->last_hit_bits = raw_bits(c);
                c->last_hit_ms = now;
            }
        }
        if (c->cm.direction == 0) {
            c->running = false;
        }
    }
    return c->running ? CT_START : CT_OK;
}

/*
 * [EN] Main background service state machine called every main-loop iteration:
 *      - Monitors heartbeat lease for pre-loading paths (`path_active && !path_started`).
 *      - Advances the 3-axis `INIT` state machine across all homing & coupling stages:
 *        * `HS_Z`            : Calibrates Z vertical axis -> starts `HS_J2_PRE`.
 *        * `HS_J2_PRE`       : Pre-calibrates J2 elbow span -> moves J2 to midpoint (`HS_J2_MID`).
 *        * `HS_J2_MID`       : Either starts `HS_COUPLE_PROBE` (full init) or jumps to `HS_J1` (quick init).
 *        * `HS_COUPLE_PROBE` : Steps J1 by `probe_pulses` -> starts `HS_COUPLE_SCAN` on J2.
 *        * `HS_COUPLE_SCAN`  : Compares J2 shift `probe_db / probe_da` against `coupling_ppm` -> returns (`HS_COUPLE_RETURN`).
 *        * `HS_COUPLE_RETURN`: Returns J1 & J2 to midpoint -> starts `HS_J1`.
 *        * `HS_J1`           : Calibrates J1 shoulder with J2 belt compensation -> moves J1 to park (`HS_J1_MID`).
 *        * `HS_J1_MID`       : Starts final J2 calibration (`HS_J2_FINAL`) at J1 park angle.
 *        * `HS_J2_FINAL`     : Locks J2 origin and moves all 3 axes to Park pose (`HS_PARK`).
 *        * `HS_PARK`         : Verifies final pose and transitions to `HS_READY` (`referenced = true`).
 *
 * [VI] Máy trạng thái dịch vụ nền được gọi ở mỗi vòng lặp `main`:
 *      - Giám sát kết nối khi đang nạp dữ liệu quỹ đạo (`path_loading`).
 *      - Điều phối toàn bộ các bước của quy trình tự động dò gốc 3 trục (`INIT`):
 *        * `HS_Z`            : Dò gốc trục đứng Z -> chuyển sang `HS_J2_PRE`.
 *        * `HS_J2_PRE`       : Đo sơ bộ hành trình J2 -> đưa J2 ra điểm giữa (`HS_J2_MID`).
 *        * `HS_J2_MID`       : Chuyển sang nhích thử `HS_COUPLE_PROBE` (hoặc nhảy sang `HS_J1` nếu `QUICK`).
 *        * `HS_COUPLE_PROBE` : Nhích thử J1 một đoạn `probe_pulses` -> quét lại J2 (`HS_COUPLE_SCAN`).
 *        * `HS_COUPLE_SCAN`  : Tính tỷ số truyền đai thực tế `probe_db / probe_da` -> quay về điểm giữa (`HS_COUPLE_RETURN`).
 *        * `HS_COUPLE_RETURN`: Đưa J1 & J2 về lại vị trí giữa -> bắt đầu dò gốc `HS_J1`.
 *        * `HS_J1`           : Dò gốc khớp vai J1 (có bù đai J2) -> đưa J1 về góc Park (`HS_J1_MID`).
 *        * `HS_J1_MID`       : Dò gốc chính xác lần cuối cho J2 (`HS_J2_FINAL`) tại vị trí Park của J1.
 *        * `HS_J2_FINAL`     : Chốt gốc tọa độ J2 và đưa cả 3 trục về tư thế Park (`HS_PARK`).
 *        * `HS_PARK`         : Kiểm tra tọa độ cuối và hoàn tất khởi tạo `HS_READY` (`referenced = true`).
 */
enum ct_result ct_service(struct ct_state *c, uint32_t now)
{
    if (c->path_active && !c->path_started) {
        const char *why = ct_inputs(c, now);
        if (why == NULL && (now - c->lease_ms >= CT_LEASE)) {
            why = "heartbeat_lost";
        }
        if (why != NULL) {
            ct_abort(c, why);
            return CT_REJECTED;
        }
        return CT_OK;
    }

    if (!c->initializing) {
        return CT_OK;
    }

    const char *why = ct_inputs(c, now);
    if (why == NULL && (now - c->lease_ms >= CT_LEASE)) {
        why = "heartbeat_lost";
    }
    if (why == NULL && (now - c->start_ms >= CT_INIT_MS)) {
        why = "time_budget";
    }
    if (why != NULL) {
        return init_fail(c, why);
    }

    if (c->input_wait) {
        enum ct_result checked = stationary_check(c, now);
        if (c->input_wait || checked != CT_OK) {
            return checked;
        }
    }

    /*
     * [EN] Service active single-axis calibration (`CT_CAL`).
     * [VI] Cập nhật tiến trình dò công tắc của trục đang hiệu chuẩn (`CT_CAL`).
     */
    if (c->mode == CT_CAL) {
        enum zj_command_result r = hc_tick(&c->cal[c->selected], &c->cm, now);
        if (c->cal[c->selected].phase == HMC_FAILED) {
            if (c->cal[c->selected].error == HMC_UNSTABLE) {
                c->input_axis = (int)c->selected;
                c->input_kind = 5;
                c->input_bits = c->last_hit_bits;
                c->input_at_ms = c->last_hit_ms;
            }
            return init_fail(c, "calibration_failed");
        }
        if (r == ZJ_STARTED) {
            return cal_drive(c);
        }
        if (r == ZJ_RESUMED) {
            /*
             * [EN] Resume same leg after false hit: retain DDA phase, direction, and counts.
             * [VI] Chạy tiếp chặng hiện tại sau khi bỏ qua nhiễu: giữ nguyên pha DDA và bộ đếm.
             */
            c->direction[c->selected] = (c->selected == CT_Z) ? -c->cm.direction : c->cm.direction;
            if (c->selected == CT_J1) {
                c->direction[CT_J2] = c->cm.direction * sign64(c->beta);
            }
            for (unsigned j = 1; j < CT_AXES; j++) {
                if (c->direction[j] != 0) {
                    c->last_dir[j] = c->direction[j];
                }
            }
            c->running = true;
            c->ramp_ms = now;
            c->reason = "initializing";
            if (c->recovered < UINT32_MAX) {
                c->recovered++;
            }
            return CT_START;
        }
        if (hc_active(&c->cal[c->selected])) {
            return CT_OK;
        }

        /*
         * [EN] Calibration of axis `a` completed! Validate measured span and compute origin:
         *      - For Z: if `z_span_um` was given, compute `factor[0] = range[0] * 1024000 / z_span_um`.
         *      - For J1/J2: verify measured angular span matches `(high_md - low_md)` within 5%.
         *      - Compute home origin step coordinate `origin[a]` from the latched negative endpoint.
         *
         * [VI] Đã đo xong hành trình trục `a`! Kiểm tra sai số tỷ lệ và tính tọa độ gốc `origin`:
         *      - Với trục Z: nếu có `z_span_um`, tính hệ số `factor[0] = range[0] * 1024000 / z_span_um`.
         *      - Với J1/J2: kiểm tra góc hành trình đo được không lệch quá 5% so với `(high_md - low_md)`.
         *      - Tính tọa độ bước gốc `origin[a]` từ vị trí chốt công tắc âm.
         */
        unsigned a = c->selected;
        c->range[a] = c->cal[a].pulses;
        uint64_t f = c->nominal[a];
        if (a == CT_Z && c->z_span_um != 0U) {
            f = (uint64_t)c->range[a] * (uint64_t)CT_FACTOR_MDEG_DIV / c->z_span_um;
        }
        uint64_t max_f = (a == CT_Z) ? 100000000ULL : 1000000ULL;
        if (f < 1ULL || f > max_f) {
            return init_fail(c, "scale_bounds");
        }
        if (a != CT_Z) {
            unsigned j = a - 1U;
            int32_t typed = c->high_md[j] - c->low_md[j];
            int32_t span = (int32_t)rounded((int64_t)c->range[a] * CT_FACTOR_MDEG_DIV, (int64_t)f);
            if (abs(span - typed) > typed / 20) {
                return init_fail(c, "scale_mismatch");
            }
            c->span_md[j] = span;
        }
        c->factor[a] = (uint32_t)f;

        if (a == CT_Z) {
            c->origin[a] = c->pos[a] - (int32_t)c->range[a];
        }
        if (a == CT_J1) {
            int64_t j1_low_steps = rounded((int64_t)c->low_md[0] * c->factor[a], CT_FACTOR_MDEG_DIV);
            c->origin[a] = c->pos[a] - (int32_t)j1_low_steps;
        }
        if (c->stage == HS_J2_FINAL) {
            int64_t coupled_park_md = rounded((int64_t)c->park_md[0] * c->coupling_ppm, CT_PPM_UNIT);
            int64_t b_md = c->low_md[1] + coupled_park_md;
            int64_t j2_low_steps = rounded(b_md * c->factor[2], CT_FACTOR_MDEG_DIV);
            c->origin[2] = c->pos[2] - (int32_t)j2_low_steps;
        }
        ct_beta_update(c);
        if (llabs(c->beta) > 64LL * CT_Q) {
            return init_fail(c, "coupling_bounds");
        }
    } else if (c->running) {
        return CT_OK;
    }

    /*
     * [EN] Advance to the next stage of the 3-axis `INIT` state machine.
     * [VI] Chuyển sang bước tiếp theo trong quy trình khởi tạo 3 trục `INIT`.
     */
    int32_t target[CT_AXES];
    memcpy(target, c->pos, sizeof(target));
    enum ct_result result = CT_OK;

    switch (c->stage) {
    case HS_Z:
        /*
         * [EN] Stage 1 finished (Z calibrated at top switch) -> start Stage 2: pre-calibrate J2.
         * [VI] Bước 1 hoàn tất (trục Z đã về công tắc trên) -> sang Bước 2: đo sơ bộ hành trình J2.
         */
        return ct_begin_cal(c, CT_J2, HS_J2_PRE, now);

    case HS_J2_PRE:
        /*
         * [EN] Stage 2 finished (J2 at negative switch) -> move J2 to midpoint (`HS_J2_MID`)
         *      so J2 has maximum clearance on both sides before moving J1.
         * [VI] Bước 2 hoàn tất (J2 đang ở công tắc âm) -> đưa J2 ra chính giữa hành trình (`HS_J2_MID`)
         *      để khuỷu tay có khoảng trống an toàn tối đa về 2 phía trước khi quay J1.
         */
        c->probe_range = c->range[2];
        c->probe_b0 = c->pos[2];
        c->probe_mid = c->pos[2] + (int32_t)c->range[2] / 2;
        target[2] += (int32_t)c->range[2] / 2;
        if (!park_ready(c, target, now)) {
            return c->initializing ? CT_OK : CT_REJECTED;
        }
        c->stage = HS_J2_MID;
        result = ct_raw_move(c, target, 200, true);
        break;

    case HS_J2_MID:
        /*
         * [EN] Stage 3 finished (J2 at midpoint):
         *      - If `auto_coupling` is disabled or `QUICK` init is active, jump straight to `HS_J1`.
         *      - Otherwise, step J1 by `probe_pulses` (`HS_COUPLE_PROBE`) to measure belt coupling.
         * [VI] Bước 3 hoàn tất (J2 đã ở chính giữa):
         *      - Nếu tắt `auto_coupling` hoặc đang chạy `INIT QUICK`, nhảy thẳng sang dò gốc `HS_J1`.
         *      - Ngược lại, nhích thử J1 một đoạn `probe_pulses` (`HS_COUPLE_PROBE`) để đo tỷ số đai.
         */
        if (!c->auto_coupling || c->quick_init) {
            return ct_begin_cal(c, CT_J1, HS_J1, now);
        }
        if (!gate_ready(c, CT_J1, false, now) || !gate_ready(c, CT_J2, true, now)) {
            return CT_OK;
        }
        c->probe_a0 = c->pos[1];
        c->probe_da = 0;
        c->probe_db = 0;
        c->probe_contact = false;
        c->probe_retry = false;
        if (blocked(c, CT_J1, 1)) {
            target[1] -= (int32_t)c->probe_pulses;
        } else {
            target[1] += (int32_t)c->probe_pulses;
        }
        c->stage = HS_COUPLE_PROBE;
        result = ct_raw_move(c, target, 50, true);
        break;

    case HS_COUPLE_PROBE:
        /*
         * [EN] Stage 4a: If the short J1 probe touched a J1 switch, reverse direction once;
         *      otherwise re-scan J2 (`HS_COUPLE_SCAN`) to see how many steps J2's switch shifted!
         * [VI] Bước 4a: Nếu nhích thử J1 chạm công tắc J1 thì đảo chiều nhích lại 1 lần;
         *      khi đã nhích xong, quét lại J2 (`HS_COUPLE_SCAN`) để đo độ lệch vị trí công tắc J2!
         */
        if (c->probe_contact) {
            if (c->probe_retry) {
                return init_fail(c, "coupling_probe_limit");
            }
            c->probe_retry = true;
            c->probe_contact = false;
            int d = c->direction[1];
            target[1] = c->probe_a0 - d * (int32_t)c->probe_pulses;
            if (!park_ready(c, target, now)) {
                c->probe_contact = true;
                c->probe_retry = false;
                return CT_OK;
            }
            result = ct_raw_move(c, target, 50, true);
            break;
        }
        c->probe_da = c->pos[1] - c->probe_a0;
        if (abs(c->probe_da) < 16) {
            return init_fail(c, "coupling_probe_short");
        }
        return ct_begin_cal(c, CT_J2, HS_COUPLE_SCAN, now);

    case HS_COUPLE_SCAN: {
        /*
         * [EN] Stage 4b: Compare the measured belt shift `measured = (probe_db / probe_da) * CT_Q`
         *      against the configured `nominal` ratio, then return J1 & J2 to midpoint (`HS_COUPLE_RETURN`).
         * [VI] Bước 4b: Đối chiếu tỷ số truyền đai thực nghiệm `measured = (probe_db / probe_da) * CT_Q`
         *      với tỷ số danh định `nominal`, sau đó đưa J1 & J2 quay lại điểm giữa (`HS_COUPLE_RETURN`).
         */
        uint32_t old_range = c->probe_range;
        uint32_t new_range = c->range[2];
        if (abs((int)old_range - (int)new_range) > (int)(c->range[2] / 100U + 4U)) {
            return init_fail(c, "coupling_repeat");
        }
        c->probe_db = c->pos[2] - c->probe_b0;
        int64_t measured = rounded((int64_t)c->probe_db * CT_Q * sign64(c->probe_da), abs(c->probe_da));
        c->beta_measured = (int32_t)measured;

        /*
         * [EN] Reject noisy or contradictory coupling measurement before launching J1 sweep.
         * [VI] Từ chối nếu phép đo tỷ số truyền đai bị nhiễu hoặc ngược dấu trước khi quét J1.
         */
        if (llabs(measured) > 64LL * CT_Q || abs(c->probe_db) < 8) {
            c->beta = 0;
            return init_fail(c, "coupling_measurement");
        }
        c->beta = measured;
        if (sign64(measured) != sign64(c->coupling_ppm)) {
            return init_fail(c, "coupling_sign");
        }
        int64_t nominal = rounded(
            (int64_t)c->coupling_ppm * c->nominal[2] * CT_Q,
            CT_PPM_UNIT * c->nominal[1]
        );
        int64_t tol = llabs(nominal) / 20 + (CT_Q + abs(c->probe_da) - 1) / abs(c->probe_da);
        if (llabs(measured - nominal) > tol) {
            return init_fail(c, "coupling_mismatch");
        }
        c->beta = nominal;
        c->coupling_ready = true;
        target[1] = c->probe_a0;
        target[2] = c->probe_mid;
        if (!park_ready(c, target, now)) {
            return CT_OK;
        }
        c->stage = HS_COUPLE_RETURN;
        result = ct_raw_move(c, target, 200, true);
        break;
    }

    case HS_COUPLE_RETURN:
        /*
         * [EN] Stage 4c finished (back at midpoint) -> start Stage 5: calibrate J1 shoulder (`HS_J1`).
         * [VI] Bước 4c hoàn tất (đã về lại điểm giữa) -> sang Bước 5: dò gốc khớp vai J1 (`HS_J1`).
         */
        return ct_begin_cal(c, CT_J1, HS_J1, now);

    case HS_J1: {
        /*
         * [EN] Stage 5 finished (J1 calibrated) -> Stage 6: move J1 to its park angle `park_md[0]`
         *      while compensating J2 by `d_j1 * beta / CT_Q` (`HS_J1_MID`).
         * [VI] Bước 5 hoàn tất (đã dò xong J1) -> Bước 6: đưa J1 về góc `park_md[0]` đồng thời
         *      phát xung bù đai cho J2 một lượng `d_j1 * beta / CT_Q` (`HS_J1_MID`).
         */
        int64_t j1_park_steps = rounded((int64_t)c->park_md[0] * c->factor[1], CT_FACTOR_MDEG_DIV);
        target[1] = c->origin[1] + (int32_t)j1_park_steps;
        int64_t d_j1 = (int64_t)target[1] - c->pos[1];
        target[2] += (int32_t)rounded(d_j1 * c->beta, CT_Q);
        if (!park_ready(c, target, now)) {
            return c->initializing ? CT_OK : CT_REJECTED;
        }
        c->stage = HS_J1_MID;
        result = ct_raw_move(c, target, 400, true);
        break;
    }

    case HS_J1_MID:
        /*
         * [EN] Stage 6 finished (J1 at park angle) -> Stage 7: perform final J2 calibration (`HS_J2_FINAL`).
         * [VI] Bước 6 hoàn tất (J1 đã ở góc Park) -> Bước 7: dò gốc chính xác lần cuối cho J2 (`HS_J2_FINAL`).
         */
        return ct_begin_cal(c, CT_J2, HS_J2_FINAL, now);

    case HS_J2_FINAL: {
        /*
         * [EN] Stage 7 finished (J2 calibrated) -> Stage 8: move J2 to `park_md[1]` and lower Z
         *      from the upper switch by `z_clear_um` clearance (`HS_PARK`).
         * [VI] Bước 7 hoàn tất (đã chốt gốc J2) -> Bước 8: đưa J2 về góc `park_md[1]` và hạ trục Z
         *      từ công tắc trên xuống một khoảng an toàn `z_clear_um` (`HS_PARK`).
         */
        int64_t coupled_park_md = rounded((int64_t)c->park_md[0] * c->coupling_ppm, CT_PPM_UNIT);
        int64_t b_md = c->park_md[1] + coupled_park_md;
        int64_t j2_park_steps = rounded(b_md * c->factor[2], CT_FACTOR_MDEG_DIV);
        target[2] = c->origin[2] + (int32_t)j2_park_steps;

        int64_t clearance = rounded((int64_t)c->z_clear_um * c->factor[0], CT_FACTOR_MDEG_DIV);
        if (clearance < 16 || clearance >= (int64_t)(c->range[0] / 2U)) {
            return init_fail(c, "z_clearance");
        }
        target[0] = c->origin[0] + (int32_t)c->range[0] - (int32_t)clearance;
        if (!park_ready(c, target, now)) {
            return c->initializing ? CT_OK : CT_REJECTED;
        }
        c->stage = HS_PARK;
        result = ct_raw_move(c, target, 800, true);
        break;
    }

    case HS_PARK:
        /*
         * [EN] Stage 8 finished (all 3 axes at Park pose): verify soft limits, save `saved_range`
         *      for future `INIT QUICK` commands, and mark the robot `referenced = true` (`HS_READY`)!
         * [VI] Bước 8 hoàn tất (cả 3 trục đã ở tư thế Park): kiểm tra giới hạn mềm, lưu `saved_range`
         *      phục vụ lệnh `INIT QUICK` về sau, và bật cờ hoàn tất `referenced = true` (`HS_READY`)!
         */
        if (!ct_pose_valid(c, c->pos, 1000)) {
            return init_fail(c, "park_bounds");
        }
        c->initializing = false;
        c->quick_init = false;
        c->has_calibrated = true;
        memcpy(c->saved_range, c->range, sizeof(c->range));
        c->referenced = true;
        c->reference_epoch++;
        c->stage = HS_READY;
        c->mode = CT_IDLE;
        c->reason = "initialized";
        return CT_OK;

    default:
        return init_fail(c, "stage_error");
    }

    return (result == CT_REJECTED) ? init_fail(c, "park_budget") : result;
}

/*
 * [EN] Compute the 3-bit axis step mask (`bit a = 1` if axis `a` emits a pulse on this tick):
 *      1. Backlash Take-Up: If backlash compensation pulses (`comp_left`) remain on J1 or J2,
 *         emit those first without changing the geometric coordinate `c->pos[a]`.
 *      2. Bresenham / DDA Accumulator: Otherwise, add weight `w` (`delta[a]` in move mode,
 *         or `cal_weight[a]` in calibration mode) to `accum[a]`. Whenever `accum[a] >= scale`,
 *         subtract `scale` and set bit `a` in `mask` so all 3 axes reach their target
 *         proportionally on the exact same final tick!
 *      3. Per-Edge Safety Guard: Verify that the resulting `next` step position does not
 *         violate soft limits or run into a triggered limit switch.
 *
 * [VI] Tính mặt nạ 3 bit chọn trục phát xung ở nhịp Timer hiện tại (`bit a = 1` nếu trục `a` phát xung):
 *      1. Bù Độ Rơ Cơ Khí (Backlash): Nếu còn xung bù rơ (`comp_left`) trên J1 hoặc J2, ưu tiên
 *         phát hết xung bù rơ trước (không cộng vào tọa độ hình học `c->pos[a]`).
 *      2. Bộ Tích Lũy Nội Suy DDA / Bresenham: Cộng trọng số `w` (`delta[a]` khi chạy nội suy,
 *         hoặc `cal_weight[a]` khi dò gốc) vào bộ tích lũy `accum[a]`. Mỗi khi `accum[a] >= scale`,
 *         trừ đi `scale` và bật bit `a` trong `mask`, đảm bảo cả 3 trục về đích đồng thời!
 *      3. Bảo Vệ Từng Xung: Kiểm tra tọa độ bước kế tiếp `next` có vượt giới hạn mềm hoặc chạm
 *         công tắc hành trình không trước khi cho phép phát xung.
 */
unsigned ct_next_mask(struct ct_state *c)
{
    unsigned mask = 0;

    /*
     * [EN] 1. Backlash take-up phase before normal segment stepping.
     * [VI] 1. Pha phát xung bù độ rơ cơ khí khi đảo chiều trước khi chạy quỹ đạo chính.
     */
    if (c->mode == CT_MOVE && (c->comp_left[1] != 0U || c->comp_left[2] != 0U)) {
        c->comp_step = true;
        int d1 = (c->comp_left[1] != 0U) ? c->direction[1] : 0;
        int d2 = (c->comp_left[2] != 0U) ? c->direction[2] : 0;
        int64_t net_elbow_q = (int64_t)d2 * CT_Q - c->beta * (int64_t)d1;
        int elbow_edge = sign64(net_elbow_q);
        if (blocked(c, 1, d1) || blocked(c, 2, elbow_edge)) {
            ct_abort(c, "edge_limit");
            return 0;
        }
        for (unsigned a = 1; a < CT_AXES; a++) {
            if (c->comp_left[a] != 0U) {
                mask |= (1U << a);
            }
        }
        return mask;
    }

    /*
     * [EN] 2. Standard multi-axis DDA accumulator step.
     * [VI] 2. Bước cộng dồn bộ nội suy đường thẳng đa trục DDA.
     */
    c->comp_step = false;
    uint64_t scale = (c->mode == CT_CAL) ? (uint64_t)c->cal_scale : c->ticks;
    if (scale == 0ULL) {
        return 0;
    }

    int32_t next[CT_AXES];
    memcpy(next, c->pos, sizeof(next));

    for (unsigned a = 0; a < CT_AXES; a++) {
        uint64_t w = (c->mode == CT_CAL) ? (uint64_t)c->cal_weight[a] : c->delta[a];
        c->accum[a] += w;
        if (c->accum[a] >= scale) {
            c->accum[a] -= scale;
            mask |= (1U << a);
            next[a] += c->direction[a];
        }
    }

    /*
     * [EN] 3. Per-edge soft-limit and limit-switch guard on the resulting step vector `next`.
     * [VI] 3. Kiểm tra giới hạn mềm và công tắc hành trình cho từng bước vi phân `next`.
     */
    if (c->mode == CT_MOVE && !ct_pose_valid(c, next, 0)) {
        ct_abort(c, "soft_limit");
        return 0;
    }
    if (c->mode != CT_CAL) {
        int step_z = sign64((int64_t)next[0] - c->pos[0]);
        int step_j1 = sign64((int64_t)next[1] - c->pos[1]);
        int64_t net_elbow_q = ((int64_t)next[2] - c->pos[2]) * CT_Q -
                              c->beta * ((int64_t)next[1] - c->pos[1]);
        int elbow_edge = sign64(net_elbow_q);

        if (c->stage == HS_COUPLE_PROBE && blocked(c, CT_J1, step_j1)) {
            c->running = false;
            c->probe_contact = true;
            return 0;
        }
        if (blocked(c, 0, step_z) || blocked(c, 1, step_j1) || blocked(c, 2, elbow_edge)) {
            ct_abort(c, "edge_limit");
            return 0;
        }
    }
    return mask;
}

/*
 * [EN] Commit one emitted rising pulse on axis `a` to the position & total counters.
 * [VI] Cập nhật tọa độ bước `pos[a]` và tổng xung `total[a]` sau khi phát 1 xung trên trục `a`.
 */
void ct_count(struct ct_state *c, unsigned a)
{
    if (c->comp_step) {
        if (c->comp_left[a] != 0U) {
            c->comp_left[a]--;
        }
        c->comp_total[a]++;
        c->total[a]++;
        return;
    }
    c->pos[a] += c->direction[a];
    c->total[a]++;
    if (c->mode == CT_CAL && a == c->selected) {
        zj_count_pulse(&c->cm);
    }
}

/*
 * [EN] Finalize the current DDA tick after all active axes have emitted their HIGH edge.
 * [VI] Kết thúc một nhịp nội suy DDA sau khi tất cả các trục cần phát xung đã kích sườn lên.
 */
void ct_finish_tick(struct ct_state *c)
{
    if (c->comp_step) {
        c->comp_step = false;
        return;
    }
    if (c->mode == CT_CAL) {
        if (c->cm.direction == 0) {
            c->running = false;
        }
        return;
    }
    c->tick++;
    if (c->tick >= c->ticks && !c->path_active) {
        c->running = false;
        c->reason = c->initializing ? "initializing" : "complete";
    }
}

/*
 * [EN] Exact integer square root `floor(sqrt(n))` using bitwise digit-by-digit algorithm:
 *      - Works in base 4 (`bit` starts at `4^15 = 1 << 30` and shifts right by 2 each step).
 *      - At each step, tests whether `(result + bit)` can be subtracted from the remainder `n`.
 *      - Used inside the ISR to compute velocity `v = sqrt(v^2(s))` in ~15 CPU cycles without FPU!
 *
 * [VI] Thuật toán khai căn bậc hai số nguyên chính xác `floor(sqrt(n))` bằng dịch bit:
 *      - Tính từng chữ số hệ cơ số 4 (`bit` bắt đầu từ `4^15 = 1 << 30`, mỗi vòng dịch phải 2 bit).
 *      - Ở mỗi bước, kiểm tra phần dư `n` có đủ lớn để trừ đi `(result + bit)` hay không.
 *      - Dùng trong ngắt ISR để tính vận tốc `v = sqrt(v^2(s))` cực nhanh mà không cần bộ FPU!
 */
static uint32_t root32(uint32_t n)
{
    uint32_t result = 0;
    uint32_t bit = 1U << 30;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (n >= result + bit) {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

/*
 * [EN] 20-bit fixed-point linear interpolation: `a + ((b - a) * u) >> 20` for `0 <= u <= 2^20`.
 * [VI] Nội suy tuyến tính dấu phẩy tĩnh 20-bit: `a + ((b - a) * u) >> 20` với `0 <= u <= 2^20`.
 */
static uint32_t blend20(uint32_t a, uint32_t b, uint32_t u)
{
    return a + (uint32_t)(((uint64_t)(b - a) * u) >> 20);
}

/*
 * [EN] Evaluate squared velocity `v^2(tick)` along a quintic smootherstep ramp
 *      from `low^2` to `high^2` over `length` ticks using monotone de Casteljau
 *      subdivision on Bernstein control points `[0, 0, 0, 1, 1, 1]`:
 *      - Why de Casteljau? Evaluating `10u^3 - 15u^4 + 6u^5` directly has a negative
 *        term (`-15u^4`), whereas blending `[0, 0, 0, 1, 1, 1]` uses ONLY non-negative
 *        convex combinations `blend20(x, y, u)` with `x <= y`, guaranteeing zero overflow!
 *
 * [VI] Tính bình phương vận tốc `v^2(tick)` dọc theo đường cong bậc 5 mượt (quintic
 *      smootherstep) từ `low^2` tới `high^2` trên chiều dài `length` xung bằng thuật
 *      toán chia đôi de Casteljau trên các điểm kiểm soát Bernstein `[0, 0, 0, 1, 1, 1]`:
 *      - Tại sao dùng de Casteljau? Vì tính trực tiếp `10u^3 - 15u^4 + 6u^5` có hạng tử
 *        âm (`-15u^4`), trong khi nội suy tuyến tính liên tiếp trên `[0, 0, 0, 1, 1, 1]`
 *        chỉ toàn phép cộng số không âm tăng dần, tuyệt đối không bao giờ bị tràn số!
 */
static uint32_t smooth_square(uint32_t low, uint32_t high, uint32_t tick, uint32_t length)
{
    if (length == 0U || tick >= length || low == high) {
        return high * high;
    }
    if (tick == 0U) {
        return low * low;
    }
    /* [EN] Normalized progress `u` in Q20 `[0 .. 1048576]`. / [VI] Tiến độ chuẩn hóa `u` hệ Q20 `[0 .. 1048576]`. */
    uint32_t u = (uint32_t)((uint64_t)tick * 1048576U / length);
    uint32_t p = blend20(0, u, u);
    uint32_t q = blend20(u, 1048576U, u);
    uint32_t a = blend20(0, p, u);
    uint32_t b = blend20(p, q, u);
    uint32_t d = blend20(q, 1048576U, u);
    uint32_t s = blend20(blend20(a, b, u), blend20(b, d, u), u);
    return low * low + (uint32_t)(((uint64_t)(high * high - low * low) * s) >> 20);
}

/*
 * [EN] Compute the pulse period (in microseconds) for the current tick:
 *      - Case 1 (Backlash take-up): Runs at a gentle 200 pulses/s (5000 us period).
 *      - Case 2 (Calibration `CT_CAL`): Uses a time-based linear ramp over `ZJ_RAMP_MS`,
 *        plus a 256-pulse soft deceleration window before expected limit-switch contact.
 *      - Case 3 (Continuous path `path_active`): Evaluates quintic smootherstep `v^2`
 *        on the entry and exit ramps and takes `rate = root32(min(up, down))`.
 *      - Case 4 (Point-to-point `MOVE`/`CT_PARK`): Uses a 200-tick trapezoidal ramp.
 *
 * [VI] Tính chu kỳ phát xung (đơn vị micro-giây) tại nhịp hiện tại:
 *      - Trường hợp 1 (Bù độ rơ): Chạy ở tốc độ êm 200 xung/giây (chu kỳ 5000 us).
 *      - Trường hợp 2 (Dò gốc `CT_CAL`): Tăng tốc tuyến tính theo thời gian trong `ZJ_RAMP_MS`,
 *        kèm cửa sổ giảm tốc mềm 256 xung trước khi chạm công tắc hành trình.
 *      - Trường hợp 3 (Vẽ quỹ đạo `path_active`): Tính bình phương vận tốc bậc 5 ở đầu/cuối đoạn
 *        rồi khai căn `rate = root32(min(up, down))`.
 *      - Trường hợp 4 (Chạy điểm-tới-điểm `MOVE`/`CT_PARK`): Dùng đường cong hình thang 200 xung.
 */
uint32_t ct_period_us(const struct ct_state *c, uint32_t now)
{
    /* [EN] Case 1: Backlash take-up steps. / [VI] Trường hợp 1: Phát xung bù độ rơ. */
    if (c->mode == CT_MOVE && (c->comp_left[1] != 0U || c->comp_left[2] != 0U)) {
        return 1000000U / 200U;
    }

    uint32_t rate = 50U;
    if (c->mode == CT_CAL) {
        /* [EN] Case 2: Calibration velocity profile. / [VI] Trường hợp 2: Quy hoạch vận tốc khi dò gốc. */
        uint32_t elapsed = now - c->cm.start_ms;
        uint32_t restarted = now - c->ramp_ms;
        if (restarted < elapsed) {
            elapsed = restarted;
        }
        elapsed = (elapsed > ZJ_SETUP_MS) ? (elapsed - ZJ_SETUP_MS) : 0U;
        if (elapsed > ZJ_RAMP_MS) {
            elapsed = ZJ_RAMP_MS;
        }
        uint32_t initial = c->cal[c->selected].fine_rate;
        if (initial > c->cm.rate) {
            initial = c->cm.rate;
        }
        uint32_t primary = initial + (c->cm.rate - initial) * elapsed / ZJ_RAMP_MS;

        /*
         * [EN] Soft deceleration over the final 256 pulses before an expected limit switch
         *      contact (`n1` during `HMC_SWEEP_N`, or `probe_range` on J2 `HMC_SWEEP_P`)
         *      so the axis contacts the switch gently at `fine_rate`.
         * [VI] Giảm tốc mềm trong 256 xung cuối trước khi chạm công tắc hành trình tại
         *      vị trí dự kiến (`n1` ở pha `HMC_SWEEP_N`, hoặc `probe_range` của J2 ở pha
         *      `HMC_SWEEP_P`), giúp cơ cấu chạm công tắc êm ái ở tốc độ `fine_rate`.
         */
        enum hc_phase phase = c->cal[c->selected].phase;
        uint32_t expected_span = 0U;
        if (phase == HMC_SWEEP_N) {
            expected_span = c->cal[c->selected].n1;
        } else if (phase == HMC_SWEEP_P && c->selected == CT_J2) {
            expected_span = c->probe_range;
        }
        if (expected_span >= 512U && c->cm.move + 256U > expected_span) {
            uint32_t rem_pulses = (expected_span > c->cm.move) ? (expected_span - c->cm.move) : 0U;
            uint32_t decel = initial + (c->cm.rate - initial) * rem_pulses / 256U;
            if (primary > decel) {
                primary = decel;
            }
        }
        rate = (uint32_t)((uint64_t)primary * c->cal_scale / CT_Q);

    } else if (c->path_active && c->path_started) {
        /* [EN] Case 3: Continuous path quintic smootherstep profile. / [VI] Trường hợp 3: Vẽ quỹ đạo bậc 5. */
        if (c->tick >= c->path_up && (c->ticks - c->tick) >= c->path_down) {
            rate = c->rate;
        } else {
            uint32_t up = smooth_square(c->path_entry, c->rate, c->tick, c->path_up);
            uint32_t down = smooth_square(c->path_exit, c->rate, c->ticks - c->tick, c->path_down);
            uint32_t square = (up < down) ? up : down;
            rate = root32(square);
        }

    } else {
        /* [EN] Case 4: Point-to-point 200-tick trapezoidal ramp. / [VI] Trường hợp 4: Chạy điểm-tới-điểm hình thang. */
        uint32_t since_restart = c->tick - c->ramp_tick;
        uint32_t rem = c->ticks - c->tick;
        uint32_t distance = (since_restart < rem) ? since_restart : rem;
        uint32_t ramp = (distance > 200U) ? 200U : distance;
        uint32_t initial = (c->delta[1] == 0U && c->delta[2] == 0U) ? c->z_fine_rate : 50U;
        if (initial > c->rate) {
            initial = c->rate;
        }
        rate = initial + (c->rate - initial) * ramp / 200U;
    }

    if (rate < 50U) {
        rate = 50U;
    }
    bool fast_z = (c->mode == CT_CAL && c->selected == CT_Z) ||
                  (c->mode == CT_MOVE && !c->path_active && c->delta[1] == 0U && c->delta[2] == 0U);
    uint32_t ceiling = fast_z ? CT_Z_CAL_RATE : CT_MAX_RATE;
    if (rate > ceiling) {
        rate = ceiling;
    }

    if (c->mode == CT_CAL && c->selected == CT_Z) {
        return (1000000U + rate - 1U) / rate;
    }
    return 1000000U / rate;
}
