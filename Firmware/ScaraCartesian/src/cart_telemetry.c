/*
 * ============================================================================
 * FILE: cart_telemetry.c
 * ============================================================================
 * [EN] Telemetry Snapshot Capture and Protocol-V5 JSON Serialization
 *      Converts internal firmware state into the JSON status and ACK packets
 *      consumed by the Python GUI (`cartesian_nc_protocol.py`).
 *
 * [VI] Chụp Ảnh Trạng thái & Đóng gói Chuỗi JSON Giao thức V5
 *      Chuyển đổi các biến trạng thái nội bộ của Firmware thành gói tin JSON
 *      `status` (mỗi 100 ms) và `ack` (sau mỗi lệnh) gửi lên phần mềm Python.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - In `ct_status_capture`, reported positions `s->p[a]` and goals `s->g[a]`
 *   are relative to the home origin (`c->pos[a] - c->origin[a]`).
 * - Bitmasks (`switches`, `raw`, `ready`, `holding`, `pol`, `conflicts`) pack
 *   all 3 axes into a single integer so students can inspect every switch wire
 *   directly from the JSON log!
 * - Trong hàm `ct_status_capture`, tọa độ báo về `s->p[a]` đã được trừ đi điểm
 *   gốc `c->origin[a]`. Các trường mặt nạ bit (`switches`, `raw`, `ready`, ...)
 *   nén trạng thái của cả 3 trục vào một số nguyên giúp dễ dàng kiểm tra dây
 *   công tắc từ cửa sổ Log trên máy tính.
 * ============================================================================
 */

#include "cart_telemetry.h"
#include <stdio.h>
#include <string.h>

/*
 * [EN] Copy fields from `c` into `s` quickly while interrupts are locked:
 *      - Step 1: Copy session, job, tick progress, and path FIFO credit counters.
 *      - Step 2: Copy high-level state flags (`busy`, `referenced`, `stage`, `reason`)
 *        and switch glitch recovery diagnostics.
 *      - Step 3: Compute instantaneous step rate `pps` and copy belt coupling values.
 *      - Step 4: Loop over axes `0..2` (`Z`, `J1`, `J2`) to compute home-relative step
 *        positions and pack per-axis switch/driver flags into compact bitmasks.
 *
 * [VI] Sao chép nhanh các trường dữ liệu từ `c` sang `s` trong khi đang khóa ngắt:
 *      - Bước 1: Sao chép mã `session`, `job`, tiến độ xung `tick`/`ticks` và bộ đếm FIFO `path`.
 *      - Bước 2: Sao chép các cờ trạng thái tổng quát (`busy`, `referenced`, `stage`, `reason`)
 *        và thông tin chẩn đoán chống nhiễu công tắc.
 *      - Bước 3: Tính tốc độ xung tức thời `pps` và sao chép các thông số bù dây đai.
 *      - Bước 4: Duyệt qua 3 trục (`Z`, `J1`, `J2`) để tính tọa độ bước tương đối so với gốc
 *        và đóng gói trạng thái công tắc/driver vào các mặt nạ bit (`mask`, `raw`, `ready`, ...).
 */
void ct_status_capture(
    struct ct_status *s,
    const struct ct_state *c,
    bool timer_running,
    uint32_t timer_late,
    uint32_t now
)
{
    memset(s, 0, sizeof(*s));

    /*
     * [EN] Step 1: Session, job tracking, and path FIFO credit counters.
     * [VI] Bước 1: Thông tin phiên kết nối, mã công việc và bộ đếm hàng đợi quỹ đạo.
     */
    s->session = c->session;
    s->job = c->job;
    s->epoch = c->reference_epoch;
    s->tick = c->tick;
    s->ticks = c->ticks;
    s->late = timer_late;

    s->path[0] = c->path_total;
    s->path[1] = c->path_received;
    s->path[2] = c->path_done;
    s->path[3] = CT_PATH_CAP - c->path_size; /* [EN] Available FIFO slots (credits) / [VI] Số ô trống còn lại trong FIFO */
    s->starved = c->path_starved;

    /*
     * [EN] Step 2: High-level controller state and switch recovery diagnostics.
     * [VI] Bước 2: Trạng thái hoạt động tổng quát và thông tin chẩn đoán công tắc.
     */
    s->busy = ct_busy(c) || timer_running;
    s->referenced = c->referenced;
    s->fault = c->fault;
    s->stage = c->stage;
    s->reason = c->reason;

    s->recovered = c->recovered;
    s->input_wait = c->input_wait;
    s->idle_ignored = c->idle_ignored;
    s->gate_wait = c->gate_wait;
    s->gate_axis = c->gate_axis;
    s->input_axis = c->input_axis;
    s->input_kind = c->input_kind;
    s->input_bits = c->input_bits;
    s->input_at = c->input_at_ms;

    /*
     * [EN] Step 3: Motion mode, instantaneous pulse rate (pps), and belt coupling parameters.
     * [VI] Bước 3: Chế độ chạy, tần số xung tức thời (pps) và các hệ số truyền động đai.
     */
    s->mode = c->mode;
    s->selected = c->selected;
    if (c->running) {
        s->pps = 1000000U / ct_period_us(c, now);
    } else {
        s->pps = 0U;
    }

    s->coupling_ppm = c->coupling_ppm;
    s->beta_q = (int32_t)c->beta;
    s->beta_measured = c->beta_measured;
    s->span_md[0] = c->span_md[0];
    s->span_md[1] = c->span_md[1];
    s->comp_pulses[0] = c->comp_total[1];
    s->comp_pulses[1] = c->comp_total[2];
    s->probe_da = c->probe_da;
    s->probe_db = c->probe_db;
    s->coupling_ready = c->coupling_ready;

    /*
     * [EN] Step 4: Per-axis coordinates, calibration results, and packed bitmasks:
     *      - `raw` and `mask` use 2 bits per axis: `bit (2*a)` = positive switch, `bit (2*a + 1)` = negative switch.
     *      - `io_errors`, `ready`, `hold`, `pol`, `conflicts` use 1 bit per axis (`bit a`).
     * [VI] Bước 4: Tọa độ từng trục, kết quả đo hành trình và các mặt nạ bit trạng thái:
     *      - `raw` và `mask` dùng 2 bit mỗi trục: `bit (2*a)` = công tắc dương, `bit (2*a + 1)` = công tắc âm.
     *      - `io_errors`, `ready`, `hold`, `pol`, `conflicts` dùng 1 bit mỗi trục (`bit a`).
     */
    for (unsigned a = 0; a < CT_AXES; a++) {
        s->p[a] = c->pos[a] - c->origin[a];
        s->g[a] = c->goal[a] - c->origin[a];
        s->total[a] = c->total[a];
        s->brief[a] = c->sw[a].brief_both;
        s->false_hits[a] = c->cal[a].false_hits;

        unsigned io_err_bit = (c->sw[a].error != 0) ? 1U : 0U;
        s->io_errors |= (io_err_bit << a);

        unsigned raw_pos_bit = (c->sw[a].raw_pos != 0) ? 1U : 0U;
        unsigned raw_neg_bit = (c->sw[a].raw_neg != 0) ? 2U : 0U;
        s->raw |= ((raw_pos_bit | raw_neg_bit) << (2U * a));

        s->range[a] = c->range[a];
        s->f[a] = c->factor[a];
        s->n1[a] = c->cal[a].n1;
        s->n2[a] = c->cal[a].n2;
        s->phases[a] = c->cal[a].phase;
        s->errors[a] = c->cal[a].error;
        s->failed[a] = c->cal[a].failed_phase;

        unsigned pos_active = (c->sw[a].top || c->sw[a].raw_pos != 0) ? 1U : 0U;
        unsigned neg_active = (c->sw[a].bottom || c->sw[a].raw_neg != 0) ? 1U : 0U;
        s->mask |= (pos_active << (2U * a));
        s->mask |= (neg_active << (2U * a + 1U));

        unsigned ready_bit = zj_ready(&c->sw[a]) ? 1U : 0U;
        unsigned hold_bit = c->holding[a] ? 1U : 0U;
        unsigned pol_bit = c->pol[a] ? 1U : 0U;
        unsigned conflict_bit = c->sw[a].conflict ? 1U : 0U;

        s->ready |= (ready_bit << a);
        s->hold |= (hold_bit << a);
        s->pol |= (pol_bit << a);
        s->direction[a] = c->direction[a];
        s->conflicts |= (conflict_bit << a);
    }
}

/*
 * [EN] Serialize `struct ct_status` into a single-line JSON status message.
 * [VI] Đóng gói cấu trúc `struct ct_status` thành chuỗi JSON một dòng gửi qua USB.
 */
int ct_status_format(char *output, size_t capacity, const struct ct_status *s, uint32_t now)
{
    return snprintf(
        output,
        capacity,
        "{\"type\":\"status\",\"protocol\":5,\"fw\":\"SCARA_CARTESIAN_NC_V5\",\"build\":\"SCARA_CART_NC_HOME_V5_R9\","
        "\"session\":%lu,\"job\":%lu,\"up_ms\":%lu,\"busy\":%d,\"referenced\":%d,\"epoch\":%lu,\"stage\":%u,"
        "\"pos\":[%ld,%ld,%ld],\"goal\":[%ld,%ld,%ld],\"total\":[%llu,%llu,%llu],\"range\":[%lu,%lu,%lu],\"factor\":[%lu,%lu,%lu],"
        "\"n1\":[%lu,%lu,%lu],\"n2\":[%lu,%lu,%lu],\"phase\":[%u,%u,%u],\"cal_error\":[%u,%u,%u],\"failed_phase\":[%u,%u,%u],"
        "\"switches\":%u,\"ready\":%u,\"holding\":%u,\"pol\":%u,\"conflicts\":%u,\"tick\":%lu,\"ticks\":%lu,"
        "\"raw\":%u,\"errors\":%u,\"input_wait\":%d,\"recovered\":%lu,\"input_axis\":%d,\"input_kind\":%u,\"input_bits\":%u,\"input_at\":%lu,\"brief\":[%lu,%lu,%lu],"
        "\"idle_ignored\":%lu,\"gate_wait\":%d,\"gate_axis\":%u,\"false_hits\":[%lu,%lu,%lu],"
        "\"mode\":%u,\"selected\":%u,\"pps\":%u,\"direction\":[%d,%d,%d],\"dir_levels\":%u,"
        "\"coupling_ppm\":%ld,\"beta_q\":%ld,\"beta_measured\":%ld,\"probe_da\":%ld,\"probe_db\":%ld,\"coupling_ready\":%d,"
        "\"span_md\":[%ld,%ld],\"comp_pulses\":[%lu,%lu],"
        "\"path\":[%u,%u,%u,%u],\"starved\":%lu,"
        "\"motor_error\":%d,\"fault\":%d,\"timer_late\":%lu,\"timer_errno\":%d,\"reason\":\"%s\"}\n",
        (unsigned long)s->session,
        (unsigned long)s->job,
        (unsigned long)now,
        s->busy,
        s->referenced,
        (unsigned long)s->epoch,
        (unsigned)s->stage,
        (long)s->p[0], (long)s->p[1], (long)s->p[2],
        (long)s->g[0], (long)s->g[1], (long)s->g[2],
        (unsigned long long)s->total[0], (unsigned long long)s->total[1], (unsigned long long)s->total[2],
        (unsigned long)s->range[0], (unsigned long)s->range[1], (unsigned long)s->range[2],
        (unsigned long)s->f[0], (unsigned long)s->f[1], (unsigned long)s->f[2],
        (unsigned long)s->n1[0], (unsigned long)s->n1[1], (unsigned long)s->n1[2],
        (unsigned long)s->n2[0], (unsigned long)s->n2[1], (unsigned long)s->n2[2],
        s->phases[0], s->phases[1], s->phases[2],
        s->errors[0], s->errors[1], s->errors[2],
        s->failed[0], s->failed[1], s->failed[2],
        s->mask, s->ready, s->hold, s->pol, s->conflicts,
        (unsigned long)s->tick, (unsigned long)s->ticks,
        s->raw, s->io_errors, s->input_wait, (unsigned long)s->recovered,
        s->input_axis, s->input_kind, s->input_bits, (unsigned long)s->input_at,
        (unsigned long)s->brief[0], (unsigned long)s->brief[1], (unsigned long)s->brief[2],
        (unsigned long)s->idle_ignored, s->gate_wait, s->gate_axis,
        (unsigned long)s->false_hits[0], (unsigned long)s->false_hits[1], (unsigned long)s->false_hits[2],
        s->mode, s->selected, s->pps,
        s->direction[0], s->direction[1], s->direction[2], s->dir_levels,
        (long)s->coupling_ppm, (long)s->beta_q, (long)s->beta_measured,
        (long)s->probe_da, (long)s->probe_db, s->coupling_ready,
        (long)s->span_md[0], (long)s->span_md[1],
        (unsigned long)s->comp_pulses[0], (unsigned long)s->comp_pulses[1],
        s->path[0], s->path[1], s->path[2], s->path[3], (unsigned long)s->starved,
        s->motor_error, s->fault, (unsigned long)s->late, s->timer_error, s->reason
    );
}

/*
 * [EN] Serialize a command acknowledgment JSON line (`{"type":"ack",...}`).
 * [VI] Đóng gói bản tin JSON xác nhận lệnh (`{"type":"ack",...}`).
 */
int ct_ack_format(
    char *output,
    size_t capacity,
    uint32_t session,
    uint32_t job,
    const char *operation,
    bool ok,
    const char *reason
)
{
    return snprintf(
        output,
        capacity,
        "{\"type\":\"ack\",\"protocol\":5,\"session\":%lu,\"job\":%lu,\"op\":\"%s\",\"ok\":%d,\"reason\":\"%s\"}\n",
        (unsigned long)session,
        (unsigned long)job,
        operation,
        ok,
        reason
    );
}
