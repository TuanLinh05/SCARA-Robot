/*
 * ============================================================================
 * FILE: cart_commands.c
 * ============================================================================
 * [EN] Protocol-V5 Command Parser & Dispatcher for 3-Axis SCARA Firmware
 *      Parses ASCII commands sent by the PC host (`STOP`, `STATUS`, `HELLO`,
 *      `KEEP`, `PATH`, `SEG`, `GO`, `HOLD`, `POL`, `COUPLE`, `BACKLASH`,
 *      `GEOM`, `SPAN`, `PARK`, `TUNE`, `INIT`, `MOVE`), validates all safety
 *      preconditions and parameter ranges, and updates `struct ct_state`.
 *
 * [VI] Bộ Giải mã & Điều phối Lệnh Giao thức V5 cho Firmware SCARA 3 Trục
 *      Phân tích các câu lệnh ASCII từ máy tính gửi xuống (`STOP`, `STATUS`,
 *      `HELLO`, `KEEP`, `PATH`, `SEG`, `GO`, `HOLD`, `POL`, `COUPLE`, `BACKLASH`,
 *      `GEOM`, `SPAN`, `PARK`, `TUNE`, `INIT`, `MOVE`), kiểm tra chặt chẽ các
 *      điều kiện an toàn và giới hạn tham số trước khi cập nhật `struct ct_state`.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - Every rejected command sets `c->reply` to a specific diagnostic token
 *   (e.g., `"session"`, `"job"`, `"busy"`, `"unreferenced"`, `"soft_limit"`,
 *   `"path_profile"`, `"configuration"`). Check the `"reason"` field in the
 *   JSON ACK packet on the PC to see exactly which guard triggered!
 * - Mọi lệnh bị từ chối đều gán mã lỗi cụ thể vào `c->reply` (ví dụ: `"session"`,
 *   `"job"`, `"busy"`, `"unreferenced"`, `"soft_limit"`, `"configuration"`).
 *   Khi gỡ lỗi, hãy xem trường `"reason"` trong gói tin ACK để biết chính xác
 *   điều kiện nào đã chặn lệnh!
 * ============================================================================
 */

#include "cart_core_internal.h"
#include "command_text.h"
#include <stdlib.h>
#include <string.h>

/*
 * [EN] Helper to reject a command: stores the diagnostic reason in `c->reply`,
 *      aborts any active motion if the controller is currently busy, and
 *      returns `CT_REJECTED`.
 * [VI] Hàm từ chối lệnh: lưu nguyên nhân lỗi vào `c->reply`, đồng thời hủy
 *      chuyển động đang chạy (nếu có) để đảm bảo an toàn tuyệt đối.
 */
static enum ct_result reject(struct ct_state *c, const char *why)
{
    c->reply = why;
    if (ct_busy(c)) {
        ct_abort(c, why);
    }
    return CT_REJECTED;
}

/*
 * [EN] Callback invoked by `main.c` after toggling a motor driver's ENA pin
 *      for a `HOLD` command. If GPIO verification failed, latches `motor_fault`.
 * [VI] Hàm được `main.c` gọi sau khi đóng/ngắt chân ENA của driver động cơ
 *      theo lệnh `HOLD`. Nếu chân GPIO không đổi mức đúng yêu cầu, khóa lỗi `motor_fault`.
 */
void ct_hold_complete(struct ct_state *c, bool success)
{
    if (success) {
        c->holding[c->hold_axis] = c->hold_value;
    } else {
        c->fault = true;
        ct_abort(c, "motor_fault");
    }
}

/*
 * [EN] Main command entry point. Tokenizes `line` and dispatches by verb `w[0]`.
 * [VI] Hàm xử lý lệnh chính: tách từ dòng lệnh `line` và phân nhánh theo từ khóa `w[0]`.
 */
enum ct_result ct_command(struct ct_state *c, const char *line, uint32_t now)
{
    char buffer[192];
    char *w[13];
    unsigned n = 0;

    if (!command_split(line, buffer, sizeof(buffer), w, 13, &n, false)) {
        return reject(c, "invalid_command");
    }
    c->reply = "ok";

    /* ========================================================================
     * 1. STOP [REASON_TOKEN]
     * [EN] Immediately halts any active motion or calibration without requiring
     *      a matching session ID (safety-first emergency stop).
     * [VI] Dừng khẩn cấp mọi chuyển động hoặc quá trình dò gốc ngay lập tức
     *      mà không cần kiểm tra mã phiên `session` (ưu tiên an toàn tối đa).
     * ======================================================================== */
    if (strcmp(w[0], "STOP") == 0) {
        const char *why = "stopped";
        if (n == 2U) {
            static const char *const tokens[] = {
                "USER", "ESC", "FOCUS", "MINIMIZE", "DISCONNECT",
                "CLOSE", "STALE", "ACK", "MODEL"
            };
            static const char *const reasons[] = {
                "stop_user", "stop_escape", "stop_focus", "stop_minimize", "stop_disconnect",
                "stop_close", "stop_stale", "stop_ack_timeout", "stop_model"
            };
            bool found = false;
            unsigned count = (unsigned)(sizeof(tokens) / sizeof(tokens[0]));
            for (unsigned i = 0; i < count; i++) {
                if (strcmp(w[1], tokens[i]) == 0) {
                    why = reasons[i];
                    found = true;
                    break;
                }
            }
            if (!found) {
                return reject(c, "invalid_command");
            }
        } else if (n != 1U) {
            return reject(c, "invalid_command");
        }

        if (ct_busy(c)) {
            ct_abort(c, why);
        } else if (n == 2U || strncmp(c->reason, "stop_", 5) != 0) {
            c->reason = why;
        }
        return CT_OK;
    }

    /* ========================================================================
     * 2. STATUS
     * [EN] No-op command that returns an ACK; status broadcast happens periodically.
     * [VI] Lệnh kiểm tra kết nối trả về ACK; bản tin trạng thái vẫn gửi định kỳ.
     * ======================================================================== */
    if (n == 1U && strcmp(w[0], "STATUS") == 0) {
        return CT_OK;
    }

    /*
     * [EN] All remaining commands require a valid session ID in `w[1]`.
     * [VI] Tất cả các lệnh bên dưới đều bắt buộc có mã phiên `session` hợp lệ ở `w[1]`.
     */
    int64_t v[14] = {0};
    if (n < 2U || !command_number_i64(w[1], 1, UINT32_MAX, &v[1])) {
        return reject(c, "session");
    }

    /* ========================================================================
     * 3. HELLO <session>
     * [EN] Establishes a new host session and resets job sequence counters.
     * [VI] Thiết lập phiên làm việc mới với máy tính và đặt lại bộ đếm `job`.
     * ======================================================================== */
    if (strcmp(w[0], "HELLO") == 0 && n == 2U) {
        if (ct_busy(c)) {
            ct_abort(c, "session_changed");
        }
        c->session = (uint32_t)v[1];
        c->last_job = 0;
        c->job = 0;
        return CT_OK;
    }

    /*
     * [EN] Reject any command whose session ID does not match the active session.
     * [VI] Từ chối mọi lệnh có mã `session` không khớp với phiên hiện tại.
     */
    if (c->session == 0U || (uint32_t)v[1] != c->session) {
        return reject(c, "session");
    }

    /* ========================================================================
     * 4. KEEP <session> <job>
     * [EN] Renews the 350 ms watchdog lease (`lease_ms`) for the active job.
     * [VI] Gia hạn thời gian chờ watchdog 350 ms (`lease_ms`) cho công việc đang chạy.
     * ======================================================================== */
    if (strcmp(w[0], "KEEP") == 0 && n == 3U) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] != c->job) {
            return reject(c, "job");
        }
        c->lease_ms = now;
        if (c->initializing && c->mode == CT_CAL) {
            c->cm.lease_ms = now;
            c->cal[c->selected].lease_ms = now;
        }
        return CT_OK;
    }

    /* ========================================================================
     * 5. PATH <session> <job> <total_segments>
     * [EN] Opens a continuous-path drawing job of `total_segments` (1..500).
     * [VI] Mở một công việc chạy quỹ đạo liên tục gồm `total_segments` đoạn (1..500).
     * ======================================================================== */
    if (strcmp(w[0], "PATH") == 0 && n == 4U) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] <= c->last_job) {
            return reject(c, "job");
        }
        if (ct_busy(c)) {
            return reject(c, "busy");
        }
        if (!c->referenced) {
            return reject(c, "unreferenced");
        }
        const char *why = ct_inputs(c, now);
        if (why != NULL) {
            return reject(c, why);
        }
        if (!command_number_i64(w[3], 1, 500, &v[3])) {
            return reject(c, "path_invalid");
        }

        c->job = (uint32_t)v[2];
        c->last_job = (uint32_t)v[2];
        c->lease_ms = now;
        c->start_ms = now;
        c->path_total = (uint16_t)v[3];
        c->path_received = 0;
        c->path_done = 0;
        c->path_head = 0;
        c->path_size = 0;
        memcpy(c->path_tail, c->pos, sizeof(c->pos));
        c->path_active = true;
        c->path_started = false;
        c->mode = CT_MOVE;
        c->reason = "path_loading";
        return CT_OK;
    }

    /* ========================================================================
     * 6. SEG <session> <job> <idx> <z> <j1> <j2> <peak> <entry> <exit> <accel>
     * [EN] Appends one preflighted trajectory segment into `path_queue` FIFO:
     *      - Verifies constant paper Z across the stroke (`s.target[CT_Z] == path_tail[CT_Z]`).
     *      - Verifies soft-limit workspace bounds and quintic ramp feasibility.
     * [VI] Nạp một đoạn quỹ đạo vào bộ đệm FIFO `path_queue`:
     *      - Đảm bảo cao độ bút Z không đổi trong suốt nét vẽ.
     *      - Kiểm tra giới hạn mềm và tính khả thi của đường cong tăng/giảm tốc bậc 5.
     * ======================================================================== */
    if (strcmp(w[0], "SEG") == 0 && n == 11U) {
        if (!c->path_active ||
            !command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] != c->job ||
            !command_number_i64(w[3], 0, 499, &v[3]) || (uint16_t)v[3] != c->path_received ||
            c->path_received >= c->path_total || c->path_size >= CT_PATH_CAP) {
            return reject(c, "path_sequence");
        }

        struct ct_segment s = {0};
        uint32_t ticks = 0;
        for (unsigned a = 0; a < CT_AXES; a++) {
            if (!command_number_i64(w[a + 4U], -CT_MAX_POS, CT_MAX_POS, &v[a + 4U])) {
                return reject(c, "target");
            }
            int64_t p = v[a + 4U] + c->origin[a];
            int64_t d = p - c->path_tail[a];
            if (p < -CT_MAX_POS || p > CT_MAX_POS || llabs(d) > 1000000LL) {
                return reject(c, "target");
            }
            s.target[a] = (int32_t)p;
            if ((uint32_t)llabs(d) > ticks) {
                ticks = (uint32_t)llabs(d);
            }
        }

        /*
         * [EN] A drawing stroke cannot change paper Z; pen lifts use ordinary MOVE.
         * [VI] Trong một nét vẽ liên tục, trục Z phải giữ nguyên; nhấc/hạ bút dùng lệnh MOVE.
         */
        if (ticks == 0U || s.target[CT_Z] != c->path_tail[CT_Z] || !ct_pose_valid(c, s.target, 1000)) {
            return reject(c, "path_invalid");
        }

        /*
         * [EN] Parse velocity profile parameters:
         *      - `w[7]` = `peak_rate`  (50 .. CT_MAX_RATE pps)
         *      - `w[8]` = `entry_rate` (50 .. peak_rate pps)
         *      - `w[9]` = `exit_rate`  (50 .. peak_rate pps)
         *      - `w[10]`= `accel`      (50 .. 200,000 pps^2)
         * [VI] Đọc các tham số quy hoạch vận tốc của đoạn:
         *      - `w[7]` = `peak_rate`  (vận tốc đỉnh: 50 .. CT_MAX_RATE xung/giây)
         *      - `w[8]` = `entry_rate` (vận tốc đầu đoạn: 50 .. peak_rate xung/giây)
         *      - `w[9]` = `exit_rate`  (vận tốc cuối đoạn: 50 .. peak_rate xung/giây)
         *      - `w[10]`= `accel`      (gia tốc: 50 .. 200.000 xung/giây^2)
         */
        if (!command_number_i64(w[7], 50, CT_MAX_RATE, &v[7]) ||
            !command_number_i64(w[8], 50, v[7], &v[8]) ||
            !command_number_i64(w[9], 50, v[7], &v[9]) ||
            !command_number_i64(w[10], 50, 200000, &v[10])) {
            return reject(c, "path_profile");
        }

        int64_t peak_rate = v[7];
        int64_t entry_rate = v[8];
        int64_t exit_rate = v[9];
        int64_t accel = v[10];

        /*
         * [EN] First segment must start at 50 pps, and final segment must exit at 50 pps.
         * [VI] Đoạn đầu tiên của nét phải bắt đầu ở 50 pps, và đoạn cuối phải kết thúc ở 50 pps.
         */
        bool is_first_seg = (c->path_received == 0U);
        bool is_last_seg = (c->path_received + 1U == c->path_total);
        if ((is_first_seg && entry_rate != 50) || (is_last_seg && exit_rate != 50)) {
            return reject(c, "path_profile");
        }

        /*
         * [EN] Kinematic feasibility check (`|v_entry^2 - v_exit^2| <= 2 * accel * ticks`)
         *      and quintic smootherstep ramp length check (`up_ticks + down_ticks <= ticks`).
         * [VI] Kiểm tra công thức động học (`|v_entry^2 - v_exit^2| <= 2 * accel * ticks`)
         *      và tổng chiều dài tăng/giảm tốc bậc 5 (`up_ticks + down_ticks <= ticks`).
         */
        if (llabs(entry_rate * entry_rate - exit_rate * exit_rate) > 2LL * accel * (int64_t)ticks) {
            return reject(c, "path_profile");
        }

        uint32_t up_ticks = ct_path_ramp((uint32_t)peak_rate, (uint32_t)entry_rate, (uint32_t)accel);
        uint32_t down_ticks = ct_path_ramp((uint32_t)peak_rate, (uint32_t)exit_rate, (uint32_t)accel);
        if (up_ticks + down_ticks > ticks) {
            return reject(c, "path_profile");
        }

        s.peak = (uint16_t)peak_rate;
        s.entry = (uint16_t)entry_rate;
        s.exit = (uint16_t)exit_rate;
        s.accel = (uint32_t)accel;

        unsigned slot = (c->path_head + c->path_size) % CT_PATH_CAP;
        c->path_queue[slot] = s;
        c->path_size++;
        c->path_received++;
        memcpy(c->path_tail, s.target, sizeof(s.target));
        return CT_OK;
    }

    /* ========================================================================
     * 7. GO <session> <job>
     * [EN] Launches execution of the buffered path once pre-fill is complete.
     * [VI] Bắt đầu chạy chuỗi quỹ đạo sau khi đã nạp đủ số đoạn khởi đầu vào FIFO.
     * ======================================================================== */
    if (strcmp(w[0], "GO") == 0 && n == 3U) {
        uint16_t required_prefill = (c->path_total < CT_PATH_CAP) ? c->path_total : CT_PATH_CAP;
        if (!c->path_active || c->path_started ||
            !command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] != c->job ||
            c->path_size < required_prefill) {
            return reject(c, "path_not_buffered");
        }
        const char *why = ct_inputs(c, now);
        if (why != NULL) {
            return reject(c, why);
        }
        c->lease_ms = now;
        c->start_ms = now;
        if (!ct_path_advance(c)) {
            return reject(c, "path_invalid");
        }
        c->reason = "path_running";
        return CT_START;
    }

    /* ========================================================================
     * 8. HOLD <session> <J1|J2> <0|1>
     * [EN] Enables (`1`) or releases (`0`) holding torque on arm motor J1 or J2.
     *      Releasing or changing hold invalidates the home reference (`referenced = false`).
     * [VI] Bật (`1`) hoặc nhả (`0`) lực giữ trục J1/J2 để sinh viên có thể xoay tay
     *      kiểm tra cơ khí. Khi nhả phanh, cờ gốc tọa độ `referenced` sẽ bị xóa.
     * ======================================================================== */
    if (strcmp(w[0], "HOLD") == 0 && n == 4U) {
        unsigned a = 0;
        if (strcmp(w[2], "J1") == 0) {
            a = 1;
        } else if (strcmp(w[2], "J2") == 0) {
            a = 2;
        }
        if (a == 0U || !command_number_i64(w[3], 0, 1, &v[3]) || ct_busy(c) || c->fault) {
            return reject(c, "hold_rejected");
        }
        c->hold_axis = a;
        c->hold_value = (bool)v[3];
        c->referenced = false;
        return CT_HOLD_CHANGE;
    }

    /* ========================================================================
     * 9. POL <session> <Z|J1|J2> <0|1>
     * [EN] Configures motor DIR pin polarity for axis Z, J1, or J2.
     * [VI] Cấu hình cực tính chân chiều quay DIR cho trục Z, J1 hoặc J2.
     * ======================================================================== */
    if (strcmp(w[0], "POL") == 0 && n == 4U) {
        unsigned a = 3;
        if (strcmp(w[2], "Z") == 0) {
            a = 0;
        } else if (strcmp(w[2], "J1") == 0) {
            a = 1;
        } else if (strcmp(w[2], "J2") == 0) {
            a = 2;
        }
        if (a >= 3U || !command_number_i64(w[3], 0, 1, &v[3]) || ct_busy(c)) {
            return reject(c, "pol_rejected");
        }
        if (c->pol[a] != (bool)v[3]) {
            c->has_calibrated = false;
        }
        c->pol[a] = (bool)v[3];
        c->referenced = false;
        return CT_OK;
    }

    /* ========================================================================
     * 10. COUPLE <session> <auto_enable:0|1> <probe_pulses:16..256>
     * [EN] Configures automatic belt-coupling verification during `INIT`.
     * [VI] Cấu hình chế độ tự động đo kiểm tỷ số truyền dây đai chéo khi `INIT`.
     * ======================================================================== */
    if (strcmp(w[0], "COUPLE") == 0 && n == 4U) {
        if (ct_busy(c) ||
            !command_number_i64(w[2], 0, 1, &v[2]) ||
            !command_number_i64(w[3], 16, 256, &v[3])) {
            return reject(c, "coupling_config");
        }
        c->auto_coupling = (bool)v[2];
        c->probe_pulses = (uint32_t)v[3];
        c->referenced = false;
        return CT_OK;
    }

    /* ========================================================================
     * 11. BACKLASH <session> <j1_pulses:0..256> <j2_pulses:0..256>
     * [EN] Configures backlash take-up pulses on direction reversal for J1 and J2.
     * [VI] Cấu hình số xung bù độ rơ dây đai/hộp số khi đảo chiều quay J1 và J2.
     * ======================================================================== */
    if (strcmp(w[0], "BACKLASH") == 0 && n == 4U) {
        if (ct_busy(c) ||
            !command_number_i64(w[2], 0, 256, &v[2]) ||
            !command_number_i64(w[3], 0, 256, &v[3])) {
            return reject(c, "backlash_config");
        }
        c->backlash[CT_J1] = (uint16_t)v[2];
        c->backlash[CT_J2] = (uint16_t)v[3];
        return CT_OK;
    }

    /* ========================================================================
     * 12. Calibration & Geometry Configuration (`GEOM`, `SPAN`, `PARK`, `TUNE`)
     * [EN] All 4 configuration commands (`cfg_mask == 15`) must be received
     *      before `INIT` is allowed to run.
     * [VI] Bốn lệnh cấu hình (`GEOM`, `SPAN`, `PARK`, `TUNE` ứng với `cfg_mask == 15`)
     *      phải được nạp đủ trước khi cho phép chạy lệnh dò gốc `INIT`.
     * ======================================================================== */
    bool config = (strcmp(w[0], "GEOM") == 0 || strcmp(w[0], "SPAN") == 0 ||
                   strcmp(w[0], "PARK") == 0 || strcmp(w[0], "TUNE") == 0);
    if (config) {
        if (ct_busy(c)) {
            return reject(c, "busy");
        }

        if (strcmp(w[0], "GEOM") == 0) {
            /*
             * [EN] `GEOM <session> <coupling_ppm> <z_nom> <j1_nom> <j2_nom> <z_micro> <j1_micro> <j2_micro>`
             * [VI] Lệnh `GEOM`: nạp tỷ số truyền đai `coupling_ppm`, hệ số xung `nominal[0..2]` và vi bước `micro[0..2]`.
             */
            if (n != 9U ||
                !command_number_i64(w[2], -2000000, 2000000, &v[2]) ||
                !command_number_i64(w[3], 1024, 100000000, &v[3]) ||
                !command_number_i64(w[4], 1024, 1000000, &v[4]) ||
                !command_number_i64(w[5], 1024, 1000000, &v[5])) {
                return reject(c, "geometry");
            }
            for (unsigned a = 0; a < 3U; a++) {
                /*
                 * [EN] Microstep setting must be a power of 2 in [1..32].
                 *      Bitwise check: `(m & (m - 1)) == 0` is true ONLY for powers of 2!
                 * [VI] Hệ số vi bước (microstep) phải là lũy thừa của 2 từ 1 đến 32.
                 *      Mẹo kiểm tra bit: `(m & (m - 1)) == 0` chỉ đúng khi `m` là lũy thừa của 2!
                 */
                if (!command_number_i64(w[a + 6U], 1, 32, &v[a + 6U])) {
                    return reject(c, "microstep");
                }
                int64_t m = v[a + 6U];
                if ((m & (m - 1)) != 0) {
                    return reject(c, "microstep");
                }
            }
            if (c->coupling_ppm != (int32_t)v[2]) {
                c->has_calibrated = false;
            }
            c->coupling_ppm = (int32_t)v[2];
            for (unsigned a = 0; a < 3U; a++) {
                if (c->nominal[a] != (uint32_t)v[a + 3U] || c->micro[a] != (uint32_t)v[a + 6U]) {
                    c->has_calibrated = false;
                }
                c->nominal[a] = (uint32_t)v[a + 3U];
                c->micro[a] = (uint32_t)v[a + 6U];
            }
            c->cfg_mask |= 1U;

        } else if (strcmp(w[0], "SPAN") == 0) {
            /*
             * [EN] `SPAN <session> <j1_low_md> <j1_high_md> <j2_low_md> <j2_high_md> <z_span_um>`
             * [VI] Lệnh `SPAN`: cài đặt giới hạn góc mềm của khớp J1, J2 (mdeg) và hành trình trục Z (um).
             */
            if (n != 7U) {
                return reject(c, "span");
            }
            for (unsigned i = 2; i <= 5U; i++) {
                if (!command_number_i64(w[i], -180000, 180000, &v[i])) {
                    return reject(c, "angle");
                }
            }
            if (!command_number_i64(w[6], 0, 2000000, &v[6])) {
                return reject(c, "z_span");
            }
            for (unsigned j = 0; j < 2U; j++) {
                int64_t lo = v[2U + 2U * j];
                int64_t hi = v[3U + 2U * j];
                if (hi - lo < 10000 || hi - lo > 360000) {
                    return reject(c, "span");
                }
                if (c->low_md[j] != (int32_t)lo || c->high_md[j] != (int32_t)hi) {
                    c->has_calibrated = false;
                }
                c->low_md[j] = (int32_t)lo;
                c->high_md[j] = (int32_t)hi;
            }
            if (c->z_span_um != (uint32_t)v[6]) {
                c->has_calibrated = false;
            }
            c->z_span_um = (uint32_t)v[6];
            c->cfg_mask |= 2U;

        } else if (strcmp(w[0], "PARK") == 0) {
            /*
             * [EN] `PARK <session> <j1_park_md> <j2_park_md> <z_clear_um>`
             * [VI] Lệnh `PARK`: cài đặt tư thế nghỉ sau khi dò gốc (`park_md[0..1]`, `z_clear_um`).
             */
            if (n != 5U ||
                !command_number_i64(w[2], -180000, 180000, &v[2]) ||
                !command_number_i64(w[3], -180000, 180000, &v[3]) ||
                !command_number_i64(w[4], 100, 100000, &v[4])) {
                return reject(c, "park");
            }
            for (unsigned j = 0; j < 2U; j++) {
                if (v[j + 2U] <= c->low_md[j] + 2000 || v[j + 2U] >= c->high_md[j] - 2000) {
                    return reject(c, "park");
                }
            }
            /*
             * [EN] Prevent parking near the straight-out (0 deg) or folded-back (180 deg) SCARA singularity!
             * [VI] Cấm chọn góc Park của khuỷu J2 quá gần điểm kỳ dị thẳng hàng (0 độ) hoặc gập sát (180 độ)!
             */
            if (llabs(v[3]) < 10000 || llabs(v[3]) > 170000) {
                return reject(c, "park_singular");
            }
            c->park_md[0] = (int32_t)v[2];
            c->park_md[1] = (int32_t)v[3];
            c->z_clear_um = (uint32_t)v[4];
            c->cfg_mask |= 4U;

        } else {
            /*
             * [EN] `TUNE <session> <arm_rate> <z_rate> <arm_scan> <z_scan> <arm_back> <z_back> [z_fine] [j2_rate j2_fine]`
             *      Configures homing speeds, scan budgets, and backoff distances.
             * [VI] Lệnh `TUNE`: cài đặt tốc độ dò gốc, giới hạn quét tối đa và khoảng lùi công tắc.
             */
            bool valid_arity = (n == 8U || n == 9U || n == 11U);
            if (!valid_arity ||
                !command_number_i64(w[2], 100, 800, &v[2]) ||
                !command_number_i64(w[3], 100, CT_Z_CAL_RATE, &v[3]) ||
                !command_number_i64(w[4], 256, 1000000, &v[4]) ||
                !command_number_i64(w[5], 256, 1000000, &v[5]) ||
                !command_number_i64(w[6], 32, 8192, &v[6]) ||
                !command_number_i64(w[7], 32, 8192, &v[7]) ||
                v[6] >= v[4] || v[7] >= v[5]) {
                return reject(c, "tuning");
            }
            if (n >= 9U && !command_number_i64(w[8], 50, v[3], &v[8])) {
                return reject(c, "tuning");
            }
            if (n == 11U && (!command_number_i64(w[9], 100, 1600, &v[9]) ||
                             !command_number_i64(w[10], 50, v[9], &v[10]))) {
                return reject(c, "tuning");
            }

            c->arm_rate = (uint32_t)v[2];
            c->z_rate = (uint32_t)v[3];
            c->arm_scan = (uint32_t)v[4];
            c->z_scan = (uint32_t)v[5];
            c->arm_back = (uint32_t)v[6];
            c->z_back = (uint32_t)v[7];
            c->cfg_mask |= 8U;

            if (n >= 9U) {
                c->z_fine_rate = (uint32_t)v[8];
            } else if (c->z_fine_rate > c->z_rate) {
                c->z_fine_rate = c->z_rate;
            }

            if (n == 11U) {
                c->j2_rate = (uint32_t)v[9];
                c->j2_fine_rate = (uint32_t)v[10];
            } else {
                c->j2_rate = c->arm_rate;
                c->j2_fine_rate = (c->j2_rate < 200U) ? c->j2_rate : 200U;
            }
        }

        c->referenced = false;
        return CT_OK;
    }

    /* ========================================================================
     * 13. INIT <session> <job> [QUICK]
     * [EN] Launches the 3-axis homing & calibration state machine (`HS_Z`).
     *      When `QUICK` is appended and `c->has_calibrated` is true, each axis
     *      only latches its negative switch (`SEEK_N -> CLEAR_N -> LATCH_N`)
     *      and reuses the previously calibrated `saved_range`, skipping the
     *      coupling probe to cut homing time by ~75%.
     * [VI] Khởi chạy quy trình tự động dò gốc 3 trục (`HS_Z`). Nếu có thêm tham số
     *      `QUICK` và robot đã từng hiệu chuẩn toàn diện (`has_calibrated == true`),
     *      mỗi trục chỉ cần chốt công tắc âm và tái sử dụng `saved_range`, giúp
     *      rút ngắn ~75% thời gian về gốc.
     * ======================================================================== */
    if (strcmp(w[0], "INIT") == 0 && (n == 3U || n == 4U)) {
        if (n == 4U && strcmp(w[3], "QUICK") != 0) {
            return reject(c, "command");
        }
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] <= c->last_job) {
            return reject(c, "job");
        }
        if (ct_busy(c)) {
            return reject(c, "busy");
        }
        const char *why = ct_inputs(c, now);
        if (why != NULL) {
            return reject(c, why);
        }
        if (c->cfg_mask != 15U || c->reference_epoch == UINT32_MAX) {
            return reject(c, "configuration");
        }

        bool use_quick = (n == 4U) && c->has_calibrated;
        c->job = (uint32_t)v[2];
        c->last_job = (uint32_t)v[2];
        c->lease_ms = now;
        c->start_ms = now;
        c->input_axis = -1;
        c->input_kind = 0;
        c->wait_axes = 0;
        c->wait_bits = 0;
        c->referenced = false;
        c->initializing = true;
        c->quick_init = use_quick;
        if (!use_quick) {
            c->coupling_ready = false;
            c->beta_measured = 0;
        }
        c->probe_contact = false;
        c->probe_retry = false;
        c->probe_da = 0;
        c->probe_db = 0;
        c->comp_step = false;

        memset(c->span_md, 0, sizeof(c->span_md));
        memset(c->comp_left, 0, sizeof(c->comp_left));
        memset(c->last_dir, 0, sizeof(c->last_dir));
        memset(c->factor, 0, sizeof(c->factor));
        memset(c->range, 0, sizeof(c->range));

        for (unsigned a = 0; a < 3U; a++) {
            c->cal[a].phase = HMC_IDLE;
            c->cal[a].failed_phase = HMC_IDLE;
            c->cal[a].error = HMC_NONE;
            c->cal[a].n1 = 0;
            c->cal[a].n2 = 0;
            c->cal[a].reference = false;
            c->cal[a].quick = use_quick;
            c->cal[a].pulses = use_quick ? c->saved_range[a] : 0U;
        }

        ct_beta_update(c);
        if (llabs(c->beta) > 64LL * CT_Q) {
            return reject(c, "coupling_bounds");
        }
        return ct_begin_cal(c, CT_Z, HS_Z, now);
    }

    /* ========================================================================
     * 14. MOVE <session> <job> <z_steps> <j1_steps> <j2_steps> <rate_pps>
     * [EN] Executes a coordinated 3-axis point-to-point move after verifying
     *      that the robot is referenced and the target pose is within soft limits.
     * [VI] Thực thi chuyển động nội suy đồng thời 3 trục từ điểm tới điểm sau khi
     *      kiểm tra robot đã có gốc tọa độ và điểm đích nằm trong giới hạn mềm.
     * ======================================================================== */
    if (strcmp(w[0], "MOVE") == 0 && n == 7U) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || (uint32_t)v[2] <= c->last_job) {
            return reject(c, "job");
        }
        if (ct_busy(c)) {
            return reject(c, "busy");
        }
        if (!c->referenced) {
            return reject(c, "unreferenced");
        }
        const char *why = ct_inputs(c, now);
        if (why != NULL) {
            return reject(c, why);
        }

        int32_t target[CT_AXES];
        for (unsigned a = 0; a < 3U; a++) {
            if (!command_number_i64(w[a + 3U], -CT_MAX_POS, CT_MAX_POS, &v[a + 3U])) {
                return reject(c, "target");
            }
            int64_t p = v[a + 3U] + c->origin[a];
            if (p < -CT_MAX_POS || p > CT_MAX_POS) {
                return reject(c, "target");
            }
            target[a] = (int32_t)p;
        }

        /*
         * [EN] Pure Z moves can run up to `CT_Z_CAL_RATE` (6400 pps); arm moves cap at `CT_MAX_RATE` (1600 pps).
         * [VI] Chuyển động chỉ riêng trục Z được phép chạy tới 6400 pps; có khớp tay máy giới hạn ở 1600 pps.
         */
        bool is_pure_z_move = (target[1] == c->pos[1]) && (target[2] == c->pos[2]);
        uint32_t max_rate = is_pure_z_move ? CT_Z_CAL_RATE : CT_MAX_RATE;
        if (!command_number_i64(w[6], 50, max_rate, &v[6]) || !ct_pose_valid(c, target, 1000)) {
            return reject(c, "soft_limit");
        }

        c->job = (uint32_t)v[2];
        c->last_job = (uint32_t)v[2];
        c->lease_ms = now;
        c->start_ms = now;
        c->input_axis = -1;
        c->input_kind = 0;

        enum ct_result r = ct_raw_move(c, target, (uint32_t)v[6], false);
        if (r == CT_REJECTED) {
            return reject(c, "move_budget");
        }
        c->reason = c->running ? "moving" : "complete";
        return r;
    }

    return reject(c, "invalid_command");
}
