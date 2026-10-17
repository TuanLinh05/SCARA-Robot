/*
 * ============================================================================
 * FILE: cart_core.h
 * ============================================================================
 * [EN] 3-Axis SCARA Core State, Fixed-Point DDA Interpolator & Homing Stages
 *      Defines the central `struct ct_state` holding all 3 axes (`Z`, `J1`, `J2`),
 *      differential belt coupling (`beta`), backlash compensation, FIFO path
 *      queue (`CT_PATH_CAP = 32`), and multi-stage initialization state machine.
 *
 * [VI] Cấu trúc Trạng thái Lõi 3 Trục SCARA, Bộ Nội suy DDA & Quy trình Homing
 *      Định nghĩa cấu trúc trung tâm `struct ct_state` quản lý toàn bộ 3 trục
 *      (`Z`, `J1`, `J2`), hệ số bù truyền động dây đai chéo (`beta`), bù độ rơ
 *      cơ khí (`backlash`), hàng đợi quỹ đạo liên tục (`CT_PATH_CAP = 32` đoạn)
 *      và máy trạng thái khởi tạo hệ tọa độ tự động.
 *
 * [IMPORTANT ARCHITECTURE NOTE / LƯU Ý KIẾN TRÚC QUAN TRỌNG]:
 * - Wire axis index order is ALWAYS: `0 = CT_Z`, `1 = CT_J1`, `2 = CT_J2`.
 * - The pulse ISR (`pulse_tick` in `main.c`) uses ONLY integer/fixed-point
 *   arithmetic (`CT_Q = 2^20 = 1048576`) because STM32F103 Cortex-M3 has no
 *   hardware FPU!
 * - Thứ tự chỉ số trục luôn cố định: `0 = CT_Z` (trục đứng), `1 = CT_J1` (vai),
 *   `2 = CT_J2` (khuỷu).
 * - Ngắt phát xung (`pulse_tick` trong `main.c`) chỉ sử dụng số nguyên dấu phẩy
 *   tĩnh (`CT_Q = 2^20 = 1048576`) vì vi điều khiển STM32F103 Cortex-M3 không có
 *   bộ xử lý số thực phần cứng (FPU)!
 * ============================================================================
 */

#ifndef SCARA_CART_CORE_H
#define SCARA_CART_CORE_H

#include "home_worker.h"

/*
 * [EN] Axis indices: Z vertical prism (0), Joint 1 shoulder (1), Joint 2 elbow motor (2).
 * [VI] Chỉ số trục: Trục tịnh tiến Z (0), Khớp vai J1 (1), Động cơ khớp khuỷu J2 (2).
 */
enum {
    CT_Z = 0,
    CT_J1 = 1,
    CT_J2 = 2,
    CT_AXES = 3
};

/*
 * [EN] Return status for command execution and background service ticks.
 * [VI] Mã kết quả trả về khi xử lý lệnh hoặc hàm dịch vụ nền `ct_service`.
 */
enum ct_result {
    CT_OK,          /* [EN] Accepted, no motor timer start needed / [VI] Thành công, không cần bật mới Timer */
    CT_START,       /* [EN] Motion ready: caller must start pulse timer / [VI] Bắt đầu chạy: cần kích hoạt Timer phát xung */
    CT_REJECTED,    /* [EN] Command/state rejected (see `c->reply`) / [VI] Lệnh bị từ chối (xem lý do ở `c->reply`) */
    CT_HOLD_CHANGE  /* [EN] Caller must toggle motor ENA pin / [VI] Yêu cầu đóng/ngắt chân ENA giữ động cơ */
};

/*
 * [EN] Active motion mode of the 3-axis controller.
 * [VI] Chế độ chuyển động hiện tại của bộ điều khiển 3 trục.
 */
enum ct_mode {
    CT_IDLE,  /* [EN] Stationary / [VI] Đang đứng yên */
    CT_MOVE,  /* [EN] Coordinated point-to-point or path move / [VI] Chạy nội suy điểm-tới-điểm hoặc quỹ đạo vẽ */
    CT_CAL,   /* [EN] Single-axis limit switch calibration / [VI] Đang dò công tắc hành trình hiệu chuẩn */
    CT_PARK   /* [EN] Internal positioning move during INIT / [VI] Chạy di chuyển nội bộ trong quy trình INIT */
};

/*
 * [EN] Multi-stage homing & auto-coupling initialization sequence (`INIT` command).
 * [VI] Các bước tuần tự trong quy trình dò gốc và đo tỷ số truyền đai tự động (`INIT`).
 */
enum ct_stage {
    HS_IDLE,           /* [EN] Not initialized yet / [VI] Chưa khởi tạo gốc */
    HS_Z,              /* [EN] Step 1: Calibrate Z axis / [VI] Bước 1: Dò gốc và đo hành trình trục Z */
    HS_J2_PRE,         /* [EN] Step 2: Pre-calibrate J2 / [VI] Bước 2: Đo sơ bộ hành trình trục J2 */
    HS_J2_MID,         /* [EN] Step 3: Park J2 at midpoint / [VI] Bước 3: Đưa J2 ra chính giữa hành trình */
    HS_J1,             /* [EN] Step 5: Calibrate J1 (with J2 belt compensation) / [VI] Bước 5: Dò gốc trục J1 (có bù đai J2) */
    HS_J1_MID,         /* [EN] Step 6: Move J1 to park angle / [VI] Bước 6: Đưa J1 về góc Park */
    HS_J2_FINAL,       /* [EN] Step 7: Final J2 calibration at J1 park angle / [VI] Bước 7: Dò gốc chính xác J2 khi J1 ở vị trí Park */
    HS_PARK,           /* [EN] Step 8: Move all 3 axes to Park pose / [VI] Bước 8: Đưa cả 3 trục về tư thế Park hoàn tất */
    HS_READY,          /* [EN] Homing complete & referenced! / [VI] Đã có gốc tọa độ, sẵn sàng làm việc! */
    HS_FAILED,         /* [EN] Homing failed / [VI] Quy trình dò gốc thất bại */
    HS_COUPLE_PROBE,   /* [EN] Step 4a: Probe J1 by `probe_pulses` to measure belt coupling / [VI] Bước 4a: Nhích thử J1 để đo độ kéo theo của dây đai */
    HS_COUPLE_SCAN,    /* [EN] Step 4b: Re-scan J2 to measure shift `dB/dA` / [VI] Bước 4b: Quét lại J2 để tính tỷ số truyền đai thực tế */
    HS_COUPLE_RETURN   /* [EN] Step 4c: Return J1/J2 to pre-probe midpoint / [VI] Bước 4c: Đưa J1/J2 trở lại vị trí giữa trước khi dò J1 */
};

/*
 * [EN] Fixed-point scaling and operational limits:
 *      - CT_Q            : 2^20 (1,048,576) fixed-point unit for DDA & coupling math.
 *      - CT_MAX_POS      : Absolute step count ceiling (4,000,000 steps).
 *      - CT_MAX_RATE     : Maximum coordinated XY/arm step rate (1600 pulses/s).
 *      - CT_Z_CAL_RATE   : Maximum Z-only step rate (6400 pulses/s).
 *      - CT_LEASE        : Host heartbeat watchdog timeout (350 ms).
 *      - CT_INIT_MS      : Maximum duration for full 3-axis INIT (30 minutes).
 *      - CT_INPUT_WAIT_MS: Max recovery window for transient switch bounce (200 ms).
 *      - CT_PATH_CAP     : Ring-buffer capacity for streamed path segments (32 segments).
 *
 * [VI] Các hằng số dấu phẩy tĩnh và giới hạn vận hành:
 *      - CT_Q            : Hệ số nhân 2^20 (1.048.576) cho toán nguyên DDA và bù dây đai.
 *      - CT_MAX_POS      : Giới hạn tọa độ xung tuyệt đối (4.000.000 xung).
 *      - CT_MAX_RATE     : Tốc độ xung tối đa khi nội suy tay máy (1600 xung/giây).
 *      - CT_Z_CAL_RATE   : Tốc độ xung tối đa khi chỉ chạy riêng trục Z (6400 xung/giây).
 *      - CT_LEASE        : Thời gian chờ gói tin KEEP tối đa (350 ms).
 *      - CT_INIT_MS      : Thời gian tối đa cho toàn bộ lệnh INIT (30 phút).
 *      - CT_INPUT_WAIT_MS: Thời gian chờ ổn định công tắc tối đa khi bị nhiễu (200 ms).
 *      - CT_PATH_CAP     : Sức chứa bộ đệm vòng cho quỹ đạo vẽ liên tục (32 đoạn).
 */
#define CT_Q             1048576LL
#define CT_MAX_POS       4000000
#define CT_MAX_RATE      1600U
#define CT_Z_CAL_RATE    6400U
#define CT_LEASE         350U
#define CT_INIT_MS       1800000U
#define CT_INPUT_WAIT_MS 200U
#define CT_PATH_CAP      32U

/*
 * [EN] A single buffered trajectory segment in the continuous path FIFO (`PATH`/`SEG`/`GO`).
 * [VI] Cấu trúc một đoạn quỹ đạo trong hàng đợi FIFO khi vẽ nét liên tục (`PATH`/`SEG`/`GO`).
 */
struct ct_segment {
    int32_t target[CT_AXES]; /* [EN] Absolute motor step target / [VI] Đích đến tính bằng số bước động cơ */
    uint32_t accel;          /* [EN] Acceleration in pulses/s^2 / [VI] Gia tốc (xung/giây^2) */
    uint16_t peak;           /* [EN] Peak cruise rate (pps) / [VI] Vận tốc đỉnh của đoạn (xung/giây) */
    uint16_t entry;          /* [EN] Entry boundary rate (pps) / [VI] Vận tốc đầu đoạn (xung/giây) */
    uint16_t exit;           /* [EN] Exit boundary rate (pps) / [VI] Vận tốc cuối đoạn (xung/giây) */
};

/*
 * [EN] Master 3-axis SCARA controller state structure.
 * [VI] Cấu trúc dữ liệu tổng lưu toàn bộ trạng thái bộ điều khiển 3 trục SCARA.
 */
struct ct_state {
    /*
     * [EN] Limit-switch monitors (`sw[3]`), calibration motion monitor (`cm`),
     *      and per-axis homing worker state machines (`cal[3]`).
     * [VI] Bộ lọc công tắc hành trình 3 trục (`sw[3]`), bộ giám sát trục đang dò gốc (`cm`),
     *      và máy trạng thái đo hành trình của từng trục (`cal[3]`).
     */
    struct zj_state sw[CT_AXES], cm;
    struct hc_state cal[CT_AXES];

    /*
     * [EN] Step coordinates and scale factors for each axis:
     *      - `pos`    : Current raw motor step position (signed steps since power-on).
     *      - `origin` : Raw motor step coordinate corresponding to 0 mm (Z) or 0 deg (J1/J2).
     *      - `goal`   : Target raw motor step coordinate for the current move.
     *      - `total`  : Cumulative step pulses emitted since boot.
     *      - `range`  : Measured end-to-end travel span in pulses (`cal[a].pulses`).
     *      - `factor` : Active Q10 scale factor (`pulses_per_mm_or_deg * 1024`).
     *      - `nominal`: Configured nominal Q10 scale factor from `GEOM`.
     *      - `micro`  : Configured driver microstepping (`1, 2, 4, 8, 16, 32`).
     *
     * [VI] Tọa độ bước và hệ số tỷ lệ của từng trục:
     *      - `pos`    : Vị trí bước động cơ thô hiện tại (tính từ lúc bật nguồn).
     *      - `origin` : Tọa độ bước thô ứng với điểm gốc 0 mm (trục Z) hoặc 0 độ (J1/J2).
     *      - `goal`   : Tọa độ bước đích của lệnh di chuyển hiện tại.
     *      - `total`  : Tổng số xung đã phát ra từ lúc khởi động.
     *      - `range`  : Tổng số xung đo được giữa 2 công tắc hành trình.
     *      - `factor` : Hệ số quy đổi xung Q10 thực tế (`số_xung_trên_mm_hoặc_độ * 1024`).
     *      - `nominal`: Hệ số quy đổi xung Q10 danh định nhận từ lệnh `GEOM`.
     *      - `micro`  : Cấu hình vi bước của driver (`1, 2, 4, 8, 16, 32`).
     */
    int32_t pos[CT_AXES], origin[CT_AXES], goal[CT_AXES];
    uint64_t total[CT_AXES];
    uint32_t range[CT_AXES], factor[CT_AXES], nominal[CT_AXES], micro[CT_AXES];

    /*
     * [EN] Angular limits in millidegrees (`1 deg = 1000 mdeg`), Z travel in micrometers
     *      (`1 mm = 1000 um`), belt coupling in ppm (`1e6 = 1.0`), and homing tuning rates.
     * [VI] Giới hạn góc tính bằng phần nghìn độ (`1 độ = 1000 mdeg`), hành trình Z tính
     *      bằng micromet (`1 mm = 1000 um`), hệ số truyền đai (ppm) và tốc độ dò gốc.
     */
    int32_t low_md[2], high_md[2], park_md[2], span_md[2], coupling_ppm, beta_measured;
    uint32_t z_span_um, z_clear_um;
    uint32_t arm_rate, j2_rate, j2_fine_rate, z_rate, z_fine_rate;
    uint32_t arm_scan, z_scan, arm_back, z_back;

    /*
     * [EN] Session/job tracking, watchdog timestamps, DDA accumulators, and backlash state.
     * [VI] Quản lý phiên kết nối (`session`/`job`), thời gian watchdog, bộ tích lũy DDA và bù độ rơ.
     */
    uint32_t session, job, last_job, lease_ms, start_ms, tick, ticks, rate, reference_epoch;
    uint32_t delta[CT_AXES], comp_total[CT_AXES];
    uint16_t backlash[CT_AXES], comp_left[CT_AXES];
    uint64_t accum[CT_AXES];
    int direction[CT_AXES], joint_dir[CT_AXES], last_dir[CT_AXES];
    int64_t beta, cal_scale, cal_weight[CT_AXES];

    /*
     * [EN] Auto-coupling probe state: measures `dB / dA` from the same J2 endpoint
     *      before and after a short bounded J1 probe during `INIT`.
     * [VI] Trạng thái tự động đo tỷ số truyền đai chéo `dB / dA` bằng cách nhích thử J1
     *      và đo độ dịch chuyển điểm chạm công tắc J2 trong quá trình `INIT`.
     */
    bool auto_coupling, coupling_ready, probe_contact, probe_retry, comp_step;
    bool quick_init, has_calibrated;
    uint32_t probe_pulses, probe_range, saved_range[CT_AXES];
    int32_t probe_a0, probe_b0, probe_mid, probe_da, probe_db;

    /*
     * [EN] Axis selection, configuration bitmask (`cfg_mask == 15` when GEOM/SPAN/PARK/TUNE
     *      are all loaded), polarity, holding torque flags, and overall status flags.
     * [VI] Trục đang chọn, mặt nạ cấu hình (`cfg_mask == 15` khi đã nhận đủ 4 lệnh cấu hình),
     *      cực tính chân DIR, trạng thái giữ phanh động cơ và các cờ trạng thái hệ thống.
     */
    unsigned selected, cfg_mask, hold_axis;
    bool pol[CT_AXES], holding[CT_AXES], running, initializing, referenced, fault, hold_value;

    /*
     * [EN] During initialization only: a transient double-open on a switch pair
     *      freezes pulses temporarily while retaining DDA accumulators and worker
     *      state until a bounded stationary check (`stationary_check`) completes.
     * [VI] Trong quá trình INIT: nếu xuất hiện nhiễu thoáng qua trên cặp công tắc,
     *      tạm dừng phát xung nhưng giữ nguyên bộ cộng tích lũy DDA cho tới khi
     *      kiểm tra tĩnh (`stationary_check`) xác nhận công tắc đã ổn định trở lại.
     */
    bool input_wait, wait_running;
    int wait_cm_dir;
    enum zj_reason wait_cm_reason;
    unsigned wait_axes, wait_bits;
    uint32_t wait_start_ms, wait_stable_ms, ramp_ms, ramp_tick, recovered;

    /*
     * [EN] Switch event diagnostics and pre-transition stability gate state.
     * [VI] Các biến chẩn đoán sự kiện công tắc và cổng chờ ổn định trước khi chuyển pha.
     */
    int input_axis; /* [EN] -1 means no input event since accepted job / [VI] -1 nghĩa là chưa có sự kiện công tắc nào */
    unsigned input_bits, input_kind;
    uint32_t input_at_ms;
    unsigned last_hit_bits;
    uint32_t last_hit_ms;
    uint32_t idle_ignored, gate_start_ms;
    bool gate_wait, wait_guard;
    unsigned gate_axis;

    enum ct_mode mode;
    enum ct_stage stage;
    const char *reason, *reply;

    /*
     * [EN] Bounded FIFO path queue (`CT_PATH_CAP = 32`). Segment transitions
     *      happen seamlessly inside the pulse ISR after draining the final HIGH pulse.
     * [VI] Hàng đợi FIFO lưu các đoạn quỹ đạo vẽ liên tục (`CT_PATH_CAP = 32`).
     *      Việc chuyển tiếp giữa các đoạn diễn ra mượt mà ngay trong ngắt Timer.
     */
    struct ct_segment path_queue[CT_PATH_CAP];
    int32_t path_tail[CT_AXES];
    uint32_t path_accel, path_starved;
    uint32_t path_up, path_down;
    uint16_t path_total, path_received, path_done, path_head, path_size;
    uint16_t path_entry, path_exit;
    bool path_active, path_started;
};

/*
 * [EN] Initialize `struct ct_state` with safe power-on defaults.
 * [VI] Khởi tạo cấu trúc `struct ct_state` với các thông số mặc định an toàn khi bật nguồn.
 */
void ct_init(struct ct_state *c);

/*
 * [EN] Return true if the robot is currently stepping, homing (`initializing`), or running a path.
 * [VI] Trả về `true` nếu robot đang phát xung, đang dò gốc (`initializing`), hoặc đang chạy quỹ đạo.
 */
bool ct_busy(const struct ct_state *c);

/*
 * [EN] Emergency/controlled stop: halts motion, clears `referenced`, and records `reason`.
 * [VI] Dừng khẩn cấp: ngắt chuyển động, xóa cờ gốc tọa độ `referenced` và ghi lại mã lỗi `reason`.
 */
void ct_abort(struct ct_state *c, const char *reason);

/*
 * [EN] Feed raw positive/negative limit switch readings for `axis` at timestamp `now` (ms).
 * [VI] Cập nhật mức tín hiệu thô của cặp công tắc âm/dương cho trục `axis` tại thời điểm `now` (ms).
 */
void ct_sample(struct ct_state *c, unsigned axis, int positive, int negative, uint32_t now);

/*
 * [EN] Verify real-time safety before emitting a step pulse or continuing motion.
 * [VI] Kiểm tra an toàn thời gian thực trước mỗi nhịp phát xung.
 */
bool ct_safe(struct ct_state *c, uint32_t now);

/*
 * [EN] Advance DDA accumulators (or backlash counters) and return 3-bit mask of axes to step.
 * [VI] Tính toán bộ tích lũy DDA (hoặc bù độ rơ) và trả về mặt nạ 3-bit các trục cần phát xung.
 */
unsigned ct_next_mask(struct ct_state *c);

/*
 * [EN] Increment position and pulse counters after emitting a rising edge on `axis`.
 * [VI] Cập nhật tọa độ và bộ đếm xung sau khi phát sườn lên trên trục `axis`.
 */
void ct_count(struct ct_state *c, unsigned axis);

/*
 * [EN] Complete the current DDA tick after all selected axes have pulsed HIGH.
 * [VI] Hoàn tất một nhịp nội suy DDA sau khi các trục được chọn đã phát xung mức CAO.
 */
void ct_finish_tick(struct ct_state *c);

/*
 * [EN] Compute the current step pulse period in microseconds (`1,000,000 / rate_pps`).
 * [VI] Tính chu kỳ phát xung hiện tại theo đơn vị micro-giây (`1.000.000 / tốc_độ_xung`).
 */
uint32_t ct_period_us(const struct ct_state *c, uint32_t now);

/*
 * [EN] Pop and start the next trajectory segment from `path_queue` FIFO inside the ISR.
 * [VI] Lấy đoạn quỹ đạo kế tiếp từ hàng đợi FIFO `path_queue` ra chạy ngay trong ngắt ISR.
 */
bool ct_path_advance(struct ct_state *c);

/*
 * [EN] Background state machine tick for `INIT` homing sequence and path loading watchdog.
 * [VI] Hàm dịch vụ nền điều phối quy trình dò gốc `INIT` và giám sát nạp quỹ đạo.
 */
enum ct_result ct_service(struct ct_state *c, uint32_t now);

/*
 * [EN] Parse and execute one ASCII command line from the PC host.
 * [VI] Giải mã và thực thi một dòng lệnh ASCII gửi xuống từ máy tính.
 */
enum ct_result ct_command(struct ct_state *c, const char *line, uint32_t now);

/*
 * [EN] Report whether toggling the hardware `ENA` pin succeeded after a `HOLD` command.
 * [VI] Báo cáo kết quả đóng/ngắt chân phần cứng `ENA` sau khi thực hiện lệnh `HOLD`.
 */
void ct_hold_complete(struct ct_state *c, bool success);

#endif /* SCARA_CART_CORE_H */
