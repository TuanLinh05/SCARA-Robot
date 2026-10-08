#ifndef SCARA_CART_CORE_H
#define SCARA_CART_CORE_H
#include "home_worker.h"
/* Wire order Z, J1, J2. Integer arithmetic only in the pulse ISR. */
enum { CT_Z, CT_J1, CT_J2, CT_AXES };
enum ct_result { CT_OK, CT_START, CT_REJECTED, CT_HOLD_CHANGE };
enum ct_mode { CT_IDLE, CT_MOVE, CT_CAL, CT_PARK };
enum ct_stage { HS_IDLE, HS_Z, HS_J2_PRE, HS_J2_MID, HS_J1, HS_J1_MID,
    HS_J2_FINAL, HS_PARK, HS_READY, HS_FAILED,
    HS_COUPLE_PROBE, HS_COUPLE_SCAN, HS_COUPLE_RETURN };
#define CT_Q 1048576LL
#define CT_MAX_POS 4000000
#define CT_MAX_RATE 1600U
#define CT_Z_CAL_RATE 6400U
#define CT_LEASE 350U
#define CT_INIT_MS 1800000U
#define CT_INPUT_WAIT_MS 200U
#define CT_PATH_CAP 32U
struct ct_segment {
    int32_t target[CT_AXES];
    uint32_t accel;
    uint16_t peak, entry, exit;
};
struct ct_state {
    struct zj_state sw[CT_AXES], cm;
    struct hc_state cal[CT_AXES];
    int32_t pos[CT_AXES], origin[CT_AXES], goal[CT_AXES];
    uint64_t total[CT_AXES];
    uint32_t range[CT_AXES], factor[CT_AXES], nominal[CT_AXES], micro[CT_AXES];
    /* factor = pulses per mm/degree * 1024; angles in millidegrees. */
    int32_t low_md[2], high_md[2], park_md[2], coupling_ppm;
    uint32_t z_span_um, z_clear_um, arm_rate, j2_rate, j2_fine_rate, z_rate, z_fine_rate, arm_scan, z_scan, arm_back, z_back;
    uint32_t session, job, last_job, lease_ms, start_ms, tick, ticks, rate, reference_epoch;
    uint32_t delta[CT_AXES];
    uint64_t accum[CT_AXES];
    int direction[CT_AXES], joint_dir[CT_AXES];
    int64_t beta, cal_scale, cal_weight[CT_AXES];
    /* Measure dB/dA from the same J2 endpoint before/after a bounded A probe. */
    bool auto_coupling, coupling_ready, probe_contact, probe_retry;
    uint32_t probe_pulses, probe_range;
    int32_t probe_a0, probe_b0, probe_mid, probe_da, probe_db;
    unsigned selected, cfg_mask, hold_axis;
    bool pol[CT_AXES], holding[CT_AXES], running, initializing, referenced, fault, hold_value;
    /* During initialization only: first double-open freezes pulses. Keep the
     * DDA accumulator and worker state until a bounded stationary check ends. */
    bool input_wait, wait_running;
    int wait_cm_dir;
    enum zj_reason wait_cm_reason;
    unsigned wait_axes, wait_bits;
    uint32_t wait_start_ms, wait_stable_ms, ramp_ms, ramp_tick, recovered;
    int input_axis; /* -1 means no input event since the accepted job. */
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
    /* Bounded FIFO; the host preflights the whole stroke before uploading.
     * Segment transitions happen only after draining the final HIGH pulse. */
    struct ct_segment path_queue[CT_PATH_CAP];
    int32_t path_tail[CT_AXES];
    uint32_t path_accel, path_starved;
    uint32_t path_up, path_down;
    uint16_t path_total, path_received, path_done, path_head, path_size;
    uint16_t path_entry, path_exit;
    bool path_active, path_started;
};
void ct_init(struct ct_state *c);
bool ct_busy(const struct ct_state *c);
void ct_abort(struct ct_state *c, const char *reason);
void ct_sample(struct ct_state *c, unsigned axis, int positive, int negative, uint32_t now);
bool ct_safe(struct ct_state *c, uint32_t now);
unsigned ct_next_mask(struct ct_state *c);
void ct_count(struct ct_state *c, unsigned axis);
void ct_finish_tick(struct ct_state *c);
uint32_t ct_period_us(const struct ct_state *c, uint32_t now);
bool ct_path_advance(struct ct_state *c);
enum ct_result ct_service(struct ct_state *c, uint32_t now);
enum ct_result ct_command(struct ct_state *c, const char *line, uint32_t now);
void ct_hold_complete(struct ct_state *c, bool success);
#endif

