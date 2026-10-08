#ifndef SCARA_HOME_WORKER_H
#define SCARA_HOME_WORKER_H
#include "jog_core.h"
#define HMC_PHASE_MS 600000U
#define HMC_TOTAL_MS 1800000U
#define HMC_CONFIRM_MS 200U
enum hc_phase { HMC_IDLE, HMC_SEEK_N, HMC_CLEAR_N, HMC_LATCH_N, HMC_SWEEP_P,
    HMC_CLEAR_P, HMC_LATCH_P, HMC_SWEEP_N, HMC_CLEAR_N2, HMC_LATCH_N2, HMC_DONE, HMC_FAILED };
enum hc_error { HMC_NONE, HMC_CANCELLED, HMC_LINK, HMC_IO, HMC_INPUT, HMC_BOUNDS,
    HMC_UNSTABLE, HMC_REPEAT, HMC_BAD_COMMAND };
struct hc_state {
    enum hc_phase phase;
    enum hc_phase failed_phase;
    enum hc_error error;
    /* Last successful model, retained if a later attempt fails. RAM only. */
    uint32_t pulses, angle_mdeg, micro, epoch;
    int64_t origin;
    bool reference;
    /* Current attempt. */
    uint32_t n1, n2, run_angle, run_micro, rate, fine_rate, max_scan, backoff;
    uint32_t false_hits;
    uint32_t start_ms, lease_ms, settle_ms, hit_ms;
    uint64_t start_total;
    int64_t negative_anchor, positive_anchor;
    bool settling, extra, arrived;
};
bool hc_active(const struct hc_state *c);
void hc_cancel(struct hc_state *c, struct zj_state *s, enum zj_reason reason, enum hc_error error);
enum zj_command_result hc_start(struct hc_state *c, struct zj_state *s,
    uint32_t angle, uint32_t rate, uint32_t max_scan, uint32_t backoff, uint32_t micro, uint32_t now);
enum zj_command_result hc_tick(struct hc_state *c, struct zj_state *s, uint32_t now);
#endif

