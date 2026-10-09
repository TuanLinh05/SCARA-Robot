#ifndef SCARA_CART_TELEMETRY_H
#define SCARA_CART_TELEMETRY_H

#include "cart_core.h"
#include <stddef.h>

#define CT_STATUS_CAPACITY 1536U
#define CT_ACK_CAPACITY 192U
/* Copy only published fields while the caller holds its state lock. Formatting
 * uses this compact snapshot after unlocking; the worker/FIFO is never copied. */
struct ct_status {
    int32_t p[CT_AXES], g[CT_AXES];
    uint32_t range[CT_AXES], f[CT_AXES], n1[CT_AXES], n2[CT_AXES];
    uint64_t total[CT_AXES];
    unsigned phases[CT_AXES], errors[CT_AXES], failed[CT_AXES];
    unsigned mask, ready, hold, pol, conflicts;
    uint32_t session, job, epoch, tick, ticks, late;
    unsigned path[4];
    uint32_t starved, recovered, input_at, brief[CT_AXES], false_hits[CT_AXES], idle_ignored;
    int input_axis;
    unsigned input_kind, input_bits, raw, io_errors;
    bool input_wait, gate_wait;
    unsigned gate_axis;
    int direction[CT_AXES];
    unsigned dir_levels, mode, selected, pps;
    int32_t coupling_ppm, beta_q, probe_da, probe_db;
    bool coupling_ready, busy, referenced, fault;
    enum ct_stage stage;
    const char *reason;
    int motor_error, timer_error;
};
void ct_status_capture(struct ct_status *status, const struct ct_state *control,
    bool timer_running, uint32_t timer_late, uint32_t now);
int ct_status_format(char *output, size_t capacity, const struct ct_status *status, uint32_t now);
int ct_ack_format(char *output, size_t capacity, uint32_t session, uint32_t job,
    const char *operation, bool ok, const char *reason);

#endif
