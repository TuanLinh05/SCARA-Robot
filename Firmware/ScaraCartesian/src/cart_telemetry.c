#include "cart_telemetry.h"
#include <stdio.h>
#include <string.h>

void ct_status_capture(struct ct_status *s, const struct ct_state *c,
    bool timer_running, uint32_t timer_late, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    s->session = c->session; s->job = c->job; s->epoch = c->reference_epoch;
    s->tick = c->tick; s->ticks = c->ticks; s->late = timer_late;
    s->path[0]=c->path_total; s->path[1]=c->path_received;
    s->path[2]=c->path_done; s->path[3]=CT_PATH_CAP-c->path_size;
    s->starved=c->path_starved;
    s->busy = ct_busy(c) || timer_running; s->referenced = c->referenced; s->fault = c->fault;
    s->stage = c->stage; s->reason = c->reason;
    s->recovered=c->recovered; s->input_wait=c->input_wait;
    s->idle_ignored=c->idle_ignored; s->gate_wait=c->gate_wait; s->gate_axis=c->gate_axis;
    s->input_axis=c->input_axis; s->input_kind=c->input_kind;
    s->input_bits=c->input_bits; s->input_at=c->input_at_ms;
    s->mode=c->mode; s->selected=c->selected;
    s->pps=c->running ? 1000000U/ct_period_us(c,now) : 0;
    s->coupling_ppm=c->coupling_ppm; s->beta_q=(int32_t)c->beta;
    s->probe_da=c->probe_da; s->probe_db=c->probe_db; s->coupling_ready=c->coupling_ready;
    for (unsigned a = 0; a < CT_AXES; a++) {
        s->p[a] = c->pos[a] - c->origin[a]; s->g[a] = c->goal[a] - c->origin[a];
        s->total[a] = c->total[a];
        s->brief[a]=c->sw[a].brief_both;
        s->false_hits[a]=c->cal[a].false_hits;
        s->io_errors |= (c->sw[a].error ? 1U : 0U) << a;
        s->raw |= ((c->sw[a].raw_pos ? 1U : 0U) | (c->sw[a].raw_neg ? 2U : 0U)) << (2*a);
        s->range[a] = c->range[a]; s->f[a] = c->factor[a];
        s->n1[a] = c->cal[a].n1; s->n2[a] = c->cal[a].n2;
        s->phases[a] = c->cal[a].phase; s->errors[a] = c->cal[a].error; s->failed[a] = c->cal[a].failed_phase;
        s->mask |= (c->sw[a].top || c->sw[a].raw_pos ? 1U : 0U) << (2 * a);
        s->mask |= (c->sw[a].bottom || c->sw[a].raw_neg ? 1U : 0U) << (2 * a + 1);
        s->ready |= (zj_ready(&c->sw[a]) ? 1U : 0U) << a;
        s->hold |= (c->holding[a] ? 1U : 0U) << a;
        s->pol |= (c->pol[a] ? 1U : 0U) << a;
        s->direction[a]=c->direction[a];
        s->conflicts |= (c->sw[a].conflict ? 1U : 0U) << a;
    }
}

int ct_status_format(char *output, size_t capacity, const struct ct_status *s, uint32_t now)
{
    return snprintf(output, capacity,
        "{\"type\":\"status\",\"protocol\":5,\"fw\":\"SCARA_CARTESIAN_NC_V5\",\"build\":\"SCARA_CART_NC_HOME_V5_R9\","
        "\"session\":%lu,\"job\":%lu,\"up_ms\":%lu,\"busy\":%d,\"referenced\":%d,\"epoch\":%lu,\"stage\":%u,"
        "\"pos\":[%ld,%ld,%ld],\"goal\":[%ld,%ld,%ld],\"total\":[%llu,%llu,%llu],\"range\":[%lu,%lu,%lu],\"factor\":[%lu,%lu,%lu],"
        "\"n1\":[%lu,%lu,%lu],\"n2\":[%lu,%lu,%lu],\"phase\":[%u,%u,%u],\"cal_error\":[%u,%u,%u],\"failed_phase\":[%u,%u,%u],"
        "\"switches\":%u,\"ready\":%u,\"holding\":%u,\"pol\":%u,\"conflicts\":%u,\"tick\":%lu,\"ticks\":%lu,"
        "\"raw\":%u,\"errors\":%u,\"input_wait\":%d,\"recovered\":%lu,\"input_axis\":%d,\"input_kind\":%u,\"input_bits\":%u,\"input_at\":%lu,\"brief\":[%lu,%lu,%lu],"
        "\"idle_ignored\":%lu,\"gate_wait\":%d,\"gate_axis\":%u,\"false_hits\":[%lu,%lu,%lu],"
        "\"mode\":%u,\"selected\":%u,\"pps\":%u,\"direction\":[%d,%d,%d],\"dir_levels\":%u,"
        "\"coupling_ppm\":%ld,\"beta_q\":%ld,\"probe_da\":%ld,\"probe_db\":%ld,\"coupling_ready\":%d,"
        "\"path\":[%u,%u,%u,%u],\"starved\":%lu,"
        "\"motor_error\":%d,\"fault\":%d,\"timer_late\":%lu,\"timer_errno\":%d,\"reason\":\"%s\"}\n",
        (unsigned long)s->session, (unsigned long)s->job, (unsigned long)now, s->busy, s->referenced,
        (unsigned long)s->epoch, (unsigned)s->stage,
        (long)s->p[0], (long)s->p[1], (long)s->p[2], (long)s->g[0], (long)s->g[1], (long)s->g[2],
        (unsigned long long)s->total[0], (unsigned long long)s->total[1], (unsigned long long)s->total[2],
        (unsigned long)s->range[0], (unsigned long)s->range[1], (unsigned long)s->range[2],
        (unsigned long)s->f[0], (unsigned long)s->f[1], (unsigned long)s->f[2],
        (unsigned long)s->n1[0], (unsigned long)s->n1[1], (unsigned long)s->n1[2],
        (unsigned long)s->n2[0], (unsigned long)s->n2[1], (unsigned long)s->n2[2],
        s->phases[0], s->phases[1], s->phases[2], s->errors[0], s->errors[1], s->errors[2], s->failed[0], s->failed[1], s->failed[2],
        s->mask, s->ready, s->hold, s->pol, s->conflicts, (unsigned long)s->tick, (unsigned long)s->ticks,
        s->raw, s->io_errors, s->input_wait, (unsigned long)s->recovered, s->input_axis, s->input_kind, s->input_bits, (unsigned long)s->input_at,
        (unsigned long)s->brief[0], (unsigned long)s->brief[1], (unsigned long)s->brief[2],
        (unsigned long)s->idle_ignored,s->gate_wait,s->gate_axis,
        (unsigned long)s->false_hits[0],(unsigned long)s->false_hits[1],(unsigned long)s->false_hits[2],
        s->mode,s->selected,s->pps,s->direction[0],s->direction[1],s->direction[2],s->dir_levels,
        (long)s->coupling_ppm,(long)s->beta_q,(long)s->probe_da,(long)s->probe_db,s->coupling_ready,
        s->path[0],s->path[1],s->path[2],s->path[3],(unsigned long)s->starved,
        s->motor_error, s->fault, (unsigned long)s->late, s->timer_error, s->reason);
}

int ct_ack_format(char *output, size_t capacity, uint32_t session, uint32_t job,
    const char *operation, bool ok, const char *reason)
{
    return snprintf(output, capacity,
        "{\"type\":\"ack\",\"protocol\":5,\"session\":%lu,\"job\":%lu,\"op\":\"%s\",\"ok\":%d,\"reason\":\"%s\"}\n",
        (unsigned long)session, (unsigned long)job, operation, ok, reason);
}
