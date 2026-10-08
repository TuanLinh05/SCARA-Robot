#include "home_worker.h"
#include <limits.h>
bool hc_active(const struct hc_state *c) { return c->phase >= HMC_SEEK_N && c->phase <= HMC_LATCH_N2; }
void hc_cancel(struct hc_state *c, struct zj_state *s, enum zj_reason reason, enum hc_error error)
{
    if (!hc_active(c)) return;
    c->failed_phase = c->phase; c->phase = HMC_FAILED; c->error = error; c->reference = false;
    s->job_budget_ms = 0; zj_stop(s, reason);
}
static bool logical_limit(const struct zj_state *s, int direction, bool raw)
{
    bool positive = (direction > 0) == s->positive_high;
    return positive ? (raw ? s->raw_pos : s->top) : (raw ? s->raw_neg : s->bottom);
}
static enum zj_reason endpoint(const struct zj_state *s, int direction)
{ return (direction > 0) == s->positive_high ? ZJ_TOP : ZJ_BOTTOM; }
static enum zj_command_result leg(struct hc_state *c, struct zj_state *s, enum hc_phase phase, uint32_t now)
{
    c->phase = phase; c->settling = c->extra = c->arrived = false;
    bool clearing = phase == HMC_CLEAR_N || phase == HMC_CLEAR_N2 || phase == HMC_CLEAR_P;
    int direction = (phase == HMC_CLEAR_N || phase == HMC_CLEAR_N2 || phase == HMC_SWEEP_P || phase == HMC_LATCH_P) ? 1 : -1;
    uint32_t limit = clearing ? c->backoff :
        (phase == HMC_LATCH_N || phase == HMC_LATCH_P || phase == HMC_LATCH_N2) ? c->backoff * 2U : c->max_scan;
    uint32_t rate = phase == HMC_SEEK_N || phase == HMC_SWEEP_P || phase == HMC_SWEEP_N ? c->rate : c->fine_rate;
    s->job_budget_ms = HMC_PHASE_MS;
    enum zj_command_result result = zj_begin(s, direction, rate, limit, now);
    s->start_ms = now; /* Also bound a settle at an already-active first stop. */
    s->lease_ms = c->lease_ms; /* Internal transitions never renew the host lease. */
    if (result == ZJ_REJECTED && !(phase == HMC_SEEK_N && s->reason == endpoint(s, -1)))
        hc_cancel(c, s, s->reason, HMC_INPUT);
    return result == ZJ_STARTED ? ZJ_STARTED : ZJ_OK;
}
enum zj_command_result hc_start(struct hc_state *c, struct zj_state *s,
    uint32_t angle, uint32_t rate, uint32_t max_scan, uint32_t backoff, uint32_t micro, uint32_t now)
{
    if (!c->fine_rate) c->fine_rate=ZJ_MIN_RATE;
    if (hc_active(c) || s->direction || angle < 1000U || angle > 360000U || rate < 50U || rate > ZJ_MAX_RATE ||
        c->fine_rate<ZJ_MIN_RATE || c->fine_rate>rate ||
        max_scan < 256U || max_scan > 1000000U || backoff < 32U || backoff > 8192U || backoff >= max_scan ||
        (micro != 1 && micro != 2 && micro != 4 && micro != 8 && micro != 16 && micro != 32) || c->epoch == UINT32_MAX)
        return ZJ_REJECTED;
    c->reference = false; c->n1 = c->n2 = c->false_hits = 0; c->error = HMC_NONE; c->failed_phase = HMC_IDLE;
    c->run_angle = angle; c->run_micro = micro; c->rate = rate; c->max_scan = max_scan; c->backoff = backoff;
    c->start_ms = c->lease_ms = now; c->start_total = s->total;
    return leg(c, s, HMC_SEEK_N, now);
}
enum zj_command_result hc_tick(struct hc_state *c, struct zj_state *s, uint32_t now)
{
    if (!hc_active(c)) return ZJ_OK;
    if (now - c->lease_ms >= ZJ_LEASE_MS) { hc_cancel(c, s, ZJ_TIMEOUT, HMC_LINK); return ZJ_OK; }
    bool both_raw = s->raw_pos && s->raw_neg;
    if (s->error || s->conflict || both_raw || !zj_ready(s) || now - s->sample_ms > ZJ_SENSOR_MS) {
        hc_cancel(c, s, s->error ? ZJ_SENSOR : s->conflict || both_raw ? ZJ_CONFLICT : !zj_ready(s) ? ZJ_NOT_READY : ZJ_STALE_SENSOR, HMC_INPUT);
        return ZJ_OK;
    }
    if (now - c->start_ms >= HMC_TOTAL_MS || now - s->start_ms >= HMC_PHASE_MS ||
        s->total - c->start_total >= (uint64_t)c->max_scan * 3U + (uint64_t)c->backoff * 9U) {
        hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS); return ZJ_OK;
    }
    bool clearing = c->phase == HMC_CLEAR_N || c->phase == HMC_CLEAR_N2 || c->phase == HMC_CLEAR_P;
    int toward = (c->phase == HMC_SWEEP_P || c->phase == HMC_LATCH_P || c->phase == HMC_CLEAR_P) ? 1 : -1;
    if (clearing) {
        if (s->direction) {
            if (!c->extra && !logical_limit(s, toward, false) && !logical_limit(s, toward, true)) {
                if (s->move > c->backoff - 16U) { hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS); return ZJ_OK; }
                s->target = s->move + 16U; c->extra = true;
            }
            return ZJ_OK;
        }
        if (s->reason != ZJ_DONE || !c->extra || logical_limit(s, toward, true)) {
            hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS); return ZJ_OK;
        }
        return leg(c, s, c->phase == HMC_CLEAR_P ? HMC_LATCH_P : c->phase == HMC_CLEAR_N2 ? HMC_LATCH_N2 : HMC_LATCH_N, now);
    }
    if (s->direction) return ZJ_OK;
    if (s->reason != endpoint(s, toward) || (!s->move && c->phase != HMC_SEEK_N)) {
        enum hc_error error = s->reason == ZJ_SENSOR || s->reason == ZJ_STALE_SENSOR ||
            s->reason == ZJ_NOT_READY || s->reason == ZJ_CONFLICT ? HMC_INPUT :
            s->reason == ZJ_TIMEOUT || s->reason == ZJ_USB ? HMC_LINK :
            s->reason == ZJ_GPIO || s->reason == ZJ_TIMER ? HMC_IO : HMC_BOUNDS;
        hc_cancel(c, s, s->reason == ZJ_TOP || s->reason == ZJ_BOTTOM ? ZJ_CAL : s->reason, error); return ZJ_OK;
    }
    /* First HIGH already stopped the ISR. Use GPIO sampling history, not
     * main-loop cadence, to confirm the contact. A transient that closes
     * stably is not a measured endpoint: resume without resetting budgets. */
    if (!c->arrived) { c->arrived = true; c->hit_ms = now; }
    bool positive=(toward>0)==s->positive_high;
    bool high=logical_limit(s,toward,true);
    uint32_t opened=positive ? s->opened_ms : s->opened_bottom_ms;
    uint32_t closed=positive ? s->closed_ms : s->closed_bottom_ms;
    bool closing=positive ? s->closing : s->closing_bottom;
    if (!high && closing && !logical_limit(s,toward,false) && now-closed>=ZJ_RELEASE_MS) {
        if (s->target && s->move>=s->target) { hc_cancel(c,s,ZJ_CAL,HMC_BOUNDS); return ZJ_OK; }
        s->direction=toward; s->reason=ZJ_IDLE; c->arrived=c->settling=false;
        if (c->false_hits<UINT32_MAX) c->false_hits++;
        return ZJ_RESUMED;
    }
    if (!(high && now-opened>=ZJ_RELEASE_MS)) {
        if (now-c->hit_ms>=HMC_CONFIRM_MS) hc_cancel(c,s,ZJ_CAL,HMC_UNSTABLE);
        return ZJ_OK;
    }
    switch (c->phase) {
    case HMC_SEEK_N: return leg(c, s, HMC_CLEAR_N, now);
    case HMC_LATCH_N:
        c->negative_anchor = s->position; return leg(c, s, HMC_SWEEP_P, now);
    case HMC_SWEEP_P: return leg(c, s, HMC_CLEAR_P, now);
    case HMC_LATCH_P: {
        if (s->position <= c->negative_anchor) { hc_cancel(c, s, ZJ_CAL, HMC_REPEAT); return ZJ_OK; }
        uint64_t distance = (uint64_t)s->position - (uint64_t)c->negative_anchor;
        if (distance < 64U || distance > c->max_scan) { hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS); return ZJ_OK; }
        c->n1 = (uint32_t)distance; c->positive_anchor = s->position;
        return leg(c, s, HMC_SWEEP_N, now);
    }
    case HMC_SWEEP_N: return leg(c, s, HMC_CLEAR_N2, now);
    case HMC_LATCH_N2: {
        if (c->positive_anchor <= s->position) { hc_cancel(c, s, ZJ_CAL, HMC_REPEAT); return ZJ_OK; }
        uint64_t distance = (uint64_t)c->positive_anchor - (uint64_t)s->position;
        if (distance < 64U || distance > c->max_scan) { hc_cancel(c, s, ZJ_CAL, HMC_BOUNDS); return ZJ_OK; }
        c->n2 = (uint32_t)distance;
        uint32_t mean = (c->n1 + c->n2 + 1U) / 2U;
        uint32_t difference = c->n1 > c->n2 ? c->n1 - c->n2 : c->n2 - c->n1;
        uint32_t tolerance = mean / 100U; if (tolerance < 4U) tolerance = 4U;
        if (difference > tolerance) { hc_cancel(c, s, ZJ_CAL, HMC_REPEAT); return ZJ_OK; }
        c->pulses = mean; c->angle_mdeg = c->run_angle; c->micro = c->run_micro;
        c->origin = s->position; c->reference = true; c->epoch++;
        c->phase = HMC_DONE; c->error = HMC_NONE; s->job_budget_ms = 0; zj_stop(s, ZJ_DONE);
        return ZJ_OK;
    }
    default: hc_cancel(c, s, ZJ_CAL, HMC_BAD_COMMAND); return ZJ_OK;
    }
}

