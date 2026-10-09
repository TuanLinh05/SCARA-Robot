#include "cart_core_internal.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>
static int sign64(int64_t x) { return (x > 0) - (x < 0); }
static int64_t rounded(int64_t x, int64_t d)
{ return x >= 0 ? (x + d / 2) / d : -((-x + d / 2) / d); }
static unsigned raw_bits(const struct ct_state *c)
{
    unsigned bits=0;
    for (unsigned a=0; a<CT_AXES; a++)
        bits |= ((c->sw[a].raw_pos ? 1U : 0U) | (c->sw[a].raw_neg ? 2U : 0U)) << (2*a);
    return bits;
}
static void input_event(struct ct_state *c, unsigned axis, unsigned kind, uint32_t now)
{
    c->input_axis=(int)axis; c->input_bits=raw_bits(c);
    c->input_kind=kind; c->input_at_ms=now;
}
/* J1 affects the relative elbow through the belt, including compensation
 * with zero net elbow travel. Z does not change either arm joint. */
static bool involved(const struct ct_state *c, unsigned a)
{
    if (!c->initializing) return true;
    if (c->mode==CT_CAL) return a==c->selected || (a==CT_J2 && c->selected==CT_J1);
    if (c->mode==CT_PARK) return c->delta[a]!=0 || (a==CT_J2 && c->delta[CT_J1]!=0);
    return true;
}
static void pause_input(struct ct_state *c, unsigned a, uint32_t now, bool guard)
{
    if (!c->input_wait) {
        c->wait_running=c->running; c->wait_cm_dir=c->cm.direction; c->wait_cm_reason=c->cm.reason;
        c->wait_axes=0; c->wait_bits=0; c->wait_guard=false;
        c->wait_start_ms=c->wait_stable_ms=now; c->input_wait=true; c->running=false;
        input_event(c,a,1,now); c->reason="checking_inputs";
    }
    c->wait_axes |= 1U<<a; c->wait_guard |= guard;
}
bool ct_busy(const struct ct_state *c) { return c->running || c->initializing || c->path_active; }
void ct_init(struct ct_state *c)
{
    memset(c, 0, sizeof(*c));
    c->input_axis=-1;
    for (unsigned a = 0; a < CT_AXES; a++) {
        zj_init_joint(&c->sw[a]); c->holding[a] = true;
        /* J2 DIR HIGH turns opposite to J1. Counts use one geometric sign. */
        c->pol[a] = a != CT_J2;
    }
    zj_init_joint(&c->cm);
    c->nominal[0] = 1638400; c->nominal[1] = 13653; c->nominal[2] = 40960;
    c->micro[0] = 16; c->micro[1] = c->micro[2] = 8;
    for (unsigned j = 0; j < 2; j++) { c->low_md[j] = -90000; c->high_md[j] = 90000; }
    /* Motor B locked: absolute arm-2 heading=-A/3, relative elbow=-4A/3.
     * Keeping the ELBOW fixed therefore needs B=+4A/3. */
    c->coupling_ppm = 1333333; c->park_md[1] = 45000;
    c->auto_coupling=true; c->probe_pulses=128;
    c->z_clear_um = 3000; c->arm_rate = 200; c->z_rate = 4800; c->z_fine_rate=400;
    c->j2_rate=800; c->j2_fine_rate=200;
    c->arm_scan = 40000; c->z_scan = 300000; c->arm_back = 512; c->z_back = 1024;
    c->reason = c->reply = "unreferenced";
}
void ct_abort(struct ct_state *c, const char *reason)
{
    if (c->initializing) {
        hc_cancel(&c->cal[c->selected], &c->cm, ZJ_STOP, HMC_CANCELLED);
        c->stage = HS_FAILED;
    }
    c->running = c->initializing = c->referenced = false;
    c->input_wait=false;
    c->gate_wait=false;
    c->mode = CT_IDLE; c->reason = reason;
    c->path_active=c->path_started=false; c->path_size=0;
    memset(c->direction, 0, sizeof(c->direction));
}
void ct_sample(struct ct_state *c, unsigned a, int positive, int negative, uint32_t now)
{
    if (a >= CT_AXES) return;
    bool had_conflict=c->sw[a].conflict;
    bool had_both=c->sw[a].raw_pos && c->sw[a].raw_neg;
    int had_error=c->sw[a].error;
    zj_switches(&c->sw[a], positive, negative, now);
    if ((!had_conflict && c->sw[a].conflict) || (!had_error && c->sw[a].error)) {
        input_event(c,a,c->sw[a].error ? 4U : 2U,now);
        if (!ct_busy(c) && !c->referenced) c->reason=c->sw[a].error ? "switch_io" : "both_latched";
    }
    if ((c->sw[a].conflict || c->sw[a].error) && (ct_busy(c) || c->referenced)) {
        ct_abort(c,c->sw[a].error ? "switch_io" : "both_latched");
    } else if (c->referenced && positive==1 && negative==1) {
        /* Idle referenced states must remain valid telemetry too. Never
         * silently retain a reference with a newly invalid switch pair. */
        input_event(c,a,1,now); ct_abort(c,"both_open");
    }
    if (c->initializing && positive==1 && negative==1 && !c->sw[a].conflict) {
        if (involved(c,a)) pause_input(c,a,now,c->mode==CT_CAL && c->selected==CT_J1 && a==CT_J2);
        else if (!had_both) {
            if (c->idle_ignored<UINT32_MAX) c->idle_ignored++;
            input_event(c,a,6,now); /* stationary pair, sustained faults still latch */
        }
    } else if (c->initializing && ((c->mode==CT_CAL && c->selected==CT_J1) ||
        c->stage==HS_COUPLE_PROBE) && a==CT_J2 && (positive==1 || negative==1)) {
        pause_input(c,a,now,true);
    }
    if (c->initializing && c->mode == CT_CAL && a == c->selected) {
        int previous=c->cm.direction;
        zj_switches(&c->cm, positive, negative, now);
        if (previous && !c->cm.direction && !c->input_wait && (c->cm.reason==ZJ_TOP || c->cm.reason==ZJ_BOTTOM)) {
            c->last_hit_bits=raw_bits(c); c->last_hit_ms=now;
        }
    }
}
const char *ct_inputs(struct ct_state *c, uint32_t now)
{
    if (c->fault) return "motor_fault";
    for (unsigned a = 0; a < CT_AXES; a++) {
        const struct zj_state *s = &c->sw[a];
        if (!c->holding[a]) return "released";
        if (s->error) {
            if (c->input_axis!=(int)a || c->input_kind!=4) input_event(c,a,4,now);
            return "switch_io";
        }
        if (s->conflict) {
            if (c->input_axis!=(int)a || c->input_kind!=2) input_event(c,a,2,now);
            return "both_latched";
        }
        if (s->raw_pos && s->raw_neg && !c->input_wait && involved(c,a)) {
            input_event(c,a,1,now); return "both_open";
        }
        if (!zj_ready(s)) return "switch_not_ready";
        if (now - s->sample_ms > ZJ_SENSOR_MS) return "switch_stale";
    }
    return NULL;
}
static bool blocked(const struct ct_state *c, unsigned a, int joint_dir)
{
    if (!joint_dir) return false;
    bool positive = (joint_dir > 0) == c->pol[a];
    const struct zj_state *s = &c->sw[a];
    return positive ? s->top || s->raw_pos : s->bottom || s->raw_neg;
}
void ct_beta_update(struct ct_state *c)
{
    if(c->coupling_ready) return; /* pulse ratio is already measured, not nominal */
    uint32_t f1 = c->factor[1] ? c->factor[1] : c->nominal[1];
    uint32_t f2 = c->factor[2] ? c->factor[2] : c->nominal[2];
    /* Arm factors <=1e6 and coupling <=2e6; product fits signed int64. */
    c->beta = rounded((int64_t)c->coupling_ppm * f2 * CT_Q, (int64_t)1000000 * f1);
}
/* Model limits in motor coordinates. Physical switches still guard every edge. */
bool ct_pose_valid(const struct ct_state *c, const int32_t p[CT_AXES], int margin_md)
{
    int64_t z = (int64_t)p[0] - c->origin[0];
    if (z < 0 || z > c->range[0]) return false;
    int64_t a_md = rounded(((int64_t)p[1] - c->origin[1]) * 1024000, c->factor[1]);
    int64_t b_md = rounded(((int64_t)p[2] - c->origin[2]) * 1024000, c->factor[2]);
    int64_t elbow_md = b_md - rounded(a_md * c->coupling_ppm, 1000000);
    return a_md >= c->low_md[0] + margin_md && a_md <= c->high_md[0] - margin_md &&
        elbow_md >= c->low_md[1] + margin_md && elbow_md <= c->high_md[1] - margin_md;
}
bool ct_safe(struct ct_state *c, uint32_t now)
{
    if (!c->running) return false;
    const char *why = ct_inputs(c, now);
    if (!why && now - c->lease_ms >= CT_LEASE) why = "heartbeat_lost";
    if (!why && now - c->start_ms >= (c->initializing ? CT_INIT_MS : 600000U)) why = "time_budget";
    for (unsigned a = 0; a < CT_AXES && !why; a++)
        if (c->pos[a] <= -CT_MAX_POS || c->pos[a] >= CT_MAX_POS) why = "position_budget";
    if (why) { ct_abort(c, why); return false; }
    if (c->mode == CT_CAL) {
        if (!c->cm.direction || !zj_safe(&c->cm, now)) {
            c->running = false; return false; /* worker confirms/backoffs at endpoint */
        }
    } else {
        if(c->stage==HS_COUPLE_PROBE && blocked(c,CT_J1,c->direction[CT_J1])) {
            /* Near a J1 endpoint: drain HIGH, then retry the short probe away.
             * No reference is created by this provisional J1 contact. */
            c->running=false; c->probe_contact=true; return false;
        }
        for (unsigned a = 0; a < CT_AXES; a++) if (blocked(c, a, c->joint_dir[a])) {
            ct_abort(c, a == CT_Z ? "z_limit" : a == CT_J1 ? "j1_limit" : "j2_limit"); return false;
        }
        if (c->mode == CT_MOVE && (!c->referenced || !ct_pose_valid(c, c->pos, 0))) {
            ct_abort(c, "soft_limit"); return false;
        }
    }
    return true;
}
enum ct_result ct_raw_move(struct ct_state *c, const int32_t target[CT_AXES], uint32_t rate, bool park)
{
    c->tick = c->ticks = 0; c->rate = rate;
    c->ramp_tick=0;
    for (unsigned a = 0; a < CT_AXES; a++) {
        int64_t d = (int64_t)target[a] - c->pos[a];
        if (target[a] < -CT_MAX_POS || target[a] > CT_MAX_POS || llabs(d) > 1000000)
            return CT_REJECTED;
        c->goal[a] = target[a]; c->direction[a] = sign64(d);
        c->delta[a] = (uint32_t)llabs(d); c->accum[a] = 0;
        if (c->delta[a] > c->ticks) c->ticks = c->delta[a];
    }
    if (c->ticks > 590U * rate) return CT_REJECTED;
    c->joint_dir[0] = c->direction[0]; c->joint_dir[1] = c->direction[1];
    c->joint_dir[2] = sign64(((int64_t)target[2] - c->pos[2]) * CT_Q -
        c->beta * ((int64_t)target[1] - c->pos[1]));
    c->mode = park ? CT_PARK : CT_MOVE; c->running = c->ticks != 0;
    return c->running ? CT_START : CT_OK;
}
/* Max derivative of quintic smootherstep is 15/8. v^2(s) ramps
 * therefore need ceil(15 * (peak^2-end^2) / (16 * acceleration)) ticks. */
uint32_t ct_path_ramp(uint32_t peak, uint32_t end, uint32_t accel)
{ return (uint32_t)((15ULL*(peak*peak-end*end)+16U*accel-1U)/(16U*accel)); }
bool ct_path_advance(struct ct_state *c)
{
    if (!c->path_active) return false;
    if (c->path_started && c->tick < c->ticks) return false;
    if (c->path_started) c->path_done++;
    if (c->path_done == c->path_total) {
        c->path_active=c->path_started=c->running=false;
        c->reason="complete"; return false;
    }
    if (!c->path_size) {
        if (c->path_starved<UINT32_MAX) c->path_starved++;
        ct_abort(c,"path_starved"); return false;
    }
    const struct ct_segment *s=&c->path_queue[c->path_head];
    enum ct_result r=ct_raw_move(c,s->target,s->peak,false);
    c->path_accel=s->accel; c->path_entry=s->entry; c->path_exit=s->exit;
    c->path_up=ct_path_ramp(s->peak,s->entry,s->accel);
    c->path_down=ct_path_ramp(s->peak,s->exit,s->accel);
    c->path_head=(c->path_head+1U)%CT_PATH_CAP; c->path_size--;
    c->path_started=true;
    if (r!=CT_START) { ct_abort(c,"path_invalid"); return false; }
    return true;
}
static enum ct_result cal_drive(struct ct_state *c)
{
    memset(c->direction, 0, sizeof(c->direction));
    memset(c->cal_weight, 0, sizeof(c->cal_weight));
    memset(c->accum, 0, sizeof(c->accum));
    unsigned a = c->selected;
    c->direction[a] = a == CT_Z ? -c->cm.direction : c->cm.direction;
    c->cal_weight[a] = CT_Q; c->cal_scale = CT_Q;
    if (a == CT_J1) {
        c->direction[2] = c->direction[1] * sign64(c->beta);
        c->cal_weight[2] = llabs(c->beta);
        if (c->cal_weight[2] > c->cal_scale) c->cal_scale = c->cal_weight[2];
    }
    c->running = c->cm.direction != 0;
    c->ramp_ms=c->cm.start_ms;
    return c->running ? CT_START : CT_OK;
}
static bool pair_stable(const struct zj_state *s)
{ return !(s->raw_pos && s->raw_neg) && s->top==(bool)s->raw_pos && s->bottom==(bool)s->raw_neg; }
static bool gate_ready(struct ct_state *c, unsigned a, bool clear, uint32_t now)
{
    const struct zj_state *s=&c->sw[a];
    if (pair_stable(s) && (!clear || (!s->raw_pos && !s->raw_neg))) {
        if (c->gate_wait && c->gate_axis==a) c->gate_wait=false;
        return true;
    }
    if (!c->gate_wait || c->gate_axis!=a) { c->gate_start_ms=now; c->gate_axis=a; c->gate_wait=true; }
    c->reason="waiting_switch";
    if (now-c->gate_start_ms>=CT_INPUT_WAIT_MS) {
        input_event(c,a,7,now); ct_abort(c,clear ? "j2_guard" : "switch_transition_unstable");
    }
    return false;
}
static bool park_ready(struct ct_state *c, const int32_t target[CT_AXES], uint32_t now)
{
    for(unsigned a=0;a<CT_AXES;a++)
        if ((target[a]!=c->pos[a] || (a==CT_J2 && target[CT_J1]!=c->pos[CT_J1])) && !gate_ready(c,a,false,now)) return false;
    c->gate_wait=false; return true;
}
enum ct_result ct_begin_cal(struct ct_state *c, unsigned a, enum ct_stage stage, uint32_t now)
{
    if (!gate_ready(c,a,false,now) || (a==CT_J1 && !gate_ready(c,CT_J2,true,now)))
        return c->initializing ? CT_OK : CT_REJECTED;
    c->gate_wait=false;
    c->selected = a; c->stage = stage; c->mode = CT_CAL;
    c->cm = c->sw[a]; c->cm.position = 0; c->cm.total = 0; c->cm.direction = 0;
    /* Z worker negative endpoint is UP, so it finishes at the upper switch. */
    c->cm.positive_high = a == CT_Z ? !c->pol[a] : c->pol[a];
    c->cal[a].lease_ms = now;
    c->cal[a].fine_rate=a==CT_Z ? c->z_fine_rate : a==CT_J2 ? c->j2_fine_rate : ZJ_MIN_RATE;
    uint32_t span = a == CT_Z ? 1000U : (uint32_t)(c->high_md[a - 1] - c->low_md[a - 1]);
    enum zj_command_result r = hc_start(&c->cal[a], &c->cm, span,
        a == CT_Z ? c->z_rate : a==CT_J2 ? c->j2_rate : c->arm_rate, a == CT_Z ? c->z_scan : c->arm_scan,
        a == CT_Z ? c->z_back : c->arm_back, c->micro[a], now);
    c->cal[a].lease_ms = c->cm.lease_ms = c->lease_ms;
    if (r == ZJ_REJECTED) { ct_abort(c, "cal_start"); return CT_REJECTED; }
    if (!c->cm.direction && (c->cm.reason==ZJ_TOP || c->cm.reason==ZJ_BOTTOM)) {
        c->last_hit_bits=raw_bits(c); c->last_hit_ms=now;
    }
    c->reason = "initializing";
    return cal_drive(c);
}
static enum ct_result init_fail(struct ct_state *c, const char *reason)
{ ct_abort(c, reason); return CT_REJECTED; }
static enum ct_result stationary_check(struct ct_state *c, uint32_t now)
{
    unsigned mask=0;
    for (unsigned a=0; a<CT_AXES; a++) if (c->wait_axes & (1U<<a)) mask |= 3U << (2*a);
    unsigned bits=raw_bits(c)&mask;
    if (bits!=c->wait_bits) { c->wait_bits=bits; c->wait_stable_ms=now; }
    bool double_open=false;
    for (unsigned a=0; a<CT_AXES; a++) if (((bits>>(2*a))&3U)==3U) double_open=true;
    if (now-c->wait_start_ms>=CT_INPUT_WAIT_MS) {
        c->input_kind=3; return init_fail(c,"switch_unstable");
    }
    if (double_open || now-c->wait_stable_ms<ZJ_RELEASE_MS) return CT_OK;
    if (c->wait_guard && (c->sw[CT_J2].raw_pos || c->sw[CT_J2].raw_neg)) {
        input_event(c,CT_J2,7,now); return init_fail(c,"j2_guard");
    }
    c->input_wait=false; if (c->recovered<UINT32_MAX) c->recovered++;
    c->reason="initializing"; c->ramp_ms=now; c->ramp_tick=c->tick;
    c->running=c->wait_running;
    if (c->mode==CT_CAL) {
        c->cm.direction=c->wait_cm_dir; c->cm.reason=c->wait_cm_reason;
        /* Never inherit a stale HIGH confirmation across the pause. */
        c->cal[c->selected].settling=false;
        if (c->cm.direction) {
            bool physical_positive=(c->cm.direction>0)==c->cm.positive_high;
            bool hit=physical_positive ? c->cm.raw_pos : c->cm.raw_neg;
            if (hit) {
                c->cm.direction=0;
                c->cm.reason=physical_positive ? ZJ_TOP : ZJ_BOTTOM;
                c->last_hit_bits=raw_bits(c); c->last_hit_ms=now;
            }
        }
        if (!c->cm.direction) c->running=false;
    }
    return c->running ? CT_START : CT_OK;
}
enum ct_result ct_service(struct ct_state *c, uint32_t now)
{
    if (c->path_active && !c->path_started) {
        const char *why=ct_inputs(c,now);
        if (!why && now-c->lease_ms>=CT_LEASE) why="heartbeat_lost";
        if (why) { ct_abort(c,why); return CT_REJECTED; }
        return CT_OK;
    }
    if (!c->initializing) return CT_OK;
    const char *why = ct_inputs(c, now);
    if (!why && now - c->lease_ms >= CT_LEASE) why = "heartbeat_lost";
    if (!why && now - c->start_ms >= CT_INIT_MS) why = "time_budget";
    if (why) return init_fail(c, why);
    if (c->input_wait) {
        enum ct_result checked=stationary_check(c,now);
        if (c->input_wait || checked!=CT_OK) return checked;
    }
    if (c->mode == CT_CAL) {
        enum zj_command_result r = hc_tick(&c->cal[c->selected], &c->cm, now);
        if (c->cal[c->selected].phase == HMC_FAILED) {
            if (c->cal[c->selected].error==HMC_UNSTABLE) {
                c->input_axis=(int)c->selected; c->input_kind=5;
                c->input_bits=c->last_hit_bits; c->input_at_ms=c->last_hit_ms;
            }
            return init_fail(c, "calibration_failed");
        }
        if (r == ZJ_STARTED) return cal_drive(c);
        if (r == ZJ_RESUMED) {
            /* Same leg: retain DDA phase, direction, counts and search limit. */
            c->direction[c->selected]=c->selected==CT_Z ? -c->cm.direction : c->cm.direction;
            if (c->selected==CT_J1) c->direction[CT_J2]=c->cm.direction*sign64(c->beta);
            c->running=true; c->ramp_ms=now; c->reason="initializing";
            if (c->recovered<UINT32_MAX) c->recovered++;
            return CT_START;
        }
        if (hc_active(&c->cal[c->selected])) return CT_OK;
        unsigned a = c->selected;
        c->range[a] = c->cal[a].pulses;
        uint64_t f = a == CT_Z ? (c->z_span_um ? (uint64_t)c->range[a] * 1024000 / c->z_span_um : c->nominal[a]) :
            (uint64_t)c->range[a] * 1024000 / (uint32_t)(c->high_md[a - 1] - c->low_md[a - 1]);
        if (f < 1 || f > (a == CT_Z ? 100000000U : 1000000U)) return init_fail(c, "scale_bounds");
        c->factor[a] = (uint32_t)f;
        if((a==CT_J1 || c->stage==HS_J2_FINAL) && c->coupling_ready) {
            int64_t k=rounded((int64_t)c->probe_db*sign64(c->probe_da)*c->factor[1]*1000000,
                (int64_t)abs(c->probe_da)*c->factor[2]);
            if(llabs(k)>2000000) return init_fail(c,"coupling_geometry");
            c->coupling_ppm=(int32_t)k;
        }
        if (a == CT_Z) c->origin[a] = c->pos[a] - (int32_t)c->range[a];
        if (a == CT_J1) c->origin[a] = c->pos[a] - (int32_t)rounded((int64_t)c->low_md[0] * c->factor[a], 1024000);
        if (c->stage == HS_J2_FINAL) {
            int64_t b_md = c->low_md[1] + rounded((int64_t)c->park_md[0] * c->coupling_ppm, 1000000);
            c->origin[2] = c->pos[2] - (int32_t)rounded(b_md * c->factor[2], 1024000);
        }
        ct_beta_update(c);
        if (llabs(c->beta) > 64 * CT_Q) return init_fail(c, "coupling_bounds");
    } else if (c->running) return CT_OK;
    int32_t target[CT_AXES]; memcpy(target, c->pos, sizeof(target));
    enum ct_result result = CT_OK;
    switch (c->stage) {
    case HS_Z: return ct_begin_cal(c, CT_J2, HS_J2_PRE, now);
    case HS_J2_PRE:
        c->probe_range=c->range[2];
        c->probe_b0=c->pos[2]; c->probe_mid=c->pos[2]+(int32_t)c->range[2]/2;
        target[2] += (int32_t)c->range[2] / 2;
        if (!park_ready(c,target,now)) return c->initializing ? CT_OK : CT_REJECTED;
        c->stage = HS_J2_MID;
        result = ct_raw_move(c, target, 200, true); break;
    case HS_J2_MID:
        if(!c->auto_coupling) return ct_begin_cal(c,CT_J1,HS_J1,now);
        if(!gate_ready(c,CT_J1,false,now) || !gate_ready(c,CT_J2,true,now)) return CT_OK;
        c->probe_a0=c->pos[1]; c->probe_da=c->probe_db=0;
        c->probe_contact=c->probe_retry=false;
        target[1]+=blocked(c,CT_J1,1) ? -(int32_t)c->probe_pulses : (int32_t)c->probe_pulses;
        c->stage=HS_COUPLE_PROBE; result=ct_raw_move(c,target,50,true); break;
    case HS_COUPLE_PROBE:
        if(c->probe_contact) {
            if(c->probe_retry) return init_fail(c,"coupling_probe_limit");
            c->probe_retry=true; c->probe_contact=false;
            int d=c->direction[1];
            target[1]=c->probe_a0-d*(int32_t)c->probe_pulses;
            if(!park_ready(c,target,now)) { c->probe_contact=true; c->probe_retry=false; return CT_OK; }
            result=ct_raw_move(c,target,50,true); break;
        }
        c->probe_da=c->pos[1]-c->probe_a0;
        if(abs(c->probe_da)<16) return init_fail(c,"coupling_probe_short");
        return ct_begin_cal(c,CT_J2,HS_COUPLE_SCAN,now);
    case HS_COUPLE_SCAN: {
        uint32_t old_range=c->probe_range, new_range=c->range[2];
        if(abs((int)old_range-(int)new_range)>(int)(c->range[2]/100+4)) return init_fail(c,"coupling_repeat");
        c->probe_db=c->pos[2]-c->probe_b0;
        int64_t measured=rounded((int64_t)c->probe_db*CT_Q*sign64(c->probe_da),abs(c->probe_da));
        /* A noisy or contradictory measurement must not launch a long sweep.
         * The signed result is retained in telemetry even on failure. */
        if(llabs(measured)>64*CT_Q || abs(c->probe_db)<8) {
            c->beta=0; return init_fail(c,"coupling_measurement");
        }
        c->beta=measured;
        if(sign64(measured)!=sign64(c->coupling_ppm)) return init_fail(c,"coupling_sign");
        c->coupling_ready=true;
        target[1]=c->probe_a0; target[2]=c->probe_mid;
        if(!park_ready(c,target,now)) return CT_OK;
        c->stage=HS_COUPLE_RETURN; result=ct_raw_move(c,target,200,true); break;
    }
    case HS_COUPLE_RETURN: return ct_begin_cal(c,CT_J1,HS_J1,now);
    case HS_J1:
        target[1] = c->origin[1] + (int32_t)rounded((int64_t)c->park_md[0] * c->factor[1], 1024000);
        target[2] += (int32_t)rounded(((int64_t)target[1] - c->pos[1]) * c->beta, CT_Q);
        if (!park_ready(c,target,now)) return c->initializing ? CT_OK : CT_REJECTED;
        c->stage = HS_J1_MID;
        result = ct_raw_move(c, target, 400, true); break;
    case HS_J1_MID: return ct_begin_cal(c, CT_J2, HS_J2_FINAL, now);
    case HS_J2_FINAL: {
        int64_t b_md = c->park_md[1] + rounded((int64_t)c->park_md[0] * c->coupling_ppm, 1000000);
        target[2] = c->origin[2] + (int32_t)rounded(b_md * c->factor[2], 1024000);
        int64_t clearance = rounded((int64_t)c->z_clear_um * c->factor[0], 1024000);
        if (clearance < 16 || clearance >= c->range[0] / 2) return init_fail(c, "z_clearance");
        target[0] = c->origin[0] + (int32_t)c->range[0] - (int32_t)clearance;
        if (!park_ready(c,target,now)) return c->initializing ? CT_OK : CT_REJECTED;
        c->stage = HS_PARK;
        result = ct_raw_move(c, target, 800, true); break;
    }
    case HS_PARK:
        if (!ct_pose_valid(c, c->pos, 1000)) return init_fail(c, "park_bounds");
        c->initializing = false; c->referenced = true; c->reference_epoch++;
        c->stage = HS_READY; c->mode = CT_IDLE; c->reason = "initialized"; return CT_OK;
    default: return init_fail(c, "stage_error");
    }
    return result == CT_REJECTED ? init_fail(c, "park_budget") : result;
}
unsigned ct_next_mask(struct ct_state *c)
{
    unsigned mask = 0;
    uint64_t scale = c->mode == CT_CAL ? (uint64_t)c->cal_scale : c->ticks;
    if (!scale) return 0;
    int32_t next[CT_AXES]; memcpy(next, c->pos, sizeof(next));
    for (unsigned a = 0; a < CT_AXES; a++) {
        uint64_t w = c->mode == CT_CAL ? (uint64_t)c->cal_weight[a] : c->delta[a];
        c->accum[a] += w;
        if (c->accum[a] >= scale) {
            c->accum[a] -= scale; mask |= 1U << a; next[a] += c->direction[a];
        }
    }
    if (c->mode == CT_MOVE && !ct_pose_valid(c, next, 0)) { ct_abort(c, "soft_limit"); return 0; }
    if (c->mode != CT_CAL) {
        /* DDA rounding can move the elbow by one microstep even with a zero
         * net elbow delta. Guard this individual edge, not just the segment. */
        int elbow_edge = sign64(((int64_t)next[2] - c->pos[2]) * CT_Q -
            c->beta * ((int64_t)next[1] - c->pos[1]));
        if(c->stage==HS_COUPLE_PROBE && blocked(c,CT_J1,sign64((int64_t)next[1]-c->pos[1]))) {
            c->running=false; c->probe_contact=true; return 0;
        }
        if (blocked(c, 0, sign64((int64_t)next[0] - c->pos[0])) ||
            blocked(c, 1, sign64((int64_t)next[1] - c->pos[1])) || blocked(c, 2, elbow_edge)) {
            ct_abort(c, "edge_limit"); return 0;
        }
    }
    return mask;
}
void ct_count(struct ct_state *c, unsigned a)
{
    c->pos[a] += c->direction[a]; c->total[a]++;
    if (c->mode == CT_CAL && a == c->selected) zj_count_pulse(&c->cm);
}
void ct_finish_tick(struct ct_state *c)
{
    if (c->mode == CT_CAL) { if (!c->cm.direction) c->running = false; return; }
    if (++c->tick >= c->ticks && !c->path_active) { c->running = false; c->reason = c->initializing ? "initializing" : "complete"; }
}
static uint32_t root32(uint32_t n)
{
    uint32_t result=0, bit=1U<<30;
    while (bit>n) bit>>=2;
    while (bit) {
        if (n>=result+bit) { n-=result+bit; result=(result>>1)+bit; }
        else result>>=1;
        bit>>=2;
    }
    return result;
}
static uint32_t blend20(uint32_t a, uint32_t b, uint32_t u)
{ return a+(uint32_t)((uint64_t)(b-a)*u>>20); }
static uint32_t smooth_square(uint32_t low, uint32_t high, uint32_t tick, uint32_t length)
{
    if (!length || tick>=length) return high*high;
    uint32_t u=(uint32_t)((uint64_t)tick*1048576U/length);
    /* de Casteljau for controls [0,0,0,1,1,1]. Positive convex blends
     * stay monotone in fixed point, unlike cancellation in 6u^5-15u^4+10u^3. */
    uint32_t p=blend20(0,u,u), q=blend20(u,1048576U,u);
    uint32_t a=blend20(0,p,u), b=blend20(p,q,u), d=blend20(q,1048576U,u);
    uint32_t s=blend20(blend20(a,b,u),blend20(b,d,u),u);
    return low*low+(uint32_t)((uint64_t)(high*high-low*low)*s>>20);
}
uint32_t ct_period_us(const struct ct_state *c, uint32_t now)
{
    uint32_t rate;
    if (c->mode == CT_CAL) {
        uint32_t elapsed=now-c->cm.start_ms, restarted=now-c->ramp_ms;
        if (restarted<elapsed) elapsed=restarted;
        elapsed=elapsed>ZJ_SETUP_MS ? elapsed-ZJ_SETUP_MS : 0;
        if (elapsed>ZJ_RAMP_MS) elapsed=ZJ_RAMP_MS;
        uint32_t initial=c->cal[c->selected].fine_rate;
        if (initial>c->cm.rate) initial=c->cm.rate;
        uint32_t primary=initial+(c->cm.rate-initial)*elapsed/ZJ_RAMP_MS;
        rate=(uint32_t)((uint64_t)primary*c->cal_scale/CT_Q);
    } else if (c->path_active && c->path_started) {
        uint32_t up=smooth_square(c->path_entry,c->rate,c->tick,c->path_up);
        uint32_t down=smooth_square(c->path_exit,c->rate,c->ticks-c->tick,c->path_down);
        uint32_t square=up<down ? up : down;
        rate=root32(square);
    } else {
        uint32_t since_restart=c->tick-c->ramp_tick;
        uint32_t distance = since_restart < c->ticks - c->tick ? since_restart : c->ticks - c->tick;
        uint32_t ramp = distance > 200 ? 200 : distance;
        uint32_t initial=!c->delta[1] && !c->delta[2] ? c->z_fine_rate : 50U;
        if (initial>c->rate) initial=c->rate;
        rate = initial + (c->rate - initial) * ramp / 200;
    }
    if (rate < 50) rate = 50;
    uint32_t ceiling=c->mode==CT_CAL && c->selected==CT_Z ? CT_Z_CAL_RATE : CT_MAX_RATE;
    if (rate > ceiling) rate = ceiling;
    return c->mode==CT_CAL && c->selected==CT_Z ? (1000000U+rate-1U)/rate : 1000000U/rate;
}
