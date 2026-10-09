#include "cart_core_internal.h"
#include "command_text.h"
#include <stdlib.h>
#include <string.h>

static enum ct_result reject(struct ct_state *c, const char *why)
{
    c->reply = why; if (ct_busy(c)) ct_abort(c, why);
    return CT_REJECTED;
}
void ct_hold_complete(struct ct_state *c, bool success)
{
    if (success) c->holding[c->hold_axis] = c->hold_value;
    else { c->fault = true; ct_abort(c, "motor_fault"); }
}
enum ct_result ct_command(struct ct_state *c, const char *line, uint32_t now)
{
    char buffer[192], *w[13]; unsigned n;
    if (!command_split(line, buffer, sizeof(buffer), w, 13, &n, false))
        return reject(c, "invalid_command");
    c->reply = "ok";
    if (!strcmp(w[0], "STOP")) {
        const char *why="stopped";
        if (n==2) {
            const char *tokens[]={"USER","ESC","FOCUS","MINIMIZE","DISCONNECT","CLOSE","STALE","ACK","MODEL"};
            const char *reasons[]={"stop_user","stop_escape","stop_focus","stop_minimize","stop_disconnect","stop_close","stop_stale","stop_ack_timeout","stop_model"};
            bool found=false;
            for(unsigned i=0;i<sizeof(tokens)/sizeof(tokens[0]);i++)
                if (!strcmp(w[1],tokens[i])) { why=reasons[i]; found=true; break; }
            if (!found) return reject(c,"invalid_command");
        } else if (n!=1) return reject(c,"invalid_command");
        if (ct_busy(c)) ct_abort(c, why);
        else if (n==2 || strncmp(c->reason,"stop_",5)) c->reason=why;
        return CT_OK;
    }
    if (n == 1 && !strcmp(w[0], "STATUS")) return CT_OK;
    int64_t v[14] = {0};
    if (n < 2 || !command_number_i64(w[1], 1, UINT32_MAX, &v[1])) return reject(c, "session");
    if (!strcmp(w[0], "HELLO") && n == 2) {
        if (ct_busy(c)) ct_abort(c, "session_changed");
        c->session = (uint32_t)v[1]; c->last_job = c->job = 0; return CT_OK;
    }
    if (!c->session || v[1] != c->session) return reject(c, "session");
    if (!strcmp(w[0], "KEEP") && n == 3) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || v[2] != c->job) return reject(c, "job");
        c->lease_ms = now;
        if (c->initializing && c->mode == CT_CAL)
            c->cm.lease_ms = c->cal[c->selected].lease_ms = now;
        return CT_OK;
    }
    if (!strcmp(w[0],"PATH") && n==4) {
        if (!command_number_i64(w[2],1,UINT32_MAX,&v[2]) || v[2]<=c->last_job) return reject(c,"job");
        if (ct_busy(c)) return reject(c,"busy");
        if (!c->referenced) return reject(c,"unreferenced");
        const char *why=ct_inputs(c,now); if(why) return reject(c,why);
        if (!command_number_i64(w[3],1,500,&v[3])) return reject(c,"path_invalid");
        c->job=c->last_job=(uint32_t)v[2]; c->lease_ms=c->start_ms=now;
        c->path_total=(uint16_t)v[3]; c->path_received=c->path_done=c->path_head=c->path_size=0;
        memcpy(c->path_tail,c->pos,sizeof(c->pos));
        c->path_active=true; c->path_started=false; c->mode=CT_MOVE;
        c->reason="path_loading"; return CT_OK;
    }
    if (!strcmp(w[0],"SEG") && n==11) {
        if (!c->path_active || !command_number_i64(w[2],1,UINT32_MAX,&v[2]) || v[2]!=c->job ||
            !command_number_i64(w[3],0,499,&v[3]) || v[3]!=c->path_received || c->path_received>=c->path_total ||
            c->path_size>=CT_PATH_CAP) return reject(c,"path_sequence");
        struct ct_segment s={0}; uint32_t ticks=0;
        for(unsigned a=0;a<CT_AXES;a++) {
            if(!command_number_i64(w[a+4],-CT_MAX_POS,CT_MAX_POS,&v[a+4])) return reject(c,"target");
            int64_t p=v[a+4]+c->origin[a],d=p-c->path_tail[a];
            if(p<-CT_MAX_POS || p>CT_MAX_POS || llabs(d)>1000000) return reject(c,"target");
            s.target[a]=(int32_t)p; if(llabs(d)>ticks) ticks=(uint32_t)llabs(d);
        }
        /* A stroke cannot change paper Z. Pen lifts use ordinary MOVE. */
        if (!ticks || s.target[CT_Z]!=c->path_tail[CT_Z] || !ct_pose_valid(c,s.target,1000)) return reject(c,"path_invalid");
        if (!command_number_i64(w[7],50,CT_MAX_RATE,&v[7]) || !command_number_i64(w[8],50,v[7],&v[8]) ||
            !command_number_i64(w[9],50,v[7],&v[9]) || !command_number_i64(w[10],50,200000,&v[10])) return reject(c,"path_profile");
        if ((!c->path_received && v[8]!=50) || (c->path_received+1U==c->path_total && v[9]!=50)) return reject(c,"path_profile");
        if (llabs(v[8]*v[8]-v[9]*v[9])>2*v[10]*ticks) return reject(c,"path_profile");
        if(ct_path_ramp((uint32_t)v[7],(uint32_t)v[8],(uint32_t)v[10])+
            ct_path_ramp((uint32_t)v[7],(uint32_t)v[9],(uint32_t)v[10])>ticks) return reject(c,"path_profile");
        s.peak=(uint16_t)v[7]; s.entry=(uint16_t)v[8]; s.exit=(uint16_t)v[9]; s.accel=(uint32_t)v[10];
        c->path_queue[(c->path_head+c->path_size)%CT_PATH_CAP]=s;
        c->path_size++; c->path_received++; memcpy(c->path_tail,s.target,sizeof(s.target));
        return CT_OK;
    }
    if (!strcmp(w[0],"GO") && n==3) {
        if(!c->path_active || c->path_started || !command_number_i64(w[2],1,UINT32_MAX,&v[2]) || v[2]!=c->job ||
            c->path_size<(c->path_total<CT_PATH_CAP ? c->path_total : CT_PATH_CAP)) return reject(c,"path_not_buffered");
        const char *why=ct_inputs(c,now); if(why) return reject(c,why);
        c->lease_ms=c->start_ms=now;
        if(!ct_path_advance(c)) return reject(c,"path_invalid");
        c->reason="path_running"; return CT_START;
    }
    if (!strcmp(w[0], "HOLD") && n == 4) {
        unsigned a = !strcmp(w[2], "J1") ? 1 : !strcmp(w[2], "J2") ? 2 : 0;
        if (!a || !command_number_i64(w[3], 0, 1, &v[3]) || ct_busy(c) || c->fault) return reject(c, "hold_rejected");
        c->hold_axis = a; c->hold_value = (bool)v[3]; c->referenced = false; return CT_HOLD_CHANGE;
    }
    if (!strcmp(w[0], "POL") && n == 4) {
        unsigned a = !strcmp(w[2], "Z") ? 0 : !strcmp(w[2], "J1") ? 1 : !strcmp(w[2], "J2") ? 2 : 3;
        if (a >= 3 || !command_number_i64(w[3], 0, 1, &v[3]) || ct_busy(c)) return reject(c, "pol_rejected");
        c->pol[a] = (bool)v[3]; c->referenced = false; return CT_OK;
    }
    if(!strcmp(w[0],"COUPLE") && n==4) {
        if(ct_busy(c) || !command_number_i64(w[2],0,1,&v[2]) || !command_number_i64(w[3],16,256,&v[3])) return reject(c,"coupling_config");
        c->auto_coupling=(bool)v[2]; c->probe_pulses=(uint32_t)v[3]; c->referenced=false; return CT_OK;
    }
    bool config = !strcmp(w[0], "GEOM") || !strcmp(w[0], "SPAN") || !strcmp(w[0], "PARK") || !strcmp(w[0], "TUNE");
    if (config) {
        if (ct_busy(c)) return reject(c, "busy");
        if (!strcmp(w[0], "GEOM")) {
            if (n != 9 || !command_number_i64(w[2], -2000000, 2000000, &v[2]) ||
                !command_number_i64(w[3], 1024, 100000000, &v[3]) ||
                !command_number_i64(w[4], 1024, 1000000, &v[4]) || !command_number_i64(w[5], 1024, 1000000, &v[5]))
                return reject(c, "geometry");
            for (unsigned a = 0; a < 3; a++) {
                if (!command_number_i64(w[a + 6], 1, 32, &v[a + 6]) || (v[a + 6] & (v[a + 6] - 1))) return reject(c, "microstep");
            }
            c->coupling_ppm = (int32_t)v[2];
            for (unsigned a = 0; a < 3; a++) { c->nominal[a] = (uint32_t)v[a + 3]; c->micro[a] = (uint32_t)v[a + 6]; }
            c->cfg_mask |= 1;
        } else if (!strcmp(w[0], "SPAN")) {
            if (n != 7) return reject(c, "span");
            for (unsigned i = 2; i <= 5; i++) if (!command_number_i64(w[i], -180000, 180000, &v[i])) return reject(c, "angle");
            if (!command_number_i64(w[6], 0, 2000000, &v[6])) return reject(c, "z_span");
            for (unsigned j = 0; j < 2; j++) {
                int64_t lo = v[2 + 2 * j], hi = v[3 + 2 * j];
                if (hi - lo < 10000 || hi - lo > 360000) return reject(c, "span");
                c->low_md[j] = (int32_t)lo; c->high_md[j] = (int32_t)hi;
            }
            c->z_span_um = (uint32_t)v[6]; c->cfg_mask |= 2;
        } else if (!strcmp(w[0], "PARK")) {
            if (n != 5 || !command_number_i64(w[2], -180000, 180000, &v[2]) ||
                !command_number_i64(w[3], -180000, 180000, &v[3]) || !command_number_i64(w[4], 100, 100000, &v[4])) return reject(c, "park");
            for (unsigned j = 0; j < 2; j++) {
                if (v[j + 2] <= c->low_md[j] + 2000 || v[j + 2] >= c->high_md[j] - 2000) return reject(c, "park");
            }
            if (llabs(v[3]) < 10000 || llabs(v[3]) > 170000) return reject(c, "park_singular");
            c->park_md[0] = (int32_t)v[2]; c->park_md[1] = (int32_t)v[3]; c->z_clear_um = (uint32_t)v[4]; c->cfg_mask |= 4;
        } else {
            if ((n != 8 && n!=9 && n!=11) || !command_number_i64(w[2], 100, 800, &v[2]) || !command_number_i64(w[3], 100, CT_Z_CAL_RATE, &v[3]) ||
                !command_number_i64(w[4], 256, 1000000, &v[4]) || !command_number_i64(w[5], 256, 1000000, &v[5]) ||
                !command_number_i64(w[6], 32, 8192, &v[6]) || !command_number_i64(w[7], 32, 8192, &v[7]) ||
                v[6] >= v[4] || v[7] >= v[5] ||
                (n>=9 && !command_number_i64(w[8],50,v[3],&v[8])) ||
                (n==11 && (!command_number_i64(w[9],100,1600,&v[9]) || !command_number_i64(w[10],50,v[9],&v[10])))) return reject(c, "tuning");
            c->arm_rate = (uint32_t)v[2]; c->z_rate = (uint32_t)v[3]; c->arm_scan = (uint32_t)v[4];
            c->z_scan = (uint32_t)v[5]; c->arm_back = (uint32_t)v[6]; c->z_back = (uint32_t)v[7]; c->cfg_mask |= 8;
            c->z_fine_rate=n>=9 ? (uint32_t)v[8] : (c->z_fine_rate>c->z_rate ? c->z_rate : c->z_fine_rate);
            c->j2_rate=n==11 ? (uint32_t)v[9] : c->arm_rate;
            c->j2_fine_rate=n==11 ? (uint32_t)v[10] : (200U>c->j2_rate ? c->j2_rate : 200U);
        }
        c->referenced = false; return CT_OK;
    }
    if (!strcmp(w[0], "INIT") && n == 3) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || v[2] <= c->last_job) return reject(c, "job");
        if (ct_busy(c)) return reject(c, "busy");
        const char *why = ct_inputs(c, now); if (why) return reject(c, why);
        if (c->cfg_mask != 15 || c->reference_epoch == UINT32_MAX) return reject(c, "configuration");
        c->job = c->last_job = (uint32_t)v[2]; c->lease_ms = c->start_ms = now;
        c->input_axis=-1; c->input_kind=0; c->wait_axes=c->wait_bits=0;
        c->referenced = false; c->initializing = true;
        c->coupling_ready=c->probe_contact=c->probe_retry=false;
        c->probe_da=c->probe_db=0;
        memset(c->factor, 0, sizeof(c->factor)); memset(c->range, 0, sizeof(c->range));
        for (unsigned a = 0; a < 3; a++) {
            c->cal[a].phase=c->cal[a].failed_phase=HMC_IDLE;
            c->cal[a].error=HMC_NONE; c->cal[a].n1=c->cal[a].n2=0;
            c->cal[a].reference=false;
        }
        ct_beta_update(c);
        if (llabs(c->beta) > 64 * CT_Q) return reject(c, "coupling_bounds");
        return ct_begin_cal(c, CT_Z, HS_Z, now);
    }
    if (!strcmp(w[0], "MOVE") && n == 7) {
        if (!command_number_i64(w[2], 1, UINT32_MAX, &v[2]) || v[2] <= c->last_job) return reject(c, "job");
        if (ct_busy(c)) return reject(c, "busy");
        if (!c->referenced) return reject(c, "unreferenced");
        const char *why = ct_inputs(c, now); if (why) return reject(c, why);
        int32_t target[CT_AXES];
        for (unsigned a = 0; a < 3; a++) {
            if (!command_number_i64(w[a + 3], -CT_MAX_POS, CT_MAX_POS, &v[a + 3])) return reject(c, "target");
            int64_t p = v[a + 3] + c->origin[a];
            if (p < -CT_MAX_POS || p > CT_MAX_POS) return reject(c, "target");
            target[a] = (int32_t)p;
        }
        if (!command_number_i64(w[6], 50, CT_MAX_RATE, &v[6]) || !ct_pose_valid(c, target, 1000)) return reject(c, "soft_limit");
        c->job = c->last_job = (uint32_t)v[2]; c->lease_ms = c->start_ms = now;
        c->input_axis=-1; c->input_kind=0;
        enum ct_result r = ct_raw_move(c, target, (uint32_t)v[6], false);
        if (r == CT_REJECTED) return reject(c, "move_budget");
        c->reason = c->running ? "moving" : "complete";
        return r;
    }
    return reject(c, "invalid_command");
}
