#include "../src/cart_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static struct ct_state c;
static uint64_t us, due;
static bool keepalive = true, stuck = false;
static bool expect_elbow_hold=true;
static int open_axis = -1, open_side;
static int both_axis=-1;
static double f1 = 40.0 / 3.0, f2 = 40.0;
static int32_t off[3];
static double q1(void) { return (c.pos[1] + off[1]) / f1; }
static double q2(void) { return (c.pos[2] + off[2]) / f2 - (4.0 / 3.0) * q1(); }
static void sample(void)
{
    uint32_t now = (uint32_t)(us / 1000);
    double coordinate[3] = {c.pos[0] + off[0], q1(), q2()};
    for (unsigned a = 0; a < 3; a++) {
        bool p = coordinate[a] >= (a ? 90 : 40000), n = coordinate[a] <= (a ? -90 : 0);
        /* PB10 is the DIR-HIGH stop: negative normalized elbow on this motor. */
        if(a==CT_J2) { bool t=p; p=n; n=t; }
        if (stuck) p = n = false;
        if ((int)a == open_axis) { if (open_side) p = true; else n = true; }
        if ((int)a == both_axis) p=n=true;
        ct_sample(&c, a, p, n, now);
    }
}
static enum ct_result cmd(const char *s)
{
    enum ct_result r = ct_command(&c, s, (uint32_t)(us / 1000));
    if (r == CT_START) due = us + 50000;
    return r;
}
static void boot(double a, double b)
{
    ct_init(&c); us = 30000; due = 0; keepalive = true; stuck = false; expect_elbow_hold=true; open_axis = both_axis = -1;
    off[0] = 16000; off[1] = (int32_t)lround(a * f1); off[2] = (int32_t)lround((b + (4.0 / 3.0) * a) * f2);
    sample(); us += 21000; sample();
    assert(cmd("HELLO 7") == CT_OK);
    assert(cmd("GEOM 7 1333333 1638400 13653 40960 16 8 8") == CT_OK);
    assert(cmd("SPAN 7 -90000 90000 -90000 90000 0") == CT_OK);
    assert(cmd("PARK 7 0 45000 3000") == CT_OK);
    assert(cmd("TUNE 7 400 800 40000 100000 512 1024") == CT_OK);
}
static void advance(unsigned milliseconds)
{
    uint64_t end = us + (uint64_t)milliseconds * 1000;
    while (us < end) {
        us += 500; sample(); uint32_t now = (uint32_t)(us / 1000);
        if (keepalive && ct_busy(&c)) {
            char s[80]; snprintf(s, sizeof(s), "KEEP 7 %u", c.job); cmd(s);
        }
        if (c.running) ct_safe(&c, now);
        if (c.running && us >= due) {
            if (ct_safe(&c, now)) {
                uint32_t period = ct_period_us(&c, now);
                unsigned mask = ct_next_mask(&c);
                for (unsigned a = 0; a < 3; a++) if (mask & (1U << a)) ct_count(&c, a);
                if (c.running) ct_finish_tick(&c);
                due = us + period;
            }
        }
        if (c.stage == HS_J1 && expect_elbow_hold) {
            if(fabs(q2())>=1.0) fprintf(stderr,"elbow drift a=%f b=%f beta=%lld probe=%d/%d f1=%f\n",q1(),q2(),(long long)c.beta,c.probe_da,c.probe_db,f1);
            assert(fabs(q2()) < 1.0);
        }
        enum ct_result r = ct_service(&c, now);
        if (r == CT_START) due = us + 50000;
    }
}
static void finish_home(void)
{
    for (unsigned i = 0; i < 18000 && ct_busy(&c); i++) advance(100);
    if (!c.referenced) fprintf(stderr, "init failed reason=%s stage=%d phase=%d err=%d n1=%u n2=%u q1=%f q2=%f z=%d\n",
        c.reason,c.stage,c.cal[c.selected].phase,c.cal[c.selected].error,c.cal[c.selected].n1,c.cal[c.selected].n2,q1(),q2(),c.pos[0]+off[0]);
    assert(c.referenced && c.stage == HS_READY);
    assert(abs((int)c.range[1] - (int)lround(180*f1)) <= 3 && abs((int)c.range[2] - 7200) <= 4 && c.range[0] == 40000);
    assert(fabs(q1()) < .2 && fabs(q2() - 45) < .2);
    assert(abs(c.pos[0] + off[0] - 35200) <= 2);
}
static void initialize(void) { assert(cmd("INIT 7 1")==CT_START); finish_home(); }
int main(void)
{
    boot(20, -10); assert(cmd("MOVE 7 1 20000 0 1800 800") == CT_REJECTED);
    initialize();
    int32_t origin[3]; memcpy(origin, c.origin, sizeof(origin));
    assert(cmd("MOVE 7 2 20000 400 2800 800") == CT_START);
    for (unsigned i = 0; i < 10000 && c.running; i++) advance(20);
    assert(c.referenced && !c.running);
    assert(c.pos[0]-origin[0] == 20000 && c.pos[1]-origin[1] == 400 && c.pos[2]-origin[2] == 2800);
    assert(fabs(q1()-30)<.2 && fabs(q2()-30)<.2);
    assert(cmd("MOVE 7 3 25000 400 2400 800") == CT_START);
    keepalive = false; advance(400); assert(!ct_busy(&c) && !c.referenced && !strcmp(c.reason,"heartbeat_lost"));
    boot(-50, 30); initialize();
    assert(cmd("MOVE 7 2 30000 -400 0 800") == CT_START);
    uint64_t before[3]; memcpy(before,c.total,sizeof(before));
    open_axis = 2; open_side = 1; advance(20);
    assert(!ct_busy(&c) && !c.referenced && !strcmp(c.reason,"j2_limit"));
    assert(!memcmp(before,c.total,sizeof(before)));
    boot(0,0); assert(cmd("INIT 7 1") == CT_START); advance(200);
    assert(cmd("STOP") == CT_OK && !ct_busy(&c) && !c.referenced);
    boot(0,0); assert(cmd("INIT 7 1") == CT_START); advance(200);
    open_axis=0; open_side=1; advance(1); open_axis=-1; advance(250);
    assert(ct_busy(&c) && c.running && c.cal[0].error==HMC_NONE && c.cal[0].false_hits==1);
    finish_home(); /* a false single opening does not abandon the calibration */
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    for(unsigned t=0;t<220 && ct_busy(&c);t++) { open_axis=t%10<4 ? 0 : -1; open_side=1; advance(1); }
    assert(!c.referenced && c.cal[0].error==HMC_UNSTABLE && c.input_axis==0 && c.input_kind==5);
    boot(0,0); assert(cmd("INIT 7 1") == CT_START);
    for (unsigned i=0;i<10000 && c.stage!=HS_J1;i++) advance(100);
    assert(c.stage==HS_J1); open_axis=2; open_side=1; advance(1); assert(c.input_wait && !c.running); advance(25);
    assert(!ct_busy(&c) && !c.referenced && !strcmp(c.reason,"j2_guard"));
    boot(0,0); stuck = true; assert(cmd("INIT 7 1") == CT_START);
    for (unsigned i = 0; i < 18000 && ct_busy(&c); i++) advance(100);
    assert(!c.referenced && c.stage == HS_FAILED && c.cal[0].error == HMC_BOUNDS);
    boot(0,0); us = ((uint64_t)UINT32_MAX - 100) * 1000; sample();
    assert(cmd("INIT 7 1") == CT_START); advance(400); assert(ct_busy(&c));
    ct_abort(&c,"usb_lost"); assert(!ct_busy(&c) && !c.referenced);
    boot(0,0); open_axis = 1; open_side = 1; sample(); ct_sample(&c,1,1,1,(uint32_t)(us/1000));
    assert(cmd("INIT 7 1") == CT_REJECTED && !strcmp(c.reply,"both_open"));
    /* One DDA shoulder edge can decrease q2 even on a net-positive segment. */
    boot(0,0); initialize();
    assert(cmd("MOVE 7 2 35000 100 2100 800")==CT_START);
    c.accum[1]=c.ticks-1; c.accum[2]=0;
    ct_sample(&c,2,1,0,(uint32_t)(us/1000));
    assert(ct_next_mask(&c)==0 && !c.referenced && !strcmp(c.reason,"edge_limit"));
    /* Active double-open freezes; inactive short events must not interrupt Z.
     * All pairs still latch sustained double-open and stop the entire task. */
    for(unsigned axis=0;axis<3;axis++) {
        boot(20,-10); assert(cmd("INIT 7 1")==CT_START); advance(300);
        uint64_t saved[3]; memcpy(saved,c.total,sizeof(saved));
        both_axis=(int)axis; advance(1);
        assert(c.input_axis==(int)axis);
        if(axis==0) assert(c.input_wait && !c.running);
        else assert(!c.input_wait && c.running && c.idle_ignored==1);
        both_axis=-1; advance(10);
        if(axis==0) assert(c.input_wait && !memcmp(saved,c.total,sizeof(saved)));
        else assert(c.total[0]>saved[0]);
        advance(20); assert(!c.input_wait && c.running);
        if(axis==0) assert(c.recovered==1 && !memcmp(saved,c.total,sizeof(saved)));
        else assert(c.recovered==0 && c.total[0]>saved[0]);
        finish_home();
    }
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    both_axis=1; advance(1); memcpy(before,c.total,sizeof(before)); advance(25);
    assert(!ct_busy(&c) && !c.input_wait && c.sw[1].conflict && c.input_axis==1 && c.input_kind==2);
    assert(!strcmp(c.reason,"both_latched"));
    memcpy(before,c.total,sizeof(before));
    both_axis=-1; advance(100); assert(!ct_busy(&c) && c.sw[1].conflict && !memcmp(before,c.total,sizeof(before)));
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    for(unsigned t=0;t<220 && ct_busy(&c);t++) { both_axis=t%10<4 ? 0 : -1; advance(1); }
    assert(!ct_busy(&c) && c.input_kind==3 && !strcmp(c.reason,"switch_unstable"));
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    both_axis=0; advance(1); assert(c.input_wait); memcpy(before,c.total,sizeof(before));
    assert(cmd("STOP")==CT_OK); both_axis=-1; advance(100);
    assert(!ct_busy(&c) && !c.input_wait && !memcmp(before,c.total,sizeof(before)));
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    both_axis=1; advance(1); c.lease_ms=(uint32_t)(us/1000)-CT_LEASE; keepalive=false;
    advance(1); assert(!ct_busy(&c) && !c.input_wait && !strcmp(c.reason,"heartbeat_lost"));
    /* A real target endpoint remains asserted after a short double-open.
     * Resume must confirm the endpoint; never issue another edge into it. */
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    c.pos[0]=40000-off[0]; both_axis=0; advance(1); memcpy(before,c.total,sizeof(before));
    both_axis=-1; advance(25);
    assert(!c.input_wait && c.running && c.cal[0].phase==HMC_CLEAR_N && c.direction[0]<0);
    assert(!memcmp(before,c.total,sizeof(before)));
    finish_home();
    /* Recovery also preserves the coupled shoulder DDA and parking moves. */
    const enum ct_stage checkpoints[]={HS_J1,HS_PARK};
    for(unsigned checkpoint=0;checkpoint<2;checkpoint++) {
        boot(20,-10); assert(cmd("INIT 7 1")==CT_START);
        for(unsigned t=0;t<15000 && (c.stage!=checkpoints[checkpoint] || !c.running ||
            (checkpoint==0 && c.cal[1].phase!=HMC_SWEEP_P));t++) advance(100);
        assert(c.stage==checkpoints[checkpoint] && c.running);
        uint64_t accum[3], counts[3]; memcpy(accum,c.accum,sizeof(accum)); memcpy(counts,c.total,sizeof(counts));
        both_axis=checkpoint==0 ? 1 : 0; advance(1); both_axis=-1; advance(30);
        assert(c.running && !c.input_wait && c.recovered==1);
        assert(!memcmp(accum,c.accum,sizeof(accum)) && !memcmp(counts,c.total,sizeof(counts)));
        finish_home();
    }
    boot(0,0); c.cal[2].error=HMC_UNSTABLE; c.cal[2].failed_phase=HMC_SWEEP_P; c.cal[2].n1=123;
    assert(cmd("INIT 7 1")==CT_START);
    assert(c.cal[2].error==HMC_NONE && c.cal[2].failed_phase==HMC_IDLE && !c.cal[2].n1);
    ct_abort(&c,"stopped");
    boot(0,0); assert(cmd("TUNE 7 200 4800 40000 100000 512 1024 400 800 200")==CT_OK);
    assert(cmd("INIT 7 1")==CT_START);
    uint32_t fast_start=(uint32_t)(us/1000);
    assert(c.cm.rate==4800 && c.cal[0].fine_rate==400);
    assert(ct_period_us(&c,fast_start+ZJ_SETUP_MS)==2500);
    assert(ct_period_us(&c,fast_start+ZJ_SETUP_MS+ZJ_RAMP_MS)==209);
    ct_abort(&c,"stopped");
    boot(0,0); assert(cmd("TUNE 7 200 1600 40000 100000 512 1024 400")==CT_OK);
    assert(cmd("INIT 7 1")==CT_START);
    uint32_t rate_start=(uint32_t)(us/1000);
    assert(c.cm.rate==1600 && c.cal[0].fine_rate==400);
    assert(ct_period_us(&c,rate_start+ZJ_SETUP_MS)==2500); /* initial 0.25 mm/s */
    assert(ct_period_us(&c,rate_start+ZJ_SETUP_MS+ZJ_RAMP_MS)==625); /* 1 mm/s */
    assert(cmd("STOP FOCUS")==CT_OK && !strcmp(c.reason,"stop_focus"));
    assert(cmd("STOP")==CT_OK && !strcmp(c.reason,"stop_focus")); /* serial close does not hide the source */
    assert(cmd("STOP ESC")==CT_OK && !strcmp(c.reason,"stop_escape"));
    assert(cmd("STOP BAD")==CT_REJECTED);
    boot(0,0); assert(cmd("TUNE 7 200 1600 40000 100000 512 1024 400")==CT_OK);
    initialize();
    char move[128]; snprintf(move,sizeof(move),"MOVE 7 2 %d 0 %d 1600",c.pos[0]-c.origin[0]-16000,c.pos[2]-c.origin[2]);
    assert(cmd(move)==CT_START && !c.delta[1] && !c.delta[2]);
    for(unsigned t=0;t<3000 && c.running;t++) advance(10);
    assert(c.referenced && !c.running);
    /* A stable GPIO contact must qualify even if the main loop was delayed
     * beyond the old 200-ms confirmation window. Sensor sampling stays live. */
    boot(0,0); assert(cmd("INIT 7 1")==CT_START); advance(200);
    c.pos[0]=40000-off[0]; advance(1); assert(!c.running);
    for(unsigned t=0;t<150;t++) {
        us+=2000; sample(); char keep[64]; snprintf(keep,sizeof(keep),"KEEP 7 %u",c.job); cmd(keep);
    }
    assert(ct_service(&c,(uint32_t)(us/1000))==CT_START);
    assert(c.cal[0].phase==HMC_CLEAR_N && c.direction[0]<0);
    ct_abort(&c,"stopped");
    /* A recent inactive glitch cannot become an already-hit endpoint when
     * switching from J2 parking to J1 seek. Wait for its LOW qualification. */
    boot(20,-10); assert(cmd("INIT 7 1")==CT_START);
    for(unsigned t=0;t<500000 && !(c.stage==HS_J2_MID && c.running && c.ticks-c.tick==1);t++) advance(1);
    assert(c.stage==HS_J2_MID && c.ticks-c.tick==1);
    both_axis=1; advance(1); both_axis=-1;
    bool gated=false;
    for(unsigned t=0;t<25;t++) { advance(1); if(c.gate_wait) { gated=true; assert(!c.running && c.cal[1].phase==HMC_IDLE); break; } }
    assert(gated); advance(30);
    assert(!c.gate_wait && c.stage==HS_COUPLE_PROBE && c.cal[1].phase==HMC_IDLE && c.direction[1]);
    finish_home();
    /* Independent J2 rates must not raise J1's calibrated motion rate. */
    boot(0,0); assert(cmd("TUNE 7 200 1600 40000 100000 512 1024 400 800 200")==CT_OK);
    assert(c.arm_rate==200 && c.j2_rate==800 && c.j2_fine_rate==200);
    assert(cmd("INIT 7 1")==CT_START); finish_home();
    /* Brief single openings on the two arm axes recover with the SAME
     * search budget and compensation phase, rather than restarting a scan. */
    for(unsigned a=1;a<=2;a++) {
        boot(20,-10); assert(cmd("INIT 7 1")==CT_START);
        enum ct_stage desired=a==1 ? HS_J1 : HS_J2_PRE;
        enum hc_phase phase=a==1 ? HMC_SWEEP_P : HMC_SEEK_N;
        for(unsigned t=0;t<10000 && !(c.stage==desired && c.cal[a].phase==phase && c.running);t++) advance(100);
        assert(c.stage==desired && c.running); advance(200);
        uint32_t target=c.cm.target, moved=c.cm.move; uint64_t accum[3], counts[3];
        memcpy(accum,c.accum,sizeof(accum)); memcpy(counts,c.total,sizeof(counts));
        open_axis=(int)a; open_side=1; advance(1); assert(!c.running);
        open_axis=-1; advance(30);
        assert(c.running && c.cal[a].false_hits==1 && c.cm.target==target && c.cm.move==moved);
        assert(!memcmp(accum,c.accum,sizeof(accum)) && !memcmp(counts,c.total,sizeof(counts)));
        finish_home();
    }
    /* Reproduce the reported failure with real -A/3 absolute passive motion.
     * The old 2/3 elbow coefficient hits J2 while seeking J1; 4/3 holds it. */
    boot(80,-10); expect_elbow_hold=false;
    assert(cmd("COUPLE 7 0 64")==CT_OK); /* reproduce the fixed-coefficient old behavior */
    assert(cmd("GEOM 7 666667 1638400 13653 40960 16 8 8")==CT_OK);
    assert(cmd("INIT 7 1")==CT_START);
    for(unsigned t=0;t<15000 && ct_busy(&c);t++) advance(100);
    assert(!c.referenced && !strcmp(c.reason,"j2_guard") && c.stage==HS_FAILED);
    boot(80,-10); initialize(); assert(llabs(c.beta-4*CT_Q)<CT_Q/100);
    boot(-80,10); initialize(); assert(llabs(c.beta-4*CT_Q)<CT_Q/100);
    /* Real J1 pulse scale can differ from the nominal 3:1 setting. Measure
     * dB/dA before the J1 sweep; never use the nominal beta for that sweep. */
    f1=80.0/3.0; boot(20,-10); initialize();
    assert(c.coupling_ready && llabs(c.beta-2*CT_Q)<CT_Q/100);
    assert(abs(c.coupling_ppm-1333333)<5000);
    f1=40.0/3.0;
    /* Three times the nominal J1 scale reproduces the user's 20--40 degree
     * collision: fixed beta=4 overcompensates, measured beta~=4/3 holds. */
    f1=40; boot(20,-10); expect_elbow_hold=false;
    assert(cmd("COUPLE 7 0 128")==CT_OK); assert(cmd("INIT 7 1")==CT_START);
    for(unsigned t=0;t<18000 && ct_busy(&c);t++) advance(100);
    assert(!c.referenced && !strcmp(c.reason,"j2_guard"));
    assert(fabs(q1()-20)>20 && fabs(q1()-20)<40);
    boot(20,-10); initialize(); assert(c.coupling_ready && llabs(c.beta-4*CT_Q/3)<CT_Q/50);
    f1=40.0/3.0;
    boot(20,-10); assert(cmd("GEOM 7 666667 1638400 13653 40960 16 8 8")==CT_OK);
    initialize(); assert(c.coupling_ready && llabs(c.beta-4*CT_Q)<CT_Q/100);
    boot(20,-10); assert(cmd("GEOM 7 -1333333 1638400 13653 40960 16 8 8")==CT_OK);
    assert(cmd("INIT 7 1")==CT_START);
    for(unsigned t=0;t<18000 && ct_busy(&c);t++) advance(100);
    assert(!c.referenced && !strcmp(c.reason,"coupling_sign") && c.probe_da && c.probe_db);
    /* Starting close to either J1 endpoint still finds a short safe probe. */
    boot(89.8,0); initialize(); assert(c.probe_retry && c.probe_da<0);
    boot(-90,0); initialize(); assert(c.probe_da>0);
    boot(20,-10); assert(cmd("INIT 7 1")==CT_START);
    for(unsigned t=0;t<18000 && c.stage!=HS_COUPLE_PROBE;t++) advance(100);
    assert(c.stage==HS_COUPLE_PROBE); keepalive=false; advance(400);
    assert(!ct_busy(&c) && !c.referenced && !strcmp(c.reason,"heartbeat_lost"));
    puts("core: full coupled HOME, repeatability, exact DDA, cross-axis switches, watchdog, abort, stuck switch, wrap passed");
    return 0;
}





