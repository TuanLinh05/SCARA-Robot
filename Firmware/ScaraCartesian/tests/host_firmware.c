/* Executes the actual NC firmware GPIO/timer/USB layer with virtual peripherals. */
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <math.h>
#define main firmware_main
#include "../src/main.c"
#undef main
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
struct device mock_devices[4];
static uint64_t clock_us, timer_base_us;
static int levels[32], fail_pin, fail_value, forced_pin, forced_value, counter_error;
static unsigned rises[3];
static unsigned timer_starts;
static uint64_t direction_at[3], minimum_setup;
static uint64_t rise_at[3], last_width[3];
static bool booting, counter_on, inject_late;
static bool mechanical;
/* Independent physical pose. Updated by PUL rising edges and actual DIR pads,
 * never by control.pos: a wrong polarity must move the simulated mechanism wrong. */
static int32_t physical_pos[3];
static bool check_elbow_hold;
static unsigned compensated_edges;
static double physical_f1=40.0/3.0;
static int noise_axis=-1;
static uint64_t noise_until;
static void mechanics_levels(void);
static uint32_t gpio_delay;
static struct counter_top_cfg last_cfg;
static jmp_buf boot_exit;
static char report[8192];
static size_t report_used;
uint32_t k_uptime_get_32(void) { return (uint32_t)(clock_us / 1000); }
uint32_t k_cycle_get_32(void) { return (uint32_t)(clock_us * 72U); }
void k_busy_wait(uint32_t us) { clock_us += us; }
void k_msleep(int ms) { if (booting) longjmp(boot_exit, 1); clock_us += (uint64_t)ms * 1000; }
static unsigned pad_index(const struct device *p, unsigned pin) { return pin + (p->id == 3 ? 16 : 0); }
int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value)
{
    unsigned index = pad_index(p->port, p->pin);
    if ((int)index == fail_pin && value == fail_value) return -EIO;
    int raw = value ^ ((p->dt_flags & GPIO_ACTIVE_LOW) != 0);
    for (unsigned a = 0; a < 3; a++) if (index == pad_index(motors[a].p.port, motors[a].p.pin)) {
        if (raw && !levels[index]) {
            uint64_t setup_time=clock_us-direction_at[a];
            if(setup_time<minimum_setup) minimum_setup=setup_time;
            rises[a]++; rise_at[a] = clock_us;
            if(mechanical) {
                int dir=levels[pad_index(motors[a].d.port,motors[a].d.pin)] ? 1 : -1;
                physical_pos[a]+=a==CT_J2 ? -dir : dir;
                if(check_elbow_hold && control.stage==HS_J1) {
                    assert(levels[pad_index(motors[CT_J1].d.port,motors[CT_J1].d.pin)] !=
                           levels[pad_index(motors[CT_J2].d.port,motors[CT_J2].d.pin)]);
                    double elbow=physical_pos[2]/40.0-(4.0/3.0)*physical_pos[1]/physical_f1;
                    assert(fabs(elbow)<1.0); compensated_edges++;
                }
            }
        }
        if (!raw && levels[index]) last_width[a] = clock_us - rise_at[a];
    }
    for(unsigned a=0;a<3;a++) if(index==pad_index(motors[a].d.port,motors[a].d.pin) && raw!=levels[index]) {
        assert(!levels[pad_index(motors[a].p.port,motors[a].p.pin)]);
        direction_at[a]=clock_us;
    }
    levels[index] = raw; return 0;
}
int gpio_pin_configure_dt(const struct gpio_dt_spec *p, unsigned flags)
{ if (flags & GPIO_INPUT) return 0; return gpio_pin_set_dt(p, (flags & GPIO_OUTPUT_ACTIVE) != 0); }
int gpio_pin_get_raw(const struct device *p, unsigned pin)
{ unsigned index = pad_index(p, pin); return (int)index == forced_pin ? forced_value : levels[index]; }
int gpio_pin_get(const struct device *p, unsigned pin) { return gpio_pin_get_raw(p, pin); }
int gpio_port_get_raw(const struct device *p, gpio_port_value_t *value)
{ if (mechanical) mechanics_levels(); clock_us += gpio_delay; *value = 0; for (unsigned i = 0; i < 16; i++) *value |= (gpio_port_value_t)levels[pad_index(p,i)] << i; return 0; }
uint32_t counter_get_frequency(const struct device *d) { (void)d; return 1000000; }
int counter_start(const struct device *d) { (void)d; timer_starts++; counter_on = true; return counter_error; }
int counter_stop(const struct device *d) { (void)d; counter_on = false; return 0; }
int counter_set_top_value(const struct device *d, const struct counter_top_cfg *cfg)
{
    (void)d; last_cfg = *cfg;
    if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) timer_base_us = clock_us;
    if ((inject_late || clock_us - timer_base_us >= cfg->ticks) && (cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
        if (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) timer_base_us = clock_us;
        return -ETIME;
    }
    return counter_error;
}
int usb_enable(void (*cb)(enum usb_dc_status_code, const uint8_t *)) { cb(USB_DC_CONFIGURED, NULL); return 0; }
int uart_poll_in(const struct device *d, unsigned char *ch) { (void)d; (void)ch; return -1; }
void uart_poll_out(const struct device *d, unsigned char ch)
{ (void)d; assert(report_used + 1 < sizeof(report)); report[report_used++] = (char)ch; report[report_used] = 0; }
static void clear_report(void) { report_used = 0; report[0] = 0; }
static void mechanics_levels(void)
{
    double a=physical_pos[1]/physical_f1;
    double b=physical_pos[2]/40.0-(4.0/3.0)*a;
    double x[3]={physical_pos[0],a,b};
    for(unsigned i=0;i<3;i++) {
        bool plus=x[i]>=(i ? 90 : 40000), minus=x[i]<=(i ? -90 : 0);
        levels[pad_index(limits[i][0].port,limits[i][0].pin)]=i==CT_J2 ? minus : plus;
        levels[pad_index(limits[i][1].port,limits[i][1].pin)]=i==CT_J2 ? plus : minus;
    }
    if(noise_axis>=0 && clock_us<noise_until) {
        unsigned a=(unsigned)noise_axis;
        levels[pad_index(limits[a][0].port,limits[a][0].pin)]=1;
        levels[pad_index(limits[a][1].port,limits[a][1].pin)]=1;
    }
}
static void sample(void) { read_limits(); }
static void setup(void)
{
    clock_us = timer_base_us = 0; fail_pin = forced_pin = -1; fail_value = forced_value = counter_error = 0;
    memset(levels,0,sizeof(levels)); memset(rises,0,sizeof(rises)); memset(last_width,0,sizeof(last_width));
    memset(direction_at,0,sizeof(direction_at)); timer_starts=0; minimum_setup=UINT64_MAX;
    motor_error = timer_errno = 0; timer_running = usb_configured = counter_on = inject_late = mechanical = false;
    high_mask = usb_epoch = pulse_start_cycles = low_us = timer_late = gpio_delay = 0; clear_report();
    noise_axis=-1; noise_until=0;
    physical_pos[0]=16000; physical_pos[1]=267; physical_pos[2]=667;
    check_elbow_hold=false; compensated_edges=0;
    physical_f1=40.0/3.0;
    for (int i = 0; i < 4; i++) mock_devices[i].id = i;
    booting = true; if (!setjmp(boot_exit)) firmware_main(); booting = false;
    sample(); clock_us += 21000; sample(); command("HELLO 7");
}
static void referenced_fixture(void)
{
    setup(); control.referenced = true; control.stage = HS_READY; control.reference_epoch = 1;
    control.pos[0] = 20000; control.pos[2] = 1800;
    control.range[0] = 40000; control.range[1] = 2400; control.range[2] = 7200;
    control.factor[0] = 1638400; control.factor[1] = 13653; control.factor[2] = 40960;
    control.beta = 4 * CT_Q; clear_report();
}
static void advance(unsigned ms, bool keep)
{
    uint64_t end = clock_us + (uint64_t)ms * 1000;
    while (clock_us < end) {
        uint64_t next = clock_us + 2000;
        if (counter_on && timer_base_us + last_cfg.ticks + 1 < next)
            next = timer_base_us + last_cfg.ticks + 1;
        if (next > end) next = end;
        if (next < clock_us) next = clock_us;
        clock_us = next;
        sample();
        if (keep && ct_busy(&control)) {
            char s[80]; snprintf(s,sizeof(s),"KEEP 7 %u",control.job); ct_command(&control,s,k_uptime_get_32());
        }
        if (counter_on && clock_us >= timer_base_us + last_cfg.ticks + 1) {
            timer_base_us = clock_us; pulse_tick(timer,NULL);
        }
        if (timer_running && (!control.running || !ct_safe(&control,k_uptime_get_32()))) stop_pulses();
        if (!high_mask) {
            enum ct_result r = ct_service(&control,k_uptime_get_32());
            if (r == CT_START) start_motor();
            else if (!control.running && timer_running) stop_pulses();
        }
    }
}
static void configure_home(void)
{
    sample(); clock_us+=21000; sample();
    command("GEOM 7 1333333 1638400 13653 40960 16 8 8");
    command("SPAN 7 -90000 90000 -90000 90000 0");
    command("PARK 7 0 45000 3000");
    command("TUNE 7 400 800 40000 100000 512 1024");
}
static void three_segments(void)
{
    command("PATH 7 1 3");
    command("SEG 7 1 0 20000 20 1880 1000 50 1000 200000");
    command("SEG 7 1 1 20000 40 1960 1000 1000 1000 2000");
    command("SEG 7 1 2 20000 20 1880 1000 1000 50 200000");
    command("GO 7 1");
}
static void path_tests(void)
{
    referenced_fixture(); uint64_t before_us=clock_us;
    three_segments(); advance(600,true);
    assert(control.referenced && !ct_busy(&control) && !counter_on && !high_mask);
    assert(control.path_done==3 && control.path_received==3 && control.path_size==0);
    assert(control.pos[0]==20000 && control.pos[1]==20 && control.pos[2]==1880);
    assert(rises[0]==0 && rises[1]==60 && rises[2]==240 && timer_starts==1);
    assert(minimum_setup>=50 && clock_us-before_us>=290000);
    referenced_fixture(); three_segments(); advance(80,true);
    unsigned before=rises[2]; command("STOP USER"); advance(400,true);
    assert(rises[2]==before && !control.referenced && !counter_on && !control.path_active);
    referenced_fixture(); three_segments(); advance(80,true);
    levels[16]=1; before=rises[2]; advance(5,true);
    assert(!control.referenced && !counter_on && rises[2]==before);
    referenced_fixture(); command("PATH 7 1 1");
    command("SEG 7 1 0 20000 250 2800 1000 50 50 2000"); command("GO 7 1"); advance(400,false);
    assert(!control.referenced && !counter_on && !strcmp(control.reason,"heartbeat_lost"));
    referenced_fixture(); three_segments(); advance(80,true);
    usb_status(USB_DC_DISCONNECTED,NULL);
    assert(!counter_on && !control.referenced && !control.path_active);
    referenced_fixture(); command("PATH 7 1 40");
    for(unsigned i=0;i<32;i++) {
        char line[160]; clear_report(); snprintf(line,sizeof(line),"SEG 7 1 %u 20000 %u %u 50 50 50 2000",i,i+1,1800+4*(i+1)); command(line);
    }
    clear_report(); command("GO 7 1"); advance(6000,true);
    assert(!control.referenced && !counter_on && control.path_starved==1 && !strcmp(control.reason,"path_starved"));
    assert(rises[1]==32 && rises[2]==128); /* never invent or repeat a point */
    referenced_fixture(); command("PATH 7 1 2"); command("SEG 7 1 1 20000 20 1880 1000 50 50 2000");
    assert(!control.referenced && !control.path_active && !counter_on);
    referenced_fixture(); command("PATH 7 1 1"); command("SEG 7 1 0 20001 20 1880 1000 50 50 2000");
    assert(!control.referenced && !counter_on); /* Z forbidden in ink FIFO */
    referenced_fixture(); command("PATH 7 1 1"); advance(400,false);
    assert(!ct_busy(&control) && !control.referenced && !strcmp(control.reason,"heartbeat_lost"));
    puts("R8 path: exact pulses, one timer start, DIR setup after HIGH drain, STOP/USB/lease/switch, sequence/Z rejection and starvation passed");
}
static int stroke_file(const char *path)
{
    FILE *file=fopen(path,"r"); assert(file);
    int32_t initial[3]; assert(fscanf(file,"%d %d %d",&initial[0],&initial[1],&initial[2])==3);
    struct ct_segment segments[500]; unsigned n=0;
    while(n<500 && fscanf(file,"%d %d %d %hu %hu %hu %u",&segments[n].target[0],&segments[n].target[1],
        &segments[n].target[2],&segments[n].peak,&segments[n].entry,&segments[n].exit,&segments[n].accel)==7) n++;
    fclose(file); assert(n>0);
    uint64_t duration[2]; unsigned expected[3]={0};
    for(unsigned pass=0;pass<2;pass++) {
        referenced_fixture(); memcpy(control.pos,initial,sizeof(initial)); uint64_t began=clock_us;
        char line[180]; unsigned sent=0;
        if(pass) { snprintf(line,sizeof(line),"PATH 7 1 %u",n); command(line); }
        while(sent<n) {
            struct ct_segment *s=&segments[sent]; clear_report();
            if(pass) snprintf(line,sizeof(line),"SEG 7 1 %u %d %d %d %u %u %u %u",sent,s->target[0],s->target[1],s->target[2],s->peak,s->entry,s->exit,s->accel);
            else snprintf(line,sizeof(line),"MOVE 7 %u %d %d %d %u",sent+1,s->target[0],s->target[1],s->target[2],s->peak);
            command(line); sent++;
            if(!pass) { while(counter_on) advance(1,true); }
            else if(sent==CT_PATH_CAP) break;
            assert(control.referenced);
        }
        if(pass) {
            clear_report(); command("GO 7 1");
            while(ct_busy(&control)) {
                if(sent<n && control.path_size<CT_PATH_CAP) {
                    struct ct_segment *s=&segments[sent]; clear_report();
                    snprintf(line,sizeof(line),"SEG 7 1 %u %d %d %d %u %u %u %u",sent,s->target[0],s->target[1],s->target[2],s->peak,s->entry,s->exit,s->accel);
                    command(line); sent++;
                }
                advance(10,true);
            }
        }
        assert(control.referenced && !counter_on && !high_mask);
        assert(!memcmp(control.pos,segments[n-1].target,sizeof(initial)));
        duration[pass]=clock_us-began;
        if(!pass) memcpy(expected,rises,sizeof(expected)); else assert(!memcmp(expected,rises,sizeof(expected)) && timer_starts==1);
        assert(rises[0]==0 && minimum_setup>=50);
    }
    printf("{\"segments\":%u,\"old_seconds\":%.6f,\"continuous_seconds\":%.6f,\"rises\":[%u,%u,%u],\"timer_starts\":%u}\n",n,duration[0]/1e6,duration[1]/1e6,expected[0],expected[1],expected[2],timer_starts);
    return 0;
}
int main(int argc, char **argv)
{
    if(argc==3 && !strcmp(argv[1],"--stroke-file")) return stroke_file(argv[2]);
    setup(); assert(!control.referenced && !control.running && !counter_on);
    for (unsigned a=0;a<3;a++) assert(!rises[a] && !levels[pad_index(motors[a].e.port,motors[a].e.pin)]);
    command("MOVE 7 1 25000 0 2000 800"); assert(!control.running && !counter_on);
    referenced_fixture(); command("MOVE 7 1 20031 -11 1819 800");
    assert(control.running && counter_on);
    for (unsigned n=0;n<500 && counter_on;n++) advance(50,true);
    assert(!control.running && !counter_on && !high_mask && control.referenced);
    assert(control.pos[0]==20031 && control.pos[1]==-11 && control.pos[2]==1819);
    assert(rises[0]==31 && rises[1]==11 && rises[2]==19);
    assert(levels[31]==0 && levels[7]==0 && !control.pol[2]);
    for(unsigned a=0;a<3;a++) assert(last_width[a]>=50 && !levels[pad_index(motors[a].p.port,motors[a].p.pin)]);
    if(argc>1 && !strcmp(argv[1],"--report")) { clear_report(); status_report(); fputs(report,stdout); return 0; }
    path_tests();
    setup(); mechanical=true; configure_home();
    command("TUNE 7 200 4800 40000 100000 512 1024 400 800 200"); command("INIT 7 1");
    advance(2200,true);
    assert(control.mode==CT_CAL && control.selected==CT_Z && control.cm.rate==4800 && ct_period_us(&control,k_uptime_get_32())==209);
    assert(rises[0]>4000 && last_width[0]>=50);
    for(unsigned n=0;n<18000 && ct_busy(&control);n++) advance(100,true);
    assert(control.referenced && control.range[0]==40000 && !counter_on && !high_mask);
    puts("R9 fast Z: 4800pps search, 400pps latch, >=50us HIGH, exact two-limit full home passed");
    referenced_fixture(); inject_late=true; command("MOVE 7 1 20040 0 1800 800"); advance(2000,true);
    assert(!control.fault && control.referenced && rises[0]==40 && timer_late>0 && !counter_on);
    referenced_fixture(); command("MOVE 7 1 21000 0 1800 800"); advance(70,true);
    unsigned before=rises[0]; command("STOP"); advance(300,true);
    assert(!control.referenced && !counter_on && !high_mask && rises[0]==before && last_width[0]>=50);
    referenced_fixture(); command("MOVE 7 1 21000 0 1800 800"); advance(450,false);
    assert(!control.referenced && !counter_on && !strcmp(control.reason,"heartbeat_lost"));
    referenced_fixture(); command("HOLD 7 J1 0");
    assert(!control.referenced && !control.holding[1] && levels[30]==1 && levels[2]==0);
    command("HOLD 7 J1 1"); assert(control.holding[1] && !levels[30] && !control.referenced);
    referenced_fixture(); command("MOVE 7 1 21000 100 2400 800"); advance(60,true);
    levels[27]=1; /* Normalized positive elbow is PB11, physical DIR LOW. */
    advance(10,true); assert(!control.referenced && !counter_on && !strcmp(control.reason,"j2_limit"));
    referenced_fixture(); command("MOVE 7 1 21000 0 1800 800"); advance(55,true);
    levels[26]=levels[27]=1; before=rises[0]; advance(1,true);
    assert(!control.input_wait && !control.referenced && !counter_on && control.input_axis==2);
    assert(!strcmp(control.reason,"both_open") && rises[0]==before);
    levels[26]=levels[27]=0; advance(100,true); assert(!counter_on && rises[0]==before);
    referenced_fixture(); levels[16]=levels[17]=1; sample();
    assert(!control.referenced && control.input_axis==1 && !control.input_wait);
    clock_us+=21000; sample();
    assert(control.sw[1].conflict && control.input_kind==2 && !strcmp(control.reason,"both_latched"));
    assert(!counter_on && !rises[0] && !rises[1] && !rises[2]);
    referenced_fixture(); ct_sample(&control,2,-EIO,-EIO,k_uptime_get_32());
    assert(!control.referenced && control.sw[2].error && control.input_kind==4 && control.input_axis==2);
    referenced_fixture(); forced_pin=31; forced_value=0;
    command("MOVE 7 1 20000 100 2000 800"); assert(control.fault && !counter_on && !control.referenced);
    forced_pin=-1; command("HELLO 8"); assert(control.fault);
    referenced_fixture(); fail_pin=6; fail_value=1;
    command("MOVE 7 1 20000 100 1900 800"); advance(100,true);
    assert(control.fault && !counter_on && !control.referenced && rises[1]==1 && rises[2]==0 && last_width[1]>=50);
    referenced_fixture(); command("MOVE 7 1 21000 0 1800 800"); advance(60,true);
    usb_status(USB_DC_DISCONNECTED,NULL); assert(!counter_on && !control.referenced && !control.session);
    referenced_fixture(); counter_error=-EIO; command("MOVE 7 1 21000 0 1800 800"); assert(control.fault && !counter_on);
    referenced_fixture(); clock_us=((uint64_t)UINT32_MAX-70)*1000; gpio_delay=400; sample();
    command("MOVE 7 1 20040 0 1800 800"); advance(1500,true);
    assert(!control.fault && control.referenced && rises[0]==40);
    /* Full startup -> configuration -> mechanical HOME -> final HIGH drain.
     * Uses the real timer ISR, worker service and six GPIO input mappings. */
    /* Reproduce the previous sign error against the physical DIR-based model. */
    setup(); mechanical=true; configure_home(); command("POL 7 J2 1"); command("COUPLE 7 0 64"); command("INIT 7 1");
    for(unsigned n=0;n<18000 && ct_busy(&control);n++) advance(100,true);
    assert(!control.referenced && !counter_on && !high_mask && !strcmp(control.reason,"j2_guard"));
    /* With automatic measurement the same wrong polarity is rejected BEFORE
     * the long J1 sweep; keep the signed probe in telemetry for diagnosis. */
    setup(); mechanical=true; configure_home(); command("POL 7 J2 1"); command("INIT 7 1");
    for(unsigned n=0;n<18000 && ct_busy(&control);n++) advance(100,true);
    assert(!control.referenced && !counter_on && !strcmp(control.reason,"coupling_sign"));
    assert(control.beta<0 && control.probe_da && control.probe_db && rises[1]<=128);
    assert(control.cal[1].phase==HMC_IDLE);
    for(unsigned scale=2;scale<=3;scale++) {
    setup(); mechanical=true; physical_f1=(40.0/3.0)*scale; physical_pos[1]=(int32_t)lround(20*physical_f1);
    check_elbow_hold=true; configure_home(); command("INIT 7 1");
    for(unsigned n=0;n<18000 && ct_busy(&control);n++) advance(100,true);
    assert(control.referenced && control.coupling_ready && compensated_edges>1000);
    assert(llabs(control.beta-4*CT_Q/scale)<CT_Q/50 && abs((int)control.range[1]-(int)(2400*scale))<=3);
    assert(abs(control.coupling_ppm-1333333)<10000);
    }
    for(unsigned noisy=0;noisy<3;noisy++) {
    setup(); mechanical=true; check_elbow_hold=true; configure_home();
    command("INIT 7 1");
    advance(55,true); assert(control.running);
    unsigned saved_rises[3]; memcpy(saved_rises,rises,sizeof(saved_rises));
    noise_axis=(int)noisy; noise_until=clock_us+4000; advance(1,true);
    if(noisy==0) {
        assert(control.input_wait && !control.running && !counter_on && !high_mask);
        assert(!memcmp(saved_rises,rises,sizeof(saved_rises)));
        advance(10,true); assert(control.input_wait && !counter_on);
        assert(!memcmp(saved_rises,rises,sizeof(saved_rises)));
        advance(20,true); assert(!control.input_wait && counter_on && control.recovered==1);
        assert(!memcmp(saved_rises,rises,sizeof(saved_rises)));
    } else {
        assert(!control.input_wait && control.running && counter_on && control.idle_ignored==1);
        advance(30,true); assert(counter_on && rises[0]>saved_rises[0] && !control.recovered);
    }
    for(unsigned n=0;n<18000 && ct_busy(&control);n++) advance(100,true);
    if(!control.referenced) fprintf(stderr,"IO init failed %s phase=%u error=%u timer=%d\n",control.reason,control.cal[control.selected].phase,control.cal[control.selected].error,timer_errno);
    assert(control.referenced && !counter_on && !high_mask && control.stage==HS_READY);
    assert(abs((int)control.range[1]-2400)<=3 && abs((int)control.range[2]-7200)<=4);
    assert(control.range[0]==40000 && abs(control.pos[0]+16000-35200)<=2);
    assert(compensated_edges>1000);
    const int32_t initial[3]={16000,267,667};
    for(unsigned a=0;a<3;a++) assert(physical_pos[a]==control.pos[a]+initial[a]);
    assert(fabs(physical_pos[1]/(40.0/3.0))<.2);
    assert(fabs(physical_pos[2]/40.0-(4.0/3.0)*physical_pos[1]/(40.0/3.0)-45)<.2);
    /* XYZ must use the same motor normalization after measured home. */
    /* MOVE contains displacement from the measured origin, not raw counters. */
    command("MOVE 7 2 20000 400 2800 800"); assert(control.running);
    for(unsigned n=0;n<10000 && ct_busy(&control);n++) advance(100,true);
    assert(control.referenced && !counter_on && !high_mask);
    for(unsigned a=0;a<3;a++) assert(physical_pos[a]==control.pos[a]+initial[a]);
    double shoulder=physical_pos[1]/(40.0/3.0), elbow=physical_pos[2]/40.0-(4.0/3.0)*shoulder;
    if(fabs(shoulder-30)>=.2 || fabs(elbow-30)>=.2 || physical_pos[0]!=20000)
        fprintf(stderr,"physical XYZ q1=%.5f q2=%.5f z=%d origin=[%d,%d,%d]\n",
            shoulder,elbow,physical_pos[0],control.origin[0],control.origin[1],control.origin[2]);
    assert(fabs(shoulder-30)<.2 && fabs(elbow-30)<.2 && physical_pos[0]==20000);
    }
    puts("motor signs: old J2 polarity reproduces guard failure; opposite DIR, physical NC mapping, HOME and XYZ passed");
    puts("input filtering: active first-edge stop, idle glitches do not interrupt Z, exact full HOME, strict XYZ passed");
    /* Telemetry diagnostics must not truncate and appear as a USB timeout. */
    referenced_fixture(); control.referenced=false; control.stage=HS_FAILED;
    control.session=control.job=control.reference_epoch=control.recovered=timer_late=UINT32_MAX;
    control.tick=control.ticks=1000000; control.input_axis=2; control.input_kind=5;
    control.input_bits=63; control.input_at_ms=UINT32_MAX; control.reason="calibration_failed";
    control.idle_ignored=UINT32_MAX;
    control.path_total=control.path_received=control.path_done=500; control.path_starved=UINT32_MAX;
    control.probe_da=-256; control.probe_db=-16384; control.beta=64*CT_Q;
    control.direction[0]=control.direction[1]=control.direction[2]=-1;
    for(unsigned a=0;a<3;a++) {
        control.pos[a]=control.goal[a]=CT_MAX_POS; control.origin[a]=-CT_MAX_POS;
        control.total[a]=UINT64_MAX; control.range[a]=control.cal[a].n1=control.cal[a].n2=1000000;
        control.factor[a]=100000000; control.cal[a].phase=HMC_FAILED; control.cal[a].failed_phase=HMC_LATCH_N2;
        control.cal[a].error=HMC_BAD_COMMAND; control.sw[a].brief_both=UINT32_MAX;
        control.cal[a].false_hits=UINT32_MAX;
        control.sw[a].raw_pos=control.sw[a].raw_neg=1; control.sw[a].top=control.sw[a].bottom=control.sw[a].conflict=true;
        control.sw[a].error=-EIO;
    }
    clear_report(); status_report(); assert(report_used>0 && report_used<1536);
    if(argc>1 && !strcmp(argv[1],"--max-report")) { fputs(report,stdout); return 0; }
    puts("actual firmware ISR: GPIO counts/widths, late timer recovery, three-axis STOP, ENA, readback faults, switch guard, USB, lease, timestamp wrap passed");
    return 0;
}



