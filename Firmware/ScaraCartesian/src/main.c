/* Three-axis NC home/calibration and coordinated Cartesian motor execution. */
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/irq.h>
#include <stdio.h>
#include <string.h>
#include "cart_core.h"
#define USER_NODE DT_PATH(zephyr_user)
#define PULSE_HIGH_US 50U
struct motor_pins { struct gpio_dt_spec p, d, e; };
static const struct motor_pins motors[CT_AXES] = {
    { GPIO_DT_SPEC_GET(USER_NODE, pul_gpios), GPIO_DT_SPEC_GET(USER_NODE, dir_gpios), GPIO_DT_SPEC_GET(USER_NODE, ena_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j1_pul_gpios), GPIO_DT_SPEC_GET(USER_NODE, j1_dir_gpios), GPIO_DT_SPEC_GET(USER_NODE, j1_ena_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j2_pul_gpios), GPIO_DT_SPEC_GET(USER_NODE, j2_dir_gpios), GPIO_DT_SPEC_GET(USER_NODE, j2_ena_gpios) },
};
static const struct gpio_dt_spec limits[CT_AXES][2] = {
    { GPIO_DT_SPEC_GET(USER_NODE, z_pos_gpios), GPIO_DT_SPEC_GET(USER_NODE, z_neg_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j1_pos_gpios), GPIO_DT_SPEC_GET(USER_NODE, j1_neg_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j2_pos_gpios), GPIO_DT_SPEC_GET(USER_NODE, j2_neg_gpios) },
};
static struct ct_state control;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const usb = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *const timer = DEVICE_DT_GET(DT_NODELABEL(jog_timer));
static uint32_t timer_hz, usb_epoch, pulse_start_cycles, low_us, timer_late;
static unsigned high_mask;
static bool timer_running, usb_configured;
static int motor_error, timer_errno;
static void read_limits(void)
{
    for (unsigned a = 0; a < CT_AXES; a++) {
        gpio_port_value_t bits = 0;
        int error = gpio_port_get_raw(limits[a][0].port, &bits);
        ct_sample(&control, a, error ? error : (int)((bits >> limits[a][0].pin) & 1U),
            error ? error : (int)((bits >> limits[a][1].pin) & 1U), k_uptime_get_32());
    }
}
static void wait_minimum_high(void)
{
    if (!high_mask) return;
    uint32_t elapsed = k_cyc_to_us_floor32(k_cycle_get_32() - pulse_start_cycles);
    if (elapsed < PULSE_HIGH_US) k_busy_wait(PULSE_HIGH_US - elapsed);
}
/* Stop GPIO only: ordinary CAL endpoint stops must retain the worker state. */
static void stop_pulses(void)
{
    counter_stop(timer); timer_running = false;
    wait_minimum_high();
    for (unsigned a = 0; a < CT_AXES; a++) {
        int error = gpio_pin_set_dt(&motors[a].p, 0);
        k_busy_wait(2);
        if (error || gpio_pin_get_raw(motors[a].p.port, motors[a].p.pin) != 0) {
            if (!motor_error) motor_error = 4;
            control.fault = true; ct_abort(&control, "motor_gpio_error");
        }
    }
    high_mask = 0;
}
static void halt(const char *reason)
{ ct_abort(&control, reason); stop_pulses(); }
static bool pin_level(const struct gpio_dt_spec *p, int expected, int fault)
{
    if (gpio_pin_get_raw(p->port, p->pin) == expected) return true;
    if (!motor_error) motor_error = fault;
    control.fault = true; halt("motor_gpio_error"); return false;
}
static bool write_level(const struct gpio_dt_spec *p, int value, int fault)
{
    if (gpio_pin_set_dt(p, value)) {
        if (!motor_error) motor_error = fault;
        control.fault = true; halt("motor_gpio_error"); return false;
    }
    k_busy_wait(2); return pin_level(p, value, fault);
}
static int direction_level(unsigned a)
{ return (control.direction[a] > 0) == control.pol[a]; }
static void pulse_tick(const struct device *dev, void *data);
static int timer_period(uint32_t us, bool reset)
{
    struct counter_top_cfg cfg = {
        .ticks = (uint32_t)(((uint64_t)timer_hz * us) / 1000000U) - 1U,
        .callback = pulse_tick,
        .flags = reset ? 0 : COUNTER_TOP_CFG_DONT_RESET | COUNTER_TOP_CFG_RESET_WHEN_LATE,
    };
    int error = counter_set_top_value(timer, &cfg);
    if (error == -ETIME && !reset) { if (timer_late < UINT32_MAX) timer_late++; return 0; }
    if (error) timer_errno = error;
    return error;
}
static void timer_fault(void)
{ control.fault = true; halt("timer_error"); }
static void pulse_tick(const struct device *dev, void *data)
{
    (void)dev; (void)data;
    unsigned key = irq_lock();
    if (motor_error) { halt("motor_gpio_error"); goto out; }
    read_limits(); /* Timestamp must follow GPIO reads, including across ms wrap. */
    if (high_mask) {
        wait_minimum_high();
        unsigned mask = high_mask;
        for (unsigned a = 0; a < CT_AXES; a++)
            if ((mask & (1U << a)) && !write_level(&motors[a].p, 0, 4)) goto out;
        high_mask = 0;
        if (control.path_active && control.path_started && control.tick==control.ticks) {
            if (!ct_path_advance(&control)) { stop_pulses(); goto out; }
            if (!ct_safe(&control,k_uptime_get_32())) { stop_pulses(); goto out; }
            for (unsigned a=0;a<CT_AXES;a++)
                if (control.direction[a] && !write_level(&motors[a].d,direction_level(a),1)) goto out;
            /* LOW is >=312 us at 1600pps: DIR has >=50us setup before
             * the next edge, without restarting the timer or a 50ms pause. */
            if (low_us<100U) low_us=100U;
        }
        if (!control.running || !ct_safe(&control, k_uptime_get_32())) { stop_pulses(); goto out; }
        if (timer_period(low_us, false)) timer_fault();
        goto out;
    }
    if (!ct_safe(&control, k_uptime_get_32())) { stop_pulses(); goto out; }
    for (unsigned a = 0; a < CT_AXES; a++) {
        if (!pin_level(&motors[a].e, 0, 2) || !pin_level(&motors[a].p, 0, 4)) goto out;
        if (control.direction[a] && !pin_level(&motors[a].d, direction_level(a), 1)) goto out;
    }
    uint32_t period = ct_period_us(&control, k_uptime_get_32()), high_us = period / 2U;
    low_us = period - high_us;
    unsigned mask = ct_next_mask(&control);
    if (!control.running) { stop_pulses(); goto out; }
    for (unsigned a = 0; a < CT_AXES; a++) if (mask & (1U << a)) {
        high_mask |= 1U << a; pulse_start_cycles = k_cycle_get_32();
        if (!write_level(&motors[a].p, 1, 3)) goto out;
        pulse_start_cycles = k_cycle_get_32();
        ct_count(&control, a);
    }
    ct_finish_tick(&control);
    /* Even the final rising edge drains HIGH before a new direction/leg. */
    if (timer_period(high_us, false)) timer_fault();
out:
    irq_unlock(key);
}
static void start_motor(void)
{
    bool ok = !motor_error && !timer_running && !high_mask;
    for (unsigned a = 0; a < CT_AXES && ok; a++) {
        ok = pin_level(&motors[a].p, 0, 4) && pin_level(&motors[a].e, 0, 2);
        if (ok && control.direction[a]) ok = write_level(&motors[a].d, direction_level(a), 1);
    }
    if (!ok) { halt("motor_gpio_error"); return; }
    if (timer_period(ZJ_SETUP_MS * 1000U, true) || counter_start(timer)) timer_fault();
    else timer_running = true;
}
static void sensor_thread(void *a, void *b, void *c)
{
    (void)a; (void)b; (void)c;
    while (1) {
        unsigned key = irq_lock(); read_limits();
        bool indicator = false;
        for (unsigned i = 0; i < CT_AXES; i++) indicator |= control.sw[i].top || control.sw[i].bottom || control.sw[i].conflict;
        gpio_pin_set_dt(&led, indicator);
        if (timer_running && (!control.running || !ct_safe(&control, k_uptime_get_32()))) stop_pulses();
        irq_unlock(key); k_msleep(2);
    }
}
K_THREAD_DEFINE(sensor_tid, 1024, sensor_thread, NULL, NULL, NULL, -1, 0, SYS_FOREVER_MS);
static void usb_status(enum usb_dc_status_code code, const uint8_t *param)
{
    (void)param; unsigned key = irq_lock();
    if (code == USB_DC_CONFIGURED || code == USB_DC_RESUME) usb_configured = true;
    if (code == USB_DC_DISCONNECTED || code == USB_DC_RESET || code == USB_DC_SUSPEND || code == USB_DC_ERROR) {
        usb_configured = false; usb_epoch++; control.session = 0; halt("usb_lost");
    }
    irq_unlock(key);
}
static void output_line(const char *s)
{ if (usb_configured) for (; *s; s++) uart_poll_out(usb, (unsigned char)*s); }
static void status_report(void)
{
    /* Compact snapshot avoids copying the whole worker onto the main stack. */
    char output[1536];
    struct {
        int32_t p[3], g[3]; uint32_t range[3], f[3], n1[3], n2[3];
        uint64_t total[3];
        unsigned phases[3], errors[3], failed[3], mask, ready, hold, pol, conflicts;
        uint32_t session, job, epoch, tick, ticks, late;
        unsigned path[4]; uint32_t starved;
        uint32_t recovered, input_at, brief[3], false_hits[3], idle_ignored;
        int input_axis;
        unsigned input_kind, input_bits, raw, io_errors;
        bool input_wait, gate_wait;
        unsigned gate_axis;
        int direction[3]; unsigned dir_levels, mode, selected, pps;
        int32_t coupling_ppm, beta_q, probe_da, probe_db;
        bool coupling_ready;
        bool busy, referenced, fault; enum ct_stage stage;
        const char *reason;
    } s = {0};
    unsigned key = irq_lock();
    s.session = control.session; s.job = control.job; s.epoch = control.reference_epoch;
    s.tick = control.tick; s.ticks = control.ticks; s.late = timer_late;
    s.path[0]=control.path_total; s.path[1]=control.path_received;
    s.path[2]=control.path_done; s.path[3]=CT_PATH_CAP-control.path_size;
    s.starved=control.path_starved;
    s.busy = ct_busy(&control) || timer_running; s.referenced = control.referenced; s.fault = control.fault;
    s.stage = control.stage; s.reason = control.reason;
    s.recovered=control.recovered; s.input_wait=control.input_wait;
    s.idle_ignored=control.idle_ignored; s.gate_wait=control.gate_wait; s.gate_axis=control.gate_axis;
    s.input_axis=control.input_axis; s.input_kind=control.input_kind;
    s.input_bits=control.input_bits; s.input_at=control.input_at_ms;
    s.mode=control.mode; s.selected=control.selected;
    s.pps=control.running ? 1000000U/ct_period_us(&control,k_uptime_get_32()) : 0;
    s.coupling_ppm=control.coupling_ppm; s.beta_q=(int32_t)control.beta;
    s.probe_da=control.probe_da; s.probe_db=control.probe_db; s.coupling_ready=control.coupling_ready;
    for (unsigned a = 0; a < 3; a++) {
        s.p[a] = control.pos[a] - control.origin[a]; s.g[a] = control.goal[a] - control.origin[a];
        s.total[a] = control.total[a];
        s.brief[a]=control.sw[a].brief_both;
        s.false_hits[a]=control.cal[a].false_hits;
        s.io_errors |= (control.sw[a].error ? 1U : 0U) << a;
        s.raw |= ((control.sw[a].raw_pos ? 1U : 0U) | (control.sw[a].raw_neg ? 2U : 0U)) << (2*a);
        s.range[a] = control.range[a]; s.f[a] = control.factor[a];
        s.n1[a] = control.cal[a].n1; s.n2[a] = control.cal[a].n2;
        s.phases[a] = control.cal[a].phase; s.errors[a] = control.cal[a].error; s.failed[a] = control.cal[a].failed_phase;
        s.mask |= (control.sw[a].top || control.sw[a].raw_pos ? 1U : 0U) << (2 * a);
        s.mask |= (control.sw[a].bottom || control.sw[a].raw_neg ? 1U : 0U) << (2 * a + 1);
        s.ready |= (zj_ready(&control.sw[a]) ? 1U : 0U) << a;
        s.hold |= (control.holding[a] ? 1U : 0U) << a;
        s.pol |= (control.pol[a] ? 1U : 0U) << a;
        s.direction[a]=control.direction[a];
        s.dir_levels |= (gpio_pin_get_raw(motors[a].d.port,motors[a].d.pin)==1 ? 1U : 0U) << a;
        s.conflicts |= (control.sw[a].conflict ? 1U : 0U) << a;
    }
    int fault = motor_error, error = timer_errno; irq_unlock(key);
    int n = snprintf(output, sizeof(output),
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
        (unsigned long)s.session, (unsigned long)s.job, (unsigned long)k_uptime_get_32(), s.busy, s.referenced,
        (unsigned long)s.epoch, (unsigned)s.stage,
        (long)s.p[0], (long)s.p[1], (long)s.p[2], (long)s.g[0], (long)s.g[1], (long)s.g[2],
        (unsigned long long)s.total[0], (unsigned long long)s.total[1], (unsigned long long)s.total[2],
        (unsigned long)s.range[0], (unsigned long)s.range[1], (unsigned long)s.range[2],
        (unsigned long)s.f[0], (unsigned long)s.f[1], (unsigned long)s.f[2],
        (unsigned long)s.n1[0], (unsigned long)s.n1[1], (unsigned long)s.n1[2],
        (unsigned long)s.n2[0], (unsigned long)s.n2[1], (unsigned long)s.n2[2],
        s.phases[0], s.phases[1], s.phases[2], s.errors[0], s.errors[1], s.errors[2], s.failed[0], s.failed[1], s.failed[2],
        s.mask, s.ready, s.hold, s.pol, s.conflicts, (unsigned long)s.tick, (unsigned long)s.ticks,
        s.raw, s.io_errors, s.input_wait, (unsigned long)s.recovered, s.input_axis, s.input_kind, s.input_bits, (unsigned long)s.input_at,
        (unsigned long)s.brief[0], (unsigned long)s.brief[1], (unsigned long)s.brief[2],
        (unsigned long)s.idle_ignored,s.gate_wait,s.gate_axis,
        (unsigned long)s.false_hits[0],(unsigned long)s.false_hits[1],(unsigned long)s.false_hits[2],
        s.mode,s.selected,s.pps,s.direction[0],s.direction[1],s.direction[2],s.dir_levels,
        (long)s.coupling_ppm,(long)s.beta_q,(long)s.probe_da,(long)s.probe_db,s.coupling_ready,
        s.path[0],s.path[1],s.path[2],s.path[3],(unsigned long)s.starved,
        fault, s.fault, (unsigned long)s.late, error, s.reason);
    if (n > 0 && (size_t)n < sizeof(output)) output_line(output);
}
static void command(const char *line)
{
    unsigned key = irq_lock();
    if (!usb_configured) { halt("usb_lost"); irq_unlock(key); return; }
    read_limits();
    if (timer_running && !control.running) stop_pulses();
    enum ct_result result = ct_command(&control, line, k_uptime_get_32());
    if (result == CT_HOLD_CHANGE) {
        unsigned a = control.hold_axis; bool success = false;
        if (!motor_error && !timer_running && !high_mask && pin_level(&motors[a].p, 0, 4)) {
            int error = gpio_pin_set_dt(&motors[a].e, control.hold_value);
            k_busy_wait(2);
            if (!error) success = pin_level(&motors[a].e, control.hold_value ? 0 : 1, 2);
        }
        ct_hold_complete(&control, success);
        if (!success) { stop_pulses(); result = CT_REJECTED; control.reply = "motor_fault"; }
    } else if (result == CT_START) {
        start_motor();
        if (control.fault) { result = CT_REJECTED; control.reply = control.reason; }
    } else if (!control.running && timer_running) stop_pulses();
    char op[16] = "INVALID", output[192]; sscanf(line, "%15s", op);
    for (const char *p = op; *p; p++) if (*p < 'A' || *p > 'Z') { strcpy(op, "INVALID"); break; }
    snprintf(output, sizeof(output),
        "{\"type\":\"ack\",\"protocol\":5,\"session\":%lu,\"job\":%lu,\"op\":\"%s\",\"ok\":%d,\"reason\":\"%s\"}\n",
        (unsigned long)control.session, (unsigned long)control.job, op, result != CT_REJECTED, control.reply);
    irq_unlock(key);
    if (strcmp(op, "KEEP")) output_line(output);
}
int main(void)
{
    ct_init(&control);
    if (!gpio_is_ready_dt(&led) || !device_is_ready(timer) || !device_is_ready(usb)) return 0;
    for (unsigned a = 0; a < CT_AXES; a++) {
        const struct motor_pins *m = &motors[a];
        if (!gpio_is_ready_dt(&m->p) || !gpio_is_ready_dt(&m->d) || !gpio_is_ready_dt(&m->e) ||
            gpio_pin_configure_dt(&m->p, GPIO_OUTPUT_INACTIVE) || gpio_pin_configure_dt(&m->d, GPIO_OUTPUT_INACTIVE) ||
            gpio_pin_configure_dt(&m->e, GPIO_OUTPUT_ACTIVE)) return 0;
        for (unsigned i = 0; i < 2; i++)
            if (!gpio_is_ready_dt(&limits[a][i]) || gpio_pin_configure_dt(&limits[a][i], GPIO_INPUT | GPIO_PULL_UP)) return 0;
    }
    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE)) return 0;
    timer_hz = counter_get_frequency(timer);
    if (timer_hz != 1000000U) { motor_error = 5; timer_fault(); }
    k_thread_start(sensor_tid);
    if (usb_enable(usb_status)) return 0;
    char line[192]; size_t used = 0; bool discard = false;
    uint32_t rx_epoch = 0, report_ms = 0;
    while (1) {
        unsigned char ch; unsigned key = irq_lock();
        uint32_t epoch = usb_epoch; bool connected = usb_configured; irq_unlock(key);
        if (rx_epoch != epoch || !connected) {
            used = 0; discard = false; rx_epoch = epoch;
            while (uart_poll_in(usb, &ch) == 0) {}
        }
        for (unsigned n = 0; n < 256 && uart_poll_in(usb, &ch) == 0; n++) {
            if (ch == '\r') continue;
            if (ch == '\n') {
                if (!discard && used) { line[used] = 0; command(line); }
                used = 0; discard = false;
            } else if (!discard) {
                if (used >= sizeof(line) - 1 || ch < 32 || ch > 126) {
                    key = irq_lock(); halt("invalid_command"); irq_unlock(key);
                    used = 0; discard = true;
                } else line[used++] = (char)ch;
            }
        }
        key = irq_lock();
        read_limits();
        if (timer_running && (!control.running || !ct_safe(&control, k_uptime_get_32()))) stop_pulses();
        /* Worker may update a backoff target while LOW; DIR changes only idle. */
        if (!high_mask) {
            enum ct_result result = ct_service(&control, k_uptime_get_32());
            if (result == CT_START) start_motor();
            else if (!control.running && timer_running) stop_pulses();
        }
        irq_unlock(key);
        uint32_t now = k_uptime_get_32();
        if (now - report_ms >= 100U) { report_ms = now; status_report(); }
        k_msleep(2);
    }
    return 0;
}




