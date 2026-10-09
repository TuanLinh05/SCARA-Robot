/* Three-axis NC home/calibration and coordinated Cartesian motor execution. */
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/irq.h>
#include <string.h>
#include "cart_core.h"
#include "cart_telemetry.h"
#include "command_stream.h"
#include "command_text.h"
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
    char output[CT_STATUS_CAPACITY];
    struct ct_status status;
    unsigned key = irq_lock();
    ct_status_capture(&status, &control, timer_running, timer_late, k_uptime_get_32());
    for (unsigned a = 0; a < CT_AXES; a++)
        status.dir_levels |= (gpio_pin_get_raw(motors[a].d.port, motors[a].d.pin) == 1 ? 1U : 0U) << a;
    status.motor_error = motor_error;
    status.timer_error = timer_errno;
    irq_unlock(key);
    int n = ct_status_format(output, sizeof(output), &status, k_uptime_get_32());
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
    char op[16], output[CT_ACK_CAPACITY];
    command_operation(line, op);
    ct_ack_format(output, sizeof(output), control.session, control.job,
        op, result != CT_REJECTED, control.reply);
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
    struct command_stream receiver = {0};
    uint32_t rx_epoch = 0, report_ms = 0;
    while (1) {
        unsigned char ch; unsigned key = irq_lock();
        uint32_t epoch = usb_epoch; bool connected = usb_configured; irq_unlock(key);
        if (rx_epoch != epoch || !connected) {
            command_stream_reset(&receiver); rx_epoch = epoch;
            while (uart_poll_in(usb, &ch) == 0) {}
        }
        for (unsigned n = 0; n < 256 && uart_poll_in(usb, &ch) == 0; n++) {
            enum command_stream_result received = command_stream_feed(&receiver, ch);
            if (received == COMMAND_COMPLETE) command(receiver.line);
            else if (received == COMMAND_INVALID) {
                key = irq_lock(); halt("invalid_command"); irq_unlock(key);
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




