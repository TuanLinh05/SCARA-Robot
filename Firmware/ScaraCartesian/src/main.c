/*
 * ============================================================================
 * FILE: main.c
 * ============================================================================
 * [EN] Zephyr RTOS Hardware Entry Point, Hardware Timer ISR & USB Loop
 *      Connects the platform-independent SCARA motion engine (`cart_core.c`)
 *      to STM32 physical hardware:
 *      - 3 Stepper Drivers (`PUL`, `DIR`, `ENA` GPIO pins for `Z`, `J1`, `J2`).
 *      - 6 Normally-Closed Limit Switches (`POS`, `NEG` GPIO pins for 3 axes).
 *      - 1 MHz Hardware Counter/Timer (`jog_timer`) generating jitter-free
 *        stepper pulses inside the `pulse_tick()` Interrupt Service Routine.
 *      - USB CDC-ACM Virtual Serial Port (`zephyr_console`) for PC communication.
 *
 * [VI] Điểm Nhập Hệ thống Zephyr RTOS, Ngắt Timer Phần cứng & Vòng lặp USB
 *      Kết nối bộ điều khiển chuyển động lõi (`cart_core.c`) với phần cứng
 *      vi điều khiển STM32:
 *      - 3 Driver Động cơ Bước (các chân `PUL`, `DIR`, `ENA` cho `Z`, `J1`, `J2`).
 *      - 6 Công tắc Hành trình Thường Đóng (`POS`, `NEG` cho 3 trục).
 *      - Bộ định thời phần cứng 1 MHz (`jog_timer`) phát xung bước chính xác
 *        bên trong trình phục vụ ngắt `pulse_tick()`.
 *      - Cổng nối tiếp ảo USB CDC-ACM (`zephyr_console`) giao tiếp với máy tính.
 *
 * [DEBUG TIP / MẸO GỠ LỖI]:
 * - Every GPIO write to `PUL`, `DIR`, or `ENA` is immediately verified by reading
 *   back the physical pin register (`pin_level()`) after a 2 us settle delay!
 *   If `motor_error` is non-zero in telemetry:
 *     * `motor_error == 1`: `DIR` pin readback mismatch.
 *     * `motor_error == 2`: `ENA` pin readback mismatch.
 *     * `motor_error == 3`: `PUL` pin failed to go HIGH.
 *     * `motor_error == 4`: `PUL` pin failed to return LOW.
 *     * `motor_error == 5`: Hardware timer frequency is not 1 MHz.
 * - Mọi thao tác xuất tín hiệu ra chân `PUL`, `DIR`, `ENA` đều được đọc ngược lại
 *   từ thanh ghi phần cứng (`pin_level()`) sau 2 us để kiểm tra chéo! Nếu biến
 *   `motor_error` khác 0 trên giao diện, hãy tra bảng mã lỗi từ 1..5 ở trên để
 *   biết chính xác chân GPIO nào đang bị chập mạch hoặc cấu hình sai!
 * ============================================================================
 */

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

/*
 * [EN] Devicetree node `/zephyr,user` (defined in `app.overlay`) and minimum
 *      stepper pulse HIGH width (`50 us`).
 * [VI] Nút cấu hình phần cứng `/zephyr,user` (trong file `app.overlay`) và độ rộng
 *      xung mức CAO tối thiểu (`50 us`) cho driver động cơ bước.
 */
#define USER_NODE     DT_PATH(zephyr_user)
#define PULSE_HIGH_US 50U

/*
 * [EN] Stepper driver GPIO pin triplet: Pulse (`p`), Direction (`d`), Enable (`e`).
 * [VI] Bộ 3 chân điều khiển mỗi driver động cơ bước: Xung (`p`), Chiều (`d`), Cho phép (`e`).
 */
struct motor_pins {
    struct gpio_dt_spec p;
    struct gpio_dt_spec d;
    struct gpio_dt_spec e;
};

/*
 * [EN] Devicetree GPIO specifications for all 3 stepper drivers (`Z`, `J1`, `J2`)
 *      and all 6 limit switches (`[axis][0] = POS`, `[axis][1] = NEG`).
 * [VI] Danh sách chân GPIO lấy từ Devicetree cho 3 driver động cơ (`Z`, `J1`, `J2`)
 *      và 6 công tắc hành trình (`[trục][0] = Dương`, `[trục][1] = Âm`).
 */
static const struct motor_pins motors[CT_AXES] = {
    {
        GPIO_DT_SPEC_GET(USER_NODE, pul_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, dir_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, ena_gpios)
    },
    {
        GPIO_DT_SPEC_GET(USER_NODE, j1_pul_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, j1_dir_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, j1_ena_gpios)
    },
    {
        GPIO_DT_SPEC_GET(USER_NODE, j2_pul_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, j2_dir_gpios),
        GPIO_DT_SPEC_GET(USER_NODE, j2_ena_gpios)
    },
};

static const struct gpio_dt_spec limits[CT_AXES][2] = {
    { GPIO_DT_SPEC_GET(USER_NODE, z_pos_gpios),  GPIO_DT_SPEC_GET(USER_NODE, z_neg_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j1_pos_gpios), GPIO_DT_SPEC_GET(USER_NODE, j1_neg_gpios) },
    { GPIO_DT_SPEC_GET(USER_NODE, j2_pos_gpios), GPIO_DT_SPEC_GET(USER_NODE, j2_neg_gpios) },
};

/*
 * [EN] Global controller instance, peripheral handles, and ISR synchronization state:
 *      - `timer_hz`          : Hardware counter clock frequency (expected 1,000,000 Hz).
 *      - `usb_epoch`         : Increments on every USB disconnect/reset to flush stale RX bytes.
 *      - `pulse_start_cycles`: CPU cycle timestamp when `PUL` was driven HIGH.
 *      - `low_us`            : Duration (in microseconds) of the LOW half-period of the current step.
 *      - `high_mask`         : 3-bit mask of axes whose `PUL` pins are currently HIGH.
 *
 * [VI] Biến toàn cục lưu trạng thái bộ điều khiển, thiết bị ngoại vi và đồng bộ ngắt ISR:
 *      - `timer_hz`          : Tần số bộ đếm phần cứng (yêu cầu đúng 1.000.000 Hz = 1 MHz).
 *      - `usb_epoch`         : Tăng lên mỗi khi rút/reset cáp USB để xóa sạch bộ đệm nhận cũ.
 *      - `pulse_start_cycles`: Thời điểm chu kỳ CPU khi kích chân `PUL` lên mức CAO.
 *      - `low_us`            : Thời gian nghỉ mức THẤP (micro-giây) của nửa chu kỳ xung hiện tại.
 *      - `high_mask`         : Mặt nạ 3-bit đánh dấu các trục đang có chân `PUL` ở mức CAO.
 */
static struct ct_state control;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const usb = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *const timer = DEVICE_DT_GET(DT_NODELABEL(jog_timer));

static uint32_t timer_hz, usb_epoch, pulse_start_cycles, low_us, timer_late;
static unsigned high_mask;
static bool timer_running, usb_configured;
static int motor_error, timer_errno;

/*
 * [EN] Read both limit switches of each axis atomically from the GPIO port register
 *      (when both pins share a port) and feed the samples into `ct_sample()`.
 * [VI] Đọc đồng thời cặp công tắc hành trình âm/dương của từng trục từ thanh ghi
 *      cổng GPIO và nạp vào hàm lọc chống dội `ct_sample()`.
 */
static void read_limits(void)
{
    for (unsigned a = 0; a < CT_AXES; a++) {
        gpio_port_value_t pos_bits = 0;
        gpio_port_value_t neg_bits = 0;

        int pos_err = gpio_port_get_raw(limits[a][0].port, &pos_bits);
        int neg_err = 0;
        if (limits[a][1].port == limits[a][0].port) {
            neg_err = pos_err;
            neg_bits = pos_bits;
        } else {
            neg_err = gpio_port_get_raw(limits[a][1].port, &neg_bits);
        }

        int pos_level = pos_err;
        if (pos_err == 0) {
            pos_level = (int)((pos_bits >> limits[a][0].pin) & 1U);
        }

        int neg_level = neg_err;
        if (neg_err == 0) {
            neg_level = (int)((neg_bits >> limits[a][1].pin) & 1U);
        }

        ct_sample(&control, a, pos_level, neg_level, k_uptime_get_32());
    }
}

/*
 * [EN] Ensure any active HIGH step pulse has been held for at least `PULSE_HIGH_US` (50 us)
 *      before pulling the `PUL` pin back LOW.
 * [VI] Đảm bảo xung `PUL` ở mức CAO đã đủ tối thiểu `PULSE_HIGH_US` (50 us) trước khi
 *      kéo xuống mức THẤP để opto-coupler trong driver kịp nhận xung.
 */
static void wait_minimum_high(void)
{
    if (high_mask == 0U) {
        return;
    }
    uint32_t elapsed = k_cyc_to_us_floor32(k_cycle_get_32() - pulse_start_cycles);
    if (elapsed < PULSE_HIGH_US) {
        k_busy_wait(PULSE_HIGH_US - elapsed);
    }
}

/*
 * [EN] Stop the hardware pulse timer and bring all `PUL` pins cleanly to LOW.
 *      Does NOT reset `control`'s calibration worker state so endpoint stops can settle.
 * [VI] Dừng bộ định thời phần cứng và kéo toàn bộ các chân xung `PUL` về mức THẤP an toàn.
 */
static void stop_pulses(void)
{
    counter_stop(timer);
    timer_running = false;
    wait_minimum_high();

    for (unsigned a = 0; a < CT_AXES; a++) {
        int error = gpio_pin_set_dt(&motors[a].p, 0);
        k_busy_wait(2);
        if (error != 0 || gpio_pin_get_raw(motors[a].p.port, motors[a].p.pin) != 0) {
            if (motor_error == 0) {
                motor_error = 4;
            }
            control.fault = true;
            ct_abort(&control, "motor_gpio_error");
        }
    }
    high_mask = 0;
}

/*
 * [EN] Abort controller state with `reason` and immediately stop hardware pulses.
 * [VI] Hủy trạng thái điều khiển với nguyên nhân `reason` và ngắt xung phần cứng ngay.
 */
static void halt(const char *reason)
{
    ct_abort(&control, reason);
    stop_pulses();
}

/*
 * [EN] Verify that physical GPIO pin `p` reads `expected` (0 or 1); otherwise latch `fault`.
 * [VI] Kiểm tra mức điện áp thực tế trên chân GPIO `p` có khớp với `expected` hay không.
 */
static bool pin_level(const struct gpio_dt_spec *p, int expected, int fault)
{
    if (gpio_pin_get_raw(p->port, p->pin) == expected) {
        return true;
    }
    if (motor_error == 0) {
        motor_error = fault;
    }
    control.fault = true;
    halt("motor_gpio_error");
    return false;
}

/*
 * [EN] Drive GPIO pin `p` to logical `value`, wait 2 us, and verify physical pin level.
 * [VI] Xuất mức logic `value` ra chân GPIO `p`, chờ 2 us và đọc kiểm tra lại.
 */
static bool write_level(const struct gpio_dt_spec *p, int value, int fault)
{
    if (gpio_pin_set_dt(p, value) != 0) {
        if (motor_error == 0) {
            motor_error = fault;
        }
        control.fault = true;
        halt("motor_gpio_error");
        return false;
    }
    k_busy_wait(2);
    return pin_level(p, value, fault);
}

/*
 * [EN] Compute physical `DIR` pin level (0 or 1) for axis `a` from direction and polarity.
 * [VI] Tính mức logic xuất ra chân chiều quay `DIR` của trục `a`.
 */
static int direction_level(unsigned a)
{
    return (control.direction[a] > 0) == control.pol[a];
}

static void pulse_tick(const struct device *dev, void *data);

/*
 * [EN] Program the 1 MHz hardware counter top value for the next half-period (`us` microseconds).
 * [VI] Cài đặt chu kỳ đếm tiếp theo (`us` micro-giây) cho bộ định thời phần cứng 1 MHz.
 */
static int timer_period(uint32_t us, bool reset)
{
    uint32_t counter_flags = 0U;
    if (!reset) {
        counter_flags = COUNTER_TOP_CFG_DONT_RESET | COUNTER_TOP_CFG_RESET_WHEN_LATE;
    }
    struct counter_top_cfg cfg = {
        .ticks = (uint32_t)(((uint64_t)timer_hz * us) / 1000000U) - 1U,
        .callback = pulse_tick,
        .flags = counter_flags,
    };
    int error = counter_set_top_value(timer, &cfg);
    if (error == -ETIME && !reset) {
        if (timer_late < UINT32_MAX) {
            timer_late++;
        }
        return 0;
    }
    if (error != 0) {
        timer_errno = error;
    }
    return error;
}

static void timer_fault(void)
{
    control.fault = true;
    halt("timer_error");
}

/*
 * [EN] Hardware Timer Interrupt Service Routine (`pulse_tick`):
 *      Alternates between two phases on every interrupt:
 *      - Phase A (`high_mask != 0`): Pulls active `PUL` pins LOW, advances to the
 *        next path segment if the current segment finished (`ct_path_advance`), and
 *        schedules the LOW half-period (`low_us`).
 *      - Phase B (`high_mask == 0`): Verifies safety (`ct_safe`), computes the next
 *        pulse period (`ct_period_us`) and axis mask (`ct_next_mask`), drives active
 *        `PUL` pins HIGH, updates step counters (`ct_count`), and schedules `high_us`.
 *
 * [VI] Trình Phục vụ Ngắt Định thời Phần cứng (`pulse_tick`):
 *      Luân phiên thực hiện 2 nửa chu kỳ xung:
 *      - Nửa chu kỳ THẤP (`high_mask != 0`): Kéo các chân `PUL` về 0, chuyển sang
 *        đoạn quỹ đạo kế tiếp nếu đã đi hết đoạn hiện tại (`ct_path_advance`), và
 *        hẹn giờ nửa chu kỳ nghỉ `low_us`.
 *      - Nửa chu kỳ CAO (`high_mask == 0`): Kiểm tra an toàn (`ct_safe`), tính chu kỳ
 *        xung (`ct_period_us`) và mặt nạ trục cần phát xung (`ct_next_mask`), kích
 *        chân `PUL` lên 1, cộng bộ đếm bước (`ct_count`) và hẹn giờ `high_us`.
 */
static void pulse_tick(const struct device *dev, void *data)
{
    (void)dev;
    (void)data;

    unsigned key = irq_lock();
    if (motor_error != 0) {
        halt("motor_gpio_error");
        goto out;
    }

    /*
     * [EN] Always sample limit switches first before making any step decision.
     * [VI] Luôn đọc công tắc hành trình đầu tiên trước khi quyết định phát xung.
     */
    read_limits();

    /*
     * [EN] Phase A: Finish HIGH pulse by pulling `PUL` pins LOW.
     * [VI] Pha A: Kết thúc sườn dương bằng cách kéo các chân `PUL` xuống mức THẤP.
     */
    if (high_mask != 0U) {
        wait_minimum_high();
        unsigned mask = high_mask;
        for (unsigned a = 0; a < CT_AXES; a++) {
            if ((mask & (1U << a)) && !write_level(&motors[a].p, 0, 4)) {
                goto out;
            }
        }
        high_mask = 0;

        /*
         * [EN] Seamless segment transition at the end of a continuous path segment.
         * [VI] Chuyển đoạn quỹ đạo mượt mà ngay sau khi hoàn tất sườn xuống của xung cuối.
         */
        if (control.path_active && control.path_started && control.tick == control.ticks) {
            if (!ct_path_advance(&control)) {
                stop_pulses();
                goto out;
            }
            if (!ct_safe(&control, k_uptime_get_32())) {
                stop_pulses();
                goto out;
            }
            for (unsigned a = 0; a < CT_AXES; a++) {
                if (control.direction[a] != 0 &&
                    !write_level(&motors[a].d, direction_level(a), 1)) {
                    goto out;
                }
            }
            if (low_us < 100U) {
                low_us = 100U;
            }
        }

        if (!control.running || !ct_safe(&control, k_uptime_get_32())) {
            stop_pulses();
            goto out;
        }
        if (timer_period(low_us, false) != 0) {
            timer_fault();
        }
        goto out;
    }

    /*
     * [EN] Phase B: Emit rising HIGH edge on axes selected by `ct_next_mask()`.
     * [VI] Pha B: Phát sườn lên mức CAO cho các trục được chọn bởi `ct_next_mask()`.
     */
    if (!ct_safe(&control, k_uptime_get_32())) {
        stop_pulses();
        goto out;
    }
    for (unsigned a = 0; a < CT_AXES; a++) {
        if (!pin_level(&motors[a].e, 0, 2) || !pin_level(&motors[a].p, 0, 4)) {
            goto out;
        }
        if (control.direction[a] != 0 && !pin_level(&motors[a].d, direction_level(a), 1)) {
            goto out;
        }
    }

    uint32_t period = ct_period_us(&control, k_uptime_get_32());
    uint32_t high_us = period / 2U;
    low_us = period - high_us;

    unsigned mask = ct_next_mask(&control);
    if (!control.running) {
        stop_pulses();
        goto out;
    }

    for (unsigned a = 0; a < CT_AXES; a++) {
        if (mask & (1U << a)) {
            high_mask |= (1U << a);
            pulse_start_cycles = k_cycle_get_32();
            if (!write_level(&motors[a].p, 1, 3)) {
                goto out;
            }
            pulse_start_cycles = k_cycle_get_32();
            ct_count(&control, a);
        }
    }

    ct_finish_tick(&control);
    if (timer_period(high_us, false) != 0) {
        timer_fault();
    }

out:
    irq_unlock(key);
}

/*
 * [EN] Set up `DIR` pins and start the hardware counter with an initial 50 ms setup delay.
 * [VI] Thiết lập chân chiều quay `DIR` và khởi động Timer phần cứng với độ trễ ổn định 50 ms.
 */
static void start_motor(void)
{
    bool ok = (motor_error == 0) && (!timer_running) && (high_mask == 0U);
    for (unsigned a = 0; a < CT_AXES && ok; a++) {
        ok = pin_level(&motors[a].p, 0, 4) && pin_level(&motors[a].e, 0, 2);
        if (ok && control.direction[a] != 0) {
            ok = write_level(&motors[a].d, direction_level(a), 1);
        }
    }
    if (!ok) {
        halt("motor_gpio_error");
        return;
    }
    if (timer_period(ZJ_SETUP_MS * 1000U, true) != 0 || counter_start(timer) != 0) {
        timer_fault();
    } else {
        timer_running = true;
    }
}

/*
 * [EN] USB device status callback: halts robot immediately if USB disconnects or resets.
 * [VI] Hàm callback trạng thái USB: dừng robot ngay lập tức nếu cáp USB bị rút hoặc treo.
 */
static void usb_status(enum usb_dc_status_code code, const uint8_t *param)
{
    (void)param;
    unsigned key = irq_lock();
    if (code == USB_DC_CONFIGURED || code == USB_DC_RESUME) {
        usb_configured = true;
    }
    if (code == USB_DC_DISCONNECTED || code == USB_DC_RESET ||
        code == USB_DC_SUSPEND || code == USB_DC_ERROR) {
        usb_configured = false;
        usb_epoch++;
        control.session = 0;
        halt("usb_lost");
    }
    irq_unlock(key);
}

/*
 * [EN] Write a null-terminated ASCII string to the USB CDC-ACM UART.
 * [VI] Gửi chuỗi ký tự kết thúc bằng '\0' lên máy tính qua cổng USB CDC-ACM.
 */
static void output_line(const char *s)
{
    if (usb_configured) {
        for (; *s != '\0'; s++) {
            uart_poll_out(usb, (unsigned char)*s);
        }
    }
}

/*
 * [EN] Capture a thread-safe snapshot under `irq_lock()`, unlock IRQs, and broadcast
 *      the 10 Hz JSON status packet.
 * [VI] Chụp nhanh trạng thái hệ thống trong vùng khóa ngắt, mở khóa ngắt rồi định dạng
 *      và gửi bản tin JSON trạng thái (chu kỳ 100 ms = 10 Hz).
 */
static void status_report(void)
{
    static char output[CT_STATUS_CAPACITY];
    static struct ct_status status;

    unsigned key = irq_lock();
    ct_status_capture(&status, &control, timer_running, timer_late, k_uptime_get_32());
    for (unsigned a = 0; a < CT_AXES; a++) {
        unsigned level = (gpio_pin_get_raw(motors[a].d.port, motors[a].d.pin) == 1) ? 1U : 0U;
        status.dir_levels |= (level << a);
    }
    status.motor_error = motor_error;
    status.timer_error = timer_errno;
    irq_unlock(key);

    int n = ct_status_format(output, sizeof(output), &status, k_uptime_get_32());
    if (n > 0 && (size_t)n < sizeof(output)) {
        output_line(output);
    }
}

/*
 * [EN] Process one complete ASCII command line received from the PC host and send ACK.
 * [VI] Xử lý một dòng lệnh ASCII hoàn chỉnh nhận từ máy tính và gửi phản hồi ACK.
 */
static void command(const char *line)
{
    char op[16];
    char output[CT_ACK_CAPACITY];
    command_operation(line, op);

    unsigned key = irq_lock();
    if (!usb_configured) {
        halt("usb_lost");
        irq_unlock(key);
        return;
    }

    read_limits();
    if (timer_running && !control.running) {
        stop_pulses();
    }

    enum ct_result result = ct_command(&control, line, k_uptime_get_32());
    if (result == CT_HOLD_CHANGE) {
        /*
         * [EN] Toggle motor driver `ENA` pin (active-LOW in devicetree: `hold_value == true`
         *      drives physical pin LOW = `0` to energize motor coils).
         * [VI] Đóng/ngắt chân `ENA` của driver (cấu hình tích cực mức THẤP: `hold_value == true`
         *      xuất mức điện áp THẤP = `0` để cấp dòng giữ cứng trục động cơ).
         */
        unsigned a = control.hold_axis;
        bool success = false;
        if (motor_error == 0 && !timer_running && high_mask == 0U &&
            pin_level(&motors[a].p, 0, 4)) {
            int error = gpio_pin_set_dt(&motors[a].e, control.hold_value);
            k_busy_wait(2);
            if (error == 0) {
                int expected_raw = control.hold_value ? 0 : 1;
                success = pin_level(&motors[a].e, expected_raw, 2);
            }
        }
        ct_hold_complete(&control, success);
        if (!success) {
            stop_pulses();
            result = CT_REJECTED;
            control.reply = "motor_fault";
        }
    } else if (result == CT_START) {
        start_motor();
        if (control.fault) {
            result = CT_REJECTED;
            control.reply = control.reason;
        }
    } else if (!control.running && timer_running) {
        stop_pulses();
    }

    uint32_t session = control.session;
    uint32_t job = control.job;
    const char *reply = control.reply;
    irq_unlock(key);

    /*
     * [EN] All commands except high-frequency `KEEP` heartbeats receive a JSON ACK.
     * [VI] Mọi lệnh (trừ gói tin giữ kết nối `KEEP`) đều được trả về bản tin xác nhận ACK.
     */
    if (strcmp(op, "KEEP") != 0) {
        ct_ack_format(output, sizeof(output), session, job, op, result != CT_REJECTED, reply);
        output_line(output);
    }
}

/*
 * [EN] Firmware `main()` entry point: initializes GPIOs, 1 MHz timer, USB CDC-ACM,
 *      and runs the 2 ms main service/telemetry loop.
 * [VI] Hàm `main()` của Firmware: khởi tạo các chân GPIO, bộ định thời 1 MHz, cổng USB,
 *      và chạy vòng lặp dịch vụ chính với chu kỳ ngủ 2 ms.
 */
int main(void)
{
    ct_init(&control);

    if (!gpio_is_ready_dt(&led) || !device_is_ready(timer) || !device_is_ready(usb)) {
        return 0;
    }

    for (unsigned a = 0; a < CT_AXES; a++) {
        const struct motor_pins *m = &motors[a];
        if (!gpio_is_ready_dt(&m->p) || !gpio_is_ready_dt(&m->d) || !gpio_is_ready_dt(&m->e) ||
            gpio_pin_configure_dt(&m->p, GPIO_OUTPUT_INACTIVE) != 0 ||
            gpio_pin_configure_dt(&m->d, GPIO_OUTPUT_INACTIVE) != 0 ||
            gpio_pin_configure_dt(&m->e, GPIO_OUTPUT_ACTIVE) != 0) {
            return 0;
        }
        for (unsigned i = 0; i < 2U; i++) {
            if (!gpio_is_ready_dt(&limits[a][i]) ||
                gpio_pin_configure_dt(&limits[a][i], GPIO_INPUT | GPIO_PULL_UP) != 0) {
                return 0;
            }
        }
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) != 0) {
        return 0;
    }

    timer_hz = counter_get_frequency(timer);
    if (timer_hz != 1000000U) {
        motor_error = 5;
        timer_fault();
    }

    if (usb_enable(usb_status) != 0) {
        return 0;
    }

    struct command_stream receiver = {0};
    uint32_t rx_epoch = 0;
    uint32_t report_ms = 0;

    while (1) {
        unsigned char ch = 0;
        unsigned key = irq_lock();
        uint32_t epoch = usb_epoch;
        bool connected = usb_configured;
        irq_unlock(key);

        /*
         * [EN] Flush partial serial buffer on USB reconnect or disconnect.
         * [VI] Xóa sạch bộ đệm nhận lệnh nếu cáp USB vừa kết nối lại hoặc bị ngắt.
         */
        if (rx_epoch != epoch || !connected) {
            command_stream_reset(&receiver);
            rx_epoch = epoch;
            while (uart_poll_in(usb, &ch) == 0) {
            }
        }

        /*
         * [EN] Poll up to 256 incoming USB bytes per iteration.
         * [VI] Đọc tối đa 256 byte từ cổng USB mỗi vòng lặp.
         */
        for (unsigned n = 0; n < 256U && uart_poll_in(usb, &ch) == 0; n++) {
            enum command_stream_result received = command_stream_feed(&receiver, ch);
            if (received == COMMAND_COMPLETE) {
                command(receiver.line);
            } else if (received == COMMAND_INVALID) {
                key = irq_lock();
                halt("invalid_command");
                irq_unlock(key);
            }
        }

        /*
         * [EN] Sample limit switches, update onboard LED, and run `ct_service()`.
         * [VI] Đọc công tắc hành trình, cập nhật đèn LED báo chạm cữ và chạy `ct_service()`.
         */
        key = irq_lock();
        read_limits();
        bool indicator = false;
        for (unsigned i = 0; i < CT_AXES; i++) {
            indicator |= control.sw[i].top || control.sw[i].bottom || control.sw[i].conflict;
        }
        gpio_pin_set_dt(&led, indicator);

        if (timer_running && (!control.running || !ct_safe(&control, k_uptime_get_32()))) {
            stop_pulses();
        }

        /*
         * [EN] Service state machine transitions only while `PUL` pins are LOW (`!high_mask`).
         * [VI] Chỉ chuyển trạng thái hoặc đổi chiều `DIR` khi chân xung đang ở mức THẤP (`!high_mask`).
         */
        if (high_mask == 0U) {
            enum ct_result result = ct_service(&control, k_uptime_get_32());
            if (result == CT_START) {
                start_motor();
            } else if (!control.running && timer_running) {
                stop_pulses();
            }
        }
        irq_unlock(key);

        /*
         * [EN] Broadcast JSON telemetry every 100 ms (10 Hz).
         * [VI] Phát bản tin trạng thái JSON định kỳ mỗi 100 ms (tần số 10 Hz).
         */
        uint32_t now = k_uptime_get_32();
        if (now - report_ms >= 100U) {
            report_ms = now;
            status_report();
        }
        k_msleep(2);
    }
    return 0;
}
