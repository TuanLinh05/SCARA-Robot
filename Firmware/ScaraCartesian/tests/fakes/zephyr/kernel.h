#ifndef ZJ_HOST_ZEPHYR_H
#define ZJ_HOST_ZEPHYR_H
/* Narrow host shim for exercising the real main.c ISR and pin mapping. */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define SYS_FOREVER_MS (-1)
struct device { int id; };
extern struct device mock_devices[4];
#define DT_PATH(x) 0
#define DT_ALIAS(x) 0
#define DT_CHOSEN(x) 1
#define DT_NODELABEL(x) 2
#define DEVICE_DT_GET(node) (&mock_devices[node])
uint32_t k_uptime_get_32(void);
uint32_t k_cycle_get_32(void);
static inline uint32_t k_cyc_to_us_floor32(uint32_t cycles) { return cycles / 72U; }
void k_busy_wait(uint32_t us);
void k_msleep(int ms);
static inline bool device_is_ready(const struct device *d) { (void)d; return true; }
static inline unsigned irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned key) { (void)key; }
static inline void k_thread_start(int tid) { (void)tid; }
#define K_THREAD_DEFINE(name, stack, entry, a, b, c, priority, options, delay) \
    static int name; \
    static void (*const mock_thread_entry)(void *, void *, void *) \
        __attribute__((unused)) = entry
struct gpio_dt_spec { const struct device *port; unsigned pin; unsigned dt_flags; };
typedef uint32_t gpio_port_value_t;
#define GPIO_ACTIVE_LOW 1U
#define GPIO_OUTPUT_INACTIVE 2U
#define GPIO_OUTPUT_ACTIVE 4U
#define GPIO_INPUT 8U
#define GPIO_PULL_UP 16U
#define MOCK_PIN_pul_gpios 0
#define MOCK_PIN_dir_gpios 1
#define MOCK_PIN_ena_gpios 2
#define MOCK_PIN_gpios 13
#define MOCK_FLAGS_pul_gpios 0
#define MOCK_FLAGS_dir_gpios 0
#define MOCK_FLAGS_ena_gpios GPIO_ACTIVE_LOW
#define MOCK_FLAGS_gpios GPIO_ACTIVE_LOW
#define MOCK_PIN_j1_pul_gpios 8
#define MOCK_PIN_j1_dir_gpios 15
#define MOCK_PIN_j1_ena_gpios 14
#define MOCK_PIN_j2_pul_gpios 6
#define MOCK_PIN_j2_dir_gpios 7
#define MOCK_PIN_j2_ena_gpios 13
#define MOCK_FLAGS_j1_pul_gpios 0
#define MOCK_FLAGS_j1_dir_gpios 0
#define MOCK_FLAGS_j1_ena_gpios GPIO_ACTIVE_LOW
#define MOCK_FLAGS_j2_pul_gpios 0
#define MOCK_FLAGS_j2_dir_gpios 0
#define MOCK_FLAGS_j2_ena_gpios GPIO_ACTIVE_LOW
#define MOCK_PORT_pul_gpios 0
#define MOCK_PORT_dir_gpios 0
#define MOCK_PORT_ena_gpios 0
#define MOCK_PORT_gpios 0
#define MOCK_PORT_j1_pul_gpios 0
#define MOCK_PORT_j1_dir_gpios 3
#define MOCK_PORT_j1_ena_gpios 3
#define MOCK_PORT_j2_pul_gpios 0
#define MOCK_PORT_j2_dir_gpios 0
#define MOCK_PORT_j2_ena_gpios 3
#define MOCK_PIN_z_pos_gpios 3
#define MOCK_PIN_z_neg_gpios 4
#define MOCK_PIN_j1_pos_gpios 0
#define MOCK_PIN_j1_neg_gpios 1
#define MOCK_PIN_j2_pos_gpios 10
#define MOCK_PIN_j2_neg_gpios 11
#define MOCK_FLAGS_z_pos_gpios GPIO_PULL_UP
#define MOCK_FLAGS_z_neg_gpios GPIO_PULL_UP
#define MOCK_FLAGS_j1_pos_gpios GPIO_PULL_UP
#define MOCK_FLAGS_j1_neg_gpios GPIO_PULL_UP
#define MOCK_FLAGS_j2_pos_gpios GPIO_PULL_UP
#define MOCK_FLAGS_j2_neg_gpios GPIO_PULL_UP
#define MOCK_PORT_z_pos_gpios 0
#define MOCK_PORT_z_neg_gpios 0
#define MOCK_PORT_j1_pos_gpios 3
#define MOCK_PORT_j1_neg_gpios 3
#define MOCK_PORT_j2_pos_gpios 3
#define MOCK_PORT_j2_neg_gpios 3
#define GPIO_DT_SPEC_GET(node, prop) { &mock_devices[MOCK_PORT_##prop], MOCK_PIN_##prop, MOCK_FLAGS_##prop }
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *p) { (void)p; return true; }
int gpio_pin_configure_dt(const struct gpio_dt_spec *p, unsigned flags);
int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value);
int gpio_pin_get(const struct device *port, unsigned pin);
int gpio_pin_get_raw(const struct device *port, unsigned pin);
int gpio_port_get_raw(const struct device *port, gpio_port_value_t *value);
struct counter_top_cfg {
    uint32_t ticks;
    void (*callback)(const struct device *, void *);
    void *user_data;
    uint32_t flags;
};
#define COUNTER_TOP_CFG_DONT_RESET 1U
#define COUNTER_TOP_CFG_RESET_WHEN_LATE 2U
uint32_t counter_get_frequency(const struct device *d);
int counter_start(const struct device *d);
int counter_stop(const struct device *d);
int counter_set_top_value(const struct device *d, const struct counter_top_cfg *cfg);
enum usb_dc_status_code { USB_DC_CONFIGURED, USB_DC_RESUME, USB_DC_DISCONNECTED,
    USB_DC_RESET, USB_DC_SUSPEND, USB_DC_ERROR };
int usb_enable(void (*cb)(enum usb_dc_status_code, const uint8_t *));
int uart_poll_in(const struct device *d, unsigned char *c);
void uart_poll_out(const struct device *d, unsigned char c);
#endif
