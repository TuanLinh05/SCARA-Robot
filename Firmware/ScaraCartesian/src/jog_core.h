#ifndef SCARA_JOG_CORE_H
#define SCARA_JOG_CORE_H
#include <stdbool.h>
#include <stdint.h>

#define ZJ_RELEASE_MS 20U
#define ZJ_CONFLICT_MS 20U
#define ZJ_LEASE_MS 350U
#define ZJ_SENSOR_MS 30U
#define ZJ_MAX_RATE 6400U
/* Separate Cartesian copy: slow repeatable switch approach at 50 pulses/s. */
#define ZJ_MIN_RATE 50U
#define ZJ_SETUP_MS 50U
#define ZJ_RAMP_MS 2000U

enum zj_reason { ZJ_IDLE, ZJ_STOP, ZJ_TOP, ZJ_TIMEOUT, ZJ_SENSOR,
                 ZJ_STALE_SENSOR, ZJ_NOT_READY, ZJ_INVALID, ZJ_DONE,
                 ZJ_TIMER, ZJ_USB, ZJ_BOTTOM, ZJ_CONFLICT, ZJ_GPIO, ZJ_BUDGET, ZJ_RELEASED, ZJ_CAL };
enum zj_command_result { ZJ_OK, ZJ_STARTED, ZJ_REJECTED, ZJ_HOLD_CHANGE, ZJ_RESUMED };

/* Callers serialize access: IRQ lock on target, normal calls in host tests. */
struct zj_state {
    bool positive_high;
    bool ready, top, ready_bottom, bottom, conflict;
    bool seen, closing, closing_bottom, both_pending;
    int raw_pos, raw_neg, error, error_top, error_bottom, direction;
    uint32_t sample_ms, closed_ms, closed_bottom_ms, job_budget_ms;
    uint32_t opened_ms, opened_bottom_ms;
    uint32_t both_ms, brief_both;
    uint32_t session, last_id, job, target, move;
    uint32_t rate, start_ms, lease_ms;
    uint64_t total;
    int64_t position;
    enum zj_reason reason;
};
void zj_init(struct zj_state *s);
void zj_init_joint(struct zj_state *s);
void zj_stop(struct zj_state *s, enum zj_reason reason);
/* NC to GND with pull-up: HIGH means limit asserted or open wiring.
 * Read the pair together. Trigger immediately; release after stable LOW. */
void zj_switches(struct zj_state *s, int positive, int negative, uint32_t now);
bool zj_ready(const struct zj_state *s);
bool zj_safe(struct zj_state *s, uint32_t now);
void zj_count_pulse(struct zj_state *s);
uint32_t zj_period_us(const struct zj_state *s, uint32_t now);
/* Shared guarded start for a client jog or an internal calibration leg.
 * Does not alter session/job IDs. Caller still controls the pulse timer. */
enum zj_command_result zj_begin(struct zj_state *s, int direction, uint32_t rate, uint32_t target, uint32_t now);
enum zj_command_result zj_command(struct zj_state *s, const char *line, uint32_t now);
const char *zj_reason_name(enum zj_reason reason);
#endif
