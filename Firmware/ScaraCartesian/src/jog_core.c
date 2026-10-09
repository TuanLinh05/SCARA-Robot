#include "jog_core.h"
#include <limits.h>
#include "command_text.h"
#include <string.h>

void zj_init(struct zj_state *s)
{
    memset(s, 0, sizeof(*s));
    s->positive_high = true;
    s->top = s->bottom = true; s->raw_pos = s->raw_neg = 1;
}
/* Manual hold uses physical endpoints and the host lease, not a 3s cutoff.
 * Calibration legs install their own bounded deadlines. */
void zj_init_joint(struct zj_state *s) { zj_init(s); }
void zj_stop(struct zj_state *s, enum zj_reason reason)
{
    s->direction = 0;
    s->reason = reason;
}

bool zj_ready(const struct zj_state *s)
{ return s->ready && s->ready_bottom; }

static void sample_limit(struct zj_state *s, bool lower, int level, uint32_t now)
{
    bool *ready = lower ? &s->ready_bottom : &s->ready;
    bool *active = lower ? &s->bottom : &s->top;
    int *sensor_error = lower ? &s->error_bottom : &s->error_top;
    bool *closing = lower ? &s->closing_bottom : &s->closing;
    uint32_t *closed_ms = lower ? &s->closed_bottom_ms : &s->closed_ms;
    if (level != 0 && level != 1) {
        *closing = false; *active = true;
        if (!*sensor_error) *sensor_error = level < 0 ? level : -1;
        if (!s->error) s->error = *sensor_error;
        zj_stop(s, ZJ_SENSOR);
        return;
    }
    if (level) {
        *active = true; *ready = true; *closing = false;
    } else {
        if (!*closing) { *closing = true; *closed_ms = now; }
        if (now - *closed_ms >= ZJ_RELEASE_MS) { *active = false; *ready = true; }
    }
}

void zj_switches(struct zj_state *s, int positive, int negative, uint32_t now)
{
    bool continuous=s->seen && now-s->sample_ms<=ZJ_SENSOR_MS;
    if (positive==1 && (!continuous || s->raw_pos!=1)) s->opened_ms=now;
    if (negative==1 && (!continuous || s->raw_neg!=1)) s->opened_bottom_ms=now;
    /* A sampling gap cannot count as continuous contact closure. */
    if (s->seen && now - s->sample_ms > ZJ_SENSOR_MS) {
        s->closing = s->closing_bottom = false;
        s->both_pending = false;
    }
    s->seen = true; s->sample_ms = now;
    s->raw_pos = positive == 0 ? 0 : 1; s->raw_neg = negative == 0 ? 0 : 1;
    sample_limit(s, false, positive, now); sample_limit(s, true, negative, now);
    if (s->error) { s->both_pending = false; zj_stop(s, ZJ_SENSOR); return; }
    if (positive == 1 && negative == 1) {
        if (!s->both_pending) { s->both_pending = true; s->both_ms = now; }
        if (now - s->both_ms >= ZJ_CONFLICT_MS) s->conflict = true;
        /* A transient overlap still stops immediately; never auto-resume.
         * Only sustained overlap latches the axis until MCU reset. */
        zj_stop(s, ZJ_CONFLICT); return;
    }
    if (s->both_pending && !s->conflict && s->brief_both < UINT32_MAX) s->brief_both++;
    s->both_pending = false;
    if (s->conflict) { zj_stop(s, ZJ_CONFLICT); return; }
    if (s->direction && ((s->direction > 0) == s->positive_high ? s->top : s->bottom))
        zj_stop(s, (s->direction > 0) == s->positive_high ? ZJ_TOP : ZJ_BOTTOM);
}

static enum zj_reason sensor_guard(const struct zj_state *s, int direction, uint32_t now)
{
    if (s->error) return ZJ_SENSOR;
    if (s->conflict) return ZJ_CONFLICT;
    if (!zj_ready(s)) return ZJ_NOT_READY;
    if (now - s->sample_ms > ZJ_SENSOR_MS)
        return ZJ_STALE_SENSOR;
    if (s->raw_pos && s->raw_neg) return ZJ_CONFLICT;
    /* Endpoints are tied to physical DIR polarity, not editable GUI signs. */
    bool physical_positive = (direction > 0) == s->positive_high;
    if (physical_positive && (s->top || s->raw_pos)) return ZJ_TOP;
    if (!physical_positive && (s->bottom || s->raw_neg)) return ZJ_BOTTOM;
    return ZJ_IDLE;
}

bool zj_safe(struct zj_state *s, uint32_t now)
{
    if (!s->direction) return false;
    enum zj_reason reason = sensor_guard(s, s->direction, now);
    if (reason != ZJ_IDLE) { zj_stop(s, reason); return false; }
    if (now - s->lease_ms >= ZJ_LEASE_MS) { zj_stop(s, ZJ_TIMEOUT); return false; }
    /* Bounded internal calibration legs; manual jog leaves this at zero. */
    if (s->job_budget_ms && now - s->start_ms >= s->job_budget_ms) {
        zj_stop(s, ZJ_BUDGET); return false;
    }
    if (s->total == UINT64_MAX || s->move == UINT32_MAX ||
        (s->direction > 0 && s->position == INT64_MAX) ||
        (s->direction < 0 && s->position == INT64_MIN)) {
        zj_stop(s, ZJ_INVALID);
        return false;
    }
    return true;
}

void zj_count_pulse(struct zj_state *s)
{
    s->position += s->direction;
    s->total++;
    s->move++;
    if (s->target && s->move >= s->target) zj_stop(s, ZJ_DONE);
}

uint32_t zj_period_us(const struct zj_state *s, uint32_t now)
{
    /* Conservative start, then ramp to requested rate over two seconds. */
    uint32_t initial = ZJ_MIN_RATE;
    uint32_t elapsed = now - s->start_ms;
    elapsed = elapsed > ZJ_SETUP_MS ? elapsed - ZJ_SETUP_MS : 0;
    if (elapsed > ZJ_RAMP_MS) elapsed = ZJ_RAMP_MS;
    uint32_t rate = initial + (s->rate - initial) * elapsed / ZJ_RAMP_MS;
    if (!rate) rate = ZJ_MIN_RATE;
    return (1000000U + rate - 1U) / rate;
}

enum zj_command_result zj_begin(struct zj_state *s, int direction, uint32_t rate, uint32_t target, uint32_t now)
{
    if (s->direction || (direction != 1 && direction != -1) || rate < ZJ_MIN_RATE ||
        rate > ZJ_MAX_RATE || target > 1000000U) {
        zj_stop(s, ZJ_INVALID); return ZJ_REJECTED;
    }
    s->target = target; s->rate = rate; s->move = 0;
    enum zj_reason reason = sensor_guard(s, direction, now);
    if (reason != ZJ_IDLE) { zj_stop(s, reason); return ZJ_REJECTED; }
    s->direction = direction; s->start_ms = s->lease_ms = now; s->reason = ZJ_IDLE;
    return ZJ_STARTED;
}

enum zj_command_result zj_command(struct zj_state *s, const char *line, uint32_t now)
{
    char buffer[128], *words[7];
    unsigned count;
    if (!command_split(line, buffer, sizeof(buffer), words, 7, &count, true)) goto invalid;
    if (count == 1 && !strcmp(words[0], "STOP")) {
        zj_stop(s, ZJ_STOP); return ZJ_OK;
    }
    if (count == 1 && !strcmp(words[0], "STATUS")) return ZJ_OK;
    uint32_t session, id, rate, target;
    if (count == 2 && !strcmp(words[0], "HELLO") && command_number_u32(words[1], &session) && session) {
        zj_stop(s, ZJ_STOP);
        s->session = session; s->last_id = s->job = 0;
        return ZJ_OK;
    }
    if (count == 3 && !strcmp(words[0], "KEEP") && command_number_u32(words[1], &session) &&
        command_number_u32(words[2], &id) && session == s->session && session && id == s->job &&
        s->direction && now - s->lease_ms < ZJ_LEASE_MS) {
        s->lease_ms = now; return ZJ_OK;
    }
    /* Late KEEP cannot revive motion, even if main has not checked timeout. */
    if (count == 3 && !strcmp(words[0], "KEEP")) return ZJ_REJECTED;
    if (count == 6 && !strcmp(words[0], "JOG") && command_number_u32(words[1], &session) &&
        command_number_u32(words[2], &id) && command_number_u32(words[4], &rate) && command_number_u32(words[5], &target) &&
        session && session == s->session && id > s->last_id && !s->direction &&
        (!strcmp(words[3], "U") || !strcmp(words[3], "D")) &&
        rate >= ZJ_MIN_RATE && rate <= ZJ_MAX_RATE && target <= 1000000) {
        s->last_id = s->job = id; /* Consume even rejected jobs; no automatic retry. */
        int direction = words[3][0] == 'U' ? 1 : -1;
        return zj_begin(s, direction, rate, target, now);
    }
invalid:
    zj_stop(s, ZJ_INVALID);
    return ZJ_REJECTED;
}

const char *zj_reason_name(enum zj_reason reason)
{
    static const char *const names[] = { "idle", "stop", "top_limit", "link_timeout",
        "sensor_error", "sensor_stale", "sensor_not_ready", "invalid_command",
        "complete", "timer_error", "usb_lost", "bottom_limit", "limits_conflict",
        "motor_gpio_error", "jog_budget", "driver_released", "calibration_error" };
    return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "unknown";
}
