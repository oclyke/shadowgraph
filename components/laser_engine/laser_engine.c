#include "laser_engine.h"
#include "byte_queue.h"
#include "laser_command.h"

#include "driver/gptimer.h"
#include "esp_attr.h"

#define LASER_Q_CAP    4096u       // power of two
#define TIMER_HZ       1000000u    // 1 tick = 1 us
#define MIN_LEAD_TICKS 4u          // don't arm an alarm closer than this many ticks

// Internal-RAM queue backing store: the consumer reads it from an IRAM-safe ISR,
// so it must not live in (cached) flash. Static .bss is internal DRAM.
static uint8_t            s_qbuf[LASER_Q_CAP];
static byte_queue_t       s_q;
static laser_engine_cfg_t s_cfg;
static gptimer_handle_t   s_timer;
static uint64_t           s_next_deadline;   // intended fire time, in timer ticks
static volatile uint32_t  s_corrupt_count;   // bumped in ISR; inspect from a task

// ---------------------------------------------------------------------------
// DAC wire encoders (build the exact bytes the drivers would send, inline, so
// the ISR never calls the flash-resident, blocking driver cores).
// ---------------------------------------------------------------------------

// DAC8871: 16-bit code, MSB first.
static inline void IRAM_ATTR galvo_write(isr_spi_dev_t *d, uint16_t code)
{
    uint8_t b[2] = { (uint8_t)(code >> 8), (uint8_t)code };
    isr_spi_write(d, b, 16);
}

// DAC80004 WRITEn_UPDATEn: 32-bit frame {Rw=0, cmd, add, dat[16], mod=0}.
// Byte layout mirrors the DACx0004 driver's DAT0..DAT3 packing.
static inline void IRAM_ATTR laser_write_ch(dacx0004_add_e ch, uint16_t v)
{
    uint8_t b[4] = {
        (uint8_t)DACX0004_CMD_WRITEn_UPDATEn,            // Rw=0 | cmd
        (uint8_t)(((uint8_t)ch << 4) | (v >> 12)),       // add | dat[15:12]
        (uint8_t)(v >> 4),                               // dat[11:4]
        (uint8_t)(v << 4),                               // dat[3:0] | mod=0
    };
    isr_spi_write(s_cfg.laser, b, 32);
}

static void IRAM_ATTR blank_laser(void)
{
    laser_write_ch(s_cfg.ch_r, 0);
    laser_write_ch(s_cfg.ch_g, 0);
    laser_write_ch(s_cfg.ch_b, 0);
}

static void IRAM_ATTR dispatch(const laser_command_t *c)
{
    switch (c->type) {
        case LASER_CMD_GOTO:
            // LDAC is held low (transparent latch), so the outputs update on
            // write; X then Y land within ~a couple of us of each other.
            galvo_write(s_cfg.galvo_x, c->pos.x);
            galvo_write(s_cfg.galvo_y, c->pos.y);
            break;
        case LASER_CMD_LASER:
            laser_write_ch(s_cfg.ch_r, c->col.r);
            laser_write_ch(s_cfg.ch_g, c->col.g);
            laser_write_ch(s_cfg.ch_b, c->col.b);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Timer helpers (IRAM under CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM).
// ---------------------------------------------------------------------------
static inline uint64_t IRAM_ATTR now_ticks(void)
{
    uint64_t c = 0;
    gptimer_get_raw_count(s_timer, &c);
    return c;
}

static void IRAM_ATTR arm_at(uint64_t deadline)
{
    gptimer_alarm_config_t alarm = {
        .alarm_count                = deadline,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(s_timer, &alarm);
}

// Consumer-side flush (safe: the consumer owns the read cursor).
static void IRAM_ATTR flush_q(void)
{
    uint8_t scratch[32];
    while (byte_queue_avail(&s_q) > 0) {
        byte_queue_read(&s_q, scratch, sizeof scratch);
    }
}

// Process instantaneous commands back-to-back; on DWELL arm the timer and
// return; on empty blank the laser and re-arm a short retry. Runs in the ISR.
static void IRAM_ATTR drain(void)
{
    for (;;) {
        uint8_t type;
        if (byte_queue_peek(&s_q, &type, 1) < 1) {
            // Underrun: keep the beam safe, hold galvo, retry shortly. Reset
            // the schedule to ~now so resumed data doesn't trigger a catch-up
            // storm of compressed dwells.
            blank_laser();
            s_next_deadline = now_ticks() + s_cfg.retry_us;
            arm_at(s_next_deadline);
            return;
        }

        if ((laser_command_type_t)type == LASER_CMD_DWELL) {
            laser_command_t c;
            if (!laser_command_pop(&s_q, &c)) { goto corrupt; }
            s_next_deadline += c.dwell.dt;        // anchored — never now + dt
            uint64_t now = now_ticks();
            if (s_next_deadline > now + MIN_LEAD_TICKS) {
                arm_at(s_next_deadline);
                if (now_ticks() < s_next_deadline) {
                    return;                        // safely armed; wait for alarm
                }
                // The count passed us during arming: fall through and keep
                // draining; the next dwell will re-arm.
            }
            continue;   // overrun or sub-floor dwell: catch up immediately
        }

        laser_command_t c;
        if (!laser_command_pop(&s_q, &c)) { goto corrupt; }
        dispatch(&c);
    }

corrupt:
    // Unknown/partial record (should be impossible with atomic push). Flush and
    // blank for safety, then wait for the next wake. No logging from the ISR;
    // a task can watch s_corrupt_count.
    s_corrupt_count++;
    flush_q();
    blank_laser();
    s_next_deadline = now_ticks() + s_cfg.retry_us;
    arm_at(s_next_deadline);
}

// Timer ISR: the consumer itself. Drains the queue and drives the DACs directly
// (isr_spi is IRAM-safe and polled), then re-arms for the next dwell. Nothing to
// wake, so no higher-priority task is unblocked.
static bool IRAM_ATTR on_alarm(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_ctx)
{
    (void)timer; (void)edata; (void)user_ctx;
    drain();
    return false;
}

bool laser_engine_init(const laser_engine_cfg_t *cfg)
{
    if (cfg == NULL || cfg->galvo_x == NULL || cfg->galvo_y == NULL ||
        cfg->laser == NULL) {
        return false;
    }
    s_cfg = *cfg;
    if (s_cfg.retry_us == 0) s_cfg.retry_us = 1000;

    if (!byte_queue_init(&s_q, s_qbuf, LASER_Q_CAP)) {
        return false;
    }

    gptimer_config_t tcfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_HZ,
    };
    if (gptimer_new_timer(&tcfg, &s_timer) != ESP_OK) return false;

    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    if (gptimer_register_event_callbacks(s_timer, &cbs, NULL) != ESP_OK) return false;
    if (gptimer_enable(s_timer) != ESP_OK) return false;

    return true;
}

void laser_engine_start(void)
{
    // Put the hardware in a known, safe state. isr_spi works in task context
    // too, so this one-time bring-up is fine here.
    galvo_write(s_cfg.galvo_x, 0x8000);   // mid-scale = zero deflection
    galvo_write(s_cfg.galvo_y, 0x8000);
    blank_laser();

    gptimer_start(s_timer);
    s_next_deadline = now_ticks();

    // Kick off the first drain from the ISR (not here) so the consumer only ever
    // runs in one context — no task/ISR reentrancy on the queue or schedule.
    arm_at(now_ticks() + MIN_LEAD_TICKS);
}

bool laser_engine_goto(uint16_t x, uint16_t y) {
    return laser_command_push_goto(&s_q, x, y);
}
bool laser_engine_laser(uint16_t r, uint16_t g, uint16_t b) {
    return laser_command_push_laser(&s_q, r, g, b);
}
bool laser_engine_dwell(uint32_t dt_us) {
    return laser_command_push_dwell(&s_q, dt_us);
}
