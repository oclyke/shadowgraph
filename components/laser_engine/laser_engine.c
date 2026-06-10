#include "laser_engine.h"
#include "byte_queue.h"
#include "laser_command.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_log.h"

#define LASER_Q_CAP    4096u       // power of two
#define TIMER_HZ       1000000u    // 1 tick = 1 us
#define MIN_LEAD_TICKS 4u          // don't arm an alarm closer than this many ticks

static const char *TAG = "laser_engine";

static uint8_t            s_qbuf[LASER_Q_CAP];
static byte_queue_t       s_q;
static laser_engine_cfg_t s_cfg;
static gptimer_handle_t   s_timer;
static TaskHandle_t       s_task;
static uint64_t           s_next_deadline;   // intended fire time, in timer ticks

// ---------------------------------------------------------------------------
// Timer ISR: only wakes the output task. SPI happens in task context because
// the IDF SPI master driver is not ISR-safe.
// ---------------------------------------------------------------------------
static bool IRAM_ATTR on_alarm(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_ctx)
{
    (void)timer; (void)edata; (void)user_ctx;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

static inline uint64_t now_ticks(void)
{
    uint64_t c = 0;
    gptimer_get_raw_count(s_timer, &c);
    return c;
}

static void arm_at(uint64_t deadline)
{
    gptimer_alarm_config_t alarm = {
        .alarm_count                = deadline,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(s_timer, &alarm);
}

static void blank_laser(void)
{
    dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_r, 0);
    dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_g, 0);
    dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_b, 0);
}

// Consumer-side flush (safe: the consumer owns the read cursor).
static void flush_q(void)
{
    uint8_t scratch[32];
    while (byte_queue_avail(&s_q) > 0) {
        byte_queue_read(&s_q, scratch, sizeof scratch);
    }
}

static void dispatch(const laser_command_t *c)
{
    switch (c->type) {
        case LASER_CMD_GOTO:
            // LDAC is held low (transparent latch), so the output updates on
            // write; X then Y land within ~a couple of us of each other.
            dac8871_set_code(s_cfg.galvo_x, c->pos.x);
            dac8871_set_code(s_cfg.galvo_y, c->pos.y);
            break;
        case LASER_CMD_LASER:
            dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_r, c->col.r);
            dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_g, c->col.g);
            dacx0004_write_update_channel(s_cfg.laser, s_cfg.ch_b, c->col.b);
            break;
        default:
            break;
    }
}

// Process instantaneous commands back-to-back; on DWELL arm the timer and
// return; on empty blank the laser and re-arm a short retry.
static void drain(void)
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
    // blank for safety, then wait for the next wake.
    ESP_LOGW(TAG, "corrupt queue record; flushing");
    flush_q();
    blank_laser();
    s_next_deadline = now_ticks() + s_cfg.retry_us;
    arm_at(s_next_deadline);
}

static void output_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        drain();
    }
}

bool laser_engine_init(const laser_engine_cfg_t *cfg)
{
    if (cfg == NULL || cfg->galvo_x == NULL || cfg->galvo_y == NULL ||
        cfg->laser == NULL) {
        return false;
    }
    s_cfg = *cfg;
    if (s_cfg.retry_us == 0)  s_cfg.retry_us  = 1000;
    if (s_cfg.task_prio == 0) s_cfg.task_prio = configMAX_PRIORITIES - 1;

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

    BaseType_t ok;
    if (s_cfg.task_core < 0) {
        ok = xTaskCreate(output_task, "laser_out", 4096, NULL,
                         s_cfg.task_prio, &s_task);
    } else {
        ok = xTaskCreatePinnedToCore(output_task, "laser_out", 4096, NULL,
                                     s_cfg.task_prio, &s_task, s_cfg.task_core);
    }
    return ok == pdPASS;
}

void laser_engine_start(void)
{
    // Prime the DACs (forces the drivers' lazy spi_bus_add_device off the hot
    // path) and put the hardware in a known, safe state.
    dac8871_set_code(s_cfg.galvo_x, 0x8000);
    dac8871_set_code(s_cfg.galvo_y, 0x8000);
    blank_laser();

    gptimer_start(s_timer);
    s_next_deadline = now_ticks();
    xTaskNotifyGive(s_task);   // kick the first drain
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
