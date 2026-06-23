#include "laser_engine.h"
#include "point_ring.h"

#include "driver/gptimer.h"
#include "esp_attr.h"

#define LASER_RING_CAP 1024u       // points; power of two (8 KB of internal DRAM)
#define TIMER_HZ       1000000u    // 1 tick = 1 us
#define DEFAULT_RATE   30000u      // ILDA "30K" standard point rate

// Point backing store in internal DRAM: the consumer reads it from an IRAM-safe
// ISR, so it must not live in (cached) flash. Static .bss is internal DRAM.
static laser_point_t      s_pbuf[LASER_RING_CAP];
static point_ring_t       s_ring;
static laser_engine_cfg_t s_cfg;
static gptimer_handle_t   s_timer;

// Count of ticks that found the ring empty (producer fell behind). Written only
// by the ISR, read by any task; a free-running counter so a torn read is at
// worst off by one and the trend is what matters.
static volatile uint32_t  s_underruns;

// ---------------------------------------------------------------------------
// DAC wire encoders: build the exact bytes the drivers would send, inline, so
// the ISR never calls the flash-resident, blocking driver cores.
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

// ILDA signed coordinate (center 0) -> DAC8871 unsigned code (center 0x8000).
// Unsigned wrap makes this exact across the full range: -32768->0, 0->0x8000,
// +32767->0xFFFF.
static inline uint16_t IRAM_ATTR ilda_to_code(int16_t c)
{
    return (uint16_t)((int32_t)c + 0x8000);
}

// 8-bit color channel -> 16-bit DAC value, bit-replicated so 0xFF maps to full
// scale 0xFFFF (and 0x00 to 0x0000).
static inline uint16_t IRAM_ATTR color8_to_dac(uint8_t c)
{
    return (uint16_t)(((uint16_t)c << 8) | c);
}

// Timer ISR: the consumer itself. Pops exactly one point and drives the DACs
// directly (isr_spi is IRAM-safe and polled). The cadence is a fixed auto-reload
// alarm, so there is nothing to re-arm and no scheduling math. On underrun it
// holds the galvo at its last position and blanks the beam for safety.
static bool IRAM_ATTR on_alarm(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_ctx)
{
    (void)timer; (void)edata; (void)user_ctx;

    laser_point_t p;
    if (!point_ring_pop(&s_ring, &p)) {
        s_underruns++;          // producer fell behind this tick
        blank_laser();          // underrun: hold last galvo position, beam off
        return false;
    }

    galvo_write(s_cfg.galvo_x, ilda_to_code(p.x));
    galvo_write(s_cfg.galvo_y, ilda_to_code(p.y));
    if (p.status & POINT_BLANK) {
        blank_laser();          // positioned but dark (e.g. travel between figures)
    } else {
        laser_write_ch(s_cfg.ch_r, color8_to_dac(p.r));
        laser_write_ch(s_cfg.ch_g, color8_to_dac(p.g));
        laser_write_ch(s_cfg.ch_b, color8_to_dac(p.b));
    }
    return false;
}

bool laser_engine_init(const laser_engine_cfg_t *cfg)
{
    if (cfg == NULL || cfg->galvo_x == NULL || cfg->galvo_y == NULL ||
        cfg->laser == NULL) {
        return false;
    }
    s_cfg = *cfg;
    if (s_cfg.point_rate_hz == 0) s_cfg.point_rate_hz = DEFAULT_RATE;

    if (!point_ring_init(&s_ring, s_pbuf, LASER_RING_CAP)) {
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

    // Fixed-rate cadence: auto-reload to 0 and fire every `period` ticks, so the
    // ISR draws exactly one point per tick at point_rate_hz (modulo the integer
    // rounding of TIMER_HZ / rate, e.g. 30000 -> 33 ticks -> ~30.3 kpps).
    uint32_t period = TIMER_HZ / s_cfg.point_rate_hz;
    if (period == 0) period = 1;
    gptimer_alarm_config_t alarm = {
        .alarm_count                = period,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true,
    };
    if (gptimer_set_alarm_action(s_timer, &alarm) != ESP_OK) return false;

    return true;
}

void laser_engine_start(void)
{
    // Put the hardware in a known, safe state. isr_spi works in task context
    // too, so this one-time bring-up is fine here.
    galvo_write(s_cfg.galvo_x, 0x8000);   // mid-scale = zero deflection
    galvo_write(s_cfg.galvo_y, 0x8000);
    blank_laser();

    gptimer_start(s_timer);   // auto-reload alarm fires the consumer from here on
}

bool laser_engine_point(const laser_point_t *p) {
    return point_ring_push(&s_ring, p);
}
uint32_t laser_engine_points(const laser_point_t *pts, uint32_t n) {
    return point_ring_push_bulk(&s_ring, pts, n);
}
uint32_t laser_engine_underruns(void) {
    return s_underruns;
}
