#include "isr_spi.h"

#include <string.h>
#include "esp_attr.h"
#include "hal/spi_ll.h"        // SPI_LL_GET_HW, spi_ll_master_select_cs (inline)

// On the ESP32 the default SPI clock source is APB (80 MHz). isr_spi only
// supports SPI_CLK_SRC_DEFAULT, which keeps clock-divider math identical to
// what the IDF driver computes for the priming writes.
#define ISR_SPI_CLK_SRC_HZ   80000000

esp_err_t isr_spi_dev_init(isr_spi_dev_t *d, spi_host_device_t host,
                           int cs_id, int mode, int clk_hz)
{
    if (d == NULL || cs_id < 0 || cs_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(d, 0, sizeof *d);

    d->hal.hw          = SPI_LL_GET_HW(host);
    d->hal.dma_enabled = false;

    // Pre-compute the clock-divider register value. half_duplex/no_compensate
    // are both set: these are write-only devices (we never sample MISO), so the
    // RX-timing dummy-bit compensation — which would otherwise reject high
    // clocks — does not apply and must not be added.
    spi_hal_timing_param_t tp = {
        .clk_src_hz     = ISR_SPI_CLK_SRC_HZ,
        .half_duplex    = 0,
        .no_compensate  = 1,
        .expected_freq  = clk_hz,
        .duty_cycle     = 128,   // 50 %
        .input_delay_ns = 0,
        .use_gpio       = false,
    };
    esp_err_t ret = spi_hal_cal_clock_conf(&tp, &d->dev.timing_conf);
    if (ret != ESP_OK) {
        return ret;
    }
    d->dev.timing_conf.clock_source     = SPI_CLK_SRC_DEFAULT;
    d->dev.timing_conf.source_real_freq = ISR_SPI_CLK_SRC_HZ;
    d->dev.timing_conf.rx_sample_point  = SPI_SAMPLING_POINT_PHASE_0;

    d->dev.mode          = mode;
    d->dev.cs_pin_id     = cs_id;
    d->dev.cs_setup      = 0;
    d->dev.cs_hold       = 0;
    d->dev.tx_lsbfirst   = 0;   // MSB first
    d->dev.rx_lsbfirst   = 0;
    d->dev.half_duplex   = 0;
    d->dev.sio           = 0;
    d->dev.no_compensate = 1;
    d->dev.positive_cs   = 0;

    // Apply clock/mode/CS to the peripheral now (task context).
    spi_hal_setup_device(&d->hal, &d->dev);
    return ESP_OK;
}

void IRAM_ATTR isr_spi_write(isr_spi_dev_t *d, const uint8_t *data, int nbits)
{
    // Re-select this device's CS every transfer so multiple isr_spi_dev_t that
    // share one bus (e.g. galvo X on cs0 and galvo Y on cs1) stay correct
    // regardless of which one wrote last.
    spi_ll_master_select_cs(d->hal.hw, d->dev.cs_pin_id);

    spi_hal_trans_config_t t = {
        .cmd            = 0,
        .cmd_bits       = 0,
        .addr_bits      = 0,
        .dummy_bits     = 0,
        .tx_bitlen      = nbits,
        .rx_bitlen      = 0,
        .addr           = 0,
        .send_buffer    = (uint8_t *)data,
        .rcv_buffer     = NULL,
        .line_mode      = { .cmd_lines = 1, .addr_lines = 1, .data_lines = 1 },
        .cs_keep_active = 0,
    };
    spi_hal_setup_trans(&d->hal, &d->dev, &t);
    spi_hal_push_tx_buffer(&d->hal, &t);
    spi_hal_enable_data_line(d->hal.hw, true, false);   // MOSI on, MISO off
    spi_hal_user_start(&d->hal);
    while (!spi_hal_usr_is_done(&d->hal)) {
        // busy-wait; a 32-bit word at 40 MHz completes in <1 us
    }
}
