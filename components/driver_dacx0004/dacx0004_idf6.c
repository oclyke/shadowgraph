#include "dacx0004_idf6.h"
#include "driver/gpio.h"

static dacx0004_status_e shift_sr(uint8_t *pdat, uint32_t len, void *arg);
static dacx0004_status_e shift_sr_rw(uint8_t *tx, uint8_t *rx, uint32_t len, void *arg);
static dacx0004_status_e set_ldac(bool lvl, void *arg);
static dacx0004_status_e set_clr(bool lvl, void *arg);

dacx0004_if_t dacx0004_if_idf6 = {
    .shift_sr    = shift_sr,
    .shift_sr_rw = shift_sr_rw,
    .set_sync    = NULL,    // sync is managed by the SPI CS line
    .set_ldac    = set_ldac,
    .set_clr     = set_clr,
};

static esp_err_t ensure_spi(dacx0004_idf6_arg_t *a)
{
    if (a->spi_initialized) return ESP_OK;
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (int)a->clk_freq,
        .mode           = 2,
        .spics_io_num   = a->sync_pin,
        .queue_size     = (int)a->spi_q_size,
    };
    esp_err_t ret = spi_bus_add_device(a->host, &devcfg, &a->spi);
    if (ret == ESP_OK) a->spi_initialized = true;
    return ret;
}

static dacx0004_status_e shift_sr(uint8_t *pdat, uint32_t len, void *arg)
{
    dacx0004_idf6_arg_t *a = arg;
    if (len % 4 != 0) return DACX0004_STAT_ERR_INVALID_ARG;
    if (ensure_spi(a) != ESP_OK) return DACX0004_STAT_ERR;
    esp_err_t ret = ESP_OK;
    for (uint32_t i = 0; i < len / 4; i++) {
        spi_transaction_t t = {
            .length    = 32,
            .tx_buffer = pdat + i * 4,
        };
        ret |= spi_device_polling_transmit(a->spi, &t);
    }
    return ret == ESP_OK ? DACX0004_STAT_OK : DACX0004_STAT_ERR;
}

static dacx0004_status_e shift_sr_rw(uint8_t *tx, uint8_t *rx, uint32_t len, void *arg)
{
    dacx0004_idf6_arg_t *a = arg;
    if (len % 4 != 0) return DACX0004_STAT_ERR_INVALID_ARG;
    if (ensure_spi(a) != ESP_OK) return DACX0004_STAT_ERR;
    esp_err_t ret = ESP_OK;
    for (uint32_t i = 0; i < len / 4; i++) {
        spi_transaction_t t = {
            .length    = 32,
            .tx_buffer = tx + i * 4,
            .rx_buffer = rx + i * 4,
        };
        ret |= spi_device_polling_transmit(a->spi, &t);
    }
    return ret == ESP_OK ? DACX0004_STAT_OK : DACX0004_STAT_ERR;
}

static dacx0004_status_e set_ldac(bool lvl, void *arg)
{
    dacx0004_idf6_arg_t *a = arg;
    if (!a->ldac_initialized) {
        gpio_reset_pin(a->ldac_pin);
        gpio_set_direction(a->ldac_pin, GPIO_MODE_OUTPUT);
        a->ldac_initialized = true;
    }
    gpio_set_level(a->ldac_pin, (uint32_t)lvl);
    return DACX0004_STAT_OK;
}

static dacx0004_status_e set_clr(bool lvl, void *arg)
{
    dacx0004_idf6_arg_t *a = arg;
    if (!a->clr_initialized) {
        gpio_reset_pin(a->clr_pin);
        gpio_set_direction(a->clr_pin, GPIO_MODE_OUTPUT);
        a->clr_initialized = true;
    }
    gpio_set_level(a->clr_pin, (uint32_t)lvl);
    return DACX0004_STAT_OK;
}
