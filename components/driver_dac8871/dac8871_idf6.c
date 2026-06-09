#include "dac8871_idf6.h"
#include "driver/gpio.h"

static dac8871_status_e write_16b(uint16_t dat, void *arg);
static dac8871_status_e set_ldac(bool lvl, void *arg);
static dac8871_status_e set_rst(bool lvl, void *arg);

dac8871_if_t dac8871_if_idf6 = {
    .write_16b = write_16b,
    .set_ldac  = set_ldac,
    .set_rst   = set_rst,
};

static dac8871_status_e write_16b(uint16_t dat, void *arg)
{
    dac8871_idf6_arg_t *a = arg;
    if (!a->spi_initialized) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = (int)a->clk_freq,
            .mode           = 1,
            .spics_io_num   = a->cs_pin,
            .queue_size     = (int)a->spi_q_size,
        };
        if (spi_bus_add_device(a->host, &devcfg, &a->spi) != ESP_OK)
            return DAC8871_STAT_ERR;
        a->spi_initialized = true;
    }
    spi_transaction_t t = {
        .length = 16,
        .flags  = SPI_TRANS_USE_TXDATA,
    };
    t.tx_data[0] = (uint8_t)(dat >> 8);
    t.tx_data[1] = (uint8_t)(dat & 0xFF);
    return spi_device_transmit(a->spi, &t) == ESP_OK ? DAC8871_STAT_OK : DAC8871_STAT_ERR;
}

static dac8871_status_e set_ldac(bool lvl, void *arg)
{
    dac8871_idf6_arg_t *a = arg;
    if (!a->ldac_initialized) {
        gpio_reset_pin(a->ldac_pin);
        gpio_set_direction(a->ldac_pin, GPIO_MODE_OUTPUT);
        a->ldac_initialized = true;
    }
    gpio_set_level(a->ldac_pin, (uint32_t)lvl);
    return DAC8871_STAT_OK;
}

static dac8871_status_e set_rst(bool lvl, void *arg)
{
    dac8871_idf6_arg_t *a = arg;
    if (!a->rst_initialized) {
        gpio_reset_pin(a->rst_pin);
        gpio_set_direction(a->rst_pin, GPIO_MODE_OUTPUT);
        a->rst_initialized = true;
    }
    gpio_set_level(a->rst_pin, (uint32_t)lvl);
    return DAC8871_STAT_OK;
}
