#pragma once

#include <stdbool.h>
#include "dac8871.h"
#include "driver/spi_master.h"

typedef struct {
    spi_device_handle_t spi;
    spi_host_device_t   host;
    uint32_t            clk_freq;
    uint32_t            spi_q_size;
    int                 clk_pin;
    int                 mosi_pin;
    int                 cs_pin;
    int                 ldac_pin;
    int                 rst_pin;
    bool                spi_initialized;
    bool                ldac_initialized;
    bool                rst_initialized;
} dac8871_idf6_arg_t;

extern dac8871_if_t dac8871_if_idf6;
