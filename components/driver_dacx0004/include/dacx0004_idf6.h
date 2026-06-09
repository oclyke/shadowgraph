#pragma once

#include <stdbool.h>
#include "dacx0004.h"
#include "driver/spi_master.h"

typedef struct {
    spi_device_handle_t spi;
    spi_host_device_t   host;
    uint32_t            clk_freq;
    uint32_t            spi_q_size;
    int                 clk_pin;
    int                 mosi_pin;
    int                 miso_pin;
    int                 sync_pin;
    int                 ldac_pin;
    int                 clr_pin;
    bool                spi_initialized;
    bool                ldac_initialized;
    bool                clr_initialized;
} dacx0004_idf6_arg_t;

extern dacx0004_if_t dacx0004_if_idf6;
