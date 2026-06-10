#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"   // spi_host_device_t
#include "hal/spi_hal.h"         // spi_hal_context_t, spi_hal_dev_config_t

#ifdef __cplusplus
extern "C" {
#endif

// ISR-safe, polled SPI writer for a single device on an *already brought-up*
// bus.
//
// Why this exists: the IDF SPI master driver (spi_device_transmit /
// spi_device_polling_transmit) takes a bus semaphore on every transfer and so
// cannot be called from an ISR. This wrapper drives the peripheral directly
// through the GPSPI HAL's IRAM-resident transaction functions (spi_hal_iram.c,
// placed in IRAM by CONFIG_SPI_MASTER_ISR_IN_IRAM) plus a couple of inline LL
// pokes. A write is a short register sequence followed by a busy-wait on the
// "transaction done" bit — no locks, no blocking, safe from an IRAM-safe ISR.
//
// Preconditions (done once, from task context, before isr_spi_dev_init):
//   1. spi_bus_initialize(host, ...) for the bus.
//   2. The device's CS pin must be routed to the peripheral CS signal for
//      `cs_id`. The simplest way is to add the device to the bus with the IDF
//      driver (spi_bus_add_device, e.g. via one priming write through the
//      normal driver) — the driver assigns CS lines in add order (first device
//      added = cs_id 0, second = 1, ...). isr_spi then takes the bus over and
//      the IDF driver path is never used again for it.
typedef struct {
    spi_hal_context_t    hal;   // .hw (peripheral regs) + dma_enabled=false
    spi_hal_dev_config_t dev;   // clock/mode/CS, computed once at init
} isr_spi_dev_t;

// Configure `d` to drive the device on `host` using hardware CS line `cs_id`
// (0..2, matching spi_bus_add_device order), SPI `mode` (0..3) and bus clock
// `clk_hz`. Programs the peripheral for this device (clock divider, mode, CS).
// Returns ESP_OK, or an error if the requested clock cannot be realized.
esp_err_t isr_spi_dev_init(isr_spi_dev_t *d, spi_host_device_t host,
                           int cs_id, int mode, int clk_hz);

// Write `nbits` bits from `data` (sent MSB first, full-duplex, RX ignored).
// IRAM-safe: callable from an IRAM-safe ISR. Busy-waits until the transfer
// completes (a few hundred ns at tens of MHz for the 16/32-bit DAC words).
void isr_spi_write(isr_spi_dev_t *d, const uint8_t *data, int nbits);

#ifdef __cplusplus
}
#endif
