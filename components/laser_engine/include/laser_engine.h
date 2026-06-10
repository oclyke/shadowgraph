#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "isr_spi.h"
#include "dacx0004.h"   // dacx0004_add_e (channel addressing)

// Time-ordered command engine: a renderer pushes goto/laser/dwell commands into
// a lock-free queue; a hardware timer paces a consumer that drains the queue and
// drives the DACs.
//
// The consumer runs *entirely in the gptimer ISR* — there is no output task.
// The ISR pops commands and writes the DACs directly through isr_spi (an
// IRAM-safe, polled SPI path), then re-arms itself for the next dwell. This
// removes the per-dwell task wake / context-switch that used to sit between the
// timer firing and the SPI write. See the project memory plan for the design.
//
// Threading: the producer API must be called from a SINGLE renderer task
// (single-producer). The consumer is the single timer ISR (single-consumer).
//
// Hardware setup contract (caller, before laser_engine_init):
//   - SPI buses initialized; each DAC's CS pin routed to its peripheral CS line
//     (e.g. via one priming write through the IDF driver, which adds the device
//     and assigns CS lines in order).
//   - The three isr_spi_dev_t handles below initialized with isr_spi_dev_init.

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    isr_spi_dev_t *galvo_x;      // ISR-safe writer for DAC8871 X (16-bit code)
    isr_spi_dev_t *galvo_y;      // ISR-safe writer for DAC8871 Y
    isr_spi_dev_t *laser;        // ISR-safe writer for DAC80004 (RGB)
    dacx0004_add_e ch_r;         // laser channel assignments (A/B/C)
    dacx0004_add_e ch_g;
    dacx0004_add_e ch_b;
    uint32_t       retry_us;     // re-arm interval on underrun (0 -> 1000)
} laser_engine_cfg_t;

// Set up the queue and the gptimer (consumer ISR). The isr_spi handles in cfg
// must already be initialized. Returns false on bad config or a setup failure.
bool laser_engine_init(const laser_engine_cfg_t *cfg);

// Put the hardware in a safe state (center galvo, blank laser) and begin
// consuming the queue from the timer ISR.
void laser_engine_start(void);

// Producer API (call from one renderer task). Each returns false if the queue
// is full; the caller should retry or yield.
bool laser_engine_goto (uint16_t x, uint16_t y);
bool laser_engine_laser(uint16_t r, uint16_t g, uint16_t b);
bool laser_engine_dwell(uint32_t dt_us);

#ifdef __cplusplus
}
#endif
