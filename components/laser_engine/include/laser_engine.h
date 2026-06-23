#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "isr_spi.h"
#include "dacx0004.h"   // dacx0004_add_e (channel addressing)
#include "point_ring.h" // laser_point_t (the unit produced/consumed)

// ILDA-style point-stream engine: a renderer pushes points (position + color)
// into a lock-free ring; a hardware timer paces a consumer that pops exactly one
// point per tick at a fixed sample rate and drives the DACs.
//
// The consumer runs *entirely in the gptimer ISR* — there is no output task.
// The ISR pops a point and writes the DACs directly through isr_spi (an
// IRAM-safe, polled SPI path). Because the cadence is a fixed auto-reload, the
// ISR does no scheduling math or interpolation: per tick it just maps the
// point's ILDA coordinates to galvo codes and its 8-bit color to the laser DAC.
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
    uint32_t       point_rate_hz; // points drawn per second (0 -> 30000, ILDA 30K)
} laser_engine_cfg_t;

// Set up the point ring and the gptimer (consumer ISR) at the configured fixed
// sample rate. The isr_spi handles in cfg must already be initialized. Returns
// false on bad config or a setup failure.
bool laser_engine_init(const laser_engine_cfg_t *cfg);

// Put the hardware in a safe state (center galvo, blank laser) and begin
// consuming points from the timer ISR.
void laser_engine_start(void);

// Producer API (call from one renderer task). Position is ILDA-signed (center 0,
// +Y up); color is 8-bit per channel; status carries POINT_BLANK / POINT_LAST.
// point  returns false if the ring is full (the caller should retry or yield).
// points pushes as many of n as fit and returns the number accepted.
bool     laser_engine_point (const laser_point_t *p);
uint32_t laser_engine_points(const laser_point_t *pts, uint32_t n);

// Free-running count of timer ticks that found the ring empty (the producer fell
// behind and the beam was blanked for that tick). A rising count means the
// renderer can't keep up with point_rate_hz.
uint32_t laser_engine_underruns(void);

#ifdef __cplusplus
}
#endif
