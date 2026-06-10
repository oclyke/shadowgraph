#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"

// Time-ordered command engine: a renderer pushes goto/laser/dwell commands,
// a pinned high-priority task drains them and drives the DACs, paced by a
// hardware timer. See the project memory plan for the full design.
//
// Threading: the producer API must be called from a SINGLE renderer task
// (single-producer). The engine owns one consumer task (single-consumer).
typedef struct {
    dac8871_dev_t  *galvo_x;     // initialized DAC8871 device (X axis)
    dac8871_dev_t  *galvo_y;     // initialized DAC8871 device (Y axis)
    dacx0004_dev_t *laser;       // initialized DAC80004 device (RGB)
    dacx0004_add_e  ch_r;        // laser channel assignments
    dacx0004_add_e  ch_g;
    dacx0004_add_e  ch_b;
    uint32_t        retry_us;    // re-arm interval on underrun (0 -> 1000)
    int             task_core;   // core to pin the output task (-1 = no affinity)
    uint32_t        task_prio;   // 0 -> configMAX_PRIORITIES-1
} laser_engine_cfg_t;

// Set up the queue, timer, and consumer task. DAC devices must already be
// initialized (SPI buses up). Returns false on bad config or a setup failure.
bool laser_engine_init(const laser_engine_cfg_t *cfg);

// Prime the DACs to a safe state and begin consuming the queue.
void laser_engine_start(void);

// Producer API (call from one renderer task). Each returns false if the queue
// is full; the caller should retry or yield.
bool laser_engine_goto (uint16_t x, uint16_t y);
bool laser_engine_laser(uint16_t r, uint16_t g, uint16_t b);
bool laser_engine_dwell(uint32_t dt_us);
