#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// renderer: a source multiplexer. Exactly one *source* is active at a time and
// produces laser commands; the mux owns the single producer task, so whatever
// the active source does inside pump() is the laser_engine's single producer.
// A UI (or a control packet, or an Art-Net channel) switches sources at runtime
// via renderer_select(); the swap happens at a safe boundary between pumps.
//
// A source is a small vtable. Contract:
//   start(ctx): becoming active — (re)initialize, blank/safe state.
//   stop(ctx):  switching away — release the beam / cleanup.
//   pump(ctx):  emit a *bounded* batch of commands via laser_engine_*, then
//               return. pump MUST block or yield when it has no work (e.g. on a
//               full engine queue or an empty socket) so the mux task doesn't
//               spin. It is only ever called on the mux task.
typedef struct {
    const char *name;
    void (*start)(void *ctx);
    void (*stop)(void *ctx);
    void (*pump)(void *ctx);
    void *ctx;
} renderer_source_t;

#ifndef RENDERER_MAX_SOURCES
#define RENDERER_MAX_SOURCES 8
#endif

typedef struct {
    int      task_core;   // core to pin the mux task to (-1 = no affinity)
    uint32_t task_prio;   // 0 -> 5
} renderer_cfg_t;

// Create the mux and start its producer task. Returns false on failure.
bool renderer_init(const renderer_cfg_t *cfg);

// Register a source under id [0, RENDERER_MAX_SOURCES). The pointer must remain
// valid for the program's lifetime. Returns false on a bad id. Convention: id 0
// is the safe default/idle source.
bool renderer_register(int id, const renderer_source_t *src);

// Request a switch to source `id`, applied by the mux task between pumps. Safe
// to call from any task. A no-op if `id` is out of range or unregistered.
void renderer_select(int id);

// The currently active source id, or -1 if none has run yet.
int renderer_active(void);

#ifdef __cplusplus
}
#endif
