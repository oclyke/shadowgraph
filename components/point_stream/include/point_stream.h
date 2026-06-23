#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "point_ring.h"   // laser_point_t

// Network scene ingest for the ILDA point engine.
//
// A *scene* is a complete frame of points the device loops locally; the host
// streams a new scene to replace it (see tools/svg2scene --stream). The store is
// a lock-free **triple buffer** of three slots in internal DRAM: the TCP receiver
// fills a back slot and publishes it; the renderer takes the latest published
// slot when it wants it. Producer and consumer never touch the same slot, so the
// renderer can push the active scene into the ring with no tearing and no lock.
//
// Memory note: every slot lives in static .bss (internal DRAM). The renderer must
// push points only from here (or another DRAM buffer) — never from a flash-resident
// const — or it will stall on the flash cache (see the sin-LUT regression).

#ifdef __cplusplus
extern "C" {
#endif

// Maximum points per scene (per slot). Three slots of this size are reserved.
#ifndef POINT_STREAM_MAX_PTS
#define POINT_STREAM_MAX_PTS 2048u
#endif

// TCP wire: ["SCN1"][u32 count, little-endian][count * 8-byte laser_point_t].
#define POINT_STREAM_MAGIC "SCN1"
#define POINT_STREAM_ACK   0x06u

// Initialize the triple-buffer store. Call once before point_stream_start and
// before the renderer first calls point_stream_get.
void point_stream_init(void);

// Start the TCP scene-receiver task on the given port. Returns false on failure.
// (Available on the device build only.)
bool point_stream_start(uint16_t port);

// Producer (receiver) side — fill-in-place then publish, avoiding a copy:
//   laser_point_t *b = point_stream_back();      // DRAM slot, capacity MAX_PTS
//   ...write up to MAX_PTS points into b...
//   point_stream_commit(n);                      // publish n of them
laser_point_t *point_stream_back(void);
void           point_stream_commit(uint32_t n);

// Producer convenience: copy n points (clamped to MAX_PTS) into the back slot and
// publish them.
void point_stream_publish(const laser_point_t *pts, uint32_t n);

// Renderer side: fetch the latest published scene. On success sets *pts/*count and
// returns true; returns false if no scene has been published yet. The returned
// buffer stays valid until the next point_stream_get call.
bool point_stream_get(const laser_point_t **pts, uint32_t *count);

#ifdef __cplusplus
}
#endif
