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
// Wire format: **standard ILDA** (Image Data Transfer Format). The device parses
// 2D/3D true-colour sections (format 5 / format 4) straight off the socket, so it
// accepts any conforming ILDA scene — not just svg2scene output (`nc dev 7777 <
// scene.ild` works). Each data section is published as a scene; a 0-record
// terminating header ends the stream and the device replies with a 1-byte ACK.
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

#define POINT_STREAM_ACK   0x06u   // sent after a full ILDA stream is received

// ILDA record decode (portable; used by the receiver, exposed for testing). The
// supported true-colour formats: 5 = 2D (8-byte record), 4 = 3D (10-byte, Z
// dropped). Returns the record size in bytes for a format, or 0 if unsupported.
uint32_t point_stream_ild_recsize(uint8_t format);
// Decode one ILDA data record (big-endian X/Y, status, B,G,R) into a laser_point_t.
// Returns false for an unsupported format. `rec` must hold recsize(format) bytes.
bool point_stream_ild_record(uint8_t format, const uint8_t *rec, laser_point_t *out);

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
