#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "laser_command.h"

#ifdef __cplusplus
extern "C" {
#endif

// scene_stream: the receive-side reassembly layer for UDP-streamed scenes.
//
// A scene is one ordered stream of laser_command records (the same TV encoding
// the engine consumes). The sender slices it into UDP datagrams, each tagging a
// contiguous run of commands with per-command sequence numbers. Datagrams may
// arrive out of order, be duplicated (the sender blindly re-sends for
// redundancy), or be lost.
//
// This component:
//   - parses/validates the wire header (scene_packet_*),
//   - reassembles commands into an in-order window keyed by sequence number,
//   - drops duplicates and already-delivered commands (idempotent re-sends),
//   - hands the contiguous run of ready commands to a caller-supplied emit
//     callback (which feeds the laser engine), applying backpressure, and
//   - on caller request, skips an unrecoverable gap to keep the stream live.
//
// PURE LOGIC: no clock, no sockets, no locks. It is single-threaded by
// contract — the caller serializes ingest/drain/skip (the net glue holds a
// mutex; these run at millisecond scale, off the engine's microsecond path).

// ---- wire framing ---------------------------------------------------------
// 16-byte little-endian header, followed by `payload_len` bytes of back-to-back
// TV-encoded commands (laser_command_encode format).
#define SCENE_PACKET_MAGIC    0x5347u   // 'S','G' on the wire (LE)
#define SCENE_PACKET_VERSION  1u
#define SCENE_PACKET_HDR_SIZE 16u

// flags
#define SCENE_FLAG_KEYFRAME   0x01u     // reset the receiver to this base_seq

typedef struct {
    uint16_t magic;        // SCENE_PACKET_MAGIC
    uint8_t  version;      // SCENE_PACKET_VERSION
    uint8_t  flags;        // SCENE_FLAG_*
    uint32_t stream_id;    // scene/session id; a change resets the receiver
    uint32_t base_seq;     // command index of the first command in the payload
    uint16_t count;        // number of commands in the payload
    uint16_t payload_len;  // payload byte length
} scene_packet_hdr_t;

// Serialize a header + payload into dst. Returns total bytes written, or 0 if
// dst is too small. Sets magic/version for you from hdr's other fields.
uint32_t scene_packet_build(uint8_t *dst, uint32_t cap,
                            const scene_packet_hdr_t *hdr,
                            const uint8_t *payload, uint32_t payload_len);

// Parse and validate a received datagram. Returns false on a short buffer, bad
// magic, or wrong version. On success, fills *hdr and points *payload at the
// command bytes; *payload_len is clamped to the bytes actually present (a
// truncated datagram yields a short payload that ingest decodes best-effort).
bool scene_packet_parse(const uint8_t *buf, uint32_t len,
                        scene_packet_hdr_t *hdr,
                        const uint8_t **payload, uint32_t *payload_len);

// ---- reassembly window ----------------------------------------------------
typedef struct {
    laser_command_t cmd;
    bool            present;
} scene_slot_t;

typedef struct {
    scene_slot_t *slots;        // caller-owned array of `cap` slots
    uint32_t      cap;          // window depth (power of two)
    uint32_t      mask;         // cap - 1
    uint32_t      deliver_seq;  // next command index to deliver
    uint32_t      stream_id;    // current locked stream
    bool          started;      // locked onto a stream yet?

    // stats (monotonic; useful for diagnostics/telemetry)
    uint32_t      stored;         // commands newly inserted
    uint32_t      dropped_dup;    // already present or already delivered
    uint32_t      dropped_ahead;  // beyond the window (too far in the future)
    uint32_t      gaps_skipped;   // skip_gap() advances past a lost command
} scene_stream_t;

// Initialize over a caller-provided slot array. cap must be a power of two.
// Returns false on a bad argument.
bool scene_stream_init(scene_stream_t *s, scene_slot_t *slots, uint32_t cap);

// Drop all buffered state and re-lock to `stream_id`/`base_seq` on the next
// ingest. (ingest calls this itself on a stream change or keyframe.)
void scene_stream_reset(scene_stream_t *s);

// Insert a parsed packet's commands at their sequence positions. Handles
// stream-id change / keyframe reset. Returns the count newly stored.
uint32_t scene_stream_ingest(scene_stream_t *s,
                             const scene_packet_hdr_t *hdr,
                             const uint8_t *payload, uint32_t payload_len);

// Deliver the contiguous run from deliver_seq. For each ready command, calls
// emit(cmd, ctx); a false return means downstream is full, so delivery stops
// with that command still buffered (retry on the next call). Returns the number
// delivered.
typedef bool (*scene_emit_fn)(const laser_command_t *cmd, void *ctx);
uint32_t scene_stream_drain(scene_stream_t *s, scene_emit_fn emit, void *ctx);

// True when delivery is stalled at a missing command but buffered data exists
// further ahead — i.e. a gap the caller may choose to skip after a timeout.
bool scene_stream_stalled_at_gap(const scene_stream_t *s);

// Advance deliver_seq past a leading gap to the next buffered command within
// the window, dropping the missing commands. Returns the number of sequence
// positions skipped (0 if not stalled at a gap, or nothing is buffered ahead —
// a genuine underrun, which the caller lets the engine blank through).
uint32_t scene_stream_skip_gap(scene_stream_t *s);

#ifdef __cplusplus
}
#endif
