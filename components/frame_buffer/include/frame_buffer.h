#pragma once

#include <stdint.h>
#include <stdbool.h>

// frame_buffer: a FIFO of variable-length frames in one continuous byte arena.
//
// A *frame* is an opaque blob (for shadowgraph, a run of laser_command bytes).
// Frames are laid down back-to-back in the arena and addressed by a caller-
// assigned id. The device loops one *playing* frame; new frames stream in behind
// the write head; old frames (behind the playing one) are reclaimed to make room.
// See docs/FRAME_STREAMING.md for the full contract.
//
// Key properties:
//   - Single continuous arena, caller-sized (e.g. 96 KB).
//   - FORBIDDEN WRAP: a frame never straddles the physical end of the arena, so
//     the playing frame is always one contiguous (ptr, len) slice. When a frame
//     will not fit in the contiguous tail, the tail bytes are padded (charged to
//     the preceding frame's span) and the new frame is placed at offset 0.
//   - Reserve/commit: the writer reserves a contiguous region, fills it in place
//     (e.g. recv() straight off a socket), then commits it with an id. This is
//     zero-copy on the receive path.
//   - Reclaim is floored at the playing frame: the writer may evict frames OLDER
//     than the playing one, never the playing frame or any frame ahead of it.
//
// Threading: NOT internally synchronized. The caller must serialize all calls
// (the firmware wraps these in a mutex; operations are task-level, ms-scale, off
// the ISR path). Single-threaded host tests need no lock.

#ifdef __cplusplus
extern "C" {
#endif

// One resident frame. `off`/`len` describe the payload the reader replays;
// `span` is the arena footprint (>= len) including any trailing pad that wraps
// the write head, so eviction reclaims pad and payload together.
typedef struct {
    uint16_t id;
    uint32_t off;     // payload offset into the arena
    uint32_t len;     // payload length
    uint32_t span;    // arena bytes occupied (len + trailing pad)
} frame_desc_t;

typedef struct {
    uint8_t      *arena;
    uint32_t      cap;        // arena size in bytes
    frame_desc_t *descs;      // caller-provided descriptor ring
    uint32_t      desc_mask;  // desc_cap - 1 (desc_cap must be a power of two)

    uint32_t      d_head;     // free-running: next descriptor slot
    uint32_t      d_tail;     // free-running: oldest descriptor
    uint32_t      head_off;   // next placement offset, [0, cap]
    uint32_t      fill;       // arena bytes currently occupied

    uint32_t      pend_off;   // in-flight reservation
    uint32_t      pend_len;
    bool          pending;

    uint32_t      read_idx;      // next frame to dequeue on advance (free-running)
    uint32_t      cur_idx;       // currently displayed frame
    bool          playing;       // a frame is currently displayed
} frame_buffer_t;

// Initialize over a caller-owned arena (cap bytes) and descriptor array
// (desc_cap entries, desc_cap a power of two = max resident frame count).
// Returns false on a bad argument.
bool frame_buffer_init(frame_buffer_t *fb, uint8_t *arena, uint32_t cap,
                       frame_desc_t *descs, uint32_t desc_cap);

// Reserve a contiguous region of `len` payload bytes for the next frame,
// evicting frames behind the playing one as needed. Returns a writable pointer
// into the arena, or NULL if the frame cannot fit even after all reclaimable
// frames are evicted (backpressure — the caller should stop reading its source)
// or if len is 0 or larger than the arena. Only one reservation may be in flight.
uint8_t *frame_buffer_reserve(frame_buffer_t *fb, uint32_t len);

// Commit the in-flight reservation as a frame addressed by `id`. After this the
// frame is resident and findable via frame_buffer_get. Returns false if there is
// no pending reservation.
bool frame_buffer_commit(frame_buffer_t *fb, uint16_t id);

// Drop a pending reservation without committing it (e.g. the receive failed).
void frame_buffer_abort(frame_buffer_t *fb);

// Find a resident frame by id. On success sets *payload/*len to its bytes and
// returns true; returns false if no resident frame has that id.
bool frame_buffer_get(const frame_buffer_t *fb, uint16_t id,
                      const uint8_t **payload, uint32_t *len);

// Playout (FIFO consume, "go next" semantics — NO wrap).
//   - advance: dequeue the next frame in commit order and make it the displayed
//     frame; returns its bytes via *payload/*len and true. If no unplayed frame
//     remains (advanced past the last received frame), the queue goes EMPTY:
//     nothing is displayed and it returns false (the caller should blank — the
//     engine underruns safely). Further advances stay empty until a new frame is
//     received and advanced into.
//   - current: re-read the displayed frame without advancing (the pump calls this
//     each loop). Returns false when the queue is empty (blank).
// The displayed frame, and any received-but-not-yet-played frames ahead of it,
// are the reclaim floor: the writer never evicts them.
bool frame_buffer_advance(frame_buffer_t *fb, const uint8_t **payload, uint32_t *len);
bool frame_buffer_current(frame_buffer_t *fb, const uint8_t **payload, uint32_t *len);

// Introspection (clamping logic / tests).
uint32_t frame_buffer_count(const frame_buffer_t *fb);          // resident frames
bool     frame_buffer_oldest_id(const frame_buffer_t *fb, uint16_t *id);
bool     frame_buffer_newest_id(const frame_buffer_t *fb, uint16_t *id);

#ifdef __cplusplus
}
#endif
