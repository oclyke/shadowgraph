#pragma once

#include <stdint.h>
#include <stdbool.h>

// The atomic counters are declared with the C atomic type when compiled as C,
// and with std::atomic<uint32_t> (layout-compatible) when included from C++ —
// so host-side C++ unit tests can include this header directly. (Mirrors the
// byte_queue convention this ring replaces.)
#ifdef __cplusplus
  #include <atomic>
  using point_ring_atomic_u32 = std::atomic<uint32_t>;
#else
  #include <stdatomic.h>
  typedef _Atomic uint32_t point_ring_atomic_u32;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// One displayed point, laid out after the ILDA Image Data Transfer Format
// (2D true-color, "format 5"): signed 16-bit coordinates centered at 0 with +X
// to the right and +Y up, 8-bit-per-channel color, and a status byte carrying
// the blanking and last-point flags. This 8-byte record is the unit the engine
// consumes — one per tick at a fixed sample rate.
typedef struct {
    int16_t x, y;        // ILDA position: -32768..32767, 0 = center
    uint8_t status;      // POINT_BLANK | POINT_LAST
    uint8_t r, g, b;     // 8-bit color (0 = off)
} laser_point_t;

#define POINT_BLANK 0x40u   // beam off at this point (position still honored)
#define POINT_LAST  0x80u   // last point of a frame (for a higher playout clock)

// Single-producer / single-consumer lock-free ring of laser_point_t. The
// producer (renderer task) calls point_ring_push / _push_bulk; the consumer
// (timer ISR) calls point_ring_pop. head advances with a release store only
// after a point is fully written, so the consumer never observes a torn point.
//
// Counters are free-running and unsigned; capacity is a power of two and the
// buffer is indexed with (counter & mask). Modular subtraction (head - tail)
// yields the fill level correctly even across the 32-bit counter wrap.
typedef struct {
    laser_point_t          *buf;
    uint32_t                mask;   // cap - 1; cap must be a power of two
    point_ring_atomic_u32   head;   // producer advances; free-running
    point_ring_atomic_u32   tail;   // consumer advances; free-running
} point_ring_t;

// cap must be a power of two; buf must point to at least cap points.
// Returns false on a bad argument (null, or cap not a power of two).
bool     point_ring_init (point_ring_t *r, laser_point_t *buf, uint32_t cap);

uint32_t point_ring_free (point_ring_t *r);   // points writable now (producer view)
uint32_t point_ring_count(point_ring_t *r);   // points readable now (consumer view)

// Push one point. Returns false if the ring is full (left unchanged).
bool     point_ring_push (point_ring_t *r, const laser_point_t *p);

// Push as many of the n points as fit; returns the number accepted (0..n).
uint32_t point_ring_push_bulk(point_ring_t *r, const laser_point_t *p, uint32_t n);

// Pop one point into *out. Returns false if the ring is empty. ISR/IRAM-safe.
bool     point_ring_pop  (point_ring_t *r, laser_point_t *out);

#ifdef __cplusplus
}
#endif
