#pragma once

#include <stdint.h>
#include <stdbool.h>

// The atomic counters are declared with the C atomic type when compiled as C,
// and with std::atomic<uint32_t> (layout-compatible) when included from C++ —
// so host-side C++ unit tests can include this header directly.
#ifdef __cplusplus
  #include <atomic>
  using byte_queue_atomic_u32 = std::atomic<uint32_t>;
#else
  #include <stdatomic.h>
  typedef _Atomic uint32_t byte_queue_atomic_u32;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Single-producer / single-consumer lock-free byte ring buffer.
//
// The producer calls byte_queue_write; the consumer calls byte_queue_read /
// byte_queue_peek. head is advanced (release store) only after a full record
// is copied in, so a multi-byte record becomes visible to the consumer
// atomically: the consumer never observes a partially written record.
//
// Counters are free-running and unsigned; capacity is a power of two and the
// buffer is indexed with (counter & mask). Modular subtraction (head - tail)
// yields the fill level correctly even across the 32-bit counter wrap.
typedef struct {
    uint8_t               *buf;
    uint32_t               mask;   // cap - 1; cap must be a power of two
    byte_queue_atomic_u32  head;   // producer advances; free-running
    byte_queue_atomic_u32  tail;   // consumer advances; free-running
} byte_queue_t;

// cap must be a power of two; buf must point to at least cap bytes.
// Returns false on a bad argument (null, or cap not a power of two).
bool     byte_queue_init (byte_queue_t *q, uint8_t *buf, uint32_t cap);

uint32_t byte_queue_free (byte_queue_t *q);   // bytes writable now (producer view)
uint32_t byte_queue_avail(byte_queue_t *q);   // bytes readable now (consumer view)

// All-or-nothing: writes exactly n bytes, or nothing if fewer than n are free.
bool     byte_queue_write(byte_queue_t *q, const void *src, uint32_t n);

// Copies up to n bytes out and advances the read cursor; returns count copied.
uint32_t byte_queue_read (byte_queue_t *q, void *dst, uint32_t n);

// Like byte_queue_read but does not advance the read cursor.
uint32_t byte_queue_peek (byte_queue_t *q, void *dst, uint32_t n);

#ifdef __cplusplus
}
#endif
