#include "point_ring.h"
#include <stddef.h>
#include <stdatomic.h>

// The consumer side (count/pop) runs inside the laser_engine timer ISR, which is
// IRAM-safe, so those functions must live in IRAM and touch no flash. On the
// host (unit tests) IRAM_ATTR is a no-op.
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

static inline bool is_pow2(uint32_t x) {
    return x != 0 && (x & (x - 1)) == 0;
}

bool point_ring_init(point_ring_t *r, laser_point_t *buf, uint32_t cap) {
    if (r == NULL || buf == NULL || !is_pow2(cap)) {
        return false;
    }
    r->buf  = buf;
    r->mask = cap - 1;
    atomic_store_explicit(&r->head, 0, memory_order_relaxed);
    atomic_store_explicit(&r->tail, 0, memory_order_relaxed);
    return true;
}

uint32_t point_ring_free(point_ring_t *r) {
    // The producer owns head, so a relaxed load of it suffices; tail is
    // published by the consumer and must be acquired.
    uint32_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    return (r->mask + 1) - (head - tail);
}

uint32_t IRAM_ATTR point_ring_count(point_ring_t *r) {
    uint32_t head = atomic_load_explicit(&r->head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    return head - tail;
}

bool point_ring_push(point_ring_t *r, const laser_point_t *p) {
    uint32_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    if ((r->mask + 1) - (head - tail) < 1) {
        return false;   // full
    }
    r->buf[head & r->mask] = *p;
    // Publish the point: its bytes are visible before head advances.
    atomic_store_explicit(&r->head, head + 1, memory_order_release);
    return true;
}

uint32_t point_ring_push_bulk(point_ring_t *r, const laser_point_t *p, uint32_t n) {
    uint32_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    uint32_t freep = (r->mask + 1) - (head - tail);
    if (n > freep) n = freep;
    for (uint32_t i = 0; i < n; i++) {
        r->buf[(head + i) & r->mask] = p[i];
    }
    atomic_store_explicit(&r->head, head + n, memory_order_release);
    return n;
}

bool IRAM_ATTR point_ring_pop(point_ring_t *r, laser_point_t *out) {
    uint32_t head = atomic_load_explicit(&r->head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    if (head - tail < 1) {
        return false;   // empty
    }
    *out = r->buf[tail & r->mask];
    atomic_store_explicit(&r->tail, tail + 1, memory_order_release);
    return true;
}
