#include "byte_queue.h"
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>

static inline bool is_pow2(uint32_t x) {
    return x != 0 && (x & (x - 1)) == 0;
}

bool byte_queue_init(byte_queue_t *q, uint8_t *buf, uint32_t cap) {
    if (q == NULL || buf == NULL || !is_pow2(cap)) {
        return false;
    }
    q->buf  = buf;
    q->mask = cap - 1;
    atomic_store_explicit(&q->head, 0, memory_order_relaxed);
    atomic_store_explicit(&q->tail, 0, memory_order_relaxed);
    return true;
}

uint32_t byte_queue_free(byte_queue_t *q) {
    // The producer owns head, so a relaxed load of it suffices; tail is
    // published by the consumer and must be acquired.
    uint32_t head = atomic_load_explicit(&q->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&q->tail, memory_order_acquire);
    return (q->mask + 1) - (head - tail);
}

uint32_t byte_queue_avail(byte_queue_t *q) {
    uint32_t head = atomic_load_explicit(&q->head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&q->tail, memory_order_relaxed);
    return head - tail;
}

bool byte_queue_write(byte_queue_t *q, const void *src, uint32_t n) {
    uint32_t head = atomic_load_explicit(&q->head, memory_order_relaxed);
    uint32_t tail = atomic_load_explicit(&q->tail, memory_order_acquire);
    uint32_t cap  = q->mask + 1;
    if (cap - (head - tail) < n) {
        return false;   // not enough room — all-or-nothing
    }
    uint32_t off   = head & q->mask;
    uint32_t first = cap - off;   // contiguous bytes until the wrap
    const uint8_t *s = (const uint8_t *)src;
    if (first >= n) {
        memcpy(q->buf + off, s, n);
    } else {
        memcpy(q->buf + off, s, first);
        memcpy(q->buf, s + first, n - first);
    }
    // Publish the data: every byte above is visible before head advances.
    atomic_store_explicit(&q->head, head + n, memory_order_release);
    return true;
}

// Copy n bytes starting at read offset `tail` (handles wrap). Does not advance.
static void copy_out(byte_queue_t *q, uint32_t tail, void *dst, uint32_t n) {
    uint32_t cap   = q->mask + 1;
    uint32_t off   = tail & q->mask;
    uint32_t first = cap - off;
    uint8_t *d = (uint8_t *)dst;
    if (first >= n) {
        memcpy(d, q->buf + off, n);
    } else {
        memcpy(d, q->buf + off, first);
        memcpy(d + first, q->buf, n - first);
    }
}

uint32_t byte_queue_read(byte_queue_t *q, void *dst, uint32_t n) {
    uint32_t head  = atomic_load_explicit(&q->head, memory_order_acquire);
    uint32_t tail  = atomic_load_explicit(&q->tail, memory_order_relaxed);
    uint32_t avail = head - tail;
    if (n > avail) n = avail;
    copy_out(q, tail, dst, n);
    atomic_store_explicit(&q->tail, tail + n, memory_order_release);
    return n;
}

uint32_t byte_queue_peek(byte_queue_t *q, void *dst, uint32_t n) {
    uint32_t head  = atomic_load_explicit(&q->head, memory_order_acquire);
    uint32_t tail  = atomic_load_explicit(&q->tail, memory_order_relaxed);
    uint32_t avail = head - tail;
    if (n > avail) n = avail;
    copy_out(q, tail, dst, n);
    return n;
}
