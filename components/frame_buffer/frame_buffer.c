// frame_buffer: a FIFO of variable-length frames in one continuous byte arena.
// See frame_buffer.h and docs/FRAME_STREAMING.md for the contract.
//
// The arena behaves as a ring of bytes, but records are never split across the
// physical end (forbidden wrap): when a frame would straddle the end, the unused
// tail is padded and the frame restarts at offset 0. The pad is charged to the
// preceding frame's `span` so a single FIFO eviction reclaims it. Eviction runs
// from the oldest frame and stops at the playing frame, so the playing frame's
// bytes are stable while the reader replays them.
#include "frame_buffer.h"
#include <stddef.h>

static inline bool is_pow2(uint32_t x) {
    return x != 0 && (x & (x - 1)) == 0;
}

bool frame_buffer_init(frame_buffer_t *fb, uint8_t *arena, uint32_t cap,
                       frame_desc_t *descs, uint32_t desc_cap) {
    if (fb == NULL || arena == NULL || descs == NULL ||
        cap == 0 || !is_pow2(desc_cap)) {
        return false;
    }
    fb->arena     = arena;
    fb->cap       = cap;
    fb->descs     = descs;
    fb->desc_mask = desc_cap - 1;
    fb->d_head    = 0;
    fb->d_tail    = 0;
    fb->head_off  = 0;
    fb->fill      = 0;
    fb->pend_off  = 0;
    fb->pend_len  = 0;
    fb->pending   = false;
    fb->read_idx  = 0;
    fb->cur_idx   = 0;
    fb->playing   = false;
    return true;
}

static uint32_t fb_count(const frame_buffer_t *fb) {
    return fb->d_head - fb->d_tail;
}

// Evict the oldest frame. Returns false if the buffer is empty or the oldest
// frame is the playing frame (the reclaim floor).
static bool evict_oldest(frame_buffer_t *fb) {
    if (fb->d_head == fb->d_tail) {
        return false;
    }
    // Reclaim floor: the displayed frame (if any) plus every received-but-unplayed
    // frame ahead of it must be retained. When the queue is empty the floor is the
    // read cursor, so everything already played is reclaimable.
    uint32_t floor = fb->playing ? fb->cur_idx : fb->read_idx;
    if (fb->d_tail == floor) {
        return false;
    }
    uint32_t idx = fb->d_tail & fb->desc_mask;
    fb->fill -= fb->descs[idx].span;
    fb->d_tail++;
    if (fb->d_head == fb->d_tail) {
        // Empty: the write head can safely restart at the arena base.
        fb->head_off = 0;
        fb->fill     = 0;
    }
    return true;
}

uint8_t *frame_buffer_reserve(frame_buffer_t *fb, uint32_t len) {
    if (len == 0 || len > fb->cap) {
        return NULL;
    }
    fb->pending = false;
    const uint32_t desc_cap = fb->desc_mask + 1;

    for (;;) {
        uint32_t tail_space = fb->cap - fb->head_off;   // contiguous bytes to end
        bool     wrap       = (len > tail_space);       // can't fit before the end
        uint32_t pad        = wrap ? tail_space : 0;
        uint32_t need       = pad + len;

        bool space_ok = (fb->cap - fb->fill) >= need;
        bool slot_ok  = fb_count(fb) < desc_cap;

        if (space_ok && slot_ok) {
            if (wrap) {
                // Charge the pad to the newest frame's span (there is always one
                // when wrapping: an empty buffer has head_off == 0, tail_space ==
                // cap, so len <= cap never wraps), then restart at the base.
                uint32_t newest = (fb->d_head - 1) & fb->desc_mask;
                fb->descs[newest].span += pad;
                fb->fill    += pad;
                fb->head_off = 0;
            }
            fb->pend_off = fb->head_off;
            fb->pend_len = len;
            fb->pending  = true;
            return fb->arena + fb->pend_off;
        }

        if (!evict_oldest(fb)) {
            return NULL;   // nothing more reclaimable — backpressure
        }
    }
}

bool frame_buffer_commit(frame_buffer_t *fb, uint16_t id) {
    if (!fb->pending) {
        return false;
    }
    uint32_t idx = fb->d_head & fb->desc_mask;
    fb->descs[idx].id   = id;
    fb->descs[idx].off  = fb->pend_off;
    fb->descs[idx].len  = fb->pend_len;
    fb->descs[idx].span = fb->pend_len;
    fb->d_head++;
    fb->head_off = fb->pend_off + fb->pend_len;
    fb->fill    += fb->pend_len;
    fb->pending  = false;
    return true;
}

void frame_buffer_abort(frame_buffer_t *fb) {
    fb->pending = false;
}

bool frame_buffer_get(const frame_buffer_t *fb, uint16_t id,
                      const uint8_t **payload, uint32_t *len) {
    for (uint32_t i = fb->d_tail; i != fb->d_head; i++) {
        uint32_t idx = i & fb->desc_mask;
        if (fb->descs[idx].id == id) {
            if (payload) *payload = fb->arena + fb->descs[idx].off;
            if (len)     *len     = fb->descs[idx].len;
            return true;
        }
    }
    return false;
}

// Point *payload/*len at the descriptor at free-running index `idx`.
static void emit_at(const frame_buffer_t *fb, uint32_t idx,
                    const uint8_t **payload, uint32_t *len) {
    const frame_desc_t *d = &fb->descs[idx & fb->desc_mask];
    if (payload) *payload = fb->arena + d->off;
    if (len)     *len     = d->len;
}

bool frame_buffer_advance(frame_buffer_t *fb,
                          const uint8_t **payload, uint32_t *len) {
    if (fb->read_idx == fb->d_head) {
        // Advanced past the last received frame — queue is empty. NO wrap.
        fb->playing = false;
        return false;
    }
    fb->cur_idx  = fb->read_idx;
    fb->read_idx++;
    fb->playing  = true;
    emit_at(fb, fb->cur_idx, payload, len);
    return true;
}

bool frame_buffer_current(frame_buffer_t *fb,
                          const uint8_t **payload, uint32_t *len) {
    if (!fb->playing) return false;
    emit_at(fb, fb->cur_idx, payload, len);
    return true;
}

uint32_t frame_buffer_count(const frame_buffer_t *fb) {
    return fb_count(fb);
}

bool frame_buffer_oldest_id(const frame_buffer_t *fb, uint16_t *id) {
    if (fb->d_head == fb->d_tail) return false;
    if (id) *id = fb->descs[fb->d_tail & fb->desc_mask].id;
    return true;
}

bool frame_buffer_newest_id(const frame_buffer_t *fb, uint16_t *id) {
    if (fb->d_head == fb->d_tail) return false;
    if (id) *id = fb->descs[(fb->d_head - 1) & fb->desc_mask].id;
    return true;
}
