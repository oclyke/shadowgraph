#include "scene_stream.h"

// ---- little-endian buffer helpers -----------------------------------------
static inline void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---- wire framing ---------------------------------------------------------
uint32_t scene_packet_build(uint8_t *dst, uint32_t cap,
                            const scene_packet_hdr_t *hdr,
                            const uint8_t *payload, uint32_t payload_len) {
    uint32_t total = SCENE_PACKET_HDR_SIZE + payload_len;
    if (dst == NULL || cap < total) {
        return 0;
    }
    put_u16(dst + 0,  SCENE_PACKET_MAGIC);
    dst[2] = SCENE_PACKET_VERSION;
    dst[3] = hdr->flags;
    put_u32(dst + 4,  hdr->stream_id);
    put_u32(dst + 8,  hdr->base_seq);
    put_u16(dst + 12, hdr->count);
    put_u16(dst + 14, (uint16_t)payload_len);
    for (uint32_t i = 0; i < payload_len; i++) {
        dst[SCENE_PACKET_HDR_SIZE + i] = payload[i];
    }
    return total;
}

bool scene_packet_parse(const uint8_t *buf, uint32_t len,
                        scene_packet_hdr_t *hdr,
                        const uint8_t **payload, uint32_t *payload_len) {
    if (buf == NULL || len < SCENE_PACKET_HDR_SIZE) {
        return false;
    }
    if (get_u16(buf + 0) != SCENE_PACKET_MAGIC || buf[2] != SCENE_PACKET_VERSION) {
        return false;
    }
    hdr->magic       = SCENE_PACKET_MAGIC;
    hdr->version     = SCENE_PACKET_VERSION;
    hdr->flags       = buf[3];
    hdr->stream_id   = get_u32(buf + 4);
    hdr->base_seq    = get_u32(buf + 8);
    hdr->count       = get_u16(buf + 12);
    hdr->payload_len = get_u16(buf + 14);

    // Clamp to what's actually present: a truncated datagram yields a short
    // payload that ingest decodes best-effort (it stops at the partial record).
    uint32_t avail = len - SCENE_PACKET_HDR_SIZE;
    uint32_t plen  = hdr->payload_len < avail ? hdr->payload_len : avail;
    if (payload)     *payload = buf + SCENE_PACKET_HDR_SIZE;
    if (payload_len) *payload_len = plen;
    return true;
}

// ---- reassembly window ----------------------------------------------------
static bool is_pow2(uint32_t v) {
    return v != 0 && (v & (v - 1)) == 0;
}

bool scene_stream_init(scene_stream_t *s, scene_slot_t *slots, uint32_t cap) {
    if (s == NULL || slots == NULL || !is_pow2(cap)) {
        return false;
    }
    s->slots         = slots;
    s->cap           = cap;
    s->mask          = cap - 1;
    s->deliver_seq   = 0;
    s->stream_id     = 0;
    s->started       = false;
    s->stored        = 0;
    s->dropped_dup   = 0;
    s->dropped_ahead = 0;
    s->gaps_skipped  = 0;
    for (uint32_t i = 0; i < cap; i++) {
        slots[i].present = false;
    }
    return true;
}

void scene_stream_reset(scene_stream_t *s) {
    s->started = false;
    for (uint32_t i = 0; i < s->cap; i++) {
        s->slots[i].present = false;
    }
}

uint32_t scene_stream_ingest(scene_stream_t *s,
                             const scene_packet_hdr_t *hdr,
                             const uint8_t *payload, uint32_t payload_len) {
    // (Re)lock the receiver on first packet, a stream-id change, or a keyframe.
    if (!s->started || hdr->stream_id != s->stream_id ||
        (hdr->flags & SCENE_FLAG_KEYFRAME)) {
        scene_stream_reset(s);
        s->started     = true;
        s->stream_id   = hdr->stream_id;
        s->deliver_seq = hdr->base_seq;
    }

    uint32_t stored = 0;
    uint32_t off    = 0;
    for (uint32_t j = 0; j < hdr->count; j++) {
        laser_command_t cmd;
        uint32_t consumed = 0;
        if (!laser_command_decode(payload + off, payload_len - off, &cmd, &consumed)) {
            break;   // truncated or corrupt payload — take what decoded cleanly
        }
        off += consumed;

        uint32_t seq   = hdr->base_seq + j;
        int32_t  delta = (int32_t)(seq - s->deliver_seq);   // modular distance
        if (delta < 0) {
            s->dropped_dup++;            // already delivered (late/duplicate)
        } else if ((uint32_t)delta >= s->cap) {
            s->dropped_ahead++;          // beyond the window; sender must resend
        } else {
            scene_slot_t *slot = &s->slots[seq & s->mask];
            if (slot->present) {
                s->dropped_dup++;        // duplicate within the window
            } else {
                slot->cmd     = cmd;
                slot->present = true;
                s->stored++;
                stored++;
            }
        }
    }
    return stored;
}

uint32_t scene_stream_drain(scene_stream_t *s, scene_emit_fn emit, void *ctx) {
    uint32_t delivered = 0;
    while (s->started) {
        scene_slot_t *slot = &s->slots[s->deliver_seq & s->mask];
        if (!slot->present) {
            break;                       // caught up, or stalled at a gap
        }
        if (!emit(&slot->cmd, ctx)) {
            break;                       // downstream full; keep slot, retry later
        }
        slot->present = false;
        s->deliver_seq++;
        delivered++;
    }
    return delivered;
}

bool scene_stream_stalled_at_gap(const scene_stream_t *s) {
    if (!s->started || s->slots[s->deliver_seq & s->mask].present) {
        return false;                    // not started, or next command is ready
    }
    // Stalled only counts if something is actually buffered ahead to skip to.
    for (uint32_t i = 1; i < s->cap; i++) {
        if (s->slots[(s->deliver_seq + i) & s->mask].present) {
            return true;
        }
    }
    return false;
}

uint32_t scene_stream_skip_gap(scene_stream_t *s) {
    if (!s->started || s->slots[s->deliver_seq & s->mask].present) {
        return 0;                        // not stalled at a gap
    }
    for (uint32_t i = 1; i < s->cap; i++) {
        if (s->slots[(s->deliver_seq + i) & s->mask].present) {
            s->deliver_seq += i;         // drop the missing commands, resync
            s->gaps_skipped++;
            return i;
        }
    }
    return 0;                            // nothing ahead: genuine underrun
}
