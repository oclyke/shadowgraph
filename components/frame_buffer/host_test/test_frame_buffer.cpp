// Host unit tests for frame_buffer: the variable-length frame FIFO arena.
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

extern "C" {
#include "frame_buffer.h"
}

namespace {

// A test fixture wrapping an arena + descriptor ring of caller-chosen sizes.
struct Fb {
    std::vector<uint8_t>      arena;
    std::vector<frame_desc_t> descs;
    frame_buffer_t            fb{};

    Fb(uint32_t cap, uint32_t desc_cap) : arena(cap), descs(desc_cap) {
        EXPECT_TRUE(frame_buffer_init(&fb, arena.data(), cap, descs.data(), desc_cap));
    }

    // Push a frame of `len` bytes filled with `fill`, addressed by id.
    // Returns false if the buffer had no room (backpressure).
    bool push(uint16_t id, uint32_t len, uint8_t fill) {
        uint8_t *p = frame_buffer_reserve(&fb, len);
        if (!p) return false;
        memset(p, fill, len);
        EXPECT_TRUE(frame_buffer_commit(&fb, id));
        return true;
    }

    // Assert frame `id` is resident, `len` bytes, every byte == fill.
    void expect_frame(uint16_t id, uint32_t len, uint8_t fill) {
        const uint8_t *p = nullptr;
        uint32_t got = 0;
        ASSERT_TRUE(frame_buffer_get(&fb, id, &p, &got)) << "id " << id << " missing";
        EXPECT_EQ(got, len);
        for (uint32_t i = 0; i < got; i++)
            ASSERT_EQ(p[i], fill) << "id " << id << " byte " << i << " corrupted";
    }
};

TEST(FrameBuffer, InitRejectsBadArgs) {
    frame_buffer_t fb;
    std::vector<uint8_t> arena(1024);
    std::vector<frame_desc_t> descs(8);
    EXPECT_FALSE(frame_buffer_init(&fb, nullptr, 1024, descs.data(), 8));
    EXPECT_FALSE(frame_buffer_init(&fb, arena.data(), 0, descs.data(), 8));
    EXPECT_FALSE(frame_buffer_init(&fb, arena.data(), 1024, descs.data(), 7)); // not pow2
    EXPECT_TRUE(frame_buffer_init(&fb, arena.data(), 1024, descs.data(), 8));
}

TEST(FrameBuffer, PushGetRoundTrip) {
    Fb t(1024, 8);
    ASSERT_TRUE(t.push(1, 100, 0xAA));
    ASSERT_TRUE(t.push(2, 200, 0xBB));
    EXPECT_EQ(frame_buffer_count(&t.fb), 2u);
    t.expect_frame(1, 100, 0xAA);
    t.expect_frame(2, 200, 0xBB);

    const uint8_t *p; uint32_t l;
    EXPECT_FALSE(frame_buffer_get(&t.fb, 99, &p, &l));   // absent id
}

TEST(FrameBuffer, FramesArePlacedContiguouslyAndDoNotOverlap) {
    Fb t(1024, 8);
    ASSERT_TRUE(t.push(1, 100, 0x11));
    ASSERT_TRUE(t.push(2, 100, 0x22));
    const uint8_t *p1, *p2; uint32_t l1, l2;
    ASSERT_TRUE(frame_buffer_get(&t.fb, 1, &p1, &l1));
    ASSERT_TRUE(frame_buffer_get(&t.fb, 2, &p2, &l2));
    EXPECT_EQ(p2, p1 + l1);   // back-to-back, no gap
}

TEST(FrameBuffer, EvictsConsumedFramesToMakeRoom) {
    // cap fits ~2 frames of 400. Consume the first two (advance past them), then a
    // 3rd push reclaims the consumed (behind-the-cursor) oldest frame.
    Fb t(1000, 8);
    ASSERT_TRUE(t.push(1, 400, 0x11));
    ASSERT_TRUE(t.push(2, 400, 0x22));
    const uint8_t *p; uint32_t l;
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 1
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 2 (id 1 consumed)
    ASSERT_TRUE(t.push(3, 400, 0x33));                 // evicts consumed id 1

    EXPECT_FALSE(frame_buffer_get(&t.fb, 1, &p, &l));  // evicted
    t.expect_frame(2, 400, 0x22);
    t.expect_frame(3, 400, 0x33);
    EXPECT_EQ(frame_buffer_count(&t.fb), 2u);
}

TEST(FrameBuffer, UnplayedFramesAreRetainedNotDropped) {
    // Strict FIFO: frames not yet reached by the play cursor must never be
    // dropped. With nothing consumed, a 3rd frame that doesn't fit backpressures
    // (reserve returns NULL) rather than evicting a queued-but-unplayed frame.
    Fb t(1000, 8);
    ASSERT_TRUE(t.push(1, 400, 0x11));
    ASSERT_TRUE(t.push(2, 400, 0x22));
    EXPECT_EQ(frame_buffer_reserve(&t.fb, 400), nullptr);
    t.expect_frame(1, 400, 0x11);
    t.expect_frame(2, 400, 0x22);
}

TEST(FrameBuffer, DisplayedFrameIsNeverEvictedAndCausesBackpressure) {
    Fb t(1000, 8);
    ASSERT_TRUE(t.push(1, 400, 0x11));
    ASSERT_TRUE(t.push(2, 400, 0x22));
    const uint8_t *p; uint32_t l;
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));   // display id 1 (the floor)
    EXPECT_EQ(p[0], 0x11);

    // A 3rd frame can't fit without evicting id 1, which is displayed -> NULL.
    EXPECT_EQ(frame_buffer_reserve(&t.fb, 400), nullptr);
    t.expect_frame(1, 400, 0x11);   // still intact
    t.expect_frame(2, 400, 0x22);

    // Advance to id 2; now id 1 is reclaimable and the push succeeds.
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));   // display id 2
    EXPECT_EQ(p[0], 0x22);
    ASSERT_TRUE(t.push(3, 400, 0x33));
    EXPECT_FALSE(frame_buffer_get(&t.fb, 1, &p, &l));
    t.expect_frame(2, 400, 0x22);
    t.expect_frame(3, 400, 0x33);
}

TEST(FrameBuffer, ForbiddenWrapKeepsFrameContiguousAndIntact) {
    // cap=100. A=40@0, B=40@40. C=40 won't fit in tail [80,100): pad 20 charged
    // to B, C placed at 0 (after A is evicted). Verify C is one contiguous slice
    // entirely within the arena and B's payload survives the pad.
    Fb t(100, 8);
    ASSERT_TRUE(t.push(1, 40, 0x11));
    ASSERT_TRUE(t.push(2, 40, 0x22));
    const uint8_t *p; uint32_t l;
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 1
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 2 (id 1 consumed)
    ASSERT_TRUE(t.push(3, 40, 0x33));   // wraps: evicts consumed 1, pads tail, C @ 0

    const uint8_t *pc; uint32_t lc;
    ASSERT_TRUE(frame_buffer_get(&t.fb, 3, &pc, &lc));
    EXPECT_EQ(lc, 40u);
    // Contiguous and in-bounds (no straddle).
    EXPECT_GE(pc, t.fb.arena);
    EXPECT_LE(pc + lc, t.fb.arena + t.fb.cap);
    EXPECT_EQ(pc, t.fb.arena);          // restarted at the base

    t.expect_frame(2, 40, 0x22);        // survived the pad
    t.expect_frame(3, 40, 0x33);
    EXPECT_FALSE(frame_buffer_get(&t.fb, 1, &p, &l));  // evicted
}

TEST(FrameBuffer, ReserveRejectsOversizeAndZero) {
    Fb t(256, 8);
    EXPECT_EQ(frame_buffer_reserve(&t.fb, 0), nullptr);
    EXPECT_EQ(frame_buffer_reserve(&t.fb, 257), nullptr);   // > cap
    EXPECT_NE(frame_buffer_reserve(&t.fb, 256), nullptr);   // exactly cap is OK
}

TEST(FrameBuffer, AbortDropsReservation) {
    Fb t(256, 8);
    EXPECT_NE(frame_buffer_reserve(&t.fb, 100), nullptr);
    frame_buffer_abort(&t.fb);
    EXPECT_FALSE(frame_buffer_commit(&t.fb, 1));   // nothing pending
    EXPECT_EQ(frame_buffer_count(&t.fb), 0u);
}

TEST(FrameBuffer, OldestNewestIds) {
    Fb t(1024, 8);
    uint16_t id;
    EXPECT_FALSE(frame_buffer_oldest_id(&t.fb, &id));   // empty
    t.push(10, 50, 0x1);
    t.push(20, 50, 0x2);
    t.push(30, 50, 0x3);
    ASSERT_TRUE(frame_buffer_oldest_id(&t.fb, &id)); EXPECT_EQ(id, 10);
    ASSERT_TRUE(frame_buffer_newest_id(&t.fb, &id)); EXPECT_EQ(id, 30);
}

TEST(FrameBuffer, DescriptorRingFullEvictsConsumedToMakeSlot) {
    // Tiny descriptor ring (2 slots) but a big arena: the slot limit, not bytes,
    // forces eviction. With both frames consumed, the 3rd push reclaims a slot
    // from the consumed oldest frame.
    Fb t(8192, 2);
    ASSERT_TRUE(t.push(1, 10, 0x11));
    ASSERT_TRUE(t.push(2, 10, 0x22));
    const uint8_t *p; uint32_t l;
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 1
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l));  // display id 2 (id 1 consumed)
    ASSERT_TRUE(t.push(3, 10, 0x33));   // only 2 slots -> evicts consumed id 1
    EXPECT_EQ(frame_buffer_count(&t.fb), 2u);
    EXPECT_FALSE(frame_buffer_get(&t.fb, 1, &p, &l));
    t.expect_frame(2, 10, 0x22);
    t.expect_frame(3, 10, 0x33);
}

TEST(FrameBuffer, AdvanceConsumesForwardThenEmptyNoWrap) {
    Fb t(4096, 8);
    ASSERT_TRUE(t.push(10, 30, 0xA0));
    ASSERT_TRUE(t.push(20, 30, 0xB0));

    const uint8_t *p; uint32_t l;
    // Nothing is displayed until the first advance.
    EXPECT_FALSE(frame_buffer_current(&t.fb, &p, &l));
    // advance walks forward in commit order; current re-reads without advancing.
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l)); EXPECT_EQ(p[0], 0xA0);
    ASSERT_TRUE(frame_buffer_current(&t.fb, &p, &l)); EXPECT_EQ(p[0], 0xA0);
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l)); EXPECT_EQ(p[0], 0xB0);
    // Advancing past the last received frame empties the queue — NO wrap.
    EXPECT_FALSE(frame_buffer_advance(&t.fb, &p, &l));
    EXPECT_FALSE(frame_buffer_current(&t.fb, &p, &l));
    EXPECT_FALSE(frame_buffer_advance(&t.fb, &p, &l));   // stays empty
    // A newly received frame can then be advanced into.
    ASSERT_TRUE(t.push(30, 30, 0xC0));
    ASSERT_TRUE(frame_buffer_advance(&t.fb, &p, &l)); EXPECT_EQ(p[0], 0xC0);
}

TEST(FrameBuffer, AdvanceAndCurrentOnEmptyReturnFalse) {
    Fb t(1024, 8);
    const uint8_t *p; uint32_t l;
    EXPECT_FALSE(frame_buffer_current(&t.fb, &p, &l));
    EXPECT_FALSE(frame_buffer_advance(&t.fb, &p, &l));
}

// Stress: many varied-size pushes, advancing onto each as it is committed so the
// displayed frame is the reclaim floor. The displayed frame must read back intact
// through repeated arena wraps.
TEST(FrameBuffer, WrapStressKeepsDisplayedFrameIntact) {
    Fb t(4096, 64);
    uint16_t id = 0;
    for (int i = 0; i < 5000; i++) {
        uint32_t len = 17 + (i * 37) % 900;     // 17..916, never near cap
        uint8_t  fill = (uint8_t)(id & 0xFF);
        uint8_t *p = frame_buffer_reserve(&t.fb, len);
        ASSERT_NE(p, nullptr) << "iter " << i;
        memset(p, fill, len);
        ASSERT_TRUE(frame_buffer_commit(&t.fb, id));
        // Advance onto the just-committed frame (it becomes the floor).
        const uint8_t *rp; uint32_t rl;
        ASSERT_TRUE(frame_buffer_advance(&t.fb, &rp, &rl)) << "iter " << i;
        ASSERT_EQ(rl, len);
        ASSERT_GE(rp, t.fb.arena);
        ASSERT_LE(rp + rl, t.fb.arena + t.fb.cap);   // never straddles
        for (uint32_t b = 0; b < rl; b++) ASSERT_EQ(rp[b], fill);
        id++;
    }
}

}  // namespace
