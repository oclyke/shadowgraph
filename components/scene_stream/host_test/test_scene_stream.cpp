// Host unit tests for the scene_stream reassembly layer (gtest).
#include "scene_stream.h"
#include "laser_command.h"

#include <cstring>
#include <vector>
#include <gtest/gtest.h>

namespace {

// Build a GOTO whose x carries an identifying tag, so delivered commands are
// easy to assert on in order.
laser_command_t tag_cmd(uint16_t tag) {
    laser_command_t c;
    c.type  = LASER_CMD_GOTO;
    c.pos.x = tag;
    c.pos.y = 0;
    return c;
}

// Serialize a run of commands into a payload buffer (back-to-back records).
std::vector<uint8_t> encode_payload(const std::vector<laser_command_t> &cmds) {
    std::vector<uint8_t> out;
    uint8_t rec[16];
    for (const auto &c : cmds) {
        uint32_t n = laser_command_encode(rec, sizeof rec, &c);
        out.insert(out.end(), rec, rec + n);
    }
    return out;
}

// Build a full datagram for [base_seq, base_seq+cmds.size()).
std::vector<uint8_t> build_packet(uint32_t stream_id, uint32_t base_seq,
                                  const std::vector<laser_command_t> &cmds,
                                  uint8_t flags = 0) {
    std::vector<uint8_t> payload = encode_payload(cmds);
    scene_packet_hdr_t hdr{};
    hdr.flags     = flags;
    hdr.stream_id = stream_id;
    hdr.base_seq  = base_seq;
    hdr.count     = (uint16_t)cmds.size();
    std::vector<uint8_t> dst(SCENE_PACKET_HDR_SIZE + payload.size());
    uint32_t n = scene_packet_build(dst.data(), dst.size(), &hdr,
                                    payload.data(), (uint32_t)payload.size());
    dst.resize(n);
    return dst;
}

// Parse a datagram and ingest it.
uint32_t feed(scene_stream_t *s, const std::vector<uint8_t> &pkt) {
    scene_packet_hdr_t hdr{};
    const uint8_t *payload = nullptr;
    uint32_t plen = 0;
    EXPECT_TRUE(scene_packet_parse(pkt.data(), (uint32_t)pkt.size(),
                                   &hdr, &payload, &plen));
    return scene_stream_ingest(s, &hdr, payload, plen);
}

// Emit collector. `limit` caps how many it accepts (to exercise backpressure):
// once `limit` is hit, emit returns false (downstream full).
struct Collector {
    std::vector<uint16_t> tags;   // delivered GOTO x tags, in order
    int limit = -1;               // -1 = unlimited
    static bool emit(const laser_command_t *c, void *ctx) {
        auto *self = static_cast<Collector *>(ctx);
        if (self->limit == 0) return false;
        if (self->limit > 0) self->limit--;
        self->tags.push_back(c->pos.x);
        return true;
    }
};

struct Fixture {
    static constexpr uint32_t CAP = 16;
    scene_slot_t slots[CAP];
    scene_stream_t s;
    Fixture() { EXPECT_TRUE(scene_stream_init(&s, slots, CAP)); }
    uint32_t drain(Collector &c) { return scene_stream_drain(&s, &Collector::emit, &c); }
};

}  // namespace

// ---- framing --------------------------------------------------------------

TEST(SceneFraming, BuildParseRoundTrip) {
    auto pkt = build_packet(0xAABBCCDD, 1000, { tag_cmd(7), tag_cmd(8) },
                            SCENE_FLAG_KEYFRAME);
    scene_packet_hdr_t hdr{};
    const uint8_t *payload = nullptr;
    uint32_t plen = 0;
    ASSERT_TRUE(scene_packet_parse(pkt.data(), (uint32_t)pkt.size(),
                                   &hdr, &payload, &plen));
    EXPECT_EQ(hdr.magic, SCENE_PACKET_MAGIC);
    EXPECT_EQ(hdr.version, SCENE_PACKET_VERSION);
    EXPECT_EQ(hdr.flags, SCENE_FLAG_KEYFRAME);
    EXPECT_EQ(hdr.stream_id, 0xAABBCCDDu);
    EXPECT_EQ(hdr.base_seq, 1000u);
    EXPECT_EQ(hdr.count, 2u);
    EXPECT_EQ(plen, 10u);   // two 5-byte gotos
}

TEST(SceneFraming, RejectsBadMagicVersionShort) {
    auto pkt = build_packet(1, 0, { tag_cmd(1) });
    scene_packet_hdr_t hdr{};
    const uint8_t *payload = nullptr;
    uint32_t plen = 0;

    EXPECT_FALSE(scene_packet_parse(pkt.data(), 4, &hdr, &payload, &plen)); // short

    auto bad_magic = pkt; bad_magic[0] ^= 0xFF;
    EXPECT_FALSE(scene_packet_parse(bad_magic.data(), (uint32_t)bad_magic.size(),
                                    &hdr, &payload, &plen));

    auto bad_ver = pkt; bad_ver[2] = 0xFE;
    EXPECT_FALSE(scene_packet_parse(bad_ver.data(), (uint32_t)bad_ver.size(),
                                    &hdr, &payload, &plen));
}

TEST(SceneFraming, TruncatedPayloadClampsAndDecodesBestEffort) {
    Fixture f;
    auto pkt = build_packet(1, 0, { tag_cmd(1), tag_cmd(2), tag_cmd(3) });
    pkt.resize(pkt.size() - 3);   // chop the tail of the 3rd command

    scene_packet_hdr_t hdr{};
    const uint8_t *payload = nullptr;
    uint32_t plen = 0;
    ASSERT_TRUE(scene_packet_parse(pkt.data(), (uint32_t)pkt.size(),
                                   &hdr, &payload, &plen));
    EXPECT_EQ(hdr.count, 3u);          // header still claims 3
    uint32_t stored = scene_stream_ingest(&f.s, &hdr, payload, plen);
    EXPECT_EQ(stored, 2u);             // only 2 full records survived

    Collector c;
    f.drain(c);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 1, 2 }));
}

// ---- in-order / reordering -----------------------------------------------

TEST(SceneWindow, InOrderDeliversAll) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0), tag_cmd(1), tag_cmd(2) }));
    Collector c;
    EXPECT_EQ(f.drain(c), 3u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1, 2 }));
}

// The first packet of a stream locks deliver_seq to its base_seq, so a receiver
// can join mid-stream. Streams therefore open with a KEYFRAME at the true start.
TEST(SceneWindow, FirstPacketLocksDeliverSeqForMidStreamJoin) {
    Fixture f;
    feed(&f.s, build_packet(1, 50, { tag_cmd(50), tag_cmd(51) }));
    EXPECT_EQ(f.s.deliver_seq, 50u);
    Collector c;
    EXPECT_EQ(f.drain(c), 2u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 50, 51 }));
    // A straggler from before the join point is treated as already-past.
    EXPECT_EQ(feed(&f.s, build_packet(1, 48, { tag_cmd(48) })), 0u);
    EXPECT_EQ(f.s.dropped_dup, 1u);
}

TEST(SceneWindow, OutOfOrderReassembles) {
    Fixture f;
    // Establish the stream at seq 0, deliver 0,1 (deliver_seq -> 2).
    feed(&f.s, build_packet(1, 0, { tag_cmd(0), tag_cmd(1) }, SCENE_FLAG_KEYFRAME));
    Collector c;
    EXPECT_EQ(f.drain(c), 2u);
    // Now 4,5 arrive before 2,3; nothing flushes until 2,3 land.
    feed(&f.s, build_packet(1, 4, { tag_cmd(4), tag_cmd(5) }));
    EXPECT_EQ(f.drain(c), 0u);
    feed(&f.s, build_packet(1, 2, { tag_cmd(2), tag_cmd(3) }));
    EXPECT_EQ(f.drain(c), 4u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1, 2, 3, 4, 5 }));
}

// ---- dedup / blind resend -------------------------------------------------

TEST(SceneDedup, DuplicatePacketIsIdempotent) {
    Fixture f;
    auto pkt = build_packet(1, 0, { tag_cmd(0), tag_cmd(1) });
    EXPECT_EQ(feed(&f.s, pkt), 2u);
    EXPECT_EQ(feed(&f.s, pkt), 0u);      // blind resend: nothing new
    EXPECT_EQ(f.s.dropped_dup, 2u);

    Collector c;
    f.drain(c);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1 }));  // not doubled
}

TEST(SceneDedup, AlreadyDeliveredResendDropped) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0), tag_cmd(1) }));
    Collector c;
    f.drain(c);                          // deliver_seq now 2

    // A late resend of already-played commands is ignored.
    EXPECT_EQ(feed(&f.s, build_packet(1, 0, { tag_cmd(0), tag_cmd(1) })), 0u);
    EXPECT_EQ(f.s.dropped_dup, 2u);
    EXPECT_EQ(f.drain(c), 0u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1 }));
}

TEST(SceneWindow, BeyondWindowDropped) {
    Fixture f;   // CAP = 16
    // Lock deliver_seq = 0 but don't drain, so the window is [0, 16).
    feed(&f.s, build_packet(1, 0, { tag_cmd(0) }, SCENE_FLAG_KEYFRAME));
    // seq 16 == deliver_seq + CAP is just past the window.
    feed(&f.s, build_packet(1, 16, { tag_cmd(99) }));
    EXPECT_EQ(f.s.dropped_ahead, 1u);
    // seq 15 is the last slot still inside the window — storable.
    EXPECT_EQ(feed(&f.s, build_packet(1, 15, { tag_cmd(15) })), 1u);
    EXPECT_EQ(f.s.dropped_ahead, 1u);
}

// ---- backpressure ---------------------------------------------------------

TEST(SceneBackpressure, StopsWhenDownstreamFullAndResumes) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0), tag_cmd(1), tag_cmd(2), tag_cmd(3) }));

    Collector c;
    c.limit = 2;                         // downstream accepts only 2
    EXPECT_EQ(f.drain(c), 2u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1 }));
    EXPECT_EQ(f.s.deliver_seq, 2u);      // 2,3 still buffered

    c.limit = -1;                        // downstream drains
    EXPECT_EQ(f.drain(c), 2u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 1, 2, 3 }));
}

// ---- gap skip -------------------------------------------------------------

TEST(SceneGap, SkipAdvancesPastLostCommand) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0) }));
    // seq 1 lost; 2,3 arrive.
    feed(&f.s, build_packet(1, 2, { tag_cmd(2), tag_cmd(3) }));

    Collector c;
    EXPECT_EQ(f.drain(c), 1u);                       // delivers 0, stalls at 1
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0 }));
    EXPECT_TRUE(scene_stream_stalled_at_gap(&f.s));

    EXPECT_EQ(scene_stream_skip_gap(&f.s), 1u);       // skip the single lost cmd
    EXPECT_EQ(f.s.gaps_skipped, 1u);
    EXPECT_FALSE(scene_stream_stalled_at_gap(&f.s));

    EXPECT_EQ(f.drain(c), 2u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 2, 3 }));
}

TEST(SceneGap, NoSkipWhenNothingBuffered) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0) }));
    Collector c;
    f.drain(c);                                       // deliver_seq = 1, empty ahead
    EXPECT_FALSE(scene_stream_stalled_at_gap(&f.s));  // underrun, not a skippable gap
    EXPECT_EQ(scene_stream_skip_gap(&f.s), 0u);
}

// ---- stream reset ---------------------------------------------------------

TEST(SceneReset, KeyframeResetsWindowAndCursor) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0) }, SCENE_FLAG_KEYFRAME));
    Collector c;
    EXPECT_EQ(f.drain(c), 1u);                                  // deliver 0 -> seq 1
    feed(&f.s, build_packet(1, 5, { tag_cmd(5), tag_cmd(6) })); // buffered behind a gap
    EXPECT_EQ(f.drain(c), 0u);                                  // stuck at seq 1

    // A keyframe restarts the scene at a new base_seq; stale 5,6 drop.
    feed(&f.s, build_packet(1, 100, { tag_cmd(100) }, SCENE_FLAG_KEYFRAME));
    EXPECT_EQ(f.s.deliver_seq, 100u);
    EXPECT_EQ(f.drain(c), 1u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 100 }));
}

TEST(SceneReset, StreamIdChangeResets) {
    Fixture f;
    feed(&f.s, build_packet(1, 0, { tag_cmd(0) }));
    Collector c;
    f.drain(c);

    feed(&f.s, build_packet(2, 0, { tag_cmd(42) }));   // new session, seq restarts
    EXPECT_EQ(f.s.stream_id, 2u);
    EXPECT_EQ(f.drain(c), 1u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0, 42 }));
}

// ---- sequence wrap --------------------------------------------------------

TEST(SceneWrap, DeliversAcrossU32Wrap) {
    Fixture f;
    uint32_t base = 0xFFFFFFFEu;          // 0xFFFFFFFE, 0xFFFFFFFF, 0x00000000
    feed(&f.s, build_packet(1, base, { tag_cmd(0xAA), tag_cmd(0xBB), tag_cmd(0xCC) }));
    Collector c;
    EXPECT_EQ(f.drain(c), 3u);
    EXPECT_EQ(c.tags, (std::vector<uint16_t>{ 0xAA, 0xBB, 0xCC }));
    EXPECT_EQ(f.s.deliver_seq, 1u);       // wrapped past 0
}
