// Host unit tests for the TV command codec (gtest).
#include "laser_command.h"
#include "byte_queue.h"

#include <cstring>
#include <gtest/gtest.h>

TEST(LaserCommand, Sizes) {
    EXPECT_EQ(laser_command_size(LASER_CMD_GOTO), 5u);
    EXPECT_EQ(laser_command_size(LASER_CMD_LASER), 7u);
    EXPECT_EQ(laser_command_size(LASER_CMD_DWELL), 5u);
    EXPECT_EQ(laser_command_size(static_cast<laser_command_type_t>(0xFF)), 0u);
}

TEST(LaserCommand, RoundTrip) {
    byte_queue_t q;
    uint8_t buf[64];
    byte_queue_init(&q, buf, 64);
    EXPECT_TRUE(laser_command_push_goto(&q, 0x1234, 0xABCD));
    EXPECT_TRUE(laser_command_push_laser(&q, 0x1111, 0x2222, 0x3333));
    EXPECT_TRUE(laser_command_push_dwell(&q, 0xDEADBEEF));

    laser_command_t c;
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_GOTO);
    EXPECT_EQ(c.pos.x, 0x1234);
    EXPECT_EQ(c.pos.y, 0xABCD);

    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_LASER);
    EXPECT_EQ(c.col.r, 0x1111);
    EXPECT_EQ(c.col.g, 0x2222);
    EXPECT_EQ(c.col.b, 0x3333);

    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_DWELL);
    EXPECT_EQ(c.dwell.dt, 0xDEADBEEFu);

    EXPECT_FALSE(laser_command_pop(&q, &c));   // empty
}

TEST(LaserCommand, QueueFull) {
    byte_queue_t q;
    uint8_t buf[8];                    // holds one 5-byte goto, not two
    byte_queue_init(&q, buf, 8);
    EXPECT_TRUE(laser_command_push_goto(&q, 1, 2));
    EXPECT_FALSE(laser_command_push_goto(&q, 3, 4));   // 3 free, goto needs 5
    laser_command_t c;
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.pos.x, 1);
    EXPECT_EQ(c.pos.y, 2);
}

TEST(LaserCommand, RecordWrapsBuffer) {
    byte_queue_t q;
    uint8_t buf[8];
    byte_queue_init(&q, buf, 8);
    laser_command_t c;
    EXPECT_TRUE(laser_command_push_goto(&q, 0xAAAA, 0xBBBB));   // offset 0 -> 5
    ASSERT_TRUE(laser_command_pop(&q, &c));                     // read offset -> 5
    EXPECT_TRUE(laser_command_push_goto(&q, 0x1234, 0x5678));   // 5 bytes from 5 -> wraps
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.pos.x, 0x1234);
    EXPECT_EQ(c.pos.y, 0x5678);
}

TEST(LaserCommand, UnknownTypeGuard) {
    byte_queue_t q;
    uint8_t buf[8];
    byte_queue_init(&q, buf, 8);
    uint8_t bogus = 0x7F;
    byte_queue_write(&q, &bogus, 1);
    laser_command_t c;
    EXPECT_FALSE(laser_command_pop(&q, &c));
}

// --- buffer codec (encode/decode) ------------------------------------------

TEST(LaserCommandCodec, EncodeDecodeRoundTrip) {
    const laser_command_t cmds[] = {
        { .type = LASER_CMD_GOTO,  .pos   = { 0x1234, 0xABCD } },
        { .type = LASER_CMD_LASER, .col   = { 0x1111, 0x2222, 0x3333 } },
        { .type = LASER_CMD_DWELL, .dwell = { 0xDEADBEEF } },
    };
    for (const auto &in : cmds) {
        uint8_t buf[16];
        uint32_t n = laser_command_encode(buf, sizeof buf, &in);
        ASSERT_EQ(n, laser_command_size(in.type));

        laser_command_t out;
        uint32_t consumed = 0;
        ASSERT_TRUE(laser_command_decode(buf, n, &out, &consumed));
        EXPECT_EQ(consumed, n);
        EXPECT_EQ(out.type, in.type);
        // Compare via re-encode so the union's active member is irrelevant.
        uint8_t buf2[16];
        ASSERT_EQ(laser_command_encode(buf2, sizeof buf2, &out), n);
        EXPECT_EQ(0, memcmp(buf, buf2, n));
    }
}

// The buffer codec must produce byte-for-byte the same wire format as the
// queue push/pop path — they are the two ends of one protocol.
TEST(LaserCommandCodec, MatchesQueueWireFormat) {
    byte_queue_t q;
    uint8_t qbuf[64];
    byte_queue_init(&q, qbuf, 64);
    ASSERT_TRUE(laser_command_push_laser(&q, 0xCAFE, 0xBABE, 0x0FF0));

    uint8_t queued[7];
    ASSERT_EQ(byte_queue_read(&q, queued, sizeof queued), sizeof queued);

    laser_command_t c = { .type = LASER_CMD_LASER, .col = { 0xCAFE, 0xBABE, 0x0FF0 } };
    uint8_t encoded[7];
    ASSERT_EQ(laser_command_encode(encoded, sizeof encoded, &c), sizeof encoded);
    EXPECT_EQ(0, memcmp(queued, encoded, sizeof encoded));
}

TEST(LaserCommandCodec, EncodeRejectsSmallBufferAndBadType) {
    laser_command_t c = { .type = LASER_CMD_LASER, .col = { 1, 2, 3 } };
    uint8_t buf[7];
    EXPECT_EQ(laser_command_encode(buf, 6, &c), 0u);          // needs 7
    EXPECT_EQ(laser_command_encode(buf, sizeof buf, &c), 7u); // exact fit ok

    laser_command_t bad = { .type = static_cast<laser_command_type_t>(0x55) };
    EXPECT_EQ(laser_command_encode(buf, sizeof buf, &bad), 0u);
}

TEST(LaserCommandCodec, DecodeRejectsPartialAndBadType) {
    laser_command_t out;
    uint32_t consumed = 123;

    uint8_t empty[1];
    EXPECT_FALSE(laser_command_decode(empty, 0, &out, &consumed));

    // Valid type byte but a record truncated short of its declared size.
    uint8_t partial[4] = { LASER_CMD_GOTO, 0x34, 0x12, 0x00 };  // goto needs 5
    EXPECT_FALSE(laser_command_decode(partial, sizeof partial, &out, &consumed));
    EXPECT_EQ(consumed, 123u);   // left untouched on failure

    uint8_t bogus[8] = { 0x7F };
    EXPECT_FALSE(laser_command_decode(bogus, sizeof bogus, &out, &consumed));
}

// Decode walks a packed buffer of back-to-back records via *consumed, the way
// the network reassembly path will unpack a packet payload.
TEST(LaserCommandCodec, DecodeWalksPackedStream) {
    byte_queue_t q;
    uint8_t qbuf[64];
    byte_queue_init(&q, qbuf, 64);
    ASSERT_TRUE(laser_command_push_goto(&q, 10, 20));
    ASSERT_TRUE(laser_command_push_laser(&q, 0xFFFF, 0, 0));
    ASSERT_TRUE(laser_command_push_dwell(&q, 50));

    uint8_t stream[17];                       // 5 + 7 + 5
    uint32_t total = byte_queue_avail(&q);
    ASSERT_EQ(total, sizeof stream);
    ASSERT_EQ(byte_queue_read(&q, stream, sizeof stream), sizeof stream);

    uint32_t off = 0;
    int count = 0;
    laser_command_t c;
    uint32_t consumed = 0;
    while (off < total && laser_command_decode(stream + off, total - off, &c, &consumed)) {
        off += consumed;
        count++;
    }
    EXPECT_EQ(count, 3);
    EXPECT_EQ(off, total);   // consumed the whole packed buffer exactly
}

TEST(LaserCommand, MixedFifoOrder) {
    byte_queue_t q;
    uint8_t buf[64];
    byte_queue_init(&q, buf, 64);
    EXPECT_TRUE(laser_command_push_laser(&q, 0xFFFF, 0, 0));    // red on
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(laser_command_push_goto(&q, (uint16_t)i, (uint16_t)(i * 2)));
    }
    EXPECT_TRUE(laser_command_push_dwell(&q, 100));
    EXPECT_TRUE(laser_command_push_laser(&q, 0, 0, 0));         // off

    laser_command_t c;
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_LASER);
    EXPECT_EQ(c.col.r, 0xFFFF);
    for (int i = 0; i < 5; i++) {
        ASSERT_TRUE(laser_command_pop(&q, &c));
        EXPECT_EQ(c.type, LASER_CMD_GOTO);
        EXPECT_EQ(c.pos.x, (uint16_t)i);
        EXPECT_EQ(c.pos.y, (uint16_t)(i * 2));
    }
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_DWELL);
    EXPECT_EQ(c.dwell.dt, 100u);
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_LASER);
    EXPECT_EQ(c.col.r, 0);
    EXPECT_EQ(c.col.g, 0);
    EXPECT_EQ(c.col.b, 0);
    EXPECT_FALSE(laser_command_pop(&q, &c));
}
