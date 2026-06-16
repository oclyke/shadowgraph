// Host unit tests for the TV command codec (gtest).
#include "laser_command.h"
#include "byte_queue.h"

#include <gtest/gtest.h>

TEST(LaserCommand, Sizes) {
    EXPECT_EQ(laser_command_size(LASER_CMD_GOTO), 5u);
    EXPECT_EQ(laser_command_size(LASER_CMD_LASER), 7u);
    EXPECT_EQ(laser_command_size(LASER_CMD_DWELL), 5u);
    EXPECT_EQ(laser_command_size(LASER_CMD_CURVE), 21u);
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

TEST(LaserCommand, CurveRoundTrip) {
    byte_queue_t q;
    uint8_t buf[64];
    byte_queue_init(&q, buf, 64);
    EXPECT_TRUE(laser_command_push_curve(&q,
        0x1111, 0x2222,            // P1
        0x3333, 0x4444,            // P2
        0x5555, 0x6666,            // P3
        0x00ABCDEF, 0x00123456));  // v_in, v_out (wire speed; opaque to round-trip)

    laser_command_t c;
    ASSERT_TRUE(laser_command_pop(&q, &c));
    EXPECT_EQ(c.type, LASER_CMD_CURVE);
    EXPECT_EQ(c.curve.x1, 0x1111);
    EXPECT_EQ(c.curve.y1, 0x2222);
    EXPECT_EQ(c.curve.x2, 0x3333);
    EXPECT_EQ(c.curve.y2, 0x4444);
    EXPECT_EQ(c.curve.x3, 0x5555);
    EXPECT_EQ(c.curve.y3, 0x6666);
    EXPECT_EQ(c.curve.v_in,  0x00ABCDEFu);
    EXPECT_EQ(c.curve.v_out, 0x00123456u);

    EXPECT_FALSE(laser_command_pop(&q, &c));   // empty
}

TEST(LaserCommand, CurveQueueFull) {
    byte_queue_t q;
    uint8_t buf[32];                           // holds one 21-byte curve, not two
    byte_queue_init(&q, buf, 32);
    EXPECT_TRUE (laser_command_push_curve(&q, 1,2,3,4,5,6, 7,8));
    EXPECT_FALSE(laser_command_push_curve(&q, 1,2,3,4,5,6, 7,8));   // 11 free, needs 21
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
