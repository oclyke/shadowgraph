// Host tests for the point_stream triple-buffer core (the device-only TCP task
// is compiled out without ESP_PLATFORM). Verifies publish/get latest-wins
// semantics, length clamping, and the tear-free slot invariant.
#include "gtest/gtest.h"

extern "C" {
#include "point_stream.h"
}

static laser_point_t pt(int16_t x) {
    return laser_point_t{x, (int16_t)(x + 1), 0, (uint8_t)x, 0, 0};
}

TEST(PointStream, EmptyBeforeFirstPublish) {
    point_stream_init();
    const laser_point_t *p = nullptr;
    uint32_t n = 123;
    EXPECT_FALSE(point_stream_get(&p, &n));
}

TEST(PointStream, PublishThenGet) {
    point_stream_init();
    laser_point_t scene[3] = {pt(10), pt(20), pt(30)};
    point_stream_publish(scene, 3);

    const laser_point_t *p = nullptr;
    uint32_t n = 0;
    ASSERT_TRUE(point_stream_get(&p, &n));
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(p[0].x, 10);
    EXPECT_EQ(p[2].x, 30);

    // No new scene: get returns the same buffer/length.
    const laser_point_t *p2 = nullptr;
    uint32_t n2 = 0;
    ASSERT_TRUE(point_stream_get(&p2, &n2));
    EXPECT_EQ(p2, p);
    EXPECT_EQ(n2, 3u);
}

TEST(PointStream, LatestWins) {
    point_stream_init();
    laser_point_t a[2] = {pt(1), pt(2)};
    laser_point_t b[4] = {pt(5), pt(6), pt(7), pt(8)};
    point_stream_publish(a, 2);
    point_stream_publish(b, 4); // two publishes before any get → newest wins

    const laser_point_t *p = nullptr;
    uint32_t n = 0;
    ASSERT_TRUE(point_stream_get(&p, &n));
    EXPECT_EQ(n, 4u);
    EXPECT_EQ(p[0].x, 5);
}

TEST(PointStream, CommitClampsToMax) {
    point_stream_init();
    point_stream_commit(POINT_STREAM_MAX_PTS + 100);
    const laser_point_t *p = nullptr;
    uint32_t n = 0;
    ASSERT_TRUE(point_stream_get(&p, &n));
    EXPECT_EQ(n, POINT_STREAM_MAX_PTS);
}

// The active (consumer) buffer must never be the slot a subsequent producer
// publish writes into — that is the tear-free guarantee. After taking scene A,
// the back buffer (where the next publish lands) must differ from A's pointer.
TEST(PointStream, ConsumerAndProducerSlotsDiffer) {
    point_stream_init();
    laser_point_t a[1] = {pt(1)};
    point_stream_publish(a, 1);

    const laser_point_t *held = nullptr;
    uint32_t n = 0;
    ASSERT_TRUE(point_stream_get(&held, &n));

    // The producer's next back slot is distinct from the slot we're holding.
    EXPECT_NE(point_stream_back(), held);

    // Filling and publishing the back slot must not disturb the held buffer.
    laser_point_t b[1] = {pt(99)};
    laser_point_t *back = point_stream_back();
    back[0] = b[0];
    point_stream_commit(1);
    EXPECT_EQ(held[0].x, 1); // unchanged while we still hold it
}

// ILDA record decode: big-endian X/Y, status (blank/last bits pass through),
// and B,G,R order mapped to our r,g,b.
TEST(PointStream, IldRecsize) {
    EXPECT_EQ(point_stream_ild_recsize(5), 8u);
    EXPECT_EQ(point_stream_ild_recsize(4), 10u);
    EXPECT_EQ(point_stream_ild_recsize(1), 0u); // indexed: unsupported
}

TEST(PointStream, IldRecordFormat5) {
    // X=0x0102, Y=-2, status=last|blank, B,G,R = 0xBB,0xC4,0xD5
    uint8_t rec[8] = {0x01, 0x02, 0xFF, 0xFE, POINT_LAST | POINT_BLANK, 0xBB, 0xC4, 0xD5};
    laser_point_t out{};
    ASSERT_TRUE(point_stream_ild_record(5, rec, &out));
    EXPECT_EQ(out.x, 0x0102);
    EXPECT_EQ(out.y, -2);
    EXPECT_EQ(out.status, POINT_LAST | POINT_BLANK);
    EXPECT_EQ(out.b, 0xBB);
    EXPECT_EQ(out.g, 0xC4);
    EXPECT_EQ(out.r, 0xD5);
}

TEST(PointStream, IldRecordFormat4DropsZ) {
    // 3D: X=0x0102, Y=3, Z=0x7FFF (ignored), status=0, B,G,R = 1,2,3
    uint8_t rec[10] = {0x01, 0x02, 0x00, 0x03, 0x7F, 0xFF, 0x00, 0x01, 0x02, 0x03};
    laser_point_t out{};
    ASSERT_TRUE(point_stream_ild_record(4, rec, &out));
    EXPECT_EQ(out.x, 0x0102);
    EXPECT_EQ(out.y, 3);
    EXPECT_EQ(out.b, 0x01);
    EXPECT_EQ(out.g, 0x02);
    EXPECT_EQ(out.r, 0x03);
}

TEST(PointStream, IldRecordRejectsUnsupported) {
    uint8_t rec[8] = {0};
    laser_point_t out{};
    EXPECT_FALSE(point_stream_ild_record(1, rec, &out));
}
