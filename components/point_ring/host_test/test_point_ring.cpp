// Host unit tests for point_ring (gtest). Build via the CMake project here.
#include "point_ring.h"

#include <gtest/gtest.h>
#include <cstring>
#include <thread>

static laser_point_t mkpt(int16_t x, int16_t y, uint8_t r = 0, uint8_t g = 0,
                          uint8_t b = 0, uint8_t status = 0) {
    return laser_point_t{x, y, status, r, g, b};
}
static bool eq(const laser_point_t &a, const laser_point_t &b) {
    return std::memcmp(&a, &b, sizeof a) == 0;
}

TEST(PointRing, InitRejectsBadArgs) {
    point_ring_t r;
    laser_point_t buf[16];
    EXPECT_TRUE(point_ring_init(&r, buf, 16));
    EXPECT_FALSE(point_ring_init(&r, buf, 15));   // not a power of two
    EXPECT_FALSE(point_ring_init(&r, buf, 0));
    EXPECT_FALSE(point_ring_init(nullptr, buf, 16));
}

TEST(PointRing, EmptyState) {
    point_ring_t r;
    laser_point_t buf[16];
    point_ring_init(&r, buf, 16);
    EXPECT_EQ(point_ring_count(&r), 0u);
    EXPECT_EQ(point_ring_free(&r), 16u);
    laser_point_t out;
    EXPECT_FALSE(point_ring_pop(&r, &out));
}

TEST(PointRing, RoundTrip) {
    point_ring_t r;
    laser_point_t buf[16];
    point_ring_init(&r, buf, 16);
    laser_point_t in = mkpt(-1000, 2000, 0x11, 0x22, 0x33, POINT_BLANK);
    EXPECT_TRUE(point_ring_push(&r, &in));
    EXPECT_EQ(point_ring_count(&r), 1u);
    EXPECT_EQ(point_ring_free(&r), 15u);
    laser_point_t out;
    EXPECT_TRUE(point_ring_pop(&r, &out));
    EXPECT_TRUE(eq(in, out));
    EXPECT_EQ(point_ring_count(&r), 0u);
}

TEST(PointRing, FullRejectsPush) {
    point_ring_t r;
    laser_point_t buf[4];
    point_ring_init(&r, buf, 4);
    for (int i = 0; i < 4; i++) {
        laser_point_t p = mkpt((int16_t)i, (int16_t)i);
        EXPECT_TRUE(point_ring_push(&r, &p));     // fills capacity
    }
    EXPECT_EQ(point_ring_free(&r), 0u);
    laser_point_t extra = mkpt(9, 9);
    EXPECT_FALSE(point_ring_push(&r, &extra));    // full, unchanged
    EXPECT_EQ(point_ring_count(&r), 4u);
}

TEST(PointRing, BulkPushClampsToFree) {
    point_ring_t r;
    laser_point_t buf[8];
    point_ring_init(&r, buf, 8);
    laser_point_t in[10];
    for (int i = 0; i < 10; i++) in[i] = mkpt((int16_t)i, (int16_t)(-i));
    EXPECT_EQ(point_ring_push_bulk(&r, in, 10), 8u);   // only 8 fit
    EXPECT_EQ(point_ring_count(&r), 8u);
    for (int i = 0; i < 8; i++) {
        laser_point_t out;
        EXPECT_TRUE(point_ring_pop(&r, &out));
        EXPECT_TRUE(eq(in[i], out));                    // FIFO order preserved
    }
}

TEST(PointRing, Wrap) {
    point_ring_t r;
    laser_point_t buf[4];
    point_ring_init(&r, buf, 4);
    // Drain past the buffer end so the next writes straddle the wrap.
    for (int i = 0; i < 3; i++) {
        laser_point_t p = mkpt((int16_t)i, 0), out;
        point_ring_push(&r, &p);
        point_ring_pop(&r, &out);                       // read offset advances to 3
    }
    laser_point_t a = mkpt(100, 1), b = mkpt(200, 2), c = mkpt(300, 3);
    EXPECT_TRUE(point_ring_push(&r, &a));               // index 3
    EXPECT_TRUE(point_ring_push(&r, &b));               // index 0 (wrapped)
    EXPECT_TRUE(point_ring_push(&r, &c));               // index 1
    laser_point_t out;
    EXPECT_TRUE(point_ring_pop(&r, &out)); EXPECT_TRUE(eq(a, out));
    EXPECT_TRUE(point_ring_pop(&r, &out)); EXPECT_TRUE(eq(b, out));
    EXPECT_TRUE(point_ring_pop(&r, &out)); EXPECT_TRUE(eq(c, out));
}

TEST(PointRing, CounterWrap) {
    // Force the free-running counters near UINT32_MAX to exercise the modular
    // arithmetic across the wrap boundary.
    point_ring_t r;
    laser_point_t buf[8];
    point_ring_init(&r, buf, 8);
    r.head = 0xFFFFFFFEu;
    r.tail = 0xFFFFFFFEu;
    EXPECT_EQ(point_ring_count(&r), 0u);
    EXPECT_EQ(point_ring_free(&r), 8u);
    laser_point_t in[8];
    for (int i = 0; i < 8; i++) in[i] = mkpt((int16_t)(i + 1), (int16_t)(i + 1));
    EXPECT_EQ(point_ring_push_bulk(&r, in, 8), 8u);     // head: 0xFFFFFFFE -> 0x6
    EXPECT_EQ(point_ring_count(&r), 8u);
    for (int i = 0; i < 8; i++) {
        laser_point_t out;
        EXPECT_TRUE(point_ring_pop(&r, &out));
        EXPECT_TRUE(eq(in[i], out));
    }
}

// Concurrent SPSC stress: one producer thread, the main thread consuming.
// Exercises the acquire/release handoff and FIFO integrity under real threads.
TEST(PointRing, SpscStress) {
    point_ring_t r;
    laser_point_t buf[64];
    point_ring_init(&r, buf, 64);
    constexpr uint32_t N = 2000000;

    std::thread producer([&] {
        uint32_t i = 0;
        while (i < N) {
            laser_point_t p = mkpt((int16_t)(i & 0xFFFF), (int16_t)((i >> 16) & 0xFFFF));
            if (point_ring_push(&r, &p)) i++;
        }
    });

    uint32_t i = 0;
    bool ok = true;
    while (i < N) {
        laser_point_t p;
        if (point_ring_pop(&r, &p)) {
            if (p.x != (int16_t)(i & 0xFFFF) || p.y != (int16_t)((i >> 16) & 0xFFFF)) {
                ok = false;
                break;
            }
            i++;
        }
    }
    producer.join();
    EXPECT_TRUE(ok);
    EXPECT_EQ(i, N);
}
