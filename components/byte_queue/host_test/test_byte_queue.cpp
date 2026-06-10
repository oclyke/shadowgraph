// Host unit tests for byte_queue (gtest). Build via the CMake project here.
#include "byte_queue.h"

#include <gtest/gtest.h>
#include <cstring>
#include <thread>

TEST(ByteQueue, InitRejectsBadArgs) {
    byte_queue_t q;
    uint8_t buf[16];
    EXPECT_TRUE(byte_queue_init(&q, buf, 16));
    EXPECT_FALSE(byte_queue_init(&q, buf, 15));   // not a power of two
    EXPECT_FALSE(byte_queue_init(&q, buf, 0));
    EXPECT_FALSE(byte_queue_init(nullptr, buf, 16));
}

TEST(ByteQueue, EmptyState) {
    byte_queue_t q;
    uint8_t buf[16];
    byte_queue_init(&q, buf, 16);
    EXPECT_EQ(byte_queue_avail(&q), 0u);
    EXPECT_EQ(byte_queue_free(&q), 16u);
}

TEST(ByteQueue, RoundTrip) {
    byte_queue_t q;
    uint8_t buf[16];
    byte_queue_init(&q, buf, 16);
    uint8_t in[5] = {1, 2, 3, 4, 5}, out[5] = {0};
    EXPECT_TRUE(byte_queue_write(&q, in, 5));
    EXPECT_EQ(byte_queue_avail(&q), 5u);
    EXPECT_EQ(byte_queue_free(&q), 11u);
    EXPECT_EQ(byte_queue_read(&q, out, 5), 5u);
    EXPECT_EQ(memcmp(in, out, 5), 0);
    EXPECT_EQ(byte_queue_avail(&q), 0u);
}

TEST(ByteQueue, AllOrNothing) {
    byte_queue_t q;
    uint8_t buf[8], in[8] = {0};
    byte_queue_init(&q, buf, 8);
    EXPECT_TRUE(byte_queue_write(&q, in, 8));     // exactly fills capacity
    EXPECT_EQ(byte_queue_free(&q), 0u);
    EXPECT_FALSE(byte_queue_write(&q, in, 1));    // full

    byte_queue_init(&q, buf, 8);
    byte_queue_write(&q, in, 5);
    EXPECT_FALSE(byte_queue_write(&q, in, 4));    // only 3 free; nothing written
    EXPECT_EQ(byte_queue_avail(&q), 5u);          // unchanged
}

TEST(ByteQueue, Wrap) {
    byte_queue_t q;
    uint8_t buf[8];
    byte_queue_init(&q, buf, 8);
    uint8_t a[6] = {1, 2, 3, 4, 5, 6}, tmp[6];
    byte_queue_write(&q, a, 6);
    byte_queue_read(&q, tmp, 6);                  // read offset now at 6
    uint8_t b[6] = {10, 20, 30, 40, 50, 60}, out[6] = {0};
    EXPECT_TRUE(byte_queue_write(&q, b, 6));       // straddles the buffer end
    EXPECT_EQ(byte_queue_read(&q, out, 6), 6u);
    EXPECT_EQ(memcmp(b, out, 6), 0);
}

TEST(ByteQueue, PeekDoesNotAdvance) {
    byte_queue_t q;
    uint8_t buf[16];
    byte_queue_init(&q, buf, 16);
    uint8_t in[4] = {9, 8, 7, 6}, out[4] = {0};
    byte_queue_write(&q, in, 4);
    EXPECT_EQ(byte_queue_peek(&q, out, 4), 4u);
    EXPECT_EQ(memcmp(in, out, 4), 0);
    EXPECT_EQ(byte_queue_avail(&q), 4u);          // peek did not advance
    memset(out, 0, 4);
    EXPECT_EQ(byte_queue_read(&q, out, 4), 4u);
    EXPECT_EQ(memcmp(in, out, 4), 0);
}

TEST(ByteQueue, PartialRead) {
    byte_queue_t q;
    uint8_t buf[16];
    byte_queue_init(&q, buf, 16);
    uint8_t in[3] = {1, 2, 3}, out[8] = {0};
    byte_queue_write(&q, in, 3);
    EXPECT_EQ(byte_queue_read(&q, out, 8), 3u);   // only 3 were available
}

TEST(ByteQueue, CounterWrap) {
    // Force the free-running counters near UINT32_MAX to exercise the modular
    // arithmetic across the wrap boundary. (head/tail are std::atomic here.)
    byte_queue_t q;
    uint8_t buf[16];
    byte_queue_init(&q, buf, 16);
    q.head = 0xFFFFFFFEu;
    q.tail = 0xFFFFFFFEu;
    EXPECT_EQ(byte_queue_avail(&q), 0u);
    EXPECT_EQ(byte_queue_free(&q), 16u);
    uint8_t in[8], out[8];
    for (int i = 0; i < 8; i++) in[i] = static_cast<uint8_t>(i + 1);
    EXPECT_TRUE(byte_queue_write(&q, in, 8));      // head: 0xFFFFFFFE -> 0x6
    EXPECT_EQ(byte_queue_avail(&q), 8u);
    EXPECT_EQ(byte_queue_read(&q, out, 8), 8u);
    EXPECT_EQ(memcmp(in, out, 8), 0);
}

// Concurrent SPSC stress: one producer thread, the main thread consuming.
// Exercises the acquire/release handoff and FIFO integrity under real threads.
TEST(ByteQueue, SpscStress) {
    byte_queue_t q;
    uint8_t buf[64];
    byte_queue_init(&q, buf, 64);
    constexpr uint32_t N = 2000000;

    std::thread producer([&] {
        uint32_t i = 0;
        while (i < N) {
            uint8_t b = static_cast<uint8_t>(i & 0xFF);
            if (byte_queue_write(&q, &b, 1)) i++;
        }
    });

    uint32_t i = 0;
    bool ok = true;
    while (i < N) {
        uint8_t b;
        if (byte_queue_read(&q, &b, 1) == 1) {
            if (b != static_cast<uint8_t>(i & 0xFF)) { ok = false; break; }
            i++;
        }
    }
    producer.join();
    EXPECT_TRUE(ok);
    EXPECT_EQ(i, N);
}
