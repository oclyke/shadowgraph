// Host unit tests for the portable half of artnet_control: the ArtDmx packet
// parser and the DMX -> control-state decode. The device receiver task and the
// shared-state store (ESP_PLATFORM only) are not exercised here.
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "artnet_control.h"

namespace {

// Build a well-formed ArtDmx packet carrying `dmx` for `universe`.
std::vector<uint8_t> make_artdmx(uint16_t universe, const std::vector<uint8_t> &dmx,
                                 uint8_t seq = 0)
{
    std::vector<uint8_t> p = {'A','r','t','-','N','e','t','\0'};
    p.push_back(0x00); p.push_back(0x50);              // OpDmx, little-endian
    p.push_back(0x00); p.push_back(0x0e);              // ProtVer 14, big-endian
    p.push_back(seq);
    p.push_back(0x00);                                 // physical
    p.push_back((uint8_t)(universe & 0xff));           // SubUni
    p.push_back((uint8_t)((universe >> 8) & 0x7f));    // Net
    uint16_t len = (uint16_t)dmx.size();
    p.push_back((uint8_t)(len >> 8));                  // LengthHi (big-endian)
    p.push_back((uint8_t)(len & 0xff));                // LengthLo
    p.insert(p.end(), dmx.begin(), dmx.end());
    return p;
}

TEST(ArtnetParse, AcceptsValidPacket)
{
    std::vector<uint8_t> dmx(512, 0);
    dmx[0] = 200; dmx[5] = 42;
    auto pkt = make_artdmx(/*universe=*/1, dmx);

    uint16_t universe = 0xffff, dmxlen = 0;
    const uint8_t *data = nullptr;
    ASSERT_TRUE(artnet_parse(pkt.data(), pkt.size(), &universe, &data, &dmxlen));
    EXPECT_EQ(universe, 1u);
    EXPECT_EQ(dmxlen, 512u);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data[0], 200);
    EXPECT_EQ(data[5], 42);
}

TEST(ArtnetParse, ExtractsNetAndSubUniUniverse)
{
    // Net = 2, SubUni = 0x13 -> port address 0x0213 = 531.
    auto pkt = make_artdmx(/*universe=*/0x0213, std::vector<uint8_t>(8, 0));
    uint16_t universe = 0;
    ASSERT_TRUE(artnet_parse(pkt.data(), pkt.size(), &universe, nullptr, nullptr));
    EXPECT_EQ(universe, 0x0213u);
}

TEST(ArtnetParse, RejectsBadId)
{
    auto pkt = make_artdmx(1, std::vector<uint8_t>(8, 0));
    pkt[0] = 'X';
    EXPECT_FALSE(artnet_parse(pkt.data(), pkt.size(), nullptr, nullptr, nullptr));
}

TEST(ArtnetParse, RejectsWrongOpcode)
{
    auto pkt = make_artdmx(1, std::vector<uint8_t>(8, 0));
    pkt[8] = 0x00; pkt[9] = 0x20;   // OpPoll, not OpDmx
    EXPECT_FALSE(artnet_parse(pkt.data(), pkt.size(), nullptr, nullptr, nullptr));
}

TEST(ArtnetParse, RejectsOldProtocolVersion)
{
    auto pkt = make_artdmx(1, std::vector<uint8_t>(8, 0));
    pkt[10] = 0x00; pkt[11] = 0x0d;   // ProtVer 13 (< 14)
    EXPECT_FALSE(artnet_parse(pkt.data(), pkt.size(), nullptr, nullptr, nullptr));
}

TEST(ArtnetParse, RejectsTruncatedHeaderAndData)
{
    auto pkt = make_artdmx(1, std::vector<uint8_t>(16, 0));
    // Header-only is too short.
    EXPECT_FALSE(artnet_parse(pkt.data(), 10, nullptr, nullptr, nullptr));
    // Claims 16 slots but only 8 bytes of data are present.
    EXPECT_FALSE(artnet_parse(pkt.data(), 18 + 8, nullptr, nullptr, nullptr));
}

TEST(ArtnetParse, RejectsOutOfRangeLength)
{
    auto pkt = make_artdmx(1, std::vector<uint8_t>(2, 0));
    pkt[16] = 0x00; pkt[17] = 0x00;   // length 0 (< 2)
    EXPECT_FALSE(artnet_parse(pkt.data(), pkt.size(), nullptr, nullptr, nullptr));
}

TEST(ArtnetDecode, ModeThreshold)
{
    std::vector<uint8_t> dmx(ARTNET_NUM_CHANNELS, 0);
    artnet_control_state_t st;

    dmx[0] = 0;   artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.mode, ARTNET_MODE_PATTERN);
    dmx[0] = 127; artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.mode, ARTNET_MODE_PATTERN);
    dmx[0] = 128; artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.mode, ARTNET_MODE_STREAM);
    dmx[0] = 255; artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.mode, ARTNET_MODE_STREAM);
}

TEST(ArtnetDecode, FrequencyMapsToOneThroughEight)
{
    std::vector<uint8_t> dmx(ARTNET_NUM_CHANNELS, 0);
    artnet_control_state_t st;

    dmx[1] = 0;   dmx[2] = 255;
    artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.fx, ARTNET_FREQ_MIN);   // 1
    EXPECT_EQ(st.fy, ARTNET_FREQ_MAX);   // 8

    dmx[1] = 73;   // round((3-1)*255/7) -> decodes back to ratio 3
    artnet_control_decode(dmx.data(), dmx.size(), 1, &st);
    EXPECT_EQ(st.fx, 3u);
}

TEST(ArtnetDecode, ScalarChannelsMapToPhysicalUnits)
{
    std::vector<uint8_t> dmx(ARTNET_NUM_CHANNELS, 0);
    dmx[3] = 255;   // size
    dmx[4] = 255;   // hue
    dmx[5] = 255;   // intensity
    dmx[6] = 255;   // morph
    artnet_control_state_t st;
    artnet_control_decode(dmx.data(), dmx.size(), 1, &st);

    EXPECT_EQ(st.amplitude, ARTNET_AMP_MAX);
    EXPECT_NEAR(st.hue_deg, 360.0f, 0.01f);
    EXPECT_NEAR(st.intensity, 1.0f, 0.01f);
    EXPECT_NEAR(st.morph_rate, ARTNET_MORPH_MAX_RAD_S, 0.01f);
}

TEST(ArtnetDecode, HonorsBaseChannel)
{
    // Patch the fixture at channel 10: mode byte lives at index 9.
    std::vector<uint8_t> dmx(32, 0);
    dmx[9]  = 200;   // mode -> stream
    dmx[12] = 255;   // size (channel 13 = base+3)
    artnet_control_state_t st;
    artnet_control_decode(dmx.data(), dmx.size(), /*base=*/10, &st);

    EXPECT_EQ(st.mode, ARTNET_MODE_STREAM);
    EXPECT_EQ(st.amplitude, ARTNET_AMP_MAX);
}

TEST(ArtnetDecode, ChannelsBeyondFrameReadAsZero)
{
    // A frame shorter than base+channels: everything past the end decodes as 0.
    std::vector<uint8_t> dmx(3, 255);
    artnet_control_state_t st;
    artnet_control_decode(dmx.data(), dmx.size(), /*base=*/100, &st);

    EXPECT_EQ(st.mode, ARTNET_MODE_PATTERN);
    EXPECT_EQ(st.amplitude, 0);
    EXPECT_NEAR(st.intensity, 0.0f, 0.001f);
}

TEST(ArtnetDefaults, AreAPatternLissajous)
{
    artnet_control_state_t st;
    artnet_control_defaults(&st);
    EXPECT_EQ(st.mode, ARTNET_MODE_PATTERN);
    EXPECT_EQ(st.fx, 3u);
    EXPECT_EQ(st.fy, 2u);
    EXPECT_GT(st.amplitude, 0);
    EXPECT_GT(st.intensity, 0.0f);
}

// Build a minimal ArtPoll (discovery query): header + OpPoll + ProtVer + flags.
std::vector<uint8_t> make_artpoll()
{
    std::vector<uint8_t> p = {'A','r','t','-','N','e','t','\0'};
    p.push_back(0x00); p.push_back(0x20);   // OpPoll, little-endian
    p.push_back(0x00); p.push_back(0x0e);   // ProtVer 14
    p.push_back(0x00);                      // TalkToMe
    p.push_back(0x00);                      // Priority
    return p;
}

TEST(ArtnetPoll, DetectsPollAndRejectsOthers)
{
    auto poll = make_artpoll();
    EXPECT_TRUE(artnet_is_poll(poll.data(), poll.size()));

    // An ArtDmx is Art-Net but not a poll.
    auto dmx = make_artdmx(1, std::vector<uint8_t>(8, 0));
    EXPECT_FALSE(artnet_is_poll(dmx.data(), dmx.size()));

    // Wrong magic / too short.
    poll[0] = 'X';
    EXPECT_FALSE(artnet_is_poll(poll.data(), poll.size()));
    EXPECT_FALSE(artnet_is_poll(poll.data(), 4));
}

TEST(ArtnetPollReply, IsWellFormed)
{
    const uint8_t ip[4] = {192, 168, 1, 50};
    uint8_t out[ARTNET_POLLREPLY_SIZE];
    size_t n = artnet_pollreply_build(out, sizeof(out), ip, /*universe=*/0x0123,
                                      "shadowgraph", "shadowgraph laser projector");
    ASSERT_EQ(n, (size_t)ARTNET_POLLREPLY_SIZE);

    EXPECT_EQ(0, memcmp(out, "Art-Net\0", 8));
    EXPECT_EQ(out[8], 0x00);                 // OpPollReply low byte
    EXPECT_EQ(out[9], 0x21);                 // OpPollReply high byte
    EXPECT_EQ(0, memcmp(out + 10, ip, 4));   // node IP
    EXPECT_EQ(out[14], 0x36);                // port 6454, little-endian
    EXPECT_EQ(out[15], 0x19);
    EXPECT_STREQ((const char *)(out + 26), "shadowgraph");          // ShortName
    EXPECT_STREQ((const char *)(out + 44), "shadowgraph laser projector"); // LongName
    // Universe split across NetSwitch / SubSwitch / SwOut.
    EXPECT_EQ(out[18], (0x0123 >> 8) & 0x7f);
    EXPECT_EQ(out[19], (0x0123 >> 4) & 0x0f);
    EXPECT_EQ(out[190], 0x0123 & 0x0f);
}

TEST(ArtnetPollReply, RejectsTooSmallBuffer)
{
    uint8_t out[ARTNET_POLLREPLY_SIZE - 1];
    const uint8_t ip[4] = {10, 0, 0, 1};
    EXPECT_EQ(artnet_pollreply_build(out, sizeof(out), ip, 1, "x", "y"), (size_t)0);
}

}  // namespace
