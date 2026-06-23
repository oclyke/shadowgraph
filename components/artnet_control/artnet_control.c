#include "artnet_control.h"

#include <string.h>

// The packet parse + DMX decode are portable so the host unit test links them
// directly; the UDP receiver task and the shared-state store are device-only.
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI_F ((float)(2.0 * M_PI))

// ---------------------------------------------------------------------------
// Art-Net ArtDmx parse (portable). Wire layout (little-endian OpCode, big-endian
// ProtVer/Length per the Art-Net spec):
//   0  "Art-Net\0"            8 bytes
//   8  OpCode (LE)            0x5000 = OpDmx
//   10 ProtVerHi/Lo (BE)      >= 14
//   12 Sequence  13 Physical
//   14 SubUni    15 Net       -> 15-bit port address = (Net<<8)|SubUni
//   16 LengthHi/Lo (BE)       DMX slot count, 2..512, even
//   18 DMX data
// ---------------------------------------------------------------------------
#define ARTNET_ID        "Art-Net"     // 7 chars + implicit NUL = 8 bytes
#define ARTNET_OP_POLL      0x2000u
#define ARTNET_OP_POLLREPLY 0x2100u
#define ARTNET_OP_DMX       0x5000u
#define ARTNET_HEADER    18u

bool artnet_parse(const uint8_t *pkt, size_t len,
                  uint16_t *universe, const uint8_t **dmx, uint16_t *dmxlen)
{
    if (!pkt || len < ARTNET_HEADER) return false;
    if (memcmp(pkt, ARTNET_ID, 8) != 0) return false;     // includes the NUL

    uint16_t opcode = (uint16_t)(pkt[8] | ((uint16_t)pkt[9] << 8));  // little-endian
    if (opcode != ARTNET_OP_DMX) return false;

    uint16_t protver = (uint16_t)(((uint16_t)pkt[10] << 8) | pkt[11]);  // big-endian
    if (protver < 14) return false;

    uint16_t slots = (uint16_t)(((uint16_t)pkt[16] << 8) | pkt[17]);    // big-endian
    if (slots < 2 || slots > 512) return false;
    if (len < (size_t)ARTNET_HEADER + slots) return false;

    if (universe) *universe = (uint16_t)(((uint16_t)(pkt[15] & 0x7f) << 8) | pkt[14]);
    if (dmx)      *dmx      = pkt + ARTNET_HEADER;
    if (dmxlen)   *dmxlen   = slots;
    return true;
}

// ---------------------------------------------------------------------------
// Discovery: ArtPoll (query, OpCode 0x2000) and ArtPollReply (response, 0x2100).
// A controller broadcasts an ArtPoll; we reply with our IP + name so it learns
// where to send. ArtPollReply is a fixed 239-byte record; only a handful of
// fields are load-bearing for discovery (IP, port, names, the universe we
// listen on) — the rest are spec-mandated and left zero. Field offsets follow
// the Art-Net 4 spec.
// ---------------------------------------------------------------------------
bool artnet_is_poll(const uint8_t *pkt, size_t len)
{
    if (!pkt || len < 12) return false;                  // ID + OpCode + ProtVer
    if (memcmp(pkt, ARTNET_ID, 8) != 0) return false;
    uint16_t opcode = (uint16_t)(pkt[8] | ((uint16_t)pkt[9] << 8));
    return opcode == ARTNET_OP_POLL;
}

size_t artnet_pollreply_build(uint8_t *out, size_t cap, const uint8_t ip[4],
                              uint16_t universe, const char *short_name,
                              const char *long_name)
{
    if (!out || cap < ARTNET_POLLREPLY_SIZE) return 0;

    memset(out, 0, ARTNET_POLLREPLY_SIZE);
    memcpy(out, ARTNET_ID, 8);                           // "Art-Net\0"
    out[8]  = (uint8_t)(ARTNET_OP_POLLREPLY & 0xff);     // OpCode, little-endian
    out[9]  = (uint8_t)(ARTNET_OP_POLLREPLY >> 8);       //   -> 0x00, 0x21
    if (ip) { out[10] = ip[0]; out[11] = ip[1]; out[12] = ip[2]; out[13] = ip[3]; }
    out[14] = 0x36; out[15] = 0x19;                      // Port 6454, little-endian
    out[16] = 0;    out[17] = 1;                         // VersInfo Hi/Lo
    out[18] = (uint8_t)((universe >> 8) & 0x7f);         // NetSwitch (bits 14..8)
    out[19] = (uint8_t)((universe >> 4) & 0x0f);         // SubSwitch (bits 7..4)
    out[20] = 0x00; out[21] = 0xff;                      // OEM = unknown
    out[23] = 0xd0;                                      // Status1: indicators normal
    if (short_name) { strncpy((char *)&out[26], short_name, 17); out[43]  = '\0'; }
    if (long_name)  { strncpy((char *)&out[44], long_name,  63); out[107] = '\0'; }
    out[173] = 1;                                        // NumPortsLo = 1
    out[174] = 0x80;                                     // PortTypes[0]: Art-Net -> output
    out[182] = 0x80;                                     // GoodOutput[0]: transmitting
    out[190] = (uint8_t)(universe & 0x0f);               // SwOut[0] (address low nibble)
    out[200] = 0x00;                                     // Style = StNode
    return ARTNET_POLLREPLY_SIZE;
}

// ---------------------------------------------------------------------------
// DMX frame -> control state. Channels are read relative to base_channel; any
// channel past the end of the frame reads as 0 (so a short frame degrades to the
// "all faders down" state rather than reading garbage).
// ---------------------------------------------------------------------------
void artnet_control_decode(const uint8_t *dmx, uint16_t len,
                           uint16_t base_channel, artnet_control_state_t *out)
{
    if (!out) return;

    uint8_t ch[ARTNET_NUM_CHANNELS] = {0};
    uint32_t first = (base_channel == 0) ? 0u : (uint32_t)(base_channel - 1u);
    for (uint32_t i = 0; i < ARTNET_NUM_CHANNELS; i++) {
        uint32_t idx = first + i;
        if (dmx && idx < len) ch[i] = dmx[idx];
    }

    out->mode       = (ch[0] >= 128) ? ARTNET_MODE_STREAM : ARTNET_MODE_PATTERN;
    out->fx         = (uint8_t)(ARTNET_FREQ_MIN + (uint32_t)ch[1] * (ARTNET_FREQ_MAX - ARTNET_FREQ_MIN) / 255u);
    out->fy         = (uint8_t)(ARTNET_FREQ_MIN + (uint32_t)ch[2] * (ARTNET_FREQ_MAX - ARTNET_FREQ_MIN) / 255u);
    out->amplitude  = (int16_t)((uint32_t)ch[3] * ARTNET_AMP_MAX / 255u);
    out->hue_deg    = (float)ch[4] * (360.0f / 255.0f);
    out->intensity  = (float)ch[5] / 255.0f;
    out->morph_rate = (float)ch[6] * (ARTNET_MORPH_MAX_RAD_S / 255.0f);
    out->phase_off  = (float)ch[7] * (TWO_PI_F / 255.0f);

    out->blank_width      = (float)ch[8]  * (ARTNET_BLANK_WIDTH_MAX      / 255.0f);
    out->blank_slide_rate = (float)ch[9]  * (ARTNET_BLANK_SLIDE_MAX      / 255.0f);
    out->color_t_span     = (float)ch[10] * (ARTNET_COLOR_SPAN_MAX       / 255.0f);
    out->color_cycle_rate = (float)ch[11] * (ARTNET_COLOR_CYCLE_MAX      / 255.0f);
    out->fx_morph_rate    = (float)ch[12] * (ARTNET_FREQ_MORPH_RATE_MAX  / 255.0f);
    out->fy_morph_rate    = (float)ch[13] * (ARTNET_FREQ_MORPH_RATE_MAX  / 255.0f);
    out->freq_morph_depth = (float)ch[14] * (ARTNET_FREQ_MORPH_DEPTH_MAX / 255.0f);
}

void artnet_control_defaults(artnet_control_state_t *out)
{
    if (!out) return;
    out->mode       = ARTNET_MODE_PATTERN;
    out->fx         = 3;
    out->fy         = 2;
    out->amplitude  = ARTNET_AMP_MAX;     // full (linear-range) size
    out->hue_deg    = 0.0f;
    out->intensity  = 0.25f;
    out->morph_rate = 1.0f;               // ~6 s per full morph
    out->phase_off  = 0.0f;
    // New effects default off, so the out-of-the-box figure is the plain
    // morphing Lissajous; dial them in over Art-Net.
    out->blank_width      = 0.0f;
    out->blank_slide_rate = 0.0f;
    out->color_t_span     = 0.0f;
    out->color_cycle_rate = 0.0f;
    out->fx_morph_rate    = 0.0f;
    out->fy_morph_rate    = 0.0f;
    out->freq_morph_depth = 0.0f;
}

// ---------------------------------------------------------------------------
// Device runtime: shared state + Art-Net UDP receiver task.
// ---------------------------------------------------------------------------
#if defined(ESP_PLATFORM)

static const char *TAG = "artnet";

#define ARTNET_UDP_PORT 6454

// Names a controller (or tools/artnetctl --discover) shows for this node.
#define ARTNET_SHORT_NAME "shadowgraph"
#define ARTNET_LONG_NAME  "shadowgraph laser projector"

// Fetch this node's IPv4 as four octets a.b.c.d. Tries the station netif first
// (normal STA streaming setup), then the SoftAP. Returns false before a lease.
static bool self_ip(uint8_t ip[4])
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (!netif) return false;

    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) return false;

    uint32_t a = info.ip.addr;   // network byte order: low byte is the first octet
    ip[0] = (uint8_t)(a & 0xff);
    ip[1] = (uint8_t)((a >> 8) & 0xff);
    ip[2] = (uint8_t)((a >> 16) & 0xff);
    ip[3] = (uint8_t)((a >> 24) & 0xff);
    return true;
}

static portMUX_TYPE          s_lock = portMUX_INITIALIZER_UNLOCKED;
static artnet_control_state_t s_state;        // guarded by s_lock
static uint16_t              s_universe;
static uint16_t              s_base_channel;

void artnet_control_init(const artnet_control_state_t *defaults,
                         uint16_t universe, uint16_t base_channel)
{
    artnet_control_state_t seed;
    if (defaults) seed = *defaults;
    else          artnet_control_defaults(&seed);

    taskENTER_CRITICAL(&s_lock);
    s_state        = seed;
    s_universe     = universe;
    s_base_channel = base_channel;
    taskEXIT_CRITICAL(&s_lock);
}

void artnet_control_get(artnet_control_state_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}

static void receiver_task(void *arg)
{
    (void)arg;
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) { ESP_LOGE(TAG, "socket() failed"); vTaskDelete(NULL); return; }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);     // receives broadcast + unicast
    addr.sin_port        = htons(ARTNET_UDP_PORT);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(:%d) failed", ARTNET_UDP_PORT);
        close(fd); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "listening on UDP :%d (universe %u, base ch %u)",
             ARTNET_UDP_PORT, (unsigned)s_universe, (unsigned)s_base_channel);

    static uint8_t buf[600];   // an ArtDmx with a full 512-slot universe is 530 B
    bool seen = false;
    for (;;) {
        struct sockaddr_in src = {0};
        socklen_t slen = sizeof(src);
        int n = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&src, &slen);
        if (n <= 0) continue;

        // Discovery: answer an ArtPoll with our IP + name, unicast back to the
        // poller (works whether it polled from :6454 or an ephemeral port).
        if (artnet_is_poll(buf, (size_t)n)) {
            uint8_t ip[4];
            uint8_t reply[ARTNET_POLLREPLY_SIZE];
            if (self_ip(ip) &&
                artnet_pollreply_build(reply, sizeof(reply), ip, s_universe,
                                       ARTNET_SHORT_NAME, ARTNET_LONG_NAME)) {
                sendto(fd, reply, sizeof(reply), 0, (struct sockaddr *)&src, slen);
            }
            continue;
        }

        uint16_t universe = 0, dmxlen = 0;
        const uint8_t *dmx = NULL;
        if (!artnet_parse(buf, (size_t)n, &universe, &dmx, &dmxlen)) continue;
        if (universe != s_universe) continue;     // not our fixture's universe

        artnet_control_state_t st;
        artnet_control_decode(dmx, dmxlen, s_base_channel, &st);
        taskENTER_CRITICAL(&s_lock);
        s_state = st;
        taskEXIT_CRITICAL(&s_lock);

        if (!seen) {
            ESP_LOGI(TAG, "first frame: mode=%s fx=%u fy=%u size=%d",
                     st.mode == ARTNET_MODE_STREAM ? "stream" : "pattern",
                     (unsigned)st.fx, (unsigned)st.fy, (int)st.amplitude);
            seen = true;
        }
    }
}

bool artnet_control_start(void)
{
    BaseType_t ok = xTaskCreate(receiver_task, "artnet", 4096, NULL, 4, NULL);
    return ok == pdPASS;
}

#endif // ESP_PLATFORM
