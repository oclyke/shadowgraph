#include "artnet_control.h"

#include <string.h>

// The packet parse + DMX decode are portable so the host unit test links them
// directly; the UDP receiver task and the shared-state store are device-only.
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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
#define ARTNET_OP_DMX    0x5000u
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
}

// ---------------------------------------------------------------------------
// Device runtime: shared state + Art-Net UDP receiver task.
// ---------------------------------------------------------------------------
#if defined(ESP_PLATFORM)

static const char *TAG = "artnet";

#define ARTNET_UDP_PORT 6454

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
        int n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) continue;

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
