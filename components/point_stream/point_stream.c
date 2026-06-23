#include "point_stream.h"

#include <string.h>
#include <stdatomic.h>

// The triple-buffer core is portable C so the host unit test links it directly;
// the TCP server task is device-only (lwIP sockets / FreeRTOS).
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#endif

#define NSLOT 3u
#define DIRTY 0x80u   // mailbox "fresh scene" flag (slot index is the low bits)

// Three scene slots in internal DRAM. The renderer pushes points straight from
// here into the ring — no flash on the hot path.
static laser_point_t      s_buf[NSLOT][POINT_STREAM_MAX_PTS];
static uint32_t           s_len[NSLOT];
static _Atomic uint8_t    s_mailbox;     // ready slot | DIRTY
static uint8_t            s_write_idx;    // producer-owned back slot
static uint8_t            s_read_idx;     // consumer-owned slot
static bool               s_have;         // consumer has taken at least one scene

void point_stream_init(void)
{
    // The three indices start distinct (0 write, 1 read, 2 mailbox); each
    // exchange below only ever swaps two of them, so they stay distinct — the
    // invariant that makes the triple buffer tear-free.
    s_write_idx = 0;
    s_read_idx  = 1;
    atomic_store_explicit(&s_mailbox, 2u, memory_order_relaxed);  // not DIRTY
    s_len[0] = s_len[1] = s_len[2] = 0;
    s_have = false;
}

laser_point_t *point_stream_back(void)
{
    return s_buf[s_write_idx];
}

void point_stream_commit(uint32_t n)
{
    if (n > POINT_STREAM_MAX_PTS) n = POINT_STREAM_MAX_PTS;
    s_len[s_write_idx] = n;
    // Publish the back slot; take whatever slot was in the mailbox as the new
    // back slot. release so the points + length are visible before the swap.
    uint8_t pub  = (uint8_t)(s_write_idx | DIRTY);
    uint8_t prev = atomic_exchange_explicit(&s_mailbox, pub, memory_order_acq_rel);
    s_write_idx  = (uint8_t)(prev & ~DIRTY);
}

void point_stream_publish(const laser_point_t *pts, uint32_t n)
{
    if (n > POINT_STREAM_MAX_PTS) n = POINT_STREAM_MAX_PTS;
    memcpy(s_buf[s_write_idx], pts, (size_t)n * sizeof(laser_point_t));
    point_stream_commit(n);
}

bool point_stream_get(const laser_point_t **pts, uint32_t *count)
{
    if (atomic_load_explicit(&s_mailbox, memory_order_acquire) & DIRTY) {
        // A fresh scene is waiting: swap our read slot in for it (acquire so the
        // producer's writes are visible).
        uint8_t taken = atomic_exchange_explicit(&s_mailbox, s_read_idx,
                                                 memory_order_acq_rel);
        s_read_idx = (uint8_t)(taken & ~DIRTY);
        s_have = true;
    }
    if (!s_have) return false;
    *pts   = s_buf[s_read_idx];
    *count = s_len[s_read_idx];
    return true;
}

// ---------------------------------------------------------------------------
// ILDA record decode (portable). ILDA true-colour records are big-endian X/Y
// then a status byte then B,G,R; the status bits (0x80 last / 0x40 blank) are
// identical to ours, so they pass straight through. Format 5 is 2D (status at
// offset 4); format 4 is 3D (a 2-byte Z we drop, status at offset 6).
// ---------------------------------------------------------------------------
uint32_t point_stream_ild_recsize(uint8_t format)
{
    switch (format) {
        case 5: return 8;   // 2D true colour
        case 4: return 10;  // 3D true colour (Z dropped)
        default: return 0;  // unsupported (indexed-colour 0/1/2 not handled)
    }
}

bool point_stream_ild_record(uint8_t format, const uint8_t *rec, laser_point_t *out)
{
    uint32_t off;
    if (format == 5)      off = 4;
    else if (format == 4) off = 6;   // skip the 2-byte Z
    else return false;

    out->x      = (int16_t)(((uint16_t)rec[0] << 8) | rec[1]);
    out->y      = (int16_t)(((uint16_t)rec[2] << 8) | rec[3]);
    out->status = rec[off] & (POINT_BLANK | POINT_LAST);
    out->b      = rec[off + 1];
    out->g      = rec[off + 2];
    out->r      = rec[off + 3];
    return true;
}

// ---------------------------------------------------------------------------
// Device-only: the TCP scene-receiver task.
// ---------------------------------------------------------------------------
#if defined(ESP_PLATFORM)

static const char *TAG = "point_stream";
static uint16_t    s_port;

static bool recv_all(int fd, uint8_t *buf, size_t n)
{
    size_t got = 0;
    while (got < n) {
        int r = recv(fd, buf + got, n - got, 0);
        if (r <= 0) return false;   // peer closed or error
        got += (size_t)r;
    }
    return true;
}

static void server_task(void *arg)
{
    (void)arg;
    int listenfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenfd < 0) { ESP_LOGE(TAG, "socket() failed"); vTaskDelete(NULL); return; }

    int one = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(s_port);
    if (bind(listenfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed"); close(listenfd); vTaskDelete(NULL); return;
    }
    if (listen(listenfd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed"); close(listenfd); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "scene server listening on :%u", (unsigned)s_port);

    for (;;) {
        int fd = accept(listenfd, NULL, NULL);
        if (fd < 0) continue;
        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        // Parse ILDA sections off the wire until a 0-record terminator (then
        // ACK) or the peer closes. Each data section is published as a scene.
        for (;;) {
            uint8_t ihdr[32];
            if (!recv_all(fd, ihdr, sizeof(ihdr))) break;
            if (memcmp(ihdr, "ILDA", 4) != 0) {
                ESP_LOGW(TAG, "not an ILDA header"); break;
            }
            uint8_t  format = ihdr[7];
            uint32_t count  = ((uint32_t)ihdr[24] << 8) | ihdr[25];   // BE u16

            if (count == 0) {
                // Terminating header: end of this ILDA stream — ack and keep the
                // connection open for a possible next file.
                uint8_t ack = POINT_STREAM_ACK;
                send(fd, &ack, 1, 0);
                continue;
            }

            uint32_t recsize = point_stream_ild_recsize(format);
            if (recsize == 0) {
                ESP_LOGW(TAG, "unsupported ILDA format %u", (unsigned)format);
                break;
            }

            // Decode records into the DRAM back slot (clamped to MAX_PTS; any
            // excess is consumed off the socket but dropped).
            laser_point_t *back = point_stream_back();
            uint32_t stored = 0;
            bool ok = true;
            for (uint32_t i = 0; i < count; i++) {
                uint8_t rec[10];
                if (!recv_all(fd, rec, recsize)) { ok = false; break; }
                if (stored < POINT_STREAM_MAX_PTS) {
                    point_stream_ild_record(format, rec, &back[stored]);
                    stored++;
                }
            }
            if (!ok) break;
            point_stream_commit(stored);
            ESP_LOGI(TAG, "scene: %u points (ILDA fmt %u)",
                     (unsigned)stored, (unsigned)format);
        }
        close(fd);
    }
}

bool point_stream_start(uint16_t port)
{
    s_port = port;
    BaseType_t ok = xTaskCreate(server_task, "scene_srv", 4096, NULL, 4, NULL);
    return ok == pdPASS;
}

#endif // ESP_PLATFORM
