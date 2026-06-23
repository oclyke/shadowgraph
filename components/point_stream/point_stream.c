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

        // Serve scenes on this connection until the peer closes or errors.
        for (;;) {
            uint8_t hdr[8];
            if (!recv_all(fd, hdr, sizeof(hdr))) break;
            if (memcmp(hdr, POINT_STREAM_MAGIC, 4) != 0) {
                ESP_LOGW(TAG, "bad magic"); break;
            }
            uint32_t count = (uint32_t)hdr[4] | ((uint32_t)hdr[5] << 8) |
                             ((uint32_t)hdr[6] << 16) | ((uint32_t)hdr[7] << 24);
            if (count == 0 || count > POINT_STREAM_MAX_PTS) {
                ESP_LOGW(TAG, "bad count %u (max %u)",
                         (unsigned)count, (unsigned)POINT_STREAM_MAX_PTS);
                break;
            }
            // Receive the records straight into the DRAM back slot (zero-copy).
            // Wire records are 8-byte little-endian laser_point_t, matching the
            // struct on this little-endian target.
            if (!recv_all(fd, (uint8_t *)point_stream_back(),
                          (size_t)count * sizeof(laser_point_t))) {
                break;
            }
            point_stream_commit(count);

            uint8_t ack = POINT_STREAM_ACK;
            send(fd, &ack, 1, 0);
            ESP_LOGI(TAG, "scene received: %u points", (unsigned)count);
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
