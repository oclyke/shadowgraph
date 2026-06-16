// frame_stream: TCP frame data plane + UDP playout-clock control plane + pump.
// See frame_stream.h and docs/FRAME_STREAMING.md.
#include <string.h>
#include <stdatomic.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "frame_stream.h"
#include "frame_buffer.h"
#include "laser_command.h"
#include "laser_engine.h"

static const char *TAG = "frame_stream";

// ---------------------------------------------------------------------------
// Arena sizing. 96 KB holds hundreds of small frames or a few large ones; the
// descriptor ring caps the resident frame count.
// ---------------------------------------------------------------------------
#define ARENA_BYTES   (96 * 1024)
#define DESC_CAP      256                 // power of two

static uint8_t      s_arena[ARENA_BYTES];
static frame_desc_t s_descs[DESC_CAP];
static frame_buffer_t s_fb;

// Serializes frame_buffer access between the TCP writer and the pump. Operations
// are task-level (ms-scale), well off the ISR path.
static SemaphoreHandle_t s_fb_lock;

// "Advance" requests published by the UDP task, drained by the pump. The host
// sends a NEXT tick per desired frame step; the pump consumes the pending count
// at each loop boundary (so a slow pump catches up, a fast pump re-loops).
static _Atomic uint32_t s_advance;

// ---------------------------------------------------------------------------
// Wire formats (little-endian; see docs/FRAME_STREAMING.md §4).
// ---------------------------------------------------------------------------
#define FRAME_MAGIC   0x4652u             // 'F','R'
#define FRAME_HDR_LEN 12                  // magic(2) ver(1) flags(1) id(2) rsv(2) len(4)
#define FRAME_ACK     0x06                // 1-byte commit ack sent back on TCP

#define CTRL_MAGIC    0x5343u             // 'S','C'
#define CTRL_LEN      4                   // magic(2) ver(1) type(1)
#define CTRL_NEXT     1                   // advance to the next frame

static inline uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void lock(void)   { xSemaphoreTake(s_fb_lock, portMAX_DELAY); }
static inline void unlock(void) { xSemaphoreGive(s_fb_lock); }

// ---------------------------------------------------------------------------
// TCP frame data plane: the SOLE writer of the arena.
// ---------------------------------------------------------------------------

// Read exactly n bytes into buf; false on EOF or error (connection should close).
static bool recv_all(int sock, void *buf, uint32_t n) {
    uint8_t *p = (uint8_t *)buf;
    while (n > 0) {
        int r = recv(sock, p, n, 0);
        if (r <= 0) return false;
        p += r;
        n -= (uint32_t)r;
    }
    return true;
}

// Handle one connected client: read framed frames until it disconnects.
static void serve_conn(int sock) {
    for (;;) {
        uint8_t hdr[FRAME_HDR_LEN];
        if (!recv_all(sock, hdr, sizeof hdr)) return;

        if (rd_u16(hdr) != FRAME_MAGIC) {
            ESP_LOGW(TAG, "bad frame magic; dropping connection");
            return;
        }
        uint16_t frame_id = rd_u16(hdr + 4);
        uint32_t len      = rd_u32(hdr + 8);

        if (len == 0 || len > ARENA_BYTES) {
            ESP_LOGW(TAG, "frame %u bad len %u; dropping connection", frame_id, len);
            return;
        }

        // Reserve space, retrying (without reading more from the socket) until
        // the pump frees room — this is the backpressure path: a stalled reserve
        // stalls socket draining and TCP flow control throttles the sender.
        uint8_t *dst;
        lock();
        dst = frame_buffer_reserve(&s_fb, len);
        unlock();
        while (dst == NULL) {
            vTaskDelay(pdMS_TO_TICKS(5));
            lock();
            dst = frame_buffer_reserve(&s_fb, len);
            unlock();
        }

        // Receive the payload straight into the reserved arena region (zero-copy).
        // The region is owned by this task until commit, so no lock is needed here.
        if (!recv_all(sock, dst, len)) {
            lock();
            frame_buffer_abort(&s_fb);
            unlock();
            return;
        }

        lock();
        frame_buffer_commit(&s_fb, frame_id);
        unlock();
        ESP_LOGD(TAG, "committed frame id=%u len=%u", frame_id, len);

        // Acknowledge the commit on the TCP connection. The host blocks on this
        // before sending its UDP NEXT, so the advance can never race ahead of the
        // frame's bytes (a local write completing does not mean we have received
        // and committed it). One byte; a failed send means the peer is gone.
        uint8_t ack = FRAME_ACK;
        if (send(sock, &ack, 1, 0) != 1) return;
    }
}

static void tcp_task(void *arg) {
    uint16_t port = (uint16_t)(uintptr_t)arg;

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "tcp socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int yes = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof addr) < 0 ||
        listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "tcp bind/listen on %u failed: errno %d", port, errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "frame TCP server on port %u", port);

    for (;;) {
        struct sockaddr_in src;
        socklen_t slen = sizeof src;
        int sock = accept(listen_sock, (struct sockaddr *)&src, &slen);
        if (sock < 0) {
            ESP_LOGW(TAG, "accept() failed: errno %d", errno);
            continue;
        }
        // Disable Nagle so small frames aren't delayed.
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
        char ip[INET_ADDRSTRLEN];
        inet_ntoa_r(src.sin_addr, ip, sizeof ip);
        ESP_LOGI(TAG, "client connected from %s", ip);
        serve_conn(sock);
        close(sock);
        ESP_LOGI(TAG, "client disconnected");
    }
}

// ---------------------------------------------------------------------------
// UDP control plane: PLAY{frame_id} sets the playout target (absolute id).
// ---------------------------------------------------------------------------
static void udp_task(void *arg) {
    uint16_t port = (uint16_t)(uintptr_t)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "udp socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0) {
        ESP_LOGE(TAG, "udp bind on %u failed: errno %d", port, errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "playout-control UDP on port %u", port);

    uint8_t buf[64];
    for (;;) {
        int n = recv(sock, buf, sizeof buf, 0);
        if (n < CTRL_LEN) continue;
        if (rd_u16(buf) != CTRL_MAGIC || buf[3] != CTRL_NEXT) continue;
        atomic_fetch_add(&s_advance, 1);   // pump drains this at a loop boundary
    }
}

// ---------------------------------------------------------------------------
// Pump: loop the active frame into laser_engine, re-latching at each boundary.
// Single producer to laser_engine.
// ---------------------------------------------------------------------------

// Push one decoded command, retrying on a full engine queue (backpressure).
static void emit(const laser_command_t *c) {
    switch (c->type) {
        case LASER_CMD_GOTO:
            while (!laser_engine_goto(c->pos.x, c->pos.y))                  vTaskDelay(1);
            break;
        case LASER_CMD_LASER:
            while (!laser_engine_laser(c->col.r, c->col.g, c->col.b))       vTaskDelay(1);
            break;
        case LASER_CMD_DWELL:
            while (!laser_engine_dwell(c->dwell.dt))                        vTaskDelay(1);
            break;
        case LASER_CMD_CURVE:
            while (!laser_engine_curve(c->curve.x1, c->curve.y1,
                                       c->curve.x2, c->curve.y2,
                                       c->curve.x3, c->curve.y3,
                                       c->curve.v_in, c->curve.v_out))      vTaskDelay(1);
            break;
        default:
            break;
    }
}

// Resolve which frame to play this loop. Returns its bytes via *payload/*len; the
// result is stable for replay without the lock because the displayed frame is the
// reclaim floor (never evicted/overwritten). Drains pending NEXT ticks: each
// dequeues the next frame in commit order. Advancing past the last received frame
// empties the queue (no wrap) — returns false so the pump blanks. Returns false
// (blank) until the first NEXT arrives.
static bool resolve_active(const uint8_t **payload, uint32_t *len) {
    uint32_t adv = atomic_exchange(&s_advance, 0);
    bool ok;

    lock();
    if (adv == 0) {
        ok = frame_buffer_current(&s_fb, payload, len);      // loop current frame
    } else {
        ok = false;
        for (uint32_t k = 0; k < adv; k++) {
            ok = frame_buffer_advance(&s_fb, payload, len);  // consume forward
        }
    }
    unlock();
    return ok;
}

static void pump_task(void *arg) {
    (void)arg;
    for (;;) {
        const uint8_t *payload;
        uint32_t len;
        if (!resolve_active(&payload, &len)) {
            // Queue empty (no frame yet, or advanced past the last one). Stop
            // feeding the engine — it underruns and blanks the laser (safe).
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // Replay the (pinned) frame once, command by command.
        uint32_t off = 0;
        while (off < len) {
            laser_command_t cmd;
            uint32_t consumed;
            if (!laser_command_decode(payload + off, len - off, &cmd, &consumed)) {
                break;   // corrupt/partial record — abandon this pass
            }
            off += consumed;
            emit(&cmd);
        }
    }
}

bool frame_stream_start(uint16_t tcp_port, uint16_t udp_port) {
    if (!frame_buffer_init(&s_fb, s_arena, ARENA_BYTES, s_descs, DESC_CAP)) {
        ESP_LOGE(TAG, "frame_buffer_init failed");
        return false;
    }
    s_fb_lock = xSemaphoreCreateMutex();
    if (s_fb_lock == NULL) {
        ESP_LOGE(TAG, "mutex create failed");
        return false;
    }
    atomic_store(&s_advance, 0);

    // The pump is the single laser_engine producer; pin it to core 1 like the
    // demo renderers so it doesn't contend with the consumer ISR on core 0.
    if (xTaskCreate(tcp_task, "fs_tcp", 4096, (void *)(uintptr_t)tcp_port, 5, NULL) != pdPASS ||
        xTaskCreate(udp_task, "fs_udp", 4096, (void *)(uintptr_t)udp_port, 5, NULL) != pdPASS ||
        xTaskCreatePinnedToCore(pump_task, "fs_pump", 4096, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return false;
    }
    return true;
}
