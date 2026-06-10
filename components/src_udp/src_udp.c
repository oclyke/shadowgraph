// src_udp: UDP scene-streaming renderer source.
//
// Two tasks touch the scene_stream window: the receive task (ingest) and the
// mux task (drain/skip, via pump). scene_stream is single-threaded by contract,
// so a mutex serializes them. This is millisecond-scale work, well off the
// engine's microsecond ISR path, so the lock is cheap.
#include "src_udp.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "scene_stream.h"
#include "laser_engine.h"

static const char *TAG = "src_udp";

// Reassembly window depth (commands). Power of two. ~16 B/slot -> 1024 ~= 16 KB.
#define WINDOW_CAP        1024u
// Largest datagram we accept (header + payload), kept under the SoftAP MTU.
#define RX_BUF_SIZE       1472u
// If delivery stalls at a missing command this long with data buffered ahead,
// skip the gap and resync — stale laser data is worthless in real time.
#define GAP_STALL_US      20000      // 20 ms

static scene_slot_t   s_slots[WINDOW_CAP];
static scene_stream_t s_stream;
static SemaphoreHandle_t s_lock;
static int            s_sock = -1;
static int64_t        s_last_progress_us;

// scene_stream -> laser_engine bridge. Returns false when the engine queue is
// full, which scene_stream_drain treats as backpressure (stops, retries later).
static bool emit_to_engine(const laser_command_t *c, void *ctx)
{
    (void)ctx;
    switch (c->type) {
        case LASER_CMD_GOTO:  return laser_engine_goto(c->pos.x, c->pos.y);
        case LASER_CMD_LASER: return laser_engine_laser(c->col.r, c->col.g, c->col.b);
        case LASER_CMD_DWELL: return laser_engine_dwell(c->dwell.dt);
        default:              return true;   // unknown: skip, keep delivering
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[RX_BUF_SIZE];
    for (;;) {
        int n = recv(s_sock, buf, sizeof buf, 0);
        if (n < (int)SCENE_PACKET_HDR_SIZE) {
            if (n < 0) vTaskDelay(pdMS_TO_TICKS(10));   // socket error: back off
            continue;
        }
        scene_packet_hdr_t hdr;
        const uint8_t *payload = NULL;
        uint32_t plen = 0;
        if (!scene_packet_parse(buf, (uint32_t)n, &hdr, &payload, &plen)) {
            continue;   // foreign/garbage datagram
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        scene_stream_ingest(&s_stream, &hdr, payload, plen);
        xSemaphoreGive(s_lock);
    }
}

// --- renderer_source_t vtable ----------------------------------------------
static void udp_start(void *ctx)
{
    (void)ctx;
    // Discard anything buffered while we were inactive and re-lock to the next
    // fresh packet, so switching in doesn't replay a stale scene.
    xSemaphoreTake(s_lock, portMAX_DELAY);
    scene_stream_reset(&s_stream);
    xSemaphoreGive(s_lock);
    s_last_progress_us = esp_timer_get_time();
}

static void udp_pump(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t delivered = scene_stream_drain(&s_stream, emit_to_engine, NULL);
    if (delivered == 0) {
        // No progress: if we're wedged behind a lost command with newer data
        // already buffered, give the resend a grace period, then skip it.
        int64_t now = esp_timer_get_time();
        if (scene_stream_stalled_at_gap(&s_stream) &&
            now - s_last_progress_us > GAP_STALL_US) {
            uint32_t skipped = scene_stream_skip_gap(&s_stream);
            if (skipped) {
                ESP_LOGW(TAG, "skipped %u lost command(s)", (unsigned)skipped);
                s_last_progress_us = now;
            }
        }
    } else {
        s_last_progress_us = esp_timer_get_time();
    }
    xSemaphoreGive(s_lock);

    // Yield when idle so the mux task doesn't spin; run flat out while there is
    // a backlog to deliver.
    if (delivered == 0) {
        vTaskDelay(1);
    }
}

static const renderer_source_t s_src = {
    .name  = "udp",
    .start = udp_start,
    .stop  = NULL,
    .pump  = udp_pump,
    .ctx   = NULL,
};

const renderer_source_t *src_udp_init(uint16_t port)
{
    if (!scene_stream_init(&s_stream, s_slots, WINDOW_CAP)) {
        ESP_LOGE(TAG, "scene_stream_init failed");
        return NULL;
    }
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "mutex alloc failed");
        return NULL;
    }

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return NULL;
    }
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr *)&addr, sizeof addr) < 0) {
        ESP_LOGE(TAG, "bind(%u) failed: errno %d", port, errno);
        close(s_sock);
        s_sock = -1;
        return NULL;
    }

    if (xTaskCreate(rx_task, "udp_rx", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "rx task create failed");
        close(s_sock);
        s_sock = -1;
        return NULL;
    }

    ESP_LOGI(TAG, "listening on udp/%u, window=%u commands", port, WINDOW_CAP);
    return &s_src;
}
