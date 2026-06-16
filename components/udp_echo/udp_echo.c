// Dead-simple UDP echo server: bind a socket, recvfrom, sendto the same bytes
// back to whoever sent them. This is the smallest possible proof that the host
// can talk to this device over WiFi — no framing, no protocol, just an echo.
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "udp_echo.h"

static const char *TAG = "udp_echo";

// Max datagram we'll accept in one recvfrom. "Hello World" is tiny; this leaves
// generous room without committing much stack.
#define UDP_ECHO_BUF_LEN 512

static void udp_echo_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;

    // IPv4 UDP socket bound to every local interface on `port`.
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() to port %u failed: errno %d", port, errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "listening for UDP on port %u", port);

    char buf[UDP_ECHO_BUF_LEN];
    for (;;) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &src_len);
        if (n < 0) {
            ESP_LOGE(TAG, "recvfrom() failed: errno %d", errno);
            continue;
        }

        // Log what we got (NUL-terminate so %s is safe) and who from.
        buf[n] = '\0';
        char src_ip[INET_ADDRSTRLEN];
        inet_ntoa_r(src.sin_addr, src_ip, sizeof(src_ip));
        ESP_LOGI(TAG, "rx %d bytes from %s:%u: \"%s\"",
                 n, src_ip, ntohs(src.sin_port), buf);

        // Echo the exact bytes back to the sender.
        int sent = sendto(sock, buf, n, 0,
                          (struct sockaddr *)&src, src_len);
        if (sent < 0) {
            ESP_LOGE(TAG, "sendto() failed: errno %d", errno);
        }
    }
}

bool udp_echo_start(uint16_t port)
{
    BaseType_t ok = xTaskCreate(udp_echo_task, "udp_echo", 4096,
                                (void *)(uintptr_t)port, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create udp_echo task");
        return false;
    }
    return true;
}
