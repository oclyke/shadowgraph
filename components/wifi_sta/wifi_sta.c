// WiFi station bring-up for networked streaming. Joins a fixed-credential
// access point (a phone hotspot, here) so this device gets an IP on the same
// link as the host running UDP tests. Credentials are intentionally in plain
// text — this is a transport for laser frames on a local link, not a security
// boundary.
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "wifi_sta.h"

static const char *TAG = "wifi_sta";

// How long wifi_sta_start() blocks waiting for an association + IP before
// returning. The driver keeps retrying in the background after this returns.
#define WIFI_CONNECT_TIMEOUT_MS 15000

// Set by the IP event handler once we have an address.
#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_events;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Failed to associate, or lost the link — keep retrying so the device
        // rejoins when the hotspot is in range again. The reason code says why:
        // e.g. 201 NO_AP_FOUND (SSID not seen — off, asleep, or 5 GHz-only),
        // 15 4WAY_HANDSHAKE_TIMEOUT / 2 AUTH_EXPIRE (usually a bad password).
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "disconnected (reason=%d); retrying", e->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

// Bring NVS up; esp_wifi stores calibration/PHY data there and won't init
// without it. Recover from a stale/too-new partition by erasing once.
static esp_err_t nvs_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

bool wifi_sta_start(const char *ssid, const char *pass)
{
    esp_err_t err = nvs_ready();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t sta_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    // Copy caller-supplied credentials into the driver's fixed-size fields.
    strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to ssid=\"%s\"", ssid);

    // Block until we get an IP or time out. The event handler keeps retrying
    // association in the background either way, so a timeout here is not fatal.
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to \"%s\"", ssid);
        return true;
    }

    ESP_LOGW(TAG, "not connected yet to \"%s\"; retrying in background", ssid);
    return false;
}
