// SoftAP bring-up for networked streaming. Stands up a fixed-credential WiFi
// access point so clients can associate; the stream transport lands on top
// later. Credentials are intentionally in plain text — this AP is not a
// security boundary, it's a transport for laser frames on a local link.
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "wifi_ap.h"

static const char *TAG = "wifi_ap";

// Plain-text on purpose: not security sensitive (see header). WPA2 requires a
// >= 8 char password; a shorter WIFI_AP_PASS makes the AP fall back to open
// (no password) with a warning.
#define WIFI_AP_SSID     "shadowgraph"
#define WIFI_AP_PASS     "letslaser"
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_MAX_CONN 4

// Station credentials: the existing network the device joins in STA mode.
#define WIFI_STA_SSID    "ioio"
#define WIFI_STA_PASS    "spicygreen"

// WPA2-PSK needs at least 8 password chars; below that the driver rejects the
// config, so we run the AP open instead.
#define WPA2_MIN_PASS_LEN 8

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " joined, aid=%d", MAC2STR(e->mac), e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " left, aid=%d", MAC2STR(e->mac), e->aid);
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

bool wifi_ap_start(void)
{
    esp_err_t err = nvs_ready();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .password       = WIFI_AP_PASS,
            .channel        = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };

    // Honor the configured password only if it's long enough for WPA2;
    // otherwise fall back to an open network so the AP still comes up.
    if (strlen(WIFI_AP_PASS) < WPA2_MIN_PASS_LEN) {
        ESP_LOGW(TAG, "password \"%s\" is < %d chars; starting OPEN (no encryption)",
                 WIFI_AP_PASS, WPA2_MIN_PASS_LEN);
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
        ap_cfg.ap.password[0] = '\0';
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP up: ssid=\"%s\" channel=%d auth=%s",
             WIFI_AP_SSID, WIFI_AP_CHANNEL,
             ap_cfg.ap.authmode == WIFI_AUTH_OPEN ? "open" : "wpa2-psk");
    return true;
}

// STA: kick off association when the stack starts, and keep retrying if the link
// drops (the device should rejoin "ioio" on its own after an AP blip).
static void on_sta_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    if (id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "disconnected from \"%s\"; retrying", WIFI_STA_SSID);
        esp_wifi_connect();
    }
}

// STA got a DHCP lease — log it; this is the address to point the host at.
static void on_sta_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id;
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "STA got IP " IPSTR " — stream target", IP2STR(&e->ip_info.ip));
}

bool wifi_sta_start(void)
{
    esp_err_t err = nvs_ready();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_sta_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_sta_got_ip, NULL, NULL));

    wifi_config_t sta_cfg = {
        .sta = {
            .ssid              = WIFI_STA_SSID,
            .password          = WIFI_STA_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());   // -> STA_START -> esp_wifi_connect()

    ESP_LOGI(TAG, "STA connecting to ssid=\"%s\" (IP logged on join)", WIFI_STA_SSID);
    return true;
}
