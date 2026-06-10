#pragma once

#include <stdbool.h>

// Minimal SoftAP bring-up for networked streaming. Brings the WiFi stack up in
// access-point mode with a fixed SSID/password (see wifi_ap.c). This is the
// first piece of the streaming path: it only stands up the AP so clients can
// associate; the stream transport lands on top later.
//
// Init order: this owns NVS, the default event loop, and esp_netif bring-up,
// so call it once early in app_main before any other networking. Safe to call
// alongside the laser engine — WiFi runs on its own task.

// Start the access point. Initializes NVS / netif / event loop as needed and
// brings up the SoftAP. Returns false on any setup failure (already logged).
bool wifi_ap_start(void);
