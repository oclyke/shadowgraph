#pragma once

#include <stdbool.h>

// WiFi bring-up for networked streaming, in either of two roles (fixed
// credentials live in wifi_ap.c):
//   - SoftAP  (wifi_ap_start):  the device hosts its own network; the host joins
//                               it and streams to the device's fixed IP.
//   - Station (wifi_sta_start): the device joins an existing network and gets a
//                               DHCP address; host + device share that LAN.
//
// Init order: each owns NVS, the default event loop, and esp_netif bring-up, so
// call exactly ONE of them once early in app_main before any other networking.
// Safe to call alongside the laser engine — WiFi runs on its own task.

// Start the access point. Initializes NVS / netif / event loop as needed and
// brings up the SoftAP. Returns false on any setup failure (already logged).
bool wifi_ap_start(void);

// Start station mode: associate with the fixed SSID/password and obtain an IP by
// DHCP. The assigned address is logged on connect — use it as the stream target.
// Returns false on a setup failure (association happens asynchronously after).
bool wifi_sta_start(void);
