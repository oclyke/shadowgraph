#pragma once

#include <stdbool.h>

// Minimal WiFi station bring-up for networked streaming. Brings the WiFi stack
// up in station mode and joins the caller-supplied SSID/password so this device
// gets an IP on the same link as the host. This is the first piece of the
// streaming path: it only establishes the link; the stream transport lands on
// top later.
//
// Init order: this owns NVS, the default event loop, and esp_netif bring-up,
// so call it once early in app_main before any other networking. Safe to call
// alongside the laser engine — WiFi runs on its own task.

// Connect to the access point named by ssid (WPA2-PSK, pass). Initializes NVS /
// netif / event loop as needed and brings up the station. Blocks briefly for an
// IP; returns true once connected, false on timeout or setup failure (the
// driver keeps retrying in the background regardless). Failures are already
// logged.
bool wifi_sta_start(const char *ssid, const char *pass);
