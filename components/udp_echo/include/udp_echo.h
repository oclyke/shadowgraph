#pragma once

#include <stdbool.h>
#include <stdint.h>

// Dead-simple UDP echo server. Spawns a background task that binds a UDP socket
// on the given port (on all interfaces) and echoes every datagram it receives
// straight back to the sender. Intended as the first end-to-end check that the
// host can reach this device over the WiFi link — send "Hello World" from the
// host and get it back.
//
// Call after networking is up (e.g. after wifi_sta_start). Returns true if the
// task was created. The task owns the socket and runs for the life of the app.
bool udp_echo_start(uint16_t port);
