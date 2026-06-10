#pragma once

#include <stdint.h>
#include "renderer.h"

// src_udp: a renderer source that plays a scene streamed over UDP. A receive
// task binds `port` and feeds incoming datagrams into a scene_stream reassembly
// window (reorder + dedup + blind-resend tolerance); the source's pump() drains
// the recovered commands into the laser engine, applying backpressure and
// skipping unrecoverable gaps after a stall so the stream stays live.
//
// Call once at startup (after the network interface is up). Binds the socket,
// starts the receive task, and returns the source vtable to register with the
// renderer. Returns NULL on a socket/setup failure.
const renderer_source_t *src_udp_init(uint16_t port);
