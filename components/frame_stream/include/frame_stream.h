#pragma once

#include <stdint.h>
#include <stdbool.h>

// frame_stream: the networked animation playout path. Brings up a TCP server
// (frame data plane) and a UDP listener (playout-clock control plane) and a pump
// task that loops the active frame into the laser_engine. See
// docs/FRAME_STREAMING.md for the full contract.
//
//   - TCP (tcp_port): the host streams length-prefixed frames; each frame is a
//     blob of laser_command bytes. The recv task is the SOLE writer of the frame
//     arena (reserve/commit). TCP flow control provides backpressure for free.
//     After committing a frame the device sends a 1-byte ack back on the TCP
//     connection, so the host can wait for the commit before ticking NEXT (a UDP
//     tick must not race ahead of the frame's bytes).
//   - UDP (udp_port): the host sends NEXT ticks to advance the playout cursor one
//     frame (relative — the host stays stateless and can't desync; a lost tick
//     just lingers one extra frame). The cursor wraps past the newest, so a
//     resident set loops.
//   - Pump: replays the active frame on repeat into laser_engine, advancing the
//     cursor at each loop boundary so switches are seamless. The pump is the
//     single producer to laser_engine (preserving its SPSC contract).
//
// Call after laser_engine_start() and after the network is up. The pump is the
// only laser_engine producer, so do not also run another renderer task.
bool frame_stream_start(uint16_t tcp_port, uint16_t udp_port);
