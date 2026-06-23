#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Art-Net control input: a remote console (or the tools/artnetctl CLI) drives the
// device over Art-Net (DMX-over-UDP, port 6454). One DMX universe carries a small
// fixture that selects the render mode and tweaks the built-in Lissajous pattern.
//
// Two planes meet here: Art-Net is a *control* plane (pick the mode + pattern
// settings); the actual show geometry still arrives on the point_stream TCP path
// when the mode is set to STREAM. Channel 1 toggles between the two.
//
// Layout split: the packet parse and the DMX->state decode are portable C (host
// unit-tested). The UDP receiver task, the shared-state store, and artnet_control_get
// are device-only (lwIP / FreeRTOS), compiled under ESP_PLATFORM.

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// DMX channel map (1-based, relative to the patched base channel)
// ---------------------------------------------------------------------------
//   1  Mode          < 128 = pattern (Lissajous), >= 128 = stream
//   2  Lissajous freq X   -> integer ratio 1..8
//   3  Lissajous freq Y   -> integer ratio 1..8
//   4  Size              -> 0 .. ARTNET_AMP_MAX (galvo linear range)
//   5  Hue               -> 0 .. 360 deg
//   6  Intensity         -> 0 .. 1 (beam brightness)
//   7  Morph rate        -> 0 .. ARTNET_MORPH_MAX_RAD_S (y-phase precession)
//   8  Phase offset      -> 0 .. 2*pi (static y-phase)
#define ARTNET_NUM_CHANNELS     8

#define ARTNET_FREQ_MIN         1u
#define ARTNET_FREQ_MAX         8u
// Max Lissajous amplitude in ILDA units (~20% of full scale, where the galvos
// stay linear; matches the original demo's GALVO_AMPLITUDE).
#define ARTNET_AMP_MAX          13107
// Max y-phase precession rate the morph channel maps to.
#define ARTNET_MORPH_MAX_RAD_S  3.0f

typedef enum {
    ARTNET_MODE_PATTERN = 0,   // trace the built-in Lissajous from the DMX settings
    ARTNET_MODE_STREAM  = 1,   // loop whatever scene point_stream last received
} artnet_mode_t;

// Decoded control state in physical units, ready for the renderer to consume.
typedef struct {
    artnet_mode_t mode;
    uint8_t       fx;          // Lissajous X frequency ratio (1..8)
    uint8_t       fy;          // Lissajous Y frequency ratio (1..8)
    int16_t       amplitude;   // figure size in ILDA units (0..ARTNET_AMP_MAX)
    float         hue_deg;     // color hue, 0..360
    float         intensity;   // beam brightness, 0..1
    float         morph_rate;  // y-phase precession, rad/s
    float         phase_off;   // static y-phase offset, rad
} artnet_control_state_t;

// ---------------------------------------------------------------------------
// Portable helpers (host-testable)
// ---------------------------------------------------------------------------

// Parse an Art-Net ArtDmx packet. On success sets *universe (15-bit Net:SubUni
// port address), *dmx (a pointer into pkt), *dmxlen (number of DMX slots, 1..512)
// and returns true. Returns false for any non-ArtDmx or malformed packet.
bool artnet_parse(const uint8_t *pkt, size_t len,
                  uint16_t *universe, const uint8_t **dmx, uint16_t *dmxlen);

// Map a DMX frame to control state. `base_channel` is the 1-based slot of the
// fixture's first channel; channels beyond `len` read as 0. `out` is fully
// populated (in physical units) on every call.
void artnet_control_decode(const uint8_t *dmx, uint16_t len,
                           uint16_t base_channel, artnet_control_state_t *out);

// Fill `out` with the offline defaults: pattern mode, a 3:2 Lissajous at ~20%
// size, hue 0, 25% intensity, a gentle morph. Used until the first DMX frame
// arrives (and as the no-network fallback).
void artnet_control_defaults(artnet_control_state_t *out);

// ---------------------------------------------------------------------------
// Device runtime (ESP_PLATFORM only)
// ---------------------------------------------------------------------------

// Seed the shared control state with `defaults` and record which universe /
// base channel the receiver should honor. Call once before artnet_control_start
// and before the renderer first calls artnet_control_get.
void artnet_control_init(const artnet_control_state_t *defaults,
                         uint16_t universe, uint16_t base_channel);

// Start the Art-Net receiver task (UDP 6454, broadcast + unicast). Returns false
// on a socket setup failure. (Device build only.)
bool artnet_control_start(void);

// Snapshot the latest control state (thread-safe). Returns the seeded defaults
// until the first matching DMX frame arrives.
void artnet_control_get(artnet_control_state_t *out);

#ifdef __cplusplus
}
#endif
