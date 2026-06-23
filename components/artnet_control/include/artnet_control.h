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
//   2  Lissajous freq X   -> base integer ratio 1..8
//   3  Lissajous freq Y   -> base integer ratio 1..8
//   4  Size              -> 0 .. ARTNET_AMP_MAX (galvo linear range)
//   5  Hue               -> 0 .. 360 deg (base color)
//   6  Intensity         -> 0 .. 1 (beam brightness)
//   7  Morph rate        -> 0 .. ARTNET_MORPH_MAX_RAD_S (y-phase precession)
//   8  Phase offset      -> 0 .. 2*pi (static y-phase)
//   9  Blank width       -> 0 .. ARTNET_BLANK_WIDTH_MAX: size of a sliding gap
//                           where the beam blanks, as a fraction of the loop
//                           (0 = no blanking). The "aliasing" artifact.
//  10  Blank slide rate  -> 0 .. ARTNET_BLANK_SLIDE_MAX: loop-fractions/sec the
//                           blank gap slides through the parametric domain t
//  11  Color-vs-t span   -> 0 .. ARTNET_COLOR_SPAN_MAX deg of hue spread ALONG
//                           the curve (0 = one constant color)
//  12  Color cycle rate  -> 0 .. ARTNET_COLOR_CYCLE_MAX deg/sec the color rotates
//                           over time (independent of the y-phase morph)
//  13  Freq X morph rate -> 0 .. ARTNET_FREQ_MORPH_RATE_MAX Hz: fx oscillates
//  14  Freq Y morph rate -> 0 .. ARTNET_FREQ_MORPH_RATE_MAX Hz: fy oscillates
//  15  Freq morph depth  -> 0 .. ARTNET_FREQ_MORPH_DEPTH_MAX: +/- added to fx/fy
#define ARTNET_NUM_CHANNELS     15

#define ARTNET_FREQ_MIN         1u
#define ARTNET_FREQ_MAX         8u
// Max Lissajous amplitude in ILDA units (~20% of full scale, where the galvos
// stay linear; matches the original demo's GALVO_AMPLITUDE).
#define ARTNET_AMP_MAX          13107
// Max y-phase precession rate the morph channel maps to.
#define ARTNET_MORPH_MAX_RAD_S      3.0f
#define ARTNET_BLANK_WIDTH_MAX      0.95f   // never blank the entire loop
#define ARTNET_BLANK_SLIDE_MAX      2.0f    // loop traversals / sec
#define ARTNET_COLOR_SPAN_MAX       720.0f  // up to two full hue wheels across t
#define ARTNET_COLOR_CYCLE_MAX      180.0f  // hue deg / sec over time
#define ARTNET_FREQ_MORPH_RATE_MAX  1.0f    // Hz
#define ARTNET_FREQ_MORPH_DEPTH_MAX 4.0f    // +/- added to base fx/fy

typedef enum {
    ARTNET_MODE_PATTERN = 0,   // trace the built-in Lissajous from the DMX settings
    ARTNET_MODE_STREAM  = 1,   // loop whatever scene point_stream last received
} artnet_mode_t;

// Decoded control state in physical units, ready for the renderer to consume.
typedef struct {
    artnet_mode_t mode;
    uint8_t       fx;          // Lissajous X base frequency ratio (1..8)
    uint8_t       fy;          // Lissajous Y base frequency ratio (1..8)
    int16_t       amplitude;   // figure size in ILDA units (0..ARTNET_AMP_MAX)
    float         hue_deg;     // base color hue, 0..360
    float         intensity;   // beam brightness, 0..1
    float         morph_rate;  // y-phase precession, rad/s
    float         phase_off;   // static y-phase offset, rad
    float         blank_width;      // sliding blank gap size, fraction of loop (0 = off)
    float         blank_slide_rate; // loop-fractions/sec the gap slides
    float         color_t_span;     // hue deg spread along the curve (0 = constant)
    float         color_cycle_rate; // hue deg/sec rotation over time
    float         fx_morph_rate;    // Hz of fx oscillation
    float         fy_morph_rate;    // Hz of fy oscillation
    float         freq_morph_depth; // +/- added to fx/fy at full morph
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

// --- Discovery (ArtPoll / ArtPollReply) ------------------------------------
// A controller broadcasts an ArtPoll; every node answers with an ArtPollReply
// carrying its IP + name, so the controller learns where to send. The device
// answers ArtPolls on its existing :6454 socket (no multicast). See the matching
// query/reply helpers in tools/artnetctl.

// An ArtPollReply is a fixed-size packet.
#define ARTNET_POLLREPLY_SIZE 239

// True if `pkt` is a valid Art-Net ArtPoll (the discovery query, OpCode 0x2000).
bool artnet_is_poll(const uint8_t *pkt, size_t len);

// Build an ArtPollReply describing this node into `out` (needs cap >=
// ARTNET_POLLREPLY_SIZE). `ip` is the node's IPv4 as four octets a.b.c.d;
// `universe` is the 15-bit port address it listens on; `short_name` (<= 17 chars
// shown) and `long_name` (<= 63) are copied into the name fields a controller
// displays. Returns bytes written, or 0 if `cap` is too small.
size_t artnet_pollreply_build(uint8_t *out, size_t cap, const uint8_t ip[4],
                              uint16_t universe, const char *short_name,
                              const char *long_name);

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
