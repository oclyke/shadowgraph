#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "byte_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Type-Value command encoding over a byte_queue. The length of each record is
// implied by its type byte (producer and consumer are the same firmware), so
// no length field is stored. Payloads are little-endian.
typedef enum {
    LASER_CMD_GOTO  = 0x01,   // x:u16, y:u16     — set galvo position (instantaneous)
    LASER_CMD_LASER = 0x02,   // r:u16, g:u16, b  — set laser color    (instantaneous)
    LASER_CMD_DWELL = 0x03,   // dt:u32 ticks     — advance time
    LASER_CMD_CURVE = 0x04,   // cubic Bézier move with entry/exit speed (see below)
} laser_command_type_t;

// LASER_CMD_CURVE: a cubic Bézier segment the engine interpolates in real time.
//   P0 is IMPLICIT — the engine's current galvo position (end of the previous
//   command) — so only P1,P2,P3 ride the wire and C0 continuity is automatic.
//   v_in/v_out are the entry/exit tangential speeds in DAC counts per second; the
//   host's global look-ahead guarantees the pair is reachable over the segment.
//   Colour is NOT carried here: a preceding LASER sets it and it is held (a blank
//   travel move is LASER 0,0,0 then a straight CURVE). See docs/CURVE_MOTION.md.
typedef struct {
    laser_command_type_t type;
    union {
        struct { uint16_t x, y; }    pos;     // LASER_CMD_GOTO
        struct { uint16_t r, g, b; } col;     // LASER_CMD_LASER
        struct { uint32_t dt; }      dwell;   // LASER_CMD_DWELL
        struct {                              // LASER_CMD_CURVE
            uint16_t x1, y1;                  //   P1 (first control point)
            uint16_t x2, y2;                  //   P2 (second control point)
            uint16_t x3, y3;                  //   P3 (end point = next P0)
            uint32_t v_in, v_out;             //   entry/exit speed, counts/second
        } curve;
    };
} laser_command_t;

// Encoded record size including the 1-byte type tag, or 0 for an unknown type.
uint32_t laser_command_size(laser_command_type_t type);

// Producer side: serialize and atomically enqueue. Returns false if the queue
// has insufficient room (the queue is left unchanged).
bool laser_command_push_goto (byte_queue_t *q, uint16_t x, uint16_t y);
bool laser_command_push_laser(byte_queue_t *q, uint16_t r, uint16_t g, uint16_t b);
bool laser_command_push_dwell(byte_queue_t *q, uint32_t dt);
bool laser_command_push_curve(byte_queue_t *q,
                              uint16_t x1, uint16_t y1,
                              uint16_t x2, uint16_t y2,
                              uint16_t x3, uint16_t y3,
                              uint32_t v_in, uint32_t v_out);

// Consumer side: decode one command into *out. Returns false if the queue is
// empty or the next record has an unknown type (caller decides how to recover).
bool laser_command_pop(byte_queue_t *q, laser_command_t *out);

#ifdef __cplusplus
}
#endif
