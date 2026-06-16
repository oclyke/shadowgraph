#pragma once

#include <stdint.h>
#include <stdbool.h>

// Real-time cubic-Bézier motion interpolator (the firmware side of LASER_CMD_CURVE).
//
// This is the single source of truth for "what does a CURVE actually draw". The
// firmware calls it from the gptimer ISR (one setpoint per tick); the host tool
// links the SAME translation unit over FFI to simulate the device exactly. There
// is no second implementation to drift. See docs/CURVE_MOTION.md.
//
// All math is INTEGER fixed-point (no float): the consumer runs in an ISR where
// the FPU is not saved. Units, throughout:
//   - position  : DAC counts (0..65535, centre 0x8000), held in int64 for headroom
//   - speed     : counts / second           (int64)
//   - accel     : counts / second^2         (int64)
//   - time/tick : microseconds              (int32)
//   - parameter : t in Q32 fixed-point, where 1.0 == (1<<32)
//
// The interpolator rides the friction circle (|a| = sqrt(a_t^2 + a_n^2) <= a_max):
// it accelerates as hard as the leftover budget after the centripetal cost of the
// turn allows, while braking in time to hit v_out and never exceeding the local
// curvature ceiling. The host's global look-ahead guarantees the (v_in, v_out)
// pair is reachable over the segment, so the firmware needs no multi-segment
// look-ahead — it brakes locally to the single v_out it was handed.

#ifdef __cplusplus
extern "C" {
#endif

// Immutable galvo limits. Shared config: the host CLI mirrors these and a test
// asserts the defaults below match what the host plans against.
typedef struct {
    int64_t v_max_cps;     // max scan speed, counts/second
    int64_t a_max_cps2;    // max acceleration, counts/second^2
    int32_t dt_tick_us;    // interpolation period (one ISR tick), microseconds
} curve_limits_t;

// Placeholder defaults (field v_max=400, a_max=2e6 units/s^2, amplitude 0x7000 =>
// 1 field unit = 28672 counts). Replace v_max/a_max with the real galvo
// datasheet numbers; host and firmware must agree.
//
// dt_tick = 20 us (50 kHz interpolation). Chosen for ISR headroom: one setpoint
// is ~8-12 us of work (3x isqrt + int64 divides + 2 galvo SPI writes), so 20 us
// leaves margin, and 50 kHz is still far above the galvo's mechanical bandwidth.
// The gptimer runs at 1 MHz for DWELL scheduling resolution — independent of this.
#define CURVE_DEFAULT_V_MAX_CPS   ((int64_t)11468800)     // 400  * 28672
#define CURVE_DEFAULT_A_MAX_CPS2  ((int64_t)57344000000)  // 2e6  * 28672
#define CURVE_DEFAULT_DT_TICK_US  ((int32_t)20)

// Interpolation state for one in-flight CURVE. Opaque in spirit, but exposed so
// the engine can hold one by value. Read `v` for the current speed (tests do).
typedef struct {
    // Expanded cubic coefficients per axis: B(t) = ((a*t + b)*t + c)*t + d, counts.
    int64_t ax, bx, cx, dx;
    int64_t ay, by, cy, dy;
    int64_t  S;            // total arc length, counts
    int64_t  s_done;       // arc length traversed so far, counts
    int64_t  v;            // current speed, counts/second
    int64_t  v_out;        // target exit speed, counts/second
    uint64_t t;            // current parameter, Q32 (1.0 == 1<<32)
    int32_t  p3x, p3y;     // endpoint, for an exact snap on completion
    curve_limits_t lim;    // limits copy
    bool     done;         // true once t has reached 1.0
} curve_state_t;

// Begin a curve. P0 is the segment's start (the engine's current galvo position);
// P1,P2,P3 are the control points and end. v_in/v_out are entry/exit speeds in
// counts/second (as carried on the wire). Computes the cubic coefficients and the
// arc length, and primes the speed to v_in.
void curve_interp_begin(curve_state_t *st, const curve_limits_t *lim,
                        int32_t p0x, int32_t p0y, int32_t p1x, int32_t p1y,
                        int32_t p2x, int32_t p2y, int32_t p3x, int32_t p3y,
                        int64_t v_in_cps, int64_t v_out_cps);

// Advance one tick (dt_tick_us). Writes the next galvo setpoint to *out_x,*out_y
// (clamped to 0..65535). Returns true while the curve is still in progress; on
// the final tick it snaps to P3, returns false, and writes the exit speed to
// *carry_v_cps (hand to the next segment's v_in). Safe to keep calling after it
// returns false (it stays done, emitting P3).
bool curve_interp_step(curve_state_t *st, uint16_t *out_x, uint16_t *out_y,
                       int64_t *carry_v_cps);

#ifdef __cplusplus
}
#endif
