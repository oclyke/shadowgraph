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
// the FPU is not saved. The dynamics are TICK-NATIVE (counts per tick), so the hot
// loop needs no per-tick time divide; see curve_interp.c "NUMBER FORMAT". Units:
//   - position  : DAC counts (0..65535, centre 0x8000), held in int64 for headroom
//   - speed     : counts / tick, Q(vshift) fixed-point (the `v*` state fields)
//   - accel     : counts / tick^2, Q(vshift)
//   - wire/carry: counts / tick * 256 (Q8, CURVE_WIRE_V_FRAC) — the CURVE record
//   - config    : curve_limits_t stays physical (counts/s); converted once in begin
//   - time/tick : microseconds (dt_tick_us, the tick duration)
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
// dt_tick_us is the interpolation period: one setpoint is emitted per tick from
// the gptimer ISR. Smaller ticks draw smoother but cost more ISR time; the work is
// ~1 isqrt64 + 4 isqrt32 + a couple int64 divides (geometry) + 2 galvo SPI writes,
// so the tick must stay comfortably above that. Independent of the gptimer's own
// 1 MHz resolution (used for DWELL scheduling), and above the galvo's mechanical
// bandwidth at any sane value.
#define CURVE_DEFAULT_V_MAX_CPS   ((int64_t)11468800)     // 400  * 28672
#define CURVE_DEFAULT_A_MAX_CPS2  ((int64_t)57344000000)  // 2e6  * 28672
#define CURVE_DEFAULT_DT_TICK_US  ((int32_t)20)

// The default limits as a struct. The CURVE_DEFAULT_* above are preprocessor
// macros — they don't exist as symbols at link time, so the host tool can't read
// them directly. This function exposes their values as a real callable symbol the
// host links over FFI, making curve_interp.h the single source of truth: the host
// plans/simulates against the EXACT numbers the device uses, nothing mirrored.
// (Unused by the firmware itself, which uses the macros directly.)
curve_limits_t curve_default_limits(void);

// Fractional bits of the wire/`carry` speed format: counts per tick with 8
// fractional bits (a u32 holds counts/tick * 256). This matches the MAXIMUM
// internal fractional precision (the interpolator picks F in 0..8 per curve), so
// the wire carries exactly what the firmware can represent — no more (Q16 would
// be bits the ISR immediately discards), no less — and wire->internal is a pure
// right shift of (8 - F). The interpolator is tick-native end to end (begin() in,
// carry out, and the CURVE wire record), so no physical units cross the firmware
// boundary; only the human-facing config (`curve_limits_t`, counts/s) and host
// reporting are physical, converted at the edges. See curve_interp.c.
#define CURVE_WIRE_V_FRAC 8

// Interpolation state for one in-flight CURVE. Held by value by the engine. The
// fields fall in three groups: the curve SHAPE and the tick-unit LIMITS are fixed
// by begin() for the whole curve; only the small EVOLVING group changes per step.
//
// Speed/accel are tick-native: Q(vshift) counts/tick (see curve_interp.c, "NUMBER
// FORMAT"). begin() converts the physical limits and the wire speed into that
// format ONCE; the hot loop never touches physical units. Use curve_speed_cps()
// for a counts/second readout (host reporting/tests only).
typedef struct {
    // -- shape: cubic in power basis, B(t) = ((a*t + b)*t + c)*t + d, DAC counts --
    int64_t  ax, bx, cx, dx;   // x-axis coefficients
    int64_t  ay, by, cy, dy;   // y-axis coefficients
    int64_t  S;                // total arc length, counts
    int32_t  p3x, p3y;         // exact endpoint (snap target on the final tick)

    // -- limits in tick units (begin() converts curve_limits_t + v_out wire once) --
    int32_t  vshift;           // F: fractional bits of the Q(F) counts/tick speed
    int32_t  v_max;            // speed ceiling,     Q(F) counts/tick
    int32_t  a_max;            // accel ceiling,     Q(F) counts/tick^2
    int32_t  v_out;            // target exit speed, Q(F) counts/tick

    // -- evolving: advanced one tick per curve_interp_step() --
    uint64_t t;                // current parameter, Q32 (1.0 == 1<<32)
    int64_t  s_done;           // arc length traversed so far, counts
    int32_t  v;                // current speed, Q(F) counts/tick
    bool     done;             // true once t has reached 1.0
} curve_state_t;

// Begin a curve. P0 is the segment's start (the engine's current galvo position);
// P1,P2,P3 are the control points and end. v_in/v_out are entry/exit speeds in
// WIRE units (counts per tick * 256, Q8 — exactly what the CURVE record carries
// and what curve_interp_step() hands back via carry). Computes the cubic
// coefficients and the arc length, converts the physical limits once, primes speed.
void curve_interp_begin(curve_state_t *st, const curve_limits_t *lim,
                        int32_t p0x, int32_t p0y, int32_t p1x, int32_t p1y,
                        int32_t p2x, int32_t p2y, int32_t p3x, int32_t p3y,
                        int64_t v_in_wire, int64_t v_out_wire);

// Advance one tick (dt_tick_us). Writes the next galvo setpoint to *out_x,*out_y
// (clamped to 0..65535). Returns true while the curve is still in progress; on
// the final tick it snaps to P3, returns false, and writes the exit speed (WIRE
// units, counts/tick * 256, Q8) to *carry_v_wire (hand to the next segment's v_in).
// Safe to keep calling after it returns false (it stays done, emitting P3).
bool curve_interp_step(curve_state_t *st, uint16_t *out_x, uint16_t *out_y,
                       int64_t *carry_v_wire);

// Read the current speed back in counts/second (needs the tick duration, since the
// state itself is tick-native). For HOST reporting and tests only; the firmware
// ISR never needs physical units.
int64_t curve_speed_cps(const curve_state_t *st, int32_t dt_tick_us);

// Convert a physical speed (counts/second) to the wire/`carry` format that
// curve_interp_begin() and the CURVE record expect (counts/tick * 256, Q8). The
// canonical physical->wire crossing: the host planner (over FFI) and any
// firmware-side producer share it so they cannot disagree. Not on the ISR path.
int64_t curve_cps_to_wire(int64_t v_cps, int32_t dt_tick_us);

#ifdef __cplusplus
}
#endif
