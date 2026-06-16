#include "curve_interp.h"

// Integer fixed-point cubic-Bézier interpolator. No float (runs in the ISR).
// See curve_interp.h for the unit conventions and docs/CURVE_MOTION.md for the
// motion model.
//
// PERFORMANCE / NUMBER FORMAT
// ---------------------------
// The ESP32 (Xtensa LX6) has no hardware 64-bit divide and no 64-bit sqrt: a
// 64-bit `/` is a multi-hundred-cycle software routine (__udivdi3) and a
// bit-by-bit isqrt64 runs 32 iterations on 64-bit operands. The naive hot loop
// would do ~8 such divides and 5 isqrt64 per tick — by far the dominant ISR cost.
//
// The dynamics are therefore carried in (counts, ticks), NOT (counts, seconds):
//
//     velocity  v : counts / tick,   Q(F) fixed-point  (F = st->vshift)
//     accel     a : counts / tick^2, Q(F) fixed-point
//
// One step is exactly one tick, so every per-tick `* dt_us / 1000000` collapses
// to a shift — all five of those 64-bit divides vanish. v_max in tick units is
// only a few hundred counts/tick, so F is chosen in begin() to keep v below 2^15:
// v^2 and the kinematic radicands (v^2 + 2*a*s) fit in int32, turning four of the
// five isqrt64 into isqrt32 and the friction-circle divide into a 32-bit divide.
//
// UNITS ARE TICK-NATIVE END TO END. The interpolator never converts to physical
// units on the hot path:
//   - in  : begin() takes v_in/v_out in WIRE units (counts/tick * 256, Q8) — the
//           exact CURVE-record format — and only RESCALES Q8 -> Q(F) (a shift).
//   - out : step() hands the exit speed back in the same wire units via carry.
//   - cfg : the physical limits (curve_limits_t, counts/s) are the one human-facing
//           input; begin() converts them to Q(F) ONCE per curve (off the hot loop).
//   - host: curve_speed_cps() converts a speed back to counts/s for reporting and
//           tests. The firmware ISR never calls it.
//
// The CURVE GEOMETRY (B'(t), the curvature radius R = |B'|^3 / |B' x B''|, and the
// arc length) is intrinsically wide — |B'| reaches ~1.8e6 counts per unit-t — and
// is independent of the time unit, so it stays in 64-bit. It is only a handful of
// ops per tick. Worst-case magnitudes are annotated so the int64 headroom
// (max 9.22e18) stays auditable.

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

#define Q32_ONE   ((uint64_t)1 << 32)   // t == 1.0
#define N_ARCLEN  16                     // arc-length integration samples
#define R_MAX     ((int64_t)1000000)     // radius clamp: above this, curvature
                                         // never limits below v_max (v_max^2/a_max
                                         // ~ 2300 counts), so treat as straight.
#define B1_CAP    ((int64_t)2000000)     // |B'| clamp before cubing (2e6^3 = 8e18
                                         // < int64 max); only bites degenerate nets.

#define VI_MAX     32767                 // ceiling for v (Q(F) counts/tick): keeps
                                         // v^2 < 2^30 so all dynamics fit int32.
                                         // Implies a supported tick range: begin()
                                         // clamps if v_max_cps*dt_us/1e6 > VI_MAX
                                         // (i.e. dt beyond a few ms at field speeds).
#define VSHIFT_MAX 8                     // cap on F so a_max<<F and R<<F stay int32

// Exact integer sqrt of a uint64 (floor). Bit-by-bit, no float. Used only for the
// wide geometry term |B'| now (1 call/tick instead of 5).
static uint64_t IRAM_ATTR isqrt64(uint64_t x) {
    uint64_t res = 0;
    uint64_t bit = (uint64_t)1 << 62;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else                {                  res >>= 1; }
        bit >>= 2;
    }
    return res;
}

// Exact integer sqrt of a uint32 (floor). Half the iterations and all 32-bit ops:
// this carries the four kinematic roots (curvature/brake/accel/friction ceilings),
// whose radicands are kept < 2^31 by the Q(F) tick format.
static uint32_t IRAM_ATTR isqrt32(uint32_t x) {
    uint32_t res = 0;
    uint32_t bit = (uint32_t)1 << 30;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else                {                 res >>= 1; }
        bit >>= 2;
    }
    return res;
}

// Evaluate B(t) on one axis via Horner. coeffs and t (Q32) -> counts.
// acc*t: |acc| ~ <=5e5, t < 4.3e9  ->  ~2.2e15, fits int64. The `>>32` makes each
// product a single Xtensa signed high-multiply (MULSH), so these stay cheap.
static inline int64_t IRAM_ATTR eval_pos(int64_t a, int64_t b, int64_t c,
                                         int64_t d, uint64_t t) {
    int64_t acc = a;
    acc = ((acc * (int64_t)t) >> 32) + b;
    acc = ((acc * (int64_t)t) >> 32) + c;
    acc = ((acc * (int64_t)t) >> 32) + d;
    return acc;
}

// First derivative B'(t) = 3a t^2 + 2b t + c (Horner). counts per unit-t.
static inline int64_t IRAM_ATTR eval_deriv(int64_t a, int64_t b, int64_t c,
                                           uint64_t t) {
    int64_t acc = 3 * a;
    acc = ((acc * (int64_t)t) >> 32) + 2 * b;
    acc = ((acc * (int64_t)t) >> 32) + c;
    return acc;
}

// Second derivative B''(t) = 6a t + 2b. counts per unit-t^2.
static inline int64_t IRAM_ATTR eval_deriv2(int64_t a, int64_t b, uint64_t t) {
    int64_t acc = 6 * a;
    acc = ((acc * (int64_t)t) >> 32) + 2 * b;
    return acc;
}

static inline int64_t iabs64(int64_t x) { return x < 0 ? -x : x; }

// WIRE (counts/tick, 8 frac bits) <-> internal Q(F). Just a shift: F in 0..8 ==
// CURVE_WIRE_V_FRAC, so sh = 8-F is in 0..8 (a right shift, never amplifying).
static inline int32_t wire_to_vi(int64_t w, int32_t F) {
    int32_t sh = CURVE_WIRE_V_FRAC - F;               // 0..8
    int64_t v = sh > 0 ? (w + ((int64_t)1 << (sh - 1))) >> sh : w;  // round to nearest
    if (v < 0) v = 0;
    return (int32_t)v;
}
static inline int64_t IRAM_ATTR vi_to_wire(int32_t v, int32_t F) {
    return (int64_t)v << (CURVE_WIRE_V_FRAC - F);
}

curve_limits_t curve_default_limits(void) {
    curve_limits_t l = {
        CURVE_DEFAULT_V_MAX_CPS,
        CURVE_DEFAULT_A_MAX_CPS2,
        CURVE_DEFAULT_DT_TICK_US,
    };
    return l;
}

// clamp(cps * mul / div, 1, VI_MAX), overflow-safe. The physical limit can be far
// larger than the tick format can hold (huge v_max/a_max, or a large dt) — the true
// product would overflow int64 — so we detect saturation from the quotient
// cap/mul and return VI_MAX without ever forming cps*mul. Pre: mul>=1, div>=1,
// VI_MAX*div fits int64 (div is 1e6 or 1e12; VI_MAX*1e12 ~ 3.3e16 < 9.2e18).
static int32_t qf_limit(int64_t cps, int64_t mul, int64_t div) {
    if (cps < 1)  cps = 1;
    if (mul < 1)  mul = 1;
    int64_t cap = (int64_t)VI_MAX * div;
    if (cps >= cap / mul) return VI_MAX;   // cps*mul would reach VI_MAX*div -> clamp
    int64_t v = cps * mul / div;           // cps*mul < cap, so no overflow
    return v < 1 ? 1 : (int32_t)v;
}

void curve_interp_begin(curve_state_t *st, const curve_limits_t *lim,
                        int32_t p0x, int32_t p0y, int32_t p1x, int32_t p1y,
                        int32_t p2x, int32_t p2y, int32_t p3x, int32_t p3y,
                        int64_t v_in_wire, int64_t v_out_wire) {
    // Expand control points to power-basis coefficients (counts):
    //   d = P0;  c = 3(P1-P0);  b = 3(P2-2P1+P0);  a = P3-3P2+3P1-P0
    st->dx = p0x;
    st->cx = 3 * (int64_t)(p1x - p0x);
    st->bx = 3 * (int64_t)(p2x - 2 * p1x + p0x);
    st->ax = (int64_t)p3x - 3 * (int64_t)p2x + 3 * (int64_t)p1x - (int64_t)p0x;
    st->dy = p0y;
    st->cy = 3 * (int64_t)(p1y - p0y);
    st->by = 3 * (int64_t)(p2y - 2 * p1y + p0y);
    st->ay = (int64_t)p3y - 3 * (int64_t)p2y + 3 * (int64_t)p1y - (int64_t)p0y;

    st->t     = 0;
    st->s_done = 0;
    st->p3x   = p3x;
    st->p3y   = p3y;
    st->done  = false;

    // --- precompute the Q(F) tick-domain limits (once, off the hot path) ------
    // All conversions are overflow-safe: v_max_cps / a_max_cps2 can be astronomically
    // larger than the tick format holds (and a large dt scales them further), so the
    // naive products v_max_cps*dt / a_max_cps2*dt^2 would overflow int64. qf_limit()
    // saturates to VI_MAX in that regime instead. dt is also clamped so dt^2<<F can't
    // overflow regardless of config.
    int32_t dt = lim->dt_tick_us;
    if (dt < 1)      dt = 1;
    if (dt > 100000) dt = 100000;   // 100 ms: absurd as a tick, but keeps dt^2<<F safe

    // Pick F (fractional bits) to maximize speed precision while keeping v_max below
    // VI_MAX. If even F=0 already saturates (huge v_max_cps, or large dt), there is no
    // fractional headroom to take and F stays 0. The guard keeps v_max_cps*dt from
    // overflowing before we know it saturates.
    int32_t F = 0;
    if (lim->v_max_cps < ((int64_t)VI_MAX * 1000000) / dt) {
        int64_t base = lim->v_max_cps * dt / 1000000;   // < VI_MAX, no overflow
        if (base < 1) base = 1;
        while (F < VSHIFT_MAX && (base << (F + 1)) < VI_MAX) F++;
    }
    st->vshift = F;

    // v_max: Q(F) counts/tick.  a_max: Q(F) counts/tick^2 (= a_max_cps2 * dt^2 / 1e12).
    st->v_max = qf_limit(lim->v_max_cps,  (int64_t)dt << F,            1000000);
    st->a_max = qf_limit(lim->a_max_cps2, ((int64_t)dt * dt) << F, 1000000000000LL);

    // Wire speed (counts/tick * 256) -> internal Q(F): a shift, no physical math.
    st->v     = wire_to_vi(v_in_wire,  F);
    st->v_out = wire_to_vi(v_out_wire, F);

    // Arc length by trapezoidal integration of |B'(t)| over N samples.
    // S = sum (|B'(t_i)| + |B'(t_{i+1})|)/2 * (1/N).  |B'| ~<=1.8e6 -> S fits.
    int64_t prev = 0;
    {
        int64_t dx0 = eval_deriv(st->ax, st->bx, st->cx, 0);
        int64_t dy0 = eval_deriv(st->ay, st->by, st->cy, 0);
        prev = (int64_t)isqrt64((uint64_t)(dx0 * dx0 + dy0 * dy0));
    }
    int64_t acc = 0;
    for (int i = 1; i <= N_ARCLEN; i++) {
        uint64_t t = (i == N_ARCLEN) ? (Q32_ONE - 1) : ((uint64_t)i << 32) / N_ARCLEN;
        int64_t dx = eval_deriv(st->ax, st->bx, st->cx, t);
        int64_t dy = eval_deriv(st->ay, st->by, st->cy, t);
        int64_t cur = (int64_t)isqrt64((uint64_t)(dx * dx + dy * dy));
        acc += (prev + cur) / 2;            // each |B'| ~<=1.8e6, N small -> fits
        prev = cur;
    }
    st->S = acc / N_ARCLEN;                 // divide once: sum of averages * (1/N)
    if (st->S < 1) st->S = 1;
}

int64_t curve_speed_cps(const curve_state_t *st, int32_t dt_tick_us) {
    if (dt_tick_us < 1) dt_tick_us = 1;
    // v is Q(vshift) counts/tick; counts/s = v / 2^vshift * 1e6 / dt_us. Host-only,
    // so the int64 divide is fine (never on the ISR path).
    return ((int64_t)st->v * 1000000) / ((int64_t)dt_tick_us << st->vshift);
}

int64_t curve_cps_to_wire(int64_t v_cps, int32_t dt_tick_us) {
    if (v_cps < 0)      v_cps = 0;
    if (dt_tick_us < 1) dt_tick_us = 1;
    // counts/tick * 256 = v_cps * dt_us / 1e6 * 256, rounded to nearest. Saturate
    // before the product can overflow int64: past VI_MAX counts/tick the
    // interpolator clamps internally, so the wire need not carry more.
    if (v_cps >= (int64_t)VI_MAX * 1000000 / dt_tick_us) {
        return (int64_t)VI_MAX << CURVE_WIRE_V_FRAC;
    }
    int64_t scale = (int64_t)1 << CURVE_WIRE_V_FRAC;       // v_cps*dt < VI_MAX*1e6
    return (v_cps * dt_tick_us * scale + 500000) / 1000000;
}

bool IRAM_ATTR curve_interp_step(curve_state_t *st, uint16_t *out_x,
                                 uint16_t *out_y, int64_t *carry_v_wire) {
    const int32_t F     = st->vshift;
    const int32_t v_max = st->v_max;
    const int32_t a_max = st->a_max;

    if (st->done) {
        if (out_x) *out_x = (uint16_t)st->p3x;
        if (out_y) *out_y = (uint16_t)st->p3y;
        if (carry_v_wire) *carry_v_wire = vi_to_wire(st->v, F);
        return false;
    }

    const int32_t vi = st->v;                                 // current speed, Q(F)

    // --- geometry at the current parameter (wide; counts / unit-t) -----------
    int64_t d1x = eval_deriv(st->ax, st->bx, st->cx, st->t);   // B'(t)
    int64_t d1y = eval_deriv(st->ay, st->by, st->cy, st->t);
    int64_t b1  = (int64_t)isqrt64((uint64_t)(d1x * d1x + d1y * d1y)); // |B'|
    if (b1 < 1) b1 = 1;                                        // guard cusp

    int64_t d2x = eval_deriv2(st->ax, st->bx, st->t);          // B''(t)
    int64_t d2y = eval_deriv2(st->ay, st->by, st->t);
    int64_t cross = iabs64(d1x * d2y - d1y * d2x);             // |B' x B''| ~<=2.9e12

    // Radius of curvature R = |B'|^3 / |B' x B''|, clamped. b1<=2e6 -> b1^3<=8e18.
    int64_t b1c = b1 > B1_CAP ? B1_CAP : b1;
    int64_t R;
    if (cross == 0) {
        R = R_MAX;
    } else {
        R = (b1c * b1c * b1c) / cross;
        if (R > R_MAX) R = R_MAX;
        if (R < 1)     R = 1;
    }

    // --- friction circle: tangential accel left after the turn (all int32) ----
    // a_n = v^2 / R (centripetal), in Q(F) cnt/tick^2. v^2 fits int32 (v<2^15);
    // R<<F <= R_MAX<<VSHIFT_MAX < 2^31. ratio = a_n / a_max in Q16.
    int32_t vsq   = vi * vi;                                   // < 2^30
    int32_t Rf    = (int32_t)(R << F);                         // < 2^31
    int32_t a_n   = vsq / Rf;
    int32_t ratio_q16;
    if (a_n >= a_max) {
        ratio_q16 = 65536;                                    // saturate; a_t -> 0
    } else {
        ratio_q16 = (int32_t)(((int64_t)a_n << 16) / a_max);
    }
    int32_t r2_q16 = (int32_t)(((int64_t)ratio_q16 * ratio_q16) >> 16);  // <=65536
    int32_t one_minus = 65536 - r2_q16;
    if (one_minus < 0)     one_minus = 0;
    if (one_minus > 65535) one_minus = 65535;                 // keep root in u32
    int32_t sfac_q16 = (int32_t)isqrt32((uint32_t)one_minus << 16);
    int32_t a_t = (int32_t)(((int64_t)a_max * sfac_q16) >> 16);
    if (a_t < 1) a_t = 1;                                     // keep progress

    // --- speed ceilings (Q(F) counts/tick) -----------------------------------
    int32_t s_rem = (int32_t)(st->S - st->s_done);
    if (s_rem < 0) s_rem = 0;
    int64_t vmax_sq = (int64_t)v_max * v_max;                 // < 2^30

    // curvature ceiling v_curv = sqrt(a_max * R). Clamp to v_max BEFORE the root
    // so the radicand stays in int32 (when not clamped it is < vmax_sq).
    int64_t radc = ((int64_t)a_max * R) << F;
    int32_t v_curv = (radc >= vmax_sq) ? v_max
                                       : (int32_t)isqrt32((uint32_t)radc);

    int32_t ds_nom = vi >> F;                                 // counts moved in a tick

    // Braking ceiling: fastest we may go now and still decel to v_out. Start one
    // nominal step EARLY so discrete sampling never overshoots v_out.
    int32_t s_brake = s_rem - ds_nom;
    if (s_brake < 0) s_brake = 0;
    int64_t rb = (int64_t)st->v_out * st->v_out
               + (((int64_t)2 * a_t * s_brake) << F);
    int32_t v_brake = (rb >= vmax_sq) ? v_max
                                      : (int32_t)isqrt32((uint32_t)rb);

    // Accel reachable over a nominal step at the current speed.
    int64_t ra = (int64_t)vi * vi + (((int64_t)2 * a_t * ds_nom) << F);
    int32_t v_acc = (ra >= vmax_sq) ? v_max
                                    : (int32_t)isqrt32((uint32_t)ra);

    int32_t v_new = v_max;
    if (v_acc   < v_new) v_new = v_acc;
    if (v_brake < v_new) v_new = v_brake;

    // Per-tick rate limits: ACCELERATE no faster than the friction-circle budget
    // a_t (one tick of it = a_t); BRAKE up to full a_max. Asymmetric on purpose.
    int32_t da_acc = a_t;
    int32_t da_dec = a_max;
    if (v_new > vi + da_acc) v_new = vi + da_acc;
    if (v_new < vi - da_dec) v_new = vi - da_dec;

    // Hard curvature ceiling (safety): not rate-limited.
    if (v_new > v_curv) v_new = v_curv;

    // Floor: at least ~1 count of progress per tick (1 count/tick == 1<<F).
    int32_t v_floor = (int32_t)1 << F;
    if (v_floor < 1) v_floor = 1;
    if (v_new < v_floor) v_new = v_floor;

    // --- advance: step along the curve by the arc length for this tick --------
    int32_t v_avg = (vi + v_new) / 2;
    int32_t ds = v_avg >> F;                                  // counts
    if (ds < 1) ds = 1;

    // dt_param (Q32) = ds / |B'|. ds<<32 ~<= 575*4.3e9 ~ 2.5e12, fits.
    uint64_t dt_param = ((uint64_t)ds << 32) / (uint64_t)b1;
    uint64_t t_next = st->t + dt_param;

    st->v = v_new;
    st->s_done += ds;

    if (t_next >= Q32_ONE) {
        // Final tick: snap exactly to P3.
        st->t    = Q32_ONE;
        st->done = true;
        if (out_x) *out_x = (uint16_t)st->p3x;
        if (out_y) *out_y = (uint16_t)st->p3y;
        if (carry_v_wire) *carry_v_wire = vi_to_wire(v_new, F);
        return false;
    }

    st->t = t_next;
    int64_t px = eval_pos(st->ax, st->bx, st->cx, st->dx, st->t);
    int64_t py = eval_pos(st->ay, st->by, st->cy, st->dy, st->t);
    if (px < 0)     px = 0;
    if (px > 65535) px = 65535;
    if (py < 0)     py = 0;
    if (py > 65535) py = 65535;
    if (out_x) *out_x = (uint16_t)px;
    if (out_y) *out_y = (uint16_t)py;
    if (carry_v_wire) *carry_v_wire = vi_to_wire(v_new, F);
    return true;
}
