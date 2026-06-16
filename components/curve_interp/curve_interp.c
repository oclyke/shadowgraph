#include "curve_interp.h"

// Integer fixed-point cubic-Bézier interpolator. No float (runs in the ISR).
// See curve_interp.h for the unit conventions and docs/CURVE_MOTION.md for the
// motion model. Every product below is annotated with its worst-case magnitude
// so the int64 headroom (max 9.22e18) is auditable.
//
// Worst-case scales used in the audits (DAC counts in 0..65535):
//   |coeff|     <= ~5.2e5   (cubic coeffs from extreme control nets)
//   |B'(t)|     <= ~1.8e6   (param-space speed; clamped to 2e6 before cubing)
//   |B''(t)|    <= ~1.6e6
//   v           <= v_max ~ 1.15e7 counts/s   ->  v*v ~ 1.32e14
//   a_max       ~ 5.7e10 counts/s^2

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

// Exact integer sqrt of a uint64 (floor). Bit-by-bit, no float.
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

// Evaluate B(t) on one axis via Horner. coeffs and t (Q32) -> counts.
// acc*t: |acc| ~ <=5e5, t < 4.3e9  ->  ~2.2e15, fits int64.
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

void curve_interp_begin(curve_state_t *st, const curve_limits_t *lim,
                        int32_t p0x, int32_t p0y, int32_t p1x, int32_t p1y,
                        int32_t p2x, int32_t p2y, int32_t p3x, int32_t p3y,
                        int64_t v_in_cps, int64_t v_out_cps) {
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

    st->lim   = *lim;
    st->v     = v_in_cps;
    st->v_out = v_out_cps;
    st->t     = 0;
    st->s_done = 0;
    st->p3x   = p3x;
    st->p3y   = p3y;
    st->done  = false;

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

bool IRAM_ATTR curve_interp_step(curve_state_t *st, uint16_t *out_x,
                                 uint16_t *out_y, int64_t *carry_v_cps) {
    const int64_t a_max = st->lim.a_max_cps2;
    const int64_t v_max = st->lim.v_max_cps;
    const int32_t dt_us = st->lim.dt_tick_us;

    if (st->done) {
        if (out_x) *out_x = (uint16_t)st->p3x;
        if (out_y) *out_y = (uint16_t)st->p3y;
        if (carry_v_cps) *carry_v_cps = st->v;
        return false;
    }

    // --- geometry at the current parameter -------------------------------
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

    // --- friction circle: tangential accel left after the turn -----------
    // a_n = v^2 / R (centripetal). ratio = a_n / a_max in Q16 (clamped to 1).
    int64_t a_n = (st->v * st->v) / R;                        // v^2 ~<=1.32e14
    int64_t ratio_q16;
    if (a_n >= a_max) {
        ratio_q16 = 65536;                                    // saturate; a_t -> 0
    } else {
        ratio_q16 = (a_n << 16) / a_max;                      // a_n<<16 ~<=3.7e15
    }
    int64_t r2_q16 = (ratio_q16 * ratio_q16) >> 16;           // <=65536
    int64_t one_minus = 65536 - r2_q16;
    if (one_minus < 0) one_minus = 0;
    int64_t sfac_q16 = (int64_t)isqrt64((uint64_t)one_minus << 16); // sqrt in Q16
    int64_t a_t = (a_max * sfac_q16) >> 16;                    // a_max*Q16 ~<=3.7e15
    if (a_t < 1) a_t = 1;                                      // keep progress

    // --- speed ceilings --------------------------------------------------
    int64_t s_rem = st->S - st->s_done;
    if (s_rem < 0) s_rem = 0;

    // curvature ceiling: v_curv = sqrt(a_max * R), capped at v_max.
    int64_t v_curv = (int64_t)isqrt64((uint64_t)(a_max * R)); // a_max*R_MAX ~5.7e16
    if (v_curv > v_max) v_curv = v_max;

    int64_t ds_nom = (st->v * (int64_t)dt_us) / 1000000;      // nominal step, counts

    // Braking ceiling: fastest we may go now and still decel to v_out. Start one
    // nominal step EARLY (s_rem - ds_nom) so discrete sampling never steps over
    // the ideal brake point and overshoots v_out.
    int64_t s_brake = s_rem - ds_nom;
    if (s_brake < 0) s_brake = 0;
    int64_t v_brake = (int64_t)isqrt64(
        (uint64_t)(st->v_out * st->v_out + 2 * a_t * s_brake)); // 2*a_t*s ~<=1e16

    // Accel reachable over a nominal step at the current speed.
    int64_t v_acc = (int64_t)isqrt64(
        (uint64_t)(st->v * st->v + 2 * a_t * ds_nom));

    int64_t v_new = v_max;
    if (v_acc   < v_new) v_new = v_acc;
    if (v_brake < v_new) v_new = v_brake;

    // Per-tick rate limits: ACCELERATE no faster than the friction-circle budget
    // a_t (riding the circle); BRAKE up to full a_max (needed to reach v_out and
    // to recover toward the curvature ceiling). Asymmetric on purpose.
    int64_t da_acc = (a_t   * (int64_t)dt_us) / 1000000;
    int64_t da_dec = (a_max * (int64_t)dt_us) / 1000000;
    if (da_acc < 1) da_acc = 1;
    if (v_new > st->v + da_acc) v_new = st->v + da_acc;
    if (v_new < st->v - da_dec) v_new = st->v - da_dec;

    // Hard curvature ceiling (safety): never exceed the speed at which the
    // centripetal accel alone hits a_max. Not rate-limited — it is a hard cap.
    if (v_new > v_curv) v_new = v_curv;

    // Floor: guarantee at least ~1 count of progress per tick so we terminate.
    int64_t v_floor = 1000000 / (dt_us > 0 ? dt_us : 1);      // 1 count / tick
    if (v_new < v_floor) v_new = v_floor;

    // --- advance: step along the curve by the arc length for this tick ----
    int64_t v_avg = (st->v + v_new) / 2;
    int64_t ds = (v_avg * (int64_t)dt_us) / 1000000;          // counts
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
        if (carry_v_cps) *carry_v_cps = st->v;
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
    if (carry_v_cps) *carry_v_cps = st->v;
    return true;
}
