// Host unit tests for the fixed-point curve interpolator (gtest).
//
// These validate the *physics*: the emitted setpoints must respect the galvo
// limits (tangential accel <= a_max, centripetal v^2*kappa <= a_max), terminate
// exactly at P3, and traverse the right arc length / time. Speed is read from the
// exact internal state (st.v); curvature is reconstructed from the emitted u16
// points (windowed, to ride out the 1-count quantisation).
#include "curve_interp.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

namespace {

struct Sample { double x, y; int64_t v; };

// Fixed limits for the tests — deliberately NOT the CURVE_DEFAULT_* macros, so
// retuning the galvo in curve_interp.h never breaks the suite. The geometry and
// tolerances below are tuned to these numbers.
curve_limits_t test_limits() {
    curve_limits_t lim;
    lim.v_max_cps  = 11468800;
    lim.a_max_cps2 = 57344000000;
    lim.dt_tick_us = 20;
    return lim;
}

// Planner speed (counts/s) -> wire format (counts/tick * 256), matching the host's
// cps_to_wire and what the firmware consumes. v_in/v_out below are given in
// counts/s for readability; begin() takes wire units.
int64_t cps_to_wire(int64_t v_cps, int32_t dt_us) {
    if (v_cps < 0) v_cps = 0;
    int64_t scale = (int64_t)1 << CURVE_WIRE_V_FRAC;
    return (v_cps * dt_us * scale + 500000) / 1000000;
}

// Run a curve to completion, recording one sample per emitted setpoint. Speed is
// read back in counts/s via curve_speed_cps (the interpolator stores it tick-native).
std::vector<Sample> run(curve_state_t &st, const curve_limits_t &lim,
                        int p0x, int p0y, int p1x, int p1y,
                        int p2x, int p2y, int p3x, int p3y,
                        int64_t v_in, int64_t v_out, int max_steps = 200000) {
    curve_interp_begin(&st, &lim, p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y,
                       cps_to_wire(v_in, lim.dt_tick_us),
                       cps_to_wire(v_out, lim.dt_tick_us));
    std::vector<Sample> out;
    uint16_t x = 0, y = 0;
    int64_t carry = 0;
    bool going = true;
    int n = 0;
    while (going && n < max_steps) {
        going = curve_interp_step(&st, &x, &y, &carry);
        out.push_back({(double)x, (double)y, curve_speed_cps(&st)});
        n++;
    }
    EXPECT_LT(n, max_steps) << "curve did not terminate";
    return out;
}

double dist(const Sample &a, const Sample &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Menger curvature of three points: kappa = 4*Area / (|ab|*|bc|*|ca|).
double menger(const Sample &a, const Sample &b, const Sample &c) {
    double area2 = std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    double ab = dist(a, b), bc = dist(b, c), ca = dist(c, a);
    if (ab < 1e-9 || bc < 1e-9 || ca < 1e-9) return 0.0;
    return 2.0 * area2 / (ab * bc * ca);
}

const double DT_S = 20 * 1e-6; // matches test_limits().dt_tick_us, not the macro

}  // namespace

// A straight at v_max should hold v_max the whole way and land exactly on P3.
TEST(CurveInterp, StraightAtVmaxIsConstant) {
    auto lim = test_limits();
    curve_state_t st;
    // Control points evenly spaced along the line => constant |B'|.
    auto s = run(st, lim, 2768, 32768, 22768, 32768, 42768, 32768, 62768, 32768,
                 lim.v_max_cps, lim.v_max_cps);
    ASSERT_GT(s.size(), 10u);
    for (const auto &p : s) {
        EXPECT_LE(p.v, lim.v_max_cps);
        EXPECT_GE(p.v, lim.v_max_cps * 95 / 100);   // never sags far below v_max
    }
    EXPECT_EQ(s.back().x, 62768.0);                  // exact snap to P3
    EXPECT_EQ(s.back().y, 32768.0);
    // Arc length ~ 60000 counts; time ~ S / v_max.
    double t_total = s.size() * DT_S;
    double expect  = 60000.0 / (double)lim.v_max_cps;
    EXPECT_NEAR(t_total, expect, expect * 0.05);
    EXPECT_NEAR((double)st.S, 60000.0, 60000.0 * 0.01);
}

// Accelerating from rest on a straight: tangential accel must stay <= a_max and
// the speed must climb to v_max.
TEST(CurveInterp, AccelFromRestRespectsAmax) {
    auto lim = test_limits();
    curve_state_t st;
    auto s = run(st, lim, 2768, 32768, 22768, 32768, 42768, 32768, 62768, 32768,
                 0, lim.v_max_cps);
    ASSERT_GT(s.size(), 30u);
    int64_t vmax_seen = 0;
    for (size_t i = 1; i < s.size(); i++) {
        double a = (double)(s[i].v - s[i - 1].v) / DT_S;       // counts/s^2
        EXPECT_LE(a, (double)lim.a_max_cps2 * 1.05) << "step " << i;
        vmax_seen = std::max(vmax_seen, s[i].v);
    }
    EXPECT_GE(vmax_seen, lim.v_max_cps * 98 / 100);            // reached cruise
    EXPECT_EQ(s.back().x, 62768.0);
}

// Decelerating to a stop on a straight: |decel| <= a_max and the exit is slow.
TEST(CurveInterp, DecelToStopRespectsAmax) {
    auto lim = test_limits();
    curve_state_t st;
    auto s = run(st, lim, 2768, 32768, 22768, 32768, 42768, 32768, 62768, 32768,
                 lim.v_max_cps, 0);
    ASSERT_GT(s.size(), 30u);
    for (size_t i = 1; i < s.size(); i++) {
        double a = (double)(s[i].v - s[i - 1].v) / DT_S;
        EXPECT_GE(a, -(double)lim.a_max_cps2 * 1.05) << "step " << i;
    }
    EXPECT_LT(s.back().v, lim.v_max_cps / 10);                 // ended slow
    EXPECT_EQ(s.back().x, 62768.0);
}

// A tight arc (radius ~1500 counts < v_max^2/a_max ~2300) must force a slowdown,
// and the centripetal accel v^2*kappa must stay within a_max.
TEST(CurveInterp, TightArcLimitsCentripetal) {
    auto lim = test_limits();
    lim.dt_tick_us = 5;          // dense sampling so curvature reconstruction is
                                 // robust regardless of the default tick rate
    curve_state_t st;
    // Quarter circle, centre (32768,32768), r=1500, cubic handle k=0.5523*r=828.
    auto s = run(st, lim, 34268, 32768, 34268, 33596, 33596, 34268, 32768, 34268,
                 lim.v_max_cps, lim.v_max_cps);
    ASSERT_GT(s.size(), 20u);

    int64_t v_min = lim.v_max_cps, v_max_seen = 0;
    for (const auto &p : s) {
        v_min = std::min(v_min, p.v);
        v_max_seen = std::max(v_max_seen, p.v);
        EXPECT_LE(p.v, lim.v_max_cps);                         // never exceeds v_max
    }
    EXPECT_LT(v_min, lim.v_max_cps * 97 / 100);                // curvature engaged

    // Windowed centripetal check: kappa from spread-out points, v exact.
    const int W = 4;
    for (size_t i = W; i + W < s.size(); i++) {
        double k = menger(s[i - W], s[i], s[i + W]);
        double a_n = (double)s[i].v * (double)s[i].v * k;
        EXPECT_LE(a_n, (double)lim.a_max_cps2 * 1.3) << "step " << i;
    }
    EXPECT_EQ(s.back().x, 32768.0);
    EXPECT_EQ(s.back().y, 34268.0);
}

// Arc-length estimate should match a fine double-precision reference.
TEST(CurveInterp, ArcLengthAccurate) {
    auto lim = test_limits();
    curve_state_t st;
    const double P0x = 10000, P0y = 10000, P1x = 20000, P1y = 50000,
                 P2x = 50000, P2y = 50000, P3x = 60000, P3y = 10000;
    curve_interp_begin(&st, &lim, P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y,
                       lim.v_max_cps, lim.v_max_cps);
    // Reference arc length: dense sampling of |B'(t)|.
    const int N = 4000;
    double ref = 0, px = P0x, py = P0y;
    for (int i = 1; i <= N; i++) {
        double t = (double)i / N, u = 1 - t;
        double x = u*u*u*P0x + 3*u*u*t*P1x + 3*u*t*t*P2x + t*t*t*P3x;
        double y = u*u*u*P0y + 3*u*u*t*P1y + 3*u*t*t*P2y + t*t*t*P3y;
        ref += std::hypot(x - px, y - py);
        px = x; py = y;
    }
    EXPECT_NEAR((double)st.S, ref, ref * 0.02);
}

// Calling step() past completion is safe and keeps reporting P3.
TEST(CurveInterp, StableAfterDone) {
    auto lim = test_limits();
    curve_state_t st;
    auto s = run(st, lim, 1000, 1000, 2000, 1000, 3000, 1000, 4000, 1000,
                 lim.v_max_cps, lim.v_max_cps);
    uint16_t x = 0, y = 0;
    int64_t carry = 0;
    for (int i = 0; i < 5; i++) {
        EXPECT_FALSE(curve_interp_step(&st, &x, &y, &carry));
        EXPECT_EQ(x, 4000);
        EXPECT_EQ(y, 1000);
    }
}
