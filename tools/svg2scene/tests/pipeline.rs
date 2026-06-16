//! End-to-end: SVG -> plan -> FFI simulation, asserting the realised motion
//! stays within the galvo limits (the whole point of the planner + interpolator).

use svg2scene::interp::CurveLimits;
use svg2scene::{analyze, order, parse, plan, segment};

const SVG: &[u8] = br##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 200 200">
  <path d="M40,100 C40,40 160,40 160,100 C160,160 40,160 40,100 Z" fill="none" stroke="#0f0"/>
  <path d="M30,30 L170,30 L100,170 Z" fill="none" stroke="#f00"/>
</svg>"##;

#[test]
fn end_to_end_respects_limits() {
    let raw = parse::parse_svg(SVG, &parse::ParseOptions::default()).unwrap();
    assert!(!raw.is_empty());
    let fitted = parse::fit_to_counts(&raw, 28672.0, 0.05);
    let ordered = order::order(fitted);
    let segmented: Vec<_> = ordered
        .iter()
        .map(|s| segment::segment_subpath(s, 0.1))
        .collect();
    let mut moves = order::build_moves(&segmented);
    assert!(moves.len() > 4);

    // Fixed limits — independent of the firmware's CURVE_DEFAULT_* so retuning the
    // galvo in curve_interp.h never breaks this test.
    let lim = CurveLimits {
        v_max_cps: 11_468_800,
        a_max_cps2: 57_344_000_000,
        dt_tick_us: 20,
    };
    plan::plan(&mut moves, &lim, 200.0);

    // Junction velocities must be feasible: each v_out reachable from v_in.
    let amax = lim.a_max_cps2 as f64;
    for m in &moves {
        let s = kurbo::ParamCurveArclen::arclen(&m.cubic, 0.1).max(1.0);
        let reachable = (m.v_in * m.v_in + 2.0 * amax * s).sqrt();
        assert!(
            m.v_out <= reachable + 1.0,
            "v_out {} unreachable from v_in {} over {}",
            m.v_out,
            m.v_in,
            s
        );
    }

    // Simulate through the real firmware interpolator and check limits.
    let pts = analyze::simulate(&moves, &lim);
    let st = analyze::stats(&pts, &lim);
    assert!(st.points > 50, "expected a dense setpoint stream");
    assert!(
        st.max_v <= lim.v_max_cps as f64 * 1.01,
        "speed {} exceeded v_max",
        st.max_v
    );
    assert!(
        st.max_a <= lim.a_max_cps2 as f64 * 1.15,
        "accel {} exceeded a_max (windowed)",
        st.max_a
    );
    assert!(st.frame_s > 0.0 && st.refresh_hz() > 0.0);
}
