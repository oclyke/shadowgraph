//! Feedrate planning: assign each move a feasible `(v_in, v_out)` so the firmware
//! never has to look past the segment it's drawing.
//!
//! 1. Cap each junction speed by the *smaller* of:
//!    - the corner (junction-deviation) limit from the tangent angle between the
//!      two segments — handles C0 corners (Grbl's model), and
//!    - the curvature limit `sqrt(a_max/kappa)` from the segments' endpoint
//!      curvature — handles smooth curving.
//!    Lit<->blank boundaries and the scene start/end are rest (v=0).
//! 2. A global forward then backward `v^2` pass makes every junction mutually
//!    reachable under `a_max`. (Discrete TOPP — see docs/CURVE_MOTION.md §3.4.)

use kurbo::{ParamCurve, ParamCurveArclen, ParamCurveCurvature, ParamCurveDeriv, Vec2};

use crate::interp::CurveLimits;
use crate::model::Move;

/// Per-move arc lengths (counts), used by the look-ahead.
pub fn arc_lengths(moves: &[Move]) -> Vec<f64> {
    moves.iter().map(|m| m.cubic.arclen(0.1).max(1.0)).collect()
}

/// Boundary speeds after the look-ahead, length `moves.len()+1`. Exposed so the
/// debug viz can draw the planned velocity at every control point.
pub fn junction_speeds(moves: &[Move], lim: &CurveLimits, corner_dev: f64) -> Vec<f64> {
    let n = moves.len();
    let vmax = lim.v_max_cps as f64;
    let amax = lim.a_max_cps2 as f64;
    let s = arc_lengths(moves);

    let mut vj = vec![0.0f64; n + 1];
    for i in 1..n {
        let a = &moves[i - 1];
        let b = &moves[i];
        if a.blank || b.blank {
            vj[i] = 0.0; // every lit<->travel boundary is a rest point
            continue;
        }
        let ti = unit(a.cubic.deriv().eval(1.0).to_vec2());
        let to = unit(b.cubic.deriv().eval(0.0).to_vec2());
        let dot = ti.dot(to).clamp(-1.0, 1.0);

        // Corner (junction deviation): straight(dot=1)->v_max, reversal(-1)->0.
        let sin_half = (0.5 * (1.0 + dot)).max(0.0).sqrt();
        let corner_cap = if sin_half >= 1.0 - 1e-6 {
            vmax
        } else {
            let r = corner_dev * sin_half / (1.0 - sin_half);
            (amax * r).sqrt().min(vmax)
        };

        // Curvature: most-limiting endpoint curvature of the two sides.
        let k = a.cubic.curvature(1.0).abs().max(b.cubic.curvature(0.0).abs());
        let curv_cap = if k < 1e-9 { vmax } else { (amax / k).sqrt().min(vmax) };

        vj[i] = corner_cap.min(curv_cap);
    }

    // Forward then backward v^2 passes.
    for k in 0..n {
        let reach = (vj[k] * vj[k] + 2.0 * amax * s[k]).sqrt();
        if reach < vj[k + 1] {
            vj[k + 1] = reach;
        }
    }
    for k in (0..n).rev() {
        let reach = (vj[k + 1] * vj[k + 1] + 2.0 * amax * s[k]).sqrt();
        if reach < vj[k] {
            vj[k] = reach;
        }
    }
    vj
}

/// Fill in every move's `v_in`/`v_out` from the junction speeds.
pub fn plan(moves: &mut [Move], lim: &CurveLimits, corner_dev: f64) {
    if moves.is_empty() {
        return;
    }
    let vj = junction_speeds(moves, lim, corner_dev);
    for (k, m) in moves.iter_mut().enumerate() {
        m.v_in = vj[k];
        m.v_out = vj[k + 1];
    }
}

fn unit(v: Vec2) -> Vec2 {
    let l = v.hypot();
    if l < 1e-9 {
        Vec2::new(1.0, 0.0)
    } else {
        v / l
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Move;
    use kurbo::{CubicBez, Point};

    /// Fixed limits for tests — independent of the firmware's CURVE_DEFAULT_*.
    fn tlim() -> CurveLimits {
        CurveLimits {
            v_max_cps: 11_468_800,
            a_max_cps2: 57_344_000_000,
            dt_tick_us: 20,
        }
    }

    fn line(ax: f64, ay: f64, bx: f64, by: f64) -> Move {
        let a = Point::new(ax, ay);
        let b = Point::new(bx, by);
        Move {
            cubic: CubicBez::new(a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b),
            color: [1.0, 1.0, 1.0],
            blank: false,
            v_in: 0.0,
            v_out: 0.0,
        }
    }

    #[test]
    fn straight_junction_runs_fast() {
        let lim = tlim();
        let m = [line(0.0, 0.0, 10000.0, 0.0), line(10000.0, 0.0, 20000.0, 0.0)];
        let vj = junction_speeds(&m, &lim, 200.0);
        assert!(vj[1] > lim.v_max_cps as f64 * 0.99, "straight should hit v_max");
    }

    #[test]
    fn sharp_corner_slows_a_lot() {
        let lim = tlim();
        // 90° corner.
        let m = [
            line(0.0, 0.0, 10000.0, 0.0),
            line(10000.0, 0.0, 10000.0, 10000.0),
        ];
        let vj = junction_speeds(&m, &lim, 200.0);
        assert!(
            vj[1] < lim.v_max_cps as f64 * 0.6,
            "corner junction {} should be well below v_max",
            vj[1]
        );
        // Reachability: never exceeds what the look-ahead allows from rest ends.
        assert!(vj[0] == 0.0 && vj[2] == 0.0);
    }
}
