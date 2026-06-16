//! Split each cubic at inflection points and curvature extrema so curvature is
//! ~monotone within every emitted segment. That makes the binding curvature
//! limit fall at a *junction*, where the planner's `v_in/v_out` can capture it —
//! so the firmware only ever has to brake to its endpoints (no mid-segment
//! curvature surprises). The firmware still clamps curvature per-tick as a
//! safety net, so under-segmentation costs smoothness, never correctness.

use kurbo::{CubicBez, ParamCurve, ParamCurveArclen, ParamCurveCurvature};

use crate::model::Subpath;

const EPS: f64 = 1e-3;

fn split_cubic(c: &CubicBez) -> Vec<CubicBez> {
    // Curvature samples up front; a (near-)straight cubic needs no splitting and
    // would otherwise pick up spurious inflection/extrema noise.
    let n = 24usize;
    let k: Vec<f64> = (0..=n)
        .map(|i| c.curvature(i as f64 / n as f64).abs())
        .collect();
    if k.iter().all(|&x| x < 1e-7) {
        return vec![*c];
    }

    let mut ts: Vec<f64> = Vec::new();

    // Inflection points (analytic, kurbo).
    for t in c.inflections() {
        if t > EPS && t < 1.0 - EPS {
            ts.push(t);
        }
    }

    // Curvature extrema: local maxima/minima of |kappa| by sampling.
    for i in 1..n {
        let dl = k[i] - k[i - 1];
        let dr = k[i + 1] - k[i];
        if dl.abs() > 1e-12 && dl.signum() != dr.signum() {
            ts.push(i as f64 / n as f64);
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < EPS);

    let mut bounds = vec![0.0];
    bounds.extend(ts);
    bounds.push(1.0);
    bounds
        .windows(2)
        .filter(|w| w[1] - w[0] > EPS)
        .map(|w| c.subsegment(w[0]..w[1]))
        .collect()
}

/// Segment every cubic in a subpath; drop sub-pixel slivers.
pub fn segment_subpath(sp: &Subpath, accuracy: f64) -> Subpath {
    let mut cubics = Vec::new();
    for c in &sp.cubics {
        for piece in split_cubic(c) {
            if piece.arclen(accuracy) >= 1.0 {
                cubics.push(piece);
            }
        }
    }
    // Never drop a whole subpath to nothing (keep at least the original).
    if cubics.is_empty() {
        cubics = sp.cubics.clone();
    }
    Subpath {
        color: sp.color,
        closed: sp.closed,
        cubics,
    }
}
