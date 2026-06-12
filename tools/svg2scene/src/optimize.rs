//! Stage 2: galvo path optimisation via `lasy`.
//!
//! Takes normalised subpaths and produces the fully interpolated output point
//! stream, applying draw-order optimisation, minimal blanking between
//! disconnected geometry, blank-delay points and sharp-corner delay points.
//! This is the "velocity / curvature aware" step — corners and blank jumps get
//! extra dwell points so the real galvos can keep up.

use lasy::{
    euler_graph_to_euler_circuit, interpolate_euler_circuit, point_graph_to_euler_graph,
    points_to_segments, segments_to_point_graph, InterpolationConfig,
};

use crate::model::{InPoint, OutPoint, Subpath};

/// Re-export of lasy's interpolation knobs with our defaults.
#[derive(Clone, Debug)]
pub struct OptimizeOptions {
    /// Minimum number of points to spread across the whole frame. The optimiser
    /// always emits at least the distance/corner/blank-driven minimum, so this
    /// is a floor used to add extra smoothness when desired.
    pub target_points: u32,
    /// Per-point accent weight applied to every input vertex (0 = smooth lines).
    pub weight: u32,
    pub interp: InterpolationConfig,
}

impl Default for OptimizeOptions {
    fn default() -> Self {
        OptimizeOptions {
            target_points: 1,
            weight: 0,
            interp: InterpolationConfig::default(),
        }
    }
}

/// Build the flat `lasy` input-point list from coloured subpaths.
///
/// Between consecutive subpaths we insert a pair of blank points (a blank copy
/// at the previous subpath's end and at the next subpath's start). Per `lasy`'s
/// segment rules this is what marks the in-between travel as a *blank* segment
/// rather than a spurious lit line, while same-position lit/blank transitions
/// are skipped. `lasy` then recomputes the truly minimal blanking itself.
pub fn build_input(subpaths: &[Subpath], weight: u32) -> Vec<InPoint> {
    let mut input: Vec<InPoint> = Vec::new();
    for sp in subpaths {
        let mut sp_pts = sp.points.iter().map(|&[x, y]| {
            InPoint::new([x as f32, y as f32], sp.color, weight)
        });
        let Some(first) = sp_pts.next() else { continue };

        if let Some(prev) = input.last().copied() {
            input.push(prev.blanked()); // blank at previous end
            input.push(first.blanked()); // blank at this start
        }
        input.push(first);
        input.extend(sp_pts);
    }
    input
}

/// Run the full `lasy` optimisation pipeline.
pub fn optimize(subpaths: &[Subpath], opts: &OptimizeOptions) -> Vec<OutPoint> {
    let input = build_input(subpaths, opts.weight);
    optimize_points(&input, opts)
}

/// Optimise an already-built input point list (handy for tests).
pub fn optimize_points(input: &[InPoint], opts: &OptimizeOptions) -> Vec<OutPoint> {
    // Need at least one lit edge to do anything useful.
    if input.len() < 2 {
        return Vec::new();
    }
    let segs = points_to_segments(input.iter().cloned());
    let pg = segments_to_point_graph(input, segs);
    let eg = point_graph_to_euler_graph(&pg);
    let ec = euler_graph_to_euler_circuit(input, &eg);
    if ec.is_empty() {
        return Vec::new();
    }
    interpolate_euler_circuit(input, &ec, &eg, opts.target_points.max(1), &opts.interp)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Subpath;

    fn square() -> Subpath {
        Subpath {
            color: [1.0, 1.0, 1.0],
            closed: true,
            points: vec![
                [-0.5, -0.5],
                [-0.5, 0.5],
                [0.5, 0.5],
                [0.5, -0.5],
                [-0.5, -0.5],
            ],
        }
    }

    #[test]
    fn single_subpath_produces_points() {
        let out = optimize(&[square()], &OptimizeOptions::default());
        assert!(!out.is_empty());
        // All points of a single lit square should be lit (none blank).
        assert!(out.iter().all(|p| p.rgb != [0.0; 3]));
    }

    #[test]
    fn two_subpaths_insert_blanks() {
        let mut a = square();
        let mut b = square();
        for p in &mut a.points {
            p[0] -= 0.4;
        }
        for p in &mut b.points {
            p[0] += 0.4;
        }
        let out = optimize(&[a, b], &OptimizeOptions::default());
        // Travelling between the two squares requires at least one blank point.
        assert!(out.iter().any(|p| p.rgb == [0.0; 3]));
    }

    #[test]
    fn empty_input_is_safe() {
        assert!(optimize(&[], &OptimizeOptions::default()).is_empty());
    }
}
