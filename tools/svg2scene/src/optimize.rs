//! Draw-order, blanking and dense interpolation via `lasy`'s euler-circuit
//! optimisation — the canonical ILDA point-stream path.
//!
//! We feed `lasy` the flattened polyline vertices (lit, coloured) with blank
//! *bridges* between disjoint polylines, then run its full pipeline:
//! `points_to_segments → segments_to_point_graph → point_graph_to_euler_graph →
//! euler_graph_to_euler_circuit → interpolate_euler_circuit`. That yields a single
//! unicursal path (no retraced lit lines), minimal blank travels, blank-delay and
//! sharp-corner-delay points, interpolated to a dense, uniform point stream — i.e.
//! exactly what the fixed-rate firmware engine wants, with no curve/CNC planning.

use std::hash::{Hash, Hasher};

use lasy::{
    euler_graph_to_euler_circuit, interpolate_euler_circuit, point_graph_to_euler_graph,
    points_to_segments, segments_to_point_graph, Blanked, InterpolationConfig, IsBlank, Lerp,
    Position, Weight,
};

use crate::model::{Polyline, Rgb};

/// Host-side optimisation knobs (no device limits — the engine just plays points).
pub struct OptimizeOptions {
    /// Target total points in the frame (density). Implied refresh =
    /// `point_rate_hz / target_points`. `lasy` may add more for delays.
    pub target_points: u32,
    /// `lasy` distance-per-point floor (normalised units).
    pub distance_per_point: f32,
    /// Points held at the end of each blank for light-modulator settle.
    pub blank_points: u32,
    /// Radians of corner angle per extra dwell point (galvo inertia at sharp turns).
    pub corner_radians: f32,
}

impl Default for OptimizeOptions {
    fn default() -> Self {
        OptimizeOptions {
            target_points: 600,
            distance_per_point: InterpolationConfig::DEFAULT_DISTANCE_PER_POINT,
            blank_points: InterpolationConfig::DEFAULT_BLANK_DELAY_POINTS,
            corner_radians: InterpolationConfig::DEFAULT_RADIANS_PER_POINT,
        }
    }
}

/// `lasy` input point: a vertex in normalised `[-1,1]` space, its colour, and a
/// blank flag. Blank *bridge* points are placed at the same position as the lit
/// endpoint they border, so the lit↔blank transition is a zero-length (skipped)
/// segment and only the blank↔blank hop becomes a travel move.
#[derive(Clone)]
pub struct InPoint {
    pos: [f32; 2],
    rgb: Rgb,
    blank: bool,
}

impl Position for InPoint {
    fn position(&self) -> [f32; 2] {
        self.pos
    }
}
impl IsBlank for InPoint {
    fn is_blank(&self) -> bool {
        self.blank
    }
}
impl Weight for InPoint {
    fn weight(&self) -> u32 {
        0
    }
}
impl Hash for InPoint {
    fn hash<H: Hasher>(&self, h: &mut H) {
        // Quantise position to i16 range and colour to u16 (matches lasy's own
        // node-dedup convention) so coincident same-colour vertices merge.
        ((self.pos[0] * i16::MAX as f32) as i32).hash(h);
        ((self.pos[1] * i16::MAX as f32) as i32).hash(h);
        for c in self.rgb {
            ((c * u16::MAX as f32) as u32).hash(h);
        }
    }
}

/// `lasy` output (raw) point: position + colour, no weight. Blanked points carry
/// rgb 0 (that is how we read blanking back out in `emit`).
#[derive(Clone)]
pub struct OutPoint {
    pub pos: [f32; 2],
    pub rgb: Rgb,
}

impl From<InPoint> for OutPoint {
    fn from(p: InPoint) -> Self {
        OutPoint {
            pos: p.pos,
            rgb: p.rgb,
        }
    }
}
impl Position for OutPoint {
    fn position(&self) -> [f32; 2] {
        self.pos
    }
}
impl Blanked for OutPoint {
    fn blanked(&self) -> Self {
        OutPoint {
            pos: self.pos,
            rgb: [0.0; 3],
        }
    }
}
impl Lerp for OutPoint {
    type Scalar = f32;
    fn lerp(&self, dest: &Self, amt: f32) -> Self {
        let l = |a: f32, b: f32| a + (b - a) * amt;
        OutPoint {
            pos: [l(self.pos[0], dest.pos[0]), l(self.pos[1], dest.pos[1])],
            rgb: [
                l(self.rgb[0], dest.rgb[0]),
                l(self.rgb[1], dest.rgb[1]),
                l(self.rgb[2], dest.rgb[2]),
            ],
        }
    }
}

impl OutPoint {
    /// A point is blank when its colour is fully off (lasy's blanking convention).
    pub fn is_blank(&self) -> bool {
        self.rgb == [0.0; 3]
    }
}

/// Build the `lasy` input stream: lit vertices per polyline, with blank bridges
/// between disjoint polylines (placed at shared positions so no spurious lit edge
/// is created across the gap).
fn build_input(polylines: &[Polyline]) -> Vec<InPoint> {
    let mut pts = Vec::new();
    let mut prev_end: Option<[f32; 2]> = None;
    for poly in polylines {
        if poly.pts.len() < 2 {
            continue;
        }
        let start = poly.pts[0];
        if let Some(pe) = prev_end {
            pts.push(InPoint { pos: pe, rgb: [0.0; 3], blank: true });
            pts.push(InPoint { pos: start, rgb: [0.0; 3], blank: true });
        }
        for &p in &poly.pts {
            pts.push(InPoint { pos: p, rgb: poly.color, blank: false });
        }
        prev_end = Some(*poly.pts.last().unwrap());
    }
    pts
}

/// Run the full `lasy` pipeline, returning the interpolated dense point stream.
pub fn optimize(polylines: &[Polyline], opts: &OptimizeOptions) -> Vec<OutPoint> {
    let pts = build_input(polylines);
    if pts.len() < 2 {
        return Vec::new();
    }

    let conf = InterpolationConfig {
        distance_per_point: opts.distance_per_point,
        blank_delay_points: opts.blank_points,
        radians_per_point: opts.corner_radians,
    };

    let segs: Vec<_> = points_to_segments(pts.iter().cloned()).collect();
    let pg = segments_to_point_graph(&pts, segs);
    let eg = point_graph_to_euler_graph(&pg);
    let ec = euler_graph_to_euler_circuit(&pts, &eg);
    if ec.is_empty() {
        return Vec::new();
    }
    interpolate_euler_circuit(&pts, &ec, &eg, opts.target_points, &conf)
}
