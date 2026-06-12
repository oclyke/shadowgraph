//! Shared data types that flow between the pipeline stages.
//!
//! The pipeline is intentionally split into independent stages, each consuming
//! and producing plain data defined here so any stage can be tested or
//! visualised in isolation:
//!
//! ```text
//!   parse    SVG bytes      -> Vec<Subpath>   (SVG user units, y-down)
//!   fit      Vec<Subpath>   -> Vec<Subpath>   (normalised [-1,1], y-up)
//!   optimize Vec<Subpath>   -> Vec<OutPoint>  (lasy: order/blank/corners)
//!   emit     Vec<OutPoint>  -> Vec<Cmd>/bytes (GOTO/LASER/DWELL wire format)
//! ```

use std::hash::{Hash, Hasher};

use lasy::{Blanked, IsBlank, Lerp, Position, Weight};

/// Linear RGB, each channel in `0.0..=1.0`.
pub type Rgb = [f32; 3];

/// A flattened, single-colour polyline.
///
/// Coordinate space depends on the stage: after [`crate::parse`] the points are
/// in SVG user units (y-down); after [`crate::parse::fit_to_unit`] they are in
/// normalised laser space (`[-1, 1]` on both axes, y-up) ready for `lasy`.
#[derive(Clone, Debug, PartialEq)]
pub struct Subpath {
    pub color: Rgb,
    pub closed: bool,
    /// Polyline vertices. For a `closed` subpath the last point equals the first.
    pub points: Vec<[f64; 2]>,
}

/// Axis-aligned bounding box over a set of subpaths. `None` if there are no points.
pub fn bounds(subpaths: &[Subpath]) -> Option<([f64; 2], [f64; 2])> {
    let mut it = subpaths.iter().flat_map(|s| s.points.iter().copied());
    let first = it.next()?;
    let mut min = first;
    let mut max = first;
    for [x, y] in it {
        min[0] = min[0].min(x);
        min[1] = min[1].min(y);
        max[0] = max[0].max(x);
        max[1] = max[1].max(y);
    }
    Some((min, max))
}

// ---------------------------------------------------------------------------
// lasy point types
// ---------------------------------------------------------------------------
//
// `lasy` is generic over an *input* point type (knows position, blank-ness and
// weight, and is hashable for node de-duplication) and an *output* point type
// (interpolatable, can produce a blanked copy). See the `lasy` crate docs.

/// Input point fed to the `lasy` optimiser. Position in `[-1, 1]`, y-up.
///
/// A blank point is one whose colour is exactly black; `lasy` uses blank points
/// to discover where the beam may travel dark (between disconnected subpaths).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InPoint {
    pub pos: [f32; 2],
    pub rgb: Rgb,
    /// Extra times this point is drawn, to accent it (0 for smooth lines).
    pub weight: u32,
}

/// Output point produced by interpolation. No weight (applied during interp).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutPoint {
    pub pos: [f32; 2],
    pub rgb: Rgb,
}

impl InPoint {
    pub fn new(pos: [f32; 2], rgb: Rgb, weight: u32) -> Self {
        InPoint { pos, rgb, weight }
    }
    /// A blank (dark) copy at the same position.
    pub fn blanked(&self) -> Self {
        InPoint { rgb: [0.0; 3], ..*self }
    }
}

impl Position for InPoint {
    fn position(&self) -> [f32; 2] {
        self.pos
    }
}

impl IsBlank for InPoint {
    fn is_blank(&self) -> bool {
        self.rgb == [0.0; 3]
    }
}

impl Weight for InPoint {
    fn weight(&self) -> u32 {
        self.weight
    }
}

impl Hash for InPoint {
    // Quantise position + colour so coincident vertices collapse to one graph
    // node (mirrors the quantisation used in the lasy test-suite example).
    fn hash<H: Hasher>(&self, h: &mut H) {
        let qx = (self.pos[0] * i16::MAX as f32) as i32;
        let qy = (self.pos[1] * i16::MAX as f32) as i32;
        let qr = (self.rgb[0] * u16::MAX as f32) as u32;
        let qg = (self.rgb[1] * u16::MAX as f32) as u32;
        let qb = (self.rgb[2] * u16::MAX as f32) as u32;
        [qx, qy].hash(h);
        [qr, qg, qb].hash(h);
    }
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

impl IsBlank for OutPoint {
    fn is_blank(&self) -> bool {
        self.rgb == [0.0; 3]
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
        OutPoint {
            pos: self.pos.lerp(&dest.pos, amt),
            rgb: self.rgb.lerp(&dest.rgb, amt),
        }
    }
}
