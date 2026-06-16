//! Shared data types that flow through the pipeline. Geometry is in DAC counts
//! (0..65535, centre 0x8000) once fitted, held as `kurbo` curves so we get arc
//! length, curvature, derivatives, inflections and subdivision for free.

use kurbo::CubicBez;

pub type Rgb = [f32; 3];

pub const GALVO_CENTER: f64 = 32768.0; // 0x8000

/// A connected run of cubic Béziers (a pen-down stroke), one colour. Chained:
/// `cubics[i].p3 == cubics[i+1].p0`.
#[derive(Clone, Debug)]
pub struct Subpath {
    pub color: Rgb,
    pub closed: bool,
    pub cubics: Vec<CubicBez>, // counts
}

impl Subpath {
    pub fn start(&self) -> kurbo::Point {
        self.cubics.first().map(|c| c.p0).unwrap_or_default()
    }
    pub fn end(&self) -> kurbo::Point {
        self.cubics.last().map(|c| c.p3).unwrap_or_default()
    }
    /// Reverse drawing direction (each cubic reversed, order reversed).
    pub fn reversed(&self) -> Subpath {
        let cubics = self
            .cubics
            .iter()
            .rev()
            .map(|c| CubicBez::new(c.p3, c.p2, c.p1, c.p0))
            .collect();
        Subpath {
            color: self.color,
            closed: self.closed,
            cubics,
        }
    }
}

/// One planned move = one `CURVE` on the wire (or a blank travel move). P0 is
/// implicit (the previous move's P3). `v_in`/`v_out` are filled by the planner
/// (counts/second).
#[derive(Clone, Debug)]
pub struct Move {
    pub cubic: CubicBez, // counts; cubic.p0 == previous move's p3
    pub color: Rgb,
    pub blank: bool, // true = travel move, beam off
    pub v_in: f64,   // counts/s (0 until planned)
    pub v_out: f64,
}
