//! Stage 1: SVG -> flattened, coloured polylines.
//!
//! Uses `usvg` to parse and normalise the document (resolving units, styles and
//! transforms; `usvg` returns path geometry already in absolute coordinates) and
//! `kurbo` to flatten Beziers/arcs into polylines with an adaptive tolerance.
//! Each resulting [`Subpath`] carries the resolved per-path stroke colour
//! (falling back to fill, then white).

use kurbo::{flatten, BezPath, PathEl, Point};
use usvg::tiny_skia_path::PathSegment;

use crate::model::{bounds, Rgb, Subpath};

/// Errors that can occur while parsing an SVG.
#[derive(Debug)]
pub enum ParseError {
    Usvg(usvg::Error),
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::Usvg(e) => write!(f, "svg parse error: {e}"),
        }
    }
}

impl std::error::Error for ParseError {}

impl From<usvg::Error> for ParseError {
    fn from(e: usvg::Error) -> Self {
        ParseError::Usvg(e)
    }
}

/// Options controlling parsing / flattening.
#[derive(Clone, Debug)]
pub struct ParseOptions {
    /// Maximum deviation (in SVG user units) when flattening curves to lines.
    /// Smaller = more points on curves = smoother but heavier output.
    pub tolerance: f64,
    /// Colour used when a path has neither stroke nor a solid fill colour.
    pub default_color: Rgb,
}

impl Default for ParseOptions {
    fn default() -> Self {
        ParseOptions {
            tolerance: 0.2,
            default_color: [1.0, 1.0, 1.0],
        }
    }
}

/// Parse SVG bytes into flattened, coloured polylines (in SVG user units, y-down).
pub fn parse_svg(data: &[u8], opts: &ParseOptions) -> Result<Vec<Subpath>, ParseError> {
    let tree = usvg::Tree::from_data(data, &usvg::Options::default())?;
    let mut out = Vec::new();
    collect_group(tree.root(), opts, &mut out);
    Ok(out)
}

fn collect_group(group: &usvg::Group, opts: &ParseOptions, out: &mut Vec<Subpath>) {
    for node in group.children() {
        match node {
            usvg::Node::Group(g) => collect_group(g, opts, out),
            usvg::Node::Path(p) => collect_path(p, opts, out),
            // Images and text are not drawable as vector strokes; skip them.
            usvg::Node::Image(_) | usvg::Node::Text(_) => {}
        }
    }
}

fn collect_path(path: &usvg::Path, opts: &ParseOptions, out: &mut Vec<Subpath>) {
    let color = path_color(path).unwrap_or(opts.default_color);
    let bez = tiny_skia_to_kurbo(path.data());

    // Flatten each subpath (sequence between MoveTo/Close) independently so we
    // keep them as separate polylines — the optimiser needs the subpath breaks.
    for sub in split_subpaths(&bez) {
        let closed = sub.elements().last() == Some(&PathEl::ClosePath);
        let mut points: Vec<[f64; 2]> = Vec::new();
        flatten(sub.elements().iter().copied(), opts.tolerance, |el| match el {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => points.push([p.x, p.y]),
            // `flatten` only yields MoveTo/LineTo (+ ClosePath for closed subpaths).
            PathEl::ClosePath => {
                if let Some(&first) = points.first() {
                    points.push(first);
                }
            }
            _ => {}
        });
        // Drop degenerate runs of identical points.
        points.dedup();
        if points.len() >= 2 {
            out.push(Subpath {
                color,
                closed,
                points,
            });
        }
    }
}

/// Resolve a path's draw colour: prefer stroke, then fill, else `None`.
fn path_color(path: &usvg::Path) -> Option<Rgb> {
    let from_paint = |paint: &usvg::Paint| match paint {
        usvg::Paint::Color(c) => Some([
            c.red as f32 / 255.0,
            c.green as f32 / 255.0,
            c.blue as f32 / 255.0,
        ]),
        // Gradients/patterns have no single colour; let the caller fall back.
        _ => None,
    };
    path.stroke()
        .and_then(|s| from_paint(s.paint()))
        .or_else(|| path.fill().and_then(|f| from_paint(f.paint())))
}

/// Convert a tiny-skia (usvg) path into a kurbo `BezPath`.
fn tiny_skia_to_kurbo(path: &usvg::tiny_skia_path::Path) -> BezPath {
    let mut bez = BezPath::new();
    for seg in path.segments() {
        match seg {
            PathSegment::MoveTo(p) => bez.move_to(pt(p)),
            PathSegment::LineTo(p) => bez.line_to(pt(p)),
            PathSegment::QuadTo(a, b) => bez.quad_to(pt(a), pt(b)),
            PathSegment::CubicTo(a, b, c) => bez.curve_to(pt(a), pt(b), pt(c)),
            PathSegment::Close => bez.close_path(),
        }
    }
    bez
}

fn pt(p: usvg::tiny_skia_path::Point) -> Point {
    Point::new(p.x as f64, p.y as f64)
}

/// Split a multi-subpath `BezPath` into one `BezPath` per subpath (each starting
/// at a MoveTo).
fn split_subpaths(bez: &BezPath) -> Vec<BezPath> {
    let mut subs = Vec::new();
    let mut cur = BezPath::new();
    for el in bez.elements() {
        if matches!(el, PathEl::MoveTo(_)) && !cur.elements().is_empty() {
            subs.push(std::mem::take(&mut cur));
        }
        cur.push(*el);
    }
    if !cur.elements().is_empty() {
        subs.push(cur);
    }
    subs
}

/// Stage 1b: normalise subpaths into laser space.
///
/// Fits the overall bounding box into `[-1, 1]` on both axes preserving aspect
/// ratio, centring the drawing, leaving `margin` (in normalised units) of head-
/// room on the tight axis, and flipping Y so the result is y-up (SVG is y-down).
pub fn fit_to_unit(subpaths: &[Subpath], margin: f64) -> Vec<Subpath> {
    let Some((min, max)) = bounds(subpaths) else {
        return Vec::new();
    };
    let span_x = (max[0] - min[0]).max(f64::EPSILON);
    let span_y = (max[1] - min[1]).max(f64::EPSILON);
    let cx = (min[0] + max[0]) / 2.0;
    let cy = (min[1] + max[1]) / 2.0;
    // Uniform scale that fits the larger span into [-(1-margin), (1-margin)].
    let usable = (1.0 - margin).max(f64::EPSILON);
    let scale = 2.0 * usable / span_x.max(span_y);

    subpaths
        .iter()
        .map(|sp| Subpath {
            color: sp.color,
            closed: sp.closed,
            points: sp
                .points
                .iter()
                .map(|&[x, y]| {
                    let nx = (x - cx) * scale;
                    // Flip Y: SVG y grows downward, laser space grows upward.
                    let ny = -(y - cy) * scale;
                    [nx, ny]
                })
                .collect(),
        })
        .collect()
}
