//! SVG -> coloured cubic-Bézier subpaths.
//!
//! Parses with `usvg` (resolves units/styles/transforms), keeps the geometry as
//! **cubics** (no flattening — that is the whole point of the curve pipeline),
//! then fits everything into DAC-count space (`[-1,1]` field -> counts, y-up).

use kurbo::{BezPath, CubicBez, PathEl, Point, QuadBez, Rect, Shape};

use crate::model::{Rgb, Subpath, GALVO_CENTER};

pub struct ParseOptions {
    pub default_color: Rgb,
}

impl Default for ParseOptions {
    fn default() -> Self {
        ParseOptions {
            default_color: [1.0, 1.0, 1.0],
        }
    }
}

/// A path straight out of the SVG: colour + geometry in SVG absolute coords.
pub struct RawPath {
    pub color: Rgb,
    pub bez: BezPath,
}

fn color_of(path: &usvg::Path, default: Rgb) -> Rgb {
    let from_paint = |paint: &usvg::Paint| -> Option<Rgb> {
        if let usvg::Paint::Color(c) = paint {
            Some([
                c.red as f32 / 255.0,
                c.green as f32 / 255.0,
                c.blue as f32 / 255.0,
            ])
        } else {
            None
        }
    };
    path.stroke()
        .and_then(|s| from_paint(s.paint()))
        .or_else(|| path.fill().and_then(|f| from_paint(f.paint())))
        .unwrap_or(default)
}

fn collect(node: &usvg::Node, out: &mut Vec<RawPath>, default: Rgb) {
    match node {
        usvg::Node::Group(g) => {
            for child in g.children() {
                collect(child, out, default);
            }
        }
        usvg::Node::Path(p) => {
            let t = p.abs_transform();
            let map = |x: f32, y: f32| -> Point {
                Point::new(
                    (t.sx * x + t.kx * y + t.tx) as f64,
                    (t.ky * x + t.sy * y + t.ty) as f64,
                )
            };
            let mut bez = BezPath::new();
            for seg in p.data().segments() {
                use usvg::tiny_skia_path::PathSegment as S;
                match seg {
                    S::MoveTo(a) => bez.move_to(map(a.x, a.y)),
                    S::LineTo(a) => bez.line_to(map(a.x, a.y)),
                    S::QuadTo(a, b) => bez.quad_to(map(a.x, a.y), map(b.x, b.y)),
                    S::CubicTo(a, b, c) => {
                        bez.curve_to(map(a.x, a.y), map(b.x, b.y), map(c.x, c.y))
                    }
                    S::Close => bez.close_path(),
                }
            }
            if !bez.elements().is_empty() {
                out.push(RawPath {
                    color: color_of(p, default),
                    bez,
                });
            }
        }
        _ => {}
    }
}

pub fn parse_svg(data: &[u8], opts: &ParseOptions) -> Result<Vec<RawPath>, String> {
    let tree = usvg::Tree::from_data(data, &usvg::Options::default())
        .map_err(|e| format!("usvg parse: {e}"))?;
    let mut out = Vec::new();
    for child in tree.root().children() {
        collect(child, &mut out, opts.default_color);
    }
    Ok(out)
}

fn line_cubic(a: Point, b: Point) -> CubicBez {
    CubicBez::new(a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b)
}

/// Fit all paths into count space: uniform scale so the larger bbox dimension
/// fills the field (minus `margin`), centred at 0x8000, y flipped (SVG is y-down,
/// the field is y-up). `amplitude` = counts from centre to field edge (±1).
pub fn fit_to_counts(paths: &[RawPath], amplitude: f64, margin: f64) -> Vec<Subpath> {
    let mut bbox: Option<Rect> = None;
    for p in paths {
        let bb = p.bez.bounding_box();
        bbox = Some(match bbox {
            Some(acc) => acc.union(bb),
            None => bb,
        });
    }
    let Some(bb) = bbox else { return Vec::new() };

    let half = (amplitude * (1.0 - margin)).max(1.0);
    let span = (bb.width().max(bb.height())).max(1e-9);
    let s = (2.0 * half) / span; // scale: larger dim -> full field width
    let (cx, cy) = (bb.center().x, bb.center().y);
    let map = |p: Point| -> Point {
        Point::new(
            GALVO_CENTER + (p.x - cx) * s,
            GALVO_CENTER - (p.y - cy) * s, // y-up
        )
    };

    let mut subpaths = Vec::new();
    for path in paths {
        let mut cur = Point::ZERO;
        let mut start = Point::ZERO;
        let mut cubics: Vec<CubicBez> = Vec::new();
        let mut closed = false;
        let mut flush = |cubics: &mut Vec<CubicBez>, closed: &mut bool| {
            if !cubics.is_empty() {
                subpaths.push(Subpath {
                    color: path.color,
                    closed: *closed,
                    cubics: std::mem::take(cubics),
                });
            }
            *closed = false;
        };
        for el in path.bez.elements() {
            match el {
                PathEl::MoveTo(p) => {
                    flush(&mut cubics, &mut closed);
                    cur = map(*p);
                    start = cur;
                }
                PathEl::LineTo(p) => {
                    let q = map(*p);
                    cubics.push(line_cubic(cur, q));
                    cur = q;
                }
                PathEl::QuadTo(c, p) => {
                    let (c, p) = (map(*c), map(*p));
                    cubics.push(QuadBez::new(cur, c, p).raise());
                    cur = p;
                }
                PathEl::CurveTo(c1, c2, p) => {
                    let (c1, c2, p) = (map(*c1), map(*c2), map(*p));
                    cubics.push(CubicBez::new(cur, c1, c2, p));
                    cur = p;
                }
                PathEl::ClosePath => {
                    if (cur - start).hypot() > 1e-6 {
                        cubics.push(line_cubic(cur, start));
                    }
                    cur = start;
                    closed = true;
                    flush(&mut cubics, &mut closed);
                }
            }
        }
        flush(&mut cubics, &mut closed);
    }
    subpaths
}
