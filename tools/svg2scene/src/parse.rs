//! SVG → coloured polylines, normalised to `[-1,1]`.
//!
//! `usvg` resolves units/styles/transforms and yields absolute path geometry;
//! `kurbo` flattens every Bézier to line segments (the only "curve handling" we
//! want — pure geometry, no dynamics). Everything is then fitted into the
//! normalised `[-1,1]` field `lasy` reasons about (y-up; SVG is y-down).

use kurbo::{BezPath, PathEl, Point, Rect, Shape};

use crate::model::{Polyline, Rgb};

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
                    S::CubicTo(a, b, c) => bez.curve_to(map(a.x, a.y), map(b.x, b.y), map(c.x, c.y)),
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

/// Flatten every path to polylines and fit into the normalised `[-1,1]` field:
/// uniform scale so the larger bbox dimension fills the field (minus `margin`),
/// centred at 0, y flipped (SVG y-down → field y-up). `flatten_tol` is given in
/// **normalised units** (fraction of the half-field) and converted to SVG units
/// internally, so it means the same thing regardless of the source SVG's scale.
pub fn flatten_and_fit(paths: &[RawPath], flatten_tol: f64, margin: f64) -> Vec<Polyline> {
    let mut bbox: Option<Rect> = None;
    for p in paths {
        let bb = p.bez.bounding_box();
        bbox = Some(match bbox {
            Some(acc) => acc.union(bb),
            None => bb,
        });
    }
    let Some(bb) = bbox else { return Vec::new() };

    let span = bb.width().max(bb.height()).max(1e-9);
    let scale = (1.0 - margin) * 2.0 / span; // larger dim → full field width (2.0)
    let (cx, cy) = (bb.center().x, bb.center().y);
    let map = |p: Point| -> [f32; 2] {
        [
            ((p.x - cx) * scale) as f32,
            (-(p.y - cy) * scale) as f32, // y-up
        ]
    };
    // flatten_tol is normalised (field half-width = 1.0); convert back to SVG units.
    let tol_svg = (flatten_tol * span / 2.0).max(1e-9);

    let mut out = Vec::new();
    for path in paths {
        // Flatten to a flat list of MoveTo/LineTo/ClosePath elements first, then
        // split into polylines on MoveTo (avoids borrow tangles in the callback).
        let mut els: Vec<PathEl> = Vec::new();
        kurbo::flatten(path.bez.iter(), tol_svg, |el| els.push(el));

        let mut run: Vec<[f32; 2]> = Vec::new();
        let flush = |run: &mut Vec<[f32; 2]>, out: &mut Vec<Polyline>| {
            if run.len() >= 2 {
                out.push(Polyline {
                    color: path.color,
                    pts: std::mem::take(run),
                });
            } else {
                run.clear();
            }
        };
        for el in els {
            match el {
                PathEl::MoveTo(p) => {
                    flush(&mut run, &mut out);
                    run.push(map(p));
                }
                PathEl::LineTo(p) => run.push(map(p)),
                PathEl::ClosePath => {
                    if let Some(&first) = run.first() {
                        run.push(first); // close the loop with a final segment
                    }
                    flush(&mut run, &mut out);
                }
                _ => {}
            }
        }
        flush(&mut run, &mut out);
    }
    out
}
