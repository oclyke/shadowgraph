//! Dependency-free SVG debug dumps, one per pipeline stage. Geometry is in the
//! normalised `[-1,1]` field (y-up); we flip y for display. Render to PNG on
//! macOS with `qlmanage -t -s 1000 -o /tmp file.svg`.

use crate::model::{Polyline, Rgb};
use crate::optimize::OutPoint;

const VB: f64 = 1000.0;

/// Normalised [-1,1] (y-up) → SVG viewBox coords (y-down).
fn sx(x: f32) -> f64 {
    (x as f64 + 1.0) * 0.5 * VB
}
fn sy(y: f32) -> f64 {
    (1.0 - (y as f64 + 1.0) * 0.5) * VB
}

fn header() -> String {
    format!(
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {VB} {VB}' \
         width='900' height='900'><rect width='{VB}' height='{VB}' fill='#0a0a0a'/>"
    )
}

fn rgb_css(c: Rgb) -> String {
    format!(
        "rgb({},{},{})",
        (c[0] * 255.0) as u8,
        (c[1] * 255.0) as u8,
        (c[2] * 255.0) as u8
    )
}

fn legend(title: &str) -> String {
    format!(
        "<text x='14' y='985' fill='#aaa' font-size='22' \
         font-family='monospace'>{title}</text>"
    )
}

/// Stage 1: the fitted, flattened polylines in their stroke colours.
pub fn parse_svg(polys: &[Polyline]) -> String {
    let mut s = header();
    for poly in polys {
        if poly.pts.len() < 2 {
            continue;
        }
        let mut d = String::new();
        for (i, p) in poly.pts.iter().enumerate() {
            d += &format!(
                "{}{:.1} {:.1} ",
                if i == 0 { "M " } else { "L " },
                sx(p[0]),
                sy(p[1])
            );
        }
        s += &format!(
            "<path d='{d}' fill='none' stroke='{}' stroke-width='2'/>",
            rgb_css(poly.color)
        );
    }
    s += &legend("parse: flattened polylines, fitted to [-1,1]");
    s + "</svg>"
}

/// Stage 2: the interpolated point stream the device draws — lit dots in colour,
/// blank travels as dashed grey lines, blanked points as small hollow circles.
pub fn points_svg(pts: &[OutPoint]) -> String {
    let mut s = header();
    // travel/connection lines
    for w in pts.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        if a.is_blank() || b.is_blank() {
            s += &format!(
                "<line x1='{:.1}' y1='{:.1}' x2='{:.1}' y2='{:.1}' stroke='#444' \
                 stroke-width='1' stroke-dasharray='6 6'/>",
                sx(a.pos[0]),
                sy(a.pos[1]),
                sx(b.pos[0]),
                sy(b.pos[1])
            );
        } else {
            s += &format!(
                "<line x1='{:.1}' y1='{:.1}' x2='{:.1}' y2='{:.1}' stroke='#222' stroke-width='1'/>",
                sx(a.pos[0]),
                sy(a.pos[1]),
                sx(b.pos[0]),
                sy(b.pos[1])
            );
        }
    }
    for p in pts {
        if p.is_blank() {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='2.2' fill='none' stroke='#555' stroke-width='0.8'/>",
                sx(p.pos[0]),
                sy(p.pos[1])
            );
        } else {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='3' fill='{}'/>",
                sx(p.pos[0]),
                sy(p.pos[1]),
                rgb_css(p.rgb)
            );
        }
    }
    s += &legend("points: emitted setpoints (dashed = blank travel)");
    s + "</svg>"
}
