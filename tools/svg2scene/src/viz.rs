//! Per-stage debug visualisation.
//!
//! Every stage can be dumped to a standalone SVG (no extra dependencies, opens
//! in any browser):
//!
//! * [`subpaths_svg`] — the flattened, coloured polylines from [`crate::parse`].
//! * [`optimized_svg`] — the optimised point stream: lit segments in colour,
//!   blank (travel) segments as faint dashed grey, every output point as a dot.
//!   This is the view that shows blanking and corner-dwell clustering.
//!
//! Both auto-fit the data into a fixed canvas and flip Y for natural display.

use crate::model::{bounds, OutPoint, Rgb, Subpath};

const CANVAS: f64 = 1000.0;
const PAD: f64 = 20.0;

/// Maps data coordinates into canvas pixels, preserving aspect ratio and
/// flipping Y so larger Y is drawn higher.
struct Canvas {
    min: [f64; 2],
    scale: f64,
    // offsets to centre the content
    ox: f64,
    oy: f64,
    h: f64,
}

impl Canvas {
    fn new(min: [f64; 2], max: [f64; 2]) -> Self {
        let span_x = (max[0] - min[0]).max(f64::EPSILON);
        let span_y = (max[1] - min[1]).max(f64::EPSILON);
        let usable = CANVAS - 2.0 * PAD;
        let scale = usable / span_x.max(span_y);
        // Centre the smaller dimension.
        let ox = PAD + (usable - span_x * scale) / 2.0;
        let oy = PAD + (usable - span_y * scale) / 2.0;
        Canvas {
            min,
            scale,
            ox,
            oy,
            h: CANVAS,
        }
    }

    fn map(&self, [x, y]: [f64; 2]) -> (f64, f64) {
        let px = self.ox + (x - self.min[0]) * self.scale;
        let py_up = self.oy + (y - self.min[1]) * self.scale;
        // Flip vertically for display (SVG y is down).
        (px, self.h - py_up)
    }
}

fn hex(rgb: Rgb) -> String {
    let c = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    format!("#{:02x}{:02x}{:02x}", c(rgb[0]), c(rgb[1]), c(rgb[2]))
}

fn header(buf: &mut String) {
    buf.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{CANVAS}\" height=\"{CANVAS}\" \
         viewBox=\"0 0 {CANVAS} {CANVAS}\">\n"
    ));
    // Dark background so coloured strokes read well.
    buf.push_str(&format!(
        "<rect width=\"{CANVAS}\" height=\"{CANVAS}\" fill=\"#101014\"/>\n"
    ));
}

/// Render flattened subpaths as coloured polylines with vertex dots.
pub fn subpaths_svg(subpaths: &[Subpath]) -> String {
    let mut buf = String::new();
    header(&mut buf);
    if let Some((min, max)) = bounds(subpaths) {
        let cv = Canvas::new(min, max);
        for sp in subpaths {
            let pts: Vec<String> = sp
                .points
                .iter()
                .map(|&p| {
                    let (x, y) = cv.map(p);
                    format!("{x:.2},{y:.2}")
                })
                .collect();
            buf.push_str(&format!(
                "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"1.5\" \
                 points=\"{}\"/>\n",
                hex(sp.color),
                pts.join(" ")
            ));
            for &p in &sp.points {
                let (x, y) = cv.map(p);
                buf.push_str(&format!(
                    "<circle cx=\"{x:.2}\" cy=\"{y:.2}\" r=\"1.6\" fill=\"{}\"/>\n",
                    hex(sp.color)
                ));
            }
        }
    }
    buf.push_str("</svg>\n");
    buf
}

/// Render the optimised point stream, distinguishing lit and blank segments.
pub fn optimized_svg(points: &[OutPoint]) -> String {
    let mut buf = String::new();
    header(&mut buf);

    // Compute bounds over the point positions.
    if !points.is_empty() {
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for p in points {
            min[0] = min[0].min(p.pos[0] as f64);
            min[1] = min[1].min(p.pos[1] as f64);
            max[0] = max[0].max(p.pos[0] as f64);
            max[1] = max[1].max(p.pos[1] as f64);
        }
        let cv = Canvas::new(min, max);

        // Segments between consecutive points.
        for w in points.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (ax, ay) = cv.map([a.pos[0] as f64, a.pos[1] as f64]);
            let (bx, by) = cv.map([b.pos[0] as f64, b.pos[1] as f64]);
            let a_blank = a.rgb == [0.0; 3];
            let b_blank = b.rgb == [0.0; 3];
            if a_blank && b_blank {
                // travel / blank segment
                buf.push_str(&format!(
                    "<line x1=\"{ax:.2}\" y1=\"{ay:.2}\" x2=\"{bx:.2}\" y2=\"{by:.2}\" \
                     stroke=\"#444\" stroke-width=\"0.8\" stroke-dasharray=\"3 3\"/>\n"
                ));
            } else {
                buf.push_str(&format!(
                    "<line x1=\"{ax:.2}\" y1=\"{ay:.2}\" x2=\"{bx:.2}\" y2=\"{by:.2}\" \
                     stroke=\"{}\" stroke-width=\"1.5\"/>\n",
                    hex(b.rgb)
                ));
            }
        }
        // Output points as dots: blank points red-ringed so dwell clusters show.
        for p in points {
            let (x, y) = cv.map([p.pos[0] as f64, p.pos[1] as f64]);
            if p.rgb == [0.0; 3] {
                buf.push_str(&format!(
                    "<circle cx=\"{x:.2}\" cy=\"{y:.2}\" r=\"1.4\" fill=\"none\" \
                     stroke=\"#cc3333\" stroke-width=\"0.6\"/>\n"
                ));
            } else {
                buf.push_str(&format!(
                    "<circle cx=\"{x:.2}\" cy=\"{y:.2}\" r=\"1.4\" fill=\"{}\"/>\n",
                    hex(p.rgb)
                ));
            }
        }
    }
    buf.push_str("</svg>\n");
    buf
}
