//! Per-stage debug visualisation.
//!
//! Every stage can be dumped to a standalone SVG (no extra dependencies, opens
//! in any browser):
//!
//! * [`subpaths_svg`] — the flattened, coloured polylines from [`crate::parse`].
//! * [`optimized_svg`] — the optimised point stream (post-`lasy`, pre-emit): lit
//!   segments in colour, blank (travel) segments as faint dashed grey, and the
//!   control points rendered according to a [`PointStyle`]. The velocity and
//!   dwell styles fan out coincident points so `lasy`'s corner-dwell clusters
//!   become visible.
//!
//! Both auto-fit the data into a fixed canvas and flip Y for natural display.

use crate::analyze::{self, PointKinematics};
use crate::model::{bounds, OutPoint, Rgb, Subpath};

const CANVAS: f64 = 1000.0;
const PAD: f64 = 20.0;

/// How control points are drawn in [`optimized_svg`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointStyle {
    /// No point markers — just the path.
    None,
    /// A dot per point in the point's own colour (blank points ringed red).
    Dot,
    /// Heat-map each point by local beam speed (slow = red … fast = blue) and
    /// fan out coincident dwell points so corner holds are visible.
    Velocity,
    /// Emphasise dwell clusters: ring radius grows with cluster size; corner
    /// holds stand out. Points fanned out so the count is legible.
    Dwell,
}

/// Maps data coordinates into canvas pixels, preserving aspect ratio and
/// flipping Y so larger Y is drawn higher.
struct Canvas {
    min: [f64; 2],
    scale: f64,
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
                "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"1.5\" points=\"{}\"/>\n",
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

/// Heat colour for a normalised speed `t` in `0..=1` (slow -> fast).
fn heat(t: f32) -> Rgb {
    // red -> orange -> green -> blue
    const STOPS: [(f32, Rgb); 4] = [
        (0.00, [1.0, 0.20, 0.20]),
        (0.33, [1.0, 0.80, 0.15]),
        (0.66, [0.30, 1.0, 0.35]),
        (1.00, [0.30, 0.55, 1.0]),
    ];
    let t = t.clamp(0.0, 1.0);
    for w in STOPS.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        if t <= t1 {
            let f = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
            return [
                c0[0] + (c1[0] - c0[0]) * f,
                c0[1] + (c1[1] - c0[1]) * f,
                c0[2] + (c1[2] - c0[2]) * f,
            ];
        }
    }
    STOPS[STOPS.len() - 1].1
}

/// Small spiral offset (in px) so coincident dwell points fan out and become
/// individually visible. Uses the golden angle for even spreading.
fn fan_offset(k: u32, run: u32) -> (f64, f64) {
    if run <= 1 {
        return (0.0, 0.0);
    }
    let ang = k as f64 * 2.399_963_23; // golden angle (radians)
    let r = 3.0 * (k as f64).sqrt();
    (r * ang.cos(), r * ang.sin())
}

fn draw_segments(buf: &mut String, cv: &Canvas, points: &[OutPoint]) {
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (ax, ay) = cv.map([a.pos[0] as f64, a.pos[1] as f64]);
        let (bx, by) = cv.map([b.pos[0] as f64, b.pos[1] as f64]);
        if a.rgb == [0.0; 3] && b.rgb == [0.0; 3] {
            buf.push_str(&format!(
                "<line x1=\"{ax:.2}\" y1=\"{ay:.2}\" x2=\"{bx:.2}\" y2=\"{by:.2}\" \
                 stroke=\"#444\" stroke-width=\"0.8\" stroke-dasharray=\"3 3\"/>\n"
            ));
        } else {
            buf.push_str(&format!(
                "<line x1=\"{ax:.2}\" y1=\"{ay:.2}\" x2=\"{bx:.2}\" y2=\"{by:.2}\" \
                 stroke=\"{}\" stroke-width=\"1.2\"/>\n",
                hex(b.rgb)
            ));
        }
    }
}

/// Render the optimised point stream with the requested control-point style.
pub fn optimized_svg(points: &[OutPoint], style: PointStyle) -> String {
    let mut buf = String::new();
    header(&mut buf);

    if points.is_empty() {
        buf.push_str("</svg>\n");
        return buf;
    }

    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    for p in points {
        min[0] = min[0].min(p.pos[0] as f64);
        min[1] = min[1].min(p.pos[1] as f64);
        max[0] = max[0].max(p.pos[0] as f64);
        max[1] = max[1].max(p.pos[1] as f64);
    }
    let cv = Canvas::new(min, max);

    draw_segments(&mut buf, &cv, points);

    if style == PointStyle::None {
        buf.push_str("</svg>\n");
        return buf;
    }

    let prof = analyze::profile(points);
    // Contrast-stretch the heat scale across the observed lit speed range (so
    // small but real velocity variation is visible). Blank points are excluded
    // — they are rendered dim, not as part of the speed gradient.
    let mut lo = f32::INFINITY;
    let mut hi = 0.0f32;
    for k in prof.iter().filter(|k| !k.blank) {
        let s = k.speed();
        lo = lo.min(s);
        hi = hi.max(s);
    }
    if !lo.is_finite() {
        lo = 0.0;
    }
    let span = (hi - lo).max(f32::EPSILON);

    for k in &prof {
        marker(&mut buf, &cv, k, style, lo, span);
    }

    legend(&mut buf, style);
    buf.push_str("</svg>\n");
    buf
}

fn marker(buf: &mut String, cv: &Canvas, k: &PointKinematics, style: PointStyle, lo: f32, span: f32) {
    let (cx0, cy0) = cv.map([k.pos[0] as f64, k.pos[1] as f64]);
    let (dx, dy) = fan_offset(k.dwell_ix, k.dwell_run);
    let (cx, cy) = (cx0 + dx, cy0 + dy);

    match style {
        PointStyle::None => {}
        PointStyle::Dot => {
            if k.blank {
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"1.4\" fill=\"none\" \
                     stroke=\"#cc3333\" stroke-width=\"0.6\"/>\n"
                ));
            } else {
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"1.4\" fill=\"{}\"/>\n",
                    hex(k.rgb)
                ));
            }
        }
        PointStyle::Velocity => {
            if k.blank {
                // Blank travel/settle points are not part of the speed story.
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"1.0\" fill=\"#555\"/>\n"
                ));
                return;
            }
            // Contrast-stretched speed: 0 = slow (corner hold) … 1 = fast.
            let t = ((k.speed() - lo) / span).clamp(0.0, 1.0);
            // Slow points get a touch larger so corner holds read as hot blobs.
            let r = 1.4 + (1.0 - t) as f64 * 2.2;
            buf.push_str(&format!(
                "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"{r:.2}\" fill=\"{}\" \
                 fill-opacity=\"0.9\"/>\n",
                hex(heat(t))
            ));
        }
        PointStyle::Dwell => {
            let r = 1.4 + (k.dwell_run.saturating_sub(1) as f64) * 1.1;
            if k.dwell_run > 1 {
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"{r:.2}\" fill=\"none\" \
                     stroke=\"#ffcc33\" stroke-width=\"0.9\"/>\n"
                ));
            } else if k.blank {
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"1.2\" fill=\"none\" \
                     stroke=\"#cc3333\" stroke-width=\"0.6\"/>\n"
                ));
            } else {
                buf.push_str(&format!(
                    "<circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"1.2\" fill=\"{}\"/>\n",
                    hex(k.rgb)
                ));
            }
        }
    }
}

fn legend(buf: &mut String, style: PointStyle) {
    let text = match style {
        PointStyle::Velocity => "velocity: red=slow (corner hold)  blue=fast / blank jump",
        PointStyle::Dwell => "dwell: yellow ring radius ∝ corner-hold sample count",
        PointStyle::Dot => "dot: point colour; red ring = blank point",
        PointStyle::None => return,
    };
    buf.push_str(&format!(
        "<text x=\"{PAD}\" y=\"{}\" fill=\"#aaa\" font-family=\"monospace\" \
         font-size=\"14\">{text}</text>\n",
        CANVAS - 12.0
    ));
}
