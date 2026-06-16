//! Dependency-free SVG debug dumps, one per pipeline stage. Render to PNG on
//! macOS with `qlmanage -t -s 1000 -o /tmp file.svg`. Geometry is in DAC counts
//! (0..65535); y is flipped for display (counts are y-up, SVG is y-down).

use kurbo::CubicBez;

use crate::analyze::SimPoint;
use crate::interp::CurveLimits;
use crate::model::{Move, Rgb, Subpath};

const VB: f64 = 65536.0;

fn fy(y: f64) -> f64 {
    VB - y
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

fn cubic_d(c: &CubicBez) -> String {
    format!(
        "M {:.1} {:.1} C {:.1} {:.1} {:.1} {:.1} {:.1} {:.1}",
        c.p0.x,
        fy(c.p0.y),
        c.p1.x,
        fy(c.p1.y),
        c.p2.x,
        fy(c.p2.y),
        c.p3.x,
        fy(c.p3.y)
    )
}

/// Speed fraction -> colour. Slow = red, mid = green, fast (v_max) = blue.
fn heat(frac: f64) -> String {
    let f = frac.clamp(0.0, 1.0);
    let r = ((1.0 - f) * 255.0) as u8;
    let b = (f * 255.0) as u8;
    let g = ((1.0 - (2.0 * f - 1.0).abs()) * 200.0) as u8;
    format!("rgb({r},{g},{b})")
}

fn legend(title: &str) -> String {
    format!(
        "<text x='400' y='1600' fill='#888' font-size='1400' \
         font-family='monospace'>{title}</text>"
    )
}

/// Stage 1: the fitted cubics in their stroke colours.
pub fn parse_svg(subs: &[Subpath]) -> String {
    let mut s = header();
    for sp in subs {
        let col = rgb_css(sp.color);
        for c in &sp.cubics {
            s += &format!(
                "<path d='{}' fill='none' stroke='{col}' stroke-width='160'/>",
                cubic_d(c)
            );
        }
    }
    s += &legend("parse: fitted cubics");
    s + "</svg>"
}

/// Stage 2: ordered strokes (in colour) with blank travel moves dashed grey.
pub fn order_svg(moves: &[Move]) -> String {
    let mut s = header();
    for m in moves {
        if m.blank {
            s += &format!(
                "<path d='{}' fill='none' stroke='#555' stroke-width='90' \
                 stroke-dasharray='400 400'/>",
                cubic_d(&m.cubic)
            );
        } else {
            s += &format!(
                "<path d='{}' fill='none' stroke='{}' stroke-width='160'/>",
                cubic_d(&m.cubic),
                rgb_css(m.color)
            );
        }
    }
    s += &legend("order: strokes + blank travels (dashed)");
    s + "</svg>"
}

/// Stage 3: segmented cubics with a dot at every segment boundary (the splits).
pub fn segment_svg(subs: &[Subpath]) -> String {
    let mut s = header();
    for sp in subs {
        let col = rgb_css(sp.color);
        for c in &sp.cubics {
            s += &format!(
                "<path d='{}' fill='none' stroke='{col}' stroke-width='120'/>",
                cubic_d(c)
            );
        }
        for c in &sp.cubics {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='220' fill='#fff'/>",
                c.p0.x,
                fy(c.p0.y)
            );
        }
        if let Some(last) = sp.cubics.last() {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='220' fill='#fff'/>",
                last.p3.x,
                fy(last.p3.y)
            );
        }
    }
    s += &legend("segment: monotone-curvature pieces (dots = splits)");
    s + "</svg>"
}

/// Stage 4 (the key one): cubics coloured by PLANNED speed after the fwd/bwd
/// passes, with a dot at every control point coloured by its junction velocity.
pub fn plan_svg(moves: &[Move], vj: &[f64], lim: &CurveLimits) -> String {
    let vmax = lim.v_max_cps as f64;
    let mut s = header();
    // Each lit segment is drawn with a gradient from its v_in colour to its v_out
    // colour (honest: a straight edge shows slow->fast->slow only in points.svg,
    // but the planned endpoint speeds are exact here). Blank travels: dashed grey.
    let mut defs = String::from("<defs>");
    let mut body = String::new();
    for (i, m) in moves.iter().enumerate() {
        if m.blank {
            body += &format!(
                "<path d='{}' fill='none' stroke='#444' stroke-width='160' \
                 stroke-dasharray='400 400'/>",
                cubic_d(&m.cubic)
            );
            continue;
        }
        let id = format!("g{i}");
        defs += &format!(
            "<linearGradient id='{id}' gradientUnits='userSpaceOnUse' \
             x1='{:.1}' y1='{:.1}' x2='{:.1}' y2='{:.1}'>\
             <stop offset='0' stop-color='{}'/><stop offset='1' stop-color='{}'/>\
             </linearGradient>",
            m.cubic.p0.x,
            fy(m.cubic.p0.y),
            m.cubic.p3.x,
            fy(m.cubic.p3.y),
            heat(m.v_in / vmax),
            heat(m.v_out / vmax)
        );
        body += &format!(
            "<path d='{}' fill='none' stroke='url(#{id})' stroke-width='200'/>",
            cubic_d(&m.cubic)
        );
    }
    defs += "</defs>";
    s += &defs;
    s += &body;
    // Junction velocity dots (this is the post-look-ahead velocity at every node).
    for (i, m) in moves.iter().enumerate() {
        let p = m.cubic.p0;
        let frac = vj[i] / vmax;
        s += &format!(
            "<circle cx='{:.1}' cy='{:.1}' r='320' fill='{}'/>",
            p.x,
            fy(p.y),
            heat(frac)
        );
    }
    if let Some(last) = moves.last() {
        let frac = vj[moves.len()] / vmax;
        s += &format!(
            "<circle cx='{:.1}' cy='{:.1}' r='320' fill='{}'/>",
            last.cubic.p3.x,
            fy(last.cubic.p3.y),
            heat(frac)
        );
    }
    s += &legend("plan: speed after fwd/bwd passes (red=slow blue=v_max)");
    s + "</svg>"
}

/// Stage 5: the actual emitted setpoints (FFI sim), each a dot coloured by speed.
pub fn points_svg(pts: &[SimPoint], lim: &CurveLimits) -> String {
    let vmax = lim.v_max_cps as f64;
    let mut s = header();
    // faint path first
    for w in pts.windows(2) {
        if w[0].blank || w[1].blank {
            continue;
        }
        s += &format!(
            "<line x1='{:.1}' y1='{:.1}' x2='{:.1}' y2='{:.1}' stroke='#222' stroke-width='60'/>",
            w[0].pos[0],
            fy(w[0].pos[1]),
            w[1].pos[0],
            fy(w[1].pos[1])
        );
    }
    for p in pts {
        if p.blank {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='120' fill='none' stroke='#444' stroke-width='40'/>",
                p.pos[0],
                fy(p.pos[1])
            );
        } else {
            s += &format!(
                "<circle cx='{:.1}' cy='{:.1}' r='150' fill='{}'/>",
                p.pos[0],
                fy(p.pos[1]),
                heat(p.v_cps as f64 / vmax)
            );
        }
    }
    s += &legend("points: emitted setpoints, coloured by speed");
    s + "</svg>"
}

/// Stage 6: speed-vs-arclength profile against v_max.
pub fn profile_svg(pts: &[SimPoint], lim: &CurveLimits) -> String {
    let vmax = lim.v_max_cps as f64;
    let (w, h, pad) = (60000.0, 60000.0, 2800.0);
    let total: f64 = pts
        .windows(2)
        .map(|p| {
            let dx = p[1].pos[0] - p[0].pos[0];
            let dy = p[1].pos[1] - p[0].pos[1];
            (dx * dx + dy * dy).sqrt()
        })
        .sum::<f64>()
        .max(1.0);
    let mut s = header();
    // axes
    s += &format!(
        "<line x1='{pad}' y1='{}' x2='{}' y2='{}' stroke='#555' stroke-width='60'/>",
        h + pad,
        w + pad,
        h + pad
    );
    // v_max line
    s += &format!(
        "<line x1='{pad}' y1='{pad}' x2='{}' y2='{pad}' stroke='#822' stroke-width='50' \
         stroke-dasharray='300 300'/>",
        w + pad
    );
    s += &format!(
        "<text x='{}' y='{}' fill='#a44' font-size='1300' font-family='monospace'>v_max</text>",
        w + pad - 9000.0,
        pad - 400.0
    );
    // profile polyline
    let mut cum = 0.0;
    let mut d = String::new();
    for (i, p) in pts.iter().enumerate() {
        if i > 0 {
            let dx = p.pos[0] - pts[i - 1].pos[0];
            let dy = p.pos[1] - pts[i - 1].pos[1];
            cum += (dx * dx + dy * dy).sqrt();
        }
        let x = pad + cum / total * w;
        let y = pad + (1.0 - (p.v_cps as f64 / vmax).clamp(0.0, 1.0)) * h;
        d += &format!("{}{:.0},{:.0} ", if i == 0 { "M " } else { "L " }, x, y);
    }
    s += &format!(
        "<path d='{d}' fill='none' stroke='#4cf' stroke-width='90'/>"
    );
    s += &legend("profile: speed vs arc length (dashed = v_max)");
    s + "</svg>"
}

/// One CSV row per emitted setpoint.
pub fn to_csv(pts: &[SimPoint], lim: &CurveLimits) -> String {
    let dt = lim.dt_tick_us as f64 * 1e-6;
    let mut s = String::from("index,t_s,x,y,blank,v_cps,v_frac\n");
    let vmax = lim.v_max_cps as f64;
    for (i, p) in pts.iter().enumerate() {
        s += &format!(
            "{i},{:.6},{:.1},{:.1},{},{},{:.4}\n",
            i as f64 * dt,
            p.pos[0],
            p.pos[1],
            p.blank as u8,
            p.v_cps,
            p.v_cps as f64 / vmax
        );
    }
    s
}
