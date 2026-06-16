//! Draw-order + blanking via `lasy`'s euler-circuit optimisation.
//!
//! lasy only understands points + straight segments, so we feed it each subpath's
//! **knot** points (cubic endpoints) — a coarse, throwaway routing graph — and map
//! the resulting order/orientation back onto the original cubics, which are emitted
//! untouched (curves never flattened). Coincident knots dedup, so subpaths that
//! share an endpoint chain without a blank; everything else gets the minimal blank
//! jumps lasy inserts. See docs/CURVE_MOTION.md §7.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use kurbo::{CubicBez, Point};
use lasy::{
    euler_circuit_to_segments, euler_graph_to_euler_circuit, point_graph_to_euler_graph,
    points_to_segments, segments_to_point_graph, IsBlank, Position, SegmentKind, Weight,
};

use crate::model::{Move, Rgb, Subpath, GALVO_CENTER};

/// A routing-graph point in lasy's normalised [-1,1] space.
#[derive(Clone)]
struct InPoint {
    pos: [f32; 2],
    lit: bool,
}
impl Position for InPoint {
    fn position(&self) -> [f32; 2] {
        self.pos
    }
}
impl IsBlank for InPoint {
    fn is_blank(&self) -> bool {
        !self.lit
    }
}
impl Weight for InPoint {
    fn weight(&self) -> u32 {
        0
    }
}
impl Hash for InPoint {
    fn hash<H: Hasher>(&self, h: &mut H) {
        // Match lasy's position quantisation (i16 range over [-1,1]).
        ((self.pos[0] * 32767.0) as i32).hash(h);
        ((self.pos[1] * 32767.0) as i32).hash(h);
        self.lit.hash(h);
    }
}

fn q(p: [f32; 2]) -> (i32, i32) {
    ((p[0] * 32767.0) as i32, (p[1] * 32767.0) as i32)
}

/// Canonical (unordered) key for the edge between two knot positions.
fn edge_key(a: [f32; 2], b: [f32; 2]) -> [i32; 4] {
    let (qa, qb) = (q(a), q(b));
    if qa <= qb {
        [qa.0, qa.1, qb.0, qb.1]
    } else {
        [qb.0, qb.1, qa.0, qa.1]
    }
}

struct EdgeInfo {
    sub: usize,
    cubic: CubicBez,
    color: Rgb,
}

fn line_cubic(a: Point, b: Point) -> CubicBez {
    CubicBez::new(a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b)
}

/// Reorder + reorient strokes using lasy's euler circuit. Returns strokes in draw
/// order; `build_moves` inserts the blank travels between them.
pub fn order(subpaths: Vec<Subpath>) -> Vec<Subpath> {
    let live: Vec<&Subpath> = subpaths.iter().filter(|s| !s.cubics.is_empty()).collect();
    if live.len() <= 1 {
        return subpaths; // nothing to route
    }

    // Normalise knot coords to [-1,1] by their bounding box (lasy hashes/angles in
    // that space). Geometry itself stays in counts via the cubic lookup.
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for sp in &live {
        for c in &sp.cubics {
            for p in [c.p0, c.p3] {
                lo[0] = lo[0].min(p.x);
                lo[1] = lo[1].min(p.y);
                hi[0] = hi[0].max(p.x);
                hi[1] = hi[1].max(p.y);
            }
        }
    }
    let span = ((hi[0] - lo[0]).max(hi[1] - lo[1])).max(1e-9);
    let norm = |p: Point| -> [f32; 2] {
        [
            (2.0 * (p.x - lo[0]) / span - 1.0) as f32,
            (2.0 * (p.y - lo[1]) / span - 1.0) as f32,
        ]
    };

    // Build the lasy point stream (knots + blank bridges) and the cubic lookup.
    let mut pts: Vec<InPoint> = Vec::new();
    let mut edges: HashMap<[i32; 4], EdgeInfo> = HashMap::new();
    let mut prev_end: Option<[f32; 2]> = None;
    for (si, sp) in live.iter().enumerate() {
        let start = norm(sp.start());
        // Two blank points bridge from the previous stroke so lasy sees no spurious
        // lit edge across the gap (blanks are skipped from the graph and re-derived).
        if let Some(pe) = prev_end {
            pts.push(InPoint { pos: pe, lit: false });
            pts.push(InPoint { pos: start, lit: false });
        }
        pts.push(InPoint { pos: start, lit: true });
        for c in &sp.cubics {
            let a = norm(c.p0);
            let b = norm(c.p3);
            pts.push(InPoint { pos: b, lit: true });
            edges.insert(
                edge_key(a, b),
                EdgeInfo {
                    sub: si,
                    cubic: *c,
                    color: sp.color,
                },
            );
        }
        prev_end = Some(norm(sp.end()));
    }

    // lasy: points -> segments -> point graph -> euler graph -> euler circuit.
    let segs: Vec<_> = points_to_segments(pts.iter().cloned()).collect();
    let pg = segments_to_point_graph(&pts, segs);
    let eg = point_graph_to_euler_graph(&pg);
    let ec = euler_graph_to_euler_circuit(&pts, &eg);
    let out: Vec<_> = euler_circuit_to_segments(&ec, &eg).collect();

    // Walk the circuit, grouping consecutive lit edges of one stroke into a Subpath
    // (oriented to the traversal direction). A blank or a stroke change flushes.
    let mut ordered: Vec<Subpath> = Vec::new();
    let mut run: Vec<CubicBez> = Vec::new();
    let mut run_sub: Option<usize> = None;
    let mut run_color: Rgb = [1.0; 3];
    let flush = |ordered: &mut Vec<Subpath>, run: &mut Vec<CubicBez>, color: Rgb| {
        if !run.is_empty() {
            ordered.push(Subpath {
                color,
                closed: false,
                cubics: std::mem::take(run),
            });
        }
    };

    for seg in &out {
        if seg.kind == SegmentKind::Blank {
            flush(&mut ordered, &mut run, run_color);
            run_sub = None;
            continue;
        }
        let ps = pts[seg.start as usize].pos;
        let pe = pts[seg.end as usize].pos;
        let Some(info) = edges.get(&edge_key(ps, pe)) else {
            continue; // unmapped (shouldn't happen for disjoint strokes)
        };
        // Orient the cubic to the traversal direction.
        let oriented = if q(norm(info.cubic.p0)) == q(ps) {
            info.cubic
        } else {
            CubicBez::new(info.cubic.p3, info.cubic.p2, info.cubic.p1, info.cubic.p0)
        };
        if run_sub != Some(info.sub) {
            flush(&mut ordered, &mut run, run_color);
            run_sub = Some(info.sub);
            run_color = info.color;
        }
        run.push(oriented);
    }
    flush(&mut ordered, &mut run, run_color);

    if ordered.is_empty() {
        return subpaths;
    }
    // lasy optimises the euler traversal (great for connected / shared-vertex
    // geometry — it never retraces a lit line) but not travel *between* disjoint
    // strokes. Finish with a greedy nearest-end reorder of the resulting strokes
    // to cut blank-jump distance.
    greedy_reorder(ordered)
}

/// Order strokes by a greedy nearest-endpoint walk from field centre, reversing a
/// stroke when its far end is closer. Cheap travel-distance reduction.
fn greedy_reorder(mut remaining: Vec<Subpath>) -> Vec<Subpath> {
    if remaining.len() <= 1 {
        return remaining;
    }
    let mut out = Vec::with_capacity(remaining.len());
    let mut cur = Point::new(GALVO_CENTER, GALVO_CENTER);
    while !remaining.is_empty() {
        let mut best = 0usize;
        let mut best_d = f64::MAX;
        let mut rev = false;
        for (i, sp) in remaining.iter().enumerate() {
            let ds = (sp.start() - cur).hypot();
            if ds < best_d {
                best_d = ds;
                best = i;
                rev = false;
            }
            let de = (sp.end() - cur).hypot();
            if de < best_d {
                best_d = de;
                best = i;
                rev = true;
            }
        }
        let sp = remaining.remove(best);
        let sp = if rev { sp.reversed() } else { sp };
        cur = sp.end();
        out.push(sp);
    }
    out
}

/// Flatten ordered strokes into a move list, inserting a blank travel move (beam
/// off) before each stroke that doesn't start where the beam currently is. The
/// first move's P0 is the field centre (the engine's start position).
pub fn build_moves(ordered: &[Subpath]) -> Vec<Move> {
    let mut moves = Vec::new();
    let mut cur = Point::new(GALVO_CENTER, GALVO_CENTER);
    for sp in ordered {
        if sp.cubics.is_empty() {
            continue;
        }
        let start = sp.start();
        if (start - cur).hypot() > 1.0 {
            moves.push(Move {
                cubic: line_cubic(cur, start),
                color: [0.0, 0.0, 0.0],
                blank: true,
                v_in: 0.0,
                v_out: 0.0,
            });
        }
        for c in &sp.cubics {
            moves.push(Move {
                cubic: *c,
                color: sp.color,
                blank: false,
                v_in: 0.0,
                v_out: 0.0,
            });
        }
        cur = sp.end();
    }
    moves
}
