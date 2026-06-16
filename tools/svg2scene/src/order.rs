//! Draw-order + blanking.
//!
//! v1 uses a greedy nearest-endpoint walk (reversing strokes when their far end
//! is closer), inserting a blank travel move between strokes — this gives the
//! ordering + minimal-ish blanking we want. The `lasy` euler-circuit (kept as a
//! dependency) can replace this later for shared-vertex figures; the interface
//! (`Vec<Subpath>` in, ordered `Vec<Subpath>` out) stays the same.

use kurbo::{CubicBez, Point};

use crate::model::{Move, Subpath, GALVO_CENTER};

fn line_cubic(a: Point, b: Point) -> CubicBez {
    CubicBez::new(a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b)
}

/// Greedy nearest-neighbour ordering starting from field centre.
pub fn order(mut remaining: Vec<Subpath>) -> Vec<Subpath> {
    let mut ordered = Vec::with_capacity(remaining.len());
    let mut cur = Point::new(GALVO_CENTER, GALVO_CENTER);
    while !remaining.is_empty() {
        let mut best = 0usize;
        let mut best_d = f64::MAX;
        let mut best_rev = false;
        for (i, sp) in remaining.iter().enumerate() {
            let ds = (sp.start() - cur).hypot();
            if ds < best_d {
                best_d = ds;
                best = i;
                best_rev = false;
            }
            let de = (sp.end() - cur).hypot();
            if de < best_d {
                best_d = de;
                best = i;
                best_rev = true;
            }
        }
        let sp = remaining.remove(best);
        let sp = if best_rev { sp.reversed() } else { sp };
        cur = sp.end();
        ordered.push(sp);
    }
    ordered
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
