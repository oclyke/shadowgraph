//! Stage 2b: per-point kinematics of the optimised stream (first-class output).
//!
//! After `lasy` interpolation, every output point is one *sample* the beam will
//! visit for one dwell tick. Because every sample takes the same dwell time, the
//! displacement between consecutive samples is directly proportional to the
//! galvo's physical velocity. This module turns the bare point list into
//! [`PointKinematics`] — speed into/out of each point, the turn angle, and the
//! dwell-cluster size — which the debug visualiser and CSV dump both consume.
//!
//! This is what reveals `lasy`'s corner handling: a sharp corner shows up as a
//! run of coincident points (`dwell_run > 1`, `speed ≈ 0`) where the beam is
//! held to let the galvo settle, while a smooth curve stays at near-constant
//! speed with `dwell_run == 1`.

use crate::model::{OutPoint, Rgb};

/// Kinematic profile of a single optimised point.
///
/// Speeds are in **normalised units per sample** (one inter-point step), in the
/// same `[-1, 1]` space as the points. Multiply by the sample rate
/// (`1 / dwell`) to get units per second — see [`PointKinematics::units_per_second`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointKinematics {
    pub index: usize,
    pub pos: [f32; 2],
    pub rgb: Rgb,
    pub blank: bool,
    /// Distance from the previous point (`0` at the first point).
    pub speed_in: f32,
    /// Distance to the next point (`0` at the last point).
    pub speed_out: f32,
    /// Turn angle at this point in radians: deviation from a straight line
    /// between the incoming and outgoing segments. `0` at the ends or where a
    /// neighbouring segment has zero length.
    pub turn: f32,
    /// Number of consecutive points sharing this exact position (`>= 1`).
    /// `> 1` marks a dwell cluster — e.g. a sharp-corner delay from `lasy`.
    pub dwell_run: u32,
    /// Index of this point within its dwell cluster (`0`-based).
    pub dwell_ix: u32,
}

impl PointKinematics {
    /// The lower of the in/out speeds — a good single "how fast here" measure
    /// (it dips to ~0 at corners and end-points).
    pub fn speed(&self) -> f32 {
        self.speed_in.min(self.speed_out)
    }

    /// Outgoing speed converted to normalised units per second for a given
    /// per-point dwell (microseconds).
    pub fn units_per_second(&self, dwell_us: u32) -> f32 {
        if dwell_us == 0 {
            0.0
        } else {
            self.speed_out * (1_000_000.0 / dwell_us as f32)
        }
    }
}

/// Summary statistics over a profiled frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct ProfileStats {
    pub points: usize,
    pub blanks: usize,
    /// Number of *lit* dwell clusters of length > 1 — sharp-corner / accent holds
    /// where the beam is held so the galvo can settle.
    pub corner_holds: usize,
    /// Number of *blank* dwell clusters — blank-delay settle holds at jump ends.
    pub blank_holds: usize,
    /// Largest dwell-cluster length seen (any kind).
    pub max_dwell_run: u32,
    /// Max lit-segment speed (normalised units per sample).
    pub max_lit_speed: f32,
}

/// Compute the kinematic profile of an optimised point stream.
pub fn profile(points: &[OutPoint]) -> Vec<PointKinematics> {
    let n = points.len();
    let mut out = Vec::with_capacity(n);

    // Identify runs of coincident points (dwell clusters).
    let mut run_len = vec![1u32; n];
    let mut run_ix = vec![0u32; n];
    let mut i = 0;
    while i < n {
        let mut j = i + 1;
        while j < n && points[j].pos == points[i].pos {
            j += 1;
        }
        let len = (j - i) as u32;
        for (k, idx) in (i..j).enumerate() {
            run_len[idx] = len;
            run_ix[idx] = k as u32;
        }
        i = j;
    }

    for idx in 0..n {
        let p = points[idx];
        let prev = idx.checked_sub(1).map(|k| points[k].pos);
        let next = points.get(idx + 1).map(|q| q.pos);
        let speed_in = prev.map(|q| dist(q, p.pos)).unwrap_or(0.0);
        let speed_out = next.map(|q| dist(p.pos, q)).unwrap_or(0.0);
        let turn = match (prev, next) {
            (Some(a), Some(c)) => straight_angle_variance(a, p.pos, c),
            _ => 0.0,
        };
        out.push(PointKinematics {
            index: idx,
            pos: p.pos,
            rgb: p.rgb,
            blank: p.rgb == [0.0; 3],
            speed_in,
            speed_out,
            turn,
            dwell_run: run_len[idx],
            dwell_ix: run_ix[idx],
        });
    }
    out
}

/// Summarise a profiled frame.
pub fn stats(prof: &[PointKinematics]) -> ProfileStats {
    let mut s = ProfileStats {
        points: prof.len(),
        ..Default::default()
    };
    for k in prof {
        if k.blank {
            s.blanks += 1;
        } else {
            s.max_lit_speed = s.max_lit_speed.max(k.speed_out);
        }
        s.max_dwell_run = s.max_dwell_run.max(k.dwell_run);
        // Count each cluster once, at its first member.
        if k.dwell_run > 1 && k.dwell_ix == 0 {
            if k.blank {
                s.blank_holds += 1;
            } else {
                s.corner_holds += 1;
            }
        }
    }
    s
}

/// Serialise a profiled frame to CSV (one row per point). `dwell_us` is used to
/// also report outgoing speed in normalised units per second.
pub fn to_csv(prof: &[PointKinematics], dwell_us: u32) -> String {
    let mut s = String::from(
        "index,x,y,blank,speed_in,speed_out,units_per_sec,turn_rad,dwell_run,dwell_ix\n",
    );
    for k in prof {
        s.push_str(&format!(
            "{},{:.6},{:.6},{},{:.6},{:.6},{:.3},{:.6},{},{}\n",
            k.index,
            k.pos[0],
            k.pos[1],
            k.blank as u8,
            k.speed_in,
            k.speed_out,
            k.units_per_second(dwell_us),
            k.turn,
            k.dwell_run,
            k.dwell_ix,
        ));
    }
    s
}

fn dist(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    (dx * dx + dy * dy).sqrt()
}

/// Deviation from a straight line through a -> b -> c, in radians (0 = straight,
/// up to PI = full reversal). Matches `lasy`'s corner metric.
fn straight_angle_variance([ax, ay]: [f32; 2], [bx, by]: [f32; 2], [cx, cy]: [f32; 2]) -> f32 {
    let (ux, uy) = (bx - ax, by - ay);
    let (vx, vy) = (cx - bx, cy - by);
    if (ux == 0.0 && uy == 0.0) || (vx == 0.0 && vy == 0.0) {
        return 0.0;
    }
    let diff = vy.atan2(vx) - uy.atan2(ux);
    let d = diff.abs();
    if d > std::f32::consts::PI {
        std::f32::consts::PI * 2.0 - d
    } else {
        d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f32, y: f32) -> OutPoint {
        OutPoint {
            pos: [x, y],
            rgb: [1.0, 1.0, 1.0],
        }
    }

    #[test]
    fn detects_dwell_cluster_at_repeated_point() {
        // A corner hold: the middle vertex is repeated 3x.
        let pts = vec![p(-1.0, 0.0), p(0.0, 0.0), p(0.0, 0.0), p(0.0, 0.0), p(0.0, 1.0)];
        let prof = profile(&pts);
        // The three coincident points form one cluster of length 3.
        assert_eq!(prof[1].dwell_run, 3);
        assert_eq!(prof[2].dwell_run, 3);
        assert_eq!(prof[1].dwell_ix, 0);
        assert_eq!(prof[3].dwell_ix, 2);
        // Speed through the held vertex is zero.
        assert_eq!(prof[2].speed(), 0.0);
        let s = stats(&prof);
        assert_eq!(s.corner_holds, 1); // lit cluster
        assert_eq!(s.blank_holds, 0);
        assert_eq!(s.max_dwell_run, 3);
    }

    #[test]
    fn straight_run_has_no_turn() {
        let pts = vec![p(-1.0, 0.0), p(0.0, 0.0), p(1.0, 0.0)];
        let prof = profile(&pts);
        assert!(prof[1].turn.abs() < 1e-6);
        assert!((prof[1].speed_in - 1.0).abs() < 1e-6);
    }

    #[test]
    fn right_angle_turn_is_half_pi() {
        let pts = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0)];
        let prof = profile(&pts);
        assert!((prof[1].turn - std::f32::consts::FRAC_PI_2).abs() < 1e-5);
    }
}
