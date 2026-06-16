//! Encode the planned move list into the firmware TV wire format.
//!
//! Every move is a `CURVE` (P0 implicit = previous P3, so the first move starts
//! at the engine's centre). A `LASER` precedes a move only when the colour
//! changes; a blank travel move is colour 0. Bytes match `laser_command`:
//!   LASER 0x02 : r,g,b u16            (7 bytes)
//!   CURVE 0x04 : P1,P2,P3 u16 ; v_in,v_out u32   (21 bytes), little-endian.
//! v_in/v_out are tick-native (counts/tick * 256); the planner's counts/s cross to
//! that format here via `cps_to_wire` — the one host-side physical->tick step.

use kurbo::Point;

use crate::interp::{cps_to_wire, CurveLimits};
use crate::model::Move;

pub struct EmitOptions {
    pub intensity: f32,
}

fn u16c(v: f64) -> u16 {
    v.round().clamp(0.0, 65535.0) as u16
}

fn col(rgb: [f32; 3], intensity: f32) -> [u16; 3] {
    let s = |c: f32| ((c * intensity).clamp(0.0, 1.0) * 65535.0).round() as u16;
    [s(rgb[0]), s(rgb[1]), s(rgb[2])]
}

fn put16(o: &mut Vec<u8>, v: u16) {
    o.extend_from_slice(&v.to_le_bytes());
}
fn put32(o: &mut Vec<u8>, v: u32) {
    o.extend_from_slice(&v.to_le_bytes());
}
fn putp(o: &mut Vec<u8>, p: Point) {
    put16(o, u16c(p.x));
    put16(o, u16c(p.y));
}

/// Encode the whole scene to wire bytes. `lim` supplies the tick duration used to
/// convert the planner's counts/s speeds into the wire's counts/tick format.
pub fn encode_scene(moves: &[Move], lim: &CurveLimits, opts: &EmitOptions) -> Vec<u8> {
    let dt = lim.dt_tick_us;
    let mut out = Vec::new();
    let mut cur_color: Option<[u16; 3]> = None;
    for m in moves {
        let color = if m.blank {
            [0u16; 3]
        } else {
            col(m.color, opts.intensity)
        };
        if cur_color != Some(color) {
            out.push(0x02); // LASER
            put16(&mut out, color[0]);
            put16(&mut out, color[1]);
            put16(&mut out, color[2]);
            cur_color = Some(color);
        }
        out.push(0x04); // CURVE (P0 implicit)
        putp(&mut out, m.cubic.p1);
        putp(&mut out, m.cubic.p2);
        putp(&mut out, m.cubic.p3);
        put32(&mut out, cps_to_wire(m.v_in, dt));
        put32(&mut out, cps_to_wire(m.v_out, dt));
    }
    out
}

/// Number of CURVE records (for reporting).
pub fn curve_count(moves: &[Move]) -> usize {
    moves.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{CubicBez, Point};

    #[test]
    fn curve_wire_layout() {
        let m = Move {
            cubic: CubicBez::new(
                Point::new(0.0, 0.0),
                Point::new(100.0, 200.0),
                Point::new(300.0, 400.0),
                Point::new(500.0, 600.0),
            ),
            color: [1.0, 0.0, 0.0],
            blank: false,
            v_in: 1000.0,
            v_out: 2000.0,
        };
        let lim = CurveLimits {
            v_max_cps: 11_468_800,
            a_max_cps2: 57_344_000_000,
            dt_tick_us: 50,
        };
        let b = encode_scene(&[m], &lim, &EmitOptions { intensity: 1.0 });
        // LASER (colour changed from none): 0x02 + r,g,b u16 = 7 bytes, then CURVE.
        assert_eq!(b[0], 0x02);
        assert_eq!(u16::from_le_bytes([b[1], b[2]]), 0xFFFF); // red full
        assert_eq!(u16::from_le_bytes([b[3], b[4]]), 0);
        let c = 7;
        assert_eq!(b[c], 0x04); // CURVE
        assert_eq!(u16::from_le_bytes([b[c + 1], b[c + 2]]), 100); // P1.x
        assert_eq!(u16::from_le_bytes([b[c + 3], b[c + 4]]), 200); // P1.y
        assert_eq!(u16::from_le_bytes([b[c + 9], b[c + 10]]), 500); // P3.x
        assert_eq!(u16::from_le_bytes([b[c + 11], b[c + 12]]), 600); // P3.y
        // Speeds are encoded in wire units (counts/tick * 256), not raw counts/s.
        assert_eq!(
            u32::from_le_bytes([b[c + 13], b[c + 14], b[c + 15], b[c + 16]]),
            cps_to_wire(1000.0, lim.dt_tick_us)
        );
        assert_eq!(
            u32::from_le_bytes([b[c + 17], b[c + 18], b[c + 19], b[c + 20]]),
            cps_to_wire(2000.0, lim.dt_tick_us)
        );
        assert_eq!(b.len(), 7 + 21); // P0 implicit
    }
}
