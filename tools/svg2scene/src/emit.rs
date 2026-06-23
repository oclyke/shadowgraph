//! Serialise the interpolated point stream to a standard **ILDA** file (Image
//! Data Transfer Format, format 5 — 2D true colour, big-endian). This is the
//! tool's native output and its wire format: the device parses ILDA directly, so
//! the same bytes that get written to a `.ild` are what `--stream` sends. The
//! ILDA status bits (0x80 last / 0x40 blank) are identical to the firmware's.

use crate::model::{IldaPoint, POINT_BLANK, POINT_LAST};
use crate::optimize::OutPoint;

pub struct EmitOptions {
    /// Counts from field centre to edge for `pos = ±1`. Stay within the galvo
    /// linear region (see `main/main.c` `GALVO_AMPLITUDE`).
    pub amplitude: f64,
    /// Brightness scale applied to all colours (0..1).
    pub intensity: f32,
}

fn to_i16(v_norm: f32, amp: f64) -> i16 {
    (v_norm as f64 * amp)
        .round()
        .clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

fn chan(c: f32, intensity: f32) -> u8 {
    ((c * intensity).clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Map the normalised, interpolated `OutPoint`s to ILDA points in DAC space, and
/// bracket the loop with a blanked point at the start position so the wrap-around
/// seam (device loops the buffer) draws no retrace line.
pub fn build_scene(out: &[OutPoint], opts: &EmitOptions) -> Vec<IldaPoint> {
    if out.is_empty() {
        return Vec::new();
    }
    let blank_at = |p: [f32; 2]| IldaPoint {
        x: to_i16(p[0], opts.amplitude),
        y: to_i16(p[1], opts.amplitude),
        blank: true,
        r: 0,
        g: 0,
        b: 0,
    };

    let first = out[0].pos;
    let mut scene = Vec::with_capacity(out.len() + 2);
    scene.push(blank_at(first)); // open blanked at the start
    for p in out {
        let blank = p.is_blank();
        scene.push(IldaPoint {
            x: to_i16(p.pos[0], opts.amplitude),
            y: to_i16(p.pos[1], opts.amplitude),
            blank,
            r: chan(p.rgb[0], opts.intensity),
            g: chan(p.rgb[1], opts.intensity),
            b: chan(p.rgb[2], opts.intensity),
        });
    }
    scene.push(blank_at(first)); // close blanked back to the start
    scene
}

fn status(p: &IldaPoint, last: bool) -> u8 {
    let mut s = 0u8;
    if p.blank {
        s |= POINT_BLANK;
    }
    if last {
        s |= POINT_LAST;
    }
    s
}

/// Standard ILDA file, format 5 (2D true colour), big-endian. One section plus a
/// zero-record terminating header.
pub fn encode_ild(scene: &[IldaPoint], frame_name: &str) -> Vec<u8> {
    fn header(out: &mut Vec<u8>, format: u8, name: &str, records: u16) {
        out.extend_from_slice(b"ILDA");
        out.extend_from_slice(&[0, 0, 0]); // reserved
        out.push(format);
        let mut pad8 = |s: &str| {
            let mut b = [b' '; 8];
            for (i, c) in s.bytes().take(8).enumerate() {
                b[i] = c;
            }
            out.extend_from_slice(&b);
        };
        pad8(name); // frame name
        pad8("shadowgr"); // company name
        out.extend_from_slice(&records.to_be_bytes()); // number of records
        out.extend_from_slice(&0u16.to_be_bytes()); // frame number
        out.extend_from_slice(&1u16.to_be_bytes()); // total frames
        out.push(0); // projector number
        out.push(0); // reserved
    }

    let n = u16::try_from(scene.len()).unwrap_or(u16::MAX);
    let mut out = Vec::new();
    header(&mut out, 5, frame_name, n);
    for (i, p) in scene.iter().take(n as usize).enumerate() {
        out.extend_from_slice(&p.x.to_be_bytes());
        out.extend_from_slice(&p.y.to_be_bytes());
        out.push(status(p, i + 1 == n as usize));
        out.push(p.b); // ILDA true-colour order is B, G, R
        out.push(p.g);
        out.push(p.r);
    }
    header(&mut out, 5, frame_name, 0); // terminating header
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lit(x: i16, y: i16, r: u8, g: u8, b: u8) -> IldaPoint {
        IldaPoint { x, y, blank: false, r, g, b }
    }

    #[test]
    fn ild_header_and_record() {
        let scene = vec![lit(0x0102, -2, 0xAA, 0xBB, 0xCC)];
        let f = encode_ild(&scene, "test");
        assert_eq!(&f[0..4], b"ILDA");
        assert_eq!(f[7], 5); // format 5
        assert_eq!(&f[24..26], &1u16.to_be_bytes()); // record count BE
        let rec = 32; // first record starts after the 32-byte header
        assert_eq!(&f[rec..rec + 2], &0x0102i16.to_be_bytes()); // X BE
        assert_eq!(&f[rec + 2..rec + 4], &(-2i16).to_be_bytes()); // Y BE
        assert_eq!(f[rec + 4], POINT_LAST);
        assert_eq!([f[rec + 5], f[rec + 6], f[rec + 7]], [0xCC, 0xBB, 0xAA]); // B,G,R
        // terminating header with 0 records at the end.
        let end = f.len() - 32;
        assert_eq!(&f[end..end + 4], b"ILDA");
        assert_eq!(&f[end + 24..end + 26], &0u16.to_be_bytes());
    }
}
