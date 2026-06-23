//! Serialise the interpolated point stream to the two output formats:
//!   * **device blob** — back-to-back 8-byte `laser_point_t` records,
//!     little-endian, byte-identical to the firmware struct so the device can
//!     `memcpy` it straight into its DRAM scene buffer.
//!   * **`.ild`** — a standard ILDA Image Data Transfer Format file, format 5
//!     (2D true colour), big-endian, for interop with other laser software.
//!
//! Both share the same status bits (`POINT_BLANK` 0x40 / `POINT_LAST` 0x80),
//! which are identical in the firmware and in the ILDA status byte.

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

/// Device-native blob: 8-byte `laser_point_t` records, little-endian.
/// Layout: `x i16 LE, y i16 LE, status u8, r u8, g u8, b u8`.
pub fn encode_blob(scene: &[IldaPoint]) -> Vec<u8> {
    let mut out = Vec::with_capacity(scene.len() * 8);
    let n = scene.len();
    for (i, p) in scene.iter().enumerate() {
        out.extend_from_slice(&p.x.to_le_bytes());
        out.extend_from_slice(&p.y.to_le_bytes());
        out.push(status(p, i + 1 == n));
        out.push(p.r);
        out.push(p.g);
        out.push(p.b);
    }
    out
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
    fn blob_record_layout() {
        let scene = vec![lit(0x0102, 0x0304, 0xAA, 0xBB, 0xCC)];
        let b = encode_blob(&scene);
        assert_eq!(b.len(), 8);
        assert_eq!(&b[0..2], &0x0102i16.to_le_bytes()); // x LE
        assert_eq!(&b[2..4], &0x0304i16.to_le_bytes()); // y LE
        assert_eq!(b[4], POINT_LAST); // sole point → last, not blank
        assert_eq!([b[5], b[6], b[7]], [0xAA, 0xBB, 0xCC]); // r,g,b
    }

    #[test]
    fn blob_blank_and_last_bits() {
        let scene = vec![
            IldaPoint { x: 1, y: 2, blank: true, r: 0, g: 0, b: 0 },
            lit(3, 4, 1, 2, 3),
        ];
        let b = encode_blob(&scene);
        assert_eq!(b[4], POINT_BLANK); // first: blanked, not last
        assert_eq!(b[12], POINT_LAST); // second record status (offset 8+4)
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
