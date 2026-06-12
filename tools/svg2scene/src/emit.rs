//! Stage 3: optimised points -> laser command stream / wire bytes.
//!
//! Each output point becomes a `GOTO` (position) plus a `DWELL` (the per-point
//! time slice that sets beam velocity), with a `LASER` inserted whenever the
//! colour changes (blank points naturally emit `LASER 0,0,0`). The byte
//! encoding matches the firmware's TV codec (`components/laser_command`) and the
//! Python reference sender `tools/stream_scene.py`:
//!
//! ```text
//!   GOTO  : 0x01, x:u16, y:u16              (5 bytes)
//!   LASER : 0x02, r:u16, g:u16, b:u16       (7 bytes)
//!   DWELL : 0x03, dt:u32                     (5 bytes)
//! ```
//!
//! all little-endian. Positions map normalised `[-1, 1]` space onto the galvo
//! DAC range centred at `GALVO_CENTER`.

use crate::model::{OutPoint, Rgb};

/// Galvo DAC mid-scale (matches firmware / stream_scene.py).
pub const GALVO_CENTER: u16 = 0x8000;

/// Command type tags (matches `components/laser_command`).
pub const CMD_GOTO: u8 = 0x01;
pub const CMD_LASER: u8 = 0x02;
pub const CMD_DWELL: u8 = 0x03;

/// A single decoded laser command (pre-serialisation, easy to inspect/test).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cmd {
    Goto { x: u16, y: u16 },
    Laser { r: u16, g: u16, b: u16 },
    Dwell { dt: u32 },
}

impl Cmd {
    /// Serialise to the little-endian TV wire format.
    pub fn encode_into(&self, out: &mut Vec<u8>) {
        match *self {
            Cmd::Goto { x, y } => {
                out.push(CMD_GOTO);
                out.extend_from_slice(&x.to_le_bytes());
                out.extend_from_slice(&y.to_le_bytes());
            }
            Cmd::Laser { r, g, b } => {
                out.push(CMD_LASER);
                out.extend_from_slice(&r.to_le_bytes());
                out.extend_from_slice(&g.to_le_bytes());
                out.extend_from_slice(&b.to_le_bytes());
            }
            Cmd::Dwell { dt } => {
                out.push(CMD_DWELL);
                out.extend_from_slice(&dt.to_le_bytes());
            }
        }
    }
}

/// Options controlling the points -> commands mapping.
#[derive(Clone, Debug)]
pub struct EmitOptions {
    /// DAC counts from centre to the edge of the projected field (`pos = ±1`).
    /// Keep within the galvo's linear region. `0x7000` ≈ 87% of half-scale.
    pub amplitude: u16,
    /// Time spent at each point, in 1 MHz timer ticks (µs). Sets beam speed.
    pub point_dwell_us: u32,
    /// Overall brightness scale applied to every colour channel (`0..=1`).
    pub intensity: f32,
}

impl Default for EmitOptions {
    fn default() -> Self {
        EmitOptions {
            amplitude: 0x7000,
            point_dwell_us: 50,
            intensity: 1.0,
        }
    }
}

/// Map a normalised axis value (`-1..=1`) to a galvo DAC code.
pub fn axis_to_dac(v: f32, amplitude: u16) -> u16 {
    let off = (v.clamp(-1.0, 1.0) * amplitude as f32).round() as i32;
    (GALVO_CENTER as i32 + off).clamp(0, u16::MAX as i32) as u16
}

fn color_to_u16(rgb: Rgb, intensity: f32) -> (u16, u16, u16) {
    let c = |v: f32| ((v * intensity).clamp(0.0, 1.0) * u16::MAX as f32).round() as u16;
    (c(rgb[0]), c(rgb[1]), c(rgb[2]))
}

/// Convert optimised points into a laser command list.
pub fn to_commands(points: &[OutPoint], opts: &EmitOptions) -> Vec<Cmd> {
    let mut cmds = Vec::with_capacity(points.len() * 3);
    let mut last_rgb: Option<(u16, u16, u16)> = None;
    for p in points {
        let x = axis_to_dac(p.pos[0], opts.amplitude);
        let y = axis_to_dac(p.pos[1], opts.amplitude);
        cmds.push(Cmd::Goto { x, y });

        let rgb = color_to_u16(p.rgb, opts.intensity);
        if last_rgb != Some(rgb) {
            cmds.push(Cmd::Laser {
                r: rgb.0,
                g: rgb.1,
                b: rgb.2,
            });
            last_rgb = Some(rgb);
        }
        cmds.push(Cmd::Dwell {
            dt: opts.point_dwell_us,
        });
    }
    cmds
}

/// Serialise a command list to the raw TV wire byte stream (the payload bytes
/// the firmware's `scene_stream` carries, with no UDP framing).
pub fn encode(cmds: &[Cmd]) -> Vec<u8> {
    let mut out = Vec::new();
    for c in cmds {
        c.encode_into(&mut out);
    }
    out
}

/// Convenience: points -> raw wire bytes.
pub fn emit(points: &[OutPoint], opts: &EmitOptions) -> Vec<u8> {
    encode(&to_commands(points, opts))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_maps_to_galvo_center() {
        assert_eq!(axis_to_dac(0.0, 0x7000), GALVO_CENTER);
        assert_eq!(axis_to_dac(1.0, 0x7000), GALVO_CENTER + 0x7000);
        assert_eq!(axis_to_dac(-1.0, 0x7000), GALVO_CENTER - 0x7000);
    }

    #[test]
    fn clamps_out_of_range() {
        assert_eq!(axis_to_dac(2.0, 0x7000), GALVO_CENTER + 0x7000);
        assert_eq!(axis_to_dac(-2.0, 0x7000), GALVO_CENTER - 0x7000);
    }

    #[test]
    fn wire_encoding_matches_reference() {
        // GOTO 0x1234,0x5678  -> 01 34 12 78 56  (little-endian)
        let mut b = Vec::new();
        Cmd::Goto { x: 0x1234, y: 0x5678 }.encode_into(&mut b);
        assert_eq!(b, [0x01, 0x34, 0x12, 0x78, 0x56]);

        b.clear();
        Cmd::Laser { r: 1, g: 2, b: 3 }.encode_into(&mut b);
        assert_eq!(b, [0x02, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00]);

        b.clear();
        Cmd::Dwell { dt: 50 }.encode_into(&mut b);
        assert_eq!(b, [0x03, 50, 0, 0, 0]);
    }

    #[test]
    fn color_change_emits_one_laser_each_run() {
        let pts = vec![
            OutPoint { pos: [0.0, 0.0], rgb: [1.0, 0.0, 0.0] },
            OutPoint { pos: [0.1, 0.0], rgb: [1.0, 0.0, 0.0] }, // same colour
            OutPoint { pos: [0.2, 0.0], rgb: [0.0, 0.0, 0.0] }, // blank
        ];
        let cmds = to_commands(&pts, &EmitOptions::default());
        let lasers = cmds.iter().filter(|c| matches!(c, Cmd::Laser { .. })).count();
        assert_eq!(lasers, 2); // red once, then blank once
    }
}
