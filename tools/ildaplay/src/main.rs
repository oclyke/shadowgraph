//! ildaplay — stream ILDA frames to the shadowgraph projector.
//!
//! Reads one or more ILDA files, **normalises** them to true colour, and sends
//! the frames to the device (port 7777) in order at a fixed frame rate. The
//! projector loops whichever frame it last received — its scene buffer persists —
//! so between our sends it keeps drawing the active frame, and each send swaps to
//! the next. A single frame is sent once and held; a multi-frame playlist loops
//! unless `--once`.
//!
//! Normalisation: the device only draws true-colour (format 5) points, but most
//! ILDA art in the wild is indexed-colour (format 0/1) or 3D (format 0/4). On
//! load we decode formats 0/1/4/5, resolve indexed colours through the ILDA
//! default palette (or an embedded format-2 palette), drop any Z, and re-emit
//! each frame as a format-5 section. Parsing tolerates a missing terminating
//! header (stops at EOF), which many real files omit.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;

const DEFAULT_PORT: u16 = 7777;
const ACK: u8 = 0x06;
const STATUS_BLANK: u8 = 0x40;
const STATUS_LAST: u8 = 0x80;

#[derive(Parser, Debug)]
#[command(name = "ildaplay", version)]
struct Args {
    /// ILDA files to play; all frames are concatenated into the playlist in order.
    #[arg(required = true)]
    files: Vec<PathBuf>,
    /// Device address ("ip" or "ip:port").
    #[arg(long)]
    host: String,
    /// Animation frame rate (frames per second).
    #[arg(long, default_value_t = 12.0)]
    fps: f64,
    /// Play the playlist once instead of looping.
    #[arg(long)]
    once: bool,
}

/// One normalised frame: a self-contained ILDA format-5 section, ready to send.
struct Frame {
    bytes: Vec<u8>,
    src_format: u8,
    points: usize,
}

/// A decoded point (true colour, 2D), the common form we re-encode from.
#[derive(Clone, Copy)]
struct Pt {
    x: i16,
    y: i16,
    blank: bool,
    r: u8,
    g: u8,
    b: u8,
}

fn ild_recsize(format: u8) -> Option<usize> {
    match format {
        0 => Some(8),  // 3D indexed: X,Y,Z, status, colour index
        1 => Some(6),  // 2D indexed: X,Y, status, colour index
        2 => Some(3),  // colour palette: R,G,B
        4 => Some(10), // 3D true colour: X,Y,Z, status, B,G,R
        5 => Some(8),  // 2D true colour: X,Y, status, B,G,R
        _ => None,
    }
}

fn be16(hi: u8, lo: u8) -> i16 {
    (((hi as u16) << 8) | lo as u16) as i16
}

/// Decode one data-section record into a true-colour 2D point. `palette` resolves
/// indexed formats. Caller guarantees `rec.len() == ild_recsize(format)`.
fn decode_record(format: u8, rec: &[u8], palette: &[(u8, u8, u8)]) -> Pt {
    let look = |idx: u8| palette.get(idx as usize).copied().unwrap_or((255, 255, 255));
    match format {
        5 => Pt {
            x: be16(rec[0], rec[1]),
            y: be16(rec[2], rec[3]),
            blank: rec[4] & STATUS_BLANK != 0,
            b: rec[5],
            g: rec[6],
            r: rec[7],
        },
        4 => Pt {
            x: be16(rec[0], rec[1]),
            y: be16(rec[2], rec[3]), // rec[4..6] = Z, dropped
            blank: rec[6] & STATUS_BLANK != 0,
            b: rec[7],
            g: rec[8],
            r: rec[9],
        },
        0 => {
            let (r, g, b) = look(rec[7]); // rec[4..6] = Z, dropped
            Pt { x: be16(rec[0], rec[1]), y: be16(rec[2], rec[3]), blank: rec[6] & STATUS_BLANK != 0, r, g, b }
        }
        1 => {
            let (r, g, b) = look(rec[5]);
            Pt { x: be16(rec[0], rec[1]), y: be16(rec[2], rec[3]), blank: rec[4] & STATUS_BLANK != 0, r, g, b }
        }
        _ => unreachable!("decode_record on unsupported format {format}"),
    }
}

/// Encode decoded points as a standalone ILDA format-5 section (32-byte header +
/// records). The final record gets the last-point flag.
fn encode_frame5(pts: &[Pt]) -> Vec<u8> {
    let n = pts.len().min(u16::MAX as usize);
    let mut out = Vec::with_capacity(32 + n * 8);
    out.extend_from_slice(b"ILDA");
    out.extend_from_slice(&[0, 0, 0, 5]); // reserved, format 5
    out.extend_from_slice(&[b' '; 8]); // frame name
    out.extend_from_slice(&[b' '; 8]); // company name
    out.extend_from_slice(&(n as u16).to_be_bytes()); // record count
    out.extend_from_slice(&0u16.to_be_bytes()); // frame number
    out.extend_from_slice(&1u16.to_be_bytes()); // total frames
    out.push(0); // projector
    out.push(0); // reserved
    for (i, p) in pts.iter().take(n).enumerate() {
        out.extend_from_slice(&p.x.to_be_bytes());
        out.extend_from_slice(&p.y.to_be_bytes());
        let mut st = 0u8;
        if p.blank {
            st |= STATUS_BLANK;
        }
        if i + 1 == n {
            st |= STATUS_LAST;
        }
        out.push(st);
        out.push(p.b);
        out.push(p.g);
        out.push(p.r);
    }
    out
}

/// The ILDA standard default 64-colour palette (used for indexed frames when no
/// format-2 palette has been provided). Index ≥ 64 falls back to white.
const DEFAULT_PALETTE: [(u8, u8, u8); 64] = [
    (255, 0, 0), (255, 16, 0), (255, 32, 0), (255, 48, 0), (255, 64, 0), (255, 80, 0),
    (255, 96, 0), (255, 112, 0), (255, 128, 0), (255, 144, 0), (255, 160, 0), (255, 176, 0),
    (255, 192, 0), (255, 208, 0), (255, 224, 0), (255, 240, 0), (255, 255, 0), (224, 255, 0),
    (192, 255, 0), (160, 255, 0), (128, 255, 0), (96, 255, 0), (64, 255, 0), (32, 255, 0),
    (0, 255, 0), (0, 255, 36), (0, 255, 73), (0, 255, 109), (0, 255, 146), (0, 255, 182),
    (0, 255, 219), (0, 255, 255), (0, 227, 255), (0, 198, 255), (0, 170, 255), (0, 142, 255),
    (0, 113, 255), (0, 85, 255), (0, 56, 255), (0, 28, 255), (0, 0, 255), (32, 0, 255),
    (64, 0, 255), (96, 0, 255), (128, 0, 255), (160, 0, 255), (192, 0, 255), (224, 0, 255),
    (255, 0, 255), (255, 32, 255), (255, 64, 255), (255, 96, 255), (255, 128, 255), (255, 160, 255),
    (255, 192, 255), (255, 224, 255), (255, 255, 255), (255, 224, 224), (255, 192, 192),
    (255, 160, 160), (255, 128, 128), (255, 96, 96), (255, 64, 64), (255, 32, 32),
];

/// Normalise an ILDA file into true-colour format-5 frames. Walks sections in
/// order (a format-2 section updates the active palette), tolerating a missing
/// terminator (stop at EOF) but erroring on a section truncated mid-records.
fn parse_frames(data: &[u8]) -> Result<Vec<Frame>, String> {
    let mut frames = Vec::new();
    let mut palette: Vec<(u8, u8, u8)> = DEFAULT_PALETTE.to_vec();
    let mut pos = 0usize;
    while pos + 32 <= data.len() {
        if &data[pos..pos + 4] != b"ILDA" {
            return Err(format!("bad ILDA header at offset {pos}"));
        }
        let format = data[pos + 7];
        let count = ((data[pos + 24] as usize) << 8) | data[pos + 25] as usize;
        if count == 0 {
            break; // terminating header
        }
        let recsize = ild_recsize(format).ok_or_else(|| format!("unknown ILDA format {format}"))?;
        let body = pos + 32;
        let end = body + count * recsize;
        if end > data.len() {
            return Err(format!("truncated ILDA section at offset {pos}"));
        }

        if format == 2 {
            // Palette section: replace the active palette.
            palette = (0..count)
                .map(|i| {
                    let r = &data[body + i * 3..];
                    (r[0], r[1], r[2])
                })
                .collect();
        } else {
            let pts: Vec<Pt> = (0..count)
                .map(|i| decode_record(format, &data[body + i * recsize..body + (i + 1) * recsize], &palette))
                .collect();
            frames.push(Frame {
                bytes: encode_frame5(&pts),
                src_format: format,
                points: pts.len(),
            });
        }
        pos = end;
    }
    Ok(frames)
}

/// A 32-byte ILDA terminating header (0 records).
fn terminator() -> [u8; 32] {
    let mut t = [0u8; 32];
    t[..4].copy_from_slice(b"ILDA");
    t[7] = 5;
    t
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut frames: Vec<Frame> = Vec::new();
    for f in &args.files {
        let data = std::fs::read(f)?;
        let fs = parse_frames(&data).map_err(|e| format!("{}: {e}", f.display()))?;
        if fs.is_empty() {
            eprintln!("warning: {} has no drawable frames", f.display());
        }
        frames.extend(fs);
    }
    if frames.is_empty() {
        return Err("no frames to play".into());
    }

    let addr = if args.host.contains(':') {
        args.host.clone()
    } else {
        format!("{}:{}", args.host, DEFAULT_PORT)
    };
    eprintln!(
        "playlist: {} frame(s); {} @ {:.1} fps -> {addr}",
        frames.len(),
        if frames.len() == 1 {
            "static (held by projector)"
        } else if args.once {
            "once"
        } else {
            "looping"
        },
        args.fps,
    );

    let mut conn = TcpStream::connect(&addr)?;
    conn.set_nodelay(true).ok();
    let term = terminator();
    let period = Duration::from_secs_f64(1.0 / args.fps.max(0.001));

    loop {
        for (i, fr) in frames.iter().enumerate() {
            let start = Instant::now();
            conn.write_all(&fr.bytes)?;
            conn.write_all(&term)?;
            conn.flush()?;
            let mut ack = [0u8; 1];
            conn.read_exact(&mut ack)?;
            if ack[0] != ACK {
                return Err(format!("unexpected ack 0x{:02x}", ack[0]).into());
            }
            eprintln!("frame {i}: {} pts (ILDA fmt {} -> 5)", fr.points, fr.src_format);

            if frames.len() == 1 {
                return Ok(()); // single frame is held; nothing to pace or repeat
            }
            if let Some(rem) = period.checked_sub(start.elapsed()) {
                std::thread::sleep(rem);
            }
        }
        if args.once {
            break;
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run(&Args::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Build an ILDA section: 32-byte header (format, count BE) + body bytes.
    fn section(format: u8, count: u16, body: &[u8]) -> Vec<u8> {
        let mut v = vec![0u8; 32];
        v[..4].copy_from_slice(b"ILDA");
        v[7] = format;
        v[24..26].copy_from_slice(&count.to_be_bytes());
        v.extend_from_slice(body);
        v
    }

    // Read the count + first record's B,G,R out of an encoded format-5 frame.
    fn frame5_first_bgr(bytes: &[u8]) -> (u16, u8, u8, u8) {
        let count = u16::from_be_bytes([bytes[24], bytes[25]]);
        (count, bytes[32 + 5], bytes[32 + 6], bytes[32 + 7])
    }

    #[test]
    fn indexed_format1_uses_default_palette() {
        // One 2D-indexed point at colour index 24 (default palette = green).
        let body = [0x01, 0x02, 0x00, 0x03, 0x00, 24];
        let data = section(1, 1, &body);
        let frames = parse_frames(&data).unwrap();
        assert_eq!(frames.len(), 1);
        let (count, b, g, r) = frame5_first_bgr(&frames[0].bytes);
        assert_eq!(count, 1);
        assert_eq!((r, g, b), (0, 255, 0)); // index 24 = green
        // last point flag set, not blank
        assert_eq!(frames[0].bytes[32 + 4], STATUS_LAST);
    }

    #[test]
    fn format2_palette_overrides_then_indexed_resolves() {
        let mut data = section(2, 1, &[0x12, 0x34, 0x56]); // palette[0] = (0x12,0x34,0x56)
        data.extend(section(1, 1, &[0, 0, 0, 0, 0x40, 0])); // blanked point, index 0
        let frames = parse_frames(&data).unwrap();
        assert_eq!(frames.len(), 1); // palette section is not a frame
        let (_c, b, g, r) = frame5_first_bgr(&frames[0].bytes);
        assert_eq!((r, g, b), (0x12, 0x34, 0x56));
        assert_eq!(frames[0].bytes[32 + 4], STATUS_BLANK | STATUS_LAST); // blank + last
    }

    #[test]
    fn format0_drops_z() {
        // 3D true... no: format 0 is 3D indexed. X=1,Y=2,Z=0x7FFF,status=0,idx=16(yellow).
        let body = [0x00, 0x01, 0x00, 0x02, 0x7F, 0xFF, 0x00, 16];
        let frames = parse_frames(&section(0, 1, &body)).unwrap();
        let (_c, b, g, r) = frame5_first_bgr(&frames[0].bytes);
        assert_eq!((r, g, b), (255, 255, 0)); // index 16 = yellow; Z ignored
    }

    #[test]
    fn tolerates_missing_terminator() {
        let data = section(5, 1, &[0, 1, 0, 2, 0, 0xAA, 0xBB, 0xCC]); // no terminator
        let frames = parse_frames(&data).unwrap();
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn errors_on_truncated_section() {
        let mut data = section(5, 5, &[0u8; 2 * 8]); // claims 5 records, 2 present
        data.truncate(32 + 2 * 8);
        assert!(parse_frames(&data).is_err());
    }
}
