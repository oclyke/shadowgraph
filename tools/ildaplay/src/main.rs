//! ildaplay — stream ILDA frames to the shadowgraph projector.
//!
//! Reads one or more ILDA files, concatenates their frames into a playlist, and
//! sends them to the device (port 7777) in order at a fixed frame rate. The
//! projector loops whichever frame it last received — its scene buffer persists —
//! so between our sends it keeps drawing the active frame, and each send swaps to
//! the next. A single frame is sent once and held (drawn until something replaces
//! it); a multi-frame playlist loops unless `--once`.
//!
//! Wire format is plain ILDA: per frame we send that frame's section bytes plus a
//! 0-record terminating header, and wait for the device's 1-byte ACK.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;

const DEFAULT_PORT: u16 = 7777;
const ACK: u8 = 0x06;

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

/// One ILDA data section (header + records) as raw bytes — a single frame.
struct Frame {
    bytes: Vec<u8>,
    format: u8,
    points: usize,
}

/// Record size for the true-colour ILDA formats we forward (others unsupported).
fn ild_recsize(format: u8) -> Option<usize> {
    match format {
        5 => Some(8),  // 2D true colour
        4 => Some(10), // 3D true colour
        _ => None,     // indexed/palette (0/1/2): convert to true colour first
    }
}

/// Split an ILDA file into its data-section frames (stops at the 0-record
/// terminator). Errors on a malformed file or an unsupported (indexed) format.
fn parse_frames(data: &[u8]) -> Result<Vec<Frame>, String> {
    let mut frames = Vec::new();
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
        let recsize = ild_recsize(format)
            .ok_or_else(|| format!("unsupported ILDA format {format} (only true-colour 4/5)"))?;
        let seclen = 32 + count * recsize;
        if pos + seclen > data.len() {
            return Err("truncated ILDA section".into());
        }
        frames.push(Frame {
            bytes: data[pos..pos + seclen].to_vec(),
            format,
            points: count,
        });
        pos += seclen;
    }
    Ok(frames)
}

/// A 32-byte ILDA terminating header (0 records).
fn terminator() -> [u8; 32] {
    let mut t = [0u8; 32];
    t[..4].copy_from_slice(b"ILDA");
    t[7] = 5; // format byte; ignored for a 0-record header
    t
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut frames: Vec<Frame> = Vec::new();
    for f in &args.files {
        let data = std::fs::read(f)?;
        let fs = parse_frames(&data).map_err(|e| format!("{}: {e}", f.display()))?;
        if fs.is_empty() {
            eprintln!("warning: {} has no frames", f.display());
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
            // One frame = its ILDA section + a terminator; device publishes the
            // section, acks at the terminator.
            conn.write_all(&fr.bytes)?;
            conn.write_all(&term)?;
            conn.flush()?;
            let mut ack = [0u8; 1];
            conn.read_exact(&mut ack)?;
            if ack[0] != ACK {
                return Err(format!("unexpected ack 0x{:02x}", ack[0]).into());
            }
            eprintln!("frame {i}: {} pts (ILDA fmt {})", fr.points, fr.format);

            // A lone frame is held by the projector — nothing to pace or repeat.
            if frames.len() == 1 {
                return Ok(());
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

    // Build an ILDA section: 32-byte header (format, count BE) + count*recsize 0s.
    fn section(format: u8, count: u16, recsize: usize) -> Vec<u8> {
        let mut v = vec![0u8; 32 + count as usize * recsize];
        v[..4].copy_from_slice(b"ILDA");
        v[7] = format;
        v[24..26].copy_from_slice(&count.to_be_bytes());
        v
    }

    #[test]
    fn parses_multiple_frames_until_terminator() {
        let mut data = section(5, 3, 8); // frame 0: 3 pts, fmt 5
        data.extend(section(4, 2, 10)); // frame 1: 2 pts, fmt 4
        data.extend(section(5, 0, 8)); // terminator
        data.extend(section(5, 9, 8)); // trailing junk after terminator (ignored)

        let frames = parse_frames(&data).unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!((frames[0].format, frames[0].points), (5, 3));
        assert_eq!((frames[1].format, frames[1].points), (4, 2));
        assert_eq!(frames[0].bytes.len(), 32 + 3 * 8);
    }

    #[test]
    fn rejects_unsupported_format() {
        let data = section(1, 2, 6); // format 1 = 2D indexed colour
        assert!(parse_frames(&data).is_err());
    }

    #[test]
    fn errors_on_truncated_section() {
        let mut data = section(5, 5, 8);
        data.truncate(32 + 2 * 8); // claims 5 records, only 2 present
        assert!(parse_frames(&data).is_err());
    }
}
