//! scenekind — classify a file as an **SVG** or an **ILDA** scene, with a sanity
//! check on the contents (not just the extension). Prints `svg` or `ild` to
//! stdout and exits 0 when recognised and structurally sound; exits non-zero
//! otherwise. `play.sh` uses it to route inputs.
//!
//! The extension is a hint; the *content* is authoritative (a mismatch is noted
//! on stderr but the content wins). ILDA is validated by walking its sections —
//! we own that format, so we check it thoroughly. SVG is a lightweight sniff for
//! an `<svg>` element; `svg2scene` does the real parse and reports precise errors.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "scenekind", version)]
struct Args {
    /// File to classify.
    file: PathBuf,
    /// Print only the kind to stdout; suppress the note on stderr.
    #[arg(short, long)]
    quiet: bool,
}

/// Record size in bytes for each standard ILDA format (used to walk sections).
fn ild_recsize(format: u8) -> Option<usize> {
    match format {
        0 => Some(8),  // 3D indexed
        1 => Some(6),  // 2D indexed
        2 => Some(3),  // colour palette
        4 => Some(10), // 3D true colour
        5 => Some(8),  // 2D true colour
        _ => None,
    }
}

/// Walk ILDA sections to validate structure. Returns (frame_count, formats seen)
/// or an error describing the corruption.
fn check_ilda(data: &[u8]) -> Result<(usize, Vec<u8>), String> {
    let mut pos = 0usize;
    let mut frames = 0usize;
    let mut formats = Vec::new();
    loop {
        if pos + 32 > data.len() {
            return Err("ILDA stream ends without a 0-record terminator".into());
        }
        if &data[pos..pos + 4] != b"ILDA" {
            return Err(format!("bad ILDA header at offset {pos}"));
        }
        let format = data[pos + 7];
        let count = ((data[pos + 24] as usize) << 8) | data[pos + 25] as usize;
        if count == 0 {
            break; // terminating header
        }
        let recsize =
            ild_recsize(format).ok_or_else(|| format!("unknown ILDA format {format}"))?;
        let seclen = 32 + count * recsize;
        if pos + seclen > data.len() {
            return Err(format!("truncated ILDA section at offset {pos}"));
        }
        frames += 1;
        formats.push(format);
        pos += seclen;
    }
    Ok((frames, formats))
}

fn looks_like_svg(data: &[u8]) -> bool {
    let n = data.len().min(8192);
    String::from_utf8_lossy(&data[..n])
        .to_ascii_lowercase()
        .contains("<svg")
}

fn run(args: &Args) -> Result<&'static str, String> {
    let data =
        std::fs::read(&args.file).map_err(|e| format!("read {}: {e}", args.file.display()))?;

    let content_ild = data.starts_with(b"ILDA");
    let content_svg = !content_ild && looks_like_svg(&data);

    let kind = if content_ild {
        "ild"
    } else if content_svg {
        "svg"
    } else {
        return Err("unrecognised content (no ILDA magic and no <svg> element)".into());
    };

    // Cross-check the extension; content is authoritative, so a mismatch is just
    // a note.
    let ext = args
        .file
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let ext_kind = match ext.as_str() {
        "svg" => Some("svg"),
        "ild" | "ilda" => Some("ild"),
        _ => None,
    };
    if let Some(ek) = ext_kind {
        if ek != kind && !args.quiet {
            eprintln!("scenekind: note: .{ext} extension but contents look like {kind}");
        }
    }

    if kind == "ild" {
        let (frames, mut formats) = check_ilda(&data)?;
        if !args.quiet {
            formats.sort_unstable();
            formats.dedup();
            eprintln!("scenekind: ILDA ok — {frames} frame(s), format(s) {formats:?}");
        }
    } else if !args.quiet {
        eprintln!("scenekind: looks like SVG");
    }
    Ok(kind)
}

fn main() -> ExitCode {
    match run(&Args::parse()) {
        Ok(kind) => {
            println!("{kind}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("scenekind: error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section(format: u8, count: u16, recsize: usize) -> Vec<u8> {
        let mut v = vec![0u8; 32 + count as usize * recsize];
        v[..4].copy_from_slice(b"ILDA");
        v[7] = format;
        v[24..26].copy_from_slice(&count.to_be_bytes());
        v
    }

    #[test]
    fn valid_ilda_with_terminator() {
        let mut d = section(5, 4, 8);
        d.extend(section(5, 0, 8)); // terminator
        let (frames, fmts) = check_ilda(&d).unwrap();
        assert_eq!(frames, 1);
        assert_eq!(fmts, vec![5]);
    }

    #[test]
    fn ilda_missing_terminator_errors() {
        let d = section(5, 4, 8); // no terminator
        assert!(check_ilda(&d).is_err());
    }

    #[test]
    fn ilda_truncated_errors() {
        let mut d = section(5, 5, 8);
        d.truncate(32 + 2 * 8);
        assert!(check_ilda(&d).is_err());
    }

    #[test]
    fn svg_sniff() {
        assert!(looks_like_svg(b"<?xml version='1.0'?>\n<svg xmlns='...'>"));
        assert!(!looks_like_svg(b"ILDA\0\0\0\x05"));
        assert!(!looks_like_svg(b"just some text"));
    }
}
