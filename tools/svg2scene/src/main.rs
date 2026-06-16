//! CLI driver: SVG -> parse -> fit -> order -> segment -> plan -> emit, writing
//! the CURVE wire bytes and (optionally) a Bazel-style debug bundle whose stages
//! are each visualised. Simulation is bit-exact via FFI to `curve_interp`.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::Parser;

use svg2scene::analyze;
use svg2scene::emit::{self, EmitOptions};
use svg2scene::interp::CurveLimits;
use svg2scene::order;
use svg2scene::parse::{self, ParseOptions};
use svg2scene::plan;
use svg2scene::segment;
use svg2scene::viz;

/// Convert an SVG into a cubic-Bézier (CURVE) galvo laser command stream.
#[derive(Parser, Debug)]
#[command(name = "svg2scene", version)]
struct Args {
    /// Input SVG file.
    input: PathBuf,

    /// Write the raw .scene wire bytes here.
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Emit the debug bundle (parse/order/segment/plan/points/profile + scene)
    /// and drop a convenience symlink to it here. Bare flag uses `./output`.
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "output")]
    debug_output_dir: Option<PathBuf>,
    /// Backing store for the real debug files (per-input subdir). Rarely set;
    /// defaults to <tempdir>/svg2scene.
    #[arg(long, value_name = "DIR")]
    debug_temporary_storage_dir: Option<PathBuf>,

    /// Field border fraction left around the drawing (0..1).
    #[arg(long, default_value_t = 0.05)]
    margin: f64,
    /// DAC counts from field centre to edge (pos = ±1). Must match the limits.
    #[arg(long, default_value_t = 28672.0)]
    amplitude: f64,

    // The galvo limits (v_max, a_max, interpolation tick) are NOT flags: they are
    // properties of the device, read from the firmware (curve_interp.h) over FFI
    // so the plan/simulation always match what the device draws. To change them,
    // edit curve_interp.h and rebuild firmware + tool.
    /// Corner rounding (junction deviation) in counts: bigger = faster through
    /// corners, more rounding. (Host planning choice, not a galvo limit.)
    #[arg(long, default_value_t = 200.0)]
    corner_dev: f64,

    /// Brightness scale applied to all colours (0..1).
    #[arg(long, default_value_t = 1.0)]
    intensity: f32,
}

fn setup_debug_dir(
    link: &Path,
    input: &Path,
    base: &Path,
) -> Result<PathBuf, Box<dyn std::error::Error>> {
    let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("scene");
    let dir = base.join(stem);
    if dir.exists() {
        std::fs::remove_dir_all(&dir)?;
    }
    std::fs::create_dir_all(&dir)?;
    place_symlink(&dir, link)?;
    Ok(dir)
}

fn place_symlink(target: &Path, link: &Path) -> Result<(), Box<dyn std::error::Error>> {
    match std::fs::symlink_metadata(link) {
        Ok(meta) if meta.file_type().is_symlink() => std::fs::remove_file(link)?,
        Ok(_) => {
            return Err(format!(
                "{} exists and is not a symlink; refusing to replace it",
                link.display()
            )
            .into())
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(e.into()),
    }
    if let Some(parent) = link.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    #[cfg(unix)]
    std::os::unix::fs::symlink(target, link)?;
    #[cfg(not(unix))]
    return Err("convenience symlinks are only supported on unix".into());
    Ok(())
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let data = std::fs::read(&args.input)?;

    let debug_dir = match &args.debug_output_dir {
        Some(link) => {
            let base = args
                .debug_temporary_storage_dir
                .clone()
                .unwrap_or_else(|| std::env::temp_dir().join("svg2scene"));
            let dir = setup_debug_dir(link, &args.input, &base)?;
            eprintln!("debug bundle: {} -> {}", link.display(), dir.display());
            Some(dir)
        }
        None => None,
    };
    let bundle = |name: &str| debug_dir.as_ref().map(|d| d.join(name));
    let write = |path: Option<PathBuf>, body: &str| -> std::io::Result<()> {
        if let Some(p) = path {
            std::fs::write(&p, body)?;
            eprintln!("wrote {}", p.display());
        }
        Ok(())
    };

    // Limits are the device's, read from the firmware — never set on the host.
    let lim = CurveLimits::device_default();

    // 0. copy the input verbatim into the bundle, so the source sits alongside
    // the per-stage views (and the numbering reads start-to-finish).
    if let Some(p) = bundle("0-input.svg") {
        std::fs::write(&p, &data)?;
        eprintln!("wrote {}", p.display());
    }

    // 1. parse + fit
    let raw = parse::parse_svg(&data, &ParseOptions::default())?;
    if raw.is_empty() {
        return Err("no drawable paths in SVG".into());
    }
    let fitted = parse::fit_to_counts(&raw, args.amplitude, args.margin);
    if fitted.is_empty() {
        return Err("nothing to draw after fit".into());
    }
    write(bundle("1-parse.svg"), &viz::parse_svg(&fitted))?;

    // 2. order (+ blanking)
    let ordered = order::order(fitted);
    write(bundle("2-order.svg"), &viz::order_svg(&order::build_moves(&ordered)))?;

    // 3. segment
    let segmented: Vec<_> = ordered.iter().map(|s| segment::segment_subpath(s, 0.1)).collect();
    write(bundle("3-segment.svg"), &viz::segment_svg(&segmented))?;

    // 4. build moves + plan (junction velocities + global look-ahead)
    let mut moves = order::build_moves(&segmented);
    if moves.is_empty() {
        return Err("no moves produced".into());
    }
    let vj = plan::junction_speeds(&moves, &lim, args.corner_dev);
    for (k, m) in moves.iter_mut().enumerate() {
        m.v_in = vj[k];
        m.v_out = vj[k + 1];
    }
    write(bundle("4-plan.svg"), &viz::plan_svg(&moves, &vj, &lim))?;

    // 5. simulate (FFI) + analyse
    let pts = analyze::simulate(&moves, &lim);
    let st = analyze::stats(&pts, &lim);
    write(bundle("5-points.svg"), &viz::points_svg(&pts, &lim))?;
    write(bundle("6-profile.svg"), &viz::profile_svg(&pts, &lim))?;
    write(bundle("6-profile.csv"), &viz::to_csv(&pts, &lim))?;

    // 6. emit
    let bytes = emit::encode_scene(&moves, &lim, &EmitOptions { intensity: args.intensity });

    let vmaxf = lim.v_max_cps as f64;
    let amaxf = lim.a_max_cps2 as f64;
    eprintln!(
        "subpaths={} curves={} bytes={}  sim points={} (blank={})",
        segmented.len(),
        emit::curve_count(&moves),
        bytes.len(),
        st.points,
        st.blanks,
    );
    eprintln!(
        "frame≈{:.2} ms ({:.1} Hz)  peak v={:.0} ({:.0}% v_max)  a={:.0} ({:.0}% a_max)  j={:.2e}",
        st.frame_s * 1000.0,
        st.refresh_hz(),
        st.max_v,
        100.0 * st.max_v / vmaxf,
        st.max_a,
        100.0 * st.max_a / amaxf,
        st.max_j,
    );
    if st.refresh_hz() > 0.0 && st.refresh_hz() < 30.0 {
        eprintln!(
            "note: {:.1} Hz is below ~30 Hz flicker — more path than the galvos can \
             draw flicker-free at these limits; simplify or raise the limits.",
            st.refresh_hz()
        );
    }

    let scene_out = args.output.clone().or_else(|| bundle("7-scene.bin"));
    if let Some(p) = &scene_out {
        std::fs::write(p, &bytes)?;
        eprintln!("wrote scene -> {}", p.display());
    } else {
        eprintln!("(no --output or --debug-output-dir; nothing written)");
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
