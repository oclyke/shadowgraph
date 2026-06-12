//! CLI driver for the `svg2scene` pipeline.
//!
//! Runs SVG -> parse -> fit -> optimise -> emit, writing the raw `.scene` wire
//! bytes and optionally dumping a debug SVG at each stage. Everything runs on
//! the host; nothing is sent to a device.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, ValueEnum};
use lasy::InterpolationConfig;

use svg2scene::analyze;
use svg2scene::emit::{self, EmitOptions};
use svg2scene::optimize::{self, OptimizeOptions};
use svg2scene::parse::{self, ParseOptions};
use svg2scene::viz::{self, PointStyle};

/// CLI mirror of [`viz::PointStyle`] (keeps the library free of a clap dep).
#[derive(Clone, Copy, Debug, ValueEnum)]
enum CliPointStyle {
    None,
    Dot,
    Velocity,
    Dwell,
}

impl From<CliPointStyle> for PointStyle {
    fn from(s: CliPointStyle) -> Self {
        match s {
            CliPointStyle::None => PointStyle::None,
            CliPointStyle::Dot => PointStyle::Dot,
            CliPointStyle::Velocity => PointStyle::Velocity,
            CliPointStyle::Dwell => PointStyle::Dwell,
        }
    }
}

/// Convert an SVG into a galvo laser command stream (GOTO/LASER/DWELL).
#[derive(Parser, Debug)]
#[command(name = "svg2scene", version)]
struct Args {
    /// Input SVG file.
    input: PathBuf,

    /// Write the raw .scene wire bytes here (TV command payload).
    #[arg(short, long)]
    output: Option<PathBuf>,

    // --- debug output ------------------------------------------------------
    /// Emit the debug artifact bundle (flattened polylines, optimised point
    /// stream, per-point kinematics CSV, and the scene) and place a convenience
    /// symlink to it here, Bazel-style. `--debug-output-dir` alone uses
    /// `./output`; pass a path to choose another location.
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "output")]
    debug_output_dir: Option<PathBuf>,
    /// Base directory that holds the real debug files (a per-input subdir is
    /// created here and linked from --debug-output-dir). Rarely changed;
    /// defaults to <tempdir>/svg2scene.
    #[arg(long, value_name = "DIR")]
    debug_temporary_storage_dir: Option<PathBuf>,
    /// Control-point rendering style in the optimise dump.
    #[arg(long, value_enum, default_value_t = CliPointStyle::Velocity)]
    point_style: CliPointStyle,

    // --- parse / flatten ---------------------------------------------------
    /// Curve flattening tolerance in SVG user units (smaller = smoother).
    #[arg(long, default_value_t = 0.2)]
    tolerance: f64,
    /// Normalised margin left around the drawing (0..1).
    #[arg(long, default_value_t = 0.05)]
    margin: f64,

    // --- lasy optimisation -------------------------------------------------
    /// Minimum total output points (floor; optimiser may add more).
    #[arg(long, default_value_t = 1)]
    target_points: u32,
    /// Along-segment point density, in points per unit of normalised distance
    /// (field is 2 units wide). Higher = more samples per edge = lower, more
    /// uniform beam velocity on straight runs. (lasy treats this as a density:
    /// points ≈ 1 + dist * value, so its 0.1 default barely samples long edges.)
    #[arg(long, default_value_t = 50.0)]
    distance_per_point: f32,
    /// lasy: points inserted after each blank for light-modulator delay.
    #[arg(long, default_value_t = InterpolationConfig::DEFAULT_BLANK_DELAY_POINTS)]
    blank_delay_points: u32,
    /// lasy: corner-delay points added per radian of turn.
    #[arg(long, default_value_t = InterpolationConfig::DEFAULT_RADIANS_PER_POINT)]
    radians_per_point: f32,

    // --- emit --------------------------------------------------------------
    /// Galvo DAC counts from centre to field edge (pos = ±1).
    #[arg(long, default_value_t = 0x7000)]
    amplitude: u16,
    /// Dwell per point in microseconds (sets beam speed).
    #[arg(long, default_value_t = 50)]
    point_dwell_us: u32,
    /// Brightness scale applied to all colours (0..1).
    #[arg(long, default_value_t = 1.0)]
    intensity: f32,
}

/// Create (or refresh) the organised storage dir for `input` under `base` and
/// point a convenience symlink at `link` to it, Bazel-style. Returns the dir.
fn setup_debug_dir(
    link: &Path,
    input: &Path,
    base: &Path,
) -> Result<PathBuf, Box<dyn std::error::Error>> {
    let stem = input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scene");
    // Stable, organised dir keyed by input name (reused & refreshed per run,
    // like Bazel's output base — avoids unbounded accumulation).
    let dir = base.join(stem);
    if dir.exists() {
        std::fs::remove_dir_all(&dir)?;
    }
    std::fs::create_dir_all(&dir)?;
    place_symlink(&dir, link)?;
    Ok(dir)
}

/// Replace `link` with a symlink to `target`. Refuses to clobber a real
/// (non-symlink) file or directory already at `link`.
fn place_symlink(target: &Path, link: &Path) -> Result<(), Box<dyn std::error::Error>> {
    match std::fs::symlink_metadata(link) {
        Ok(meta) if meta.file_type().is_symlink() => std::fs::remove_file(link)?,
        Ok(_) => {
            return Err(format!(
                "{} already exists and is not a symlink; refusing to replace it",
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

    // Set up the optional debug bundle: real files land in an organised
    // storage dir, linked from --debug-output-dir. --output (the scene) still
    // takes precedence over the bundle copy when given.
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
    let in_bundle = |name: &str| debug_dir.as_ref().map(|d| d.join(name));
    let parse_dump = in_bundle("parse.svg");
    let optimize_dump = in_bundle("optimize.svg");
    let profile_dump = in_bundle("profile.csv");
    let scene_out = args.output.clone().or_else(|| in_bundle("scene.bin"));

    // 1. parse + flatten
    let parse_opts = ParseOptions {
        tolerance: args.tolerance,
        ..Default::default()
    };
    let raw = parse::parse_svg(&data, &parse_opts)?;
    if raw.is_empty() {
        return Err("no drawable stroked/filled paths found in SVG".into());
    }

    // 1b. fit into normalised laser space
    let fitted = parse::fit_to_unit(&raw, args.margin);

    if let Some(path) = &parse_dump {
        std::fs::write(path, viz::subpaths_svg(&fitted))?;
        eprintln!("wrote parse dump   -> {}", path.display());
    }

    // 2. optimise
    let opt_opts = OptimizeOptions {
        target_points: args.target_points,
        weight: 0,
        interp: InterpolationConfig {
            distance_per_point: args.distance_per_point,
            blank_delay_points: args.blank_delay_points,
            radians_per_point: args.radians_per_point,
        },
    };
    let points = optimize::optimize(&fitted, &opt_opts);
    if points.is_empty() {
        return Err("optimiser produced no points (degenerate geometry?)".into());
    }

    if let Some(path) = &optimize_dump {
        std::fs::write(path, viz::optimized_svg(&points, args.point_style.into()))?;
        eprintln!("wrote optimize dump-> {}", path.display());
    }

    // 2b. kinematics (first-class: velocity into/out of each point, dwell holds)
    let prof = analyze::profile(&points);
    let pstats = analyze::stats(&prof);
    if let Some(path) = &profile_dump {
        std::fs::write(path, analyze::to_csv(&prof, args.point_dwell_us))?;
        eprintln!("wrote profile csv  -> {}", path.display());
    }

    // 3. emit
    let emit_opts = EmitOptions {
        amplitude: args.amplitude,
        point_dwell_us: args.point_dwell_us,
        intensity: args.intensity,
    };
    let cmds = emit::to_commands(&points, &emit_opts);
    let bytes = emit::encode(&cmds);

    // stats
    let frame_us: u64 = args.point_dwell_us as u64 * points.len() as u64;
    eprintln!(
        "subpaths={}  points={} (blank={})  corner-holds={}  blank-holds={} (max {} samples)  \
         commands={}  bytes={}  frame≈{:.2} ms ({:.0} pps)",
        raw.len(),
        pstats.points,
        pstats.blanks,
        pstats.corner_holds,
        pstats.blank_holds,
        pstats.max_dwell_run,
        cmds.len(),
        bytes.len(),
        frame_us as f64 / 1000.0,
        1_000_000.0 / args.point_dwell_us as f64,
    );

    if let Some(path) = &scene_out {
        std::fs::write(path, &bytes)?;
        eprintln!("wrote scene        -> {}", path.display());
    } else if debug_dir.is_none() {
        eprintln!("(no --output or --debug-output-dir requested; nothing written)");
    }

    Ok(())
}

fn main() -> ExitCode {
    let args = Args::parse();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
