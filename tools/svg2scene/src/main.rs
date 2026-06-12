//! CLI driver for the `svg2scene` pipeline.
//!
//! Runs SVG -> parse -> fit -> optimise -> emit, writing the raw `.scene` wire
//! bytes and optionally dumping a debug SVG at each stage. Everything runs on
//! the host; nothing is sent to a device.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use lasy::InterpolationConfig;

use svg2scene::emit::{self, EmitOptions};
use svg2scene::optimize::{self, OptimizeOptions};
use svg2scene::parse::{self, ParseOptions};
use svg2scene::viz;

/// Convert an SVG into a galvo laser command stream (GOTO/LASER/DWELL).
#[derive(Parser, Debug)]
#[command(name = "svg2scene", version)]
struct Args {
    /// Input SVG file.
    input: PathBuf,

    /// Write the raw .scene wire bytes here (TV command payload).
    #[arg(short, long)]
    output: Option<PathBuf>,

    // --- debug dumps -------------------------------------------------------
    /// Dump the flattened polylines (after fit) as an SVG.
    #[arg(long, value_name = "FILE")]
    dump_parse: Option<PathBuf>,
    /// Dump the optimised point stream (lit + blank) as an SVG.
    #[arg(long, value_name = "FILE")]
    dump_optimize: Option<PathBuf>,

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
    /// lasy: minimum distance the interpolator travels before a new point.
    #[arg(long, default_value_t = InterpolationConfig::DEFAULT_DISTANCE_PER_POINT)]
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

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let data = std::fs::read(&args.input)?;

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

    if let Some(path) = &args.dump_parse {
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

    if let Some(path) = &args.dump_optimize {
        std::fs::write(path, viz::optimized_svg(&points))?;
        eprintln!("wrote optimize dump-> {}", path.display());
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
    let blanks = points.iter().filter(|p| p.rgb == [0.0; 3]).count();
    let frame_us: u64 = args.point_dwell_us as u64 * points.len() as u64;
    eprintln!(
        "subpaths={}  points={} (blank={})  commands={}  bytes={}  frame≈{:.2} ms ({:.0} pps)",
        raw.len(),
        points.len(),
        blanks,
        cmds.len(),
        bytes.len(),
        frame_us as f64 / 1000.0,
        1_000_000.0 / args.point_dwell_us as f64,
    );

    if let Some(path) = &args.output {
        std::fs::write(path, &bytes)?;
        eprintln!("wrote scene        -> {}", path.display());
    } else if args.dump_parse.is_none() && args.dump_optimize.is_none() {
        eprintln!("(no --output and no dumps requested; nothing written)");
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
