//! CLI driver: SVG → parse → flatten/fit → optimize (lasy) → emit, writing the
//! device-native point blob and a standard `.ild`, plus an optional Bazel-style
//! debug bundle whose stages are each visualised, and an optional `--stream` push
//! to a running device.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::Parser;

use svg2scene::emit::{self, EmitOptions};
use svg2scene::optimize::{self, OptimizeOptions};
use svg2scene::parse::{self, ParseOptions};
use svg2scene::stream;
use svg2scene::viz;

/// Convert an SVG into an ILDA-style galvo laser point stream.
#[derive(Parser, Debug)]
#[command(name = "svg2scene", version)]
struct Args {
    /// Input SVG file.
    input: PathBuf,

    /// Write the device-native point blob (8-byte laser_point_t records) here.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Also write a standard ILDA `.ild` file (format 5) here.
    #[arg(long)]
    ild: Option<PathBuf>,
    /// Stream the scene to a running device over TCP (`ip` or `ip:port`).
    #[arg(long, value_name = "HOST")]
    stream: Option<String>,

    /// Emit the debug bundle (input/parse/points + scene.bin/.ild) and drop a
    /// convenience symlink to it here. Bare flag uses `./output`.
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "output")]
    debug_output_dir: Option<PathBuf>,
    /// Backing store for the real debug files (per-input subdir). Rarely set;
    /// defaults to <tempdir>/svg2scene.
    #[arg(long, value_name = "DIR")]
    debug_temporary_storage_dir: Option<PathBuf>,

    /// Field border fraction left around the drawing (0..1).
    #[arg(long, default_value_t = 0.05)]
    margin: f64,
    /// DAC counts from field centre to edge (pos = ±1). Stay in the galvo linear
    /// region (cf. main/main.c GALVO_AMPLITUDE).
    #[arg(long, default_value_t = 16000.0)]
    amplitude: f64,
    /// Brightness scale applied to all colours (0..1).
    #[arg(long, default_value_t = 1.0)]
    intensity: f32,
    /// Bézier flattening tolerance, in normalised units (fraction of half-field).
    #[arg(long, default_value_t = 0.002)]
    flatten_tol: f64,

    /// Target total points per frame (density). Implied refresh = point-rate / N.
    #[arg(long, default_value_t = 600)]
    points: u32,
    /// lasy distance-per-point floor (normalised units).
    #[arg(long, default_value_t = 0.1)]
    distance_per_point: f32,
    /// Points held at the end of each blank (light-modulator settle).
    #[arg(long, default_value_t = 10)]
    blank_points: u32,
    /// Radians of corner angle per extra dwell point (galvo inertia at sharp turns).
    #[arg(long, default_value_t = 0.6)]
    corner_radians: f32,

    /// Engine point rate (Hz) used only to report the implied refresh.
    #[arg(long, default_value_t = 30000.0)]
    point_rate_hz: f64,
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
    let write = |path: Option<PathBuf>, body: &[u8]| -> std::io::Result<()> {
        if let Some(p) = path {
            std::fs::write(&p, body)?;
            eprintln!("wrote {}", p.display());
        }
        Ok(())
    };

    if let Some(p) = bundle("0-input.svg") {
        write(Some(p), &data)?;
    }

    // 1. parse + flatten + fit
    let raw = parse::parse_svg(&data, &ParseOptions::default())?;
    if raw.is_empty() {
        return Err("no drawable paths in SVG".into());
    }
    let polys = parse::flatten_and_fit(&raw, args.flatten_tol, args.margin);
    if polys.is_empty() {
        return Err("nothing to draw after flatten/fit".into());
    }
    write(bundle("1-parse.svg"), viz::parse_svg(&polys).as_bytes())?;

    // 2. optimize (lasy euler-circuit + interpolation)
    let opt = OptimizeOptions {
        target_points: args.points,
        distance_per_point: args.distance_per_point,
        blank_points: args.blank_points,
        corner_radians: args.corner_radians,
    };
    let out = optimize::optimize(&polys, &opt);
    if out.is_empty() {
        return Err("optimizer produced no points".into());
    }
    write(bundle("2-points.svg"), viz::points_svg(&out).as_bytes())?;

    // 3. emit → scene (device blob + .ild)
    let scene = emit::build_scene(
        &out,
        &EmitOptions {
            amplitude: args.amplitude,
            intensity: args.intensity,
        },
    );
    let stem = args
        .input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scene");
    let blob = emit::encode_blob(&scene);
    let ild = emit::encode_ild(&scene, stem);

    write(args.output.clone(), &blob)?;
    write(args.ild.clone(), &ild)?;
    write(bundle("scene.bin"), &blob)?;
    write(bundle("scene.ild"), &ild)?;

    let blanks = scene.iter().filter(|p| p.blank).count();
    let refresh = if scene.is_empty() {
        0.0
    } else {
        args.point_rate_hz / scene.len() as f64
    };
    eprintln!(
        "polylines={} flat_pts={} -> scene points={} (blank={}) blob={}B",
        polys.len(),
        polys.iter().map(|p| p.pts.len()).sum::<usize>(),
        scene.len(),
        blanks,
        blob.len(),
    );
    eprintln!(
        "implied refresh ≈ {:.1} Hz at {:.0} pps",
        refresh, args.point_rate_hz
    );
    if refresh > 0.0 && refresh < 30.0 {
        eprintln!(
            "note: {refresh:.1} Hz is below ~30 Hz flicker — lower --points or simplify the art."
        );
    }
    if args.output.is_none() && args.ild.is_none() && debug_dir.is_none() && args.stream.is_none() {
        eprintln!("(no --output/--ild/--debug-output-dir/--stream; nothing written or sent)");
    }

    // 4. optional live push
    if let Some(host) = &args.stream {
        eprintln!("streaming {} points to {host} …", scene.len());
        stream::send_scene(host, &scene)?;
        eprintln!("device acked; scene is live");
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
