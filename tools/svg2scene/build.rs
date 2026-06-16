// Compile the firmware's curve interpolator straight into the host tool, so the
// simulation is the EXACT same code the device runs (no second implementation).
// See docs/CURVE_MOTION.md.
fn main() {
    let base = "../../components/curve_interp";
    let src = format!("{base}/curve_interp.c");
    let inc = format!("{base}/include");

    cc::Build::new()
        .file(&src)
        .include(&inc)
        .opt_level(2)
        .warnings(true)
        .compile("curve_interp");

    println!("cargo:rerun-if-changed={src}");
    println!("cargo:rerun-if-changed={inc}/curve_interp.h");
}
