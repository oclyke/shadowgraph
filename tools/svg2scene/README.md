# svg2scene

Convert SVG artwork into a **cubic-Bézier (`CURVE`) galvo laser command stream**
for the shadowgraph projector.

This is the host-side *planner* half of the two-stage CNC architecture in
[`docs/CURVE_MOTION.md`](../../docs/CURVE_MOTION.md): the host fits the artwork to
cubics and does the global velocity planning; the firmware's real-time
interpolator rides each curve at the friction-circle limit. Instead of streaming
hundreds of `GOTO` points, a whole edge or arc is one ~21-byte `CURVE` carrying
its control points and entry/exit speeds.

> **The simulation is bit-exact with the device.** `build.rs` compiles the
> firmware's `components/curve_interp/curve_interp.c` straight into this tool, so
> the setpoints, speeds, and limit checks you see here are *exactly* what the
> ESP32 will produce — there is no second implementation to drift.

## Pipeline

Each stage is independent and gets its own debug visualization (below).

| stage | module | in → out |
|-------|--------|----------|
| parse + fit | `parse.rs` | SVG → coloured **cubics**, fitted to DAC counts (`usvg` + `kurbo`, y-up) |
| order + blank | `order.rs` | `lasy` euler-circuit traversal (each lit line drawn once) + greedy nearest-stroke reorder for travel; blank moves between strokes |
| segment | `segment.rs` | split cubics at inflections + curvature extrema → monotone-κ pieces |
| plan | `plan.rs` | junction velocities (corner-deviation ∧ curvature) + global fwd/bwd `v²` look-ahead → feasible `v_in/v_out` |
| emit | `emit.rs` | moves → `LASER`/`CURVE` wire bytes (matches `components/laser_command`) |
| simulate + analyse | `interp.rs` (FFI) + `analyze.rs` | replay the real interpolator → setpoints, kinematics, frame time |
| visualise | `viz.rs` | one dependency-free SVG per stage |

Velocity and point density are **not** knobs — they fall out of the physical
galvo limits, and frame-rate (persistence of vision) is *reported, not set*.

## Running

**All commands in this README are run from the repository root**
(`shadowgraph/`), not from `tools/svg2scene/`. Point Cargo at the manifest with
`--manifest-path`; everything after `--` goes to the tool, and file paths resolve
relative to the repo root (so `output/`, the example, and any `--output` land
under the repo root).

```sh
# build once
cargo build --manifest-path tools/svg2scene/Cargo.toml

# emit just the scene bytes
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --output /tmp/test.scene

# …or emit the full debug bundle (symlinked at ./output) and the scene at once
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --debug-output-dir
```

See every option: `cargo run --manifest-path tools/svg2scene/Cargo.toml -- --help`.
Add `--release` for a faster binary on large SVGs.

The tool runs entirely on the host and writes a `.scene` byte payload; sending it
to the device over UDP is a later phase (see `docs/CURVE_MOTION.md` §9).

## Debugging: the `--debug-output-dir` bundle

`--debug-output-dir` writes the **whole bundle** into an organised backing store
and drops a convenience **symlink** to it where you ask — defaulting to `./output`
(Bazel-style):

```sh
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --debug-output-dir
# debug bundle: output -> /var/folders/.../T/svg2scene/test
```

- `--debug-output-dir` → symlink at `./output`; `--debug-output-dir DIR` → at `DIR`.
- The real files live under `--debug-temporary-storage-dir` (defaults to
  `<tempdir>/svg2scene`, a per-input subdir refreshed each run — like Bazel's
  output base, so it never accumulates). You rarely set this.
- The symlink is replaced each run, but the tool **refuses to clobber** a real
  (non-symlink) file/dir already sitting at the link path.
- `--output` still writes the scene to its own path in addition to the bundle.

### What's in the bundle

Files are **numbered in pipeline order**, so a plain `ls` / file-browser sort
walks the stages top to bottom:

| file | what it shows |
|------|---------------|
| `0-input.svg` | the input SVG, copied verbatim (the source for the stages below) |
| `1-parse.svg` | the fitted cubics in their stroke colours |
| `2-order.svg` | strokes in draw order; blank travel moves dashed grey |
| `3-segment.svg` | the monotone-κ pieces; **white dots mark every split** |
| `4-plan.svg` | **velocity after the fwd/bwd passes** — each segment a `v_in→v_out` gradient, and a dot at every control point coloured by its junction speed |
| `5-points.svg` | the **actual emitted setpoints** from the FFI simulation, coloured by real speed — this is what the device draws |
| `6-profile.svg` | speed vs. arc length, against `v_max` |
| `6-profile.csv` | one row per setpoint (`t_s, x, y, blank, v_cps, v_frac`) |
| `7-scene.bin` | the raw `CURVE` wire bytes |

Speed is coloured with a **viridis ramp** (dark purple = slow/corner → yellow =
`v_max`); both `4-plan.svg` and `5-points.svg` carry a labelled colour **key** at
the bottom of the frame. `4-plan.svg` is the one to read for *planned* junction
velocities; `5-points.svg` shows the *realised* speed (a straight edge reads
dark→bright→dark there as the beam accelerates through it and brakes into the next
corner).

The SVGs open in any browser. On macOS, render one to PNG with:

```sh
qlmanage -t -s 1000 -o /tmp output/plan.svg   # writes /tmp/plan.svg.png
```

The console also prints a summary: curve count, wire bytes, simulated setpoint
count, frame time / refresh Hz, and peak v / a / j as a fraction of the limits
(a `< 30 Hz` note warns when the figure has more path than the galvos can draw
flicker-free at the current limits).

## Galvo limits live in the firmware — not on the command line

The galvo limits — max scan speed, max acceleration, and the interpolation tick —
are **properties of the device, not tool options**. There are no `--v-max-cps`,
`--a-max-cps2`, or `--dt-tick-us` flags. The tool reads all three from the
firmware header `components/curve_interp/curve_interp.h` (`CURVE_DEFAULT_*`) over
FFI, so the plan and the simulation always reflect *exactly* what the device will
draw — they can never silently disagree.

**To change a limit (e.g. to a faster scanner), edit `curve_interp.h` and rebuild
both the firmware and this tool.** For example:

```c
// components/curve_interp/curve_interp.h
#define CURVE_DEFAULT_V_MAX_CPS   ((int64_t)11468800)     // max scan speed, counts/s
#define CURVE_DEFAULT_A_MAX_CPS2  ((int64_t)57344000000)  // max accel,      counts/s²
#define CURVE_DEFAULT_DT_TICK_US  ((int32_t)20)           // interpolation tick (50 kHz)
```

The current values are placeholders tuned so the example renders ~30 Hz — replace
`V_MAX`/`A_MAX` with your scanner's real numbers. (See `docs/CURVE_MOTION.md` for
how a datasheet maps onto them.)

The only knobs the tool *does* expose are host-side choices that don't have to
match the device:

| flag | unit | meaning |
|------|------|---------|
| `--corner-dev` | counts | corner rounding (junction deviation): bigger = faster through corners, more rounding |
| `--amplitude` | counts | field centre→edge (pos = ±1); how much of the field the drawing fills |
| `--margin` | 0..1 | field border left around the drawing |
| `--intensity` | 0..1 | brightness scale on all colours |

## How it maps onto the wire

Every move is a `CURVE` (opcode `0x04`): `P1,P2,P3` as `u16` DAC counts, `v_in`
and `v_out` as `u32` counts/second. **P0 is implicit** — the engine's current
position — so the first move starts at field centre and C0 continuity is free.
Colour rides separate `LASER` (`0x02`) records, emitted only when it changes; a
blank travel move is simply colour 0. This is the exact codec in
`components/laser_command`. See `docs/CURVE_MOTION.md` for the full contract.

## Tests

```sh
cargo test --manifest-path tools/svg2scene/Cargo.toml
```

Covers the FFI keystone (the firmware interpolator links and runs), the `CURVE`
wire-byte layout, junction-velocity behaviour (sharp corner ≪ straight), and an
end-to-end run whose **simulated motion stays within the galvo limits**.

## Why these crates

- **`usvg`** — robust SVG parsing; resolves units/styles/transforms, returns
  absolute path geometry with per-path colour.
- **`kurbo`** — cubic Béziers with arc length, curvature, derivatives,
  inflections, and subdivision (the segmentation + planning math).
- **`lasy`** (nannou) — euler-circuit draw-order optimisation: it traverses
  connected/shared-vertex geometry drawing each lit line exactly once (no
  retracing) and inserts the minimal blanks. Fed the cubic *knot* points only
  (never flattening the curves); the result is then greedily reordered to cut
  travel between disjoint strokes.
- **`cc`** (build) — compiles the firmware's `curve_interp.c` for the bit-exact
  FFI simulation.
