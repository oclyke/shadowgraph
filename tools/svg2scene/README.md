# svg2scene

Convert SVG artwork into a galvo-ready laser command stream
(`GOTO` / `LASER` / `DWELL`) for the shadowgraph projector.

The conversion is a pipeline of independent, individually testable stages. Each
stage consumes and produces plain data (`src/model.rs`) so it can be tested or
visualised in isolation:

| stage | module | in → out |
|-------|--------|----------|
| parse + flatten | `parse.rs` | SVG bytes → coloured polylines (`usvg` + `kurbo`) |
| fit | `parse::fit_to_unit` | polylines → normalised `[-1,1]`, y-up |
| optimise | `optimize.rs` | polylines → point stream (`lasy`: draw order, blanking, corner delays) |
| analyse | `analyze.rs` | points → per-point kinematics (velocity / turn / dwell) |
| emit | `emit.rs` | points → `GOTO`/`LASER`/`DWELL` commands → wire bytes |

The wire bytes match the firmware TV codec (`components/laser_command`) and the
Python reference sender `tools/stream_scene.py`. Sending over UDP is **not** done
here yet — this tool runs entirely on the host and writes a `.scene` payload.

## Running from the repo root

The tool lives in `tools/svg2scene/`. From the repository root, point Cargo at
its manifest with `--manifest-path` (everything after `--` is passed to the
tool). Input/output paths are resolved relative to your shell's current
directory (the repo root here):

```sh
# build once
cargo build --manifest-path tools/svg2scene/Cargo.toml

# emit just the scene
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --output /tmp/test.scene

# …or emit the full debug bundle (see below) and the scene in one go
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --debug-output-dir --point-style velocity
```

The debug SVGs open in any browser; on macOS render them to PNG with
`qlmanage -t -s 1000 -o /tmp output/optimize.svg`.

### `--debug-output-dir`: one-flag artifact bundle (Bazel-style)

`--debug-output-dir` writes the **whole bundle** (`parse.svg`, `optimize.svg`,
`profile.csv`, `scene.bin`) into an organised storage dir and drops a
convenience **symlink** to it at a location you choose — defaulting to
`./output`:

```sh
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --debug-output-dir
# debug bundle: output -> /var/folders/.../T/svg2scene/test
# ./output/{parse.svg, optimize.svg, profile.csv, scene.bin}
```

- `--debug-output-dir` → symlink at `./output`; `--debug-output-dir DIR` →
  symlink at `DIR`.
- The real files live under `--debug-temporary-storage-dir` (defaults to
  `<tempdir>/svg2scene`; you rarely need to set it), in a per-input subdir
  refreshed each run — like Bazel's output base, so it doesn't accumulate.
- The symlink is replaced on each run, but the tool **refuses to clobber** a
  real (non-symlink) file/dir already sitting at the link path.
- `--output` still writes the scene to its own path in addition to the bundle.
- `--point-style` selects how the optimise dump renders (see below).

`output` is git-ignored at the repo root.

For a faster binary on large SVGs, add `--release` (output lands in
`tools/svg2scene/target/release/svg2scene`, runnable directly afterwards).

See every option with:

```sh
cargo run --manifest-path tools/svg2scene/Cargo.toml -- --help
```

## Why these crates

- **`usvg`** — robust SVG parsing; resolves units/styles/transforms and returns
  path geometry in absolute coordinates, with per-path stroke/fill colour.
- **`kurbo`** — adaptive Bézier/arc flattening to polylines.
- **`lasy`** (nannou) — galvo path optimisation implementing Abderyim et al.:
  draw-order optimisation, minimal blanking between disconnected geometry,
  blank-delay points, and sharp-corner delay points.

## Velocity / curvature awareness — how it actually works

`lasy` keeps beam velocity manageable with **two** mechanisms, and the debug
tooling makes both visible:

1. **Sharp-corner dwell.** At a corner, `lasy` inserts repeated *coincident*
   points at the vertex (count ∝ turn angle / `--radians-per-point`). The beam
   holds there for those extra samples so the galvo can decelerate, stop, and
   re-accelerate. These show up as `dwell_run > 1` clusters in the analysis and
   as the yellow rings / red hot-spots in the dumps. A smooth curve gets no
   holds.
2. **Along-segment density** (`--distance-per-point`). Straight runs flatten to
   just their endpoints, so without added samples a long edge would be a single
   giant jump. ⚠️ Note `lasy` treats this parameter as a *density* — roughly
   `points ≈ 1 + distance * value` — so its library default of `0.1` barely
   samples long edges. **This tool defaults to `50`** (≈ a point every 0.02 of
   the 2-unit-wide field), which keeps lit velocity low and uniform. Raise it
   for smoother/slower motion, lower it for fewer points.

### Per-point kinematics (first-class)

`analyze::profile(&points)` returns a `PointKinematics` per output point:
`speed_in`, `speed_out` (normalised units per sample — proportional to physical
velocity since every point shares the same dwell; `units_per_second(dwell_us)`
converts), `turn` (radians; meaningful on smooth curves — at a held corner the
angle is encoded as the cluster size instead), and `dwell_run` / `dwell_ix`
(coincident-cluster size and index). The bundle's `profile.csv` has one row
per point.

### Debug visualisation styles (`--point-style`)

The bundle's `optimize.svg` renders the post-`lasy`, pre-emit point stream. The
control-point style is selectable (`--point-style`, default `velocity`):

- `none` — path only.
- `dot` — point colour; blank points ringed red.
- `velocity` — heat-map by local beam speed (contrast-stretched: red = slow /
  corner hold … blue = fast), coincident points fanned out, blanks dimmed.
- `dwell` — yellow ring whose radius grows with corner-hold sample count;
  the clearest view of where the beam stops.

## Tests

```sh
cargo test --manifest-path tools/svg2scene/Cargo.toml
```

Covers wire-format byte encoding (vs. the reference), DAC mapping, colour-run
`LASER` de-duplication, the `lasy` pipeline (lit-only single path, blank
insertion between two paths, empty input), and the kinematics analysis
(dwell-cluster detection, turn angles).

## Tuning knobs (`--help` for all)

- `--tolerance` — curve flattening deviation (smaller = smoother/denser curves).
- `--distance-per-point` — along-segment sample density (see above).
- `--blank-delay-points` / `--radians-per-point` — `lasy` blank-settle and
  corner-hold amounts; map onto the galvo's slew/settle characteristics.
- `--amplitude` — DAC counts centre→edge of field (keep in the linear region).
- `--point-dwell-us` — time per point; sets beam velocity.
- `--intensity` — global brightness scale.

## TODO

- Port the UDP sender (framing + per-command seq + redundancy from
  `stream_scene.py`) so `svg2scene … | stream` is one Rust binary.
- Animation: re-run the pipeline per frame in the SVG domain.
