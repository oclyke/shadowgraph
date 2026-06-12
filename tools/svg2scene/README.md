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
| emit | `emit.rs` | points → `GOTO`/`LASER`/`DWELL` commands → wire bytes |

The wire bytes match the firmware TV codec (`components/laser_command`) and the
Python reference sender `tools/stream_scene.py`. Sending over UDP is **not** done
here yet — this tool runs entirely on the host and writes a `.scene` payload.

## Why these crates

- **`usvg`** — robust SVG parsing; resolves units/styles/transforms and returns
  path geometry in absolute coordinates, with per-path stroke/fill colour.
- **`kurbo`** — adaptive Bézier/arc flattening to polylines.
- **`lasy`** (nannou) — galvo path optimisation implementing Abderyim et al.:
  draw-order optimisation, minimal blanking between disconnected geometry,
  blank-delay points, and sharp-corner delay points. This is the
  curvature/velocity-aware step.

## Usage

```sh
cargo run -- examples/test.svg -o out.scene \
    --dump-parse parse.svg \
    --dump-optimize optimize.svg
```

Key options (`--help` for all):

- `--tolerance` — curve flattening deviation (smaller = smoother/denser).
- `--distance-per-point` / `--blank-delay-points` / `--radians-per-point` —
  `lasy` tuning. These map onto the galvo's slew/settle characteristics.
- `--amplitude` — DAC counts centre→edge of field (keep in the linear region).
- `--point-dwell-us` — time per point; sets beam velocity.
- `--intensity` — global brightness scale.

### Debug visualisation

`--dump-parse` and `--dump-optimize` write standalone SVGs (open in any browser).
The optimise dump draws lit segments in colour, blank travel segments as faint
dashed grey, and every output point as a dot — so blanking and corner-dwell
clustering are visible.

## Tests

```sh
cargo test
```

Covers wire-format byte encoding (vs. the reference), DAC mapping, colour-run
`LASER` de-duplication, and the `lasy` pipeline (lit-only single path, blank
insertion between two paths, empty input).

## TODO

- Port the UDP sender (framing + per-command seq + redundancy from
  `stream_scene.py`) so `svg2scene … | stream` is one Rust binary.
- Animation: re-run the pipeline per frame in the SVG domain.
