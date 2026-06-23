# svg2scene

Convert SVG artwork into a single **standard ILDA frame** (`.ild`, format 5) for
the shadowgraph projector. Playout/streaming is a separate tool — see
[`tools/ildaplay`](../ildaplay) — which sends ILDA frames to the device at a
frame rate; this tool only does the SVG → ILDA conversion.

This is the host-side renderer for the ILDA point engine (`components/point_ring`
`laser_point_t`, `components/laser_engine`). It produces a **dense uniform point
stream** — there is no curve/CNC motion planning; the point stream *is* the
output, exactly what the firmware's fixed-rate engine consumes.

## Pipeline

| stage | module | in → out |
|-------|--------|----------|
| parse | `parse.rs` | SVG → coloured cubics (`usvg`, absolute geometry, per-path colour) |
| flatten + fit | `parse.rs` | cubics → polylines (`kurbo::flatten`), fitted to the normalised `[-1,1]` field, y-up |
| optimize | `optimize.rs` | `lasy` euler-circuit: draw-order, minimal blanking, blank-delay + sharp-corner dwell, interpolated to a dense point stream |
| emit | `emit.rs` | points → device blob (`laser_point_t`) **and** a standard `.ild` (format 5) |

`lasy` does the galvo-aware path optimisation (unicursal draw order — no retraced
lit lines — plus the blanking/corner delays from the Abderyim et al. paper). The
loop seam is bracketed with blanked points so the wrap-around draws no retrace.

## Running

All commands run from the repository root.

```sh
# build
cargo build --manifest-path tools/svg2scene/Cargo.toml

# emit the device blob + a standard .ild
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg -o /tmp/test.bin --ild /tmp/test.ild

# full debug bundle (symlinked at ./output) with per-stage visualisations
cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
    tools/svg2scene/examples/test.svg --debug-output-dir
```

The bundle holds `0-input.svg`, `1-parse.svg` (fitted polylines), `2-points.svg`
(emitted setpoints; dashed = blank travel), `scene.bin`, `scene.ild`. Render an
SVG to PNG on macOS with `qlmanage -t -s 1000 -o /tmp output/2-points.svg`.

## Getting it on the device

`svg2scene` only writes `.ild` files. To draw one (or animate several), hand the
frame(s) to [`ildaplay`](../ildaplay), which streams ILDA to the device and paces
animation frames; the projector loops whichever frame it last received:

```sh
# one frame, drawn and held
cargo run --manifest-path tools/svg2scene/Cargo.toml -- examples/test.svg -o test.ild
cargo run --manifest-path tools/ildaplay/Cargo.toml -- --host <device-ip> test.ild
```

The wire format is plain ILDA, so any conforming format-4/5 file works (even
`nc <device-ip> 7777 < test.ild`). The device's address: in STA mode (default) it
joins `ioio` and logs its DHCP IP at boot; in SoftAP mode it's `192.168.4.1`.

## Knobs

Host-side choices (the engine just plays points at its fixed rate — there are no
device "limits" to set here):

| flag | meaning |
|------|---------|
| `--amplitude` | DAC counts from field centre to edge (`pos = ±1`); how much of the field the art fills. Stay in the galvo linear region. |
| `--margin` | field border fraction left around the art (0..1) |
| `--intensity` | brightness scale on all colours (0..1) |
| `--flatten-tol` | Bézier flattening tolerance (normalised units) |
| `--points` | target points per frame (density). Implied refresh = point-rate / N |
| `--distance-per-point` / `--blank-points` / `--corner-radians` | `lasy` interpolation: spacing floor, blank settle points, corner dwell |

## Tests

```sh
cargo test --manifest-path tools/svg2scene/Cargo.toml
```

Covers the device-blob record layout and the `.ild` format-5 header/record bytes.

## Why these crates

- **`usvg`** — robust SVG parsing (units/styles/transforms → absolute geometry + colour).
- **`kurbo`** — Bézier flattening to polylines.
- **`lasy`** (nannou) — the galvo path optimiser: euler-circuit draw order, blanking, and corner/blank dwell, interpolated to a uniform point stream.
