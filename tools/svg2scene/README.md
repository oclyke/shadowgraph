# svg2scene

Convert SVG artwork into an **ILDA-style galvo laser point stream** for the
shadowgraph projector, and (optionally) stream it straight to the device.

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

## Streaming to the device

The device runs a TCP scene receiver on port 7777. Its WiFi role is selected in
`main/main.c` (`WIFI_STA_MODE`):

- **Station (default):** the device joins the `ioio` network and gets a DHCP
  address — read it from the serial monitor (`STA got IP …`) and stream to it:

  ```sh
  cargo run --manifest-path tools/svg2scene/Cargo.toml -- \
      tools/svg2scene/examples/test.svg --stream <device-ip>
  ```

- **SoftAP** (`WIFI_STA_MODE 0`): the device hosts `shadowgraph` / `letslaser`;
  join it and stream to its fixed IP `192.168.4.1`.

`--stream` sends `["SCN1"][u32 count LE][count × 8-byte laser_point_t]` and waits
for the device's 1-byte ACK; the device then loops that scene until the next one
arrives. Run again with a different SVG to swap the image live.

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
