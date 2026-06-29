# trace

Trace a flat / posterized raster image into a **layered SVG** for laser
engraving, plotting, or vinyl cutting.

Each distinct colour in the image becomes its own `<g>` layer. By default the
layers are stacked *knockout* style — the largest colour is a full silhouette at
the bottom and every smaller colour sits on top — so adjacent regions abut with
no hairline gaps. Contours are heavily simplified and (by default) smoothed into
cubic Béziers so the machine motion stays clean.

This feeds the front of the SVG pipeline: a photo or logo becomes a clean
multi-colour SVG here, which [`svg2scene`](../svg2scene) then turns into an ILDA
frame and [`ildaplay`](../ildaplay) streams to the projector.

## Running

`trace.py` is a [uv](https://docs.astral.sh/uv/) single-file script. Its
dependencies (`numpy`, `opencv-python-headless`, `Pillow`) are declared inline
with [PEP 723](https://peps.python.org/pep-0723/) metadata at the top of the
file, so there is nothing to install — `uv run` builds an ephemeral environment
on first use and caches it after:

```sh
# auto-detect the flat colours and trace
uv run trace.py examples/dobby.png dobby.svg

# the script is executable too (shebang runs it through uv)
./trace.py examples/dobby.png dobby.svg
```

The example image `examples/dobby.png` is a 3-colour posterized dog. A
**simplification level of 150** gives a clean, low-point trace well suited to the
laser (a handful of smooth blobs rather than fuzzy fur):

```sh
uv run trace.py examples/dobby.png dobby.svg --simplify 150
```

## Flags

| flag | meaning |
|------|---------|
| `input` | source raster (PNG/JPG/…); the alpha channel, if any, masks the art |
| `output` | SVG path to write (parent dirs are created) |
| `--colors N` | max colours to auto-detect, or k-means quantize to, if the image isn't already flat (default 6) |
| `--palette A,B,…` | pin an exact palette of hex colours instead of auto-detecting, e.g. `67412C,A38165,000000` |
| `--remap SRC:DST` | recolour an output layer, hex→hex, e.g. `000000:800080` (repeatable) |
| `--simplify PX` | contour simplification in px; **higher = fewer points** (default 8). 150 is a good level for `examples/dobby.png` |
| `--clean PX` | morphological open/close cleanup strength in px, scaled to the image (0 = off; default 4) |
| `--min-area F` | drop specks/holes smaller than this fraction of image area (default 4e-5) |
| `--no-stack` | tile colours edge-to-edge instead of knockout stacking |
| `--no-smooth` | emit straight polygon segments instead of Bézier curves |

It prints the detected palette and the point count per layer, then the total
points and file size — a quick gauge of whether `--simplify` is in the right
range for the machine.

## How it works

1. **Palette** — collect the opaque pixels. If the image already has ≤ `--colors`
   distinct colours it's treated as flat and ordered by frequency; otherwise
   k-means quantizes it down (`detect_palette`). `--palette` skips this.
2. **Label** — assign every pixel to its nearest palette colour, building one
   binary mask per colour (`label_pixels`).
3. **Stack** — order masks largest → smallest. In knockout mode each layer's mask
   is unioned with every smaller layer above it so colours overlap rather than
   leave seams.
4. **Clean** — `--clean` runs a morphological open+close, then tiny islands and
   holes below `--min-area` are dropped (`clean_mask`).
5. **Trace** — `cv2.findContours` + `approxPolyDP(eps=--simplify)` reduce each
   region to a sparse polygon, optionally smoothed through a Catmull-Rom → cubic
   Bézier pass (`trace_layer`, `smooth_path`).
6. **Emit** — one `<g fill=… fill-rule="evenodd">` per layer, bottom-up.

## Why these packages

- **`numpy`** — pixel/label array math.
- **`opencv-python-headless`** — k-means quantization, morphology, connected
  components, and contour tracing. The *headless* build is used because the
  script never opens a GUI window — it's smaller and pulls no display libs.
- **`Pillow`** — image loading to RGBA.
