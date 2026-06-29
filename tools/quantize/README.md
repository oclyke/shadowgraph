# quantize

Compress an image's colour space down to **N flat colours**. An N-colour palette
is chosen to represent the image, then every pixel is **binned** to its nearest
palette colour, producing a posterized, flat-colour image.

The whole thing runs in **[CIELAB](https://en.wikipedia.org/wiki/CIELAB_color_space)**,
a perceptually-uniform space, so the "distance" being minimised — both when
choosing colours and when binning pixels — is *perceived* colour difference. This
is the single biggest reason GIMP's quantizer looks good, and the closest single
lever for matching it. Two palette algorithms are offered:

- **median-cut** (default) — recursively splits the Lab colour box along its
  highest-variance axis at the median, always splitting the box with the most
  total error, until N boxes remain; each box's mean is a palette entry.
  Error-balanced subdivision in Lab spreads colours across the full tonal range
  and keeps them saturated. This is the perceptual,
  [Heckbert-style](https://en.wikipedia.org/wiki/Median_cut) box-cut at the heart
  of GIMP's *"generate optimum palette"* (GIMP layers extra heuristics on top —
  deliberate cut position, careful axis choice, occasional multiple even cuts —
  but the essence is this).
- **k-means** (`--method kmeans`) —
  [Lloyd's algorithm](https://en.wikipedia.org/wiki/K-means_clustering) in Lab.
  L2-optimal for the *prevalent* colours, but the centres are cluster means, so on
  an image dominated by one hue they can regress toward grey and look muddy.

This is the front of the laser pipeline: quantize a photo to a handful of flat
colours here, then hand the result to [`trace.py`](../trace), which turns each
flat colour into its own SVG layer.

```
photo ──quantize──▶ flat-colour PNG ──trace──▶ layered SVG ──svg2scene──▶ .ild ──ildaplay──▶ projector
```

### Two things that matter as much as the algorithm

**Colour management** and **the background** are each as consequential as the
choice of median-cut vs k-means — see the two sections below. Both default to
the GIMP-matching behaviour.

### Colour management (important for phone photos)

Images that carry an embedded **ICC profile** are converted to **sRGB before
quantizing**. This matters more than it sounds: modern iPhone captures are stored
in a wide-gamut / HDR encoding (e.g. *"Display P3 Primaries; PQ"*) whose tone
curve is nothing like sRGB. If you quantize those raw pixel values — what you get
if you ignore the profile — everything collapses into compressed, washed-out
midtones with no real blacks or highlights, looking nothing like the photo in any
colour-managed viewer (Preview, GIMP, a browser). Converting through the profile
first restores the true tones, so the palette comes out with proper darks, lights,
and saturation. Pass `--no-color-management` to quantize the raw values instead.

### The background (`--background`, default black)

GIMP quantizes a *flattened canvas*, so the area behind a cut-out subject counts
as a colour. That is why GIMP can dedicate a palette slot to near-black and keep a
dark subject's eyes, nose, and deep shadows crisp — instead of smearing them into
the nearest brown. This tool does the same: by default, transparent pixels are
treated as **black** (`--background black`) when the palette is chosen, so that
tone earns a slot. The saved image still keeps its real alpha — only palette
*selection* sees the background.

- `--background <name|#hex>` — treat transparent pixels as this colour (default
  `black`; e.g. `white`, `#808080`).
- `--background none` — choose the palette from the opaque subject only. Useful
  when the subject has no true darks and you don't want a slot "wasted" on the
  background; the trade-off is that genuinely dark features may merge into the
  darkest subject colour.

(On an image with no transparency this has no effect — there are no background
pixels to colour.)

## Running

`quantize.py` is a [uv](https://docs.astral.sh/uv/) single-file script — its
dependencies (`numpy`, `opencv-python-headless`, `Pillow`) are declared inline
with [PEP 723](https://peps.python.org/pep-0723/) metadata at the top of the
file, so there's nothing to install. `uv run` builds and caches an ephemeral
environment on first use (the same deps `trace.py` uses, so the env is shared):

```sh
# reduce the example photo to 3 flat colours (GIMP-like defaults)
uv run quantize.py examples/dobby.png out.png --colors 3

# the script is executable too (shebang runs it through uv)
./quantize.py examples/dobby.png out.png -n 6

# choose the palette from the subject only (ignore the transparent background)
uv run quantize.py examples/dobby.png out.png -n 3 --background none

# use k-means instead of median-cut, and dump the palette as a swatch strip
uv run quantize.py examples/dobby.png out.png -n 8 --method kmeans --palette-png palette.png
```

It reports the colour conversion (if any), then the chosen palette (with each
colour's prevalence) and where it wrote. With the defaults the example resolves to
a near-black / brown / tan palette — the same darks-and-lights structure GIMP
produces:

```
colour: 'Display P3 Primaries; PQ (Adaptive Gain Curve …)' -> sRGB
palette (3 colours, mediancut):
  #23150B    39.35%
  #765036    36.43%
  #AA886B    24.21%
wrote out.png  (3010x2975, 3 colours, 449 KB)
```

Then trace the flat result into a layered SVG for the laser:

```sh
uv run quantize.py examples/dobby.png flat.png -n 4
uv run ../trace/trace.py flat.png art.svg --simplify 150
```

## Flags

| flag | meaning |
|------|---------|
| `input` | source image (PNG/JPG/…); the alpha channel is preserved in the output |
| `output` | quantized image. PNG keeps alpha; JPG/BMP drop it (inferred from the extension) |
| `-n`, `--colors N` | number of palette colours (default 8) |
| `--method M` | palette algorithm: `mediancut` (default, GIMP-like, punchy) or `kmeans` (L2-optimal for prevalence, can look muddy). Both run in Lab |
| `--background C` | treat transparent pixels as colour `C` (name or `#hex`) when choosing the palette, like a flattened canvas (default `black`); `none` ignores them |
| `--sample N` | max pixels used to *fit* the palette; the full image is always binned. Keeps fitting fast on large photos (default 200k; `0` = use every pixel) |
| `--attempts N` | k-means restarts with different seeds; the lowest-error run is kept (default 4; k-means only) |
| `--seed N` | RNG seed for the fitting subsample, for reproducible palettes (default 0) |
| `--no-color-management` | skip the embedded-ICC → sRGB conversion and quantize the raw stored pixels (wide-gamut/HDR images will look muddy) |
| `--palette-png PATH` | also write the palette as a horizontal swatch strip, most-prevalent first |

## Notes

- **Perceptual (Lab) throughout.** Palette selection and pixel binning both run
  in CIELAB, so "nearest colour" means perceptually nearest, not RGB-nearest.
- **Colour-managed by default.** Embedded ICC profiles are converted to sRGB
  first (see above) — the single biggest factor in whether a phone photo
  quantizes to true tones or to mud. Use `--no-color-management` to opt out.
- **Background counts by default.** Transparent pixels participate in palette
  selection as `--background` (black), matching GIMP's flattened-canvas
  behaviour; the saved alpha is untouched. `--background none` opts out.
- **Already-flat images are lossless.** If the source has ≤ N distinct colours,
  those exact colours are used as the palette — no clustering, no drift.
- **Hard binning, no dithering.** Each pixel maps to its single nearest palette
  colour, so the output is flat regions (smooth gradients become bands). That's
  intentional: flat regions are exactly what `trace.py` needs to make clean
  layers. There's no error-diffusion/dither mode.
- **Sampling vs. binning.** Only the palette *fit* is subsampled (`--sample`);
  every pixel in the full image is binned against the final palette, so output
  resolution and detail are unaffected.

## Why these packages

- **`numpy`** — pixel/label array math, the median-cut box subdivision, and the
  chunked perceptual nearest-colour assignment.
- **`opencv-python-headless`** — sRGB ↔ CIELAB conversion (`cv2.cvtColor`) and the
  optional `--method kmeans` palette (`cv2.kmeans`). The *headless* build is used
  because the script never opens a GUI window — smaller, no display libs.
- **`Pillow`** — image load/save, format/alpha handling, and the ICC → sRGB
  colour conversion (`ImageCms`, via Little CMS).
