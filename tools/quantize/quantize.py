#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "numpy",
#     "opencv-python-headless",
#     "Pillow",
# ]
# ///
"""
quantize.py — Compress an image's colour space down to N flat colours.

An N-colour palette is chosen to represent the image, then every pixel is
*binned* to its nearest palette colour, producing a posterized, flat-colour
image. The whole process runs in CIELAB — a perceptually-uniform space — so the
"distance" being minimised is perceived colour difference, the way GIMP's
quantizer works (and the main reason its results look good). Two palette
algorithms are offered:

  median-cut (default) — recursively splits the Lab colour box along its
    highest-variance axis at the median, always splitting the box with the most
    total error, until N boxes remain; each box's mean colour is a palette entry.
    Working in Lab and balancing error this way spreads colours across the full
    tonal range and keeps them saturated. This is the perceptual median-cut at
    the heart of GIMP's "generate optimum palette".

  k-means (--method kmeans) — Lloyd's algorithm in Lab. L2-optimal for the
    prevalent colours, but the centres are cluster means, so on an image
    dominated by one hue they can regress toward grey and look muddy.

Two things matter as much as the algorithm:

  * Colour management. Images with an embedded ICC profile (e.g. iPhone photos
    in "Display P3 / PQ") are converted to sRGB first — otherwise the raw
    wide-gamut/HDR values quantize into muddy, washed-out midtones. Pass
    --no-color-management to skip it.

  * The background. Like GIMP quantizing a flattened canvas, transparent pixels
    are treated as a participating colour (--background, default black) when the
    palette is chosen, so the cut-out region's tone earns a palette slot — that
    is what lets dark subjects keep crisp blacks (eyes, shadows) instead of
    smearing them into the nearest brown. The saved image keeps its real alpha.
    Use --background none to choose the palette from opaque pixels only.

This is the front of the laser pipeline: quantize a photo to a handful of flat
colours here, then hand the result to `trace.py` (../trace), which turns each
flat colour into its own SVG layer.

USAGE
-----
Reduce a photo to 3 flat colours (median-cut, Lab):
    uv run quantize.py photo.png out.png --colors 3

Choose the palette from the subject only, ignoring the transparent background:
    uv run quantize.py photo.png out.png -n 3 --background none

Use k-means, and dump the chosen palette as a swatch strip:
    uv run quantize.py photo.png out.png -n 6 --method kmeans --palette-png pal.png

Then trace the flat result into a layered SVG:
    uv run ../trace/trace.py out.png out.svg --simplify 150
"""

import argparse
import io
import os
import numpy as np
import cv2
from PIL import Image, ImageCms


# ----------------------------- colour helpers ----------------------------

def rgb_to_hex(c):
    return "#%02X%02X%02X" % (int(c[0]), int(c[1]), int(c[2]))


def parse_color(s):
    """Parse a background colour: a name (black/white), or hex (`#rrggbb`)."""
    s = s.strip().lower()
    named = {"black": (0, 0, 0), "white": (255, 255, 255)}
    if s in named:
        return named[s]
    h = s.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def rgb_to_lab(rgb):
    """(M x 3) uint8 sRGB -> (M x 3) float32 CIELAB (L 0..100, a/b ~-128..127)."""
    f = np.ascontiguousarray(rgb.reshape(-1, 1, 3)).astype(np.float32) / 255.0
    return cv2.cvtColor(f, cv2.COLOR_RGB2Lab).reshape(-1, 3)


def lab_to_rgb(lab):
    """(K x 3) float32 CIELAB -> (K x 3) uint8 sRGB."""
    f = np.ascontiguousarray(lab.reshape(-1, 1, 3)).astype(np.float32)
    rgb = cv2.cvtColor(f, cv2.COLOR_Lab2RGB).reshape(-1, 3)
    return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)


# --------------------------- colour management ---------------------------

def to_srgb(im):
    """Return the image as RGBA in the sRGB colour space.

    If it carries an embedded ICC profile, convert through it. This matters a
    lot: modern phone captures (e.g. Apple's "Display P3 Primaries; PQ") store
    pixels in a wide-gamut / HDR encoding whose tone curve is very different from
    sRGB. Quantizing those raw values (what you get if you ignore the profile)
    bunches everything into compressed muddy midtones — no real blacks or
    highlights — which is *not* what the image looks like in any colour-managed
    viewer (or in GIMP). Converting to sRGB first restores the true tones.

    No profile -> assume the pixels are already sRGB and return them unchanged."""
    icc = im.info.get("icc_profile")
    rgba = im.convert("RGBA")
    if not icc:
        return rgba
    try:
        src = ImageCms.ImageCmsProfile(io.BytesIO(icc))
        dst = ImageCms.createProfile("sRGB")
        rgb = ImageCms.profileToProfile(
            Image.merge("RGB", rgba.split()[:3]), src, dst,
            renderingIntent=ImageCms.Intent.PERCEPTUAL, outputMode="RGB")
        r, g, b = rgb.split()
        print("colour: '%s' -> sRGB" % ImageCms.getProfileDescription(src).strip())
        return Image.merge("RGBA", (r, g, b, rgba.split()[3]))
    except Exception as e:  # unparseable/odd profile — don't crash, just warn
        print("warning: could not apply embedded colour profile (%s); "
              "quantizing raw values" % e)
        return rgba


# ------------------------------ palette ----------------------------------

def median_cut_lab(lab, n):
    """Perceptual median cut in CIELAB. Repeatedly split the box with the largest
    total squared error (population-weighted) along its highest-variance axis at
    the median, until N boxes remain. Returns the N box means as Lab colours.

    (GIMP's quantizer adds heuristics on top of this — deliberate cut position,
    careful axis choice, sometimes several even cuts per step — but the essence
    is exactly this: error-balanced, axis-aligned box subdivision in Lab.)"""
    def make(p):
        m = p.mean(0)
        return [p, m, float(((p - m) ** 2).sum())]

    boxes = [make(lab)]
    while len(boxes) < n:
        i = max(range(len(boxes)),
                key=lambda k: boxes[k][2] if len(boxes[k][0]) > 1 else -1.0)
        p = boxes[i][0]
        if len(p) <= 1 or boxes[i][2] <= 0.0:
            break  # only single-colour boxes left — nothing worth splitting
        ax = int(p.var(0).argmax())
        p = p[p[:, ax].argsort()]
        mid = len(p) // 2
        boxes[i] = make(p[:mid])
        boxes.append(make(p[mid:]))
    return np.array([b[1] for b in boxes], dtype=np.float32)


def fit_palette(pix, n, method, sample, attempts, seed):
    """Choose N palette colours (uint8 sRGB) for the pixels `pix` (M x 3 uint8).

    If there are <= N distinct colours they are returned as-is (lossless).
    Otherwise a random sample (for speed) is quantized in CIELAB by the chosen
    method ("mediancut" or "kmeans") and the representatives are returned."""
    uniq = np.unique(pix, axis=0)
    if len(uniq) <= n:
        return uniq.astype(np.uint8)

    data = pix
    if sample and len(data) > sample:
        rng = np.random.default_rng(seed)
        data = data[rng.choice(len(data), size=sample, replace=False)]
    lab = rgb_to_lab(data)

    if method == "kmeans":
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.2)
        _, _, reps = cv2.kmeans(lab.astype(np.float32), n, None, crit, attempts,
                                cv2.KMEANS_PP_CENTERS)
    else:
        reps = median_cut_lab(lab, n)
    return lab_to_rgb(reps)


def assign(pix, palette, chunk=1 << 20):
    """Bin each row of `pix` to the nearest `palette` row in the same space (here
    CIELAB, so nearest = perceptually nearest). Returns an int label array.
    Chunked, and using ||x-c||^2 = ||x||^2 - 2 x·c + ||c||^2 so no (chunk x K x 3)
    temporary is ever built."""
    pal = palette.astype(np.float32)
    pal_sq = (pal ** 2).sum(1)
    x = pix.astype(np.float32)
    out = np.empty(len(x), dtype=np.int32)
    for s in range(0, len(x), chunk):
        b = x[s:s + chunk]
        d = (b * b).sum(1)[:, None] - 2.0 * (b @ pal.T) + pal_sq[None, :]
        out[s:s + chunk] = d.argmin(1)
    return out


def palette_swatches(palette, counts, height=64, swatch=64):
    """Render the palette as a horizontal strip of equal-width swatches, ordered
    most-prevalent first, for a quick eyeball of the chosen colours."""
    order = np.argsort(-counts)
    strip = np.zeros((height, swatch * len(palette), 3), dtype=np.uint8)
    for i, idx in enumerate(order):
        strip[:, i * swatch:(i + 1) * swatch] = palette[idx]
    return strip


# ------------------------------- main ------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Compress an image's colour space to N flat colours.")
    ap.add_argument("input")
    ap.add_argument("output", help="quantized image (PNG keeps alpha; JPG drops it)")
    ap.add_argument("-n", "--colors", type=int, default=8,
                    help="number of palette colours (default 8)")
    ap.add_argument("--method", choices=("mediancut", "kmeans"), default="mediancut",
                    help="palette algorithm: mediancut (default, GIMP-like, punchy) "
                         "or kmeans (L2-optimal for prevalence, can look muddy)")
    ap.add_argument("--background", default="black",
                    help="treat transparent pixels as this colour when choosing "
                         "the palette (name or #hex), like quantizing a flattened "
                         "canvas; 'none' = ignore transparent pixels (default black)")
    ap.add_argument("--sample", type=int, default=200_000,
                    help="max pixels used to fit the palette; 0 = use all (default 200k)")
    ap.add_argument("--attempts", type=int, default=4,
                    help="k-means restarts; best result is kept (default 4; k-means only)")
    ap.add_argument("--seed", type=int, default=0,
                    help="RNG seed for the fitting subsample (default 0)")
    ap.add_argument("--no-color-management", action="store_true",
                    help="skip the embedded-ICC -> sRGB conversion and quantize "
                         "the raw stored pixels (wide-gamut/HDR images look muddy)")
    ap.add_argument("--palette-png", default=None,
                    help="also write the chosen palette as a swatch strip")
    args = ap.parse_args()

    if args.colors < 1:
        ap.error("--colors must be >= 1")

    im = Image.open(args.input)
    im = im.convert("RGBA") if args.no_color_management else to_srgb(im)
    a = np.array(im)
    H, W = a.shape[:2]
    alpha = a[:, :, 3]

    # Pixels used to choose the palette. Transparent pixels either take the
    # --background colour (GIMP-style flattened-canvas quantization) or are
    # dropped entirely (--background none). Either way the saved alpha is intact.
    flat = a[:, :, :3].reshape(-1, 3).copy()
    opaque = alpha.reshape(-1) > 0
    if args.background.lower() == "none":
        fit_mask = opaque
    else:
        flat[~opaque] = np.array(parse_color(args.background), dtype=np.uint8)
        fit_mask = np.ones(len(flat), dtype=bool)

    fit_pix = flat[fit_mask]
    if len(fit_pix) == 0:
        ap.error("no pixels to quantize (image fully transparent and "
                 "--background none)")

    # fit the palette, then bin every pixel to its perceptually-nearest colour
    palette = fit_palette(fit_pix, args.colors, args.method, args.sample,
                          args.attempts, args.seed)
    labels = assign(rgb_to_lab(flat), rgb_to_lab(palette))

    counts = np.bincount(labels[fit_mask], minlength=len(palette))
    total = counts.sum()
    print("palette (%d colours, %s):" % (len(palette), args.method))
    for idx in np.argsort(-counts):
        print("  %-9s %6.2f%%" % (rgb_to_hex(palette[idx]),
                                  100.0 * counts[idx] / total))

    # rebuild the image from the palette, preserving the original alpha
    out = a.copy()
    out[:, :, :3] = palette[labels].reshape(H, W, 3)
    out[:, :, 3] = alpha
    result = Image.fromarray(out, "RGBA")

    ext = os.path.splitext(args.output)[1].lower()
    if ext in (".jpg", ".jpeg", ".bmp"):
        result = result.convert("RGB")  # formats without alpha

    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)
    result.save(args.output)
    print("wrote %s  (%dx%d, %d colours, %d KB)"
          % (args.output, W, H, len(palette),
             os.path.getsize(args.output) // 1024))

    if args.palette_png:
        strip = palette_swatches(palette, counts)
        Image.fromarray(strip, "RGB").save(args.palette_png)
        print("wrote %s  (palette swatches)" % args.palette_png)


if __name__ == "__main__":
    main()
