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
trace.py — Trace a flat / posterized raster image into a layered SVG
suitable for laser engraving, plotting, or vinyl cutting.

Each distinct color becomes its own <g> layer. Layers are stacked "knockout"
style (bottom = full silhouette, each color above sits on top) so adjacent
regions abut with no hairline gaps. Points are heavily simplified and smoothed
into Bezier curves to keep machine motion clean.

This is a uv single-file script: the dependencies above are declared inline
(PEP 723), so `uv run trace.py …` fetches them into an ephemeral environment —
no manual install, venv, or activation needed.

USAGE
-----
Basic (auto-detect the flat colors in the image):
    uv run trace.py dog.png dog.svg

Replace one color with another in the output (e.g. black -> purple):
    uv run trace.py dog.png dog.svg --remap 000000:800080

Pin an exact palette instead of auto-detecting:
    uv run trace.py photo.png out.svg --palette 67412C,A38165,000000

More / less simplification (higher = fewer points):
    uv run trace.py dog.png dog.svg --simplify 150

Quantize a full-color photo down to N flat colors first:
    uv run trace.py photo.png out.svg --colors 4

Other flags:
    --no-stack     tile colors edge-to-edge instead of knockout stacking
    --no-smooth    emit straight polygon segments instead of Bezier curves
    --min-area F   drop specks smaller than F (fraction of image area)
    --clean K      morphological cleanup strength in px (0 = off)
"""

import argparse
import os
import numpy as np
import cv2
from PIL import Image


# ----------------------------- palette -----------------------------------

def hex_to_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def rgb_to_hex(c):
    return "#%02X%02X%02X" % (int(c[0]), int(c[1]), int(c[2]))


def detect_palette(rgb, alpha, max_colors):
    """Return a list of RGB tuples. Uses exact unique colors when the image
    is already posterized; otherwise k-means quantizes to max_colors."""
    pix = rgb[alpha > 0].reshape(-1, 3)
    uniq = np.unique(pix, axis=0)
    if len(uniq) <= max_colors:
        # already flat — order by frequency
        cols, counts = np.unique(pix, axis=0, return_counts=True)
        order = np.argsort(-counts)
        return [tuple(int(v) for v in cols[i]) for i in order]
    # quantize
    data = np.float32(pix)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, labels, centers = cv2.kmeans(data, max_colors, None, crit, 3,
                                    cv2.KMEANS_PP_CENTERS)
    centers = centers.astype(int)
    counts = np.bincount(labels.flatten(), minlength=max_colors)
    order = np.argsort(-counts)
    return [tuple(int(v) for v in centers[i]) for i in order]


def label_pixels(rgb, alpha, palette):
    """Assign every opaque pixel to its nearest palette color -> int label map.
    Transparent pixels get label -1."""
    pal = np.array(palette, dtype=np.int32)
    flat = rgb.reshape(-1, 3).astype(np.int32)
    # nearest palette color by squared distance
    d = ((flat[:, None, :] - pal[None, :, :]) ** 2).sum(2)
    lab = d.argmin(1).reshape(rgb.shape[:2])
    lab[alpha == 0] = -1
    return lab


# ----------------------------- geometry ----------------------------------

def clean_mask(m, k_clean, min_area):
    m = m.astype(np.uint8)
    if k_clean > 0:
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_clean, k_clean))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, ker)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, ker)
    # drop tiny islands
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = np.zeros_like(m)
    for i in range(1, n):
        if stats[i, 4] >= min_area:
            keep[lab == i] = 1
    # fill tiny holes
    inv = 1 - keep
    n2, lab2, stats2, _ = cv2.connectedComponentsWithStats(inv, 8)
    for i in range(1, n2):
        if stats2[i, 4] < min_area:
            keep[lab2 == i] = 1
    return keep


def smooth_path(pts):
    """Closed cubic-Bezier path through pts (Catmull-Rom -> Bezier)."""
    p = pts.astype(float)
    n = len(p)
    if n < 3:
        d = "M %.1f,%.1f " % (p[0, 0], p[0, 1])
        for q in p[1:]:
            d += "L %.1f,%.1f " % (q[0], q[1])
        return d + "Z"
    d = "M %.1f,%.1f " % (p[0, 0], p[0, 1])
    for i in range(n):
        p0, p1, p2, p3 = p[(i - 1) % n], p[i], p[(i + 1) % n], p[(i + 2) % n]
        c1 = p1 + (p2 - p0) / 6.0
        c2 = p2 - (p3 - p1) / 6.0
        d += "C %.1f,%.1f %.1f,%.1f %.1f,%.1f " % (c1[0], c1[1], c2[0],
                                                   c2[1], p2[0], p2[1])
    return d + "Z"


def poly_path(pts):
    p = pts.astype(float)
    d = "M %.1f,%.1f " % (p[0, 0], p[0, 1])
    for q in p[1:]:
        d += "L %.1f,%.1f " % (q[0], q[1])
    return d + "Z"


def trace_layer(mask, eps, min_contour_area, smooth):
    cnts, _ = cv2.findContours(mask.astype(np.uint8),
                               cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    d, npts = "", 0
    for c in cnts:
        if cv2.contourArea(c) < min_contour_area:
            continue
        approx = cv2.approxPolyDP(c, eps, True).reshape(-1, 2)
        if len(approx) < 3:
            continue
        npts += len(approx)
        d += (smooth_path(approx) if smooth else poly_path(approx)) + " "
    return d.strip(), npts


# ------------------------------- main ------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Trace a flat-color image to layered SVG.")
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--colors", type=int, default=6,
                    help="max colors to auto-detect/quantize to (default 6)")
    ap.add_argument("--palette", default=None,
                    help="comma-separated hex colors to force, e.g. 67412C,A38165,000000")
    ap.add_argument("--remap", default="", nargs="*",
                    help="recolor output layers: SRChex:DSThex (e.g. 000000:800080)")
    ap.add_argument("--simplify", type=float, default=8.0,
                    help="point simplification in px; higher = fewer points (default 8)")
    ap.add_argument("--clean", type=float, default=4.0,
                    help="cleanup kernel in px, scaled to image (0 = off)")
    ap.add_argument("--min-area", type=float, default=4e-5,
                    help="drop specks smaller than this fraction of image area")
    ap.add_argument("--no-stack", action="store_true",
                    help="tile colors edge-to-edge instead of knockout stacking")
    ap.add_argument("--no-smooth", action="store_true",
                    help="straight segments instead of Bezier curves")
    args = ap.parse_args()

    im = Image.open(args.input).convert("RGBA")
    a = np.array(im)
    H, W = a.shape[:2]
    rgb, alpha = a[:, :, :3], a[:, :, 3]
    diag = (W * W + H * H) ** 0.5

    # palette
    if args.palette:
        palette = [hex_to_rgb(x) for x in args.palette.split(",")]
    else:
        palette = detect_palette(rgb, alpha, args.colors)
    print("palette:", [rgb_to_hex(c) for c in palette])

    # output color remap
    remap = {}
    for r in args.remap:
        s, d = r.split(":")
        remap[rgb_to_hex(hex_to_rgb(s))] = "#" + d.lstrip("#").upper()

    # per-pixel labels, then per-color masks ordered big -> small
    lab = label_pixels(rgb, alpha, palette)
    masks = []
    for idx in range(len(palette)):
        m = (lab == idx)
        if m.sum() == 0:
            continue
        masks.append((idx, m, int(m.sum())))
    masks.sort(key=lambda t: -t[2])  # largest first (bottom of stack)

    # scale params to image size
    px_eps = args.simplify
    k = int(round(args.clean / 2000.0 * diag)) if args.clean else 0
    if k % 2 == 0 and k > 0:
        k += 1
    min_area = max(8, int(args.min_area * W * H))
    min_cnt = max(6, min_area // 2)

    # build (stacked or tiled) masks and trace
    layers = []
    for i, (idx, m, _) in enumerate(masks):
        if args.no_stack:
            build = m
        else:  # knockout: this color + every smaller color above it
            build = np.zeros_like(m)
            for _, mm, _c in masks[i:]:
                build |= mm
        mc = clean_mask(build, k, min_area)
        d, n = trace_layer(mc, px_eps, min_cnt, not args.no_smooth)
        fill = rgb_to_hex(palette[idx])
        fill = remap.get(fill, fill)
        layers.append((fill, d, n))
        print("  layer %-9s %5d points" % (fill, n))

    total = sum(n for _, _, n in layers)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" '
           'viewBox="0 0 %d %d" width="%d" height="%d">' % (W, H, W, H)]
    for k_i, (fill, d, _) in enumerate(layers):
        svg.append('  <g id="layer%d" fill="%s" fill-rule="evenodd" stroke="none">'
                   % (k_i, fill))
        svg.append('    <path d="%s"/>' % d)
        svg.append('  </g>')
    svg.append('</svg>')

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        f.write("\n".join(svg))
    print("wrote %s  (%d layers, %d points total, %d KB)"
          % (args.output, len(layers), total, os.path.getsize(args.output) // 1024))


if __name__ == "__main__":
    main()
