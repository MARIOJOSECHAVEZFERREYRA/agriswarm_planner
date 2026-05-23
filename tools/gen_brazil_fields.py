"""Generate synthetic Brazil FTW-style polygons with reasonable aspect ratios.

Produces polygons ~5–20 ha with aspect ratio 1.5–3:1 (instead of 8:1), placed
in the MATOPIBA agricultural region (western Bahia, UTM 23S). Each JSON has
boundary normalized to origin (0,0), a base_point just SW of the polygon, and
a UGV polyline along the long edge (parallel to the longer side so the UGV
"shadows" the UAV's natural end-of-sweep returns).

Usage:
    python tools/gen_brazil_fields.py --n 5 --out tests/test_fields/ftw
"""

import argparse
import json
import math
import os
import random

from pyproj import Transformer


EPSG_MATOPIBA = 32723  # UTM 23S — western Bahia / MATOPIBA agricultural belt


def _poly_rectangle(w, h, noise=0.05):
    """Rectangle w×h with small vertex noise for realism."""
    pts = [(0, 0), (w, 0), (w, h), (0, h)]
    jitter = min(w, h) * noise
    return [
        (x + random.uniform(-jitter, jitter), y + random.uniform(-jitter, jitter))
        for x, y in pts
    ]


def _poly_trapezoid(w, h, skew=0.15):
    """Trapezoid — w wide at base, narrower at top."""
    top_inset = w * skew
    return [
        (0, 0),
        (w, 0),
        (w - top_inset, h),
        (top_inset, h),
    ]


def _poly_irregular_quad(w, h):
    """Irregular quadrilateral with all four sides different."""
    j = min(w, h) * 0.08
    return [
        (random.uniform(0, j), random.uniform(0, j)),
        (w - random.uniform(0, j), random.uniform(0, j * 2)),
        (w - random.uniform(0, j * 2), h - random.uniform(0, j)),
        (random.uniform(0, j * 2), h - random.uniform(0, j * 2)),
    ]


def _rotate(pts, angle_deg, cx, cy):
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [
        ((x - cx) * ca - (y - cy) * sa + cx,
         (x - cx) * sa + (y - cy) * ca + cy)
        for x, y in pts
    ]


def _normalize_to_origin(pts):
    min_x = min(p[0] for p in pts)
    min_y = min(p[1] for p in pts)
    return [[round(x - min_x, 2), round(y - min_y, 2)] for x, y in pts]


def generate_brazil_field(seed):
    """Generate one synthetic Brazil field.

    Target: 5–20 ha, aspect ratio 1.5–3:1, plausible MATOPIBA location.
    """
    random.seed(seed)

    area_ha = random.uniform(5, 20)
    area_m2 = area_ha * 10_000
    aspect = random.uniform(1.5, 3.0)
    h = math.sqrt(area_m2 / aspect)
    w = aspect * h

    shape = random.choice(["rect", "trap", "irreg"])
    if shape == "rect":
        pts = _poly_rectangle(w, h)
    elif shape == "trap":
        pts = _poly_trapezoid(w, h, skew=random.uniform(0.08, 0.20))
    else:
        pts = _poly_irregular_quad(w, h)

    rotation = random.uniform(0, 90)
    pts = _rotate(pts, rotation, w / 2, h / 2)
    boundary = _normalize_to_origin(pts)

    xs = [p[0] for p in boundary]
    ys = [p[1] for p in boundary]
    max_x, max_y = max(xs), max(ys)

    base_point = [-3.0, -3.0]

    # UGV polyline along the west side of the polygon (perpendicular to the
    # likely sweep direction for horizontally-elongated fields).  This
    # "shadows" the UAV's end-of-sweep motion and keeps deadhead small.
    polyline_x = -5.0
    ugv_polyline = [
        [round(polyline_x, 2), -5.0],
        [round(polyline_x, 2), round(max_y + 5.0, 2)],
    ]

    # Random UTM easting/northing within MATOPIBA agricultural region.
    # MATOPIBA (12°S, 46°W) UTM 23S coords: easting ~395–400k, northing ~8.62M
    utm_x = random.uniform(395_000, 415_000)
    utm_y = random.uniform(8_610_000, 8_680_000)

    t = Transformer.from_crs(f"EPSG:{EPSG_MATOPIBA}", "EPSG:4326", always_xy=True)
    lng, lat = t.transform(utm_x, utm_y)

    return {
        "name": f"brazil_syn_{seed:03d}",
        "description": f"synthetic Brazil field (aspect {aspect:.1f}:1, {area_ha:.1f} ha)",
        "boundary": boundary,
        "obstacles": [],
        "base_point": base_point,
        "ugv_polyline": ugv_polyline,
        "ugv_speed": 2.0,
        "ugv_t_service": 300.0,
        "_utm_origin": [round(utm_x, 2), round(utm_y, 2)],
        "_origin_lnglat": [round(lng, 7), round(lat, 7)],
        "_utm_epsg": EPSG_MATOPIBA,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--out", default="tests/test_fields/ftw")
    ap.add_argument("--prefix", default="brazil_syn_")
    ap.add_argument("--seed", type=int, default=100)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    for i in range(args.n):
        seed = args.seed + i
        field = generate_brazil_field(seed)
        fname = f"{args.prefix}{seed:03d}.json"
        with open(os.path.join(args.out, fname), "w") as f:
            json.dump(field, f, indent=2)
        xs = [p[0] for p in field["boundary"]]
        ys = [p[1] for p in field["boundary"]]
        w = max(xs) - min(xs)
        h = max(ys) - min(ys)
        area = _shoelace_area(field["boundary"])
        print(f"  {fname:30} {w:6.1f} × {h:6.1f} m  area={area/10000:.2f} ha")


def _shoelace_area(pts):
    n = len(pts)
    s = 0.0
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


if __name__ == "__main__":
    main()
