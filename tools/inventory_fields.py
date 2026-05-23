"""Inventory all field fixtures: dimensions, obstacles, polyline stats.

Reports one row per field with the dimensions that matter for grid-search
diagnostics: field size, polyline length, polyline-to-field distance, hole
count, and a concavity ratio. Missing keys are tolerated so fields from
the non-dynamic subtrees are still listed (just without polyline stats).
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from shapely.geometry import Polygon, LineString, Point


def _ring(points):
    pts = [tuple(p) for p in points]
    if pts and pts[0] == pts[-1]:
        pts = pts[:-1]
    return pts


def _poly_from(field):
    # Two shapes appear across subtrees: {"boundary": [...]} or
    # {"coordinates": [...]} (the older AgriSwarm session format).
    raw = field.get("boundary") or field.get("coordinates")
    if raw is None:
        return None, []
    shell = _ring(raw)
    holes = [_ring(h) for h in field.get("obstacles", [])]
    try:
        p = Polygon(shell, holes)
        if not p.is_valid:
            p = p.buffer(0)
        return p, holes
    except Exception:
        return None, holes


def _polyline_field_distance(polyline, polygon):
    if polyline is None or len(polyline) < 2:
        return None, None
    ls = LineString(polyline)
    # Distance from the polyline to the polygon boundary (0 if polyline
    # is inside the polygon), and the distance from each polyline vertex
    # averaged — this approximates "how far does the UGV walk from the
    # field across the whole polyline".
    d_min = ls.distance(polygon)
    vs = [polygon.distance(Point(p)) for p in polyline]
    d_avg = sum(vs) / len(vs)
    return d_min, d_avg


def _polyline_length(polyline):
    if polyline is None or len(polyline) < 2:
        return 0.0
    total = 0.0
    for i in range(len(polyline) - 1):
        total += math.hypot(
            polyline[i + 1][0] - polyline[i][0],
            polyline[i + 1][1] - polyline[i][1],
        )
    return total


def _concavity(polygon):
    if polygon is None or polygon.is_empty:
        return None
    try:
        hull = polygon.convex_hull
        if hull.area < 1e-9:
            return None
        # 1.0 = convex, <1.0 = concave. Obstacles reduce this further.
        return polygon.area / hull.area
    except Exception:
        return None


def main():
    root = "tests/test_fields"
    files = []
    for d, _, fs in os.walk(root):
        for f in fs:
            if f.endswith(".json"):
                files.append(os.path.join(d, f))
    files.sort()

    header = (
        f"{'field':<55} {'area_m2':>10} {'nverts':>6} {'holes':>5} "
        f"{'conc':>5} {'poly_len':>8} {'d_min':>6} {'d_avg':>6}"
    )
    print(header)
    print("-" * len(header))

    for path in files:
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            print(f"{path}: read error {e}")
            continue

        poly, holes = _poly_from(data)
        short = os.path.relpath(path, root)

        if poly is None or poly.is_empty:
            print(f"{short:<55}  (no usable polygon)")
            continue

        shell = _ring(data.get("boundary") or data.get("coordinates") or [])
        nverts = len(shell)
        raw_ugv = data.get("ugv_polyline")
        polyline = [tuple(p) for p in raw_ugv] if raw_ugv else None
        plen = _polyline_length(polyline)
        d_min, d_avg = _polyline_field_distance(polyline, poly)
        conc = _concavity(poly)

        plen_s = f"{plen:.0f}" if polyline else "-"
        dmin_s = f"{d_min:.0f}" if d_min is not None else "-"
        davg_s = f"{d_avg:.0f}" if d_avg is not None else "-"
        conc_s = f"{conc:.2f}" if conc is not None else "-"

        print(
            f"{short:<55} {poly.area:>10.0f} {nverts:>6} {len(holes):>5} "
            f"{conc_s:>5} {plen_s:>8} {dmin_s:>6} {davg_s:>6}"
        )


if __name__ == "__main__":
    main()
