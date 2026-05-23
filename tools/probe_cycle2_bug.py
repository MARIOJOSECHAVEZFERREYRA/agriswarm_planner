"""Isolated test for the cycle-2 deadhead bug.

Reproduces the field + obstacles from Mission_cycles_bug.json and calls
assembler.find_connection(cut, rv) directly. Also probes GeodesicSolver
internals (_is_visible, _snap_outside_holes) to pinpoint where the
routing decision diverges from expectation.

The expected behaviour: the cut point (244.10, 325.05) has a clear
euclidean line to the RV at (6.77, 270.85) — no obstacle intersects.
Yet the mission output routes via (65.79, 366.21), adding ~52 m.
"""

import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from shapely.geometry import Polygon as ShapelyPolygon, Point, LineString

from backend.algorithms.coverage.path_assembler import PathAssembler
from backend.algorithms.coverage.margin import MarginReducer


BOUNDARY = [
    (50, 80), (420, 30), (480, 380),
    (380, 460), (200, 490), (80, 420), (20, 250),
]
OBSTACLES = [
    [(150, 200), (220, 190), (240, 240), (210, 280), (160, 270), (140, 230)],
    [(300, 150), (360, 140), (380, 180), (360, 220), (310, 230), (290, 190)],
    [(250, 320), (310, 310), (330, 350), (310, 390), (260, 400), (240, 360)],
]
MARGIN_H = 4.5
SWATH = 9.0

CUT = (244.10, 325.05)
RV = (6.77, 270.85)
WAYPOINT_IN_JSON = (65.79, 366.21)


def _build_safe_polygon():
    raw = ShapelyPolygon(shell=BOUNDARY, holes=OBSTACLES)
    if not raw.is_valid:
        raw = raw.buffer(0)
    return MarginReducer.shrink(raw, margin_h=MARGIN_H)


def _check_obstacle_intersections(safe_poly, a, b):
    line = LineString([a, b])
    print(f"  Line {a} -> {b}")
    print(f"  length = {line.length:.2f} m")
    for i, hole in enumerate(safe_poly.interiors):
        hp = ShapelyPolygon(hole)
        inter = line.intersection(hp)
        if inter.is_empty:
            status = "NO intersection"
        else:
            status = f"INTERSECTS ({inter.geom_type}, length={inter.length:.2f}m)"
        print(f"  hole[{i}] ({len(hole.coords) - 1} vertices): {status}")


def main():
    safe = _build_safe_polygon()
    print("Safe polygon:")
    print(f"  exterior vertices: {len(list(safe.exterior.coords)) - 1}")
    print(f"  interior holes: {len(list(safe.interiors))}")
    print(f"  area: {safe.area:.1f} m^2")
    print()

    print("CUT inside safe? ", safe.covers(Point(CUT)))
    print("RV  inside safe? ", safe.covers(Point(RV)))
    print()

    print("Direct line (CUT -> RV) obstacle check:")
    _check_obstacle_intersections(safe, CUT, RV)
    print()

    print("Direct line (CUT -> WAYPOINT_IN_JSON) obstacle check:")
    _check_obstacle_intersections(safe, CUT, WAYPOINT_IN_JSON)
    print()

    print("Direct line (WAYPOINT_IN_JSON -> RV) obstacle check:")
    _check_obstacle_intersections(safe, WAYPOINT_IN_JSON, RV)
    print()

    assembler = PathAssembler(safe)
    solver = assembler._solver

    print("Probing GeodesicSolver._is_visible:")
    print(f"  is_visible(CUT, RV) = {solver._is_visible(CUT, RV)}")
    print(f"  is_visible(CUT, WAYPOINT_IN_JSON) = "
          f"{solver._is_visible(CUT, WAYPOINT_IN_JSON)}")
    print(f"  is_visible(WAYPOINT_IN_JSON, RV) = "
          f"{solver._is_visible(WAYPOINT_IN_JSON, RV)}")
    print()

    print("Probing shortest_path:")
    path, dist = solver.shortest_path(CUT, RV)
    print(f"  path ({len(path)} points): {path}")
    print(f"  distance: {dist:.2f} m")
    euclid = math.hypot(CUT[0] - RV[0], CUT[1] - RV[1])
    print(f"  direct euclidean: {euclid:.2f} m")
    print(f"  detour: {dist - euclid:.2f} m "
          f"({(dist - euclid) / euclid * 100:.1f}%)")
    print()

    print("find_connection (same method the planner calls):")
    path2, dist2 = assembler.find_connection(CUT, RV)
    print(f"  path ({len(path2)} points): {path2}")
    print(f"  distance: {dist2:.2f} m")


if __name__ == "__main__":
    main()
