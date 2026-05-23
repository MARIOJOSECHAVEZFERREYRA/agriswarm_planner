"""Reproduce the Mission_cycles_bug mission under current code and
check every deadhead for obstacle intersections.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from shapely.geometry import Polygon as ShapelyPolygon, LineString

from backend.models.drone_model import Drone
from backend.services.mission_planner import DynamicMissionPlanner


BOUNDARY = [
    (50, 80), (420, 30), (480, 380),
    (380, 460), (200, 490), (80, 420), (20, 250),
]
OBSTACLES = [
    [(150, 200), (220, 190), (240, 240), (210, 280), (160, 270), (140, 230)],
    [(300, 150), (360, 140), (380, 180), (360, 220), (310, 230), (290, 190)],
    [(250, 320), (310, 310), (330, 350), (310, 390), (260, 400), (240, 360)],
]
UGV_POLYLINE = [
    (30.455, 124.261),
    (3.161, 262.439),
    (69.691, 417.676),
    (194.221, 503.824),
]
BASE_POINT = (30.455, 124.261)
OVERRIDES = {"swath": 9, "speed": 5, "app_rate": 10, "margin": 4.5}
DRONE_NAME = "DJI Agras T30"


def _line_crosses_obstacle(line, obstacle_poly):
    inter = line.intersection(obstacle_poly)
    if inter.is_empty:
        return False, 0.0
    # Length of intersection INSIDE the polygon interior (not boundary).
    interior_overlap = inter.difference(obstacle_poly.boundary)
    if interior_overlap.is_empty:
        return False, 0.0
    if hasattr(interior_overlap, 'length'):
        return True, float(interior_overlap.length)
    return True, 0.0


def main():
    print("Building planner and running mission...")
    planner = DynamicMissionPlanner(
        ugv_polyline=UGV_POLYLINE,
        ugv_speed=2.0,
        ugv_t_service=300.0,
    )
    engine = create_engine("sqlite:///agriswarm.db")
    db = sessionmaker(bind=engine)()
    try:
        result = planner.run_mission_planning(
            db=db,
            polygon_points=BOUNDARY,
            drone_name=DRONE_NAME,
            overrides=OVERRIDES,
            base_point=BASE_POINT,
            strategy_name="grid",
            obstacle_polygons=OBSTACLES,
        )
    finally:
        db.close()

    cycles = result.get("mission_cycles", [])
    angle = result.get("best_angle")
    rv_infeasible = result.get("rv_infeasible")
    print(f"winner angle={angle}  cycles={len(cycles)}  "
          f"rv_infeasible={rv_infeasible}")
    print()

    obstacle_polys = [ShapelyPolygon(o) for o in OBSTACLES]

    for cyc_i, cycle in enumerate(cycles):
        segs = cycle.get("segments", [])
        # Find closing deadhead run
        k = len(segs) - 1
        while k >= 0 and segs[k].get("segment_type") == "deadhead":
            k -= 1
        closing = segs[k + 1:] if k < len(segs) - 1 else []

        # Find opening deadhead run
        j = 0
        while j < len(segs) and segs[j].get("segment_type") == "deadhead":
            j += 1
        opening = segs[:j]

        def _analyse(label, run):
            if not run:
                return
            pts = [run[0]["p1"]] + [s["p2"] for s in run]
            total = sum(
                math.hypot(run[i]["p2"][0] - run[i]["p1"][0],
                           run[i]["p2"][1] - run[i]["p1"][1])
                for i in range(len(run))
            )
            print(f"  {label} ({len(run)} segs, {total:.1f}m):")
            for pt in pts:
                print(f"    {tuple(round(c, 2) for c in pt)}")
            for i in range(len(pts) - 1):
                line = LineString([pts[i], pts[i + 1]])
                for oi, op in enumerate(obstacle_polys):
                    crosses, length = _line_crosses_obstacle(line, op)
                    if crosses:
                        print(f"    !! seg[{i}] {pts[i]}->{pts[i+1]} "
                              f"crosses obstacle[{oi}] interior by {length:.2f}m")
            # Euclidean direct check
            direct = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
            if direct > 1e-3:
                overhead = (total - direct) / direct * 100.0
            else:
                overhead = 0.0
            print(f"    euclid direct = {direct:.1f}m  overhead = {overhead:.1f}%")

        print(f"cycle[{cyc_i}]:")
        _analyse("OPEN", opening)
        _analyse("CLOSE", closing)


if __name__ == "__main__":
    main()
