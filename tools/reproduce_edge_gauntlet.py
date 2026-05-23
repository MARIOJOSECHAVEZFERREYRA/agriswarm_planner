"""Reproduce edge_gauntlet with a right-edge UGV polyline and check
every deadhead for obstacle crossings.
"""

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
    (0.0, 0.0), (500.0, 0.0), (500.0, 423.1), (0.0, 423.1),
]
OBSTACLES = [
    [(7.7, 7.7), (115.4, 7.7), (115.4, 61.5), (7.7, 61.5)],
    [(384.6, 3.8), (492.3, 3.8), (492.3, 69.2), (384.6, 69.2)],
    [(3.8, 153.8), (76.9, 153.8), (76.9, 269.2), (3.8, 269.2)],
    [(423.1, 115.4), (496.2, 115.4), (496.2, 307.7), (423.1, 307.7)],
    [(7.7, 361.5), (138.5, 361.5), (138.5, 419.2), (7.7, 419.2)],
    [(346.2, 353.8), (492.3, 353.8), (492.3, 419.2), (346.2, 419.2)],
    [(192.3, 11.5), (307.7, 11.5), (307.7, 46.2), (192.3, 46.2)],
    [(153.8, 376.9), (269.2, 376.9), (269.2, 419.2), (153.8, 419.2)],
]
# UGV polyline: lower-right to upper-right, just outside field on the right
UGV_POLYLINE = [(505.0, 0.0), (505.0, 423.1)]
BASE_POINT = (505.0, 0.0)
OVERRIDES = {"swath": 9, "speed": 5, "app_rate": 10, "margin": 4.5}
DRONE_NAME = "DJI Agras T30"


def _crosses_obstacle_interior(line, obstacle_poly):
    inter = line.intersection(obstacle_poly)
    if inter.is_empty:
        return False, 0.0
    interior_overlap = inter.difference(obstacle_poly.boundary)
    if interior_overlap.is_empty:
        return False, 0.0
    if hasattr(interior_overlap, 'length'):
        return True, float(interior_overlap.length)
    return True, 0.0


def _run():
    planner = DynamicMissionPlanner(
        ugv_polyline=UGV_POLYLINE, ugv_speed=2.0, ugv_t_service=300.0,
    )
    engine = create_engine("sqlite:///agriswarm.db")
    db = sessionmaker(bind=engine)()
    try:
        return planner.run_mission_planning(
            db=db, polygon_points=BOUNDARY, drone_name=DRONE_NAME,
            overrides=OVERRIDES, base_point=BASE_POINT,
            strategy_name="grid", obstacle_polygons=OBSTACLES,
        )
    finally:
        db.close()


def main():
    print("Running mission on edge_gauntlet + right-edge UGV polyline...")
    result = _run()
    cycles = result.get("mission_cycles", [])
    print(f"winner angle={result.get('best_angle')}  cycles={len(cycles)}  "
          f"rv_infeasible={result.get('rv_infeasible')}")
    print()

    obstacles = [ShapelyPolygon(o) for o in OBSTACLES]
    total_crossings = 0

    for i, cycle in enumerate(cycles):
        segs = cycle.get("segments", [])
        rv_wait = cycle.get("rv_wait_s")

        # Find opening and closing deadhead runs
        j = 0
        while j < len(segs) and segs[j].get("segment_type") == "deadhead":
            j += 1
        k = len(segs) - 1
        while k >= 0 and segs[k].get("segment_type") == "deadhead":
            k -= 1

        opening = segs[:j]
        closing = segs[k + 1:] if k < len(segs) - 1 else []

        for label, run in (("OPEN", opening), ("CLOSE", closing)):
            if not run:
                continue
            pts = [run[0]["p1"]] + [s["p2"] for s in run]
            total = sum(math.hypot(s["p2"][0] - s["p1"][0],
                                    s["p2"][1] - s["p1"][1]) for s in run)
            direct = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
            overhead_pct = (total - direct) / direct * 100.0 if direct > 1e-3 else 0.0

            crossings = []
            for s_idx in range(len(pts) - 1):
                line = LineString([pts[s_idx], pts[s_idx + 1]])
                for oi, op in enumerate(obstacles):
                    crosses, olen = _crosses_obstacle_interior(line, op)
                    if crosses:
                        crossings.append((s_idx, oi, olen, pts[s_idx], pts[s_idx + 1]))

            if crossings or overhead_pct > 5.0:
                print(f"cycle[{i}] {label}: {len(run)} segs, total {total:.1f}m, "
                      f"direct {direct:.1f}m, overhead {overhead_pct:.1f}%")
                for p in pts:
                    print(f"    {tuple(round(c, 2) for c in p)}")
                for s_idx, oi, olen, a, b in crossings:
                    total_crossings += 1
                    print(f"    !! seg[{s_idx}] {a}->{b} crosses "
                          f"obstacle[{oi}] interior by {olen:.2f}m")

    print()
    print(f"Total obstacle-interior crossings across all cycles: "
          f"{total_crossings}")


if __name__ == "__main__":
    main()
